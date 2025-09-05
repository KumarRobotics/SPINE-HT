#!/usr/bin/env python3


from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Tuple, TypeAlias
import json
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from spine_multi.collaborator import (
    FALCON_4_CAPABILITIES,
    HUSKY_CAPABILITIES,
    JACKAL_CAPABILITIES,
    Collaborator,
    RobotDescription,
    RobotConfig,
    BehaviorResult,
    get_capability_set
)
from spine_multi.planner_logging import get_logger
from spine_multi.spine.class_llm import ClassLLM
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi_ros.autonomy_manager import AutonomyManager
from teaming_msgs.srv import Mission


# robot, [function, [args]]
RobotTask: TypeAlias = Tuple[str, Tuple[str, List[str]]]


class SPINEMultiNode(Node):
    def __init__(self):
        super().__init__(
            "spine_multi_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        robots = []
        team_specification = (
            self.get_parameter("team_specification").get_parameter_value().string_value
        )
        init_graph = self.get_parameter("init_graph").get_parameter_value().string_value
        init_location = (
            self.get_parameter("init_location").get_parameter_value().string_value
        )
        self._planning_limit = (
            self.get_parameter("planning_limit_idx").get_parameter_value().integer_value
        )
        self._llm_type = self.get_parameter("llm").get_parameter_value().string_value

        param_dict = self.get_parameters_by_prefix("robots")
        self._robot_configs = self.parse_params(param_dict)

        for robot_config in self._robot_configs:
            robots.append(
                RobotDescription(
                    id=robot_config.name,
                    type=robot_config.type,
                    capabilities=get_capability_set(robot_name=robot_config.name, robot_type=robot_config.type),
                    location=np.array([]),
                )
            )

        self._graph = GraphHandler(graph_path=init_graph)

        self._tracks = {}

        self.get_logger().info(f"init graph is: {self._graph.to_json_str()}")
        self.get_logger().info(f"team specification: {team_specification}")

        self._logger = get_logger(
            name="spine_multi_node", output="both", filename=self._get_log_fname()
        )

        self._collaborator = Collaborator(
            team_specification=robots,
            semantic_graph=self._graph,
            team_spec_language=team_specification,
            init_location=init_location,
            logger=self._logger,
            llm_type=self._llm_type,
        )
        self._planning_limit = 5
        self._class_llm = ClassLLM()

        self._robot_autonomy_managers = self._init_autonomy_managers(
            self._robot_configs
        )

        self._max_workers = len(self._robot_autonomy_managers)
        self._thread_executor = ThreadPoolExecutor(max_workers=self._max_workers)
        self._robot_tasks = defaultdict(list)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )
        self._graph_publisher = self.create_publisher(String, "graph", qos_profile)

        timer_cbk_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            1.0, self._graph_pub_cbk, callback_group=timer_cbk_group
        )

        self.mission_srv = self.create_service(
            Mission, "/spine_multi/mission", self._mission_cbk
        )

        self._log_info(
            f"SPINE multi initialized with llm: {self._llm_type}\n"
            f"Robots: {robot_config}\n"
            f"Team spec: {team_specification}"
        )

    def _graph_pub_cbk(self):
        graph_as_str = str(self._graph.to_json_str())
        msg = String()
        msg.data = graph_as_str
        self._graph_publisher.publish(msg)

    def _get_log_fname(self):
        log_dir = Path.home() / "data/spine-multi-logs"
        log_dir.mkdir(exist_ok=True, parents=True)
        nfiles = len(list(log_dir.glob("*log")))
        return (
            log_dir / f"{nfiles:03d}-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.log"
        )

    def _log_info(self, msg: str) -> None:
        self.get_logger().info(f"[spine multi node] {msg}")
        self._logger.info(msg)

    def parse_params(self, param_dict: Dict[str, Parameter]) -> List[RobotConfig]:
        tmp_robot_config = defaultdict(dict)

        for param_name, param_obj in param_dict.items():
            self.get_logger().info(f"{param_name}: {param_obj.value}")

            robot_name, param_name = param_name.split(".")

            tmp_robot_config[robot_name][param_name] = param_obj.value

        robot_configs = []
        for robot, config in tmp_robot_config.items():
            robot_configs.append(RobotConfig(**config))

        return robot_configs

    def _init_autonomy_managers(
        self, configs: List[RobotConfig]
    ) -> Dict[str, AutonomyManager]:
        managers = {}

        for config in configs:
            namespace = config.namespace
            managers[config.name] = AutonomyManager(
                parent_node=self,
                robot_name=config.name,
                subscription_prefix=config.subscription_prefix,
                init_graph=self._graph,
                init_node=config.init_location,
                nav_target_frame=config.nav_target_frame,
                graph_viz_topic=config.graph_viz_topic,
                track_topic=config.track_topic,
                local_costmap_topic=config.local_costmap_topic,
                tracks=self._tracks,
                logger=self._logger,
                behavior_request_pub=config.behavior_request_pub,
                behavior_request_ack_sub=config.behavior_request_ack_sub,
                behavior_result_sub=config.behavior_result_sub,
                behavior_result_ack_pub=config.behavior_result_ack_pub,
            )

        return managers

    def _set_labels(self, label_set: str) -> List[BehaviorResult]:
        # construct tasks
        set_label_assignments = []
        for robot, manager in self._robot_autonomy_managers.items():
            set_label_assignments.append((robot, ("set_labels", [label_set])))

        return self._task_robots_async(
            assignments=set_label_assignments, planning_idx=0
        )

    def _set_labels_to_tasks(self, label_set: str) -> List[BehaviorResult]:
        # construct tasks
        set_label_assignments = []
        for robot, manager in self._robot_autonomy_managers.items():
            set_label_assignments.append((robot, ("set_labels", [label_set])))
            self._robot_tasks[robot].append(("set_labels", [label_set]))

        # return self._task_robots_async(
        #     assignments=set_label_assignments, planning_idx=0
        # )

    def _mission_cbk(
        self, request: Mission.Request, response: Mission.Response
    ) -> Mission.Response:
        self.get_logger().info(f"[spine multi node] got mission: {request.spec}")

        _, mission_labels = self._class_llm.request(request.spec)
        self._log_info(f"setting mission labels: {mission_labels}")
        self._set_labels_to_tasks(",".join(mission_labels["classes"]))

        self._collaborator.init_planner(mission_specifications=request.spec)
        allocation_result = self._collaborator.get_allocation()

        self.get_logger().info(
            f"{self._collaborator.get_result_str(allocation_result)}"
        )

        for planning_idx in range(self._planning_limit):

            self.get_logger().info(
                f"[spine node] Planning idx: {planning_idx} mission is done: {allocation_result.mission_is_done}"
            )

            if allocation_result.mission_is_done:
                break

            assignments = allocation_result.translated_assignments

            # add assignments to task queue
            for robot, task in assignments:
                self._robot_tasks[robot].append(task)

            assignment_str = ""

            assignment_str += ",".join(
                [f"{r} was assigned {t}" for (r, t) in allocation_result.assignments]
            )
            self.get_logger().info(
                f"[spine node] Got assignments:\n\t {assignment_str}"
            )

            while len(assignments):  # TODO should have a timeout
                # self._log_info(f"Assigning the following async: {assignments}")

                task_results = self._task_robots_async(assignments, planning_idx)

                self._log_info(
                    f"robot results are done with dictionary: {task_results}"
                )

                for robot, behavior_results in task_results.items():
                    status_msg = f"\t{robot}\n"
                    for behavior_result in behavior_results:
                        status_msg += f"\t\t{behavior_result.behavior} - {behavior_result.success} - {behavior_result.error}\n"
                    self._log_info(status_msg)

                # breaking conditions:
                # 1. all tasks have been assigned
                # 2. any of the robots have updates
                #   TODO check this. unclear how it will interact w/ 1.
                # otherwise, we reassign the robots and reiterate
                if self._collaborator.all_tasks_assigned():
                    break
                elif any(
                    [
                        autonomy_manager._prompt_former.do_have_updates()
                        for autonomy_manager in self._robot_autonomy_managers.values()
                    ]
                ):
                    self._log_info(f"Got updates during task execution. Breaking.")
                    break
                else:
                    assignments = self._collaborator.reassign_tasks()
                    assignment_str += ",".join(
                        [f"{r} was assigned {t}" for (r, t) in assignments]
                    )

            # at the end of each planning iteration, empty mapping queue, form updates
            # and send to LLM
            updates = ""
            for robot, autonomy_manager in self._robot_autonomy_managers.items():
                autonomy_manager._tracker_component.parse_track_updates()
                robot_feedback = autonomy_manager._prompt_former.form_updates()
                updates += f"{robot} updates: {robot_feedback}\n"

            updates += f"\nPrevious assignments: {assignment_str}"

            self._log_info(f"sending updates: {updates}")

            allocation_result = self._collaborator.get_allocation(updates=updates)
            self._log_info(f"{self._collaborator.get_result_str(allocation_result)}")

        response.resp = allocation_result.mission_answer
        return response

    def _task_robots_async(
        self, assignments: List[Tuple[str, Tuple[str, List[str]]]], planning_idx: int
    ) -> Dict[str, List[BehaviorResult]]:
        """TODO: this type is unwieldy. The idea is a list of
        robot: assigned behavior and args for that behavior.
        """
        future_to_robot = {
            self._thread_executor.submit(
                self._call_behavior_async, robot, self._robot_tasks[robot], planning_idx
            ): robot
            for robot, task in assignments
        }

        results = defaultdict(list)

        for future in as_completed(future_to_robot):
            robot = future_to_robot[future]

            try:
                robot_result = future.result()
                results[robot].extend(robot_result)

                self._log_info(f"[task robots async] result: {robot_result}")

                for robot_result in robot_result:
                    if robot_result.success:
                        self.get_logger().info(
                            f"Robot {robot} behavior completed successfully"
                        )
                    else:
                        self.get_logger().error(
                            f"Robot {robot} behavior failed: {robot_result.error  }"
                        )

            except Exception as ex:
                self.get_logger().error(f"Robot {robot} generated exception: {ex}")
                results[robot].append(
                    BehaviorResult(robot, "unknown. ", False, str(ex))
                )

            self.get_logger().info(
                f"[spine node] robot results are done for {assignments}"
            )

        return results

    def _call_behavior_async(
        self, robot: str, task_seq: List[Tuple[str, List[Any]]], planning_idx: int
    ) -> List[BehaviorResult]:
        self.get_logger().info(
            f"[spine node] planning iteration {planning_idx}: commanding robot: {robot}: {task_seq}"
        )
        behaviors = [behavior for behavior, args in self._robot_tasks[robot]]

        success = "call_behavior not evaluated"

        try:
            behavior_success = self._robot_autonomy_managers[
                robot
            ].call_behavior_sequence(task_seq)

            behavior_results = []
            for behavior, success in zip(behaviors, behavior_success):
                behavior_results.append(BehaviorResult(robot, behavior, success, None))

            self._robot_tasks[robot] = []
            return behavior_results

        except Exception as ex:
            self._log_info(f"got error with behavior: {robot} {task_seq} {success}")
            self.get_logger().info(f"[spine node] Error in {robot} {behaviors}: {ex}")

            self._robot_tasks[robot] = []
            return [
                BehaviorResult(robot, behavior, False, str(ex))
                for behavior in behaviors
            ]


def main():
    rclpy.init()
    node = SPINEMultiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
