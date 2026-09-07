#!/usr/bin/env python3


import json
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Tuple, TypeAlias

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from spine_multi.collaborator import (
    FALCON_4_CAPABILITIES,
    HUSKY_CAPABILITIES,
    JACKAL_CAPABILITIES,
    BehaviorResult,
    Collaborator,
    RobotConfig,
    RobotDescription,
    get_capability_set,
)
from spine_multi.spatial_tools import SE2Transforms
from spine_multi.planner_logging import get_logger
from spine_multi.spine.class_llm import ClassLLM
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi_ros.autonomy_manager import AutonomyManager
from std_msgs.msg import String
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
        
        self.declare_parameter("llm", "gpt4")
        self._llm_type = self.get_parameter("llm").get_parameter_value().string_value

        param_dict = self.get_parameters_by_prefix("robots")
        self._robot_configs = self.parse_params(param_dict)

        for robot_config in self._robot_configs:
            robots.append(
                RobotDescription(
                    id=robot_config.name,
                    type=robot_config.type,
                    capabilities=get_capability_set(
                        robot_name=robot_config.name, robot_type=robot_config.type
                    ),
                    location=np.array([]),
                )
            )

        self._graph = GraphHandler(graph_path=init_graph)

        self._graph_viz = GraphVisualizerComponent(
            parent_node=self,
            graph=self._graph,
            target_frame="map",
            topic_name="/basestation/graph_viz",
            scale=2.0,
        )
        self._graph_viz.set_graph(self._graph)

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

        self._no_uav_graph_yet = True
        uav_graph_cbk_group = ReentrantCallbackGroup()
        self._uav_graph_sub = self.create_subscription(
            String,
            "/titan/graph",
            callback=self._uav_graph_sub,
            callback_group=uav_graph_cbk_group,
            qos_profile=qos_profile,
        )

        # if user wants to send msg mid mission
        self._interrupt_msg = ""

        timer_cbk_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            1.0, self._graph_pub_cbk, callback_group=timer_cbk_group
        )

        self.mission_srv = self.create_service(
            Mission, "/spine_multi/mission", self._mission_cbk
        )

        interrupt_cbk_group = ReentrantCallbackGroup()

        self.interrupt_srv = self.create_service(
            Mission,
            "/spine_multi/interrupt",
            self._interrupt_cbk,
            callback_group=interrupt_cbk_group,
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
            if not manager._robot_name.startswith("titan"):
                set_label_assignments.append((robot, ("set_labels", [label_set])))

        return self._task_robots_async(
            assignments=set_label_assignments, planning_idx=0
        )

    def _set_labels_to_tasks(self, label_set: str) -> List[BehaviorResult]:
        # construct tasks
        set_label_assignments = []
        for robot, manager in self._robot_autonomy_managers.items():
            if not manager._robot_name.startswith("titan"):
                set_label_assignments.append((robot, ("set_labels", [label_set])))
                self._robot_tasks[robot].append(("set_labels", [label_set]))

        # return self._task_robots_async(
        #     assignments=set_label_assignments, planning_idx=0
        # )

    def _parse_node_data(self, data: dict, transform: SE2Transforms) -> dict:
        try:
            name = data["name"]
            coords = data["coords"]
            coords_arr = np.array([float(coords[0]), float(coords[1])])

            self._log_info(f"have raw pt: {coords_arr}")
            coords_transformed = transform.transform_pt(coords_arr)

            coords_transformed = np.round(coords_transformed, 1)

            self._log_info(f"Point {coords_arr} -> {coords_transformed}")


            # normalize

            return name, {"coords": coords_transformed, "description": "from_uav"}

        except Exception as ex:
            self._log_info(f"[parse node data] [ERROR] when parsing {data}: {ex}")
            return {}

    def _parse_uav_graph(self, graph_as_str, uav_name="titan_1"):
        uav_node_suffix = "_from_uav"
        incoming_graph = json.loads(graph_as_str)

        assert uav_name in self._robot_autonomy_managers
        prompt_former = self._robot_autonomy_managers[uav_name]._prompt_former

        origin = np.array([482942.36, 4421353.33])
        rot_extra = 0.5759
        rot = -1.5708 + rot_extra * 0.25

        transform = SE2Transforms(origin=origin, rot_radians=rot, inverse=True)

        new_nodes = []
        new_connections = []
        if "regions" in incoming_graph.keys():
            for region_node in incoming_graph["regions"]:
                name, attrs = self._parse_node_data(region_node, transform = transform)
                name += uav_node_suffix
                attrs["type"] = "region"
                self._graph.update_with_node(node=name, edges=[], attrs=attrs)
                attrs["name"] = name
                new_nodes.append(attrs)

        if "objects" in incoming_graph.keys():
            for object_node in incoming_graph["objects"]:
                name, attrs = self._parse_node_data(object_node, transform=transform)
                name += uav_node_suffix
                attrs["type"] = "object"
                self._graph.update_with_node(node=name, edges=[], attrs=attrs)

                attrs["name"] = name
                new_nodes.append(attrs)

        if "region_connections" in incoming_graph.keys():
            for region_connection in incoming_graph["region_connections"]:
                source = region_connection[0] + uav_node_suffix
                target = region_connection[1] + uav_node_suffix
                if self._graph.contains_node(source) and self._graph.contains_node(
                    target
                ):
                    self._graph.update_with_edge(edge=(source, target), attrs={"type": "region"})
                    new_connections.append((source, target))
                else:
                    self._log_info(
                        f"[uav graph sub] [WARNING] {source} or {target} not in graph"
                    )

        if "object_connections" in incoming_graph.keys():
            for object_connection in incoming_graph["object_connections"]:
                source = object_connection[0] + uav_node_suffix
                target = object_connection[1] + uav_node_suffix
                if self._graph.contains_node(source) and self._graph.contains_node(
                    target
                ):
                    self._graph.update_with_edge(edge=(source, target), attrs={"type": "object"})
                    new_connections.append((source, target))
                else:
                    self._log_info(
                        f"[uav graph sub] [WARNING] {source} or {target} not in graph"
                    )

        for node in new_nodes:
            self._log_info(f"have node: {node}")

        prompt_former.update(new_nodes=new_nodes, new_connections=new_connections)
        self._graph_viz.set_graph(self._graph)


    def _uav_graph_sub(self, graph_msg: String) -> None:
        try:
            if self._no_uav_graph_yet:
                self._parse_uav_graph(
                    graph_as_str=graph_msg.data, uav_name="titan_1"
                )
                self._no_uav_graph_yet = True

        except Exception as ex:
            self._log_info(f"[uav graph sub] got exception {ex}")

    def _interrupt_cbk(
        self, request: Mission.Request, response: Mission.Response
    ) -> Mission.Response:
        self._interrupt_msg = request.spec
        response.resp = "interrupt received"
        return response

    def _mission_cbk(
        self, request: Mission.Request, response: Mission.Response
    ) -> Mission.Response:
        self.get_logger().info(f"[spine multi node] got mission: {request.spec}")

        _, mission_labels = self._class_llm.request(request.spec)
        self._log_info(f"setting mission labels: {mission_labels}")
        self._set_labels_to_tasks(",".join(mission_labels["classes"]))
 
        updates = ""
        if not self._collaborator.has_mission():
            self._collaborator.init_planner(mission_specifications=request.spec)
        else:
            updates = self._get_new_spec_updates(request.spec)

        allocation_result = self._collaborator.get_allocation(updates=updates)

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

            assignment_str += ", ".join(
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
                    assignment_str += ", ".join(
                        [f"{r} was assigned {t}" for (r, t) in assignments]
                    )

            # at the end of each planning iteration, empty mapping queue, form updates
            # and send to LLM
            for robot, autonomy_manager in self._robot_autonomy_managers.items():
                autonomy_manager._tracker_component.parse_track_updates()
                robot_feedback = autonomy_manager._prompt_former.form_updates()
                updates += f"{robot} updates: {robot_feedback}\n"

            updates += f"\nPrevious assignments: {assignment_str}\n"

            self._log_info(f"sending updates: {updates}")

            if self._interrupt_msg != "":
                updates += self._get_new_spec_updates(self._interrupt_msg)
                self._interrupt_msg = ""

            allocation_result = self._collaborator.get_allocation(updates=updates)
            self._log_info(f"{self._collaborator.get_result_str(allocation_result)}")

        response.resp = allocation_result.mission_answer
        return response

    def _get_new_spec_updates(self, spec: str) -> str:
        """Get update in API for llm

        Parameters
        ----------
        spec : str
            New mission specification

        Returns
        -------
        str
            Updates to provide to LLM
        """
        self._log_info(f"got interrupt msg: {spec}")
        updates = f"Updated mission: {spec}\n"
        return updates


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
                f"[spine node] robot results are done for {robot}"
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
