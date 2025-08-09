#!/usr/bin/env python3


from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List

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
)
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi_ros.autonomy_manager import AutonomyManager
from teaming_msgs.srv import Mission


# TODO should go into src
@dataclass
class RobotConfig:
    name: str
    namespace: str
    init_graph: str
    nav_target_frame: str
    graph_viz_topic: str
    track_topic: str
    vlm_request: str
    vlm_response: str
    vlm_ack: str
    navigation_request: str
    navigation_status: str
    navigation_ack: str
    local_costmap_topic: str
 

    # navigation_action_server: str
    # nav_status_topic: str
    # vlm_query_client_name: str
   # controller_param_server_topic: str


class SPINEMultiNode(Node):
    def __init__(self):
        super().__init__(
            "spine_multi_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        robots = [
            RobotDescription(
                id="jackal_1", type="jackal", capabilities=JACKAL_CAPABILITIES
            ),
            RobotDescription(
                id="jackal_2", type="jackal", capabilities=JACKAL_CAPABILITIES
            ),
            # RobotDescription(id="husky_1", type="husky", capabilities=HUSKY_CAPABILITIES),
            # RobotDescription(
            #     id="uav_1", type="falcon_4", capabilities=FALCON_4_CAPABILITIES
            # )
        ]
        team_specification = (
            self.get_parameter("team_specification").get_parameter_value().string_value
        )
        init_graph = self.get_parameter("init_graph").get_parameter_value().string_value
        self._graph = GraphHandler(graph_path=init_graph)

        self.get_logger().info(f"initi graph is: {self._graph.to_json_str()}")

        self._collaborator = Collaborator(
            team_specification=robots,
            semantic_graph=self._graph,
            team_spec_language=team_specification,
        )
        self._planning_limit = 5

        # just placeholder
        test_param = self.get_parameter("test_param").get_parameter_value().bool_value
        self.get_logger().info(f"test param: {test_param}")

        param_dict = self.get_parameters_by_prefix("robots")
        self._robot_configs = self.parse_params(param_dict)
        self._robot_autonomy_managers = self._init_autonomy_managers(
            self._robot_configs
        )

        self.mission_srv = self.create_service(
            Mission, "/spine_multi/mission", self._mission_cbk
        )

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
                init_graph=config.init_graph,
                nav_target_frame=config.nav_target_frame,
                graph_viz_topic=config.graph_viz_topic,
                track_topic=config.track_topic,
                local_costmap_topic=config.local_costmap_topic,
                navigation_params = {"navigation_request": config.navigation_request,
                                     "navigation_status": config.navigation_status,
                                     "navigation_ack": config.navigation_ack},
                vlm_params= {"vlm_request": config.vlm_request,
                             "vlm_response": config.vlm_response,
                             "vlm_ack": config.vlm_ack}
                )

        return managers

    def _mission_cbk(
        self, request: Mission.Request, response: Mission.Response
    ) -> Mission.Response:
        self.get_logger().info(f"[spine multi node] got mission: {request.spec}")

        self._collaborator.init_planner(mission_specifications=request.spec)
        allocation_result = self._collaborator.get_allocation()

        self.get_logger().info(
            f"{self._collaborator.get_result_str(allocation_result)}"
        )

        for planning_idx in range(self._planning_limit):

            if allocation_result.mission_is_done:
                break

            assignments = allocation_result.translated_assigmnets

            self.get_logger().info(f"Got assignments: {assignments}")

            # this requires all robots to finish their tasks before reassigning.
            # TODO need to add some async logic
            while True:
                for robot, task in assignments:
                    self.get_logger().info(
                        f"[spine node] planning iteration {planning_idx}: commanding robot: {robot}: {task} with type: {type(task[1])}"
                    )

                    behavior, args = task

                    self.get_logger().info(f"args: {args}")

                    success = self._robot_autonomy_managers[robot].call_behavior(
                        behavior, args
                    )

                    self.get_logger().info(f"behavior done")

                if self._collaborator.all_tasks_assigned():
                    break
                else:
                    assignments = self._collaborator.reassign_tasks()

            updates = ""
            for robot, autonomy_manager in self._robot_autonomy_managers.items():
                robot_feedback = autonomy_manager._prompt_former.form_updates()
                updates += f"{robot} updates: {robot_feedback}"

            self.get_logger().info(f"sending updates: {updates}")

            allocation_result = self._collaborator.get_allocation(updates=updates)
            self.get_logger().info(
                f"{self._collaborator.get_result_str(allocation_result)}"
            )

        response.resp = allocation_result.mission_answer
        return response


def main():
    rclpy.init()

    node = SPINEMultiNode()

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
