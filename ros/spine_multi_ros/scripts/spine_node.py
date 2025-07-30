#!/usr/bin/env python3

from typing import List, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from spine_multi.spine import SPINE, GraphHandler
from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent
from teaming_msgs.srv import Mission, Query

from spine_multi_ros.action_manager import ActionManager
from spine_multi_ros.nav_manager import NavigationComponent
from spine_multi_ros.tracker_client import TrackerClientComponenet


class SPINE_node(Node):

    def __init__(self):
        super().__init__("spine_node")

        self.declare_parameters(
            namespace="",
            parameters=[("init_graph", ""), ("target_frame", "odom")],
        )
        init_graph = self.get_parameter("init_graph").get_parameter_value().string_value
        target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        self.get_logger().info(f"[spine node] using graph: {init_graph}")

        self._graph = GraphHandler(init_graph)
        self._spine = SPINE(self._graph)
        self._frontier_extractor = FrontierExtractor(self._graph)
        self._prompt_former = UpdatePromptFormer()
        self._nav_component = NavigationComponent(self, frame_id=target_frame)
        self._graph_viz = GraphVisualizerComponent(
            parent_node=self,
            graph=self._graph,
            target_frame=target_frame,
            topic_name="graph_viz",
            scale=0.5,
        )
        self._graph_viz.set_graph(self._graph)

        self._tracker_componenet = TrackerClientComponenet(
            self, self._graph, self._prompt_former, self._graph_viz
        )

        vlm_cbk_group = ReentrantCallbackGroup()
        self._vlm_client = self.create_client(
            Query, "/vlm_node/query_scene", callback_group=vlm_cbk_group
        )
        self._vlm_client.wait_for_service()

        self._action_manager = ActionManager(
            parent_node=self,
            graph=self._graph,
            graph_viz=self._graph_viz,
            prompt_former=self._prompt_former,
            nav_componenet=self._nav_component,
            vlm_client=self._vlm_client,
            frontier_extractor=self._frontier_extractor,
        )

        self._behavior_library = self._action_manager.construct_behavior_library()

        self.mission_srv = self.create_service(Mission, "mission", self._mission_cbk)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        costmap_cbk_group = ReentrantCallbackGroup()
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._costmap_cbk,
            qos_profile,
            callback_group=costmap_cbk_group,
        )

        self._param_client = self.create_client(
            SetParameters, "/controller_server/set_parameters"
        )

        self.get_logger().info("[spine node] initialized")

    def _costmap_cbk(self, costmap_msg: OccupancyGrid) -> None:
        # self.get_logger().info('[spine node] get costmap')
        pose_in_map = costmap_msg.info.origin
        position = np.array([pose_in_map.position.x, pose_in_map.position.y])
        yaw = Rotation.from_quat(
            [
                pose_in_map.orientation.x,
                pose_in_map.orientation.y,
                pose_in_map.orientation.z,
                pose_in_map.orientation.w,
            ]
        ).as_euler("xyz")[0]
        costmap = np.array(costmap_msg.data).reshape(
            costmap_msg.info.height, costmap_msg.info.width
        )
        filtered_costmap = self._frontier_extractor.update_costmap(
            costmap=costmap,
            resolution_m_p_cell=costmap_msg.info.resolution,
            pos=position,
            yaw=yaw,
        )

    def realize_mission(self, plan: List[Tuple[str, str]]) -> Tuple[bool, bool]:
        success = False
        done = False
        should_break = False

        for function, arg in plan:
            self.get_logger().info(f"[spine node] On step: {str(function)}({str(arg)})")
            assert function in self._behavior_library, f"{function} is not in behavior library"

            success = self._behavior_library[function](arg)

            if function == "answer":
                done = True
                should_break = True

            if function == "replan":
                should_break = True

            self.get_logger().info(
                f"[spine node] Step : {str(function)}({str(arg)}) finished with done: {done}, should_break: {should_break}, success: {success}"
            )


            if should_break:
                return success, done


        return success, done

    def _mission_cbk(
        self, request: Mission.Request, response: Mission.Response
    ) -> Mission.Response:
        self.get_logger().info(f"[spine node] got mission: {request.spec}")

        prompt = request.spec

        while True:
            spine_resp, success, logs = self._spine.request(prompt)
            self.get_logger().info(str(spine_resp))
            success, done = self.realize_mission(spine_resp["plan"])

            if done or not success:
                self.get_logger().info(f"[spine node]: breaking mission cbk with done: {done}, sucess: {success}")
                break

            # flush track queue from planning iteration
            self._tracker_componenet.parse_track_updates()
            prompt = self._prompt_former.form_updates()
            self.get_logger().info(f"[spine node] finished planning iteration")

        response.resp = str(spine_resp["plan"])

        return response


def main():
    rclpy.init()
    node = SPINE_node()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
