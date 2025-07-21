#!/usr/bin/env python3


import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from spine_multi_ros.action_manager import ActionManager
from spine_multi_ros.nav_client import NavigationComponent
from spine_multi_ros.tracker_client import TrackerClientComponenet
from teaming_msgs.srv import Mission

from spine_multi.spine import SPINE, GraphHandler
from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent


class SPINE_node(Node):

    def __init__(self):
        super().__init__("spine_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("init_graph", ""),
            ],
        )
        init_graph = self.get_parameter("init_graph").get_parameter_value().string_value

        self.get_logger().info(f"using graph: {init_graph}")

        self._graph = GraphHandler(init_graph)
        self._spine = SPINE(self._graph)
        self._frontier_extractor = FrontierExtractor(self._graph)
        self._prompt_former = UpdatePromptFormer()
        self._nav_componenet = NavigationComponent(self, frame_id="warthog1/odom")
        self._graph_viz = GraphVisualizerComponent(
            parent_node=self,
            graph=self._graph,
            target_frame="warthog1/odom",
            topic_name="graph_viz",
            scale=0.5,
        )
        self._graph_viz.set_graph(self._graph)

        self._tracker_componenet = TrackerClientComponenet(
            self, self._graph, self._prompt_former, self._graph_viz
        )

        self._action_manager = ActionManager(
            parent_node=self,
            graph=self._graph,
            graph_viz=self._graph_viz,
            prompt_former=self._prompt_former,
            nav_componenet=self._nav_componenet,
        )

        self._behavior_library = self._action_manager.construct_behavior_library()

        self.mission_srv = self.create_service(Mission, "mission", self._mission_cbk)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._costmap_cbk, qos_profile
        )

        self.get_logger().info("SPINE node initialized")

    def _costmap_cbk(self, costmap_msg: OccupancyGrid) -> None:
        # self.get_logger().info('get costmap')
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

    def realize_mission(self, plan) -> bool:
        success = False
        for function, arg in plan:
            self.get_logger().info(f"On step: {str(function)}{str(arg)}")
            if function in self._behavior_library:
                success = self._behavior_library[function](arg)

        return success

    def _mission_cbk(self, request, response):
        self.get_logger().info(request.spec)

        spine_resp, success, logs = self._spine.request(request.spec)

        self.get_logger().info(str(spine_resp))

        self.realize_mission(spine_resp["plan"])

        response.resp = str(spine_resp["plan"])

        return response


def main():
    import time

    rclpy.init()
    node = SPINE_node()

    # for testing
    # node._graph.update_with_node('region_1', edges=[], attrs={"type": "region", "coords": (0, 0)})
    # node._graph.update_with_node('region_2', edges=['region_1'], attrs={"type": "region", "coords": (5, 0)})
    # node._graph.update_location('region_1')
    # node._graph_viz.set_graph(node._graph)
    # node._graph_nav_to_region('region_2')

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
