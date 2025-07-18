#!/usr/bin/env python3

from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from spine_multi_ros.action_manager import ActionManager
from spine_multi_ros.nav_client import NavigationComponent
from teaming_msgs.srv import Mission
from visualization_msgs.msg import Marker

from spine_multi.spine import SPINE, GraphHandler
from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent, GraphViz


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

        self._action_manager = ActionManager(
            parent_node=self,
            graph=self._graph,
            graph_viz=self._graph_viz,
            prompt_former=self._prompt_former,
            nav_componenet=self._nav_componenet,
        )

        self._behavior_library = self._action_manager.construct_behavior_library()

        self.mission_srv = self.create_service(Mission, "mission", self._mission_cbk)

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._costmap_cbk, 10
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

        # self.get_logger().info('processed costmap')

    # def _add_frontier(self, goal: np.ndarray) -> List[Node]:
    #     if self._frontier_extractor.filtered_costmap_with_info is None:
    #         self.get_logger().info(f"no costmap")
    #         return False
    #     current_location = self._graph.get_current_location()
    #     frontiers, is_at_obstacle = self._frontier_extractor.get_frontiers(
    #                     proposed_frontier=goal,
    #                     current_location=current_location,
    #                 )

    #     self._add_frontiers_to_graph(frontiers=frontiers)

    #     return frontiers

    # def extend_map(self, goal = np.array) -> bool:
    #     x = float(goal[0])
    #     y = float(goal[1])
    #     frontiers =  self._add_frontier(np.array([x, y]).reshape(1, 2))

    #     assert len(frontiers) in (0, 1)

    #     if len(frontiers) == 1:
    #         return self._goto_region(frontiers[0].id)
    #     else:
    #         return False

    # def _add_frontiers_to_graph(
    #     self, frontiers: np.ndarray, debug: Optional[bool] = False
    # ) -> Tuple[List[Node], bool]:
    #     """Compute frontiers and add them to graph.

    #     Parameters
    #     ----------
    #     debug : Optional[bool], optional
    #         If true, don't actually update graph. Just get
    #         update message, by default False

    #     Returns
    #     -------
    #     - Update message in LLM API.
    #     - frontier (assumes one currently)
    #     - is frontier at obstacle boundary
    #     """
    #     for frontier in frontiers:
    #         region_id = frontier.id
    #         region_loc = frontier.location
    #         neighbor_ids = frontier.neighbors

    #         print(f"adding node: {region_id}, {region_loc}, {neighbor_ids}")

    #         self._graph.update_with_node(
    #             node=region_id,
    #             edges=neighbor_ids,
    #             attrs={"coords": region_loc, "type": "region"},
    #         )

    #         new_node = {
    #             "name": region_id,
    #             "type": "region",
    #             "coords": f"[{region_loc[0]:0.1f}, {region_loc[1]:0.1f}]",
    #         }
    #         new_connections = [[region_id, c] for c in neighbor_ids]
    #         self._prompt_former.update(
    #             new_nodes=[new_node], new_connections=new_connections
    #         )

    #     self._graph_viz.set_graph(self._graph)

    # def _graph_nav_to_region(self, goal_region: str) -> bool:
    #     current_location = self._graph.get_current_location()
    #     if current_location == goal_region:
    #         return True

    #     path = self._graph.get_path(current_location, goal_region)
    #     self.get_logger().info(f"navigating along path: {path}")

    #     nav_success = True
    #     for node in path:
    #         if current_location == node:
    #             continue

    #         coords = self._graph.get_node_coord(node)
    #         response = self._nav_componenet.navigate_and_wait(x=coords[0], y=coords[1], yaw = 0)

    #         nav_success = response

    #         if nav_success:
    #             self._graph.update_location(node)
    #         else:
    #             self._prompt_former.update(
    #                 freeform_updates=[
    #                     f"could not navigate between [{self.current_location}, {node}]. Connection is likely blocked."
    #                 ]
    #             )
    #             self._graph.remove_edge(current_location, node)
    #             self._prompt_former.update(
    #                 removed_connections=[[current_location, node]]
    #             )

    #             return False

    #     return True

    # def _goto_region(self, region_id: str) -> bool:
    #     response = self._graph_nav_to_region(region_id)
    #     return response

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

    # node._graph.update_with_node('region_1', edges=[], attrs={"type": "region", "coords": (0, 0)})
    # node._graph.update_with_node('region_2', edges=['region_1'], attrs={"type": "region", "coords": (5, 0)})
    # node._graph.update_location('region_1')

    # node._graph_viz.set_graph(node._graph)

    # node._graph_nav_to_region('region_2')

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
