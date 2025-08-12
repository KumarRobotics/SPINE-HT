from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent

from spine_multi_ros.nav_manager import NavigationComponentTopic
from spine_multi_ros.vlm_manager import VLMManager

class ActionManager:
    def __init__(
        self,
        parent_node: Node,
        robot_name: str,
        graph: GraphHandler,
        graph_viz: GraphVisualizerComponent,
        prompt_former: UpdatePromptFormer,
        nav_componenet: NavigationComponentTopic,
        vlm_client: VLMManager,
        frontier_extractor: FrontierExtractor,
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name
        self._graph = graph
        self._frontier_extractor = frontier_extractor
        self._graph_viz = graph_viz
        self._prompt_former = prompt_former
        self._nav_componenet = nav_componenet
        self._vlm_client = vlm_client

    def construct_behavior_library(self) -> Dict[str, Callable]:
        return {
            "extend_map": self._explore_to,
            "goto": self._goto_region,
            "inspect": self._inspect_object_wrapper,
            "explore_region": lambda x: self._goto_region(x[0]),
            "map_region": self._goto_region,  # TODO placeholder
            "replan": lambda x: True,
            "clarify": lambda x: True,
            "answer": lambda x: True,
        }

    def _log_info(self, msg):
        self._parent_node.get_logger().info(f"[action manager] [{self._robot_name}] {msg}")

    def _explore_to(self, goal_x: float, goal_y: float) -> bool:
        x = float(goal_x)
        y = float(goal_y)
        success, frontiers = self._add_frontier(np.array([x, y]).reshape(1, 2))

        if not success:
            return False

        assert len(frontiers) in (0, 1)

        if len(frontiers) == 1:
            return self._goto_region(frontiers[0].id)
        else:
            return False

    def _inspect_object_wrapper(self, name: str, query: str) -> bool:
        return self._inspect_object(name, query)

    def _inspect_object(self, node_name: str, vlm_query: str) -> bool:
        nearest_region = self._graph.get_neighbors(node_name)
        assert (
            len(nearest_region) == 1
        ), f"objects should only have 1 neighbor. Got: {nearest_region}"

        nearest_region = nearest_region[0]

        self._log_info("inspect object region nav: {nearest_region} from {self._graph.get_current_location()}")

        # nav_success = self._goto_region(nearest_region)

        nav_success = self._graph_nav_to_object(node_name)

        self._log_info(
            f"finished nav with success: {nav_success}"
        )

        if not nav_success:
            self._prompt_former.update(
                freeform_updates=[
                    f"Could not inspect {node_name} because robot could not navigate "
                    f"to neighboring region {nearest_region}"
                ]
            )
            return False

        self._log_info(f"querying vlm")

        success, response = self._vlm_client._query_vlm(vlm_query)

        self._log_info(f"done vlm query: {response}")

        if success:
            self._prompt_former.update(
                attribute_updates=[{"name": node_name, "description": response}]
            )
            return True
        else:
            # TODO figure out what to do here
            self._prompt_former.update(freeform_updates=["unable to inspect object"])
            return True

    def _add_frontier(self, goal: np.ndarray) -> Tuple[bool, List[Node]]:
        if self._frontier_extractor.filtered_costmap_with_info is None:
            self._log_info(f"ERROR no costmap")
            return False, None
        current_location = self._graph.get_current_location()
        frontiers, is_at_obstacle = self._frontier_extractor.get_frontiers(
            proposed_frontier=goal,
            current_location=current_location,
        )

        self._add_frontiers_to_graph(frontiers=frontiers)

        return True, frontiers

    def _add_frontiers_to_graph(
        self, frontiers: list[Node], debug: Optional[bool] = False
    ) -> Tuple[List[Node], bool]:
        """Compute frontiers and add them to graph.

        Parameters
        ----------
        debug : Optional[bool], optional
            If true, don't actually update graph. Just get
            update message, by default False

        Returns
        -------
        - Update message in LLM API.
        - frontier (assumes one currently)
        - is frontier at obstacle boundary
        """
        for frontier in frontiers:
            region_id = frontier.id
            region_loc = frontier.location
            neighbor_ids = frontier.neighbors

            self._log_info(
                f"adding node: {region_id}, {region_loc}, {neighbor_ids}"
            )

            self._graph.update_with_node(
                node=region_id,
                edges=neighbor_ids,
                attrs={"coords": region_loc, "type": "region"},
            )

            new_node = {
                "name": region_id,
                "type": "region",
                "coords": f"[{region_loc[0]:0.1f}, {region_loc[1]:0.1f}]",
            }
            new_connections = [[region_id, c] for c in neighbor_ids]
            self._prompt_former.update(
                new_nodes=[new_node], new_connections=new_connections
            )

        self._graph_viz.set_graph(self._graph)

    def _graph_nav_to_region(self, goal_region: str) -> bool:
        current_location = self._graph.get_current_location()
        if current_location == goal_region:
            return True

        self._log_info(
            f"computing path between {current_location} -> {goal_region}"
        )

        path = self._graph.get_path(current_location, goal_region)
        self._log_info(f"navigating along path: {path}")

        nav_success = True
        for node in path:
            self._log_info(f"Next node: {node}")
            if current_location == node:
                continue

            coords = self._graph.get_node_coord(node)
            response = self._nav_componenet.navigate_and_wait(
                x=coords[0], y=coords[1], yaw=0
            )

            nav_success = response

            self._log_info(f"graph nav step done")

            if nav_success:
                self._graph.update_location(node)
                self._prompt_former.update(location_updates=[node])
            else:
                self._prompt_former.update(
                    freeform_updates=[
                        f"could not navigate between [{self._graph.get_current_location()}, {node}]. Connection is likely blocked."
                    ]
                )
                self._graph.remove_edge(current_location, node)
                self._prompt_former.update(
                    removed_connections=[[current_location, node]]
                )

                return False

        return True

    def _graph_nav_to_object(self, goal_object: str) -> bool:
        nearest_region = self._graph.get_neighbors(goal_object)

        assert (
            len(nearest_region) == 1
        ), f"objects should only have 1 neighbor. Got: {nearest_region}"

        nearest_region = nearest_region[0]

        region_coords = self._graph.get_node_coord(nearest_region)
        object_coords = self._graph.get_node_coord(goal_object)

        direction_region_to_obj = object_coords - region_coords
        angle = np.arctan2(direction_region_to_obj[1], direction_region_to_obj[0])

        success_region = self._graph_nav_to_region(nearest_region)

        response = self._nav_componenet.navigate_and_wait(
            x=region_coords[0], y=region_coords[1], yaw=angle, check_yaw=True
        )

        return response

    def _goto_region(self, region_id: str) -> bool:
        response = self._graph_nav_to_region(region_id)
        return response
