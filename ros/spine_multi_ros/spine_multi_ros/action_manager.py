from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
from rclpy.node import Node
from spine_multi_ros.nav_client import NavigationComponent

from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent, GraphViz


class ActionManager:
    def __init__(
        self,
        parent_node: Node,
        graph: GraphHandler,
        graph_viz: GraphVisualizerComponent,
        prompt_former: UpdatePromptFormer,
        nav_componenet: NavigationComponent,
    ):
        self._parent_node = parent_node
        self._graph = graph
        self._frontier_extractor = FrontierExtractor(graph)
        self._graph_viz = graph_viz
        self._prompt_former = prompt_former
        self._nav_componenet = nav_componenet

    def construct_behavior_library(self) -> Dict[str, Callable]:
        return {
            "extend_map": self.extend_map,
            "goto": self._goto_region,
            "replan": lambda x: x,
            "clarify": lambda x: x,
            "answer": lambda x: x,
        }

    def extend_map(self, goal: np.array) -> bool:
        x = float(goal[0])
        y = float(goal[1])
        frontiers = self._add_frontier(np.array([x, y]).reshape(1, 2))

        assert len(frontiers) in (0, 1)

        if len(frontiers) == 1:
            return self._goto_region(frontiers[0].id)
        else:
            return False

    def _add_frontier(self, goal: np.ndarray) -> List[Node]:
        if self._frontier_extractor.filtered_costmap_with_info is None:
            self._parent_node.get_logger().info(f"no costmap")
            return False
        current_location = self._graph.get_current_location()
        frontiers, is_at_obstacle = self._frontier_extractor.get_frontiers(
            proposed_frontier=goal,
            current_location=current_location,
        )

        self._add_frontiers_to_graph(frontiers=frontiers)

        return frontiers

    def _add_frontiers_to_graph(
        self, frontiers: np.ndarray, debug: Optional[bool] = False
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

            print(f"adding node: {region_id}, {region_loc}, {neighbor_ids}")

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

        path = self._graph.get_path(current_location, goal_region)
        self._parent_node.get_logger().info(f"navigating along path: {path}")

        nav_success = True
        for node in path:
            if current_location == node:
                continue

            coords = self._graph.get_node_coord(node)
            response = self._nav_componenet.navigate_and_wait(
                x=coords[0], y=coords[1], yaw=0
            )

            nav_success = response

            if nav_success:
                self._graph.update_location(node)
            else:
                self._prompt_former.update(
                    freeform_updates=[
                        f"could not navigate between [{self.current_location}, {node}]. Connection is likely blocked."
                    ]
                )
                self._graph.remove_edge(current_location, node)
                self._prompt_former.update(
                    removed_connections=[[current_location, node]]
                )

                return False

        return True

    def _goto_region(self, region_id: str) -> bool:
        response = self._graph_nav_to_region(region_id)
        return response
