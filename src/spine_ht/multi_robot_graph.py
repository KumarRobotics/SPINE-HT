from typing import Any, Dict, List, Tuple

import networkx as nx
import numpy as np

from spine_ht.spine.mapping.graph_util import GraphHandler


class MultiRobotGraphHandler:
    def __init__(self, graph_handler: GraphHandler, init_node: str):
        self._graph = graph_handler
        self._current_location = init_node

    def get_graph(self) -> GraphHandler:
        return self._graph

    def get_current_location(self) -> str:
        return self._current_location

    def get_node_coords(self, node: str) -> Tuple[np.ndarray, bool]:
        return self._graph.get_node_coords(node)

    def get_node_coord(self, node: str) -> np.ndarray:
        return self._graph.get_node_coord(node)

    def remove_edge(self, start: str, end: str) -> None:
        return self._graph.remove_edge(start, end)

    def get_neighbors(self, node: str) -> List[str]:
        return self._graph.get_neighbors(node)

    def update_with_node(self, node: str, edges: List[str], attrs: Dict[str, Any] = {}):
        return self._graph.update_with_node(node, edges, attrs)

    def update_node_description(self, node: str, **attrs: Dict[str, str]) -> None:
        self._graph.update_node_description(node, **attrs)

    def get_path(self, start_node: str, end_node: str):
        return self._graph.get_path(start_node, end_node)

    def update_location(self, node: str) -> None:
        self._current_location = node

    def to_json_str(self) -> str:
        return self._graph.to_json_str()

    def get_nx_graph(self) -> nx.Graph:
        return self._graph.graph

    def get_node_type(self, node: str) -> str:
        return self._graph.get_node_type(node)

    def lookup_node(self, node: str) -> Tuple[Dict, bool]:
        if self._graph.contains_node(node):
            return self._graph.get_nx_graph().nodes[node], True
        else:
            return {}, False

    def get_region_nodes_and_locs(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._graph.get_region_nodes_and_locs()

    def get_closest_reachable_node(self, node: str) -> str:
        return self._graph.get_closest_reachable_node(node)
