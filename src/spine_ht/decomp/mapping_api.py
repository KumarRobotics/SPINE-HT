from typing import Dict, List, Tuple


def remove_nodes(removed_nodes: List[str]) -> None:
    """Remove `nodes` and associated edges from graph."""


def add_nodes(new_nodes: List[Dict[str, str]]) -> None:
    """Add nodes to graph. Each node is represented as a dictionary."""


def add_connections(new_connections: List[Tuple[str, str]]) -> None:
    """Add a list of connections. Each element is a tuple of the connecting nodes."""


def remove_connections(removed_connections: List[Tuple[str, str]]) -> None:
    """Removes a list of connections. Each element is a tuple of the endpoint nodes in the edge."""


def update_robot_location(region_node: str) -> None:
    """Update robot's location in the graph to `region_node`."""


def update_node_attributes(attribute: List[Dict[str, str]]) -> None:
    """Update node's attributes. Each entry of the input is a dictionary of new node values.
    Entries will include the referent node's name."""


def no_updates() -> None:
    """There have been no updates."""
