def ugv_navigate(
    region_node: str, ugv_type: str = ["any", "jackal", "husky", "robot_name"]
) -> bool:
    """Navigate to a node in the semantic graph. This MUST refer to an existing node.

    Parameters
    ---
    region_node: str
        Existing node in the graph
    ugv_type: str
        Pick most appropriate option (jackal, husky, or any)
    """


def ugv_inspect(
    object_node: str, query: str, ugv_type=["any", "jackal", "husky", "robot_name"]
) -> str:
    """Navigate to the closest region, then inspect `object_node` for a specific attribute `query`.
    The query is processed by a vision language model (VLA), and the VLA's answer will be provided.
    This subsumes navigating to the region closest to the object.
    """


def ugv_map_region(
    region_node: str, ugv_type: str = ["any", "jackal", "husky", "robot_name"]
) -> str:
    """Navigate to `region_node,` gather a semantic description, and discover nearby objects.
    This subsumes navigation to region_node"""


def ugv_explore_to(
    x: float, y: float, ugv_type: str = ["any", "jackal", "husky", "robot_name"]
) -> str:
    """Try to add a node near coordinate (x, y) to expand your semantic map. Returns map updates.
    Note that you may not reach the coordinate exactly."""


def explore_to_node(region_node: str, ugv_type=["any", "jackal", "husky", "spot", "robot_name"]) -> str:
    """Attempt to find a path to `region_node` if none exists.

    ONLY call this if there is no existing path"""


def uav_fly_to(region_node: str) -> bool:
    """Fly to a node in the graph."""


def uav_explore_to(x: float, y: float) -> str:
    """Explore an (x, y) coordinate to expand the semantic map. Returns map updates."""
