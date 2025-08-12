def ugv_navigate(region_node: str, ugv_type: str = ["any", "jackal", "husky"]) -> bool:
    """Navigate to a node in the semantic graph. This MUST refer to an existing node."""


def ugv_inspect(
    object_node: str, query: str, ugv_type=["any", "jackal", "husky"]
) -> str:
    """Inspect `object_node` for a specific attribute `query`. The query is processed by a vision language model (VLA), and the VLA's answer will be provided."""


def ugv_map_region(
    region_node: str, ugv_type: str = ["either", "jackal", "husky"]
) -> str:
    """Explore a node in the semantic graph to add additional nodes."""


def ugv_explore_to(
    x: float, y: float, ugv_type: str = ["any", "jackal", "husky"]
) -> str:
    """Explore an (x, y) coordinate to expand the semantic map. Returns map updates."""


def uav_fly_to(region_node: str) -> bool:
    """Fly to a node in the graph."""


def uav_explore_to(x: float, y: float) -> str:
    """Explore an (x, y) coordinate to expand the semantic map. Returns map updates."""
