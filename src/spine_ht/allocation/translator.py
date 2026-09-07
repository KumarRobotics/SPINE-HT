from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

from spine_ht.decomp.util import _parse_function_call


@dataclass
class FunctionDescription:
    name: str
    expected_args: int


# fmt: off
FUNCTION_MAPPINGS = {
    "ugv_inspect": FunctionDescription('inspect', 2),
    "ugv_map_region": FunctionDescription('map_region', 1),
    "ugv_explore_to_coord": FunctionDescription('explore_to_coord', 2),
    "ugv_navigate": FunctionDescription("goto", 1),
    "explore_to_node": FunctionDescription("explore_to_node", 1),
    "uav_map": FunctionDescription("uav_map_region", 1),
    "uav_explore_to": FunctionDescription("uav_explore_to", 2)
}
# fmt: on


# TODO repetative
def translate_task_for_validation(task: str) -> Tuple[str, List[str], Dict[str, str]]:
    name, args, kwargs = _parse_function_call(task)
    mapped_function = FUNCTION_MAPPINGS[name]
    all_args = args + list(kwargs.values())
    expected_args = all_args[: mapped_function.expected_args]
    formatted_args = ",".join(expected_args)

    return f"{mapped_function.name}({formatted_args})"


def translate_task_for_execution(task: str) -> Tuple[str, List[Any]]:
    """The function names used by the multi robot coordinator may
    be different than the functions accepted by each robot.

    This is a simple translation layer.
    """
    name, args, kwargs = _parse_function_call(task)

    mapped_function = FUNCTION_MAPPINGS[name]

    all_args = args + list(kwargs.values())

    return mapped_function.name, all_args[: mapped_function.expected_args]
