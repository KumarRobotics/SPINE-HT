from dataclasses import dataclass
from typing import Any, List, Tuple

from spine_multi.decomp.util import parse_function_call


@dataclass
class FunctionDescription:
    name: str
    expected_args: int


# fmt: off
FUNCTION_MAPPINGS = {
    "ugv_inspect": FunctionDescription('inspect', 2), 
    "ugv_map": FunctionDescription('map', 1), 
    "ugv_explore_to": FunctionDescription('explore_to', 2),
    "ugv_navigate": FunctionDescription("goto", 1),
}
# fmt: on


def translate_task_for_execution(task: str) -> Tuple[str, List[Any]]:
    """The function names used by the multi robot coordinator may
    be different than the functions accepted by each robot.

    This is a simple translation layer.
    """
    name, args, kwargs = parse_function_call(task)

    mapped_function = FUNCTION_MAPPINGS[name]

    all_args = args + list(kwargs.values())

    return mapped_function.name, all_args[: mapped_function.expected_args]
