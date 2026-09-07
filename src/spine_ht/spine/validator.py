import inspect
from collections import namedtuple
from logging import Logger
from typing import Tuple

import numpy as np

import spine_ht.decomp.planning_api as planning_api
from spine_ht.spine.mapping.graph_util import GraphHandler

# this is a bit cumbersome b/c functions are mapped
VALID_ACTIONS_MULTI_SUGGEST = [
    name
    for name, obj in inspect.getmembers(planning_api, inspect.isfunction)
    if obj.__module__ == planning_api.__name__
]
VALID_ACTIONS_MULTI = set(
    [
        "goto",
        "map_region",
        "inspect",
        "extend_map",
        "explore_to_coord",
        "explore_to_node",
        "uav_map_region",
        "uav_explore_to",
    ]
)

ValidPlanFeedback = namedtuple("ValidPlanFeedback", ["success", "message"])
VALID_ACTIONS = set(
    [
        "explore_region",
        "map_region",
        "inspect",
        "clarify",
        "goto",
        "answer",
        "extend_map",
        "replan",
    ]
)

REGION_ACTIONS = set(["explore_region", "map_region", "goto"])
NAVIGATION_ACTIONS = set(["explore_region", "map_region", "goto", "inspect"])
INTERACTION_ACTIONS = set(["explore_region", "map_region", "goto", "inspect"])
OBJECT_ACTIONS = set(["inspect"])
SPATIAL_ACTION = set(["extend_map"])
EXPLORE_ACTIONS = set(["explore_region"])


class Validator:
    def __init__(self, logger: Logger, validator_type="multi"):
        self.logger = logger

        if validator_type == "multi":
            self.valid_actions = VALID_ACTIONS_MULTI
            self.suggest_actions = VALID_ACTIONS_MULTI_SUGGEST
        else:
            self.valid_actions = VALID_ACTIONS
            self.suggest_actions = VALID_ACTIONS

    def try_parse_region_arg(self, arg: str) -> Tuple[bool, np.ndarray]:
        """Try to parse argument for region related action"""
        try:
            parsed_arg = np.array([float(x) for x in arg.split(",")])
            return True, parsed_arg
        except Exception:
            return False, arg

    def try_parse_exploration_arg(self, arg: str) -> Tuple[bool, Tuple[str, float]]:
        """Try to parse argument for exploration action"""
        try:
            arg = arg.split(",")
            region = arg[0].strip()
            radius = float(arg[1].strip())
            return True, (region, radius)
        except Exception:
            return False, (arg, arg)

    def try_parse_inspection_arg(self, arg: str) -> Tuple[bool, Tuple[str, str]]:
        """Try to parse argument for inspection action"""
        try:
            arg = arg.split(",")
            object = arg[0].strip()
            query = "".join(arg[1:]).strip()
            return True, (object, query)
        except Exception:
            return False, (arg, arg)

    def clean_llm_output(self, s: str) -> str:
        return s.strip("```").strip("json")

    def clean_plan_argument(self, s: str) -> str:
        """remove extra quotes, etc."""
        return s.strip().strip("'").strip('"')

    def _try_parse_command(self, cmd: str) -> Tuple[Tuple[str, str], bool]:
        """Try to parse the LLM-generated command. Essentially syntax checking

        Returns
        -------
        Tuple[Tuple[str, str], bool]
            (function, argument), successfully parsed
        """
        try:
            # TODO bit of a hack but freeform answers may
            # break parsing. should fix later
            if cmd.startswith("answer"):
                function = "answer"
                arg = cmd[7:]
                arg = arg[:-1]
                arg = self.clean_plan_argument(arg)
                return (function, arg), True
            else:
                function, arg = cmd.split("(")
                function = function.strip()
                arg = arg.split(")")[0]
                arg = self.clean_plan_argument(arg)
                return (function, arg), True
        except ValueError:
            return (), False

    def first_element_in_arg(self, arg: str) -> str:
        if "," in arg:
            return arg.split(",")[0].strip()
        else:
            return arg

    def validate_plan(
        self, plan_as_str: str, graph: GraphHandler
    ) -> Tuple[str, ValidPlanFeedback]:
        parsed_plan = []

        for step in plan_as_str:
            # command = re.findall("[a-zA-Z0-9_]+", step)
            command, success = self._try_parse_command(step)
            # command must be 'function(arg)
            if not success:  # len(command) != 2:
                return [], ValidPlanFeedback(
                    False, f"Could not parse step: {step} in plan."
                )
            function, arg = command

            # navigation function require first argument to be a region. pull that out
            # here to simplify checking.
            first_arg = self.first_element_in_arg(arg)

            split_args = arg.split(",")

            if function == "explore_to_node" and graph.path_exists_from_current_loc(
                first_arg
            ):
                feedback = f"Path exists to {arg[0]}. Do not use `explore_to_node.` Try calling `ugv_navigate` instead."

            if function == "map_region" and len(split_args) > 1:
                feedback = f"map_region takes one region node as argument. Try calling explore_to_node({split_args[0]})"
                return [], ValidPlanFeedback(False, feedback)

            # is valid function
            if function not in self.valid_actions:
                feedback = (
                    f"Feedback: {function} is not a valid command. You must use one of the "
                    f"following commands {list(self.suggest_actions)}. Update your plan accordingly."
                )
                return [], ValidPlanFeedback(False, feedback)

            # check that argument is in the graph, if required
            elif function in INTERACTION_ACTIONS and not graph.contains_node(first_arg):
                self.logger.info(
                    f"couldn't find node {first_arg} in graph: {graph.graph.nodes}"
                )
                feedback = (
                    f"Feedback: scene does not contain {first_arg}. "
                    f"All plans must reference nodes in the current scene. "
                    f"Update your plan accordingly."
                )
                return [], ValidPlanFeedback(False, feedback)

            # if navigation command, check that it's reachable
            elif (
                function in NAVIGATION_ACTIONS
                and not graph.path_exists_from_current_loc(first_arg)
            ):
                feedback = (
                    f"Feedback: {function}({first_arg}) is not currently feasible - no path from current location, {graph._current_location}, "
                    f"to goal {first_arg}. "
                )
                (
                    closest_reachable_node,
                    target_node,
                ) = graph.get_closest_reachable_node(goal_node=first_arg)
                feedback += (
                    f"The closest pair of nodes in the connected components of {graph._current_location} and {first_arg}"
                    f" are {closest_reachable_node} and {target_node}. "
                )

                feedback += f"Try calling explore_to_node({target_node}) to find a path"

                feedback += (
                    f"\nAnd remove {function}({first_arg}) from your current plan"
                )

                # NOTE original feedback below
                # feedback += (
                #     f"If you find a connection between that pair via `map_region()`, you can reach {first_arg}. If there is no connection, you will need to find another path. "
                #     "Update your plan accordingly. Note that your relevant_map and long-term goals may stay the same."
                # )

                return [], ValidPlanFeedback(False, feedback)

            # if navigation command, check that argument is region
            elif (
                function in REGION_ACTIONS
                and not graph.get_node_type(first_arg) == "region"
            ):
                node_type = graph.get_node_type(first_arg)
                assert node_type == "object", f"Must implement logic for {node_type}"
                feedback = (
                    "Feedback: Only region nodes can be given as arguments for navigation actions: "
                    f"{list(REGION_ACTIONS)}. Got {first_arg} of type {graph.get_node_type(first_arg)} for command {function}."
                    f" Consider using one of the following functions: {list(OBJECT_ACTIONS)}. The preceding part of your plan is valid. "
                    f"Update accordingly."
                )
                return [], ValidPlanFeedback(False, feedback)

            elif function in SPATIAL_ACTION and not self.try_parse_region_arg(arg)[0]:
                feedback = (
                    f"Feedback: the extend_map function takes in a coordinate in (x, y) numeric. "
                    f"Got exception when trying to parse {arg}. Update your plan accordingly."
                )
                return [], ValidPlanFeedback(False, feedback)
            elif (
                function in EXPLORE_ACTIONS
                and not self.try_parse_exploration_arg(arg)[0]
            ):
                feedback = (
                    f"Feedback: the explore_region function takes in a region name (str) and radius (float). "
                    f"Got exception when trying to parse {arg}. Update your plan accordingly."
                )
            elif (
                function in OBJECT_ACTIONS
                and not graph.get_node_type(first_arg) == "object"
            ):
                # TODO make better
                # feedback = f"Feedback: inspect requires an object, but {first_arg} is a region. Try calling map_region({first_arg}) or explore_region({first_arg}, 3) to get information about the area, depending on the task"
                feedback = f"Feedback: inspect requires an object, but {first_arg} is a region. Try calling map_region({first_arg}) to get information about the area, depending on the task"

                return [], ValidPlanFeedback(False, feedback)
            else:
                if function in SPATIAL_ACTION:
                    success, arg = self.try_parse_region_arg(arg)

                elif function in EXPLORE_ACTIONS:
                    success, arg = self.try_parse_exploration_arg(arg)

                elif function in OBJECT_ACTIONS:
                    success, arg = self.try_parse_inspection_arg(arg)

                parsed_plan.append((function, arg))

        return parsed_plan, ValidPlanFeedback(True, "Is valid plan")
