import itertools
import logging
import textwrap
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import networkx as nx

from spine_multi.allocation.assignment import (
    RobotDescription,
    RobotTaskAssigment,
    TaskDescription,
)
from spine_multi.allocation.translator import (
    translate_task_for_execution,
    translate_task_for_validation,
)
from spine_multi.decomp.llm import MissionDecomp
from spine_multi.planner_logging import break_long_str, get_logger
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi.spine.validator import Validator

JACKAL_CAPABILITIES = [
    "ugv_map_region",
    "ugv_inspect",
    "ugv_navigate",
    "ugv_explore_to_coord",
    "explore_to_node",
    "jackal",
]
HUSKY_CAPABILITIES = [
    "ugv_map_region",
    "ugv_navigate",
    "ugv_explore_to_coord",
    "explore_to_node",
    "husky",
]
SPOT_CAPABILITIES = [
    "ugv_inspect",
    "ugv_navigate",
    "ugv_map_region",
    "ugv_explore_to_coord",
    "explore_to_node",
    "spot",
]
FALCON_4_CAPABILITIES = ["uav_goto", "uav_map", "uav_explore"]


# TODO should go into src
@dataclass
class RobotConfig:
    name: str
    namespace: str
    type: str
    subscription_prefix: str
    init_location: str
    nav_target_frame: str
    graph_viz_topic: str
    track_topic: str
    local_costmap_topic: str
    behavior_request_pub: str
    behavior_request_ack_sub: str
    behavior_result_sub: str
    behavior_result_ack_pub: str


@dataclass
class BehaviorResult:
    robot: str
    behavior: str
    success: bool
    error: str


@dataclass
class AllocationResult:
    specification: str
    team: List[RobotDescription]
    mission_graph: nx.DiGraph
    decomposition_log: Dict[str, str]

    # all traces for the mission
    mission_traces: List[List[TaskDescription]]

    # tasks to be assigned this planning iteration
    assignment_set: List[TaskDescription]

    assignments: Tuple[str, str]

    translated_assignments: Tuple[str, Tuple[str, List[str]]]

    mission_is_done: bool
    mission_answer: str


def get_capability_set(robot_name, robot_type):
    if robot_type == "jackal":
        return JACKAL_CAPABILITIES + [robot_name]
    elif robot_type == "husky":
        return HUSKY_CAPABILITIES + [robot_name]
    elif robot_type == "spot":
        return SPOT_CAPABILITIES + [robot_name]
    else:
        raise ValueError(f"{robot_type} not supported")


class Collaborator:
    def __init__(
        self,
        team_specification: List[RobotDescription],
        team_spec_language: str,  # TODO auto from above
        semantic_graph: GraphHandler,
        init_location: str,
        logger: logging.Logger,
        llm_type: Optional[str] = "gpt4",
    ):
        self._logger = logger
        self._allocator = RobotTaskAssigment(self._logger)
        self._allocator.add_robots(team_specification)
        self._semantic_graph = semantic_graph
        self._init_location = init_location
        self._mission_decomp = MissionDecomp(
            team_specification=team_spec_language, llm_type=llm_type
        )
        self._validator = Validator(self._logger)
        self._updates_given_as_tasks = set()

        # TODO this is for validation. Need to figure out
        # if this makes sense
        self._semantic_graph._current_location = init_location

        # used for reassigning  tasks
        self._iteration_trace = None

    def init_planner(self, mission_specifications: str) -> None:
        self._mission_decomp.set_specifications(
            mission=mission_specifications,
            scene_graph=self._semantic_graph.to_json_str(),
            init_location=self._init_location,
        )

    def _provide_allocated_tasks(self):
        updates = ""
        for task in self._allocator._assigned_tasks:
            if task in self._updates_given_as_tasks:
                self._updates_given_as_tasks.add(task)
                updates += f"{task},"

        if len(updates):
            return f"\nAssigned tasks: {updates}"
        else:
            return ""

    def reassign_tasks(self) -> List[Tuple[str, List[Any]]]:
        assigment_tasks = self._allocator.get_assignment_set(self._iteration_trace)
        assignments, cost, task_descriptions = self._allocator.solve_assignment(
            tasks=assigment_tasks
        )

        for assignment in task_descriptions:
            self._allocator._assigned_tasks.add(assignment)

        translated_assignments = []
        for robot, task in assignments:
            robot_task = translate_task_for_execution(task)
            translated_assignments.append((robot, robot_task))

        return translated_assignments

    def all_tasks_assigned(self) -> bool:
        """True if all tasks in the current mission graph have been assigned to a robot"""
        return sum(len(sublist) for sublist in self._iteration_trace) == len(
            self._allocator._assigned_tasks
        )

    def _get_mission_decomposition(
        self,
    ) -> Tuple[bool, List[List[TaskDescription]], nx.DiGraph, Dict[str, str]]:
        MAX_GENERATIONS = 3
        for generation_attempt_idx in range(MAX_GENERATIONS):
            mission_graph, out = self._mission_decomp.get_mission_graph()
            # self._logger.info(f"LLM provided graph: {mission_graph} with raw output {out}")

            traces, log_out = self._mission_decomp.get_mission_traces(mission_graph)

            if len(traces) == 0 and out["mission_answer"] == "":
                self._logger.info(f"WARNING No traces parsed. \nLLM output is {out}\nmission graph: {mission_graph}\nlog: {log_out}")

            llm_task_feedback = []
            for trace in itertools.chain.from_iterable(traces):
                translated_task = translate_task_for_validation(trace.id)
                subtask = [translated_task]
                _, feedback = self._validator.validate_plan(
                    subtask, graph=self._semantic_graph
                )
                if not feedback.success:
                    llm_task_feedback.append(feedback.message)

            if len(llm_task_feedback) == 0:
                return True, traces, mission_graph, out

            self._logger.info(
                f"[idx {generation_attempt_idx}]: Validation for trace: {traces} yeilded"
            )
            for feedback_msg in llm_task_feedback:
                self._logger.info(f"\n{feedback_msg}")

            self._mission_decomp.give_updates(
                "Feedback: " + ",".join(llm_task_feedback)
            )

        return False, traces, mission_graph, out

    def get_allocation(self, updates: Optional[str] = "") -> AllocationResult:
        if updates != "":
            self._mission_decomp.give_updates(updates=updates)

        success, traces, mission_graph, out = self._get_mission_decomposition()

        if not success:
            return self.get_done_result(
                mission_graph=mission_graph,
                log=out,
                answer="Error in mission decomposition",
            )

        mission_is_done = False
        if out["mission_answer"] != "":
            mission_is_done = True
            return self.get_done_result(
                mission_graph=mission_graph, log=out, answer=out["mission_answer"]
            )

        self._iteration_trace = traces

        assigment_tasks = self._allocator.get_assignment_set(traces)
        assignments, cost, task_descriptions = self._allocator.solve_assignment(
            tasks=assigment_tasks
        )
        for assignment in task_descriptions:
            self._allocator._assigned_tasks.add(assignment)

        # translate assignments into format for each robot
        translated_assignments = []
        for robot, task in assignments:
            robot_task = translate_task_for_execution(task)
            translated_assignments.append((robot, robot_task))

        allocation_result = AllocationResult(
            specification=self._mission_decomp.get_mission_specification(),
            team=self._allocator._robots,
            mission_graph=mission_graph,
            decomposition_log=out,
            mission_traces=traces,
            assignment_set=assigment_tasks,
            assignments=assignments,
            translated_assignments=translated_assignments,
            mission_is_done=mission_is_done,
            mission_answer="",
        )
        return allocation_result

    def get_done_result(
        self, mission_graph: nx.Graph, log: str, answer: str
    ) -> AllocationResult:
        return AllocationResult(
            specification=self._mission_decomp.get_mission_specification(),
            team=self._allocator._robots,
            mission_graph=mission_graph,
            decomposition_log=log,
            mission_traces=[[]],
            assignment_set=[],
            assignments=[],
            translated_assignments=[],
            mission_is_done=True,
            mission_answer=answer,
        )

    def get_result_str(self, result: AllocationResult) -> str:
        log = ""
        log += f"Specification:\n\t{result.specification}\n\n"
        log += f"Robots:\n\t{result.team}\n\n"
        log += self._mission_decomp.output_log(result.decomposition_log)
        log += f"mission traces (test):\n"
        for trace in result.mission_traces:
            for task in trace:
                log += f"\t{break_long_str(str(task))}\n\n"
            # trace = trace.replace("TaskDescription", "\n\nTaskDescription")

        # log += f"mission traces:\n"
        # for trace in result.mission_traces:
        #     # for task in trace:
        #     #     log += f"\n{break_long_str(str(task))}"
        #     # trace = trace.replace("TaskDescription", "\n\nTaskDescription")
        #     log += f"\t{break_long_str(str(trace))}\n"
        log += "\n"
        log += f"assignment set:\n"
        for assignment in result.assignment_set:
            log += f"\t{break_long_str(str(assignment))}\n"
        log += "\n"
        log += f"assignments:\n\t{result.assignments}\n\n"
        log += f"all assigned tasks\n\t{break_long_str(str(self._allocator._assigned_tasks))}\n\n"
        log += f"translated assignments:\n\t{result.translated_assignments}\n\n"
        log += f"mission complete:\n\t{result.mission_is_done}\n\n"
        return log

    def print_results(self, result: AllocationResult) -> None:
        out_log = self.get_result_str(result)

        wrapped_lines = [
            textwrap.fill(line, width=127, initial_indent="", subsequent_indent="\t")
            for line in out_log.splitlines()
        ]

        result_str = "\n".join(wrapped_lines)

        print(result_str)


if __name__ == "__main__":
    allocator = RobotTaskAssigment()

    robots = [
        RobotDescription(
            id="jackal_1", type="jackal", capabilities=JACKAL_CAPABILITIES
        ),
        RobotDescription(id="husky_1", type="husky", capabilities=HUSKY_CAPABILITIES),
        RobotDescription(
            id="uav_1", type="falcon_4", capabilities=FALCON_4_CAPABILITIES
        ),
    ]

    collaborator = Collaborator(team_specification=robots)
