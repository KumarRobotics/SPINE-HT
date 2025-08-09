import textwrap
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import networkx as nx

from spine_multi.allocation.assignment import (
    RobotDescription,
    RobotTaskAssigment,
    TaskDescription,
)
from spine_multi.allocation.translator import translate_task_for_execution
from spine_multi.decomp.llm import MissionDecomp
from spine_multi.logging import get_logger
from spine_multi.spine.mapping.graph_util import GraphHandler

JACKAL_CAPABILITIES = [
    "ugv_map",
    "ugv_inspect",
    "ugv_navigate",
    "ugv_explore_to",
    "jackal",
]
HUSKY_CAPABILITIES = [
    "ugv_map",
    "ugv_inspect",
    "ugv_navigate",
    "ugv_explore_to",
    "husky",
]
FALCON_4_CAPABILITIES = ["uav_goto", "uav_map", "uav_explore"]


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

    assigmnets: Tuple[str, str]

    translated_assigmnets: Tuple[str, Tuple[str, List[str]]]

    mission_is_done: bool
    mission_answer: str


class Collaborator:
    def __init__(
        self,
        team_specification: List[RobotDescription],
        team_spec_language: str,  # TODO auto from above
        semantic_graph: GraphHandler,
        log: Optional[bool] = True,
    ):
        self._allocator = RobotTaskAssigment()
        self._allocator.add_robots(team_specification)
        self._semantic_graph = semantic_graph
        self._mission_decomp = MissionDecomp(team_specification=team_spec_language)
        self._updates_given_as_tasks = set()
        self.log = log

        # used for reassigning  tasks
        self._iteration_trace = None

    def init_planner(self, mission_specifications: str) -> None:
        self._mission_decomp.set_specifications(
            mission=mission_specifications,
            scene_graph=self._semantic_graph.to_json_str(),
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

    def get_allocation(self, updates: Optional[str] = "") -> AllocationResult:
        if updates != "":
            self._mission_decomp.give_updates(updates=updates)

        mission_graph, out = self._mission_decomp.get_mission_graph()

        # self._mission_decomp.save_graph(mission_graph)

        mission_is_done = False
        if out["mission_answer"] != "":
            mission_is_done = True
            return self.get_done_result(
                mission_graph=mission_graph, log=out, answer=out["mission_answer"]
            )

        traces = self._mission_decomp.get_mission_traces(mission_graph)
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
            assigmnets=assignments,
            translated_assigmnets=translated_assignments,
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
            assigmnets=[],
            translated_assigmnets=[],
            mission_is_done=True,
            mission_answer=answer,
        )

    def get_result_str(self, result: AllocationResult) -> str:
        log = ""
        log += f"Specification:\n\t{result.specification}\n\n"
        log += f"Robots:\n\t{result.team}\n\n"
        log += self._mission_decomp.output_log(result.decomposition_log)
        log += f"mission traces:\n\t{result.mission_traces}\n\n"
        log += f"assignment set:\n\t{result.assignment_set}\n\n"
        log += f"assignments:\n\t{result.assigmnets}\n\n"
        log += f"translated assignments:\n\t{result.translated_assigmnets}\n\n"
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
