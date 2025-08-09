from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import linprog

JACKAL_CAPABILITIES = ["ugv_map", "ugv_inspect", "ugv_goto", "jackal"]
HUSKY_CAPABILITIES = ["ugv_map", "ugv_inspect", "ugv_goto", "husky"]
FALCON_4_CAPABILITIES = ["uav_goto", "uav_map", "uav_explore"]


@dataclass(frozen=True)
class TaskDescription:
    id: str
    requirements: List[str] = field(default_factory=list)
    cost_per_robot: Dict[str, float] = field(default_factory=dict)

    # hang on to extra info.
    args: List[str] = field(default_factory=list)
    kwargs: List[str] = field(default_factory=dict)

    def __hash__(self):
        return hash(
            (
                self.id,
                tuple(self.requirements),
                tuple(sorted(self.cost_per_robot.items())),
                tuple(self.args),
                tuple(sorted(self.kwargs.items())),
            )
        )


@dataclass
class RobotDescription:
    id: str
    type: str
    capabilities: List[str]


class RobotTaskAssigment:
    def __init__(self):
        self._robots: List[RobotDescription] = []
        self._default_feasible_cost = 1.0
        self._default_infeasible_cost = 1000.0
        self._assigned_tasks = set()

    def add_robots(self, robots: List[RobotDescription]) -> None:
        self._robots.extend(robots)

    def _build_cost_matrix(self, tasks: List[TaskDescription]) -> np.ndarray:
        n_robots = len(self._robots)
        n_tasks = len(tasks)

        cost_matrix = np.full((n_robots, n_tasks), self._default_infeasible_cost)

        for i, robot in enumerate(self._robots):
            for j, task in enumerate(tasks):
                if self._robot_can_do_task(robot, task):
                    if robot.id in task.cost_per_robot:
                        cost_matrix[i, j] = task.cost_per_robot[robot.id]
                    else:
                        cost_matrix[i, j] = self._default_feasible_cost

        return cost_matrix

    def get_assignment_set(
        self, traces: List[List[TaskDescription]]
    ) -> List[TaskDescription]:

        # just get first task in trace now
        # placeholder for more compelx logic
        assigments = []
        for trace in traces:
            for task in trace:
                if not task in self._assigned_tasks:
                    assigments.append(task)
                    break

        return assigments

    def _robot_can_do_task(
        self, robot: RobotDescription, task: TaskDescription
    ) -> bool:
        robot_capabilities = set(robot.capabilities)
        task_requirements = set(task.requirements)

        return task_requirements.issubset(robot_capabilities)

    def _add_dummy_tasks(self, tasks: List[TaskDescription]) -> List[TaskDescription]:
        """Enusure there are at least as many tasks as robots to
        avoid trivial solutions. If needed, add dummy tasks which will be
        assigned high cost.
        """
        diff = len(self._robots) - len(tasks)

        cost_per_robot = {n.id: self._default_infeasible_cost for n in self._robots}

        augmented_tasks = []
        augmented_tasks.extend(tasks)

        for i in range(diff):
            augmented_tasks.append(
                TaskDescription(id=f"dummy_{i}", cost_per_robot=cost_per_robot)
            )

        return augmented_tasks

    def solve_assignment(self, tasks: List[TaskDescription]) -> List[Tuple[str, str]]:
        """Assign robot team to given tasks

        Parameters
        ----------
        tasks : List[TaskDescription]
            Tasks to be assigned

        Returns
        -------
        List[Tuple[str, str]]
            Assigments given by (robot_id, task_id)
            Note that not all robots or tasks my be assigned.
        """
        tasks = self._add_dummy_tasks(tasks)
        return self._solve_hungarian(tasks)

    def _solve_hungarian(
        self, tasks: List[TaskDescription]
    ) -> Tuple[List[Tuple[str, str]], float, List[TaskDescription]]:
        n_robots = len(self._robots)
        n_tasks = len(tasks)

        cost_matrix = self._build_cost_matrix(tasks)
        c = cost_matrix.flatten()

        # robot constraints (equality): each robot must be assigned a task
        A_eq_robot = np.zeros((n_robots, n_robots * n_tasks))
        for i in range(n_robots):
            for j in range(n_tasks):
                A_eq_robot[i, i * n_tasks + j] = 1
        b_eq_robot = np.ones(n_robots)

        # Task constraints (inequality): each task gets at most one robot
        A_ub_task = np.zeros((n_tasks, n_robots * n_tasks))
        for j in range(n_tasks):
            for i in range(n_robots):
                A_ub_task[j, i * n_tasks + j] = 1
        b_ub_task = np.ones(n_tasks)

        # Combine constraints
        A_eq = A_eq_robot
        b_eq = b_eq_robot
        A_ub = A_ub_task
        b_ub = b_ub_task

        bounds = [(0, 1) for _ in range(n_robots * n_tasks)]

        result = linprog(
            c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method="highs"
        )

        if not result.success:
            print("Warning: No optimal solution found")
            return [], float("inf")

        assignments = []
        total_cost = 0
        solution = result.x.reshape((n_robots, n_tasks))

        assigned_tasks = []

        for i in range(len(self._robots)):
            for j in range(len(tasks)):
                if (
                    solution[i, j] > 0.5
                    and cost_matrix[i, j] < self._default_infeasible_cost
                ):
                    assignments.append((self._robots[i].id, tasks[j].id))
                    total_cost += cost_matrix[i, j]

                    assigned_tasks.append(tasks[j])

        return assignments, total_cost, assigned_tasks


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

    allocator.add_robots(robots)

    tasks = [
        TaskDescription(
            id="ugv_map(region_3)", requirements=["ugv_map"], cost_per_robot={}
        ),
        TaskDescription(
            id="ugv_map(region_2)", requirements=["ugv_map", "husky"], cost_per_robot={}
        ),
        TaskDescription(
            id="ugv_map(region_1)", requirements=["ugv_map", "husky"], cost_per_robot={}
        ),
        TaskDescription(
            id="uav_explore(region_1)", requirements=["uav_map"], cost_per_robot={}
        ),
    ]

    assigments, cost = allocator.solve_assignment(tasks)

    print(assigments)
