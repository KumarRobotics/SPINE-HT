import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt
import networkx as nx
from openai import OpenAI
from spine_multi.allocation.assignment import TaskDescription
from spine_multi.decomp.prompts import build_prompt
from spine_multi.decomp.util import _parse_function_call
from spine_multi.planner_logging import break_long_str


class MissionDecomp:
    EXPECTED_KEYS = ["reasoning", "tasks", "dependency_reasoning", "dependency_graph"]

    def __init__(self, team_specification: str):
        self.client = OpenAI()
        self.model = "gpt-4.1"

        self._mission_specification = ""
        self._scene_graph = ""
        self._init_location = ""
        self._base_prompt = build_prompt(team_specification=team_specification)

        self._msg_history = []

    def get_mission_specification(self) -> str:
        return self._mission_specification

    def set_specifications(
        self, mission: str, scene_graph: str = "", init_location: str = ""
    ) -> None:
        self._mission_specification = mission
        self._scene_graph = scene_graph
        self._init_location = init_location

    def _build_query(self) -> List[Dict[str, str]]:
        assert self._mission_specification != "", f"must set mission specification"
        return [
            {"role": "system", "content": self._base_prompt},
            {
                "role": "user",
                "content": f"mission specification: {self._mission_specification}\n scene graph: {self._scene_graph}\n initial location: {self._init_location}",
            },
        ] + self._msg_history

    def _query_llm(self, msg: List[Dict[str, str]]) -> Dict:
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=msg,
                temperature=0.05,  # was 1
                max_tokens=2048,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0,
                response_format={"type": "json_object"},
            )
            top_msg = response.choices[0].message.content
            top_msg_dict = json.loads(top_msg)

            for key in self.EXPECTED_KEYS:
                assert key in top_msg_dict.keys(), f"{key} not in graph"

            self._msg_history.append({"role": "assistant", "content": top_msg})

            return top_msg_dict
        except:
            return {}

    def _build_graph(self, tasks: List[str], deps) -> nx.DiGraph:
        graph = nx.DiGraph()
        graph.add_node("start")
        start_nodes = set()

        for task in tasks:
            graph.add_node(task)
            start_nodes.add(task)

        for source, target in deps:
            graph.add_edge(source, target)
            start_nodes.discard(target)

        for node in start_nodes:
            graph.add_edge("start", node)

        return graph

    def save_graph(self, graph: nx.Graph) -> None:
        plt.figure(figsize=(8, 6))
        nx.draw(
            graph,
            with_labels=True,
            node_color="skyblue",
            edge_color="gray",
            node_size=500,
            font_size=10,
        )

        # Save the plot to a file
        plt.savefig("graph_plot.png")  # You can use .png, .pdf, .svg, etc.
        plt.close()

    def get_mission_graph(self) -> Tuple[nx.DiGraph, Dict[str, str]]:
        query = self._build_query()
        output = self._query_llm(query)

        try:
            mission_graph = self._build_graph(
                output["tasks"], output["dependency_graph"]
            )
        except KeyError as err:
            raise KeyError(f"got {err} with output: {output}")

        return mission_graph, output

    def get_mission_traces(self, graph: nx.DiGraph) -> List[List[TaskDescription]]:
        leaf_nodes = [n for n in graph.nodes if graph.out_degree(n) == 0]
        all_traces = []
        for leaf_node in leaf_nodes:
            trace = self._parse_task_trace(
                list(nx.all_simple_paths(graph, source="start", target=leaf_node))[0][
                    1:
                ]
            )

            all_traces.append(trace)

        return all_traces

    def give_updates(self, updates: str) -> None:
        self._msg_history.append({"role": "user", "content": updates})

    def _get_function_requirements(self, function_name: str) -> List[str]:
        return [function_name]

    def _parse_task_requirements(
        self, function_name: str, task_args: List[str], task_kwargs: Dict[str, Any]
    ) -> List[str]:

        requirements = self._get_function_requirements(function_name)
        if function_name.startswith("ugv") and "ugv_type" in task_kwargs:
            if task_kwargs["ugv_type"] != "any":
                requirements.append(task_kwargs["ugv_type"])

        return requirements

    def _parse_task_trace(self, trace: List[str]) -> List[TaskDescription]:
        parsed_trace = []
        for task in trace:
            name, args, kwargs = _parse_function_call(task)
            task_requirements = self._parse_task_requirements(name, args, kwargs)
            parsed_trace.append(
                TaskDescription(
                    id=task, requirements=task_requirements, args=args, kwargs=kwargs
                )
            )

        return parsed_trace

    def output_log(self, output: Dict[str, Any]) -> str:
        log = ""
        for k, v in output.items():
            log += f"--- {k} ---\n"
            log += f"\t{break_long_str(str(v))}\n\n"

        return log


if __name__ == "__main__":

    test_dir = Path(
        "/home/zacravi/projects/dcist/src/spine-multi/ros/spine_multi_ros/data"
    )
    with open(test_dir / "graph_1.json") as f:
        graph = json.load(f)

    mission_decomp = MissionDecomp()
    graph, out = mission_decomp.get_mission_graph(
        "find the red house", scene_graph=str(graph)
    )
    mission_decomp.output_log(out)
    mission_decomp.save_graph(graph)

    debug = 0
