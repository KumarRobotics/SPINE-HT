import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend0
import matplotlib.pyplot as plt
import networkx as nx
from openai import OpenAI

from spine_multi.allocation.assignment import TaskDescription
from spine_multi.decomp.base_prompt import build_prompt
from spine_multi.decomp.util import _parse_function_call
from spine_multi.planner_logging import break_long_str


class GPT5Client:
    def __init__(self, msg_history: List[dict], expected_keys: List[str]):
        self.client = OpenAI()
        self.model = "gpt-5"  # "gpt-4.1"
        self._msg_history = msg_history
        self.EXPECTED_KEYS = expected_keys

    def query_llm(self, msg: List[Dict[str, str]]) -> Dict:
        try:
            msg[0]["role"] = "developer"
            # print(f"querying client with msg: {msg}")
            response = self.client.responses.create(
                model=self.model,
                input=msg,
                text={"format": {"type": "json_object"}, "verbosity": "medium"},
                reasoning={"effort": "medium"},
                tools=[],
                store=True,
                include=[
                    "reasoning.encrypted_content",
                    "web_search_call.action.sources",
                ],
            )
            top_msg = response.output[1].content[0].text
            # top_msg = response.choices[0].message.content
            top_msg_dict = json.loads(top_msg)

            for key in self.EXPECTED_KEYS:
                assert key in top_msg_dict.keys(), f"{key} not in graph"

            self._msg_history.append({"role": "assistant", "content": top_msg})

            return top_msg_dict
        except Exception as ex:
            print(f"got exception when querying llm: {ex}")
            return {}


class GPT4Client:
    def __init__(self, msg_history: List[dict], expected_keys: List[str]):
        self.client = OpenAI()
        self.model = "gpt-4.1"  # "gpt-4.1"
        self._msg_history = msg_history
        self.EXPECTED_KEYS = expected_keys

    def query_llm(self, msg: List[Dict[str, str]]) -> Dict:
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=msg,
                temperature=0.01,  # was 1
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


class MissionDecomp:
    EXPECTED_KEYS = ["reasoning", "tasks", "dependency_reasoning", "dependency_graph"]

    def __init__(self, team_specification: str, llm_type="gpt5"):
        self._msg_history = []
        if llm_type == "gpt5":
            self._llm_client = GPT5Client(
                msg_history=self._msg_history, expected_keys=self.EXPECTED_KEYS
            )
        elif llm_type == "gpt4":
            self._llm_client = GPT4Client(
                msg_history=self._msg_history, expected_keys=self.EXPECTED_KEYS
            )
        self._llm_type = llm_type

        self._mission_specification = ""
        self._scene_graph = ""
        self._init_location = ""
        self._base_prompt = build_prompt(team_specification=team_specification)

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
        sys_role = "system" if self._llm_type == "gpt4" else "developer"
        return [
            {"role": sys_role, "content": self._base_prompt},
            {
                "role": "user",
                "content": f"mission specification: {self._mission_specification}\n scene graph: {self._scene_graph}\n initial location: {self._init_location}",
            },
        ] + self._msg_history

    def _query_llm(self, msg: List[Dict[str, str]]) -> Dict:
        return self._llm_client.query_llm(msg)

    def _build_graph(self, tasks: List[str], deps) -> nx.DiGraph:
        graph = nx.DiGraph()
        graph.add_node("start")
        start_nodes = set()

        cleaned_deps = []
        for dep in deps:
            dep = [d for d in dep if d != ""]
            if len(dep) >= 2:
                cleaned_deps.append(dep)

        for task in tasks:
            graph.add_node(task)
            start_nodes.add(task)

        for source, target in cleaned_deps:
            graph.add_edge(source, target)
            start_nodes.discard(target)

        for node in start_nodes:
            graph.add_edge("start", node)

        return graph

    def save_graph(self, graph: nx.Graph, fpath) -> None:
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
        plt.savefig(fpath)  # You can use .png, .pdf, .svg, etc.
        plt.close()

    def get_mission_graph(self) -> Tuple[nx.DiGraph, Dict[str, str]]:
        query = self._build_query()
        output = self._query_llm(query)

        try:
            mission_graph = self._build_graph(
                output["tasks"], output["dependency_graph"]
            )

            self.save_graph(mission_graph, fpath=Path().home() / "data/graph.png")
        except KeyError as err:
            raise KeyError(f"got {err} with output: {output}")

        return mission_graph, output

    def get_mission_traces(self, graph: nx.DiGraph) -> List[List[TaskDescription]]:
        leaf_nodes = [n for n in graph.nodes if graph.out_degree(n) == 0]
        all_nodes = [n for n in graph.nodes]
        all_traces = []
        for leaf_node in leaf_nodes:
            try:
                mission_trace = list(
                    nx.all_simple_paths(graph, source="start", target=leaf_node)
                )
            except:
                raise ValueError(
                    f"Graph is ill-formed. Has leaf nodes: {leaf_node}: {graph}"
                )

            if not len(mission_trace):
                raise ValueError(f"No mission trace for: {mission_trace}")

            if len(mission_trace[0]) <= 1:
                raise ValueError(
                    f"mission trace must be longer than 1. Have trace: {mission_trace[0]}. Leafs: {leaf_nodes}. All nodes: {all_nodes}"
                )

            trace = self._parse_task_trace(mission_trace[0][1:])

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
