import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")  # Use non-interactive backend0
import matplotlib.pyplot as plt
import networkx as nx
from openai import OpenAI

from spine_multi.allocation.assignment import TaskDescription
from spine_multi.decomp.base_prompt import build_prompt
from spine_multi.decomp.util import _parse_function_call
from spine_multi.planner_logging import break_long_str

import tiktoken

SEED = 10


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


        self._tokenizer = tiktoken.get_encoding("o200k_base")
        self._total_tokens = 0
        self._home_dir = Path().home() / "spine_token_log.txt"

    def query_llm(self, msg: List[Dict[str, str]]) -> Dict:
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=msg,
                temperature=0.0,
                max_tokens=2048,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0,
                response_format={"type": "json_object"},
                seed=SEED,
            )
            top_msg = response.choices[0].message.content
            top_msg_dict = json.loads(top_msg)

            for key in self.EXPECTED_KEYS:
                assert key in top_msg_dict.keys(), f"{key} not in graph"


            self._total_tokens += len(self._tokenizer.encode(str(msg)))
            self._total_tokens += len(self._tokenizer.encode(str(top_msg)))

            with open(str(self._home_dir), "a") as f:
                f.write(f"total tokens for spine: {self._total_tokens}\n")



            self._msg_history.append({"role": "assistant", "content": top_msg})

            return top_msg_dict
        except Exception as ex:
            print(ex)
            return {}


class MissionDecomp:
    EXPECTED_KEYS = [
        "reasoning",
        "tasks",
        "dependency_reasoning",
        "dependency_graph",
        "mission_answer",
    ]

    def __init__(self, team_specification: str, llm_type="gpt4", logger = None):
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
        self._base_prompt = build_prompt(
            team_specification=team_specification, llm=self._llm_type
        )

        self._logger = logger


    def _log_info(self, msg) -> None:
        if self._logger != None:
            self._logger.info(f"[llm.py] {msg}")

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


    # def _build_graph(self, tasks: List[str], deps) -> nx.DiGraph:
    #     graph = nx.DiGraph()
    #     graph.add_node("start")
    #     start_nodes = set()

    #     cleaned_deps = []
    #     for dep in deps:
    #         dep = [d for d in dep if d != ""]
    #         if len(dep) >= 2:
    #             cleaned_deps.append(dep)

    #     for task in tasks:
    #         graph.add_node(task)
    #         start_nodes.add(task)

    #     for source, target in cleaned_deps:
    #         graph.add_edge(source, target)
    #         start_nodes.discard(target)

    #     for node in start_nodes:
    #         graph.add_edge("start", node)

    #     return graph

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
            graph.add_node(source)
            graph.add_node(target)
            start_nodes.add(source)

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
        log_output = f"leaf nodes: {leaf_nodes}. all nodes: {all_nodes}"
        for leaf_node in leaf_nodes:
            self._log_info(f"Attemping to build trace from leaf: {leaf_node}")
            try:
                mission_trace = list(
                    nx.all_simple_paths(graph, source="start", target=leaf_node)
                )

                log_output += f"\nGot trace: {mission_trace} of len {len(mission_trace)} for leaf: {leaf_node}"
            except Exception as ex:
                # TODO log warning
                self._log_info(f"[parse trace] [EXCEPTION] {ex}")
                return []
                # raise ValueError(
                #     f"Graph is ill-formed. Has leaf nodes: {leaf_node}: {graph}"
                # )

            if len(mission_trace) == 0 or len(mission_trace[0]) <= 1:
                # TODO add warning
                # raise ValueError(
                #     f"mission trace must be longer than 1. Have trace: {mission_trace[0]}. Leafs: {leaf_nodes}. All nodes: {all_nodes}"
                # )
                self._logger.info(f"skipping trace: {mission_trace}")


                pass
            else:
                trace = self._parse_task_trace(mission_trace[0][1:])
                all_traces.append(trace)

        if self._logger != None:
            self._logger.info(f"have mission traces: {all_traces}")

        return all_traces, log_output

    def give_updates(self, updates: str) -> None:
        self._msg_history.append({"role": "user", "content": updates})

    def _get_function_requirements(self, function_name: str) -> List[str]:
        return [function_name]

    def _parse_task_requirements(
        self, function_name: str, task_args: List[str], task_kwargs: Dict[str, Any]
    ) -> List[str]:

        requirements = self._get_function_requirements(function_name)
        if function_name.startswith("ugv") or function_name.startswith("explore") and "ugv_type" in task_kwargs:
            if task_kwargs["ugv_type"] != "any":
                requirements.append(task_kwargs["ugv_type"])

        return requirements

    def _parse_task_trace(self, trace: List[str]) -> List[TaskDescription]:
        try:
            parsed_trace = []
            for task in trace:
                name, args, kwargs = _parse_function_call(task)
                task_requirements = self._parse_task_requirements(name, args, kwargs)
                self._log_info(f"task: {task}. reqs: {task_requirements}")
                parsed_trace.append(
                    TaskDescription(
                        id=task, requirements=task_requirements, args=args, kwargs=kwargs
                    )
                )
        except Exception as ex:
            raise ValueError(f"[parse trace] Got ex: {ex} with input: {trace}")

        return parsed_trace

    def output_log(self, output: Dict[str, Any]) -> str:
        log = ""
        for k, v in output.items():
            log += f"--- {k} ---\n"
            log += f"\t{break_long_str(str(v))}\n\n"

        return log


if __name__ == "__main__":

    # def build_graph(tasks, deps):
    #     graph = nx.DiGraph()
    #     graph.add_node("start")
    #     start_nodes = set()

    #     cleaned_deps = []
    #     for dep in deps:
    #         dep = [d for d in dep if d != ""]
    #         if len(dep) >= 2:
    #             cleaned_deps.append(dep)

    #     for task in tasks:
    #         graph.add_node(task)
    #         start_nodes.add(task)

    #     for source, target in cleaned_deps:
    #         graph.add_edge(source, target)
    #         start_nodes.add(source)
    #         start_nodes.discard(target)

    #     for node in start_nodes:
    #         graph.add_edge("start", node)

    #     return graph



    def build_graph(tasks: List[str], deps) -> nx.DiGraph:
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
            graph.add_node(source)
            graph.add_node(target)
            start_nodes.add(source)

        for source, target in cleaned_deps:
            graph.add_edge(source, target)
            start_nodes.discard(target)

        for node in start_nodes:
            graph.add_edge("start", node)

        return graph

    deps = [
        ["uav_map(region_node='ground_9')", "ugv_map_region(region_node='road_1_from_uav', ugv_type='spot')"], 
        ["uav_map(region_node='ground_9')", "ugv_map_region(region_node='road_2_from_uav', ugv_type='spot')"],
        [
            "ugv_map_region(region_node='region_1', ugv_type='spot')",
            "ugv_inspect(object_node='discovered_cone_0', query='Describe the cone', ugv_type='spot')",
        ]
    ]

    tasks = ["ugv_map_region(region_node='road_1_from_uav', ugv_type='spot')", "ugv_map_region(region_node='road_2_from_uav', ugv_type='spot')"]

    # tasks = ["ugv_inspect(object_node='discovered_cone_0', query='Describe the cone', ugv_type='spot')", "ugv_inspect(object_node='discovered_cone_0', query='Describe the cone', ugv_type='spot')"]

    build_graph(tasks, deps)
