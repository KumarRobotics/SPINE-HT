from openai import OpenAI
import json
import matplotlib.pyplot as plt

from pathlib import Path

from language_teaming.decomp.prompts import PROMPT
import networkx as nx


class MissionDecomp:
    EXPECTED_KEYS = ["reasoning", "tasks", "dependency_reasoning", "dependency_graph"]

    def __init__(self):
        self.client = OpenAI()
        self.model = "gpt-4.1"

    def _build_query(self, mission: str, scene_graph: str = ""):
        return [
            {"role": "system", "content": PROMPT},
            {
                "role": "user",
                "content": f"mission specification: {mission}, Scene graph: {scene_graph}",
            },
        ]

    def query_llm(self, msg):
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

            return top_msg_dict
        except:
            return {}

    def _build_graph(self, tasks, deps):
        graph = nx.DiGraph()
        graph.add_node("start")
        start_nodes = set()

        for task in tasks:
            graph.add_node(task)
            start_nodes.add(task)

        for source, target in deps:
            graph.add_edge(source, target)
            start_nodes.remove(target)

        for node in start_nodes:
            graph.add_edge("start", node)

        return graph

    def save_graph(self, graph):
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
        plt.close()  # Close the plot to free memory (especially important in loops)

    def get_mission_graph(self, mission_specification: str, scene_graph: str = ""):
        query = self._build_query(
            mission=mission_specification, scene_graph=scene_graph
        )
        output = self.query_llm(query)

        mission_graph = self._build_graph(output["tasks"], output["dependency_graph"])

        return mission_graph, output

    def print_output(self, output):
        for k, v in output.items():
            print(f"--- {k} ---")
            print(v)


if __name__ == "__main__":

    test_dir = Path(__file__).parents[3] / "tests"
    with open(test_dir / "data/graph_1.json") as f:
        graph = json.load(f)

    mission_decomp = MissionDecomp()
    graph, out = mission_decomp.get_mission_graph(
        "find the red house", scene_graph=str(graph)
    )
    mission_decomp.print_output(out)
    mission_decomp.save_graph(graph)

    debug = 0
