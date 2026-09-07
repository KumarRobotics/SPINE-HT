import json
from typing import Dict, List, Optional, Tuple

from openai import OpenAI

BASE_PROMPT = [
    {
        "role": "system",
        "content": [
            {
                "text": "Agent Role: You are a robot's frontier exploration algorithm. Given an incomplete graph representation of an environment, a history of actions, and the robot's current location, you will tell the robot where to explore next. The goal of exploration is to discover a new region node that the robot can use to navigate.\nProvide the following outputs in the following JSON schema, without extra formatting:\n{ target_location: where does the robot need to go to accomplish it's goal? Do not choose existing nodes in the graph. Rather, choose new regions that reference existing nodes. \ntarget_reason: why did you choose this target? \nexisting_paths: is there an existing path to this location? If so, provide the path as a list of nodes to visit. The path must begin with the robot's current location and end with the target location. And edge must exist between adjacent nodes in the path. You can ONLY reference nodes currently in the scene graph.\nexploration_needed: does this task require exploration of a new region, or can we use an existing region to navigate? If there is an existing connection to the goal, we do not need to explore. If you are unsure, then exploration is needed.\nplan_reason: explain the reason for your plan.\nexploration_target: If exploration is needed, provide the target location the robot should try to explore next in coordinates (x, y). x corresponds to west / east, and y corresponds to south / north. Output a list. \nabsolute_coordinates: True if the coordinates are absolute, false if they are relative to the current location.\n}",
                "type": "text",
            }
        ],
    },
]


def create_prompt(
    task: str, current_location: str, history: List[str], graph_as_json: str
) -> Dict[str, Dict[str, str]]:
    prompt = []
    prompt.extend(BASE_PROMPT)

    user_input = [
        {
            "role": "user",
            "content": [
                {
                    "text": f"Instruction: {task}\nScene Graph: {graph_as_json}\nCurrent location: {current_location}, history: {history}",
                    "type": "text",
                }
            ],
        }
    ]

    prompt.extend(user_input)
    return prompt


class FrontierLLM:
    def __init__(self, n_attempts: Optional[int] = 3) -> None:
        self.client = OpenAI()
        self.n_attempts = n_attempts

    def query(
        self, task: str, current_location: str, history: List[str], graph_as_json: str
    ) -> Tuple[bool, Dict[str, str]]:
        for _ in range(self.n_attempts):
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=create_prompt(
                    task=task,
                    current_location=current_location,
                    history=history,
                    graph_as_json=graph_as_json,
                ),
                temperature=1,
                max_tokens=1024,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0,
            )

            answer = response.choices[0].message.content

            answer = answer.strip("```").strip("json")

            try:
                formatted_answer = json.loads(answer, strict=False)
                return True, formatted_answer
            except Exception:
                pass
        return False, answer
