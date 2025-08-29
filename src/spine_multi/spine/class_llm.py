import json
from typing import Dict, List, Optional, Tuple

from openai import OpenAI

BASE_SYSTEM_INSTRUCTIONS = """"
Role: Configure the vision system for an advanced graph-based planner operating on a small mobile robot. The planner receives a user task and an incomplete environment graph. Your job is to configure vision to optimally support task completion.

Begin with a concise checklist (3-7 bullets) of what you will do; keep items conceptual, not implementation-level.

Checklist:
- Parse the user-provided task and, if available, environment description.
- Identify every specific object mentioned in the task.
- Determine a concise list of object classes essential for fulfilling the task, considering task type (e.g., path finding, mapping, inspection).
- Exclude non-object or spatial concepts (e.g., 'office', 'road') from the class list.
- Tailor class length to the complexity and nature of the task.
- Prepare a brief summary of the task and a concise explanation for class selection.
- Validate the final output against all requirements before returning.

Instructions:
- The vision system performs open vocabulary object detection. For each user task, select only object classes required to complete it, ensuring all task-specified objects are included. Keep the list specific and relevant to the robot's camera perspective.
- Output must be a valid JSON string (parsable by Python's `json.loads`) with the following structure:

# Output Format
{
"classes": [<object class 1>, <object class 2>, ...],
"task_summary": "<main goal, e.g., mapping, path finding>",
"reason": "<concise explanation of your class selection>"
}

Guidelines:
- Strictly include only object classes, not spatial or abstract terms.
- All objects directly mentioned in the task must be present in the class list.
- For path finding, keep the class list ≤5; for mapping or inspection, include more (up to ~10) but avoid over-inclusion to prevent false positives.
- Limit to objects a mobile robot could realistically detect from its viewpoint.
- Incorporate relevant environment details only as needed for specificity.

Requirements:
- Do not include spatial concepts (e.g., 'road', 'field').
- All directly referenced objects in the user task must be present.
- Be specific: use precise object names rather than ambiguous categories.
- Before outputting, double-check that only valid object classes are listed, that all task-referenced objects are included, and that class count aligns with task type.
- DO NOT use spaces, use underscores instead. For example 'black car' should be 'black_car'

After preparing your output, validate that it complies with all output formatting and checklist requirements; self-correct if any requirement is missed.

Example:
User: "I left my keys on the couch."
Output:
{
"classes": ["keys", "couch"],
"task_summary": "object search",
"reason": "Both referenced objects are required for effective detection."
}
"""


BASE_PROMPT = [
    {
        "role": "system",
        "content": [
            {
                "text": BASE_SYSTEM_INSTRUCTIONS,
                "type": "text",
            }
        ],
    }
]

EXAMPLES = [
    {
        "role": "user",
        "content": [
            {
                "text": "What is in the outdoor scene?",
                "type": "text",
            }
        ],
    },
    {
        "role": "assistant",
        "content": [
            {
                "type": "text",
                "text": '{"task_summary": "the problem calls for a general description of an environment, so I will list many classes.",\
                "reason": "the vision system must detect various common outdoor objects to infer and fill in the incomplete areas. There are many types of areas that could be in a scene, including a park, a construction site, a forest, etc. I will include a wide array of objects that could be found in these areas."\n    \
                "classes": ["bike", "chair", "table", "tree", "building", "car", "road", "person", "park", "playground", "bench", "streetlight", "bus", "truck", "motorcycle", "crosswalk", "sidewalk", "river", "bridge", "food", "stop sign"],\n}',
            }
        ],
    },
    {
        "role": "user",
        "content": [
            {
                "text": "I was doing some groceries and I lost my bike. What happened?",
                "type": "text",
            }
        ],
    },
    {
        "role": "assistant",
        "content": [
            {
                "type": "text",
                "text": '{\n \
                "task_summary": "The problem implies that we need to find the users bike. Most importantly, I will list that as a class. I will also list objects where a bike might be found.", \
                "reason": "I need to find a bike and objects related to a bike", \
                "classes": ["bike", "bike rack", "stop sign", "pedestrian", "wheel"]',
            }
        ],
    },
    {
        "role": "user",
        "content": [
            {
                "text": "What happened to my phone?",
                "type": "text",
            }
        ],
    },
    {
        "role": "assistant",
        "content": [
            {
                "type": "text",
                "text": '{\n \
                "task_summary": "The problem implies that we need to find the users phone. Most importantly, I will list that as a class. I will also list objects where a phone might be found.", \
                "reason": "I need to find a bike and objects related to a bike", \
                "classes": ["phone", "desk", "charger", "chair", "table]',
            }
        ],
    },
]


def create_prompt(
    task: str, location_description: str = ""
) -> Dict[str, Dict[str, str]]:
    prompt = []
    prompt.extend(BASE_PROMPT)
    # prompt.extend(EXAMPLES)

    task_prompt = task

    if location_description != "":
        task_prompt += f" {location_description}"

    user_input = [
        {
            "role": "user",
            "content": [
                {
                    "text": task_prompt,
                    "type": "text",
                }
            ],
        }
    ]

    prompt.extend(user_input)
    return prompt


class ClassLLM:
    DEFAULT_RESPONSE = {
        "classes": ["car", "truck", "robot"],
        "reason": "Network unavailable for GPT call. Returning default classes",
        "task_summary": "Unknown. Returning default classes",
    }
    EXCLUDE_CLASSES = [
        "ground",
        "water",
        "road",
        "grass",
        "water body",
        "sand",
        "dock",
        "building",
        "park",
    ]

    def filter_classes(self, class_list: List[str]) -> List[str]:
        filtered_class_list = []
        for class_label in class_list:
            if class_label not in self.EXCLUDE_CLASSES:
                filtered_class_list.append(class_label)

        return filtered_class_list

    def __init__(self, n_attempts: Optional[int] = 3) -> None:
        self.client = OpenAI()
        self.model = "gpt-4.1"
        self.n_attempts = n_attempts

    def scene_description_from_nodes(self, nodes: List[str]) -> str:
        prompt = f"The robot is outside, and it has a scene graph representing its environment. These are the nodes in the graph: {nodes}"
        return prompt

    def try_query(self, task: str, location_description: str):
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=create_prompt(
                    task=task, location_description=location_description
                ),
                temperature=1,
                max_tokens=1024,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0,
            )
            return response, True
        except Exception as ex:
            return "", False

    def request(
        self, task: str, location_description: str = ""
    ) -> tuple[bool, dict[str, str]]:
        formatted_answer = {"classes": [], "reason": [], "task_summary": []}
        success = False

        for _ in range(self.n_attempts):
            response, success = self.try_query(
                task=task, location_description=location_description
            )

            # unclear if we should return defaults here
            if not success:
                return True, self.DEFAULT_RESPONSE

            answer = response.choices[0].message.content

            answer = answer.strip("```").strip("json")

            try:
                llm_answer = json.loads(answer, strict=False)
                success = True
            except Exception as ex:
                return False, ""

            if "classes" in llm_answer:
                llm_classes = self.filter_classes(llm_answer["classes"])
                formatted_answer["classes"].extend(llm_classes)
            if "reason" in llm_answer:
                formatted_answer["reason"].append(llm_answer["reason"])
            if "task_summary" in llm_answer:
                formatted_answer["task_summary"].append(llm_answer["task_summary"])

            return success, formatted_answer

        return False, answer
