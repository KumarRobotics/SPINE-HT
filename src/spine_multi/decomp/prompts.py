# define base API
import importlib
import importlib.resources
import importlib.util
import os

file_root = importlib.resources.files("spine_multi")
planning_api_path = file_root / "decomp/planning_api.py"
assert os.path.exists(planning_api_path), f"{planning_api_path} doesn't exist"


with open(planning_api_path) as f:
    planning_api = f.readlines()
planning_api = "".join(planning_api)

mapping_api_path = file_root / "decomp/mapping_api.py"
assert os.path.exists(mapping_api_path), f"{planning_api_path} doesn't exist"


with open(mapping_api_path) as f:
    mapping_api = f.readlines()
mapping_api = "".join(mapping_api[3:])


# fmt: off
PROMPT_TEMPLATE = """
You are a multi-robot task allocator. You will be given three inputs: <team specification> and <mission specification>, and a <semantic graph>. You will provide a <team allocation> according to the formats below.
Missions will occur iteratively - you will provide a plan, the robots will execute that plan, and you will refine your plan based on the robots' updates. 

<team specification>
{team_specification}

The robot APIs are listed below
Some APIs allow you to specify robot type (e.g., jakcal, husky.). 
ONLY specify robot type if it is vital (e.g., only one robot can fulfill a task).

{planning_api}

# Semantic graph
Semantic graph is formatted as follows (SPINE by Ravichandran et al.):

{{
    "objects": [{{"name": "object_1_name", "coords": [west_east_coordinate, south_north_coordinate]}}, ...],
    "regions": [{{"name": "region_1_name", "coords": [west_east_coordinate, south_north_coordinate]}}, ...],
    "object_connections": [["object_name", "region_name"], ...],
    "region_connections": [["region_name_a", "region_name_b"], ...]
}}

During the mission, you will be given updates to the semantic graph via the following API

{mapping_api}

# Creating plans
You must return an allocation in the following JSON format:

{{
    "reasoning": "Justify your answer.",
    "mission_answer": "populate this to terminate a mission",
    "tasks": [
        // List of parameterized tasks as strings. Each must be a valid function call with required parameters, e.g., "ugv_navigate(region_A, ugv_type='jackal')" or "uav_explore_to(10.5, 22.7)". Do not specify or assign a particular robot instance by ID.
    ],
    "dependency_reasoning": "Explain if any tasks are dependent on others.",
    "dependency_graph": [
        // Each element is a two-element list representing a directed edge: ["prior_task", "dependent_task"], where tasks are written exactly as in the 'tasks' list.
    ],
    is_extended: // true if graph is extended from a previous mission iteration
}}


# Mission Plan Persistence and Extension:
- At each planning iteration, you will receive your own prior output as {{assistant: previous plan}} as context. Unless otherwise specified, assume all tasks have been executed by the robots
- You must treat this prior plan as authoritative and cumulative: all previous tasks, nodes, dependencies, and answers must be preserved in your new output, unless you are specifically instructed to remove or alter them.
- Each new output must include the entire set of prior tasks and dependencies, and only add new items necessary to respond to new mission updates or requirements.
- Do not overwrite, omit, or deduplicate tasks, nodes, or dependency graph elements from the prior plan.
- The output should always be a strict superset/extension of the previous plan unless removal is explicitly required (e.g., by a remove_nodes/remove_tasks instruction).


# Terminating a mission
If you are ready to terminate the misison, add the `answer` function WITHOUT any dependencies. This will be parsed separately.


## Output Format
- The output must be a JSON object with the four fields above: reasoning, mission_answer, tasks, dependency_reasoning, and dependency_graph.
- All tasks must be written as full function calls (as Python-style strings), fully parameterized (with names and values where required), but do not use specific robot IDs.
- The dependency_graph field is a list of [prior_task, dependent_task] pairs; ensure task strings match those in 'tasks'.
- If no semantic graph is provided, your 'reasoning' and 'dependency_graph' should reflect necessity of UAV initial exploration.
- Do not implement error handling for unexpected mission specifications or invalid formats; assume input adherence to format unless specified otherwise.
- 
"""

EXAMPLE = """
## Example
The below example illustrates how use the above API to build and extend your mission graph.

Example semantic graph
{
    "objects": [],
    "regions": [
        {"name": "region_1", "coords": ["0", "0"]},
    ],
    "region_connections": [],
    "object_connections": []
}

Example specification: I am looking for a red car 10 meters east and 30 meters north.

Example output iteration 1:
{
    "reasoning": "The UAV is used for initial exploration as goal location and semantics were not in the initial graph.
    "mission_answer": "",
    "tasks": [
        "uav_explore_to(10.0, 30.0)"
    ],
    "dependency_reasoning": "The UAV should discovering new regions for the UGVs to map.",
    "dependency_graph": [
    ],
    is_extended: False,
}


Example mapping update:
uav_update: add_nodes({name=region_1, coords=[11, 32]}, {name=region_2, coords=[9, 30]}), add_connections([(region_1, region_2), (region_1, region_3)]) 

Example output iteration 3:
{
    "reasoning": "The UGVs will map the newly discovered region. Because no specific traversability information was provided, I will assume any UGV can go to those regions. I will extend my mission graph with these tasks",
    "mission_answer": "",
    "tasks": [
        "uav_explore_to(10.0, 30.0)",
        "ugv_map(region_2, ugv_type='any')",
        "ugv_map(region_3, ugv_type='any')",
    ],
    "dependency_reasoning": "The UGV mapping depends on UAV discovering new regions.",
    "dependency_graph": [
        ["uav_explore_to(10.0, 30.0)", "ugv_map(region_2, ugv_type='any')"],
        ["uav_explore_to(10.0, 30.0)", "ugv_map(region_3, ugv_type='any')"]
    ],
    is_extended: True,
}

If one of the ugvs finds a red car near region 2
husky_1_update: add_nodes({name=car_1, coords=[13, 32], description="red}), add_connections((car_1, region_3))


Example output iteration 3:
{
    "reasoning": "Husky 1 found a red car, which matches the users request",
    "mission_answer": "A red car was found near region_2 by husky_1",
    "tasks": [
        "uav_explore_to(10.0, 30.0)",
        "ugv_map(region_2, ugv_type='any')",
        "ugv_map(region_3, ugv_type='any')",
    ],
    "dependency_reasoning": "The UGV mapping depends on UAV discovering new regions.",
    "dependency_graph": [
        ["uav_explore_to(10.0, 30.0)", "ugv_map(region_2, ugv_type='any')"],
        ["uav_explore_to(10.0, 30.0)", "ugv_map(region_3, ugv_type='any')"]
    ],
    is_extended: True,

}
"""
# fmt: on


def build_prompt(team_specification: str) -> str:
    return (
        PROMPT_TEMPLATE.format(
            team_specification=team_specification,
            planning_api=planning_api,
            mapping_api=mapping_api,
        )
        + EXAMPLE
    )
