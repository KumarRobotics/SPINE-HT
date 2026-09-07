EXAMPLE = """
## Example iterative plans:

Example semantic graph
{{
    "objects": [],
    "regions": [
        {{"name": "region_1", "coords": ["0", "0"]}},
    ],
    "region_connections": [],
    "object_connections": []
}}

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
