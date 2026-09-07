import os
from collections import defaultdict, namedtuple
from typing import Any, Dict, List

import numpy as np
import pytest
import yaml

from spine_ht.collaborator import (
    Collaborator,
    RobotConfig,
    RobotDescription,
    get_capability_set,
)
from spine_ht.planner_logging import get_logger
from spine_ht.spine.mapping.graph_util import GraphHandler

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

TestParam = namedtuple("TestParam", ["value"])
TestConfig = namedtuple(
    "TestConfig", ["robot_configs", "init_graph", "team_specification", "init_location"]
)


def parse_robot_config(param_dict: Dict[str, Any]) -> List[RobotConfig]:
    tmp_robot_config = defaultdict(dict)

    for param_name, param_obj in param_dict.items():
        robot_name, param_name = param_name.split(".")

        tmp_robot_config[robot_name][param_name] = param_obj.value

    robot_configs = []
    for robot, config in tmp_robot_config.items():
        robot_configs.append(RobotConfig(**config))

    return robot_configs


def parse_yaml(yaml_file: str) -> TestConfig:
    with open(yaml_file) as f:
        as_dict = yaml.safe_load(f)

    config_dict = as_dict["spine_ht_node"]["ros__parameters"]

    # replicate ROS2 format
    formatted_robot_dict = {}
    for robot, params in config_dict["robots"].items():
        for k, v in params.items():
            formatted_robot_dict[f"{robot}.{k}"] = TestParam(value=v)

    robot_config = parse_robot_config(formatted_robot_dict)

    return TestConfig(
        robot_configs=robot_config,
        init_graph=config_dict["init_graph"],
        team_specification=config_dict["team_specification"],
        init_location=config_dict["init_location"],
    )


def init_collaborator(
    config_path: str, graph_path: str, team_specification="", llm_type="gpt4"
) -> Collaborator:
    spine_mult_config = parse_yaml(config_path)

    robot_configs = spine_mult_config.robot_configs

    init_location = spine_mult_config.init_location

    if team_specification == "":
        team_specification = spine_mult_config.team_specification

    robots = []

    for robot_config in robot_configs:
        robots.append(
            RobotDescription(
                id=robot_config.name,
                type=robot_config.type,
                capabilities=get_capability_set(robot_config.name, robot_config.type),
                location=np.array([]),
            )
        )

    graph = GraphHandler(graph_path=graph_path)

    logger = get_logger("spine_test", output="console")
    collaborator = Collaborator(
        team_specification=robots,
        semantic_graph=graph,
        team_spec_language=team_specification,
        init_location=init_location,
        logger=logger,
        llm_type=llm_type,
    )

    return collaborator


def init_and_get_result(
    config_path, graph_path, specification, team_specification, llm
):
    collaborator = init_collaborator(
        config_path=config_path,
        graph_path=graph_path,
        team_specification=team_specification,
        llm_type=llm,
    )
    collaborator.init_planner(specification)
    return collaborator


LLM = "gpt4"


@pytest.mark.parametrize(
    "config_path,graph_path,specification,team_specification,llm",
    [
        # fmt: off
        # (
        #     SCRIPT_DIR + "/data/configs/spine_ht_single.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/tests/data/maps/pennov_back_construction.json",
        #     "inspect construction area 1",
        #     "",
        #     LLM
        # ),
        # (
        #     SCRIPT_DIR + "/data/configs/spine_ht.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/tests/data/maps/pennov_back_construction.json",
        #     "Verify the workzones are clear",
        #     "",
        #     LLM
        # ),
        # (
        #     SCRIPT_DIR + "/data/configs/spine_ht.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/tests/data/maps/pennov_back_parking.json",
        #     "Lead our visitors to their cars. They are parked in different areas.",
        #     "",
        #     LLM
        # ),
        # (
        #     SCRIPT_DIR + "/data/configs/spine_ht.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/tests/data/maps/pennov_back_delivery.json",
        #     "Pickup the delivery and watch for oncoming traffic",
        #     "You have two clearpath jacakls. jackal_2  has a larger payload capacity.",
        #     LLM
        # ),
        # (
        #     SCRIPT_DIR + "/data/configs/spine_ht.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/tests/data/maps/pennov_back_delivery.json",
        #     "I need status on the northmost part of the scene, but that is likely out of communciation range.",
        #     "You have two clearpath jacakls. jackal_2 has good communication relay radios",
        #     LLM
        # ),
        # (
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/config/spine_configs/spine_ht_spot_jackal.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/data/maps/pennov_back_delivery.json",
        #     "Pickup the delivery and watch for oncoming traffic",
        #     "You have two robots: one boston dyanmics spot and one clearpath jackal. The spot is fast and can explore. Jackal is slower but can map and monitor.",
        #     LLM
        # ),
        # ( # TODO look at this
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/config/spine_configs/spine_ht_spot_two_jackals.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/data/maps/pennov_back_delivery.json",
        #     "Pickup the delivery and watch for oncoming traffic. Ensure network connectivity",
        #     "You have three robots: one boston dyanmics spot and two clearpath jackals. The spot is fast and can explore. Jackal 1 has a strong radio.",
        #     LLM
        # ),
        # ( # TODO look at this
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/config/spine_configs/spine_ht_spot_husky_jackal.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/data/maps/pennov_back_triage.json",
        #     "There was a storm. Inspect logistics infrastructure.",
        #     "You have three robots: a boston dyanmics spot, a clearpath husky, and a clearpath jackal. The spot is fast and can explore. The husky can navigate over rugged terrain",
        #     LLM
        # ),
        # ( # TODO look at this
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/config/spine_configs/spine_ht_spot_husky_jackal.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/data/maps/pennov_back_triage.json",
        #     "There was a storm. First, ensure network connectivity throughout the scene by positioning a communication node. Once complete, inspect logistics infrastructure with other robots. ",
        #     "You have three robots: a boston dyanmics spot, a clearpath husky, and a clearpath jackal. The spot is fast and can explore. The husky can navigate over rugged terrain. Jackal_1 has a good communcation radio",
        #     LLM
        # ),
        # ( # TODO look at this
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/config/spine_configs/spine_ht_spot_husky_jackal.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/data/maps/pennov_back_triage.json",
        #     "There was a storm. First, ensure network connectivity by positioning a central communication node. Once complete, inspect logistics infrastructure with other robots. After initial results, robots should offload data",
        #     "You have three robots: a boston dyanmics spot, a clearpath husky, and a clearpath jackal. The spot is fast and can explore. The husky can navigate over rugged terrain. Jackal_1 has a good communcation radio",
        #     LLM
        # ),
        # ( # TODO look at this
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/config/spine_configs/spine_ht_spot_husky.yaml",
        #     "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/data/maps/pennov_back_triage.json",
        #     "There was a storm. Is key infrastructure damaged?.",
        #     "You have one boston dynamics spot and one clearpath husky. The Spot is fast and the husky can traverse rugged terrain.",
        #     LLM
        # ),
        (  # TODO look at this
            "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/config/spine_configs/spine_ht_spot_husky.yaml",
            "/home/zacravi/projects/dcist/src/spine-ht/ros/spine_ht_ros/data/maps/pennov_back_triage_partial.json",
            "There was a storm. Is key infrastructure damaged?.",
            "You have one boston dynamics spot and one clearpath husky. The Spot is fast and the husky can traverse rugged terrain.",
            LLM
        ),
        # fmt: on
    ],
)
def test_goto(config_path, graph_path, specification, team_specification, llm):
    """Test add function with multiple parameter sets."""
    collaborator = init_and_get_result(
        config_path, graph_path, specification, team_specification, llm
    )
    allocation_result = collaborator.get_allocation()
    result_str = collaborator.get_result_str(allocation_result)
    print(result_str)


if __name__ == "__main__":
    inputs = (
        SCRIPT_DIR + "/data/configs/spine_ht_single.yaml",
        "/home/zacravi/projects/dcist/src/spine-ht/tests/data/maps/pennov_back_construction.json",
        "inspect construction area 1",
        "",
    )

    collaborator = init_and_get_result(*inputs)
    allocation_result = collaborator.get_allocation()
    result_str = collaborator.get_result_str(allocation_result)
    print(result_str)
