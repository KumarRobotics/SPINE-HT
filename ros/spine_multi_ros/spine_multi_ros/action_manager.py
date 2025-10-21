from logging import Logger
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
from rclpy.node import Node
from spine_multi.multi_robot_graph import MultiRobotGraphHandler
from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.mapping.frontiers import Node as GraphNode
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent

from spine_multi_ros.msg_handler import BehaviorRequestData, MessageHandler


class ActionManager:
    open_scene_prompt = (
        "You are a robot. Describe where you are so you can plan. "
        "Provide your answer as a noun with a short description. For example: empty sidewalk, road, park with trees and benches, empty parking lot, patio."
    )

    def __init__(
        self,
        parent_node: Node,
        robot_name: str,
        msg_handler: MessageHandler,
        graph: GraphHandler | MultiRobotGraphHandler,
        graph_viz: GraphVisualizerComponent,
        prompt_former: UpdatePromptFormer,
        frontier_extractor: FrontierExtractor,
        logger: Logger,
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name
        self._msg_handler = msg_handler
        self._graph = graph
        self._frontier_extractor = frontier_extractor
        self._graph_viz = graph_viz
        self._prompt_former = prompt_former
        self._logger = logger

    def get_location_pos(self) -> np.ndarray:
        location_in_graph = self._graph.get_current_location()
        return self._graph.get_node_coord(location_in_graph)

    def construct_behavior_library(self) -> Dict[str, Callable]:
        return {
            "explore_to_coord": self._explore_to,
            "goto": self._goto_region,
            "inspect": self._inspect_object,
            "explore_region": lambda x: self._goto_region(x[0]),
            "map_region": self._map_region,
            "set_labels": self._set_labels,
            "replan": lambda x: True,
            "clarify": lambda x: True,
            "answer": lambda x: True,
        }

    def construct_msg_building_library(self) -> Dict[str, Callable]:
        return {
            "explore_to_coord": self._build_explore_to_request,
            "goto": self._build_goto_region_request,
            "inspect": self._build_inspect_object_request,
            "explore_region": lambda x: self._build_goto_region_request(x[0]),
            "map_region": self._build_map_region_requests,
            "set_labels": self._build_set_label_request,
            "explore_to_node": self._build_attempt_navigate_requests,
            "uav_map_region": self._build_uav_map_request,
            "uav_explore_to": self._build_uav_explore_to_request,
            "replan": lambda x: True,
            "clarify": lambda x: True,
            "answer": lambda x: True,
        }

    def construct_msg_parsing_library(self) -> Dict[str, Callable]:
        return {
            "explore_to_coord": self._parse_explore_to_results,
            "goto": self._parse_goto_region_results,
            "inspect": self._parse_inspect_object_results,
            "explore_region": self._parse_goto_region_results,
            "map_region": self._parse_map_region_results,
            "set_labels": self._parse_set_label_results,
            "explore_to_node": self._parse_attempt_navigate_results,
            "uav_map_region": self._parse_uav_map_request,
            "uav_explore_to": self._parse_uav_explore_to_request,
            "replan": lambda x: True,
            "clarify": lambda x: True,
            "answer": lambda x: True,
        }

    def _log_info(self, msg):
        log_msg = f"[action manager] [{self._robot_name}] {msg}"

        self._parent_node.get_logger().info(log_msg)
        # self._logger.info(log_msg)

    def _build_set_label_request(
        self, label_list: str
    ) -> Tuple[BehaviorRequestData, dict]:
        label_list = label_list.replace(",", "|")
        return self._msg_handler.build_label_msg_dict(label_list), {}

    def _set_labels(self, label_list: str) -> Tuple[bool, str]:
        msg = self._msg_handler.get_label_msg(label_list=label_list)
        success = self._msg_handler.send_and_wait(msg)

        # success = self._query_dict[self._current_query_idx].success
        self._parent_node.get_logger().info(
            f"[label manager topic] [{self._robot_name}] got answer for idx {self._msg_handler._behavior_idx}: {success}"
        )
        return True

    def _goto_region(self, region_id: str) -> bool:
        requests, metadata = self._build_goto_region_request(goal_region=region_id)
        request_msgs = self._msg_handler.build_behavior_msg(requests)
        self._log_info(f"Sending goto region request: {request_msgs}")
        results = self._msg_handler.send_and_wait(request_msgs)
        self._log_info(f"Got goto region results: {results}")
        return self._parse_goto_region_results(results=results, metadata=metadata)

    def _map_region(self, region_node: str) -> bool:
        # build nav requests
        behavior_request_dicts, map_region_metadata = self._build_map_region_requests(
            region_node=region_node
        )
        behavior_request_msg = self._msg_handler.build_behavior_msg(
            behavior_request_dicts
        )
        self._log_info(f"build map region requests: {behavior_request_msg}")
        results = self._msg_handler.send_and_wait(behavior_request_msg)
        self._log_info(f"map region got results: {results}")
        return self._parse_map_region_results(
            results=results, metadata=map_region_metadata
        )

    def _inspect_object(self, node_name: str, vlm_query: str) -> bool:
        request_dict, metadata = self._build_inspect_object_request(
            target_object=node_name, vlm_query=vlm_query
        )
        requests = self._msg_handler.build_behavior_msg(request_dict)
        self._log_info(f"built inspect region requests: {requests}")
        results = self._msg_handler.send_and_wait(requests)
        self._log_info(f"got inspect object results: {results}")
        return self._parse_inspect_object_results(results=results, metadata=metadata)

    def _build_map_region_requests(
        self, region_node: str
    ) -> Tuple[BehaviorRequestData, dict]:
        nav_request, metadata = self._build_goto_region_request(goal_region=region_node)
        vlm_request = self._msg_handler.build_vlm_query_dict(self.open_scene_prompt)

        region_coords, _ = self._graph.get_node_coords(region_node)
        map_request, mapping_metadata = self._build_map_request(region_coords)
        request_sequence = nav_request + vlm_request + map_request
        metadata.update(mapping_metadata)
        return request_sequence, metadata

    def _build_uav_map_request(
        self, region_id: str
    ) -> Tuple[List[BehaviorRequestData], dict]:
        return [], {}

    def _parse_uav_map_request(self, results: List[dict], metadata: dict) -> bool:
        return True

    def _build_uav_explore_to_request(
        self, x: float, y: float
    ) -> Tuple[List[BehaviorRequestData], dict]:
        return [], {}

    def _parse_uav_explore_to_request(
        self, results: List[dict], metadata: dict
    ) -> bool:
        self._prompt_former.update(
            freeform_updates="The UAV has been tasked and will send map updates when available"
        )
        return True

    def _build_explore_to_request(self, x: float, y: float):
        explore_request = self._msg_handler.build_navigate_msg_dict(x, y, 0, False)
        map_request, map_metadata = self._build_map_request(np.array([x, y]))
        metadata = {
            "begin_exploration_from": self._graph.get_current_location(),
            "target_explore_x": x,
            "target_explore_y": y,
        }
        metadata.update(map_metadata)

        vlm_request = self._msg_handler.build_vlm_query_dict(self.open_scene_prompt)

        return [explore_request] + map_request + vlm_request, metadata

    def _build_attempt_navigate_requests(
        self,
        goal_node: str,
    ):
        closest_node, _ = self._graph.get_closest_reachable_node(goal_node)
        nav_request, metadata = self._build_goto_region_request(
            goal_region=closest_node
        )
        goal_coords = self._graph.get_node_coord(goal_node)
        explore_msg = self._msg_handler.build_navigate_msg_dict(
            goal_coords[0], goal_coords[1], 0, False
        )
        metadata["exploration_node_target"] = goal_node

        goal_coord, _ = self._graph.get_node_coords(goal_node)
        map_request, map_metadata = self._build_map_request(goal_coord)
        metadata["goal_region"] = goal_node

        metadata.update(map_metadata)

        return nav_request + [explore_msg] + map_request, metadata

    def _build_map_request(self, coords: np.ndarray):
        return [
            {
                "behavior": ("map_region", "str"),
                "map_x": (coords[0], "float"),
                "map_y": (coords[1], "float"),
            }
        ], {}

    def _parse_set_label_results(self, results: List[dict], metadata: dict) -> bool:
        return True  # TODO

    def _parse_attempt_navigate_results(self, results: List[dict], metadata) -> bool:
        nav_results = results[:-2]
        explore_results = results[-2]
        map_results = results[-1]
        nav_success = self._parse_goto_region_results(
            results=nav_results, metadata=metadata
        )
        explore_target = metadata["exploration_node_target"]
        neighbor = metadata["region_node"]

        explore_success = explore_results["success"][0]
        if explore_success:
            self._add_new_neighbors(node_id=explore_target, new_neighbors=[neighbor])

        map_success = self._parse_map_result(map_results, metadata)

        return explore_success and map_success

    def _parse_explore_to_results(self, result: List[dict], metadata: dict) -> bool:
        explore_results = result[0]
        map_results = result[1]
        vlm_results = result[2]
        explore_success = explore_results["success"][0]
        source_node = metadata["begin_exploration_from"]
        x = metadata["target_explore_x"]
        y = metadata["target_explore_y"]

        region_nodes, _ = self._graph.get_region_nodes_and_locs()

        map_success = False
        if explore_success:
            n_discovered = len([n for n in region_nodes if "discovered" in n])

            new_node = {
                "name": f"discovered_node_{n_discovered+1}",
                "type": "region",
                "coords": [float(x), float(y)],
                "edges": [source_node],
            }
            self._graph.update_with_node(
                node=f"discovered_node_{n_discovered+1}",
                attrs={"type": "region", "coords": [float(x), float(y)]},
                edges=[source_node],
            )

            self._prompt_former.update(new_nodes=[new_node])
            self._log_info(
                f"[action mananger][explore to coord] adding node: {new_node}"
            )

            self._graph_viz.set_graph(self._graph.get_graph())

            metadata["region_node"] = new_node["name"]
            map_success = self._parse_map_result(map_results, metadata)

            metadata["vlm_target"] = new_node["name"]

            self._parse_vlm_result(vlm_results, metadata)

        return explore_success and map_success

    def _parse_map_result(self, map_region_results: dict, metadata: dict) -> bool:
        region_node = metadata["region_node"]
        # filter empty results

        new_neighbors = [
            str(n) for n in map_region_results["new_neighbors"][0].split("|") if n != ""
        ]

        self._add_new_neighbors(node_id=region_node, new_neighbors=new_neighbors)

        return True

    def _parse_map_region_results(self, results: List[dict], metadata: dict) -> bool:
        if "target_nodes" not in metadata or "region_node" not in metadata:
            self._log_info(
                f"ERROR in parse map results. {metadata} must have key target_nodes and region_node"
            )

        self._log_info(f"in parse map region results. All results: {results}")

        nav_results = results[:-2]
        vlm_query_results = results[-2]
        map_region_results = results[-1]
        nav_success = self._parse_goto_region_results(
            results=nav_results, metadata=metadata
        )

        self._log_info(f"map region nav was success: {nav_success}")

        if not nav_success:
            return False

        region_node = metadata["region_node"]
        metadata["vlm_target"] = region_node
        self._parse_vlm_result(vlm_query_results=vlm_query_results, metadata=metadata)

        # # filter empty results
        # new_neighbors = [
        #     str(n) for n in map_region_results["new_neighbors"][0].split("|") if n != ""
        # ]
        # self._add_new_neighbors(node_id=region_node, new_neighbors=new_neighbors)

        self._parse_map_result(map_region_results, metadata)

        self._log_info(f"map region vlm gave result: {vlm_query_results}")
        return True

    def _clean_str(self, in_str) -> str:
        return in_str.replace("'", "").replace('"', "")

    def _add_new_neighbors(self, node_id: str, new_neighbors: List[str]):
        existing_neighbors = self._graph.get_neighbors(node_id)
        current_connections = existing_neighbors + [node_id]

        new_neighbors = [
            neighbor
            for neighbor in new_neighbors
            if neighbor not in current_connections
        ]

        if len(new_neighbors) == 0:
            return []

        all_neighbors = new_neighbors + self._graph.get_neighbors(node_id)
        new_connections = [[node_id, neighbor] for neighbor in new_neighbors]
        self._prompt_former.update(new_connections=new_connections)

        node_type = self._graph.get_node_type(node_id)
        coords = self._graph.get_node_coord(node_id)

        attrs = {"coords": coords, "type": node_type}
        self._graph.update_with_node(node=node_id, edges=all_neighbors, attrs=attrs)

        self._log_info(
            f"[action mananger][explore to] adding neighbors: {new_connections}"
        )

        self._graph_viz.set_graph(self._graph.get_graph())

        return new_neighbors

    def _parse_vlm_result(self, vlm_query_results: dict, metadata: dict) -> bool:
        if "vlm_target" not in metadata:
            self._log_info(f"ERROR vlm_target must be in metadata. Got {metadata}")

        vlm_target = metadata["vlm_target"]
        vlm_success = vlm_query_results["success"][0]

        if not vlm_success:
            return False

        vlm_description = vlm_query_results["answer"][0]

        self._graph.update_node_description(vlm_target, description=vlm_description)

        self._prompt_former.update(
            attribute_updates=[{"name": vlm_target, "description": vlm_description}]
        )

        return True

    def _explore_to(self, goal_x: float, goal_y: float) -> bool:
        x = float(goal_x)
        y = float(goal_y)
        target_loc = np.array([x, y])

        nodes, locs = self._graph.get_region_nodes_and_locs()

        self._log_info(f"locs: {locs} loc shapes: {locs.shape}")

        for node, loc in zip(nodes, locs):
            if np.linalg.norm(loc - target_loc) < 5:
                self._prompt_former.update(
                    freeform_updates=f"{node} is within 5 meters of your target. Use that instead of making a new node."
                )

                return True

        success, frontiers = self._add_frontier(np.array([x, y]).reshape(1, 2))

        if not success:
            self._log_info(f"Could not add frontier. Breaking.")
            return False

        assert len(frontiers) in (0, 1)

        if len(frontiers) == 1:
            return self._goto_region(frontiers[0].id)
        else:
            return False

    def _build_inspect_object_request(
        self, target_object: str, vlm_query: str
    ) -> Tuple[BehaviorRequestData, dict]:
        nearest_region = self._graph.get_neighbors(target_object)
        assert (
            len(nearest_region) == 1
        ), f"objects should only have 1 neighbor. Got: {nearest_region}"

        nearest_region = nearest_region[0]

        self._log_info(
            f"building inspect object region nav: {nearest_region} from {self._graph.get_current_location()}"
        )

        region_coords = self._graph.get_node_coord(nearest_region)
        object_coords = self._graph.get_node_coord(target_object)

        direction_region_to_obj = object_coords - region_coords
        angle = np.arctan2(direction_region_to_obj[1], direction_region_to_obj[0])

        goto_region_request, metadata = self._build_goto_region_request(
            goal_region=nearest_region, target_yaw=angle
        )
        inspect_object_request_dict = self._msg_handler.build_vlm_query_dict(
            query=vlm_query
        )
        request_dicts = goto_region_request + inspect_object_request_dict

        self._log_info(f"inspect object metadata: {metadata}")

        metadata.update(
            {
                "target_object": target_object,
                "vlm_target": target_object,
            }
        )

        return request_dicts, metadata

    def _parse_inspect_object_results(
        self, results: List[dict], metadata: dict
    ) -> bool:
        if "target_object" not in metadata:
            self._log_info(f"ERROR target object must be in metadata. Got {metadata}")

        nav_results = results[:-1]
        vlm_result = results[-1]

        target_object = metadata["target_object"]
        target_region = metadata["region_node"]

        nav_success = self._parse_goto_region_results(
            results=nav_results, metadata=metadata
        )

        if not nav_success:
            self._prompt_former.update(
                freeform_updates=[
                    f"Could not inspect {target_object} because robot could not navigate "
                    f"to neighboring region {target_region}"
                ]
            )
            return False

        vlm_success = vlm_result["success"][0]

        if vlm_success:
            vlm_answer = vlm_result["answer"][0]
            self._prompt_former.update(
                attribute_updates=[{"name": target_object, "description": vlm_answer}]
            )
            return True
        else:
            # TODO figure out what to do here
            self._prompt_former.update(freeform_updates=["unable to inspect object"])
            return True

    def _add_frontier(self, goal: np.ndarray) -> Tuple[bool, List[GraphNode]]:
        if self._frontier_extractor.filtered_costmap_with_info is None:
            self._log_info(f"ERROR no costmap")
            return False, None
        current_location = self._graph.get_current_location()
        frontiers, is_at_obstacle = self._frontier_extractor.get_frontiers(
            proposed_frontier=goal,
            current_location=current_location,
        )

        self._add_frontiers_to_graph(frontiers=frontiers)

        return True, frontiers

    def _add_frontiers_to_graph(
        self, frontiers: list[GraphNode], debug: Optional[bool] = False
    ) -> Tuple[List[GraphNode], bool]:
        """Compute frontiers and add them to graph.

        Parameters
        ----------
        debug : Optional[bool], optional
            If true, don't actually update graph. Just get
            update message, by default False

        Returns
        -------
        - Update message in LLM API.
        - frontier (assumes one currently)
        - is frontier at obstacle boundary
        """
        for frontier in frontiers:
            region_id = frontier.id
            region_loc = frontier.location
            neighbor_ids = frontier.neighbors

            self._log_info(f"adding node: {region_id}, {region_loc}, {neighbor_ids}")

            self._graph.update_with_node(
                node=region_id,
                edges=neighbor_ids,
                attrs={"coords": region_loc, "type": "region"},
            )

            new_node = {
                "name": region_id,
                "type": "region",
                "coords": f"[{region_loc[0]:0.1f}, {region_loc[1]:0.1f}]",
            }
            new_connections = [[region_id, c] for c in neighbor_ids]
            self._prompt_former.update(
                new_nodes=[new_node], new_connections=new_connections
            )

        self._graph_viz.set_graph(self._graph.get_graph())

    def _build_goto_region_request(
        self, goal_region: str, target_yaw: Optional[float] = None
    ) -> Tuple[BehaviorRequestData, dict]:
        nav_start_location = self._graph.get_current_location()
        if nav_start_location == goal_region and target_yaw == None:
            return [], {"region_node": goal_region, "target_nodes": []}

        self._log_info(f"computing path between {nav_start_location} -> {goal_region}")
        path = self._graph.get_path(nav_start_location, goal_region)
        self._log_info(f"path is: {path}")

        # for goal messages
        request_goals = []
        target_nodes = []
        for node in path:
            self._log_info(f"Next node: {node}")
            if nav_start_location == node and goal_region != node:
                continue

            coords = self._graph.get_node_coord(node)

            if node == goal_region and target_yaw != None:
                request_goals.append(
                    self._msg_handler.build_navigate_msg_dict(
                        coords[0], coords[1], target_yaw, True
                    )
                )
            else:
                request_goals.append(
                    self._msg_handler.build_navigate_msg_dict(
                        coords[0], coords[1], 0, False
                    )
                )

            target_nodes.append(node)

        return request_goals, {"target_nodes": target_nodes, "region_node": goal_region}

    def _parse_goto_region_results(self, results: List[dict], metadata: dict) -> bool:
        if "target_nodes" not in metadata:
            self._log_info(f"ERROR key target_nodes must be in {metadata}")

        path_nav_results = []
        for result in results:
            path_nav_results.append(result["success"][0])

        target_nodes = metadata["target_nodes"]
        nav_start_location = self._graph.get_current_location()

        self._log_info(
            f"parsing nav path results with start: {nav_start_location}, target: {target_nodes}, result: {path_nav_results}"
        )
        current_node = nav_start_location
        for target_node, result in zip(target_nodes, path_nav_results):
            nav_success = result

            if nav_success:
                current_node = target_node

            # if segment traversal was not successful, remove edge and break
            else:
                self._prompt_former.update(
                    freeform_updates=[
                        f"could not navigate between [{current_node}, {target_node}]. Connection is likely blocked or this robot {self._robot_name} is not functional."
                    ]
                )
                self._graph.remove_edge(current_node, target_node)
                self._prompt_former.update(
                    removed_connections=[[current_node, current_node]]
                )

                if current_node != nav_start_location:
                    self._graph.update_location(current_node)

                return False

        # update robot location if needed
        if current_node != nav_start_location:
            self._graph.update_location(target_node)
            self._prompt_former.update(
                location_updates=[current_node], register_updates=True
            )

        return True
