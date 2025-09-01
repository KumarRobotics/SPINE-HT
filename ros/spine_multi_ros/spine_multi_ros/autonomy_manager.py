#!/usr/bin/env python3


from logging import Logger
from typing import Any, List, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from spine_multi.multi_robot_graph import MultiRobotGraphHandler
from spine_multi.spine import GraphHandler
from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent

from spine_multi_ros.action_manager import ActionManager
from spine_multi_ros.msg_handler import MessageHandler
from spine_multi_ros.tracker_client import TrackerClientComponent


class AutonomyManager:
    """Wrapper around action handler that initializes all members
    and subscribes to perception (e.g., costmap, detection)
    """

    def __init__(
        self,
        *,
        parent_node: Node,
        robot_name: str,
        subscription_prefix: str,
        init_graph: GraphHandler,
        init_node: str,
        nav_target_frame: str,
        graph_viz_topic: str,
        track_topic: str,
        local_costmap_topic: str,
        tracks: dict,
        logger: Logger,
        behavior_request_pub: str,
        behavior_request_ack_sub: str,
        behavior_result_sub: str,
        behavior_result_ack_pub: str,
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name
        self._logger = logger
        self._subscription_prefix = subscription_prefix
        self._graph = MultiRobotGraphHandler(
            graph_handler=init_graph, init_node=init_node
        )
        self._frontier_extractor = FrontierExtractor(self._graph)
        self._prompt_former = UpdatePromptFormer()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        self._msg_handler = MessageHandler(
            parent=self._parent_node,
            robot_name=self._robot_name,
            behavior_request=behavior_request_pub,
            behavior_request_ack_sub=behavior_request_ack_sub,
            behavior_result_sub=behavior_result_sub,
            behavior_result_ack_pub=behavior_result_ack_pub,
            qos_profile=qos_profile,
        )

        self._graph_viz = GraphVisualizerComponent(
            parent_node=parent_node,
            graph=self._graph,
            target_frame=nav_target_frame,
            topic_name=graph_viz_topic,
            scale=0.5,
        )
        self._graph_viz.set_graph(self._graph.get_graph())

        self._tracker_component = TrackerClientComponent(
            parent_node,
            self._graph,
            self._prompt_former,
            self._graph_viz,
            subscription_prefix=subscription_prefix,
            track_topic=track_topic,
            tracks=tracks,
        )

        self._action_manager = ActionManager(
            parent_node=parent_node,
            robot_name=self._robot_name,
            graph=self._graph,
            graph_viz=self._graph_viz,
            prompt_former=self._prompt_former,
            frontier_extractor=self._frontier_extractor,
            logger=self._logger,
            msg_handler=self._msg_handler,
        )

        self._behavior_library = self._action_manager.construct_behavior_library()
        self._msg_building_library = (
            self._action_manager.construct_msg_building_library()
        )
        self._msg_parsing_library = self._action_manager.construct_msg_parsing_library()

        self._log_info(f"constructed behavior library")

        costmap_cbk_group = ReentrantCallbackGroup()
        self.costmap_sub = self._parent_node.create_subscription(
            OccupancyGrid,
            subscription_prefix + local_costmap_topic,
            self._costmap_cbk,
            qos_profile,
            callback_group=costmap_cbk_group,
        )
        self._log_info(f"initialized costmap cbk")

        self._log_info(f"initialized")

    def _log_info(self, msg: str) -> None:
        log_msg = f"[autonomy manager] [{self._robot_name}] {msg}"

        self._parent_node.get_logger().info(log_msg)
        # self._logger.info(log_msg)

    def _costmap_cbk(self, costmap_msg: OccupancyGrid) -> None:
        # self.get_logger().info('[spine node] get costmap')
        pose_in_map = costmap_msg.info.origin
        position = np.array([pose_in_map.position.x, pose_in_map.position.y])
        yaw = Rotation.from_quat(
            [
                pose_in_map.orientation.x,
                pose_in_map.orientation.y,
                pose_in_map.orientation.z,
                pose_in_map.orientation.w,
            ]
        ).as_euler("xyz")[0]
        costmap = np.array(costmap_msg.data).reshape(
            costmap_msg.info.height, costmap_msg.info.width
        )
        filtered_costmap = self._frontier_extractor.update_costmap(
            costmap=costmap,
            resolution_m_p_cell=costmap_msg.info.resolution,
            pos=position,
            yaw=yaw,
        )

    def call_behavior(self, behavior, args):
        return self._behavior_library[behavior](*args)

    def call_behavior_sequence(
        self, behavior_requests: List[Tuple[str, List[Any]]]
    ) -> List[bool]:
        behavior_req_data = []
        all_metadata = {}
        behavior_info = []

        self._log_info(f"in call behavior sequence with {behavior_requests}")

        for behavior, args in behavior_requests:
            if behavior not in self._msg_building_library:
                self._log_info(f"ERROR {behavior} not in msg building library")

            behavior_req_msg, metadata = self._msg_building_library[behavior](*args)

            behavior_req_data.extend(behavior_req_msg)
            behavior_info.append([behavior, len(behavior_req_msg), metadata])

        behavior_req_msg = self._msg_handler.build_behavior_msg(behavior_req_data)
        results = self._msg_handler.send_and_wait(behavior_req_msg)

        self._log_info(f"got back results: {results}")

        # now parse

        idx = 0
        behavior_success = []

        for behavior, n_msgs, metadata in behavior_info:
            msg_block = results[idx : idx + n_msgs]
            result = self._msg_parsing_library[behavior](msg_block, metadata)
            behavior_success.append(result)
            idx += n_msgs

        return behavior_success
