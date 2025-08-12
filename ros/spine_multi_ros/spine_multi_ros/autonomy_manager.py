#!/usr/bin/env python3


from logging import Logger
from typing import Optional

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from spine_multi.multi_robot_graph import MultiRobotGraphHandler
from spine_multi.spine import GraphHandler
from spine_multi.spine.mapping.frontiers import FrontierExtractor
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent
from teaming_msgs.srv import Query

from spine_multi_ros.action_manager import ActionManager
from spine_multi_ros.nav_manager import (
    NavigationComponentAction,
    NavigationComponentTopic,
)
from spine_multi_ros.set_label_manager import LabelManagerTopic
from spine_multi_ros.tracker_client import TrackerClientComponenet
from spine_multi_ros.vlm_manager import VLMManagerAction, VLMManagerTopic


class AutonomyManager:
    """Wrapper around action handler that initializes all members
    and subscribes to perception (e.g., costmap, detection)
    """

    def __init__(
        self,
        *,
        parent_node: Node,
        robot_name: str,
        init_graph: GraphHandler,
        init_node: str,
        nav_target_frame: str,
        graph_viz_topic: str,
        track_topic: str,
        local_costmap_topic: str,
        vlm_params: dict,
        navigation_params: dict,
        label_params: dict,
        tracks: dict,
        logger: Logger,
        use_actions: Optional[bool] = False,
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name
        self._logger = logger
        self._graph = MultiRobotGraphHandler(
            graph_handler=init_graph, init_node=init_node
        )
        self._frontier_extractor = FrontierExtractor(self._graph)
        self._prompt_former = UpdatePromptFormer()
        self._label_manager = LabelManagerTopic(
            parent_node=parent_node, robot_name=robot_name, **label_params
        )

        # TODO need better handling here
        if use_actions:
            self._nav_component = NavigationComponentAction(
                parent_node=parent_node, frame_id=nav_target_frame
            )
            self._vlm_manager = VLMManagerAction(parent_node=parent_node, **vlm_params)
        else:
            self._nav_component = NavigationComponentTopic(
                parent_node=parent_node, robot_name=robot_name, **navigation_params
            )
            self._vlm_manager = VLMManagerTopic(
                parent_node=parent_node, robot_name=self._robot_name, **vlm_params
            )

        self._graph_viz = GraphVisualizerComponent(
            parent_node=parent_node,
            graph=self._graph,
            target_frame=nav_target_frame,
            topic_name=graph_viz_topic,
            scale=0.5,
        )
        self._graph_viz.set_graph(self._graph.get_graph())

        self._tracker_componenet = TrackerClientComponenet(
            parent_node,
            self._graph,
            self._prompt_former,
            self._graph_viz,
            track_topic=track_topic,
            tracks=tracks,
        )

        self._log_info(f"tracker cbk init")

        self._log_info(f"vlm client init")

        self._action_manager = ActionManager(
            parent_node=parent_node,
            robot_name=self._robot_name,
            graph=self._graph,
            graph_viz=self._graph_viz,
            prompt_former=self._prompt_former,
            nav_componenet=self._nav_component,
            vlm_client=self._vlm_manager,
            frontier_extractor=self._frontier_extractor,
            logger=self._logger,
        )

        self._behavior_library = self._action_manager.construct_behavior_library()
        self._behavior_library["set_labels"] = self.set_labels

        self._log_info(f"constructed behavior library")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        costmap_cbk_group = ReentrantCallbackGroup()
        self.costmap_sub = self._parent_node.create_subscription(
            OccupancyGrid,
            local_costmap_topic,
            self._costmap_cbk,
            qos_profile,
            callback_group=costmap_cbk_group,
        )

        self._log_info(f"initialized costmap cbk")

        self._log_info(f"initialized")

    def _log_info(self, msg: str) -> None:
        log_msg = f"[autonomy manager] [{self._robot_name}] {msg}"

        self._parent_node.get_logger().info(log_msg)
        self._logger.info(log_msg)

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

    def set_labels(self, labels: str) -> bool:
        return self._label_manager._set_labels(labels)
