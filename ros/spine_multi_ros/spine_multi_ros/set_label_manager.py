import time
from abc import ABC
from dataclasses import dataclass
from typing import Dict, Tuple

from spine_multi_ros.msg_handler import MessageHandler
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int16, String
from teaming_msgs.msg import LabelRequest, LabelResponse


@dataclass
class LabelReq:
    idx: int
    ack: bool


@dataclass
class LabelResp:
    idx: int
    ack: bool
    success: bool


class LabelManager(ABC):
    def __init__(self, parent_node: Node) -> None:
        pass

    def _set_labels(self, query: str) -> Tuple[bool, str]:
        pass


class LabelManagerTopic(LabelManager):
    def __init__(
        self,
        parent_node: Node,
        robot_name: str,
        msg_handler: MessageHandler,
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name
        self._msg_handler = msg_handler

      
    def _set_labels(self, label_list: str) -> Tuple[bool, str]:
        msg = self._msg_handler.get_label_msg(label_list=label_list)
        success = self._msg_handler.send_and_wait(msg)

        # success = self._query_dict[self._current_query_idx].success
        self._parent_node.get_logger().info(
            f"[label manager topic] [{self._robot_name}] got answer for idx {self._msg_handler._behavior_idx}: {success}"
        )

        return success
