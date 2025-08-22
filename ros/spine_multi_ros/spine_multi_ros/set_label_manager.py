import time
from abc import ABC
from dataclasses import dataclass
from typing import Dict, Tuple

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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
        label_request: str = "label_request",
        label_response: str = "label_response",
        label_ack: str = "label_ack",
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name

        self._current_query_idx = -1
        self._query_dict: Dict[int, LabelResp] = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._label_req_pub = self._parent_node.create_publisher(
            LabelRequest,
            label_request,
            qos_profile,
        )

        sub_cbk_group = ReentrantCallbackGroup()
        self._response_sub = self._parent_node.create_subscription(
            LabelResponse,
            label_response,
            self._req_status_ckb,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        self._ack_pub = self._parent_node.create_publisher(
            Int16, label_ack, qos_profile
        )

    def _req_status_ckb(self, msg: LabelResponse) -> None:
        if msg.idx in self._query_dict:
            self._query_dict[msg.idx].success = msg.success
            self._query_dict[msg.idx].ack = True

            ack_msg = Int16()
            ack_msg.data = msg.idx
            self._ack_pub.publish(ack_msg)

        self._parent_node.get_logger().info(
            f"[label manager topic] [{self._robot_name}] sent ack for idx: {msg.idx}"
        )

    def _set_labels(self, label_list: str) -> Tuple[bool, str]:
        self._current_query_idx += 1

        req_msg = LabelRequest()

        string_msg = String()
        string_msg.data = label_list
        req_msg.labels = string_msg
        req_msg.idx = self._current_query_idx

        self._query_dict[self._current_query_idx] = LabelResp(
            idx=self._current_query_idx, ack=False, success=False
        )

        self._parent_node.get_logger().info(
            f"[label manager topic] [{self._robot_name}] sending idx: {self._current_query_idx} for query: {label_list}"
        )

        while not self._query_dict[self._current_query_idx].ack:
            self._label_req_pub.publish(req_msg)
            time.sleep(5)
            self._parent_node.get_logger().info(
                f"[label manager topic] [{self._robot_name}] waiting for idx: {self._current_query_idx} for query: {label_list}"
            )

        success = self._query_dict[self._current_query_idx].success
        self._parent_node.get_logger().info(
            f"[label manager topic] [{self._robot_name}] got answer for idx {self._current_query_idx}: {success}"
        )

        return True, success
