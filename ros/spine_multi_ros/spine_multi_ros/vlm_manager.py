import time
from abc import ABC
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int16, String
from teaming_msgs.msg import VLMRequest, VLMResponse
from teaming_msgs.srv import Query


@dataclass
class VLMReq:
    idx: int
    ack: bool


@dataclass
class VLMResp:
    idx: int
    ack: bool
    answer: str


class VLMManager(ABC):
    def __init__(self, parent_node: Node) -> None:
        pass

    def _query_vlm(self, query: str) -> Tuple[bool, str]:
        pass


class VLMManagerTopic(VLMManager):
    def __init__(
        self,
        parent_node: Node,
        robot_name: str,
        vlm_request: str = "vlm_request",
        vlm_response: str = "vlm_response",
        vlm_ack: str = "vlm_ack",
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name

        self._current_query_idx = -1
        self._query_dict: Dict[int, VLMResp] = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._goal_req_pub = self._parent_node.create_publisher(
            VLMRequest,
            vlm_request,
            qos_profile,
        )

        sub_cbk_group = ReentrantCallbackGroup()
        self._response_sub = self._parent_node.create_subscription(
            VLMResponse,
            vlm_response,
            self._req_status_ckb,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        self._ack_pub = self._parent_node.create_publisher(Int16, vlm_ack, qos_profile)

    def _req_status_ckb(self, msg: VLMResponse) -> None:
        if msg.idx not in self._query_dict:
            return

        self._query_dict[msg.idx].answer = msg.answer
        self._query_dict[msg.idx].ack = True

        ack_msg = Int16()
        ack_msg.data = msg.idx
        self._ack_pub.publish(ack_msg)

        # self._parent_node.get_logger().info(
        #    f"[vlm manager topic] [{self._robot_name}] sent ack for idx: {msg.idx}"
        # )

    def _query_vlm(self, query: str) -> Tuple[bool, str]:
        self._current_query_idx += 1

        req_msg = VLMRequest()

        string_msg = String()
        string_msg.data = query
        req_msg.query = string_msg
        req_msg.idx = self._current_query_idx

        self._query_dict[self._current_query_idx] = VLMResp(
            idx=self._current_query_idx, ack=False, answer=""
        )

        self._parent_node.get_logger().info(
            f"[vlm manager topic] [{self._robot_name}] sending idx: {self._current_query_idx} for query: {query}"
        )

        while not self._query_dict[self._current_query_idx].ack:
            self._goal_req_pub.publish(req_msg)
            time.sleep(1)
            # self._parent_node.get_logger().info(
            #     f"[vlm manager topic] [{self._robot_name}] waiting for idx: {self._current_query_idx} for query: {query}"
            # )

        answer = self._query_dict[self._current_query_idx].answer
        self._parent_node.get_logger().info(
            f"[vlm manager topic] [{self._robot_name}] got answer for idx {self._current_query_idx}: {answer}"
        )

        return True, answer


class VLMManagerAction(VLMManager):
    def __init__(self, parent_node: Node):
        self._parent_node = parent_node

        vlm_cbk_group = ReentrantCallbackGroup()
        self._vlm_client = self._parent_node.create_client(
            Query, "vlm_node/query_scene", callback_group=vlm_cbk_group
        )
        self._vlm_client.wait_for_service()

        self._parent_node.get_logger().info("[VLM manager action] init")

    def _query_vlm(self, query: str) -> Tuple[bool, str]:
        request = Query.Request()
        request.query = ascii(query)

        try:
            self._parent_node.get_logger().info(
                f"[action manager] formed request vlm query: {request}"
            )
            response_future = self._vlm_client.call_async(request)
            rclpy.spin_until_future_complete(self._parent_node, response_future)

            response = response_future.result()
            self._parent_node.get_logger().info(
                f"[action manager] got vlm query: {response}"
            )
            return response.success, response.answer
        except Exception as e:
            self._parent_node.get_logger().error(f"Service call failed: {e}")
            return False, ""
