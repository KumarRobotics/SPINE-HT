import time
from abc import ABC
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from spine_multi_ros.msg_handler import MessageHandler
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
        msg_handler: MessageHandler,
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name
        self._msg_handler = msg_handler

        self.open_scene_prompt = (
            "You are a robot. Describe where you are so you can plan. "
            "Provide your answer as a noun with a short description. For example: empty sidewalk, road, park with trees and benches, empty parking lot, patio."
        )

    def _log_info(self, msg):
        self._parent_node.get_logger().info(f"[vlm manager] {msg}")

    def describe_scene(self) -> Tuple[bool, str]:
        return self._query_vlm(self.open_scene_prompt)

    def _query_vlm(self, query: str) -> Tuple[bool, str]:

        query_msg = self._msg_handler.get_vlm_query_msg(query=query)

        result = self._msg_handler.send_and_wait(query_msg)

        answer = result[0]['answer'][0]

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
