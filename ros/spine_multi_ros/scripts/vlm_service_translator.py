#!/usr/bin/env python3

import time
from typing import Dict, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from spine_multi_ros.vlm_manager import VLMReq
from std_msgs.msg import Int16, String
from teaming_msgs.msg import VLMRequest, VLMResponse
from teaming_msgs.srv import Query


class VLMServiceTranslator(Node):
    def __init__(self):
        super().__init__("vlm_service_translator")
        self._vlm_query_dict: Dict[int, VLMReq] = {}

        self.declare_parameters(
            namespace="",
            parameters=[("vlm_service", "vlm_node/query_scene"), ("max_pub_count", 3)],
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        vlm_query_client_name = (
            self.get_parameter("vlm_service").get_parameter_value().string_value
        )

        self._max_pub_count = (
            self.get_parameter("max_pub_count").get_parameter_value().integer_value
        )

        vlm_cbk_group = ReentrantCallbackGroup()
        self._vlm_client = self.create_client(
            Query, vlm_query_client_name, callback_group=vlm_cbk_group
        )
        self._vlm_client.wait_for_service()

        sub_cbk_group = ReentrantCallbackGroup()
        self._vlm_req_sub = self.create_subscription(
            VLMRequest,
            "vlm_request",
            self._req_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        ack_sub_group = ReentrantCallbackGroup()
        self._ack_sub = self.create_subscription(
            Int16,
            "vlm_ack",
            self._ack_cbk,
            qos_profile,
            callback_group=ack_sub_group,
        )

        self._status_update_pub = self.create_publisher(VLMResponse, "vlm_response", 10)

    def _req_cbk(self, req: VLMRequest) -> None:
        if req.idx not in self._vlm_query_dict:
            self._process_request(req)


    def _process_request(self, req: VLMRequest):
        self._vlm_query_dict[req.idx] = VLMReq(idx=req.idx, ack=False)

        success, resp = self._query_vlm(req.query.data)

        resp_msg = VLMResponse()
        resp_msg.idx = req.idx
        string_msg = String()
        resp_msg.answer.data = resp

        for _ in range(self._max_pub_count):
            if self._vlm_query_dict[req.idx].ack:
                break

            self.get_logger().info(
                f"[vlm service translator] sending answer for {req.idx}: {resp}"
            )
            self._status_update_pub.publish(resp_msg)
            # time.sleep(5.0)

    def _ack_cbk(self, msg: Int16) -> None:
        self.get_logger().info(f"[vlm service translator] got ack for {msg.data}")
        if msg.data in self._vlm_query_dict:
            self._vlm_query_dict[msg.data].ack = True

    def _query_vlm(self, query: str) -> Tuple[bool, str]:
        request = Query.Request()
        request.query = ascii(query)

        try:
            self.get_logger().info(
                f"[action manager] formed request vlm query: {request}"
            )
            response_future = self._vlm_client.call_async(request)
            rclpy.spin_until_future_complete(self, response_future)

            response = response_future.result()
            self.get_logger().info(f"[action manager] got vlm query: {response}")
            return response.success, response.answer
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False, ""


def main():
    rclpy.init()
    node = VLMServiceTranslator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
