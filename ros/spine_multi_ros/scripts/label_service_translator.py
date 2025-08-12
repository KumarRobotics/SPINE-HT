#!/usr/bin/env python3

import time
from typing import Dict, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from spine_multi_ros.set_label_manager import LabelReq
from std_msgs.msg import Int16, String
from teaming_msgs.msg import LabelRequest, LabelResponse
from teaming_msgs.srv import SetLabels


class LabelServiceTranslator(Node):
    def __init__(self):
        super().__init__("label_service_translator")
        self._label_query_dict: Dict[int, LabelReq] = {}

        self.declare_parameters(
            namespace="",
            parameters=[("label_service", "detector/set_labels"), ("max_pub_count", 5)],
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        set_labe_client_name = (
            self.get_parameter("label_service").get_parameter_value().string_value
        )

        self._max_pub_count = (
            self.get_parameter("max_pub_count").get_parameter_value().integer_value
        )

        label_cbk_group = ReentrantCallbackGroup()
        self._label_client = self.create_client(
            SetLabels, set_labe_client_name, callback_group=label_cbk_group
        )
        self._label_client.wait_for_service()

        sub_cbk_group = ReentrantCallbackGroup()
        self._vlm_req_sub = self.create_subscription(
            LabelRequest,
            "label_request",
            self._req_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        ack_sub_group = ReentrantCallbackGroup()
        self._ack_sub = self.create_subscription(
            Int16,
            "label_ack",
            self._ack_cbk,
            qos_profile,
            callback_group=ack_sub_group,
        )

        self._status_update_pub = self.create_publisher(
            LabelResponse, "label_response", 10
        )

    def _req_cbk(self, req: LabelRequest) -> None:
        if req.idx in self._label_query_dict:
            return

        self._label_query_dict[req.idx] = LabelReq(idx=req.idx, ack=False)

        success = self._set_labels(req.labels.data)

        resp_msg = LabelResponse()
        resp_msg.idx = req.idx
        resp_msg.success = success

        self.get_logger().info(
            f"[label service translator] set label success for {req.idx}: {success}"
        )
 
        for _ in range(self._max_pub_count):
            try:
                if self._label_query_dict[req.idx].ack:
                    self.get_logger().info(
                        f"[label service translator] have ack for {req.idx}"
                    )
         
                    return

                self.get_logger().info(
                    f"[label service translator] sending answer for {req.idx}: {success}"
                )
                self._status_update_pub.publish(resp_msg)
                time.sleep(5.0)
            except Exception as ex:
                self.get_logger().info(f"[label service translator] got ex: {ex}")

    def _ack_cbk(self, msg: Int16) -> None:
        self.get_logger().info(f"[label service translator] got ack for {msg.data}")
        if msg.data in self._label_query_dict:
            self._label_query_dict[msg.data].ack = True

    def _set_labels(self, query: str) -> bool:
        request = SetLabels.Request()
        request.labels = ascii(query)

        try:
            self.get_logger().info(
                f"[label service translator] formed request for set labels: {request}"
            )
            response_future = self._label_client.call_async(request)
            rclpy.spin_until_future_complete(self, response_future)

            response = response_future.result()
            self.get_logger().info(f"[label service translator] got : {response}")
            return response.success
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False


def main():
    rclpy.init()
    node = LabelServiceTranslator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
