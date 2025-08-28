#!/usr/bin/env python3

import time
from typing import Dict, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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
            parameters=[("label_service", "detector/set_labels"), ("max_pub_count", 1),
                        ("subscription_prefix", "")],
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        set_labe_client_name = (
            self.get_parameter("label_service").get_parameter_value().string_value
        )

        self._max_pub_count = (
            self.get_parameter("max_pub_count").get_parameter_value().integer_value
        )

        subscription_prefix = (
            self.get_parameter("subscription_prefix").get_parameter_value().string_value
        )
 

        label_cbk_group = ReentrantCallbackGroup()
        self._label_client = self.create_client(
            SetLabels, set_labe_client_name, callback_group=label_cbk_group
        )
        self._label_client.wait_for_service()

        sub_cbk_group = ReentrantCallbackGroup()
        self._vlm_req_sub = self.create_subscription(
            LabelRequest,
            subscription_prefix + "label_request",
            self._req_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        ack_sub_group = ReentrantCallbackGroup()
        self._ack_sub = self.create_subscription(
            Int16,
            subscription_prefix + "label_ack",
            self._ack_cbk,
            qos_profile,
            callback_group=ack_sub_group,
        )

        self._status_update_pub = self.create_publisher(
            LabelResponse, "label_response", qos_profile
        )

    def _req_cbk(self, req: LabelRequest) -> None:
        self.get_logger().info(
            f"[label service translator] got req for {req.idx}"
        )
 
        if req.idx not in self._label_query_dict:
            self._process_request(req)


    def _process_request(self, req: LabelRequest):
        self._label_query_dict[req.idx] = LabelReq(idx=req.idx, ack=False)
        self._call_set_labels_async(req)

    def _call_set_labels_async(self, req: LabelRequest):
        request = SetLabels.Request()
        request.labels = ascii(req.labels.data)
        
        self.get_logger().info(f"[label service translator] calling set_labels async for {req.idx}")
        
        future = self._label_client.call_async(request)
        future.add_done_callback(lambda f: self._handle_service_response(f, req))

    def _handle_service_response(self, future, req: LabelRequest):
        try:
            response = future.result()
            success = response.success
            self.get_logger().info(f"[label service translator] service returned {success} for {req.idx}")
        except Exception as e:
            self.get_logger().error(f"Service call failed for {req.idx}: {e}")
            success = False
        
        # Now start the publishing loop
        resp_msg = LabelResponse()
        resp_msg.idx = req.idx
        resp_msg.success = success
        
        self._start_response_publishing(resp_msg, 0)

    def _start_response_publishing(self, resp_msg: LabelResponse, attempt: int):
        if attempt >= self._max_pub_count:
            return
            
        if resp_msg.idx in self._label_query_dict and self._label_query_dict[resp_msg.idx].ack:
            self.get_logger().info(f"[label service manager] Got ack for {resp_msg.idx}, stopping")
            return
        
        self.get_logger().info(f"[label service manager] Publishing response {attempt+1}/{self._max_pub_count} for {resp_msg.idx}")
        self._status_update_pub.publish(resp_msg)
        
        # Schedule next attempt (non-blocking)
        timer = self.create_timer(5.0, lambda: self._start_response_publishing(resp_msg, attempt + 1))
        timer.cancel()  
            


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
