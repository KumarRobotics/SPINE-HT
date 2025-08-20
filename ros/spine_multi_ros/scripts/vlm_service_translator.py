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
            reliability=ReliabilityPolicy.RELIABLE,
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

        self._active_timers: Dict[int, object] = {}

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
        self.get_logger().info(f"[vlm service translator] got req for {req.idx}")
        
        if req.idx not in self._vlm_query_dict:
            self.get_logger().info(f"[vlm service translator] Processing new request {req.idx}")
            self._process_request_async(req)
        else:
            self.get_logger().info(f"[vlm service translator] Skipping duplicate request {req.idx}")

    def _process_request_async(self, req: VLMRequest):
        """Start async processing of the request"""
        self._vlm_query_dict[req.idx] = VLMReq(idx=req.idx, ack=False)
        
        # Make service call async without blocking
        self._query_vlm_async(req)

    def _query_vlm_async(self, req: VLMRequest):
        """Make VLM service call without blocking"""
        request = Query.Request()
        request.query = ascii(req.query.data)
        
        self.get_logger().info(f"[vlm service translator] calling VLM service async for idx {req.idx} with query: {request.query}")
        
        future = self._vlm_client.call_async(request)
        future.add_done_callback(lambda f: self._handle_vlm_response(f, req))

    def _handle_vlm_response(self, future, req: VLMRequest):
        """Handle VLM service response and start publishing"""
        self.get_logger().info(f"[vlm service translator] handling vlm resp for {req.idx}")
 
        try:
            response = future.result()
            success = response.success
            answer = response.answer
            self.get_logger().info(f"[vlm service translator] VLM returned success={success} for {req.idx} with answer: {answer}")
        except Exception as e:
            self.get_logger().error(f"[vlm service translator] VLM service call failed for {req.idx}: {e}")
            success = False
            answer = ""
        
        # Create response message
        resp_msg = VLMResponse()
        resp_msg.idx = req.idx
        resp_msg.answer = String()
        resp_msg.answer.data = answer
        
        # Start non-blocking publishing loop
        self._start_response_publishing(resp_msg, 0)

    def _start_response_publishing(self, resp_msg: VLMResponse, attempt: int):
        """Publish response with retries, non-blocking"""
        self.get_logger().info(f"[vlm service translator] Starting repub for idx {resp_msg.idx} attempt {attempt}")
 
        if attempt >= self._max_pub_count:
            self.get_logger().info(f"[vlm service translator] Max attempts reached for {resp_msg.idx}")
            self._cleanup_req(attempt)
            return
            
        # Check if we got ack
        if resp_msg.idx in self._vlm_query_dict and self._vlm_query_dict[resp_msg.idx].ack:
            self.get_logger().info(f"[vlm service translator] Got ack for {resp_msg.idx}, stopping retries")
            self._cleanup_req(attempt)
            return
        
        self.get_logger().info(f"[vlm service translator] Publishing attempt {attempt+1}/{self._max_pub_count} for {resp_msg.idx}")
        self._status_update_pub.publish(resp_msg)
        
        # Schedule next attempt in 5 seconds (non-blocking)
        timer = self.create_timer(
            5.0, 
            lambda: self._timer_callback(resp_msg, attempt + 1)
        )

        self._active_timers[resp_msg.idx] = timer

    def _timer_callback(self, resp_msg: VLMResponse, attempt: int):
        if resp_msg.idx in self._active_timers:
            self._active_timers[resp_msg.idx].cancel()
            del self._active_timers[resp_msg.idx]

        self._start_response_publishing(resp_msg, attempt)

    def _cleanup_req(self, idx: int):
        if idx in self._active_timers:
            self._active_timers[idx].cancel()
            del self._active_timers[idx]


    def _ack_cbk(self, msg: Int16) -> None:
        self.get_logger().info(f"[vlm service translator] got ack for {msg.data}")
        if msg.data in self._vlm_query_dict:
            self.get_logger().info(f"[vlm service translator] Setting ack=True for {msg.data}")
            self._vlm_query_dict[msg.data].ack = True
        else:
            self.get_logger().info(f"[vlm service translator] Request {msg.data} not found in dict")


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
