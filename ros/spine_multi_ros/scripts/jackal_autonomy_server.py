#!/usr/bin/env python3

import math
import time
from abc import ABC
from concurrent.futures import Future
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Dict

import numpy as np
import rclpy
import rclpy.task
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from spine_multi.parsing import ListDictParser
from std_msgs.msg import Int16, String
from teaming_msgs.msg import BehaviorRequest, BehaviorResult, GoalRequest, GoalStatus
from teaming_msgs.srv import SetLabels, Query


class GoalStatusEnum(Enum):
    IN_PROGRESS = 0
    SUCCESS = 1
    FAILED = 2


@dataclass
class NavGoal:
    idx: int
    x: float
    y: float
    yaw: float
    status: GoalStatusEnum
    ack: bool


class JackalAutonomyManager(Node):
    def __init__(self):
        super().__init__("jackal_autonomy_manager")

        self._log_info("init")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        self.declare_parameters(
            namespace="",
            parameters=[
                ("navigation_action_server", "navigate_to_pose"),
                ("navigation_request", "navigation_request"),
                (f"robot_name", ""),
                ("frame_id", "map"),
                ("parameter_server", "controller_server/set_parameters"),
                ("subscription_prefix", ""),
                ("label_service", "/detector/set_labels"),
                ("vlm_service", "vlm_node/query_scene"),
            ],
        )

        set_label_client_name = (
            self.get_parameter("label_service").get_parameter_value().string_value
        )
        subscription_prefix = (
            self.get_parameter("subscription_prefix").get_parameter_value().string_value
        )
        vlm_service_name = (
            self.get_parameter("vlm_service").get_parameter_value().string_value
        )

        self._parser = ListDictParser()

        sub_cbk_group = ReentrantCallbackGroup()
        self._nav_req_sub = self.create_subscription(
            BehaviorRequest,
            subscription_prefix + "behavior_request",
            self._behavior_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        self._received_behaviors = set()
        self._acked_results = set()
        self._behavior_results: Dict[int, List[dict]] = {}

        self._ack_pub = self.create_publisher(
            Int16,
            "behavior_request_ack",
            qos_profile,
        )

        self._result_pub = self.create_publisher(
            BehaviorResult, "behavior_result", qos_profile
        )

        response_ack_cbk = ReentrantCallbackGroup()
        self._response_ack_sub = self.create_subscription(
            Int16,
            subscription_prefix + "behavior_result_ack",
            self._response_ack,
            qos_profile,
            callback_group=response_ack_cbk,
        )

        self._log_info("created pub / sub")

        self._set_labels_component = LabelServiceTranslator(
            parent_node=self,
            label_service=set_label_client_name,
        )

        self._log_info("set label init")

        # navigation
        self._nav_component = JackalNavigationComponent(parent_node=self)

        # vlm_query
        self._vlm_component = VLMServiceComponent(
            parent_node=self,
            vlm_service=vlm_service_name,
        )
        # TODO have queue of results to publish
        result_pub_group = ReentrantCallbackGroup()
        # self._result_pub_timer = self.create_timer(
        #     1, self._result_pub_cbk, callback_group=result_pub_group
        # )

    def _result_pub_cbk(self):
        self._log_info(f"in result pub cbk: {self._behavior_results}")
        results_to_remove = []
        for idx, result_msg in self._behavior_results.items():
            self._result_pub.publish(result_msg)
            self._log_info(f"publishing result: {result_msg}")

            if idx in self._acked_results:
                self._log_info(f"result {idx} has been acked")
                results_to_remove.append(idx)

        for to_remove in results_to_remove:
            self._behavior_results.pop(idx)

    def _response_ack(self, ack_msg: Int16):
        self._acked_results.add(ack_msg.data)
        self._log_info(f"got ack response for idx: {ack_msg.data}")

    def _build_result_msg(self, idx: int, data: List[Dict[str, Any]]) -> BehaviorResult:
        self._log_info(f"building result: {data}")

        try:
            resp_msg = BehaviorResult()
            resp_msg.idx = idx
            resp_msg_str = String()
            resp_msg_str.data = self._parser.serialize(data)

            resp_msg.result = resp_msg_str
        except Exception as ex:
            self._log_info(f"got exception {ex}")

        self._log_info("returning  result")

        return resp_msg

    def _log_info(self, msg: str):
        self.get_logger().info(f"[jackal autonomy manager] {msg}")

    def _behavior_cbk(self, msg: BehaviorRequest) -> None:
        self._log_info(f"got msg: {msg}")
        ack_msg = Int16()
        ack_msg.data = msg.idx
        self._ack_pub.publish(ack_msg)

        if msg.idx not in self._received_behaviors:
            self._received_behaviors.add(msg.idx)
            task_sequence = self._parser.parse(msg.behaviors.data)

            self._log_info(f"parsed task: {task_sequence}")

            outcomes = []
            for task in task_sequence:
                self._log_info(f"eval task: {task}")

                behavior = task["behavior"][0]

                self._log_info(f"eval behavior: {behavior}")

                if behavior == "navigate":
                    success, notes = self._nav_component.navigate_and_wait(
                        x=task["x"][0],
                        y=task["y"][0],
                        check_yaw=task["check_yaw"][0],
                        yaw=task["yaw"][0],
                    )

                    outcomes.append(
                        {"behavior": ("navigate", "str"), "success": (success, "bool")}
                    )

                    if not success:
                        break

                if behavior == "set_labels":
                    self._log_info(f"calling set labels")
                    result = self._set_labels_component.set_labels(
                        task["labels"][0], msg.idx
                    )

                    self._log_info(f"got labels call result : {result}")
                    outcomes.append(result)


                if behavior == "query_vlm":
                    query = task["query"][0]
                    self._log_info(f"calling query_vlm: {query}")

                    result = self._vlm_component.query_vlm(query, msg.idx)
                    self._log_info(f"got response: {result}")
                    outcomes.append(result)

            self._log_info(f"at end of call")

            result_msg = self._build_result_msg(idx=msg.idx, data=outcomes)
            while msg.idx not in self._acked_results:
                self._result_pub.publish(result_msg)
                time.sleep(15)
                self._log_info(f"pub response: {result_msg.result}")


class LabelServiceTranslator:
    def __init__(
        self,
        parent_node: JackalAutonomyManager,
        label_service: str,
    ):
        self._parent_node = parent_node

        label_cbk_group = ReentrantCallbackGroup()
        self._label_client = self._parent_node.create_client(
            SetLabels, label_service, callback_group=label_cbk_group
        )

        self._log_info("waiting for label service")
        self._label_client.wait_for_service()

        self._in_progress = False
        self._result = {}
        self._current_idx = 0

    def _log_info(self, msg):
        self._parent_node.get_logger().info(f"[autonomy manager] [label service] {msg}")

    def set_labels(self, query: str, idx: int) -> Dict[str, Tuple[Any, str]]:
        request = SetLabels.Request()
        request.labels = ascii(query)

        self._current_idx = idx

        # try:
        self._log_info(
            f"[label service translator] formed request for set labels: {request}"
        )
        response_future = self._label_client.call_async(request)

        response_future.add_done_callback(
            lambda f: self._handle_service_response(f, idx)
        )

        self._in_progress = True

        while self._in_progress:
            self._log_info("[nav manager] waiting for goal to complete")
            time.sleep(5)

        return self._result

    def _handle_service_response(self, future: Future, idx: int):
        try:
            response = future.result()
            success = response.success
            self._parent_node.get_logger().info(
                f"[label service translator] service returned {success} for {self._current_idx}"
            )
        except Exception as e:
            self._parent_node.get_logger().error(
                f"Service call failed for {self._current_idx}: {e}"
            )
            success = False

        self._in_progress = False

        self._result = {"behavior": ("set_labels", "str"), "success": (success, "bool")}

        # self._result_dict[self._current_idx] = self._parent_node._build_result_msg(
        #     idx=self._current_idx,
        #     data=[{"behavior": ("set_labels", "str"), "result": (success, "bool")}],
        # )

        self._log_info(f"got set label response")


class JackalNavigationComponent:
    def __init__(
        self,
        parent_node: Node,
        frame_id: str = "map",
        goal_tol: Optional[float] = 2,
        navigation_action_server: Optional[str] = "navigate_to_pose",
        status_topic: Optional[str] = "nav_component/status",
        parameter_server: Optional[str] = "controller_server/set_parameters",
    ):
        self._parent_node = parent_node
        self._frame_id = frame_id

        client_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self._parent_node,
            NavigateToPose,
            navigation_action_server,
            callback_group=client_group,
        )

        self._status_updates = self._parent_node.create_publisher(
            String, status_topic, 10
        )

        param_group = ReentrantCallbackGroup()
        self._param_client = self._parent_node.create_client(
            SetParameters,
            parameter_server,
            callback_group=param_group,
        )

        # Wait for action server to be available
        self._log_info(
            f"Waiting for Nav2 action server on {navigation_action_server}..."
        )
        self._action_client.wait_for_server()
        self._log_info(f"Nav2 action server available on {navigation_action_server}!")

        # Store current goal handle
        self._goal_handle = None
        self._in_progress = False
        self._result_future = None
        self._distance_remaining = np.inf
        self._goal_tol = goal_tol

        self._strict_yaw = False
        self._log_info_idx = 0

    def _log_info(self, msg: str) -> None:
        self._parent_node.get_logger().info(f"[navigation component action] {msg}")

    def navigate_and_wait(
        self,
        x: float,
        y: float,
        yaw: Optional[float] = 0.0,
        timeout_sec: Optional[float | None] = None,
        check_yaw: Optional[bool] = False,
    ) -> Tuple[bool, str]:
        self._in_progress = True
        self._result_future = Future()

        self.send_goal(x, y, yaw, check_yaw=check_yaw)

        while self._in_progress:
            self._parent_node.get_logger().info(
                "[nav manager] waiting for goal to complete"
            )
            time.sleep(5)

        # TODO check this
        # return self._result_future._result, ""
        return True, ""

    def send_goal(
        self,
        x: float,
        y: float,
        yaw: Optional[float] = 0.0,
        check_yaw: Optional[bool] = False,
    ):
        # Create goal message
        goal_msg = NavigateToPose.Goal()

        # Set up the pose
        goal_msg.pose.header.frame_id = self._frame_id
        goal_msg.pose.header.stamp = self._parent_node.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (convert yaw to quaternion)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if check_yaw and not self._strict_yaw:
            self.set_yaw_tolerance(float(1))
        elif not check_yaw and self._strict_yaw:
            self.set_yaw_tolerance(float(3.1))

        self._log_info(f"Sending goal: x={x}, y={y}, yaw={yaw}")

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def set_yaw_tolerance(self, tolerance_radians: float) -> bool:
        self._log_info(f"setting yaw tol: {tolerance_radians}")
        if not self._param_client.wait_for_service(timeout_sec=1.0):
            self._parent_node.get_logger().error("Parameter service not available")
            return False

        # Create parameter
        param = Parameter()
        param.name = "general_goal_checker.yaw_goal_tolerance"
        param.value = ParameterValue()
        # param.value.type = ParameterValue.double_value
        param.value.type = 3
        param.value.double_value = float(tolerance_radians)

        self._log_info(f"formed msg")

        # Create request
        request = SetParameters.Request()
        request.parameters = [param]

        # Call service
        resp = self._param_client.call(request, timeout_sec=2.0)
        self._log_info(f"got resp: {resp}")

        return resp.results[0].successful if resp else False

    def _feedback_callback(self, feedback_msg):
        """Handle feedback from navigation"""
        # self._parent_node.get_logger ().info(f"[nav client] feedback is running")
        feedback = feedback_msg.feedback

        # # Log current pose and navigation info
        current_pose = feedback.current_pose.pose
        self._distance_remaining = feedback.distance_remaining

        # self._parent_node.get_logger ().info(
        status = (
            f"[nav manager] Navigation feedback - "
            f"Distance remaining: {self._distance_remaining:.2f}m, "
            f"Current position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})"
        )
        # )

        if self._log_info_idx % 1000 == 0:
            self._log_info(f"{status}")

        self._log_info_idx += 1
        if self._log_info_idx > 1e6:
            self._log_info_idx = 0

        status_msg = String()
        status_msg.data = ascii(status)
        self._status_updates.publish(status_msg)

    def goal_response_callback(self, future: rclpy.task.Future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._parent_node.get_logger().warn("[nav manager] goal rejected")
            if self._result_future:
                self._result_future.set_result(False)
            return

        self._log_info("goal accepted")

        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: rclpy.task.Future):
        """Handle final result from navigation"""
        result = future.result().result
        status = future.result().status

        success = False
        if status == 4:  # SUCCEEDED
            self._log_info("Navigation succeeded!")
            success = True
        elif status == 5:  # CANCELED
            self._parent_node.get_logger().warn("[nav manager] Navigation was canceled")
        elif status == 6:  # ABORTED
            self._parent_node.get_logger().error("[nav manager] Navigation aborted!")
        else:
            self._parent_node.get_logger().error(
                f"[nav manager] Navigation failed with status: {status}"
            )

        if self._distance_remaining < self._goal_tol:
            success = True

        self._in_progress = False
        self._goal_handle = None

        if self._result_future and not self._result_future.done():
            self._result_future.set_result(success)

    def _cancel_goal(self):
        if self._goal_handle:
            self._parent_node.get_logger().info(
                "[nav manager] Canceling current goal..."
            )
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        self._parent_node.get_logger().info("[nav manager] Goal canceled.")
        if self._result_future and not self._result_future.done():
            self._result_future.set_result(False)


class VLMServiceComponent:
    def __init__(
        self,
        parent_node: JackalAutonomyManager,
        vlm_service: str = "vlm_node/query_scene",
    ):

        self._prarent_node = parent_node

        vlm_cbk_group = ReentrantCallbackGroup()
        self._vlm_client = self._prarent_node.create_client(
            Query, vlm_service, callback_group=vlm_cbk_group
        )

        self._log_info(f"waiting for VLM server...")
        self._vlm_client.wait_for_service()
        self._log_info(f"VLM is initialized")

        self._in_progress = False
        self._result = {}

    def _log_info(self, msg: str) -> None:
        self._prarent_node.get_logger().info(f"[jackal autonomy] [vlm component] {msg}")

    def query_vlm(self, request: str, idx: int) -> None:
        self._log_info(f"got req for {idx}")
        self._query_vlm_async(request, idx)

        self._in_progress = True

        while self._in_progress:
            self._log_info("waiting for vlm response")
            time.sleep(5)


        return self._result

    def _query_vlm_async(self, vlm_request: str, idx: int):
        """Make VLM service call without blocking"""

        request = Query.Request()
        request.query = ascii(vlm_request)

        self._log_info(
            f"[vlm service translator] calling VLM service async for idx {idx} with query: {request.query}"
        )

        future = self._vlm_client.call_async(request)
        future.add_done_callback(lambda f: self._handle_vlm_response(f, idx))

    def _handle_vlm_response(self, future, idx: int):
        """Handle VLM service response and start publishing"""
        self._log_info(f"[vlm service translator] handling vlm resp for {idx}")

        try:
            response = future.result()
            success = response.success
            answer = response.answer
            self._log_info(
                f"[vlm service translator] VLM returned success={success} for {idx} with answer: {answer}"
            )
        except Exception as e:
            self._log_info(
                f"[vlm service translator] VLM service call failed for {idx}: {e}"
            )
            success = False
            answer = "VLM service call failed"

        self._in_progress = False

        self._result =  {
                    "behavior": ("query_vlm", "str"),
                    "success": (success, "bool"),
                    "answer": (answer, "str"),
                }


def main():
    rclpy.init()
    node = JackalAutonomyManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
