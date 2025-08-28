#!/usr/bin/env python3

import math
import time
from abc import ABC
from concurrent.futures import Future
from typing import Optional

import numpy as np
import rclpy
import rclpy.task
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int16, String
from teaming_msgs.msg import GoalRequest, GoalStatus


class NavigationComponent(ABC):
    def __init__(self, parent_node: Node) -> None:
        pass

    def navigate_and_wait(
        self,
        x: float,
        y: float,
        yaw: Optional[float] = 0.0,
        timeout_sec: Optional[float | None] = None,
        check_yaw: Optional[bool] = False,
    ) -> bool:
        pass


class NavigationComponentTopic(NavigationComponent):
    def __init__(
        self,
        parent_node: Node,
        robot_name: str,
        navigation_request: str = "navigation_request",
        navigation_status: str = "navigation_status",
        navigation_ack: str = "navigation_ack",
        subscription_prefix: str = ""
    ):
        self._parent_node = parent_node
        self._robot_name = robot_name
        self._in_progress = False
        self._goal_success = False
        self._current_goal_idx = -1
        self._goal_msg_recv = False

        self._logged_goal = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        self._goal_req_pub = self._parent_node.create_publisher(
            GoalRequest,
            navigation_request,
            qos_profile,
        )

        sub_cbk_group = ReentrantCallbackGroup()
        self._goal_status_sub = self._parent_node.create_subscription(
            GoalStatus,
            subscription_prefix + navigation_status,
            self._goal_status_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        self._ack_pub = self._parent_node.create_publisher(
            Int16, navigation_ack, qos_profile
        )

        self._log_info(f"init")

    def _log_info(self, msg: str) -> None:
        self._parent_node.get_logger().info(
            f"[nav manager topic] [{self._robot_name}] {msg}"
        )

    def _goal_status_cbk(self, goal_status: GoalStatus) -> None:

        self._log_info(f"got goal status msg {goal_status}")

        if goal_status.idx != self._current_goal_idx:
            return

        self._goal_msg_recv = True

        if goal_status.status == 0:
            self._in_progress = True
        elif goal_status.status == 1:
            self._in_progress = False
            self._goal_success = True
        elif goal_status.status == 2:
            self._in_progress = False
            self._goal_success = False

        # send ack that we got goal done msg
        if self._in_progress == False:
            ack_msg = Int16()
            ack_msg.data = self._current_goal_idx
            self._ack_pub.publish(ack_msg)
            # self._log_info(
            #     f"pub ack msg"
            # )

    def navigate_and_wait(
        self,
        x: float,
        y: float,
        yaw: Optional[float] = 0.0,
        timeout_sec: Optional[float | None] = None,
        check_yaw: Optional[bool] = False,
    ) -> bool:
        self._in_progress = True
        self._logged_goal = False

        self._current_goal_idx = self._current_goal_idx + 1
        self._in_progress = True
        self._goal_msg_recv = False

        goal_msg = self._get_goal_req(x, y, yaw, self._current_goal_idx, check_yaw)

        self._log_info(
            f"constructed goal msg {self._current_goal_idx} of {x} {y} {yaw}"
        )

        while not self._goal_msg_recv:
            # if not self._logged_goal:
            self._log_info(
                f"[{self._robot_name}] sending goal msg: {self._current_goal_idx} of {x} {y} "
            )
                # self._logged_goal = True

            self._goal_req_pub.publish(goal_msg)
            time.sleep(5)

        self._log_info(
            f"waiting for goal response: {self._current_goal_idx} of {x} {y}"
        )

        while self._in_progress:
            time.sleep(5)

        self._log_info(f"goal done with succes: {self._goal_success}")

        return self._goal_success

    def _get_goal_req(
        self, x: float, y: float, yaw: float, idx: int, check_yaw: bool
    ) -> GoalRequest:
        msg = GoalRequest()
        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(yaw)
        msg.idx = idx
        msg.check_yaw = check_yaw

        return msg


class NavigationComponentAction:
    def __init__(
        self,
        parent_node: Node,
        frame_id: str = "warthog1/odom",
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
        self._goal_tol = goal_tol
        self._distance_remaining = np.inf

        self._strict_yaw = False

    def _log_info(self, msg: str) -> None:
        self._param_client.get_logger().info(f"[navigation component action] {msg}")

    def navigate_and_wait(
        self,
        x: float,
        y: float,
        yaw: Optional[float] = 0.0,
        timeout_sec: Optional[float | None] = None,
        check_yaw: Optional[bool] = False,
    ) -> bool:
        self._in_progress = True
        self._result_future = Future()

        self.send_goal(x, y, yaw, check_yaw=check_yaw)

        while self._in_progress:
            # self._parent_node.get_logger().info("[nav manager] waiting for goal to complete")
            time.sleep(5)

        return True

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
            self.set_yaw_tolerance(float(0.2))
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
