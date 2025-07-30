#!/usr/bin/env python3

import math
import time
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
from std_msgs.msg import String


class NavigationComponent:
    def __init__(
        self,
        parent_node: Node,
        frame_id: str = "warthog1/odom",
        goal_tol: Optional[float] = 2,
    ):
        self._parent_node = parent_node
        self._frame_id = frame_id

        client_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self._parent_node,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=client_group,
        )

        self._status_updates = self._parent_node.create_publisher(
            String, "~/nav_component/status", 10
        )

        param_group = ReentrantCallbackGroup()
        self._param_client = self._parent_node.create_client(
            SetParameters,
            "/controller_server/set_parameters",
            callback_group=param_group,
        )

        # Wait for action server to be available
        self._parent_node.get_logger().info("Waiting for Nav2 action server...")
        self._action_client.wait_for_server()
        self._parent_node.get_logger().info("Nav2 action server available!")

        # Store current goal handle
        self._goal_handle = None
        self._in_progress = False
        self._result_future = None
        self._goal_tol = goal_tol
        self._distance_remaining = np.inf

        self._strict_yaw = False

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
            time.sleep(0.1)

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

        self._parent_node.get_logger().info(
            f"[nav manager] Sending goal: x={x}, y={y}, yaw={yaw}"
        )

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def set_yaw_tolerance(self, tolerance_radians: float) -> bool:
        self._parent_node.get_logger().info(f"setting yaw tol: {tolerance_radians}")
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
        
        self._parent_node.get_logger().info(f"formed msg")

        # Create request
        request = SetParameters.Request()
        request.parameters = [param]

        # Call service
        resp = self._param_client.call(request, timeout_sec=2.0)
        self._parent_node.get_logger().info(f"got resp: {resp}")
 
        return resp.results[0].successful if resp else False


        future = self._param_client.call_async(request)

        rclpy.spin_until_future_complete(self._parent_node, future)

        result = future.result().results[0].successful
        self._parent_node.get_logger(f"setting yaw result: {result}")
        return result

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

        self._parent_node.get_logger().info("[nav manager] goal accepted")

        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: rclpy.task.Future):
        """Handle final result from navigation"""
        result = future.result().result
        status = future.result().status

        success = False
        if status == 4:  # SUCCEEDED
            self._parent_node.get_logger().info("[nav manager] Navigation succeeded!")
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
