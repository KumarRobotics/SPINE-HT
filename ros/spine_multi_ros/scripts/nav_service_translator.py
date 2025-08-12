#!/usr/bin/env python3

import math
from enum import Enum
from typing import Dict

import numpy as np
import rclpy
import rclpy.task
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from teaming_msgs.msg import GoalRequest, GoalStatus
from std_msgs.msg import Int16
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from dataclasses import dataclass


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


class NavServiceTranslator(Node):
    def __init__(self):
        super().__init__("nav_service_translator")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._goal_dict: Dict[str, NavGoal] = {}
        self._strict_yaw = False

        self.declare_parameters(
            namespace="",
            parameters=[
                ("navigation_action_server", "navigate_to_pose"),
                ("navigation_request", "navigation_request"),
                (f"robot_name", ""),
                ("frame_id", "map"),
                ("parameter_server", "controller_server/set_parameters"),
            ],
        )

        navigation_action_server = (
            self.get_parameter("navigation_action_server")
            .get_parameter_value()
            .string_value
        )
        navigation_request = (
            self.get_parameter("navigation_request").get_parameter_value().string_value
        )

        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        parameter_server = (
            self.get_parameter("parameter_server").get_parameter_value().string_value
        )

        client_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            navigation_action_server,
            callback_group=client_group,
        )

        sub_cbk_group = ReentrantCallbackGroup()
        self._nav_req_sub = self.create_subscription(
            GoalRequest,
            "navigation_request",
            self._req_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        self._ack_sub = self.create_subscription(
            Int16,
            "navigation_ack",
            self._ack_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        self._status_update_pub = self.create_publisher(
            GoalStatus, "navigation_status", 10
        )
        param_group = ReentrantCallbackGroup()
        self._param_client = self.create_client(
            SetParameters,
            parameter_server,
            callback_group=param_group,
        )

        self._goal_handle = None
        self._in_progress = False
        self._result_future = None
        self._goal_tol = 2.0  # TODO make var
        self._distance_remaining = np.inf
        self._current_goal_idx = -1
        self._current_goal_msg = self._get_goal_msg(idx=-1, status=-1)

        timer_cbk_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            5.0, self._timer_cbk, callback_group=timer_cbk_group
        )

        self.get_logger().info("[nav service translator] init")

    def _ack_cbk(self, ack_msg: Int16) -> None:
        ack_idx = ack_msg.data
        self.get_logger().info(f"[nav service translator] got ack for {ack_idx}")

        if ack_idx in self._goal_dict:
            self._goal_dict[ack_idx].ack = True
            self.get_logger().info(
                f"[nav service translator] registered req {ack_idx} as ack"
            )

    def _get_goal_msg(self, idx: int, status: int) -> GoalStatus:
        goal_msg = GoalStatus()
        goal_msg.idx = idx
        goal_msg.status = status
        return goal_msg

    def _timer_cbk(self) -> None:
        if self._current_goal_idx not in self._goal_dict:
            return

        nav_goal = self._goal_dict[self._current_goal_idx]

        if not nav_goal.ack:
            self.get_logger().info(
                f"sending update for idx: {self._current_goal_msg.idx}"
            )
            self._status_update_pub.publish(self._current_goal_msg)

    def _req_cbk(self, goal_req: GoalRequest) -> None:
        # goal has already been called
        if goal_req.idx in self._goal_dict:
            return

        x = goal_req.x
        y = goal_req.y
        yaw = goal_req.yaw

        # Create goal message
        goal_msg = NavigateToPose.Goal()

        # Set up the pose
        goal_msg.pose.header.frame_id = self._frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (convert yaw to quaternion)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if goal_req.check_yaw and not self._strict_yaw:
            self.set_yaw_tolerance(float(0.5))
        elif not goal_req.check_yaw and self._strict_yaw:
            self.set_yaw_tolerance(float(3.1))

        self.get_logger().info(
            f"[nav service translator] Sending goal: x={x}, y={y}, yaw={yaw}"
        )

        nav_goal = NavGoal(
            idx=goal_req.idx,
            x=x,
            y=y,
            yaw=yaw,
            status=GoalStatusEnum.IN_PROGRESS.value,
            ack=False,
        )
        self._goal_dict[goal_req.idx] = nav_goal
        self._current_goal_idx = goal_req.idx

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def set_yaw_tolerance(self, tolerance_radians: float) -> None:
        self.get_logger().info(f"setting yaw tol: {tolerance_radians}")
        if not self._param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Parameter service not available")
            return False

        # Create parameter
        param = Parameter()
        param.name = "general_goal_checker.yaw_goal_tolerance"
        param.value = ParameterValue()
        # param.value.type = ParameterValue.double_value
        param.value.type = 3
        param.value.double_value = float(tolerance_radians)

        self.get_logger().info(f"formed msg")

        # Create request
        request = SetParameters.Request()
        request.parameters = [param]

        # Call service
        resp = self._param_client.call(request, timeout_sec=2.0)
        self.get_logger().info(f"got resp: {resp}")

    def _feedback_callback(self, feedback_msg) -> None:
        """Handle feedback from navigation"""
        # self._parent_node.get_logger ().info(f"[nav client] feedback is running")
        feedback = feedback_msg.feedback

        #  Log current pose and navigation info
        current_pose = feedback.current_pose.pose
        self._distance_remaining = feedback.distance_remaining

        # self.get_logger().info(
        #    f"[nav service translator] Got feedback for idx: {self._current_goal_idx}"
        # )

        self._current_goal_msg = self._get_goal_msg(
            idx=self._current_goal_idx,
            status=self._goal_dict[self._current_goal_idx].status,
        )

    def goal_response_callback(self, future: rclpy.task.Future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("[nav service translator] goal rejected")
            if self._result_future:
                self._result_future.set_result(False)
            return

        self.get_logger().info("[nav service translator] goal accepted")

        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: rclpy.task.Future):
        """Handle final result from navigation"""
        result = future.result().result
        status = future.result().status

        success = False
        if status == 4:  # SUCCEEDED
            self.get_logger().info("[nav service translator] Navigation succeeded!")
            success = True
        elif status == 5:  # CANCELED
            self.get_logger().warn("[nav service translator] Navigation was canceled")
        elif status == 6:  # ABORTED
            self.get_logger().error("[nav service translator] Navigation aborted!")
        else:
            self.get_logger().error(
                f"[nav service translator] Navigation failed with status: {status}"
            )

        if self._distance_remaining < self._goal_tol:
            success = True

        self._in_progress = False
        self._goal_handle = None

        if self._result_future and not self._result_future.done():
            self._result_future.set_result(success)

        status = (
            GoalStatusEnum.SUCCESS.value if success else GoalStatusEnum.FAILED.value
        )
        self._current_goal_msg = self._get_goal_msg(
            idx=self._current_goal_idx, status=status
        )


def main():
    rclpy.init()
    node = NavServiceTranslator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
