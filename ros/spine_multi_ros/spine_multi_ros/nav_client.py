#!/usr/bin/env python3
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationComponent:
    """Navigation component that can be embedded in other ROS2 nodes."""

    def __init__(self, parent_node: Node, frame_id: str = "warthog1/odom"):
        """
        Initialize the navigation component.

        Parameters
        ----------
        parent_node : Node
            The parent ROS2 node to attach this component to
        frame_id : str
            Frame ID for navigation goals
        """
        self._parent_node = parent_node
        self._frame_id = frame_id

        self._navigation_success = False
        self._navigation_complete = False

        # Create action client on parent node
        self._action_client = ActionClient(
            self._parent_node, NavigateToPose, "navigate_to_pose"
        )

        # Store futures for cleanup
        self._send_goal_future = None
        self._get_result_future = None

        # Store callbacks
        self._goal_done_callback = None
        self._goal_feedback_callback = None

        self._parent_node.get_logger().info("Navigation component initialized")

    def _get_goal_msg(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        """Create a navigation goal message."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self._frame_id
        goal_msg.pose.header.stamp = self._parent_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0

        self._parent_node.get_logger().info(f"Goal: {goal_msg.pose.pose.position}")
        return goal_msg

    def send_goal(
        self, x: float, y: float, yaw: float, done_callback=None, feedback_callback=None
    ) -> None:
        """
        Send a navigation goal.

        Parameters
        ----------
        x : float
            X coordinate of goal
        y : float
            Y coordinate of goal
        yaw : float
            Yaw angle of goal
        done_callback : callable, optional
            Callback function called when goal is complete.
            Signature: done_callback(success: bool, result)
        feedback_callback : callable, optional
            Callback function called during navigation.
            Signature: feedback_callback(feedback_msg)
        """
        self._goal_done_callback = done_callback
        self._goal_feedback_callback = feedback_callback

        goal_msg = self._get_goal_msg(x, y, yaw)

        # Check if action server is available with timeout
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self._parent_node.get_logger().error(
                "Navigation action server not available!"
            )
            if self._goal_done_callback:
                self._goal_done_callback(False, None)
            return

        self._parent_node.get_logger().info("[nav client] sending async goal")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cbk
        )
        self._parent_node.get_logger().info("[nav client] sent async goal")

        self._send_goal_future.add_done_callback(self._goal_response_cbk)

    def navigate_and_wait(
        self, x: float, y: float, yaw: float, timeout_sec: float = 60.0
    ):
        """Navigate and wait for completion with timeout."""
        self._navigation_complete = False
        self._navigation_success = False
        self._parent_node.get_logger().info(f"[nav client] calling goal")

        def done_callback(success, result):
            self._navigation_complete = True
            self._navigation_success = success

        self.send_goal(x, y, yaw, done_callback=done_callback)

        # Add timeout to prevent infinite hanging
        start_time = self._parent_node.get_clock().now()
        timeout_duration = rclpy.duration.Duration(seconds=timeout_sec)

        while not self._navigation_complete and rclpy.ok():
            rclpy.spin_once(self._parent_node, timeout_sec=0.1)

            # Check for timeout
            current_time = self._parent_node.get_clock().now()
            if (current_time - start_time) > timeout_duration:
                self._parent_node.get_logger().error(
                    f"Navigation timed out after {timeout_sec} seconds"
                )
                self._navigation_complete = True
                self._navigation_success = False
                break

        self._parent_node.get_logger().info(
            f"[nav client] done with goal, success: {self._navigation_success}"
        )
        return self._navigation_success

    def check_completion(self):
        """Check if navigation is complete."""
        if self._navigation_complete:
            if self._navigation_success:
                self._parent_node.get_logger().info(
                    "Navigation completed successfully!"
                )
            else:
                self._parent_node.get_logger().error("Navigation failed!")

    def _goal_response_cbk(self, future) -> None:
        """Handle goal response."""

        self._parent_node.get_logger().info("[nav client] in goal resp callback")

        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._parent_node.get_logger().error(
                    "Goal rejected by navigation server"
                )
                # CRITICAL FIX: Set completion flags when goal is rejected
                if self._goal_done_callback:
                    self._goal_done_callback(False, None)
                return

            self._parent_node.get_logger().info("[nav client] Goal accepted")

            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self._get_result_cbk)

        except Exception as e:
            self._parent_node.get_logger().error(
                f"Error in goal response callback: {e}"
            )
            if self._goal_done_callback:
                self._goal_done_callback(False, None)

    def _get_result_cbk(self, future) -> None:
        """Handle navigation result."""
        try:
            result = future.result().result
            status = future.result().status
            success = status == GoalStatus.STATUS_SUCCEEDED

            self._parent_node.get_logger().info(
                f"Navigation result - Status: {status}, Success: {success}"
            )

            # Call user-provided callback if set
            if self._goal_done_callback:
                self._goal_done_callback(success, result)

        except Exception as e:
            self._parent_node.get_logger().error(f"Error in result callback: {e}")
            if self._goal_done_callback:
                self._goal_done_callback(False, None)

    def _feedback_cbk(self, goal_handle, feedback_msg) -> None:
        """Handle navigation feedback."""
        try:
            feedback = feedback_msg.feedback
            self._parent_node.get_logger().info(
                f"[nav client] Distance remaining: {feedback.distance_remaining:.2f}m"
            )

            # Call user-provided feedback callback if set
            if self._goal_feedback_callback:
                self._goal_feedback_callback(feedback)

        except Exception as e:
            self._parent_node.get_logger().error(f"Error in feedback callback: {e}")

    def cancel_navigation(self):
        """Cancel current navigation goal."""
        if self._get_result_future and not self._get_result_future.done():
            # You would need to store the goal_handle to cancel properly
            self._parent_node.get_logger().info("Cancelling navigation goal")
            self._navigation_complete = True
            self._navigation_success = False

    def destroy(self):
        """Clean up resources."""
        # Cancel any pending goals
        if self._send_goal_future and not self._send_goal_future.done():
            self._send_goal_future.cancel()
        if self._get_result_future and not self._get_result_future.done():
            self._get_result_future.cancel()


# class NavigationComponent:
#     """Navigation component that can be embedded in other ROS2 nodes."""

#     def __init__(self, parent_node: Node, frame_id: str = "warthog1/odom"):
#         """
#         Initialize the navigation component.

#         Parameters
#         ----------
#         parent_node : Node
#             The parent ROS2 node to attach this component to
#         frame_id : str
#             Frame ID for navigation goals
#         """
#         self._parent_node = parent_node
#         self._frame_id = frame_id

#         self._navigation_success = False
#         self._navigation_complete = False

#         # Create action client on parent node
#         self._action_client = ActionClient(
#             self._parent_node, NavigateToPose, "navigate_to_pose"
#         )

#         # Store futures for cleanup
#         self._send_goal_future = None
#         self._get_result_future = None

#         self._parent_node.get_logger().info("Navigation component initialized")

#     def _get_goal_msg(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
#         """Create a navigation goal message."""
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = self._frame_id
#         goal_msg.pose.header.stamp = self._parent_node.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = float(x)
#         goal_msg.pose.pose.position.y = float(y)
#         goal_msg.pose.pose.position.z = 0.0
#         goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
#         goal_msg.pose.pose.orientation.w = math.cos(
#             yaw / 2.0
#         )  # Fixed: was using x instead of w
#         goal_msg.pose.pose.orientation.x = 0.0
#         goal_msg.pose.pose.orientation.y = 0.0

#         self._parent_node.get_logger().info(f"Goal: {goal_msg.pose.pose.position}")
#         return goal_msg

#     def send_goal(
#         self, x: float, y: float, yaw: float, done_callback=None, feedback_callback=None
#     ) -> None:
#         """
#         Send a navigation goal.

#         Parameters
#         ----------
#         x : float
#             X coordinate of goal
#         y : float
#             Y coordinate of goal
#         yaw : float
#             Yaw angle of goal
#         done_callback : callable, optional
#             Callback function called when goal is complete.
#             Signature: done_callback(success: bool, result)
#         feedback_callback : callable, optional
#             Callback function called during navigation.
#             Signature: feedback_callback(feedback_msg)
#         """
#         self._goal_done_callback = done_callback
#         self._goal_feedback_callback = feedback_callback

#         goal_msg = self._get_goal_msg(x, y, yaw)
#         self._action_client.wait_for_server()
#         self._send_goal_future = self._action_client.send_goal_async(
#             goal_msg, feedback_callback=self._feedback_cbk
#         )
#         self._send_goal_future.add_done_callback(self._goal_response_cbk)

#     def navigate_and_wait(self, x: float, y: float, yaw: float):
#         """Navigate and wait for completion."""
#         self._navigation_complete = False
#         self._parent_node.get_logger().info(f"[nav client] calling  goal")

#         def done_callback(success, result):
#             self._navigation_complete = True
#             self._navigation_success = success

#         self.send_goal(x, y, yaw, done_callback=done_callback)

#         # check status is needed
#         # self.check_timer = self.create_timer(0.1, self.check_completion)

#         while not self._navigation_complete and rclpy.ok():
#             rclpy.spin_once(self._parent_node, timeout_sec=0.1)
#             # self._parent_node.get_logger().info(f"[nav client] still in goal")

#         self._parent_node.get_logger().info(f"[nav client] done with goal")

#         return self._navigation_success

#     def check_completion(self):
#         """Check if navigation is complete."""
#         if self._navigation_complete:
#             # self.destroy_timer(self.check_timer)

#             if self._navigation_success:
#                 self._parent_node.get_logger().info(
#                     "Navigation completed successfully!"
#                 )
#             else:
#                 self._parent_node.get_logger().error("Navigation failed!")

#     def _goal_response_cbk(self, future) -> None:
#         """Handle goal response."""
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self._parent_node.get_logger().info("Goal rejected")
#             return

#         self._parent_node.get_logger().info("Goal accepted")
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self._get_result_cbk)

#     def _get_result_cbk(self, future) -> None:
#         """Handle navigation result."""
#         result = future.result().result
#         success = future.result().status == GoalStatus.STATUS_SUCCEEDED

#         self._parent_node.get_logger().info(
#             f"Navigation result: {result}, Success: {success}"
#         )

#         # Call user-provided callback if set
#         if self._goal_done_callback:
#             self._goal_done_callback(success, result)

#     def _feedback_cbk(self, feedback_msg) -> None:
#         """Handle navigation feedback."""
#         feedback = feedback_msg.feedback
#         self._parent_node.get_logger().info(
#             f"[nav client] Distance remaining: {feedback.distance_remaining}"
#         )

#         # Call user-provided feedback callback if set
#         if self._goal_feedback_callback:
#             self._goal_feedback_callback(feedback_msg.feedback)

#     def destroy(self):
#         """Clean up resources."""
#         # Cancel any pending goals
#         if self._send_goal_future and not self._send_goal_future.done():
#             self._send_goal_future.cancel()
#         if self._get_result_future and not self._get_result_future.done():
#             self._get_result_future.cancel()


# # Original standalone node version (still available)
# class NavigationClient(Node):
#     def __init__(self):
#         super().__init__("navigation_client")

#         # Use the component internally
#         self.nav_component = NavigationComponent(self, "warthog1/odom")

#         self.get_logger().info("NavigationClient node initialized")

#     def send_goal(self, x: float, y: float, yaw: float) -> None:
#         """Send a navigation goal."""
#         self.nav_component.send_goal(x, y, yaw)


# # Example usage: Embedding in another node
class MyMainNode(Node):
    def __init__(self):
        super().__init__("my_main_node")

        self.navigation_complete = False
        self.navigation_success = False

        # Embed navigation component
        self.navigation = NavigationComponent(
            parent_node=self, frame_id="warthog1/odom"  # or whatever frame you need
        )

        # Your other node functionality here
        self.get_logger().info("Main node with navigation component initialized")

    def navigate_(self, x: float, y: float, yaw: float):
        """Navigate to a specific position."""
        self.navigation.send_goal(x, y, yaw)

    def navigate_blocking(self, x: float, y: float, yaw: float):
        """Navigate to a specific position."""
        return self.navigation.navigate_and_wait(x, y, yaw)

    def destroy_node(self):
        """Clean up when destroying the node."""
        self.navigation.destroy()
        super().destroy_node()


def main():
    rclpy.init()

    # Example 2: Use component in another node
    my_node = MyMainNode()
    result = my_node.navigate_blocking(1, 1, 0)

    print(result)

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
