#!/usr/bin/env python3

import math
import sys
import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import yaml
from typing import List, Tuple


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)

def load_waypoints_from_yaml(path: str) -> Tuple[str, List[Tuple[float, float, float]]]:
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    frame_id = data.get('frame_id', 'map')
    raw_wps = data.get('waypoints', [])

    waypoints: List[Tuple[float, float, float]] = []
    for i, wp in enumerate(raw_wps):
        if isinstance(wp, dict):
            x = float(wp['x'])
            y = float(wp['y'])
            yaw = float(wp.get('yaw', 0.0))
        elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
            x = float(wp[0])
            y = float(wp[1])
            yaw = float(wp[2]) if len(wp) > 2 else 0.0
        else:
            raise ValueError(f'Invalid waypoint at index {i}: {wp}')
        waypoints.append((x, y, yaw))

    return frame_id, waypoints



class Nav2GotoPoseClient(Node):
    def __init__(self, action_name='/navigate_to_pose'):
        super().__init__('nav2_goto_pose_client')
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._goal_handle = None
        self._cancel_in_progress = False
    
    def run_waypoints(self, waypoints: List[Tuple[float, float, float]], frame_id: str = 'map'):
        for i, (x, y, yaw) in enumerate(waypoints):
            self.get_logger().info(f'=== Waypoint {i+1}/{len(waypoints)} ===')
            ok = self.send_goal_and_wait(x, y, yaw, frame_id)
            if not ok:
                self.get_logger().error(
                    f'Waypoint {i+1} failed, stopping waypoint execution.')
                break

    def build_goal(self, x: float, y: float, yaw: float, frame_id: str = 'map'):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg.pose = pose
        return goal_msg

    def send_goal_and_wait(self, x: float, y: float, yaw: float, frame_id: str = 'map'):
        goal_msg = self.build_goal(x, y, yaw, frame_id)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._client.wait_for_server()

        self.get_logger().info(
            f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad, frame="{frame_id}"'
        )

        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        # Wait for goal handle
        rclpy.spin_until_future_complete(self, send_goal_future)
        self._goal_handle = send_goal_future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result, but we will be able to cancel from SIGINT
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if self._cancel_in_progress:
            self.get_logger().info('Goal was cancelled, exiting.')
            return False

        result = result_future.result()
        status = result.status
        if status == 4:
            self.get_logger().info('Navigation succeeded!')
            return True
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            return False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        try:
            self.get_logger().info(
                f'Feedback: dist_remaining={feedback.distance_remaining:.2f} m, '
                f'nav_time={feedback.navigation_time.sec}s', throttle_duration_sec=5.0
            )
        except Exception:
            self.get_logger().info(f'Feedback received: {feedback}')

    def cancel_goal_blocking(self):
        """Request cancel and block until the server answers."""
        if self._goal_handle is None:
            self.get_logger().warn('No goal handle to cancel.')
            return

        if self._cancel_in_progress:
            return

        self._cancel_in_progress = True
        self.get_logger().warn('Ctrl+C detected, cancelling goal...')

        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

        cancel_response = cancel_future.result()
        self.get_logger().info(f'Cancel response: {cancel_response}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: goto_pose_client.py waypoint_file [action_name]')
        sys.exit(1)

    yaml_path = sys.argv[1]
    action_name = sys.argv[2] if len(sys.argv) > 2 else '/navigate_to_pose'

    frame_id, waypoints = load_waypoints_from_yaml(yaml_path)
    print(f'Loaded {len(waypoints)} waypoints from "{yaml_path}" with frame_id="{frame_id}"')

    node = Nav2GotoPoseClient(action_name=action_name)

    # Custom SIGINT handler to cancel the goal
    def sigint_handler(signum, frame):
        # Do *not* call rclpy.shutdown() directly; cancel first.
        node.cancel_goal_blocking()
        node.destroy_node()
        rclpy.shutdown()
        # Exit the process immediately after shutdown
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    # This call blocks until either:
    #  - the goal finishes, or
    #  - we hit Ctrl+C and the SIGINT handler cancels and shuts down.
    node.run_waypoints(waypoints, frame_id=frame_id)

    # Normal shutdown path (no Ctrl+C)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
