#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from teaming_msgs.srv import TransformGoal  # update to your package name


def odom_to_SE2(odom: Odometry, ned: bool = False) -> np.ndarray:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]

    if ned:
        x = p.y
        y = p.x
        yaw = np.pi / 2 - yaw
    else:
        x, y = p.x, p.y

    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def SE2_inv(T: np.ndarray) -> np.ndarray:
    R = T[:2, :2]
    t = T[:2, 2]
    T_inv = np.eye(3)
    T_inv[:2, :2] = R.T
    T_inv[:2, 2] = -R.T @ t
    return T_inv


class GoalFrameConverter(Node):
    def __init__(self):
        super().__init__("goal_frame_converter")

        self.T_global = None
        self.T_local = None
        self.stamp_global = None
        self.stamp_local = None

        self.create_subscription(Odometry, "/odom/global", self.global_odom_cb, 10)
        self.create_subscription(Odometry, "/odom/local", self.local_odom_cb, 10)

        self.srv = self.create_service(
            TransformGoal, "transform_goal_to_local", self.handle_transform_goal
        )
        self.get_logger().info("TransformGoal service ready.")

    def global_odom_cb(self, msg: Odometry):
        self.T_global = odom_to_SE2(msg)
        self.stamp_global = rclpy.time.Time.from_msg(msg.header.stamp)

    def local_odom_cb(self, msg: Odometry):
        self.T_local = odom_to_SE2(msg)
        self.stamp_local = rclpy.time.Time.from_msg(msg.header.stamp)

    def handle_transform_goal(
        self, request: TransformGoal.Request, response: TransformGoal.Response
    ) -> TransformGoal.Response:

        # Guard: both odom sources must be available
        if self.T_global is None or self.T_local is None:
            response.success = False
            response.message = "Odometry not yet received on one or both topics."
            return response

        # Guard: check timestamp sync
        dt = abs((self.stamp_global - self.stamp_local).nanoseconds) * 1e-9
        if dt > 0.1:
            response.success = False
            response.message = f"Odometry sources out of sync by {dt:.3f}s."
            return response

        transformed = self._transform_goal(request.goal_global)

        x = transformed.pose.position.x
        y = transformed.pose.position.y

        # TODO quick fix to convert to robot local ENU frame.
        # This is a frame that must be worked out
        transformed.pose.position.x = y
        transformed.pose.position.y = -x

        if transformed is None:
            response.success = False
            response.message = "Transform computation failed."
            return response

        response.goal_local = transformed
        response.success = True
        response.message = ""
        return response

    def _transform_goal(self, goal: PoseStamped) -> PoseStamped | None:
        try:
            T_local_from_global = self.T_local @ SE2_inv(self.T_global)

            p = np.array([goal.pose.position.x, goal.pose.position.y, 1.0])
            p_local = T_local_from_global @ p

            q = goal.pose.orientation
            goal_yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
            delta_yaw = np.arctan2(T_local_from_global[1, 0], T_local_from_global[0, 0])
            new_q = Rotation.from_euler("z", goal_yaw + delta_yaw).as_quat()

            result = PoseStamped()
            result.header.stamp = self.get_clock().now().to_msg()
            result.header.frame_id = "odom"
            result.pose.position.x = float(p_local[0])
            result.pose.position.y = float(p_local[1])
            result.pose.position.z = goal.pose.position.z
            result.pose.orientation.x = new_q[0]
            result.pose.orientation.y = new_q[1]
            result.pose.orientation.z = new_q[2]
            result.pose.orientation.w = new_q[3]
            return result

        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = GoalFrameConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
