#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class CmdVelToJoy(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_joy")

        # Params (easy to tune later)
        self.declare_parameter("max_linear", 3.0)
        self.declare_parameter("max_angular", 3.0)

        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value

        # ---- State ----
        self.block_autonomy = False  # True when axes[4] == -1.0

        self.cmd_sub = self.create_subscription(
            # Twist,
            TwistStamped,
            # '/cmd_vel',
            "/cmd_vel_nav",
            self.cmd_cb,
            # 10
            20,
        )

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_cb, 20)

        self.joy_pub = self.create_publisher(Joy, "/joy", 1)
        self.last_cmd = TwistStamped()
        self.timer = self.create_timer(0.02, self.publish_joy)  # 50 Hz

        self.get_logger().info("CmdVel → Joy bridge started")

    def clamp(self, val, min_val=-1.0, max_val=1.0):
        return max(min(val, max_val), min_val)

    # -------- Manual joystick callback --------
    def joy_cb(self, msg: Joy):
        if len(msg.axes) > 4:
            # Block autonomy when trigger held
            self.block_autonomy = msg.axes[4] == -1.0

    def cmd_cb(self, msg: TwistStamped):
        self.last_cmd = msg

    def publish_joy(self):
        if self.block_autonomy:
            return

        msg = self.last_cmd

        joy = Joy()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 8

        # joy.axes[3] = self.clamp((msg.twist.linear.x * 3.0) / self.max_linear)
        joy.axes[3] = self.clamp((msg.twist.linear.x * 6.0) / self.max_linear)
        # joy.axes[2] = self.clamp((msg.twist.angular.z * 20.0) / self.max_angular)
        # joy.axes[2] = self.clamp((msg.twist.angular.z * 30.0) / self.max_angular)
        joy.axes[2] = self.clamp((msg.twist.angular.z * 150.0) / self.max_angular)

        # self.get_logger().info(f'Publishing Joy: axes[3]={joy.axes[3]:.2f}, axes[2]={joy.axes[2]:.2f}')

        self.joy_pub.publish(joy)


def main():
    rclpy.init()
    node = CmdVelToJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
