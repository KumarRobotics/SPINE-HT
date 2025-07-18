#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistConverter(Node):
    def __init__(self):
        super().__init__("twist_converter")

        # Get namespace parameter
        self.declare_parameter("namespace", "warthog1")
        namespace = self.get_parameter("namespace").get_parameter_value().string_value

        # Subscribe to TwistStamped
        self.subscription = self.create_subscription(
            TwistStamped, f"/cmd_vel_nav", self.twist_stamped_callback, 10
        )

        # Publish Twist
        self.publisher = self.create_publisher(
            Twist, f"/cmd_vel", 10  # Will be remapped to /warthog1/cmd_vel
        )

        self.get_logger().info(
            f"Converting /{namespace}/cmd_vel (TwistStamped) to Twist"
        )

    def twist_stamped_callback(self, msg):
        # Extract the twist part from TwistStamped
        twist_msg = Twist()
        twist_msg.linear = msg.twist.linear
        twist_msg.angular = msg.twist.angular
        self.publisher.publish(twist_msg)


def main():
    rclpy.init()
    node = TwistConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
