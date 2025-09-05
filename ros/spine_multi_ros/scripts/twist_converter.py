#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistConverter(Node):
    def __init__(self):
        super().__init__("twist_converter")

        # Get namespace parameter
        self.declare_parameter("namespace", "warthog1")
        self.declare_parameter("publish_stamped", True)
        self.declare_parameter("flip_x", False)
        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_z", 1.0)

        namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self._flip_x = self.get_parameter("flip_x").get_parameter_value().bool_value
        self._scale_x = self.get_parameter("scale_x").get_parameter_value().double_value
        self._scale_z = self.get_parameter("scale_z").get_parameter_value().double_value
        self._publish_stamped = self.get_parameter("publish_stamped").get_parameter_value().bool_value


        # Subscribe to TwistStamped
        self.subscription = self.create_subscription(
            TwistStamped, f"cmd_vel_nav", self.twist_stamped_callback, 10
        )


        self.get_logger().info(f"pub stamped: {self._publish_stamped}")

        # Publish Twist
        pub_type = TwistStamped if self._publish_stamped else Twist
        self.publisher = self.create_publisher(
            TwistStamped, f"cmd_vel", 10  # Will be remapped to /warthog1/cmd_vel
        )

        self.get_logger().info(
            f"Converting /{namespace}/cmd_vel (TwistStamped) to Twist"
        )

    def twist_stamped_callback(self, msg):

        if self._publish_stamped:
            if self._flip_x:
                msg.twist.linear.x *= -1

            msg.twist.linear.x *= self._scale_x
            msg.twist.angular.z *= self._scale_z

            self.publisher.publish(msg)

        else:
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
