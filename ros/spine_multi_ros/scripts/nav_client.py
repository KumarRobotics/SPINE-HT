#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from spine_multi_ros.spine_multi_ros.nav_manager import NavigationComponent
from teaming_msgs.msg import Nav


# Example usage: Embedding in another node
class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_manager")

        self.navigation_complete = False
        self.navigation_success = False

        # Embed navigation component
        self.navigation = NavigationComponent(
            parent_node=self, frame_id="warthog1/odom"  # or whatever frame you need
        )

        self.navigation_srv = self.create_service(
            Nav, "~/nav_request", self._nav_request
        )

        # Your other node functionality here
        self.get_logger().info("Main node with navigation component initialized")

    def _nav_request(self, req: Nav.Request, resp: Nav.Request):

        self.navigation.navigate_and_wait(req.x, req.y, req.yaw)

        resp.success = True

        self.get_logger().info("[nav client] responding to service ")
        return resp

    def destroy_node(self):
        """Clean up when destroying the node."""
        self.navigation.destroy()
        super().destroy_node()


def main():
    rclpy.init()

    my_node = NavigationNode()

    executor = MultiThreadedExecutor()
    executor.add_node(my_node)

    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
