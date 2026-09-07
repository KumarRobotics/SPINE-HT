#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from teaming_msgs.srv import TransformGoal

from spine_ht.spine.mapping.graph_util import GraphHandler
from spine_ht.spine.viz.viz_ros import GraphVisualizerComponent


class GraphVisualizerNode(Node):
    def __init__(self):
        super().__init__("graph_publisher")

        self._goal_transformer_client = self.create_client(
            TransformGoal, "/transform_goal_to_local"
        )
        self._goal_transformer_client.wait_for_service()

        self._graph = GraphHandler("/home/zacravi/Downloads/pennov_utm.json")

        self._graph_viz = GraphVisualizerComponent(
            parent_node=self,
            graph=self._graph,
            target_frame="map",
            topic_name="/graph_viz",
            scale=2.0,
            publish_rate=1,
            transform_function=lambda x, y: self.get_local_goal(x, y, 0.0),
        )

        self._graph_viz.set_graph(self._graph)

    def get_local_goal(
        self, x: float, y: float, yaw: float = 0.0
    ) -> tuple[float, float, float] | None:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        q = Rotation.from_euler("z", yaw).as_quat()
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        request = TransformGoal.Request()
        request.goal_global = goal

        future = self._goal_transformer_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)  # yields control to executor
        result = future.result()

        # result = self._goal_transformer_client.call(request)

        if not result.success:
            self._log_info(f"Transform failed: {result.message}")
            return None

        q_out = result.goal_local.pose.orientation
        yaw_out = Rotation.from_quat([q_out.x, q_out.y, q_out.z, q_out.w]).as_euler(
            "xyz"
        )[2]

        return (
            result.goal_local.pose.position.x,
            result.goal_local.pose.position.y,
            yaw_out,
        )


def main(args=None):
    rclpy.init(args=args)

    node = GraphVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
