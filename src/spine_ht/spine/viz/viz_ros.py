import rclpy
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from spine_ht.spine.mapping.graph_util import GraphHandler


class NodeMarker:
    def __init__(self, node_marker: Marker, text_marker: Marker):
        """Hold marker for node and corresponding text label.
        TODO text should be optional

        Parameters
        ----------
        node_marker : Marker
        text_marker : Marker
        """
        self.node_marker = node_marker
        self.text_marker = text_marker


class GraphViz:
    RED_COLOR = ColorRGBA(r=0.75, a=1.0)
    GREEN_COLOR = ColorRGBA(g=0.75, a=1.0)
    BLUE_COLOR = ColorRGBA(b=0.75, a=1.0)
    WHITE_COLOR = ColorRGBA(r=0.75, g=0.75, b=0.75, a=1.0)

    def __init__(self, graph: GraphHandler, scale: float, target_frame: str) -> None:
        """Class for visualizing graph.

        Parameters
        ----------
        graph : GraphHandler
            Graph to visualize.
        scale : float
            Marker scale.
        target_frame : str
            Frame to plot in.
        """
        self.scale = scale
        self.target_frame = target_frame

        self.graph = graph
        self.node_markers = {}
        self.edge_markers = {}

        self.build_markers()

        self.prev_num_connections = 0
        self.changed = True

    def update_graph(self, graph: GraphHandler) -> None:
        self.graph = graph
        self.build_markers()
        self.changed = True

    def publish(self, publisher) -> None:
        """Publish node and text markers.

        Parameters
        ----------
        publisher : rclpy.publisher.Publisher
            Send messages on this publisher.
        """
        # TODO look at logic
        # let's try publishing graph no matter what
        # if (
        #     self.changed
        #     or self.prev_num_connections != publisher.get_subscription_count()
        # ):
        # self.prev_num_connections = publisher.get_subscription_count()
        for node_marker in self.node_markers.values():
            publisher.publish(node_marker.node_marker)
            publisher.publish(node_marker.text_marker)
        for edge_marker in self.edge_markers.values():
            publisher.publish(edge_marker)

        self.changed = False

    def get_marker_msg(self, x: int, y: int, id: int, z=1) -> Marker:
        marker_msg = Marker()
        marker_msg.id = id
        marker_msg.header.frame_id = self.target_frame
        # ROS2 requires setting the timestamp
        marker_msg.header.stamp = rclpy.time.Time().to_msg()

        marker_msg.pose.position.x = float(x)
        marker_msg.pose.position.y = float(y)
        marker_msg.pose.position.z = float(z)
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.x = self.scale
        marker_msg.scale.y = self.scale
        marker_msg.scale.z = self.scale

        marker_msg.color = self.WHITE_COLOR

        return marker_msg

    def build_markers(self) -> None:
        """Constructs marker messages."""
        for id, node in enumerate(self.graph.get_nx_graph().nodes):
            attr = self.graph.get_nx_graph().nodes[node]

            loc_x = float(attr["coords"][0])
            loc_y = float(attr["coords"][1])
            type = attr["type"]  # either object or region

            marker_msg = self.get_marker_msg(loc_x, loc_y, 2 * id)
            marker_msg.type = marker_msg.SPHERE

            if type == "object":
                marker_msg.color = self.BLUE_COLOR
            elif type == "region":
                marker_msg.color = self.RED_COLOR

            marker_msg.action = marker_msg.ADD

            marker_msg_text = self.get_marker_msg(
                loc_x, loc_y, 2 * id + 1, z=self.scale * 3
            )
            marker_msg_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker_msg_text.type = marker_msg.TEXT_VIEW_FACING

            text_msg = f"{node} ({loc_x:0.1f}, {loc_y:0.1f})"
            for k, v in attr.items():
                if k == "type" or k == "coords":
                    continue
                text_msg += f"\n{k}: {v}"

            marker_msg_text.text = text_msg

            self.node_markers[node] = NodeMarker(marker_msg, marker_msg_text)

        for id, (n1, n2) in enumerate(self.graph.get_nx_graph().edges):
            start = [float(c) for c in self.graph.lookup_node(n1)[0]["coords"]]
            end = [float(c) for c in self.graph.lookup_node(n2)[0]["coords"]]

            base_id = 2 * len(self.graph.get_nx_graph().nodes) + 2 * id
            marker_msg_start = self.get_marker_msg(0, 0, base_id)

            marker_msg_start.type = marker_msg.LINE_STRIP
            marker_msg_start.action = marker_msg.ADD
            marker_msg_start.color = self.GREEN_COLOR
            marker_msg_start.scale.x = self.scale / 5
            marker_msg_start.points.append(Point(x=start[0], y=start[1], z=0.0))
            marker_msg_start.points.append(Point(x=end[0], y=end[1], z=0.0))

            self.edge_markers[(n1, n2)] = marker_msg_start


class GraphVisualizerComponent:
    def __init__(
        self,
        parent_node: Node,
        graph: GraphHandler,
        scale: float = 1.0,
        target_frame: str = "map",
        publish_rate: float = 1.0,
        topic_name: str = "visualization_marker",
    ):
        """
        Component version that can be embedded in another node.

        Parameters
        ----------
        parent_node : Node
            The parent ROS2 node to attach this component to
        scale : float
            Marker scale
        target_frame : str
            Frame to plot in
        publish_rate : float
            Publishing rate in Hz
        topic_name : str
            Topic name for marker publishing
        """
        self.parent_node = parent_node
        self.scale = scale
        self.target_frame = target_frame

        # Create publisher on parent node
        self.marker_publisher = parent_node.create_publisher(Marker, topic_name, 10)

        # Create timer on parent node
        publish_group = ReentrantCallbackGroup()
        self.timer = parent_node.create_timer(
            1.0 / publish_rate, self.timer_callback, callback_group=publish_group
        )

        # Initialize graph viz
        self.graph_viz = GraphViz(graph, scale=scale, target_frame=target_frame)
        self.set_graph(graph=graph)

        self.parent_node.get_logger().info("Graph Visualizer Component initialized")

    def set_graph(self, graph: GraphHandler):
        """Set the graph to visualize."""
        self.graph_viz.update_graph(graph)

    def timer_callback(self):
        """Timer callback to publish markers."""
        if self.graph_viz is not None:
            self.graph_viz.publish(self.marker_publisher)

    def destroy(self):
        """Clean up resources."""
        if self.timer:
            self.timer.destroy()


class GraphVisualizerNode(Node):
    def __init__(self):
        super().__init__("graph_visualizer")

        # Parameters
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("publish_rate", 10.0)

        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        # Publisher
        self.marker_publisher = self.create_publisher(
            Marker, "visualization_marker", 10
        )

        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Initialize with empty graph - you'll need to update this based on your use case
        self.graph_viz = None

        self.get_logger().info("Graph Visualizer Node initialized")

    def set_graph(self, graph: GraphHandler):
        """Set the graph to visualize."""
        scale = self.get_parameter("scale").get_parameter_value().double_value
        target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        if self.graph_viz is None:
            self.graph_viz = GraphViz(graph, scale, target_frame)
        else:
            self.graph_viz.update_graph(graph)

    def timer_callback(self):
        """Timer callback to publish markers."""
        if self.graph_viz is not None:
            self.graph_viz.publish(self.marker_publisher)


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
