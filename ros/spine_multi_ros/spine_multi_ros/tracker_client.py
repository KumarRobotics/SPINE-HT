#!/usr/bin/env python3

from typing import Dict, List, Optional
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from spine_multi.spine import GraphHandler
from spine_multi.spine.util import UpdatePromptFormer
from spine_multi.spine.viz.viz_ros import GraphVisualizerComponent
from teaming_msgs.msg import Track
from vision_ros2.tracker import Hypothesis, from_track_msg


class TrackerClientComponent:
    def __init__(
        self,
        parent_node: Node,
        graph: GraphHandler,
        prompt_former: UpdatePromptFormer,
        graph_viz: GraphVisualizerComponent,
        track_topic: Optional[str] = "tracks",
        tracks: Optional[Dict[int, Hypothesis]] = None,
        subscription_prefix: str = ""
    ):
        self._parent_node = parent_node
        self._graph = graph
        self._prompt_former = prompt_former
        self._graph_viz = graph_viz
        self._current_location = ""

        if tracks == None:
            self.tracks: Dict[int, Hypothesis] = {}
        else:
            self.tracks = tracks

        self.added_tracks = set()
        self.updated_tracks = set()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        track_cbk_group = ReentrantCallbackGroup()
        self.track_sub = parent_node.create_subscription(
            Track,
            subscription_prefix + track_topic,
            self._track_cbk,
            qos_profile,
            callback_group=track_cbk_group,
        )

    def _log_info(self, msg):
        self._parent_node.get_logger().info(f"[tracker client] {msg}")

    def set_current_location(self, location: str) -> None:
        self._current_location = location

    def _track_cbk(self, track: Track) -> None:
        # parent = self._current_location
        parent = self._graph.get_current_location()

        track_point = np.array(
            [track.pose.pose.position.x, track.pose.pose.position.y]
        )
        region_nodes, region_node_locs = self._graph.get_region_nodes_and_locs()
        closest_region_idx = np.linalg.norm(
            track_point - region_node_locs[:, :2], axis=-1
        ).argmin()
        parent = region_nodes[closest_region_idx]

        self._log_info(f"incoming track at {track_point}")
        self._log_info(f"assigned incoming track {track.label} to {parent} at {region_node_locs[closest_region_idx]}")

        track = from_track_msg(track, parent=parent)

        if track.idx in self.tracks:
            if not self.tracks[track.idx].is_same(track, pos_tol=1):
                self.tracks[track.idx] = track
                self.updated_tracks.add(track.idx)
                # self._parent_node.get_logger().info(
                #     f"[spine multi] [tracker] updated track: {track.idx} ({track.label})"
                # )

        elif not self._is_duplicate(track):
            self.tracks[track.idx] = track
            self.added_tracks.add(track.idx)
            self._parent_node.get_logger().info(
                f"[spine multi] [tracker] added track: {track.idx}"
            )

    def _is_duplicate(self, incoming_track: Hypothesis) -> bool:
        # TODO inefficient
        for idx, track in self.tracks.items():
            if track.is_same(incoming_track):
                return True
        return False

    def parse_track_updates(self) -> List[str]:
        """Construct new object message in the planning API.
        Objects are newly received tracks.

        Also update graph.

        Returns
        -------
        str
            New object message in LLM API.
        """
        added_track_idx = self.added_tracks.copy()
        self.added_tracks.clear()

        self._parent_node.get_logger().info(f"Have {len(added_track_idx)} new tracks")

        if len(added_track_idx) == 0:
            return ""

        for idx in added_track_idx:
            # tracks are received on route to current_location``
            # self.tracks[idx].parent = self.current_location
            track = self.tracks[idx]

            node_id = f"discovered_{track.label}_{idx}"

            new_node = {
                "name": node_id,
                "type": "object",
                "coords": f"[{track.pose[0]:0.1f}, {track.pose[1]:0.1f}]",
            }
            new_connections = [[node_id, track.parent]]
            self._prompt_former.update(
                new_nodes=[new_node], new_connections=new_connections
            )

            # add node with connection to region where it was discovered
            # only add x, y
            self._graph.update_with_node(
                node=node_id,
                edges=[track.parent],
                attrs={"coords": track.pose[:2], "type": "object"},
            )

            self._graph_viz.set_graph(self._graph.get_graph())
