from typing import List, Tuple
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException

import numpy as np


class ProgressChecker:
    def __init__(self, parent_node: Node,  timeout_s: float, world_frame, robot_frame, dist_tol: float):
        self._timeout_s = timeout_s
        self._last_active_time = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, parent_node)
        self._history_buffer = []  # better than np append?
        self._start_t = 0
        self._world_frame = world_frame
        self._robot_frame = robot_frame
        self._dist_tol = dist_tol
        self._parent_node = parent_node


    def _log_info(self, msg: str) -> None:
        self._parent_node.get_logger().info(f"[progress checker] {msg}")


    def reset(self):
        self._history_buffer = []
        self._last_active_time = None

    def has_timedout(self) ->bool:
        current_pos, could_lookup = self.lookup_robot_pos() 

        # assume in time if we can't look up transform
        if not could_lookup:
            self._log_info(f"could not look up transform")
            return True

        curr_time = self._parent_node.get_clock().now()
        min_dist_to_history = self.get_min_dist_to_buffer(current_pos)

        if self._last_active_time == None:
            self._last_active_time = curr_time

        duration = curr_time - self._last_active_time
        duration_sec = duration.nanoseconds / 1e9

        if min_dist_to_history > self._dist_tol:
            self._last_active_time = curr_time
            self.add_point(current_pos)
        else:
            self._log_info(f"[WARNING] Robot has not moved more than {self._dist_tol} in {duration_sec}")

        
        if duration_sec > self._timeout_s:
            return True

        return False


    def add_point(self, point: np.ndarray) -> None:
        self._history_buffer.append(point.reshape(2,))

    def get_min_dist_to_buffer(self, pos: np.ndarray) -> float:
        # nothing to compare
        if len(self._history_buffer) == 0:
            return np.inf

        return np.linalg.norm(pos - np.array(self._history_buffer).reshape(-1, 2), axis=-1).min()


    def lookup_robot_pos(self) -> Tuple[np.ndarray, bool]:
        try:
            transform  = self._tf_buffer.lookup_transform(
                self._world_frame, self._robot_frame, rclpy.time.Time()  # Latest available time rospy.Time(0)
            )

            point = transform.transform.translation
            np_pt = np.array([point.x, point.y])
            return np_pt, True
        except (
            TransformException
        ):
            return None, False
