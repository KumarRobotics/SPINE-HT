#!/usr/bin/env python3

import math
import time
from abc import ABC
from concurrent.futures import Future
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Dict
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import os
from nav_msgs.msg import Odometry

import numpy as np
import rclpy
import rclpy.task
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from spine_multi.parsing import ListDictParser
from std_msgs.msg import Int16, String
from teaming_msgs.msg import BehaviorRequest, BehaviorResult, GoalRequest, GoalStatus
from teaming_msgs.srv import SetLabels, Query
from spine_multi.spine.mapping.frontiers import FrontierExtractor, Node as GraphNode
from spine_multi.spine.mapping.graph_util import GraphHandler, parse_graph
import json

from bosdyn.client.math_helpers import SE2Pose

from spot_executor.spot import Spot
from spot_skills.navigation_utils import navigate_to_absolute_pose, follow_trajectory_continuous



class GoalStatusEnum(Enum):
    IN_PROGRESS = 0
    SUCCESS = 1
    FAILED = 2


@dataclass
class NavGoal:
    idx: int
    x: float
    y: float
    yaw: float
    status: GoalStatusEnum
    ack: bool


class SpotAutonomyManager(Node):
    def __init__(self):
        super().__init__("spot_autonomy_manager")

        self._log_info("init")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        self.declare_parameters(
            namespace="",
            parameters=[
                ("navigation_action_server", "navigate_to_pose"),
                ("navigation_request", "navigation_request"),
                (f"robot_name", ""),
                ("frame_id", "map"),
                ("parameter_server", "controller_server/set_parameters"),
                ("subscription_prefix", ""),
                ("label_service", "/detector/set_labels"),
                ("vlm_service", "vlm_node/query_scene"),
                ("costmap_topic", "local_costmap/costmap"),
            ],
        )


        set_label_client_name = (
            self.get_parameter("label_service").get_parameter_value().string_value
        )
        subscription_prefix = (
            self.get_parameter("subscription_prefix").get_parameter_value().string_value
        )
        vlm_service_name = (
            self.get_parameter("vlm_service").get_parameter_value().string_value
        )

        
        costmap_topic = (
            self.get_parameter("costmap_topic").get_parameter_value().string_value
        )

        spot_username = os.environ['ADT4_BOSDYN_USERNAME']
        spot_ip = os.environ['ADT4_BOSDYN_IP']
        spot_pwd = os.environ['ADT4_BOSDYN_PASSWORD']

        self._spot_client = Spot(username=spot_username, ip=spot_ip, password=spot_pwd)
        self._spot_client.sit()
        self._spot_client.stand()


        pose_on_start = self._spot_client.get_pose()
        self._pose_on_start = SE2Pose(pose_on_start[0], pose_on_start[1], pose_on_start[2])

        self._parser = ListDictParser()

        sub_cbk_group = ReentrantCallbackGroup()
        self._nav_req_sub = self.create_subscription(
            BehaviorRequest,
            subscription_prefix + "behavior_request",
            self._behavior_cbk,
            qos_profile,
            callback_group=sub_cbk_group,
        )

        self._received_behaviors = set()
        self._acked_results = set()
        self._behavior_results: Dict[int, List[dict]] = {}

        self._ack_pub = self.create_publisher(
            Int16,
            "behavior_request_ack",
            qos_profile,
        )

        self._result_pub = self.create_publisher(
            BehaviorResult, "behavior_result", qos_profile
        )

        response_ack_cbk = ReentrantCallbackGroup()
        self._response_ack_sub = self.create_subscription(
            Int16,
            subscription_prefix + "behavior_result_ack",
            self._response_ack,
            qos_profile,
            callback_group=response_ack_cbk,
        )        
        
        
        self._graph = GraphHandler("")
        self._frontier_extractor = FrontierExtractor(init_graph=self._graph)


        graph_cbk_group = ReentrantCallbackGroup()
        self.graph_sub = self.create_subscription(
            String,
            "/basestation/graph",
            self._graph_cbk,
            qos_profile,
            callback_group=graph_cbk_group
        )

        self._log_info("created pub / sub")

        self._set_labels_component = LabelServiceTranslator(
            parent_node=self,
            label_service=set_label_client_name,
        )

        self._log_info("set label init")

        # navigation
        self._nav_component = SpotNavigationComponent(parent_node=self, spot_client=self._spot_client)

        # vlm_query
        self._vlm_component = VLMServiceComponent(
            parent_node=self,
            vlm_service=vlm_service_name,
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        timer_cbk_group = ReentrantCallbackGroup()
        self._odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self._publish_odom, callback_group=timer_cbk_group)

        self._log_info("init")


    def _publish_odom(self):
        pose = self._spot_client.get_pose()
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        pose_se2 = SE2Pose(pose[0], pose[1], pose[2])
        in_local = self._pose_on_start.inverse()  * pose_se2

        msg.pose.pose.position.x = in_local.x
        msg.pose.pose.position.y = in_local.y
        msg.pose.pose.position.z = 0.0

        yaw = in_local.angle
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0  
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self._odom_publisher.publish(msg)

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id 
        transform.transform.translation.x = float(pose[0])
        transform.transform.translation.y = float(pose[1])
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)

        self._tf_broadcaster.sendTransform(transform)





    
    def _graph_cbk(self, msg: String):
        graph_as_str = msg.data 

        if len(self._graph.graph.nodes) == 0:
            self._log_info(f"Have initial graph: {graph_as_str}")
 
        try:
            graph = json.loads(graph_as_str)
            self._graph.reset(graph)

        except Exception as ex:
            self._log_info(f"could not parse graph; {ex}")


    def _result_pub_cbk(self):
        self._log_info(f"in result pub cbk: {self._behavior_results}")
        results_to_remove = []
        for idx, result_msg in self._behavior_results.items():
            self._result_pub.publish(result_msg)
            self._log_info(f"publishing result: {result_msg}")

            if idx in self._acked_results:
                self._log_info(f"result {idx} has been acked")
                results_to_remove.append(idx)

        for to_remove in results_to_remove:
            self._behavior_results.pop(idx)

    def _response_ack(self, ack_msg: Int16):
        self._acked_results.add(ack_msg.data)
        self._log_info(f"got ack response for idx: {ack_msg.data}")

    def _build_result_msg(self, idx: int, data: List[Dict[str, Any]]) -> BehaviorResult:
        self._log_info(f"building result: {data}")

        try:
            resp_msg = BehaviorResult()
            resp_msg.idx = idx
            resp_msg_str = String()
            resp_msg_str.data = self._parser.serialize(data)

            resp_msg.result = resp_msg_str
        except Exception as ex:
            self._log_info(f"got exception {ex}")

        self._log_info("returning  result")

        return resp_msg

    def _log_info(self, msg: str):
        self.get_logger().info(f"[jackal autonomy manager] {msg}")

    def _behavior_cbk(self, msg: BehaviorRequest) -> None:
        self._log_info(f"got msg: {msg}")
        ack_msg = Int16()
        ack_msg.data = msg.idx
        self._ack_pub.publish(ack_msg)

        if msg.idx not in self._received_behaviors:
            self._received_behaviors.add(msg.idx)
            task_sequence = self._parser.parse(msg.behaviors.data)

            self._log_info(f"parsed task: {task_sequence}")

            outcomes = []
            for task in task_sequence:
                self._log_info(f"eval task: {task}")

                behavior = task["behavior"][0]

                self._log_info(f"eval behavior: {behavior}")

                if behavior == "navigate":
                    success, notes = self._nav_component.navigate_and_wait(
                        x=task["x"][0],
                        y=task["y"][0],
                        check_yaw=task["check_yaw"][0],
                        yaw=task["yaw"][0],
                        pose_on_start=self._pose_on_start,
                    )

                    outcomes.append(
                        {"behavior": ("navigate", "str"), "success": (success, "bool")}
                    )

                    if not success:
                        break

                if behavior == "set_labels":
                    self._log_info(f"calling set labels")
                    incoming_labels = task["labels"][0]
                    labels = incoming_labels.replace("|", ",")
                    result = self._set_labels_component.set_labels(
                        labels, msg.idx
                    )

                    self._log_info(f"got labels call result : {result}")
                    outcomes.append(result)

                    if not result["success"]:
                        break

                if behavior == "query_vlm":
                    query = task["query"][0]
                    self._log_info(f"calling query_vlm: {query}")

                    result = self._vlm_component.query_vlm(query, msg.idx)
                    self._log_info(f"got response: {result}")
                    outcomes.append(result)

                    if not result["success"]:
                        break

                if behavior == "map_region":
                    map_x = task["map_x"][0]
                    map_y = task["map_y"][0]
                    outcomes.append(self._try_add_edges(map_x, map_y))


            self._log_info(f"at end of call")

            result_msg = self._build_result_msg(idx=msg.idx, data=outcomes)
            while msg.idx not in self._acked_results:
                self._result_pub.publish(result_msg)
                time.sleep(15)
                self._log_info(f"pub response: {result_msg.result}")
        

    def _try_add_edges(self, x: float, y: float) -> dict:

        result = {"behavior": ("map_region", "str"),
                  "success": (True, "bool"),
                  "new_neighbors": ("", "str")}

        return result


class LabelServiceTranslator:
    def __init__(
        self,
        parent_node: SpotAutonomyManager,
        label_service: str,
    ):
        self._parent_node = parent_node

        label_cbk_group = ReentrantCallbackGroup()
        self._label_client = self._parent_node.create_client(
            SetLabels, label_service, callback_group=label_cbk_group
        )

        self._log_info("waiting for label service")
        self._label_client.wait_for_service()

        self._in_progress = False
        self._result = {}
        self._current_idx = 0

    def _log_info(self, msg):
        self._parent_node.get_logger().info(f"[autonomy manager] [label service] {msg}")

    def set_labels(self, query: str, idx: int) -> Dict[str, Tuple[Any, str]]:
        request = SetLabels.Request()
        request.labels = ascii(query)

        self._current_idx = idx

        # try:
        self._log_info(
            f"[label service translator] formed request for set labels: {request}"
        )
        response_future = self._label_client.call_async(request)

        response_future.add_done_callback(
            lambda f: self._handle_service_response(f, idx)
        )

        self._in_progress = True

        while self._in_progress:
            self._log_info("[nav manager] waiting for goal to complete")
            time.sleep(5)

        return self._result

    def _handle_service_response(self, future: Future, idx: int):
        try:
            response = future.result()
            success = response.success
            self._parent_node.get_logger().info(
                f"[label service translator] service returned {success} for {self._current_idx}"
            )
        except Exception as e:
            self._parent_node.get_logger().error(
                f"Service call failed for {self._current_idx}: {e}"
            )
            success = False

        self._in_progress = False

        self._result = {"behavior": ("set_labels", "str"), "success": (success, "bool")}

        # self._result_dict[self._current_idx] = self._parent_node._build_result_msg(
        #     idx=self._current_idx,
        #     data=[{"behavior": ("set_labels", "str"), "result": (success, "bool")}],
        # )

        self._log_info(f"got set label response")


class SpotNavigationComponent:
    def __init__(
        self,
        parent_node: Node,
        spot_client: Spot,
        frame_id: str = "map",
        goal_tol: Optional[float] = 3,
        status_topic: Optional[str] = "nav_component/status",
    ):
        self._spot_client = spot_client
        self._parent_node = parent_node
        self._frame_id = frame_id


        self._status_updates = self._parent_node.create_publisher(
            String, status_topic, 10
        )

        # Store current goal handle
        self._goal_tol = goal_tol

        self._strict_yaw = False
        self._log_info_idx = 0
        self._goal_successful = False
        self._progress_idx = 0

    def print(self, level, msg):
        # total hack for spot tools
        self._progress_idx += 1
        if self._progress_idx % 500 == 0:
            self._log_info(f"{level}: {msg}")

    def path_following_progress_feedback(self, current_point, goal_point):
        # total hack for spot tools
        self._progress_idx += 1
        if self._progress_idx % 500 == 0:
            self._log_info(f"{current_point}: {goal_point}")


    def _log_info(self, msg: str) -> None:
        self._parent_node.get_logger().info(f"[navigation component action] {msg}")

    def navigate_and_wait(
        self,
        x: float,
        y: float,
        pose_on_start: SE2Pose,
        yaw: Optional[float] = 0.0,
        timeout_sec: Optional[float | None] = 15.0,
        check_yaw: Optional[bool] = False,
    ) -> Tuple[bool, str]:

        goal_world = SE2Pose(x=x, y=y, angle=yaw)

        goal = pose_on_start * goal_world

        self._log_info(f"goal world: {goal_world}, goal: {goal}, start: {pose_on_start}")


        current_pose = self._spot_client.get_pose()[:2].reshape(1, 2)
        goal_np = np.array([goal.x, goal.y]).reshape(1,2)


        dist = np.linalg.norm(goal_np - current_pose)

        # assume spot can move at 0.25m/s min 
        min_spot_speed = 0.25
        timeout_sec = dist / min_spot_speed
        self._log_info(f"Timeout is: {timeout_sec}")

        segment_size = 1.0

        
        segments = np.linspace(current_pose, goal_np, int((dist / segment_size) + 1)).squeeze().reshape(-1,2)

        if len(segments) == 1:
            segments = np.vstack([current_pose, segments])

        self._log_info(f"segments: {segments}, shape: {segments.shape}")

        self._progress_idx = 0
        finished = follow_trajectory_continuous(
            spot=self._spot_client,
            waypoints_list=segments, 
            lookahead_distance=2.0, 
            goal_tolerance= self._goal_tol,
            timeout=timeout_sec,
            feedback = self,
            )

        if check_yaw:
            navigate_to_absolute_pose(self._spot_client, goal)

        current_pose = self._spot_client.get_pose()[:2].reshape(1, 2)
        success = np.linalg.norm(goal_np - current_pose) < self._goal_tol

        return success, ""


class VLMServiceComponent:
    def __init__(
        self,
        parent_node: SpotAutonomyManager,
        vlm_service: str = "vlm_node/query_scene",
    ):

        self._prarent_node = parent_node

        vlm_cbk_group = ReentrantCallbackGroup()
        self._vlm_client = self._prarent_node.create_client(
            Query, vlm_service, callback_group=vlm_cbk_group
        )

        self._log_info(f"waiting for VLM server...")
        self._vlm_client.wait_for_service()
        self._log_info(f"VLM is initialized")

        self._in_progress = False
        self._result = {}

    def _log_info(self, msg: str) -> None:
        self._prarent_node.get_logger().info(f"[jackal autonomy] [vlm component] {msg}")

    def query_vlm(self, request: str, idx: int) -> None:
        self._log_info(f"got req for {idx}")
        self._query_vlm_async(request, idx)

        self._in_progress = True

        while self._in_progress:
            self._log_info("waiting for vlm response")
            time.sleep(5)

        return self._result

    def _query_vlm_async(self, vlm_request: str, idx: int):
        """Make VLM service call without blocking"""

        request = Query.Request()
        request.query = ascii(vlm_request)

        self._log_info(
            f"[vlm service translator] calling VLM service async for idx {idx} with query: {request.query}"
        )

        future = self._vlm_client.call_async(request)
        future.add_done_callback(lambda f: self._handle_vlm_response(f, idx))

    def _handle_vlm_response(self, future, idx: int):
        """Handle VLM service response and start publishing"""
        self._log_info(f"[vlm service translator] handling vlm resp for {idx}")

        try:
            response = future.result()
            success = response.success
            answer = response.answer
            self._log_info(
                f"[vlm service translator] VLM returned success={success} for {idx} with answer: {answer}"
            )
        except Exception as e:
            self._log_info(
                f"[vlm service translator] VLM service call failed for {idx}: {e}"
            )
            success = False
            answer = "VLM service call failed"

        self._in_progress = False

        self._result = {
            "behavior": ("query_vlm", "str"),
            "success": (success, "bool"),
            "answer": (answer, "str"),
        }


def main():
    rclpy.init()
    node = SpotAutonomyManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
