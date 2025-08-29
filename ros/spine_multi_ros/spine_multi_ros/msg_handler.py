from rclpy.node import Node

from typing import Dict, List, Tuple, Any
from spine_multi.parsing import ListDictParser
from teaming_msgs.msg import GoalRequest, GoalStatus, BehaviorRequest, BehaviorResult
from std_msgs.msg import String, Int16
from rclpy.callback_groups import ReentrantCallbackGroup
import time





class MessageHandler:
    def __init__(self, parent: Node, robot_name, behavior_request_ack_sub, behavior_result_sub, 
                 behavior_result_ack_pub, qos_profile, behavior_request: str):
        self._parent_node = parent
        self._behavior_idx = 0
        self._robot_name = robot_name
        # TODO define custom type
        self._behavior_results: Dict[int, List[Dict[str, Tuple[Any, str]]]] = {}
        self._msg_converter = ListDictParser()
        self._acked_msgs = set()

        sub_cbk_group = ReentrantCallbackGroup()
        self._behavior_req_pub = self._parent_node.create_publisher(
            BehaviorRequest,
            behavior_request,
            qos_profile,
        )

        self._behavior_req_ack_sub = self._parent_node.create_subscription(
            Int16, behavior_request_ack_sub, self._ack_cbk, qos_profile,
            callback_group=sub_cbk_group
        )

        response_sub_cbk_group = ReentrantCallbackGroup()
        self._behavior_resp_sub = self._parent_node.create_subscription(
            BehaviorResult, behavior_result_sub, self._result_sub, qos_profile,
            callback_group=response_sub_cbk_group,
        )   


        self._behavior_resp_ack_sub = self._parent_node.create_publisher(
            Int16,
            behavior_result_ack_pub, qos_profile
        )


    def _result_sub(self, msg: BehaviorResult):
        self._behavior_results[msg.idx] = self._msg_converter.parse(msg.result.data)

        self._log_info(f"Got result for idx {msg.idx}: {self._behavior_results[msg.idx]}")

        resp = Int16()
        resp.data = msg.idx

        self._behavior_resp_ack_sub.publish(resp)

    def _ack_cbk(self, msg: Int16) -> None:
        self._log_info(f"Got behavior request ack for msg id: {msg.data}")
        self._acked_msgs.add(msg.data)

    def _log_info(self, msg: str) -> None:
        self._parent_node.get_logger().info(f"[msg handler] [{self._robot_name}] {msg}")

    def navigate_and_wait(
        self, x: float, y: float, yaw: float,  check_yaw: bool):
        msg = self.get_navigate_msg(x=x, y=y, yaw=yaw, check_yaw=check_yaw)
        return self.send_and_wait(msg)

    def get_navigate_msg(
        self, x: float, y: float, yaw: float,  check_yaw: bool
    ) -> BehaviorRequest:
        msg = BehaviorRequest()
        string_msg = String()
        string_msg.data = self._msg_converter.serialize([
            {"behavior": ("navigate", "str"),
             "x": (x, 'float'),
             "y": (y, "float"),
             "yaw": (yaw, "float"),
             "check_yaw": (check_yaw, "bool")}
        ])
        msg.behaviors = string_msg
        msg.idx = self._behavior_idx 
        self._behavior_idx += 1

        return msg

    def send_label_request(self, label_list: str):
        msg = self.get_label_msg(label_list=label_list)
        return self.send_and_wait(msg)

    def get_label_msg(self, label_list: str):
        msg = BehaviorRequest()
        string_msg = String()
        string_msg.data = self._msg_converter.serialize([
            {"behavior": ("set_labels", "str"),
             "labels": (label_list, "str")}
         ])
        msg.behaviors = string_msg
        msg.idx = self._behavior_idx 
        self._behavior_idx += 1

        return msg

    def send_vlm_query(self, query: str):
        msg = self.get_vlm_query_msg(query=query)
        return self.send_and_wait(msg)

    def get_vlm_query_msg(self, query: str):
        msg = BehaviorRequest()
        string_msg = String() 
        string_msg.data = self._msg_converter.serialize([
            {"behavior": ("query_vlm", "str"), "query": (query, "string")}
        ])
        msg.behaviors = string_msg
        msg.idx = self._behavior_idx 
        self._behavior_idx += 1
        return msg


    def send_and_wait(self, msg: BehaviorRequest):
        while not msg.idx in self._acked_msgs:
            # if not self._logged_goal:
            self._log_info(
                f" sending msg: {msg}")
                # self._logged_goal = True

            self._behavior_req_pub.publish(msg)
            time.sleep(15)
        
        while msg.idx not in self._behavior_results:
            self._log_info(f"waiting for response")
            time.sleep(15)

        self._log_info(f"Got response: {self._behavior_results[msg.idx]}")
 
        return self._behavior_results[msg.idx]