import json
from collections import namedtuple
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from openai import OpenAI
from spine_multi.planner_logging import get_logger
from spine_multi.spine.mapping.graph_util import GraphHandler
from spine_multi.spine.models import OpenAILLM
from spine_multi.spine.prompts.prompts import INVALID_JSON
from spine_multi.spine.validator import Validator

ValidPlanFeedback = namedtuple("ValidPlanFeedback", ["success", "message"])
VALID_ACTIONS = set(
    [
        "explore_region",
        "map_region",
        "inspect",
        "clarify",
        "goto",
        "answer",
        "extend_map",
        "replan",
    ]
)
REGION_ACTIONS = set(["explore_region", "map_region", "goto"])
NAVIGATION_ACTIONS = set(["explore_region", "map_region", "goto", "inspect"])
INTERACTION_ACTIONS = set(["explore_region", "map_region", "goto", "inspect"])
OBJECT_ACTIONS = set(["inspect"])
SPATIAL_ACTION = set(["extend_map"])
EXPLORE_ACTIONS = set(["explore_region"])


class SPINE:
    def __init__(
        self,
        graph: GraphHandler,
        llm: Optional[str] = "openai",
        n_attempts: Optional[int] = 3,
        print_feedback: Optional[bool] = True,
    ) -> None:
        """_summary_

        Parameters
        ----------
        graph : GraphHandler
        log_name : str, optional
            Name for LLM logs, if desired
        llm : Optional[str], optional
            "openai" or "unsloth
        model_path : Optional[str], optional
            If unsloth, provide the model path

        Raises
        ------
        ValueError
            _description_
        """
        self.graph = graph
        if llm == "openai":
            self.client = OpenAILLM()
        else:
            raise ValueError(f"{llm} not supported")

        self.logger = get_logger()
        self.n_attempts = n_attempts
        self.base_request = ""
        self.msg_history = []
        self.validator = Validator(self.logger)

        self.most_recent_query = []
        self.print_feedback = print_feedback

    def clean_llm_output(self, s: str) -> str:
        return s.strip("```").strip("json")

    def preprocess_cmd_str(self, cmd_list: str) -> List[str]:
        """Apply some formatting"""

        if isinstance(cmd_list, str):
            cmd_list = cmd_list.strip("[").strip("]")
            cmd_list = cmd_list.split(",")

        if len(cmd_list) == 1:
            parsed_cmds = cmd_list
        else:
            # some arguments may have commas, thus be over split
            # go through list and add back full function-argument pairs
            parsed_cmds = []
            current_cmd = ""
            for cmd in cmd_list:
                if cmd.strip().startswith(tuple(VALID_ACTIONS)):
                    if len(current_cmd):
                        parsed_cmds.append(current_cmd)
                        current_cmd = ""

                    current_cmd = cmd
                else:
                    current_cmd += f",{cmd}"

            if len(current_cmd):
                parsed_cmds.append(current_cmd.strip())

        return parsed_cmds

    def try_parse(self, response: str) -> Tuple[Dict[str, Any], ValidPlanFeedback]:
        """Try to parse LLM response

        Returns
        -------
        Tuple[Dict[str, Any], ValidPlanFeedback]
            parse plan, any relevant feedback
        """
        try:
            if not isinstance(response, str):
                self.logger.info(f"response is of type: {type(response)}")
                response = str(response)
                # TODO log this
            response = self.clean_llm_output(response)
            try:
                as_json = json.loads(response, strict=False)
            except:
                as_json = json.loads(response.replace("'", '"'), strict=False)
            plan = self.preprocess_cmd_str(as_json["plan"])
            # plan = [p.strip() for p in as_json["plan"].split(",")]
            as_json["plan"] = plan
            return as_json, ValidPlanFeedback(True, "")
        except Exception as ex:
            return {}, ValidPlanFeedback(False, str(ex))

    def query_llm(self, msg: str) -> Tuple[str, bool]:
        """Query LLM with `msg`. Note that history is also provided
        to prompt

        Returns
        -------
        Tuple[str, bool]
            LLM response, success
        """
        self.most_recent_query = msg
        response, success = self.client.query_llm(msg)

        return response, success

    def extract_plan(self, plan_as_str: str) -> Tuple[List[str], ValidPlanFeedback]:
        return self.validator.validate_plan(plan_as_str=plan_as_str, graph=self.graph)

    def _generate_plan(
        self, msg: List[Dict[str, str]]
    ) -> Tuple[Dict[str, Any], bool, List[str]]:
        response = {}
        success = False
        logs = []

        for _ in range(self.n_attempts):
            top_msg, could_query_llm = self.query_llm(msg)

            # TODO not sure if we should keep this or try and train it in
            # top_msg = top_msg.replace("'", '"')

            if not could_query_llm:
                return {"msg": top_msg}, False, logs

            response, is_valid_json = self.try_parse(top_msg)

            if not is_valid_json.success:
                self.logger.info(
                    f"Not valid json. Got \n\t==\n\t{top_msg}\n"
                    f"=\n\twhich could not be parsed.\n\terror:{is_valid_json.message}.\n\t=="
                )
                msg.append({"role": "assistant", "content": top_msg})
                msg.append(
                    {
                        "role": "user",
                        "content": INVALID_JSON.format(is_valid_json.message),
                    }
                )
                logs.append(is_valid_json.message)
                logs.append(top_msg)
                # continue

                if self.print_feedback or True:
                    print(logs[-1])

            plan, is_valid_plan = self.extract_plan(response["plan"])

            if not is_valid_plan.success:
                self.logger.info(
                    f"got: {response}\n\tnot valid plan: {is_valid_plan.message}"
                )
                msg.append({"role": "assistant", "content": top_msg})
                msg.append({"role": "user", "content": is_valid_plan.message})

                logs.append(top_msg)
                logs.append(is_valid_plan.message)

                if self.print_feedback or True:
                    print(logs[-1])
                continue

            if is_valid_json.success and is_valid_plan.success:
                success = True
                response["plan"] = plan
                break

        self.msg_history.append({"role": "assistant", "content": top_msg})

        return response, success, logs

    def request(self, request: str) -> Tuple[Dict[str, Any], bool, List[str]]:
        self.logger.info(f"Got Request: {request}")

        # on first request
        # TODO assumes first request is instruction. should update
        if self.base_request == "":
            self.base_request = request
        # adding to existing prompt
        else:
            current_request = {"role": "user", "content": request}
            self.msg_history.append(current_request)

        msg = (
            self.client.format_prompt(
                self.base_request, self.graph.initial_graph_as_json
            )
            + self.msg_history
        )

        response, success, logs = self._generate_plan(msg)

        return response, success, logs

    def resume_request(self) -> Tuple[Dict[str, Any], bool, List[str]]:
        assert len(self.most_recent_query), f"must have history to regenerate plan"
        return self._generate_plan(self.most_recent_query)

    def clear_history(self) -> None:
        self.msg_history = []

    def update(self, content: str) -> None:
        msg = {"role": "user", "content": content}

        self.msg_history.append(msg)


if __name__ == "__main__":
    import argparse

    # default not used
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, help="Instructions", required=True)
    parser.add_argument("--graph", type=str, help="input graph", default="")
    args = parser.parse_args()

    msg = args.task

    graph_handler = GraphHandler(graph_path=args.graph)
    planner = SPINE(graph=graph_handler)
    response, success, logs = planner.request(msg)

    print(f"success: {success}")

    # print(response)

    # for log in logs:
    #     print(log)

    for k, v in response.items():
        print(f"{k}: {v}")
        print("\n")
