import time
from typing import Dict, List, Tuple

import tiktoken
from openai import OpenAI
from spine_multi.spine.prompts.prompts import get_base_prompt_update_graph


class OpenAILLM:
    def __init__(self) -> None:
        """Wrapper for OpenAI"""
        self.client = OpenAI()
        self.model = "gpt-4o"
        self.token_encoder = tiktoken.get_encoding("cl100k_base")
        self.token_history = []
        self.time_history = []

    def query_llm(self, msg: str) -> Tuple[str, bool]:
        self.token_history.append(len(self.token_encoder.encode(str(msg))))
        self.most_recent_query = msg
        try:
            t1 = time.time()
            response = self.client.chat.completions.create(
                model=self.model,
                messages=msg,
                temperature=0.05,  # was 1
                max_tokens=2048,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0,
                response_format={"type": "json_object"},
            )
            top_msg = response.choices[0].message.content
            self.time_history.append(time.time() - t1)
            return top_msg, True
        except Exception as ex:
            return "Error: network dropout", False

    def format_prompt(self, base_request: str, graph_as_json: str) -> str:
        return get_base_prompt_update_graph(
            request=base_request, scene_graph=graph_as_json
        )
