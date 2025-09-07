# define base API

# define base API
import importlib
import importlib.resources
import os

from spine_multi.decomp.examples import EXAMPLE
from spine_multi.decomp.prompts.gpt4_prompt import GPT4_POSTPEND, GPT4_PROMPT_TEMPLATE
from spine_multi.decomp.prompts.gpt5_prompt import GPT5_POSTPEND, GPT5_PROMPT_TEMPLATE

file_root = importlib.resources.files("spine_multi")
planning_api_path = file_root / "decomp/planning_api.py"
assert os.path.exists(planning_api_path), f"{planning_api_path} doesn't exist"


with open(planning_api_path) as f:
    planning_api = f.readlines()
planning_api = "".join(planning_api)

mapping_api_path = file_root / "decomp/mapping_api.py"
assert os.path.exists(mapping_api_path), f"{planning_api_path} doesn't exist"


with open(mapping_api_path) as f:
    mapping_api = f.readlines()
mapping_api = "".join(mapping_api[3:])


def build_prompt(team_specification: str, llm="gpt4") -> str:
    if llm == "gpt5":
        PROMPT_TEMPLATE = GPT5_PROMPT_TEMPLATE
        POSTPEND = GPT5_POSTPEND
    if llm == "gpt4":
        PROMPT_TEMPLATE = GPT4_PROMPT_TEMPLATE
        POSTPEND = GPT4_POSTPEND
    else:
        raise ValueError(f"{llm} not recognzied option")

    return (
        PROMPT_TEMPLATE.format(
            team_specification=team_specification,
            planning_api=planning_api,
            mapping_api=mapping_api,
        )
        # + EXAMPLE
        + POSTPEND
    )
