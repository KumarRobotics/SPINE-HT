import re
from typing import Any, Dict, List, Tuple


def _parse_function_call(input_str: str) -> Tuple[str, List[str], Dict[str, str]]:
    try:
        # Match function name and arguments part
        match = re.match(r"(\w+)\((.*)\)", input_str.strip())
        if not match:
            raise ValueError("Invalid function call format")

        func_name, args_str = match.groups()

        # Split arguments (handle commas not in quotes)
        raw_args = []
        current = ""
        in_quotes = False

        for c in args_str:
            if c in ('"', "'"):
                in_quotes = not in_quotes
                current += c
            elif c == "," and not in_quotes:
                raw_args.append(current.strip())
                current = ""
            else:
                current += c
        if current:
            raw_args.append(current.strip())

        # Separate positional and keyword arguments
        pos_args = []
        kw_args = {}
        for arg in raw_args:
            if "=" in arg:
                key, value = map(str.strip, arg.split("=", 1))
                if not (value.startswith("'") or value.startswith('"')):
                    value = f"'{value}'"  # Quote unquoted string values
                kw_args[key] = eval(value)
            else:
                if not (arg.startswith("'") or arg.startswith('"')):
                    arg = f"'{arg}'"
                pos_args.append(eval(arg))
    except Exception as ex:
        raise ValueError(f"Could not parse input str: {input_str}. {ex}")

    return func_name, pos_args, kw_args


def parse_function_call(
    input_str: str, logging=None
) -> Tuple[str, List[str], Dict[str, str]]:
    try:
        return _parse_function_call(input_str=input_str)
    except Exception as ex:
        log_msg = f"caught exception {ex} from input {input_str}"
        print(log_msg)
        if logging != None:
            logging.info(log_msg)
