import ast
import json
from typing import Any, Dict, List, Tuple


class ListDictParser:
    """
    Parser for strings in the format: [{key: (value, type), ...}, {...}]
    Where each dict has structure: key: (value, value_type)
    """

    def __init__(self):
        # Pattern to match key: (value, type) pairs
        self.pair_pattern = r"(\w+):\s*\(([^,)]+),\s*([^)]+)\)"

    def parse(self, input_string: str) -> List[Dict[str, Tuple[Any, str]]]:
        """
        Parse the input string and return a list of dictionaries.

        Args:
            input_string: String in format "[{key1: (value1, type1), key2: (value2, type2)}, ...]"

        Returns:
            List of dictionaries with parsed values
        """
        input_string = input_string.strip()

        # Remove outer brackets
        if not (input_string.startswith("[") and input_string.endswith("]")):
            raise ValueError("Input string must be wrapped in square brackets []")

        content = input_string[1:-1].strip()

        # Split by dictionaries using a simple approach
        # Find dictionary boundaries by matching braces
        dicts = self._split_dictionaries(content)

        result = []
        for dict_str in dicts:
            parsed_dict = self._parse_dict(dict_str)
            result.append(parsed_dict)

        return result

    def _split_dictionaries(self, content: str) -> List[str]:
        """
        Split the content into individual dictionary strings.
        """
        dicts = []
        current_dict = ""
        brace_count = 0
        i = 0

        while i < len(content):
            char = content[i]
            current_dict += char

            if char == "{":
                brace_count += 1
            elif char == "}":
                brace_count -= 1
                if brace_count == 0:
                    # End of current dictionary
                    dicts.append(current_dict.strip())
                    current_dict = ""
                    # Skip comma and whitespace
                    i += 1
                    while i < len(content) and content[i] in ", \n\t":
                        i += 1
                    continue

            i += 1

        return dicts

    def _parse_dict(self, dict_str: str) -> Dict[str, Tuple[Any, str]]:
        """
        Parse a single dictionary string.

        Args:
            dict_str: Dictionary string like "{key1: (value1, type1), key2: (value2, type2)}"

        Returns:
            Dictionary with parsed key-value pairs
        """
        # Remove outer braces
        dict_str = dict_str.strip()
        if dict_str.startswith("{") and dict_str.endswith("}"):
            dict_str = dict_str[1:-1]

        parsed_dict = {}

        # Use a more sophisticated approach to split key-value pairs
        pairs = self._extract_key_value_pairs(dict_str)

        for key, value_str, type_str in pairs:
            # Clean up the strings
            key = key.strip()
            value_str = value_str.strip()
            type_str = type_str.strip()

            # Convert value based on type
            parsed_value = self._convert_value(value_str, type_str)
            parsed_dict[key] = (parsed_value, type_str)

        return parsed_dict

    def _extract_key_value_pairs(self, content: str) -> List[Tuple[str, str, str]]:
        """
        Extract key-value pairs from dictionary content, handling nested structures.
        """
        pairs = []
        i = 0

        while i < len(content):
            # Skip whitespace and commas
            while i < len(content) and content[i] in " ,\n\t":
                i += 1

            if i >= len(content):
                break

            # Find the key (everything up to the colon)
            key_start = i
            while i < len(content) and content[i] != ":":
                i += 1

            if i >= len(content):
                break

            key = content[key_start:i].strip()
            i += 1  # Skip the colon

            # Skip whitespace after colon
            while i < len(content) and content[i] in " \n\t":
                i += 1

            # Expect opening parenthesis
            if i >= len(content) or content[i] != "(":
                break

            i += 1  # Skip opening parenthesis

            # Find the value (everything up to the last comma before closing parenthesis)
            value_start = i
            paren_count = 1
            brace_count = 0
            bracket_count = 0
            in_string = False
            quote_char = None

            while i < len(content) and paren_count > 0:
                char = content[i]

                if not in_string:
                    if char in "\"'":
                        in_string = True
                        quote_char = char
                    elif char == "(":
                        paren_count += 1
                    elif char == ")":
                        paren_count -= 1
                    elif char == "{":
                        brace_count += 1
                    elif char == "}":
                        brace_count -= 1
                    elif char == "[":
                        bracket_count += 1
                    elif char == "]":
                        bracket_count -= 1
                else:
                    if char == quote_char and (i == 0 or content[i - 1] != "\\"):
                        in_string = False
                        quote_char = None

                i += 1

            # Extract the content inside parentheses
            paren_content = content[value_start : i - 1]

            # Find the last comma that's not inside nested structures
            last_comma_pos = self._find_last_top_level_comma(paren_content)

            if last_comma_pos == -1:
                # No comma found, might be malformed
                continue

            value_str = paren_content[:last_comma_pos].strip()
            type_str = paren_content[last_comma_pos + 1 :].strip()

            pairs.append((key, value_str, type_str))

        return pairs

    def _find_last_top_level_comma(self, content: str) -> int:
        """
        Find the position of the last comma that's not inside nested structures.
        """
        last_comma = -1
        depth = 0
        in_string = False
        quote_char = None

        for i, char in enumerate(content):
            if not in_string:
                if char in "\"'":
                    in_string = True
                    quote_char = char
                elif char in "({[":
                    depth += 1
                elif char in ")}]":
                    depth -= 1
                elif char == "," and depth == 0:
                    last_comma = i
            else:
                if char == quote_char and (i == 0 or content[i - 1] != "\\"):
                    in_string = False
                    quote_char = None

        return last_comma

    def _convert_value(self, value_str: str, type_str: str) -> Any:
        """
        Convert string value to appropriate Python type.

        Args:
            value_str: String representation of the value
            type_str: String representation of the type

        Returns:
            Converted value
        """
        # Remove quotes if present
        if (value_str.startswith('"') and value_str.endswith('"')) or (
            value_str.startswith("'") and value_str.endswith("'")
        ):
            value_str = value_str[1:-1]

        type_str = type_str.lower()

        try:
            if type_str in ["int", "integer"]:
                return int(value_str)
            elif type_str in ["float", "double"]:
                return float(value_str)
            elif type_str in ["bool", "boolean"]:
                return value_str.lower() in ["true", "1", "yes"]
            elif type_str in ["str", "string"]:
                return str(value_str)
            elif type_str in ["list", "array"]:
                return ast.literal_eval(value_str) if value_str else []
            elif type_str in ["dict", "dictionary"]:
                return ast.literal_eval(value_str) if value_str else {}
            elif type_str == "graph":
                return self._parse_graph_string(value_str)
            else:
                # Default to string if type is unknown
                return str(value_str)
        except (ValueError, SyntaxError):
            # If conversion fails, return as string
            return str(value_str)

    def _parse_graph_string(self, graph_str: str) -> Dict[str, Any]:
        """
        Parse a graph string (complex dictionary format).

        Args:
            graph_str: String representation of a graph dictionary

        Returns:
            Parsed dictionary representing the graph
        """
        try:
            # First try ast.literal_eval (safest for Python literals)
            return ast.literal_eval(graph_str)
        except (ValueError, SyntaxError):
            try:
                # If that fails, try converting single quotes to double quotes for JSON
                json_string = graph_str.replace("'", '"')
                return json.loads(json_string)
            except json.JSONDecodeError as e:
                raise ValueError(f"Unable to parse graph string: {e}")

    def parse_complex_dict(self, input_string: str) -> Dict[str, Any]:
        """
        Parse a complex dictionary string (JSON-like format with Python syntax).

        Args:
            input_string: String representation of a dictionary

        Returns:
            Parsed dictionary
        """
        try:
            # First try direct ast.literal_eval (safest)
            return ast.literal_eval(input_string)
        except (ValueError, SyntaxError):
            try:
                # If that fails, try converting single quotes to double quotes for JSON
                json_string = input_string.replace("'", '"')
                return json.loads(json_string)
            except json.JSONDecodeError:
                # If both fail, try manual parsing
                return self._manual_dict_parse(input_string)

    def _manual_dict_parse(self, input_string: str) -> Dict[str, Any]:
        """
        Manually parse a dictionary string when ast.literal_eval fails.
        This is a fallback method for complex cases.
        """
        # Remove outer braces and whitespace
        content = input_string.strip()
        if content.startswith("{") and content.endswith("}"):
            content = content[1:-1]

        result = {}

        # Split by top-level commas (this is simplified and may need refinement)
        items = self._split_top_level_items(content)

        for item in items:
            if ":" in item:
                key_part, value_part = item.split(":", 1)
                key = key_part.strip().strip("'\"")
                value = self._parse_value(value_part.strip())
                result[key] = value

        return result

    def _split_top_level_items(self, content: str) -> List[str]:
        """
        Split content by top-level commas, respecting nested structures.
        """
        items = []
        current_item = ""
        depth = 0
        in_string = False
        quote_char = None

        i = 0
        while i < len(content):
            char = content[i]

            if not in_string:
                if char in "\"'":
                    in_string = True
                    quote_char = char
                elif char in "[{(":
                    depth += 1
                elif char in "]})":
                    depth -= 1
                elif char == "," and depth == 0:
                    items.append(current_item.strip())
                    current_item = ""
                    i += 1
                    continue
            else:
                if char == quote_char and (i == 0 or content[i - 1] != "\\"):
                    in_string = False
                    quote_char = None

            current_item += char
            i += 1

        if current_item.strip():
            items.append(current_item.strip())

        return items

    def _parse_value(self, value_str: str) -> Any:
        """
        Parse a value string into appropriate Python type.
        """
        value_str = value_str.strip()

        try:
            # Try ast.literal_eval first
            return ast.literal_eval(value_str)
        except (ValueError, SyntaxError):
            # If that fails, return as string (removing quotes if present)
            if (value_str.startswith('"') and value_str.endswith('"')) or (
                value_str.startswith("'") and value_str.endswith("'")
            ):
                return value_str[1:-1]
            return value_str

    def serialize_complex_dict(self, data: Dict[str, Any], pretty: bool = False) -> str:
        """
        Serialize a complex dictionary back to string format.

        Args:
            data: Dictionary to serialize
            pretty: Whether to format with indentation

        Returns:
            String representation of the dictionary
        """
        if pretty:
            return self._format_dict_pretty(data, indent=0)
        else:
            return str(data)

    def _format_dict_pretty(self, obj: Any, indent: int = 0) -> str:
        """
        Format dictionary with pretty printing.
        """
        indent_str = "  " * indent
        next_indent_str = "  " * (indent + 1)

        if isinstance(obj, dict):
            if not obj:
                return "{}"

            lines = ["{"]
            items = list(obj.items())

            for i, (key, value) in enumerate(items):
                formatted_value = self._format_dict_pretty(value, indent + 1)
                comma = "," if i < len(items) - 1 else ""
                lines.append(f"{next_indent_str}'{key}': {formatted_value}{comma}")

            lines.append(f"{indent_str}}}")
            return "\n".join(lines)

        elif isinstance(obj, list):
            if not obj:
                return "[]"

            lines = ["["]
            for i, item in enumerate(obj):
                formatted_item = self._format_dict_pretty(item, indent + 1)
                comma = "," if i < len(obj) - 1 else ""
                lines.append(f"{next_indent_str}{formatted_item}{comma}")

            lines.append(f"{indent_str}]")
            return "\n".join(lines)

        elif isinstance(obj, str):
            return f"'{obj}'"
        else:
            return str(obj)

    def _format_graph(self, graph_data: Dict[str, Any]) -> str:
        """
        Format graph data for serialization.
        """
        return str(graph_data)

    def serialize(self, data: List[Dict[str, Tuple[Any, str]]]) -> str:
        """
        Serialize a list of dictionaries back to the string format.

        Args:
            data: List of dictionaries where values are (value, type) tuples

        Returns:
            String in format "[{key: (value, type), ...}, ...]"
        """
        dict_strings = []

        for dict_item in data:
            pairs = []
            for key, (value, type_str) in dict_item.items():
                formatted_value = self._format_value(value, type_str)
                pairs.append(f"{key}: ({formatted_value}, {type_str})")

            dict_str = "{" + ", ".join(pairs) + "}"
            dict_strings.append(dict_str)

        return "[" + ", ".join(dict_strings) + "]"

    def _format_value(self, value: Any, type_str: str) -> str:
        """
        Format a value for serialization based on its type.

        Args:
            value: The value to format
            type_str: The type string

        Returns:
            Formatted string representation of the value
        """
        type_str = type_str.lower()

        if type_str in ["str", "string"]:
            # For strings, don't add quotes if they're simple identifiers
            if isinstance(value, str) and value.isidentifier():
                return value
            else:
                # Add quotes for strings with spaces or special characters
                return f'"{value}"' if isinstance(value, str) else str(value)
        elif type_str in ["bool", "boolean"]:
            return str(value).lower()
        elif type_str in ["list", "array", "dict", "dictionary"]:
            return str(value)
        elif type_str == "graph":
            return self._format_graph(value)
        else:
            return str(value)


# Example usage and test cases
if __name__ == "__main__":
    parser = ListDictParser()

    # Test cases based on your example
    test_cases = [
        "[{behavior: (navigate, str), x: (10, float), y: (10, float)}]",
        "[{id: (1, int), name: (Alice, str)}, {id: (2, int), name: (Bob, str)}]",
        '[{action: ("jump", str), height: (5.5, float), enabled: (true, bool)}]',
        "[{task: (cleanup, str), priority: (3, int), items: ([1,2,3], list)}]",
    ]

    for i, test_case in enumerate(test_cases, 1):
        print(f"Test Case {i}:")
        print(f"Input: {test_case}")
        try:
            result = parser.parse(test_case)
            print(f"Output: {result}")
            print(
                f"Parsed types: {[(k, type(v[0]).__name__, v[1]) for d in result for k, v in d.items()]}"
            )
            print()
        except Exception as e:
            print(f"Error: {e}")
            print()

    # Your specific example
    print("Your example:")
    sample_input = "[{behavior: (navigate, str), x: (10, float), y: (10, float)}]"
    print(f"Parsing: {sample_input}")

    result = parser.parse(sample_input)
    print(f"Result: {result}")

    # Access parsed data
    first_dict = result[0]
    print(f"First dictionary: {first_dict}")
    print(
        f"Behavior: {first_dict['behavior'][0]} (Python type: {type(first_dict['behavior'][0])})"
    )
    print(
        f"X coordinate: {first_dict['x'][0]} (Python type: {type(first_dict['x'][0])})"
    )
    print(
        f"Y coordinate: {first_dict['y'][0]} (Python type: {type(first_dict['y'][0])})"
    )

    # Show how to extract just values or just types
    print("\nExtract just values:")
    values_only = {k: v[0] for k, v in first_dict.items()}
    print(f"Values: {values_only}")

    print("\nExtract just type info:")
    types_only = {k: v[1] for k, v in first_dict.items()}
    print(f"Types: {types_only}")

    # Test serialization
    print("\n" + "=" * 50)
    print("SERIALIZATION TESTS")
    print("=" * 50)

    # Test round-trip conversion
    print("Round-trip test:")
    original = "[{behavior: (navigate, str), x: (10, float), y: (10, float)}]"
    parsed = parser.parse(original)
    serialized = parser.serialize(parsed)
    print(f"Original:   {original}")
    print(f"Serialized: {serialized}")
    print(f"Match: {original == serialized}")
    print()

    # Test with multiple dictionaries
    print("Multiple dictionaries test:")
    multi_dict_data = [
        {
            "action": ("jump", "str"),
            "height": (5.5, "float"),
            "enabled": (True, "bool"),
        },
        {"action": ("run", "str"), "speed": (12, "int"), "enabled": (False, "bool")},
    ]
    multi_serialized = parser.serialize(multi_dict_data)
    print(f"Serialized: {multi_serialized}")

    # Parse it back to verify
    multi_parsed = parser.parse(multi_serialized)
    print(f"Parsed back: {multi_parsed}")
    print()

    # Test with graph type
    print("Graph type test:")
    graph_string = "{'objects': [{'name': 'desk_1', 'coords': '[3.0, 5.0]'}], 'regions': [{'name': 'region_1', 'coords': '[0.0, 0.0]'}, {'name': 'region_2', 'coords': '[2.0, 0.0]'}, {'name': 'region_3', 'coords': '[3.0, 1.0]'}], 'object_connections': ['desk_1, region_3'], 'region_connections': ['region_1, region_2', 'region_2, region_3'], 'current_location': 'ground_1'}"

    # Create test data with graph type
    test_with_graph = f"[{{map_data: ({graph_string}, graph), level: (1, int)}}]"
    print("Testing graph parsing:")
    try:
        graph_result = parser.parse(test_with_graph)
        print("Successfully parsed graph!")
        print(f"Graph data type: {type(graph_result[0]['map_data'][0])}")
        print(f"Graph keys: {list(graph_result[0]['map_data'][0].keys())}")
        print(f"Number of objects: {len(graph_result[0]['map_data'][0]['objects'])}")
        print(f"Number of regions: {len(graph_result[0]['map_data'][0]['regions'])}")
        print(
            f"Level: {graph_result[0]['level'][0]} (type: {type(graph_result[0]['level'][0])})"
        )

        # Test serialization back
        serialized_back = parser.serialize(graph_result)
        print(f"Serialized back successfully: {len(serialized_back)} characters")

    except Exception as e:
        print(f"Graph parsing error: {e}")
        import traceback

        traceback.print_exc()
    print()

    # Test with graph type
    print("Graph type test:")
    graph_data = {
        "objects": [{"name": "desk_1", "coords": "[3.0, 5.0]"}],
        "regions": [
            {"name": "region_1", "coords": "[0.0, 0.0]"},
            {"name": "region_2", "coords": "[2.0, 0.0]"},
            {"name": "region_3", "coords": "[3.0, 1.0]"},
        ],
        "object_connections": ["desk_1, region_3"],
        "region_connections": ["region_1, region_2", "region_2, region_3"],
        "current_location": "ground_1",
    }

    # Test parsing a graph string
    graph_string = str(graph_data)
    test_with_graph = f"[{{map_data: ({graph_string}, graph), level: (1, int)}}]"
    print("Testing graph parsing:")
    try:
        graph_result = parser.parse(test_with_graph)
        print("Successfully parsed graph!")
        print(f"Graph data type: {type(graph_result[0]['map_data'][0])}")
        print(f"Graph keys: {list(graph_result[0]['map_data'][0].keys())}")
        print(f"Number of objects: {len(graph_result[0]['map_data'][0]['objects'])}")
        print(f"Number of regions: {len(graph_result[0]['map_data'][0]['regions'])}")
    except Exception as e:
        print(f"Graph parsing error: {e}")
    print()

    # Test creating data from scratch
    print("Creating data from scratch:")
    custom_data = [
        {
            "task_id": (123, "int"),
            "description": ("Process files", "str"),
            "priority": ("hi", "list[str]"),
        },
        {
            "task_id": (124, "int"),
            "description": ("Send emails", "str"),
            "priority": (0.3, "float"),
        },
    ]
    custom_serialized = parser.serialize(custom_data)
    print(f"Custom data serialized: {custom_serialized}")

    # Verify round-trip
    custom_parsed = parser.parse(custom_serialized)
    custom_re_serialized = parser.serialize(custom_parsed)
    print(f"Round-trip match: {custom_serialized == custom_re_serialized}")
