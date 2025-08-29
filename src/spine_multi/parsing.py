import re
import ast
from typing import List, Dict, Tuple, Any

class ListDictParser:
    """
    Parser for strings in the format: [{key: (value, type), ...}, {...}]
    Where each dict has structure: key: (value, value_type)
    """
    
    def __init__(self):
        # Pattern to match key: (value, type) pairs
        self.pair_pattern = r'(\w+):\s*\(([^,)]+),\s*([^)]+)\)'
    
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
        if not (input_string.startswith('[') and input_string.endswith(']')):
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
            
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    # End of current dictionary
                    dicts.append(current_dict.strip())
                    current_dict = ""
                    # Skip comma and whitespace
                    i += 1
                    while i < len(content) and content[i] in ', \n\t':
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
        if dict_str.startswith('{') and dict_str.endswith('}'):
            dict_str = dict_str[1:-1]
        
        parsed_dict = {}
        
        # Find all key: (value, type) pairs
        pairs = re.findall(self.pair_pattern, dict_str)
        
        for key, value_str, type_str in pairs:
            # Clean up the strings
            key = key.strip()
            value_str = value_str.strip()
            type_str = type_str.strip()
            
            # Convert value based on type
            parsed_value = self._convert_value(value_str, type_str)
            parsed_dict[key] = (parsed_value, type_str)
        
        return parsed_dict
    
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
        if (value_str.startswith('"') and value_str.endswith('"')) or \
           (value_str.startswith("'") and value_str.endswith("'")):
            value_str = value_str[1:-1]
        
        type_str = type_str.lower()
        
        try:
            if type_str in ['int', 'integer']:
                return int(value_str)
            elif type_str in ['float', 'double']:
                return float(value_str)
            elif type_str in ['bool', 'boolean']:
                return value_str.lower() in ['true', '1', 'yes']
            elif type_str in ['str', 'string']:
                return str(value_str)
            elif type_str in ['list', 'array']:
                return ast.literal_eval(value_str) if value_str else []
            elif type_str in ['dict', 'dictionary']:
                return ast.literal_eval(value_str) if value_str else {}
            else:
                # Default to string if type is unknown
                return str(value_str)
        except (ValueError, SyntaxError):
            # If conversion fails, return as string
            return str(value_str)
    
    def serialize(self, data: List[Dict[str, Tuple[Any, str]]]) -> str:
        """
        Serialize a list of dictionaries back to the string format.
        
        Args:
            data: List of dictionaries where values are (value, type) tuples
            
        Returns:
            String in format "[{key: (value, type), ...}, ...]"
        """
        try:
            dict_strings = []
            
            for dict_item in data:
                pairs = []
                for key, (value, type_str) in dict_item.items():
                    formatted_value = self._format_value(value, type_str)
                    pairs.append(f"{key}: ({formatted_value}, {type_str})")
                
                dict_str = "{" + ", ".join(pairs) + "}"
                dict_strings.append(dict_str)
            
            return "[" + ", ".join(dict_strings) + "]"
        except Exception as ex:
            raise ValueError(f"Exception in serialization for input: {data}: {ex}")
    
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
        
        if type_str in ['str', 'string']:
            # For strings, don't add quotes if they're simple identifiers
            if isinstance(value, str) and value.isidentifier():
                return value
            else:
                # Add quotes for strings with spaces or special characters
                return f'"{value}"' if isinstance(value, str) else str(value)
        elif type_str in ['bool', 'boolean']:
            return str(value).lower()
        elif type_str in ['list', 'array', 'dict', 'dictionary']:
            return str(value)
        else:
            return str(value)
        

# Example usage and test cases
if __name__ == "__main__":
    parser = ListDictParser()
    
    # Test cases based on your example
    test_cases = [
        '[{behavior: (navigate, str), x: (10, float), y: (10, float)}]',
        '[{id: (1, int), name: (Alice, str)}, {id: (2, int), name: (Bob, str)}]',
        '[{action: ("jump", str), height: (5.5, float), enabled: (true, bool)}]',
        '[{task: (cleanup, str), priority: (3, int), items: ([1,2,3], list)}]'
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"Test Case {i}:")
        print(f"Input: {test_case}")
        try:
            result = parser.parse(test_case)
            print(f"Output: {result}")
            print(f"Parsed types: {[(k, type(v[0]).__name__, v[1]) for d in result for k, v in d.items()]}")
            print()
        except Exception as e:
            print(f"Error: {e}")
            print()
    
    # Your specific example
    print("Your example:")
    sample_input = '[{behavior: (navigate, str), x: (10, float), y: (10, float)}]'
    print(f"Parsing: {sample_input}")
    
    result = parser.parse(sample_input)
    print(f"Result: {result}")
    
    # Access parsed data
    first_dict = result[0]
    print(f"First dictionary: {first_dict}")
    print(f"Behavior: {first_dict['behavior'][0]} (Python type: {type(first_dict['behavior'][0])})")
    print(f"X coordinate: {first_dict['x'][0]} (Python type: {type(first_dict['x'][0])})")
    print(f"Y coordinate: {first_dict['y'][0]} (Python type: {type(first_dict['y'][0])})")
    
    # Show how to extract just values or just types
    print("\nExtract just values:")
    values_only = {k: v[0] for k, v in first_dict.items()}
    print(f"Values: {values_only}")
    
    print("\nExtract just type info:")
    types_only = {k: v[1] for k, v in first_dict.items()}
    print(f"Types: {types_only}")
    
    # Test serialization
    print("\n" + "="*50)
    print("SERIALIZATION TESTS")
    print("="*50)
    
    # Test round-trip conversion
    print("Round-trip test:")
    original = '[{behavior: (navigate, str), x: (10, float), y: (10, float)}]'
    parsed = parser.parse(original)
    serialized = parser.serialize(parsed)
    print(f"Original:   {original}")
    print(f"Serialized: {serialized}")
    print(f"Match: {original == serialized}")
    print()
    
    # Test with multiple dictionaries
    print("Multiple dictionaries test:")
    multi_dict_data = [
        {'action': ('jump', 'str'), 'height': (5.5, 'float'), 'enabled': (True, 'bool')},
        {'action': ('run', 'str'), 'speed': (12, 'int'), 'enabled': (False, 'bool')}
    ]
    multi_serialized = parser.serialize(multi_dict_data)
    print(f"Serialized: {multi_serialized}")
    
    # Parse it back to verify
    multi_parsed = parser.parse(multi_serialized)
    print(f"Parsed back: {multi_parsed}")
    print()
    
    # Test with quoted strings
    print("Quoted strings test:")
    quoted_data = [{'message': ('Hello World!', 'str'), 'count': (42, 'int')}]
    quoted_serialized = parser.serialize(quoted_data)
    print(f"Serialized: {quoted_serialized}")
    
    # Test creating data from scratch
    print("Creating data from scratch:")
    custom_data = [
        {'task_id': (123, 'int'), 'description': ('Process files', 'str'), 'priority': (0.8, 'float')},
        {'task_id': (124, 'int'), 'description': ('Send emails', 'str'), 'priority': (0.3, 'float')}
    ]
    custom_serialized = parser.serialize(custom_data)
    print(f"Custom data serialized: {custom_serialized}")
    
    # Verify round-trip
    custom_parsed = parser.parse(custom_serialized)
    custom_re_serialized = parser.serialize(custom_parsed)
    print(f"Round-trip match: {custom_serialized == custom_re_serialized}")