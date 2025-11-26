import re


def decimal_from_tz_coord(coord_str: str) -> float:
    """convert tz coordinate to decimal coordinate"""
    if coord_str.startswith('+'):
        sign = 1.0
        coord_str = coord_str[1:]
    elif coord_str.startswith('-'):
        sign = -1.0
        coord_str = coord_str[1:]
    else:
        raise ValueError(f'Invalid time zone string: {coord_str} - no leading +/-')

    str_len = len(coord_str)

    if str_len == 4:
        degrees = int(coord_str[:2])
        minutes = int(coord_str[2:])
        seconds = 0
    elif str_len == 5:
        degrees = int(coord_str[:3])
        minutes = int(coord_str[3:])
        seconds = 0
    elif str_len == 6:
        degrees = int(coord_str[:2])
        minutes = int(coord_str[2:4])
        seconds = int(coord_str[4:])
    elif str_len == 7:
        degrees = int(coord_str[:3])
        minutes = int(coord_str[3:5])
        seconds = int(coord_str[5:])
    else:
        raise ValueError(f'Invalid time zone string: {coord_str} - length must be 4..7')
    return (degrees + minutes / 60 + seconds / 3600) * sign


def build_timezone_tree(zone_tab_data: str) -> dict:
    """
    Parse zone.tab data and build a tree structure based on zone names
    """
    line_regex = re.compile(r"""
        ^
        [A-Z]{2}              # CODE (Ignored)
        \t
        ([+-]\d+)([+-]\d+)    # Group 1: LAT, Group 2: LON
        \t
        ([^\t]+)              # Group 3: NAME
        (?: \t [^\n]* )?      # COMMENT (Ignored)
        $
    """, re.VERBOSE)

    tree_root = {"name": None, "children": {}}

    for line in zone_tab_data.splitlines():
        if not line.strip() or line.startswith('#'):
            continue

        match = line_regex.search(line)
        if not match:
            continue

        lat_str, lon_str, name = match.groups()
        latitude = decimal_from_tz_coord(lat_str)
        longitude = decimal_from_tz_coord(lon_str)
        path_components = name.split('/')
        current_node = tree_root
        for i, component in enumerate(path_components):
            name = component.replace('_', ' ')
            # If the component is not in the current node's children, create it
            if name not in current_node['children']:
                new_node = {
                    "name": name,
                    "children": {}
                }
                current_node['children'][name] = new_node

            current_node = current_node['children'][name]

            # If this is the last component of the name (i.e., the leaf node)
            if i == len(path_components) - 1:
                current_node['latitude'] = latitude
                current_node['longitude'] = longitude
                del current_node['children']  # leaf nodes only store coordinates
    return tree_root


def print_node(node, indent=0):
    if "children" in node:
        if node["name"]:
            print(f'{" " * indent}>{node["name"]}')
            indent += 4
        for name, child in sorted(node["children"].items()):
            print_node(child, indent)
    else:
        print(f'{" " * (indent + 1)}{node["name"]} ({node["latitude"]:.4f}, {node["longitude"]:.4f})')


# --- EXTENDED CODE STARTS HERE ---

def flatten_tree(root_node: dict) -> tuple[dict, list[dict], list[str]]:
    """
    Traverses the tree, assigns an index and a name offset to each node,
    and flattens it into a list for C++ array generation.

    Returns: (root_node_with_index, flattened_nodes, name_list)
    """
    node_index = 0
    name_offset = 0
    name_list = []  # Stores names as a list of strings
    flattened_nodes = []

    # The queue holds (node) tuples
    queue = [root_node]

    while queue:
        current_node = queue.pop(0)

        # Skip the root node itself as it has no C++ structure representation
        if current_node.get('name') is None:
            # Add root's children to queue, sorted
            sorted_children = sorted(current_node.get('children', {}).items())
            child_nodes = []
            for _, child_node in sorted_children:
                child_nodes.append(child_node)
                queue.append(child_node)

            # Store children list for linking later
            current_node['children_list'] = child_nodes
            continue

        # --- Node Processing ---

        # 1. Assign index
        current_node['index'] = node_index
        node_index += 1

        # 2. Assign name offset and build name list
        current_node['name_offset'] = name_offset
        node_name = current_node['name']
        name_list.append(node_name)
        # Note: We calculate offset here, including the \0 which will be added later
        name_offset += len(node_name) + 1

        # 3. Handle Children / Coordinates
        is_leaf = 'children' not in current_node

        if not is_leaf:
            # Sort children for deterministic array order
            sorted_children = sorted(current_node.get('children', {}).items())

            child_nodes = []
            for _, child_node in sorted_children:
                child_nodes.append(child_node)
                queue.append(child_node)

            current_node['children_list'] = child_nodes

        # 4. Add to flattened list
        flattened_nodes.append(current_node)

    # 5. Link children pointers (indices) in the flattened list
    for node in flattened_nodes:
        if 'children_list' in node:
            if node['children_list']:
                node['children_index'] = node['children_list'][0]['index']
            else:
                node['children_index'] = 0
            node['num_children'] = len(node['children_list'])

        # Clean up transient data
        node.pop('children_list', None)
        node.pop('children', None)

        # Return the list of names, not the joined string
    return root_node, flattened_nodes, name_list


def format_string_literal_concatenated(name_list: list[str], max_len: int = 80) -> str:
    """
    Formats the list of names into concatenated C++ string literals,
    up to a maximum length per line.
    """
    output_lines = []
    current_line = ""

    # We use "\\0" to represent the null terminator in the C++ literal
    null_term_literal = "\\0"

    for name in name_list:
        # The content to append: "Name\0"
        segment = name + null_term_literal

        # Check if adding the segment exceeds the max length
        if current_line and len(current_line) + len(segment) > max_len:
            # If it doesn't fit, close the current literal and start a new one
            output_lines.append(f'"{current_line}"')
            current_line = segment
        else:
            # If it fits, append it to the current line
            current_line += segment

    # Add the final, non-empty line
    if current_line:
        output_lines.append(f'"{current_line}"')

    # Join all literals using the C++ concatenation format: \n"..." \
    return ' \\\n'.join(output_lines)


def generate_cpp_header(tree_root: dict, header_filename: str) -> str:
    """
    Generates the content for a C++ header file containing
    the timezone tree data structures.
    """
    # 1. Flatten the tree and get the list of names
    _, flattened_nodes, name_list = flatten_tree(tree_root)
    name_string_total_len = sum(len(name) + 1 for name in name_list)

    # 2. C++ Structures and preamble
    cpp_content = f"""#pragma once

#include <cstdint>
#include <cstddef>

// Constants for the is_leaf field
#define NODE 0
#define LEAF 1

// Define the structure for a timezone node
struct tz_node_t
{{
    uint16_t name_offset;
    uint16_t is_leaf: 1;
    uint16_t pad: 15;
    union {{
        struct {{
            // 'children_index' is an index into the TZ_NODES array
            int children_index; 
            int num_children;
        }};
        struct {{
            float latitude;
            float longitude;
        }};
    }};
}};

// The big string containing all timezone names, null-separated.
// Total size (including final null): {name_string_total_len} bytes.
const char TZ_NAME_STRING[] = \\
"""

    # 3. Generate the Name String literal (up to 80 chars per line)

    formatted_string_literals = format_string_literal_concatenated(name_list, max_len=80)

    cpp_content += formatted_string_literals + ';\n\n'

    # 4. Generate the Node Array
    num_nodes = len(flattened_nodes)
    cpp_content += f"// Array of all timezone nodes. Total nodes: {num_nodes}\n"
    cpp_content += f"const tz_node_t TZ_NODES[{num_nodes}] = {{\n"

    # The C++ structure expects a name_offset, is_leaf, and a union for children/coords.
    for node in flattened_nodes:
        # Common fields
        name_offset = node['name_offset']

        # Use the defined constants for is_leaf
        is_leaf_val = "LEAF" if 'latitude' in node else "NODE"

        # Get the node name for the inline comment
        node_name = node.get('name', 'ROOT_CHILDREN')

        union_content = ""
        if is_leaf_val == "LEAF":
            # Leaf: coordinates (using literal floats, formatted to max 5 digits of precision)
            latitude = node['latitude']
            longitude = node['longitude']

            # Use string formatting to limit precision
            lat_str = f"{latitude:.5f}"
            lon_str = f"{longitude:.5f}"

            # struct { float latitude; float longitude; };
            union_content = f"{{ {lat_str}f, {lon_str}f }}"

        else:
            # Non-leaf: children array index and count
            children_index = node.get('children_index', 0)
            num_children = node.get('num_children', 0)

            # struct { int children_index; int num_children; };
            union_content = f"{{ {children_index}, {num_children} }}"

        # The full node initializer: { name_offset, is_leaf:1, pad:15, union_content }, // Node Name
        # Note the use of '\t' to separate the struct initializer from the comment for cleaner alignment
        cpp_content += f"    {{ {name_offset}, {is_leaf_val}, 0, {union_content} }},\t// {node_name}\n"

    cpp_content = cpp_content.rstrip(',\n') + "\n};\n"

    return cpp_content


# --- Original Example Usage (Modified to call the new function) ---

# Read the zone.tab content (assumes 'zone.tab' exists)
try:
    with open('zone.tab', 'r') as zone_file:
        zone_data = zone_file.read()
except FileNotFoundError:
    # If zone.tab isn't available for testing, use a small sample
    print("Warning: 'zone.tab' not found. Using sample data for demonstration.")
    # Added more entries to test the 80-char wrapping
    zone_data = """
#version=2024b
# Last updated: 2024-03-09
#
# Country Code	Coordinates	TZ	Comments
US	+4046-07400	America/New_York
US	+4151-08739	America/Chicago
RU	+5957+03018	Europe/Moscow
DE	+5230+01322	Europe/Berlin
UK	+5130-00007	Europe/London
FR	+4852+00220	Europe/Paris
JP	+3541+13944	Asia/Tokyo
CN	+3954+11623	Asia/Shanghai
TZ	+0648+03917	Africa/Dar_es_Salaam
KE	+0117+03649	Africa/Nairobi
BR	-1547-04753	America/Sao_Paulo
AR	-3436-05822	America/Argentina/Buenos_Aires
CL	-3327-07040	America/Santiago
"""

root = build_timezone_tree(zone_data)
# print_node(root) # Uncomment to see the tree structure

# Generate the C++ header file content
header_content = generate_cpp_header(root, "timezone_data.h")

# Print to console or save to file
print("\n" + "=" * 80)
print("Generated C++ Header Content (timezone_data.h) - Final Version with 5-Digit Float Precision:")
print("=" * 80)
print(header_content)

# Optional: Save to file
with open("timezone_data.h", "w") as f:
    f.write(header_content)
print("\nData successfully written to timezone_data.h")
