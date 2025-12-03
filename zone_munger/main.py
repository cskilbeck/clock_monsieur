import re
import csv
import sys
from datetime import datetime
from pathlib import Path

# epoch times are stored as seconds since 2025/Jan/1 00:00:00
# using uint32_t we can go up to 2161/Feb/2 06:28:15 [ 6027436800 - 1735689600 = 4291747200 (0xffcedd80) ]
# but that's a lot of data so we'll go up to 2075

MIN_EPOCH = int(datetime(2025, 1, 1).timestamp())  # Jan 1 2025 = 1735689600
MAX_EPOCH = int(datetime(2075, 1, 1).timestamp())  # Jan 1 2075


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

    tree_root = {"name": "root", "children": {}}

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
            # name = component.replace('_', ' ')
            name = component
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

def parse_timezone_csv(csv_file_path: str) -> dict:
    """
    Parse the timezone CSV file and return a dictionary mapping locations to their timezone data.
    Returns:
        dict: {location: [{'epoch_time': int, 'offset_seconds': int}, ...]}
    """
    location_data = {}

    try:
        with open(csv_file_path, 'r') as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                if len(row) < 6:
                    continue

                location = row[0]
                # region = row[1]
                # zone_name = row[2]
                epoch_time = int(row[3])
                offset_seconds = int(row[4])
                # dst_active = int(row[5])

                if location not in location_data:
                    location_data[location] = []

                location_data[location].append({
                    'epoch_time': epoch_time,
                    'offset_seconds': offset_seconds
                })

    except FileNotFoundError:
        print(f"Warning: '{csv_file_path}' not found. No timezone offset data will be included.")
        return {}

    # Sort each location's data by epoch time
    for location in location_data:
        location_data[location].sort(key=lambda x: x['epoch_time'])

        # and remove everything outside range
        # but keep at least the last one regardless
        new_data = []
        for entry in location_data[location]:
            if MIN_EPOCH < entry['epoch_time'] < MAX_EPOCH:
                new_data.append(entry)
        if len(new_data) == 0:
            entry = location_data[location][-1].copy()
            entry['epoch_time'] = max(entry['epoch_time'], MIN_EPOCH)
            new_data.append(entry)
        location_data[location] = new_data

    return location_data


def build_timezone_tree_with_offsets(zone_tab_data: str, csv_file_path: str = 'zones.csv') -> dict:
    """
    Parse zone.tab data and build a tree structure with timezone offset data.
    """
    # First build the basic tree
    tree_root = build_timezone_tree(zone_tab_data)

    # Parse timezone offset data
    timezone_data = parse_timezone_csv(csv_file_path)

    # Add timezone data to leaf nodes
    def add_timezone_data(node, path_so_far=""):
        if "children" in node:
            # Internal node
            for name, child in node["children"].items():
                child_path = f"{path_so_far}/{name}" if path_so_far else name
                add_timezone_data(child, child_path)
        else:
            # Leaf node - add timezone data
            if path_so_far in timezone_data:
                node['timezone_offsets'] = timezone_data[path_so_far]
            else:
                node['timezone_offsets'] = []  # No timezone data available

    add_timezone_data(tree_root)
    return tree_root


def generate_cpp_header(tree_root: dict) -> tuple[str, str]:
    """
    Generates the content for a C++ header file containing
    the timezone tree data structures with separate lat/lon array and timezone offsets.
    """
    # 1. Flatten the tree and get the list of names
    _, flattened_nodes, name_list = flatten_tree(tree_root)
    name_string_total_len = sum(len(name) + 1 for name in name_list) + 1

    # 2. Create separate arrays for coordinates and timezone offsets
    coord_array = []
    offset_array = []
    coord_index = 0
    offset_index = 0

    # Map leaf nodes to coordinate and offset indices
    for node in flattened_nodes:
        if 'latitude' in node:  # This is a leaf node
            # Add coordinate data
            coord_array.append({
                'latitude': node['latitude'],
                'longitude': node['longitude']
            })

            # Add timezone offset data
            timezone_offsets = node.get('timezone_offsets', [])
            offset_start_index = offset_index

            for offset_data in timezone_offsets:
                offset_array.append({
                    'epoch_start_time_seconds': offset_data['epoch_time'],
                    'offset_seconds': offset_data['offset_seconds']
                })
                offset_index += 1

            # Store indices in the node
            node['coord_index'] = coord_index
            node['offset_start_index'] = offset_start_index
            node['offset_count'] = len(timezone_offsets)
            coord_index += 1

    num_offsets = len(offset_array)
    num_details = len(coord_array)
    num_nodes = len(flattened_nodes)

    # 3. C++ Structures and preamble
    header_content = f"""#pragma once

#include <cstdint>
#include <cstddef>

constexpr int64_t MIN_EPOCH = {MIN_EPOCH}; // Jan 1st 2025

// timezone offset data
struct zone_offset_t
{{
    uint16_t epoch_start_high;  // high 16 bits of epoch start time in seconds
    uint16_t epoch_start_low;   // low 16 bits of epoch start time in seconds
    int16_t offset_seconds_10;  // GMT offset in seconds * 10
}};

// timezone details (coordinates and offset info)
struct tz_details_t
{{
    uint16_t offset_start_index;  // Index into ZONE_OFFSETS array
    uint16_t offset_count;        // Number of offset entries for this location
}};

// timezone node
struct tz_node_t
{{
    uint16_t name_offset;     // Offset into TZ_NAME_STRING
    uint16_t num_children;    // # of child nodes (>0 = treenode, 0 =leafnode (zone)) 
    uint16_t children_index;  // For non-leaf: index into TZ_NODES, for leaf: index into TZ_DETAILS
}};

extern const char TZ_NAME_STRING[{name_string_total_len}];
extern const zone_offset_t ZONE_OFFSETS[{max(1, num_offsets)}];
extern const tz_details_t TZ_DETAILS[{num_details}];
extern const tz_node_t TZ_NODES[{num_nodes}];
"""

    # 4. Generate the Name String literal (up to 80 chars per line)
    formatted_string_literals = format_string_literal_concatenated(name_list, max_len=80)
    cpp_content = f'''
#include <cstdint>
#include "timezone_data.h"
const char TZ_NAME_STRING[{name_string_total_len}] = '''

    cpp_content += formatted_string_literals + ';\n\n'

    # 5. Generate the Zone Offset Array
    num_offsets = len(offset_array)
    cpp_content += f"// Array of timezone offset data. Total offsets: {num_offsets}\n"
    cpp_content += f"const zone_offset_t ZONE_OFFSETS[{max(1, num_offsets)}] = {{\n"

    if num_offsets == 0:
        cpp_content += "    { 0, 0 }  // Placeholder entry\n"
    else:
        count = 0
        for offset in offset_array:
            absolute_epoch_time = offset['epoch_start_time_seconds']
            epoch_time = absolute_epoch_time - MIN_EPOCH
            epoch_time_u = epoch_time & 0xffffffff
            epoch_time_low = epoch_time_u & 0xffff
            epoch_time_high = (epoch_time_u >> 16) & 0xffff
            offset_seconds = offset['offset_seconds']
            cpp_content += f"    {{ 0x{epoch_time_high:04x}U, 0x{epoch_time_low:04x}U, {offset_seconds // 10} }}, // [{count}] = {absolute_epoch_time}, {offset_seconds}\n"
            count += 1
        cpp_content = cpp_content.rstrip(',\n') + "\n"

    cpp_content += "};\n\n"

    # 6. Generate the Details Array
    cpp_content += f"// Array of timezone details for leaf nodes. Total details: {num_details}\n"
    cpp_content += f"const tz_details_t TZ_DETAILS[{num_details}] = {{\n"

    for i, coord in enumerate(coord_array):
        lat_str = f"{coord['latitude']:.5f}"
        lon_str = f"{coord['longitude']:.5f}"

        # Find the corresponding node to get offset info
        offset_start_index = 0
        offset_count = 0
        for node in flattened_nodes:
            if node.get('coord_index') == i:
                offset_start_index = node.get('offset_start_index', 0)
                offset_count = node.get('offset_count', 0)
                break

        cpp_content += f"    {{ {offset_start_index}, {offset_count} }},\n"

    cpp_content = cpp_content.rstrip(',\n') + "\n};\n\n"

    # 7. Generate the Node Array
    cpp_content += f"// Array of all timezone nodes. Total nodes: {num_nodes}\n"
    cpp_content += f"const tz_node_t TZ_NODES[{num_nodes}] = {{\n"

    for node in flattened_nodes:
        name_offset = node['name_offset']
        node_name = node.get('name', 'ROOT_CHILDREN')

        # Determine if this is a leaf (num_children == 0)
        if 'latitude' in node:
            # Leaf node: num_children = 0, children_index points to details array
            num_children = 0
            children_index = node['coord_index']
        else:
            # Non-leaf node: has children
            num_children = node.get('num_children', 0)
            children_index = node.get('children_index', 0)

        cpp_content += f"    {{ {name_offset}, {num_children}, {children_index} }},\t// {node_name}\n"

    cpp_content = cpp_content.rstrip(',\n') + "\n};\n"

    return header_content, cpp_content

with open('zone.tab', 'r') as zone_file:
    zone_data = zone_file.read()

root = build_timezone_tree_with_offsets(zone_data)
header, cpp = generate_cpp_header(root)

output_path = Path("../firmware/main")
if len(sys.argv) == 2:
    output_path = Path(sys.argv[1])

header_path = output_path / "timezone_data.h"
cpp_path = output_path / "timezone_data.cpp"

with open(header_path, "w") as f:
    f.write(header)
    print(f"Header written to {header_path}")

with open(cpp_path, "w") as f:
    f.write(cpp)
    print(f"CPP written to {cpp_path}")
