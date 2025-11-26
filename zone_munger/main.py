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


with open('zone.tab', 'r') as zone_file:
    zone_data = zone_file.read()
root = build_timezone_tree(zone_data)
print_node(root)

"""
struct tz_node_t
{
    uint16_t name_offset;
    uint16_t is_leaf: 1;
    uint16_t pad: 15;
    union {
        struct {
            node *children;
            int num_children;
        };
        struct {
            float latitude;
            float longitude;
        };
    };
};


"""