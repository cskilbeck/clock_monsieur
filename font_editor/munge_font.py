import json
import sys
import argparse
from pathlib import Path
from typing import Dict, List, Tuple

# --- Constants ---
START_CHAR = 32
END_CHAR = 127
NUM_CHARS = END_CHAR - START_CHAR + 1


# --- Utility Functions ---

def calculate_widths(pixels: List[int], width: int, height: int) -> Tuple[int, int]:
    left = width  # Represents min_x (leftmost column index)
    right = -1  # Represents max_x (rightmost column index)

    for row_index in range(height):
        for col_index in range(width):
            if pixels[row_index * width + col_index] == 1:
                left = min(left, col_index)
                right = max(right, col_index)

    bits_wide = right - left + 1

    # Handle characters with no set pixels (like space ' ')
    if left == width:
        return 0,0
    return 8 - (width - left), bits_wide


def convert_to_uint8_t_rows(pixels: List[int], width: int, height: int, shift : int) -> List[int]:
    """
    Converts the flat pixel list into a list of uint8_t values (one per row),
    with data **right-aligned** (stored on the low bits).
    """
    uint8_t_rows = []

    for row_index in range(height):
        row_value = 0

        for col_index in range(width):
            pixel = pixels[row_index * width + col_index]
            if pixel == 1:
                # Bit position for right-alignment (Col 0 -> bit 4, Col 4 -> bit 0 for Width 5)
                bit_position = width - 1 - col_index
                row_value |= (1 << bit_position)

        # bits are left aligned (in the high bits)
        row_value <<= shift

        uint8_t_rows.append(row_value)

    return uint8_t_rows


# --- C/C++ File Generation ---

def generate_c_files(font_data: Dict, font_name: str, output_path: Path):
    """Generates the .h and .cpp files for a single font."""

    metadata = font_data['metadata']
    characters = font_data['characters']

    width = metadata['width']
    height = metadata['height']
    total_bitmap_size = NUM_CHARS * height

    header_filename = output_path / f"{font_name}.h"
    source_filename = output_path / f"{font_name}.cpp"

    # --- Process Data ---
    all_bitmap_data = []
    widths = []
    widths_comments = []

    for i in range(START_CHAR, END_CHAR + 1):
        char_code_str = str(i)

        pixels = characters.get(char_code_str, [0] * (width * height))

        shift, bits_wide = calculate_widths(pixels, width, height)
        widths.append(bits_wide)

        rows = convert_to_uint8_t_rows(pixels, width, height, shift)
        all_bitmap_data.extend(rows)

        widths_comments.append(f" // '{chr(i)}' ({i})")

    # --- Generate .h Header File ---

    header_content = f"""
// Generated C++ Header for Bitmap Font: {font_name}
#include <stdint.h>

#pragma once

#include "font.h"

extern font_t const {font_name}_font;
"""
    try:
        with open(header_filename, 'w') as f:
            f.write(header_content)
    except IOError as e:
        print(f"Error writing to file {header_filename}: {e}")
        return False

    # --- Generate .cpp Source File ---

    # Bitmap array definition (Omitted for brevity)
    bitmap_lines = []
    for i in range(NUM_CHARS):
        start_index = i * height
        end_index = (i + 1) * height
        char_data = all_bitmap_data[start_index:end_index]
        char_hex_data = ', '.join(f"0x{r:02X}" for r in char_data)
        char_code = START_CHAR + i
        char_line = f"    {char_hex_data}, // '{chr(char_code)}' ({char_code})"
        bitmap_lines.append(char_line)

    bitmap_array = f"""
// Total size: {NUM_CHARS} chars * {height} rows = {total_bitmap_size} bytes.
// Indexing: (char_code - {START_CHAR}) * {height} + row
static const uint8_t {font_name}_bitmap[{total_bitmap_size}] = {{
{chr(10).join(bitmap_lines)}
}};
"""

    # widths array definition
    width_lines = []
    for i in range(NUM_CHARS):
        hex_data = f"0x{widths[i]:02X}"
        comment = widths_comments[i]
        width_lines.append(f"    {hex_data},{comment}")

    widths_array = f"""
// Array storing character widths
// Indexing: char_code - {START_CHAR}
static const uint8_t {font_name}_widths[{NUM_CHARS}] = {{
{chr(10).join(width_lines)}
}};
"""

    # Final struct instance (Omitted for brevity)
    font_struct = f"""
// The main font structure instance, referencing the static arrays above.
const struct font_t {font_name}_font = {{
    .font_width = {width},
    .font_height = {height},
    .bitmap = {font_name}_bitmap,
    .widths = {font_name}_widths
}};
"""

    source_content = f"""
// Generated C++ Source for Bitmap Font: {font_name}
#include "{header_filename.name}"

{bitmap_array}
{widths_array}
{font_struct}
"""
    try:
        with open(source_filename, 'w') as f:
            f.write(source_content)

        print(f"{font_name}")
        return True
    except IOError as e:
        print(f"Error writing to file {source_filename}: {e}")
        return False


# --------------------------------------------------------------------------
## 🚀 Main Execution Loop
# --------------------------------------------------------------------------

def run_conversion():
    """Handles command-line arguments and processes multiple fonts."""

    parser = argparse.ArgumentParser(description="Converts JSON font files to C++ header and source files.")
    parser.add_argument("--source", help="Folder containing the JSON files", type=Path, default='fonts')
    parser.add_argument("--output", help="Folder for output files", type=Path, default='../firmware/main/fonts')
    try:
        args = parser.parse_args()
    except argparse.ArgumentError as e:
        print(f"Error: {e}")
        sys.exit(1)

    source_folder = args.source
    output_folder = args.output

    font_names = source_folder.glob("*.json")
    total = 0
    failed = 0
    succeeded = 0

    for font_name in font_names:

        total += 1

        json_input_file = Path(font_name)

        try:
            with open(json_input_file, 'r') as f:
                font_data = json.load(f)

            if 'metadata' not in font_data or 'characters' not in font_data:
                raise ValueError("JSON file must contain 'metadata' and 'characters' keys.")

            if generate_c_files(font_data, Path(font_name).stem, output_folder):
                succeeded += 1

        except FileNotFoundError:
            print(f"Error: Input file '{json_input_file}' not found. Skipping.")
            failed += 1
        except json.JSONDecodeError:
            print(f"Error: Failed to decode JSON from '{json_input_file}'. Check file format. Skipping.")
            failed += 1
        except Exception as e:
            print(f"An unexpected error occurred for {font_name}: {e}. Skipping.")
            failed += 1

    print(f"Total processed: {total}, {succeeded} succeeded, {failed} failed.")


if __name__ == "__main__":
    run_conversion()