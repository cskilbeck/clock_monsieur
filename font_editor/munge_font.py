import json
import sys
from typing import Dict, List, Tuple

# --- Constants ---
START_CHAR = 32
END_CHAR = 127
NUM_CHARS = END_CHAR - START_CHAR + 1


# --- Utility Functions ---

def calculate_bounds(pixels: List[int], width: int, height: int) -> Tuple[int, int, int]:
    """
    Calculates and packs the low and high column indices (0-indexed).

    low_col_index: The index of the first (leftmost) column with a set pixel.
    high_col_index: The index of the last (rightmost) column with a set pixel.

    The final byte is packed as: [high_col_index (high nibble)][low_col_index (low nibble)]
    """
    low_col_index = width  # Represents min_x (leftmost column index)
    high_col_index = -1  # Represents max_x (rightmost column index)

    for row_index in range(height):
        for col_index in range(width):
            if pixels[row_index * width + col_index] == 1:
                low_col_index = min(low_col_index, col_index)
                high_col_index = max(high_col_index, col_index)

    # Handle characters with no set pixels (like space ' ')
    if low_col_index == width:
        low_col_index, high_col_index = 0, 0

    # 🚨 CHANGE APPLIED HERE 🚨
    # Pack bounds into a single uint8_t:
    # high nibble (4 bits): high_col_index (rightmost boundary)
    # low nibble (4 bits): low_col_index (leftmost boundary)
    bounds_byte = (high_col_index << 4) | low_col_index

    return bounds_byte, low_col_index, high_col_index


def convert_to_uint8_t_rows(pixels: List[int], width: int, height: int) -> List[int]:
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

        uint8_t_rows.append(row_value)

    return uint8_t_rows


# --- C/C++ File Generation ---

def generate_c_files(font_data: Dict, font_name: str):
    """Generates the .h and .cpp files for a single font."""

    metadata = font_data['metadata']
    characters = font_data['characters']

    width = metadata['width']
    height = metadata['height']
    total_bitmap_size = NUM_CHARS * height

    header_filename = f"{font_name}.h"
    source_filename = f"{font_name}.cpp"

    # --- Process Data ---
    all_bitmap_data = []
    all_bounds_data = []
    bounds_comments = []

    for i in range(START_CHAR, END_CHAR + 1):
        char_code_str = str(i)

        pixels = characters.get(char_code_str, [0] * (width * height))

        rows = convert_to_uint8_t_rows(pixels, width, height)
        all_bitmap_data.extend(rows)

        bounds_byte, low_col_index, high_col_index = calculate_bounds(pixels, width, height)
        all_bounds_data.append(bounds_byte)

        bounds_comments.append(f" /* '{chr(i)}' (Low Col: {low_col_index}, High Col: {high_col_index}) */")

    # --- Generate .h Header File ---

    header_content = f"""
// Generated C++ Header for Bitmap Font: {font_name}
// Character Range: ASCII {START_CHAR} to {END_CHAR}
#include <stdint.h>

#pragma once

// Assumed external structure definition (omitted here for modularity):
/*
struct font_t {{
    uint8_t width;
    uint8_t height;
    const uint8_t *bitmap;
    const uint8_t *bounds;
}};
*/

// Declaration of the font structure instance defined in {source_filename}
extern const struct font_t {font_name}_font;
"""
    try:
        with open(header_filename, 'w') as f:
            f.write(header_content)
    except IOError as e:
        print(f"❌ Error writing to file {header_filename}: {e}")
        return

    # --- Generate .cpp Source File ---

    # Bitmap array definition (Omitted for brevity)
    bitmap_lines = []
    for i in range(NUM_CHARS):
        start_index = i * height
        end_index = (i + 1) * height
        char_data = all_bitmap_data[start_index:end_index]
        char_hex_data = ', '.join(f"0x{r:02X}" for r in char_data)
        char_code = START_CHAR + i
        char_line = f"    {char_hex_data}, /* '{chr(char_code)}' (ASCII {char_code}) */"
        bitmap_lines.append(char_line)

    bitmap_array = f"""
// Array storing all character bitmaps sequentially.
// Data is **Right-Aligned** (occupies low bits 0 to {width - 1}).
// Total size: {NUM_CHARS} chars * {height} rows = {total_bitmap_size} bytes.
// Indexing: (char_code - {START_CHAR}) * {height} + row
static const uint8_t {font_name}_bitmap[{total_bitmap_size}] = {{
{chr(10).join(bitmap_lines)}
}};
"""

    # Bounds array definition
    bounds_lines = []
    for i in range(NUM_CHARS):
        hex_data = f"0x{all_bounds_data[i]:02X}"
        comment = bounds_comments[i]
        bounds_lines.append(f"    {hex_data},{comment}")

    bounds_array = f"""
// Array storing character bounds (column indices).
// Indexing: char_code - {START_CHAR}
// uint8_t format: [High Col Index (4 bits)][Low Col Index (4 bits)]
static const uint8_t {font_name}_bounds[{NUM_CHARS}] = {{
{chr(10).join(bounds_lines)}
}};
"""

    # Final struct instance (Omitted for brevity)
    font_struct = f"""
// The main font structure instance, referencing the static arrays above.
const struct font_t {font_name}_font = {{
    .width = {width},
    .height = {height},
    .bitmap = {font_name}_bitmap,
    .bounds = {font_name}_bounds
}};
"""

    source_content = f"""
// Generated C++ Source for Bitmap Font: {font_name}
#include "{header_filename}"

// Definition of the struct font_t for local linkage of the data.
struct font_t {{
    uint8_t width;
    uint8_t height;
    const uint8_t *bitmap;
    const uint8_t *bounds;
}};

{bitmap_array}
{bounds_array}
{font_struct}
"""
    try:
        with open(source_filename, 'w') as f:
            f.write(source_content)

        print(f"✅ Successfully processed {font_name}. Generated {header_filename} and {source_filename}.")
        return True
    except IOError as e:
        print(f"❌ Error writing to file {source_filename}: {e}")
        return False


# --------------------------------------------------------------------------
## 🚀 Main Execution Loop
# --------------------------------------------------------------------------

def run_conversion():
    """Handles command-line arguments and processes multiple fonts."""

    if len(sys.argv) < 2:
        print("Usage: python font_converter.py <font_name_1> [font_name_2] ...")
        sys.exit(1)

    font_names = sys.argv[1:]
    success_count = 0

    print(f"Starting font conversion for {len(font_names)} font(s)...")

    for font_name in font_names:
        json_input_file = f"{font_name}.json"

        try:
            print(f"\n--- Processing: {font_name} ---")

            with open(json_input_file, 'r') as f:
                font_data = json.load(f)

            if 'metadata' not in font_data or 'characters' not in font_data:
                raise ValueError("JSON file must contain 'metadata' and 'characters' keys.")

            if generate_c_files(font_data, font_name):
                success_count += 1

        except FileNotFoundError:
            print(f"❌ Error: Input file '{json_input_file}' not found. Skipping.")
        except json.JSONDecodeError:
            print(f"❌ Error: Failed to decode JSON from '{json_input_file}'. Check file format. Skipping.")
        except Exception as e:
            print(f"An unexpected error occurred for {font_name}: {e}. Skipping.")

    print(f"\n--- Conversion Finished ---")
    print(f"Total processed: {len(font_names)}. Successfully generated: {success_count}.")


if __name__ == "__main__":
    run_conversion()