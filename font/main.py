import json
from pathlib import Path
import argparse

parser = argparse.ArgumentParser()

parser.add_argument("input", help="Font filename")
parser.add_argument("output", help="Output file")

args = parser.parse_args()

if not (args.input and args.output):
    raise ValueError("Must specify name and output")

with open(f'{args.output}', "w") as outfile:

    def output(s):
        outfile.write(f"{s}\n")
        print(s)

    with open(f'{args.input}', "r") as f:
        font = json.load(f)

    output("// struct glyph_t {")
    output("//     uint8_t width: 4;")
    output("//     uint8_t shift: 4;")
    output("//     uint8_t data[7];")
    output("// };")

    font_name = Path(args.input).stem

    output(f"glyph_t const {font_name}[] = {{")
    output('')

    for ch in range(32, 128):
        try:
            c = font[f"{ch}"]
        except KeyError:
            c = [0, 0, 0, 0, 0, 0, 0]
        left = 8
        right = 0
        data = []
        if len(c) < 7:
            continue
        image = []
        for row in c[:7]:
            row_str = ""
            for bit in range(0, 8):
                mask = 1 << bit
                dot = "  "
                if row & mask:
                    left = min(left, bit)
                    right = max(right, bit)
                    dot = "@@"
                row_str = f"{row_str}{dot}"
            image.append(row_str)
            data.append(row)
        width = right - left + 1
        if width < 0:
            width = 3
        shift = left
        output(f'{{ {width},{shift}, {{ {",".join([str(i) for i in data])} }} }}, // {ch} "{chr(ch)}"')
        output('')
        for line in image:
            output(f"// {line}")
        output('')
        ch += 1
    output("};")
