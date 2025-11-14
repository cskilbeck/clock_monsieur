import json

with open('font_data.h', 'w') as outfile:

    def output(s):
        outfile.write(f'{s}\n')
        print(s)

    with open("font.json", "r") as f:
        font = json.load(f)

    output("struct glyph_t {")
    output("    uint8_t width: 4;")
    output("    uint8_t shift: 4;")
    output("    uint8_t data[7];")
    output("};")

    output("glyph_t font_6x7[] = {")

    for ch in range(32,128):
        try:
            c = font[f'{ch}']
        except KeyError:
            c = [0,0,0,0,0,0,0]
        left = 8
        right = 0
        data = []
        output(f'// CHAR {ch} "{chr(ch)}"')
        if len(c) < 7:
            continue
        for row in c[:7]:
            row_str = ""
            for bit in range(0,8):
                mask = 1 << bit
                dot = '.'
                if row & mask:
                    left = min(left, bit)
                    right = max(right, bit)
                    dot = '#'
                row_str = f'{row_str}{dot}'
            output(f'// {row_str}')
            data.append(row)
        width = right - left + 1
        if width < 0:
            width = 3
        shift = left
        output(f'{{ {width},{shift}, {{ {",".join([str(i) for i in data])} }} }},')
        ch += 1
    output("};")
