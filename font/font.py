import argparse

from PIL import Image

border = 1
char_width = 12
char_height = 18
char_count_width = 16

parser = argparse.ArgumentParser(description='convert font image to header')
parser.add_argument('input', type=str, help='font image to convert')
parser.add_argument('output', type=str, help='file to write font to')

args = parser.parse_args()

with Image.open(args.input).convert('RGBA') as im:
    pixels = im.load()

    def get_pixel(char, x, y):
        char_x = border + int(char % char_count_width) * \
            (char_width + border) + x
        char_y = border + int(char / char_count_width) * \
            (char_height + border) + y

        return pixels[char_x, char_y]

    with open(args.output, "w") as f:
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")

        f.write(
            "static const uint16_t font_data[2][256][%d] = {\n" % char_height)

        for pixel_val in [255, 0]:
            f.write("  {\n")
            for char in range(256):
                f.write("    {")
                for y in range(char_height):
                    val = int(0)
                    for x in range(char_width):
                        pixel = get_pixel(char, x, y)
                        if pixel[3] == 0:
                            continue

                        if pixel[0] == pixel_val:
                            val |= 1 << (2 + x)

                    f.write("0x%x" % val)
                    if (y != char_height-1):
                        f.write(", ")
                    else:
                        f.write("")

                f.write("},\n")
            f.write("  },\n")

        f.write("};\n")
