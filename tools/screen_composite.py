# -*- coding: utf-8 -*-
"""Composite what the panel will actually show: the drawing plus the two text zones.

Not a faithful renderer - the fonts here are stand-ins, drawn as blocks of roughly
the right size in roughly the right place. What it IS faithful about is the two
things that decide whether a drawing is usable on this screen:

  * where the text sits over the picture (top band from row 0, state line from
    row 116), and
  * whether the picture's own colours leave the text legible, because both are
    drawn in the same white.

Usage: python tools\\screen_composite.py art_72x120.png out.png "POINTER MODE" 87
"""
import sys
from PIL import Image, ImageDraw

SCREEN_W, SCREEN_H = 72, 144        # logical panel size, portrait
CAPY_X, CAPY_Y = 1, 9               # where renderers.rs puts the drawing
BATT_BAND = (0, 28)                 # battery bar + digits, drawn over the picture
STATE_BAND_Y = 116                  # the state line's band starts here


def main():
    art_path, out_path = sys.argv[1], sys.argv[2]
    state = sys.argv[3] if len(sys.argv) > 3 else "POINTER MODE"
    batt = int(sys.argv[4]) if len(sys.argv) > 4 else 87

    canvas = Image.new("RGB", (SCREEN_W, SCREEN_H), (0, 0, 0))
    art = Image.open(art_path).convert("RGB")
    canvas.paste(art, (CAPY_X, CAPY_Y))
    d = ImageDraw.Draw(canvas)

    # Battery: a filled bar and a percentage, white, top of screen.
    bar = (4, 18, 68, 26)
    d.rectangle(bar, outline=(255, 255, 255))
    fill_w = int((bar[2] - bar[0] - 4) * batt / 100.0)
    if fill_w > 0:
        d.rectangle((bar[0] + 2, bar[1] + 2, bar[0] + 2 + fill_w, bar[3] - 2),
                    fill=(255, 255, 255))
    # A stand-in for the digits: two short strokes at 6x13-ish size.
    d.rectangle((26, 2, 30, 14), fill=(255, 255, 255))
    d.rectangle((34, 2, 38, 14), fill=(255, 255, 255))
    d.rectangle((42, 2, 46, 14), fill=(255, 255, 255))

    # State line: centred white text on its band.
    tw = len(state) * 6
    x0 = max(1, (SCREEN_W - tw) // 2)
    d.rectangle((x0, STATE_BAND_Y, x0 + tw, STATE_BAND_Y + 12),
                fill=(255, 255, 255))

    canvas.save(out_path)
    print("wrote %s (%dx%d)" % (out_path, SCREEN_W, SCREEN_H))


if __name__ == "__main__":
    main()