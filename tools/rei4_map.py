# -*- coding: utf-8 -*-
r"""Decide each pixel's colour by rule instead of by nearest-colour quantisation.

Why: on a pale drawing the nearest-colour step is what kills the picture. Hair at
(190,200,225), collar at (245,245,245) and skin at (235,215,205) are all closest to
white, so the whole upper half of the drawing collapses onto one colour and the
figure loses its identity. Nearest-colour cannot be argued with - it is doing its
job - so the classification is done here, where we know what the parts mean:

    dark and colourless            -> black    the linework
    bright and colourless          -> white    collar, highlights, pale skin
    strongly coloured and warm     -> red      the ribbon, the eyes
    cool and bright                -> cyan     the hair (see --hair)
    cool and dark                  -> blue     the waistcoat
    anything else                  -> white

Everything is decided on the reduced 72x120 image, after the linework rule from the
first pass (dark and colourless) has been applied, so outlines are pure black and
safe from any later quantiser.

Usage:
  python tools\rei4_map.py <nobg.png> <out_prefix> [--hair cyan|blue]
"""
import argparse

from PIL import Image, ImageEnhance

OUT_W, OUT_H = 72, 120
RATIO = OUT_H / OUT_W

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)


def frame(W, H, centre_x, top, width_frac):
    w = int(W * width_frac)
    h = int(w * RATIO)
    if h > H:
        h = H
        w = int(h / RATIO)
    x0 = max(0, min(W - w, int(W * centre_x - w / 2)))
    y0 = max(0, min(H - h, int(H * top)))
    return (x0, y0, x0 + w, y0 + h)


def classify(r, g, b, hair, red_sat):
    bright = (r * 299 + g * 587 + b * 114) // 1000
    sat = max(r, g, b) - min(r, g, b)
    if bright < 178 and sat < 60:
        return BLACK
    if sat >= red_sat and r > b:       # the ribbon and the eyes, before the hair rule
        return RED
    if b >= r and sat >= 45:
        return hair if bright >= 170 else BLUE
    if bright >= 215 and sat < 45:
        return WHITE
    return WHITE if bright >= 190 else BLACK


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("prefix")
    ap.add_argument("--hair", choices=("cyan", "blue"), default="cyan")
    ap.add_argument("--sat", type=float, default=2.2,
                    help="pre-boost, so pale hair reaches the saturation test")
    ap.add_argument("--red-sat", type=int, default=130,
                    help="how saturated a warm pixel must be to count as the ribbon; "
                         "lower catches skin shading and turns the face blotchy")
    ap.add_argument("--tag", default="",
                    help="suffix for the output names, so several settings can coexist")
    args = ap.parse_args()

    hair = CYAN if args.hair == "cyan" else BLUE
    src = Image.open(args.src).convert("RGB")
    W, H = src.size
    print("source %dx%d, hair -> %s" % (W, H, args.hair))

    boxes = {
        "face": frame(W, H, 0.50, 0.02, 0.46),
        "bust": frame(W, H, 0.50, 0.05, 0.72),
    }
    for name, box in boxes.items():
        small = src.crop(box).resize((OUT_W, OUT_H), Image.LANCZOS)
        small = ImageEnhance.Color(small).enhance(args.sat)
        px = small.load()
        counts = {BLACK: 0, WHITE: 0, RED: 0, CYAN: 0, BLUE: 0}
        for y in range(OUT_H):
            for x in range(OUT_W):
                c = classify(*px[x, y], hair=hair, red_sat=args.red_sat)
                px[x, y] = c
                counts[c] = counts.get(c, 0) + 1
        path = "%s_%s_%s%s.png" % (args.prefix, name, args.hair, args.tag)
        small.save(path)
        print("wrote %s  box=%s  black %d white %d red %d cyan %d blue %d"
              % (path, box, counts[BLACK], counts[WHITE], counts[RED],
                 counts[CYAN], counts[BLUE]))


if __name__ == "__main__":
    main()
