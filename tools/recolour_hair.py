# -*- coding: utf-8 -*-
"""Rei's hair is pale blue, and pale blue is not one of the panel's eight colours --
it lands on white, the same white as her face, her blouse and the page. Ask the
drawing to tell us which pixels are hair instead.

Hair pixels here are the ones that lean blue: blue leads red by a clear margin and
is not behind green. A white blouse has b-r ~ 0 and is excluded by that margin; the
cool shadow under a collar leans blue only slightly, which is what the margin is
tuned against.

Pass one / pass two:
  * qualifying pixels become blue outright (there is no second blue to shade with),
  * except the brightest ones, which stay white - that keeps a highlight running
    through the hair instead of flattening it to one solid slab.

Usage (run from the keypoint-nmk root):
  python tools\\recolour_hair.py in.png out.png [--lead 18] [--tol 6] [--hi 225]
"""
import argparse
from PIL import Image

BLUE = (0, 0, 255)
WHITE = (255, 255, 255)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--lead", type=int, default=18,
                    help="how far blue must lead red to count as hair")
    ap.add_argument("--tol", type=int, default=6,
                    help="how far blue may trail green and still count")
    ap.add_argument("--floor", type=int, default=110,
                    help="darker than this and we leave it alone (outlines, shadow)")
    ap.add_argument("--hi", type=int, default=225,
                    help="brighter than this keeps white, as the highlight")
    ap.add_argument("--report", action="store_true")
    args = ap.parse_args()

    im = Image.open(args.src).convert("RGB")
    w, h = im.size
    px = im.load()
    out = im.copy()
    dst = out.load()

    n_hair = n_hi = 0
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if b - r >= args.lead and b >= g - args.tol and min(r, g, b) >= args.floor:
                n_hair += 1
                if min(r, g, b) >= args.hi:
                    dst[x, y] = WHITE
                    n_hi += 1
                else:
                    dst[x, y] = BLUE
    out.save(args.dst)
    print("wrote %s  (hair px: %d, of which highlight kept white: %d)"
          % (args.dst, n_hair, n_hi))

    if args.report:
        # Show what got caught, so a mis-tuned margin is visible rather than guessed.
        probe = im.copy()
        pd = probe.load()
        for y in range(h):
            for x in range(w):
                r, g, b = px[x, y]
                if b - r >= args.lead and b >= g - args.tol and min(r, g, b) >= args.floor:
                    pd[x, y] = (0, 255, 0) if min(r, g, b) < args.hi else (255, 0, 0)
        probe.resize((w * 2, h * 2), Image.NEAREST).save(args.dst.replace(".png", "_mask.png"))
        print("wrote %s (green = turned blue, red = kept as highlight)"
              % args.dst.replace(".png", "_mask.png"))


if __name__ == "__main__":
    main()