# -*- coding: utf-8 -*-
"""Pre-pass for screen art: knock the *background* down to black, and only the background.

Why: the panel is reflective and the firmware draws its text in white, so a drawing
that keeps its white page is a white rectangle on the screen - it swallows the state
line and the battery digits. Black is the cheap, quiet colour here.

The naive version (white -> black, everywhere) over-reaches: a character's face, a
white shirt, the whites of an eye are all white too, and they are *inside* the
subject. Turn those black and the picture reads as a negative. So the background is
found by connectivity instead - flood from the borders through near-white pixels, and
only what the flood reaches is background. Enclosed white survives.

  --threshold T   a pixel counts as "page white" when its *minimum* channel is >= T
                  (default 235). min() rather than brightness, so pale saturated
                  colours - light blue hair, cream - are not eaten with the paper.
  --grow G        also treat pixels within G steps of the threshold as floodable
                  (default 0); raise it if a halo of unflooded white sticks to the
                  subject's outline.

Usage:
  python tools\\white_to_black.py in.png out.png
  python tools\\white_to_black.py in.png out.png --mode all      (the naive version)
"""
import argparse
from collections import deque

from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--threshold", type=int, default=235)
    ap.add_argument("--grow", type=int, default=0)
    ap.add_argument("--mode", choices=("border", "all"), default="border",
                    help="border = only white connected to the image edge (default); "
                         "all = every near-white pixel")
    args = ap.parse_args()

    im = Image.open(args.src).convert("RGB")
    w, h = im.size
    px = im.load()
    limit = args.threshold - args.grow

    def is_page(x, y):
        r, g, b = px[x, y]
        return min(r, g, b) >= limit

    background = bytearray(w * h)  # 1 = knock to black

    if args.mode == "all":
        for y in range(h):
            for x in range(w):
                if is_page(x, y):
                    background[y * w + x] = 1
    else:
        # Flood from every border pixel that is page-white. 4-connected, which is
        # enough: an 8-connected flood leaks through single-pixel diagonal gaps in
        # an outline and can reach the inside of a face.
        q = deque()
        for x in range(w):
            for y in (0, h - 1):
                if is_page(x, y) and not background[y * w + x]:
                    background[y * w + x] = 1
                    q.append((x, y))
        for y in range(h):
            for x in (0, w - 1):
                if is_page(x, y) and not background[y * w + x]:
                    background[y * w + x] = 1
                    q.append((x, y))
        while q:
            x, y = q.popleft()
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < w and 0 <= ny < h and not background[ny * w + nx] \
                        and is_page(nx, ny):
                    background[ny * w + nx] = 1
                    q.append((nx, ny))

    out = Image.new("RGB", (w, h), (0, 0, 0))
    dst = out.load()
    dropped = 0
    for y in range(h):
        for x in range(w):
            if background[y * w + x]:
                dropped += 1
            else:
                dst[x, y] = px[x, y]

    out.save(args.dst)
    print("wrote %s  (mode=%s, background removed: %d px, %.1f%%)"
          % (args.dst, args.mode, dropped, 100.0 * dropped / (w * h)))


if __name__ == "__main__":
    main()