# -*- coding: utf-8 -*-
"""Keep the drawing's linework when shrinking to the panel's 72x120.

The problem this solves: the panel has eight colours and no grey, and this drawing
is pale - light blue hair, white collar, skin. Ordinary LANCZOS reduction turns a
one-pixel outline into a light grey pixel, and the quantiser then has to choose
between white, cyan and magenta, so the outline disappears and the figure reads as
a white blob.

So the linework is not left to the quantiser: it is detected on the reduced image
and painted pure black onto the colour layer, which may then be as flat as it likes.

A plain brightness threshold does not work here - it swallows the waistcoat, which
is a dark saturated blue at more or less the same brightness as the outlines. The
outlines are not just dark, they are dark *and* colourless, so the test is
brightness below `T` **and** saturation below `S`. On the reduced image:

    outline        ~40 brightness, ~20 saturation   -> line
    waistcoat      ~110 brightness, ~90 saturation  -> stays
    shadowed skin  ~170 brightness, ~60 saturation  -> stays (T is below it)
    collar / hair  ~240 / ~190 brightness           -> stays

Usage:  python tools\rei3_line.py <src.png> <out_prefix>
"""
import sys

from PIL import Image

src, out = sys.argv[1], sys.argv[2]
im = Image.open(src).convert("RGB")

tone = im.resize((72, 120), Image.LANCZOS)
tone_px = tone.load()

# Three settings, because how much of the drawing survives is a matter of taste and
# can only be judged by looking at it.
for t, s in ((150, 60), (178, 60), (178, 95)):
    out_im = tone.copy()
    px = out_im.load()
    kept = 0
    for y in range(120):
        for x in range(72):
            r, g, b = tone_px[x, y]
            bright = (r * 299 + g * 587 + b * 114) // 1000
            sat = max(r, g, b) - min(r, g, b)
            if bright < t and sat < s:
                px[x, y] = (0, 0, 0)
                kept += 1
    path = "%s_b%d_s%d.png" % (out, t, s)
    out_im.save(path)
    print("wrote %s  (linework %d px, %.1f%%)" % (path, kept, 100.0 * kept / (72 * 120)))
