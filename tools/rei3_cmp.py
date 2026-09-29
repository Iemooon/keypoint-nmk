# -*- coding: utf-8 -*-
"""Side-by-side preview: several 72x120 candidates, scaled 3x for viewing."""
import sys

from PIL import Image

paths = sys.argv[1:-1]
dst = sys.argv[-1]
ims = [Image.open(p).convert("RGB") for p in paths]
w = sum(i.size[0] for i in ims) + 4 * (len(ims) - 1)
c = Image.new("RGB", (w, max(i.size[1] for i in ims)), (30, 30, 30))
x = 0
for i in ims:
    c.paste(i, (x, 0))
    x += i.size[0] + 4
c.resize((c.size[0] * 3, c.size[1] * 3), Image.NEAREST).save(dst)
print("wrote %s %s" % (dst, c.size))
