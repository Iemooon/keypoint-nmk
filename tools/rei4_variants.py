# -*- coding: utf-8 -*-
r"""More ways to draw the pale portrait on an eight-colour 72x120 panel.

Why this exists: the drawing Lemon uploaded is pale - light blue hair, white
collar, pale skin - and the panel has no grey. Reduced eleven-fold, the face is a
dozen pixels wide and hair, collar and skin all land on the same "white", so the
picture reads as a blank blob with outlines. Enlarging the subject is therefore the
main lever, not colour handling:

  face   head and shoulders only   the face stops being a dozen pixels
  bust   head to waist
  full   the whole drawing, as the current build has it

and on top of that, saturation is pushed so the hair separates from the collar
instead of both being white.

The linework rule from the first pass is kept: a pixel that is both dark
(brightness < LINE_BRIGHT) and colourless (saturation < LINE_SAT) is painted pure
black before anything else touches the picture, so the outlines survive both the
quantiser and any dithering that follows.

Run white_to_black.py first - this reads its output, so the background is already
black and does not have to be re-detected.

Usage:  python tools\rei4_variants.py <nobg.png> <out_prefix>
"""
import sys

from PIL import Image, ImageEnhance

LINE_BRIGHT = 178
LINE_SAT = 60
OUT_W, OUT_H = 72, 120
RATIO = OUT_H / OUT_W  # the panel is tall; every crop keeps this shape


def frame(W, H, centre_x, top, width_frac):
    """A RATIO-shaped box: `centre_x`/`top` are fractions of the source."""
    w = int(W * width_frac)
    h = int(w * RATIO)
    if h > H:                      # never taller than the drawing itself
        h = H
        w = int(h / RATIO)
    x0 = int(W * centre_x - w / 2)
    y0 = int(H * top)
    x0 = max(0, min(W - w, x0))
    y0 = max(0, min(H - h, y0))
    return (x0, y0, x0 + w, y0 + h)


def punch_lines(small):
    """Force dark, colourless pixels to pure black - the outlines."""
    px = small.load()
    n = 0
    for y in range(small.size[1]):
        for x in range(small.size[0]):
            r, g, b = px[x, y]
            if (r * 299 + g * 587 + b * 114) // 1000 < LINE_BRIGHT \
                    and max(r, g, b) - min(r, g, b) < LINE_SAT:
                px[x, y] = (0, 0, 0)
                n += 1
    return n


def main():
    src, prefix = sys.argv[1], sys.argv[2]
    src_im = Image.open(src).convert("RGB")
    W, H = src_im.size
    print("source %dx%d" % (W, H))

    plans = [
        ("face_plain", frame(W, H, 0.53, 0.02, 0.46), 1.0),
        ("face_sat",   frame(W, H, 0.53, 0.02, 0.46), 2.2),
        ("bust_sat",   frame(W, H, 0.53, 0.06, 0.72), 2.2),
        ("full_sat",   None, 2.2),
    ]
    for name, box, sat in plans:
        im = src_im.crop(box) if box else src_im
        small = im.resize((OUT_W, OUT_H), Image.LANCZOS)
        if sat != 1.0:
            small = ImageEnhance.Color(small).enhance(sat)
        n = punch_lines(small)
        path = "%s_%s.png" % (prefix, name)
        small.save(path)
        print("wrote %s  box=%s sat=%.1f  linework %d px" % (path, box, sat, n))


if __name__ == "__main__":
    main()
