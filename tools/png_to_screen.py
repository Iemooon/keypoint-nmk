# -*- coding: utf-8 -*-
"""Turn a PNG into a bitmap the keypoint-nmk screen can draw in colour.

WHAT THE SCREEN WANTS
---------------------
The panel takes four bits per pixel: `0bRGB0`, so one bit for red, one for green,
one for blue and the low bit unused. That is eight colours and nothing else - it is
a reflective memory LCD, there is no backlight and no grey level, so a pixel is
either fully on or fully off in each of the three channels. A photograph will come
out as flat colour fields; line art, icons and cartoon drawings are what this can
actually show.

Two pixels share a byte, and the EVEN column goes in the HIGH nibble. Rows are
stored one after another, `stride` bytes each.

WHAT THIS SCRIPT DOES
---------------------
Reads the PNG, resizes it to the screen's drawing size, reduces every pixel to the
nearest of the eight colours, packs the nibbles and writes a Rust source file
holding the array plus its dimensions.

Usage:

    python tools\\png_to_screen.py art.png --name CAPY_COLOUR --out keyboard\\src\\screen\\art.rs

Defaults are the size the capybara artwork uses (72 x 120, drawn onto a 72-row
screen by box-reduction, exactly like the 1-bit artwork is now).
"""
import argparse
import sys

# The panel's eight colours. A pixel is one bit per channel, so these are the only
# values that exist; anything else is an approximation of one of them.
COLOURS = [
    (0x0, 0, 0, 0),        # black
    (0x2, 0, 0, 255),      # blue
    (0x4, 0, 255, 0),      # green
    (0x6, 0, 255, 255),    # cyan
    (0x8, 255, 0, 0),      # red
    (0xA, 255, 0, 255),    # magenta
    (0xC, 255, 255, 0),    # yellow
    (0xE, 255, 255, 255),  # white
]


def nearest_nibble(r, g, b):
    """Nearest of the eight colours, by squared distance in RGB."""
    best = None
    best_d = None
    for nib, cr, cg, cb in COLOURS:
        d = (r - cr) ** 2 + (g - cg) ** 2 + (b - cb) ** 2
        if best_d is None or d < best_d:
            best_d = d
            best = nib
    return best


# The 4x4 ordered-dither threshold matrix. Values 0..15; the pixel is nudged by
# (value/16 - 0.5) * strength before it is snapped to a colour, so the same source
# value lands on different colours depending on where it sits on the glass. That is
# what makes a regular pattern of dots where error diffusion makes noise.
BAYER4 = [
    [0, 8, 2, 10],
    [12, 4, 14, 6],
    [3, 11, 1, 9],
    [15, 7, 13, 5],
]


def colour_of(nib):
    for n, cr, cg, cb in COLOURS:
        if n == nib:
            return cr, cg, cb
    return 0, 0, 0


def pack(img, width, height, threshold, dither="none", strength=128):
    """Resize and pack to 4bpp, even column in the high nibble.

    With `dither="fs"`, the eight colours are not picked one pixel at a time. Each
    pixel is still snapped to the nearest one, but the difference between what was
    asked for and what was drawn is handed to the neighbours, so a field of skin
    colour comes out as a spray of white and red whose average is the skin colour
    the eye reports. `dither="bayer"` instead nudges each pixel by a fixed repeating
    offset, which makes a regular dot pattern rather than noise.

    Two rules keep either one usable on this particular artwork:

    * the dead zone still applies, so a near-black soft edge becomes black rather
      than a red or blue fringe;
    * a pixel whose source is black is drawn black and takes no part - it neither
      emits error nor absorbs any that lands on it. This is what keeps the
      knocked-out background (which is black here) from filling up with coloured
      speckle, the usual failure of dithering on line art.
    """
    from PIL import Image

    img = img.convert("RGB").resize((width, height), Image.LANCZOS)
    src = [[img.getpixel((x, y)) for x in range(width)] for y in range(height)]
    stride = (width + 1) // 2
    data = bytearray(stride * height)

    # A pixel is background when its own source is black. Not a guess: the
    # drawings arrive with the background knocked out to black on purpose.
    is_bg = [[all(c < threshold for c in src[y][x]) for x in range(width)]
             for y in range(height)]

    # Floats: snapping to eight colours throws away a lot of value per pixel, and
    # in integers most of the carried error would round away to nothing.
    buf = [[[float(c) for c in src[y][x]] for x in range(width)]
           for y in range(height)]

    for y in range(height):
        for x in range(width):
            if is_bg[y][x]:
                nib = 0x0
            elif dither == "fs":
                r, g, b = buf[y][x]
                if r < threshold and g < threshold and b < threshold:
                    nib = 0x0
                else:
                    nib = nearest_nibble(r, g, b)
                cr, cg, cb = colour_of(nib)
                er, eg, eb = r - cr, g - cg, b - cb
                for dx, dy, w in ((1, 0, 7 / 16.0), (-1, 1, 3 / 16.0),
                                  (0, 1, 5 / 16.0), (1, 1, 1 / 16.0)):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and ny < height and not is_bg[ny][nx]:
                        buf[ny][nx][0] += er * w
                        buf[ny][nx][1] += eg * w
                        buf[ny][nx][2] += eb * w
            elif dither == "bayer":
                off = (BAYER4[y % 4][x % 4] / 16.0 - 0.5) * strength
                r, g, b = (min(255.0, max(0.0, c + off)) for c in src[y][x])
                if r < threshold and g < threshold and b < threshold:
                    nib = 0x0
                else:
                    nib = nearest_nibble(r, g, b)
            else:
                r, g, b = src[y][x]
                if r < threshold and g < threshold and b < threshold:
                    nib = 0x0
                else:
                    nib = nearest_nibble(r, g, b)

            i = y * stride + x // 2
            if x % 2 == 0:
                data[i] = (data[i] & 0x0F) | (nib << 4)
            else:
                data[i] = (data[i] & 0xF0) | nib
    return bytes(data), stride


def rust_source(name, data, width, height, stride, source_note):
    lines = []
    lines.append("//! Generated by tools/png_to_screen.py - do not edit by hand.")
    lines.append("//!")
    lines.append("//! %s" % source_note)
    lines.append("//!")
    lines.append("//! %d x %d, four bits per pixel in the panel's `0bRGB0` order," % (width, height))
    lines.append("//! %d bytes per row, two pixels per byte, even column in the high nibble." % stride)
    lines.append("")
    lines.append("/// %d x %d drawing." % (width, height))
    lines.append("pub const %s_W: u32 = %d;" % (name, width))
    lines.append("pub const %s_H: u32 = %d;" % (name, height))
    lines.append("/// Bytes per row; two pixels share each byte.")
    lines.append("pub const %s_STRIDE: usize = %d;" % (name, stride))
    lines.append("")
    lines.append("#[rustfmt::skip]")
    lines.append("pub const %s: [u8; %d] = [" % (name, len(data)))
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        lines.append("    " + " ".join("0x%02x," % b for b in chunk))
    lines.append("];")
    lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("png", help="source image")
    ap.add_argument("--name", default="ART", help="Rust constant name (default ART)")
    ap.add_argument("--out", default="-", help="output .rs path, or - for stdout")
    ap.add_argument("--width", type=int, default=72, help="drawing width (default 72)")
    ap.add_argument("--height", type=int, default=120, help="drawing height (default 120)")
    ap.add_argument("--black-threshold", type=int, default=40,
                    help="channel values below this all become black (default 40)")
    ap.add_argument("--dither", choices=("none", "fs", "bayer"), default="none",
                    help="none: snap each pixel on its own (default). "
                         "fs: Floyd-Steinberg error diffusion. "
                         "bayer: 4x4 ordered dot pattern.")
    ap.add_argument("--dither-strength", type=float, default=128.0,
                    help="bayer only: how far a pixel may be nudged (default 128)")
    ap.add_argument("--preview", default=None,
                    help="also write a PNG of what the screen will show")
    args = ap.parse_args()

    try:
        from PIL import Image
    except ImportError:
        sys.exit("Pillow is not installed: python -m pip install Pillow")

    img = Image.open(args.png)
    data, stride = pack(img, args.width, args.height, args.black_threshold,
                        dither=args.dither, strength=args.dither_strength)

    how = {
        "none": "reduced to the panel's eight colours with no dithering",
        "fs": "dithered onto the panel's eight colours with Floyd-Steinberg error diffusion",
        "bayer": "dithered onto the panel's eight colours with a 4x4 ordered pattern "
                 "(strength %.0f)" % args.dither_strength,
    }[args.dither]
    note = "Source: %s, %s." % (args.png, how)
    text = rust_source(args.name, data, args.width, args.height, stride, note)

    if args.out == "-":
        # Windows consoles are not all UTF-8; the generated text is pure ASCII
        # except for the note, which is ASCII here too, so this is safe.
        sys.stdout.write(text)
    else:
        with open(args.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print("wrote %s (%d bytes of pixel data, %dx%d, stride %d)"
              % (args.out, len(data), args.width, args.height, stride))
        # How much of the image actually survived quantisation: if almost
        # everything came out black or white, the source is probably too
        # photographic for eight colours.
        from collections import Counter
        counts = Counter()
        for b in data:
            counts[(b >> 4) & 0x0F] += 1
            counts[b & 0x0F] += 1
        total = args.width * args.height
        print("colour usage:")
        for nib, cr, cg, cb in COLOURS:
            n = counts.get(nib, 0)
            print("  0x%x rgb(%3d,%3d,%3d)  %6d px  %5.1f%%"
                  % (nib, cr, cg, cb, n, 100.0 * n / total))

    if args.preview:
        prev = Image.new("RGB", (args.width, args.height))
        px = prev.load()
        for y in range(args.height):
            for x in range(args.width):
                i = y * stride + x // 2
                nib = (data[i] >> 4) if x % 2 == 0 else (data[i] & 0x0F)
                for cn, cr, cg, cb in COLOURS:
                    if cn == nib:
                        px[x, y] = (cr, cg, cb)
                        break
        prev.save(args.preview)
        print("wrote %s (what the screen will show)" % args.preview)


if __name__ == "__main__":
    main()