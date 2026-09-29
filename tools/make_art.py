# -*- coding: utf-8 -*-
"""Assemble the two portrait drawings into one Rust module for the firmware.

Takes the `.rs` files that `png_to_screen.py` writes (one per drawing) and emits
`keyboard/src/screen/art.rs`, which is what the halves actually include: a small type plus
one static per half.

Left half shows the first drawing, right half the second. The firmware picks by
constant, so swapping which drawing sits on which half means editing the two `bin`
files, not this script.

Usage (run from the keypoint-nmk root):
  python tools\\make_art.py --left tools\\art-src\\rei7_bust.rs --right tools\\art-src\\asuka_b128.rs
"""
import argparse
import re
from pathlib import Path

MODULE_HEADER = """//! The drawings the two halves show on their screens.
//!
//! Portrait, {w} x {h}, four bits per pixel in the panel's `0bRGB0` order
//! (0 = black, which the glass draws as nothing at all), {stride} bytes per row,
//! two pixels per byte, even column in the high nibble.
//!
//! DO NOT EDIT BY HAND - regenerate with `tools/make_art.py`, which takes the
//! `.rs` files produced by `tools/png_to_screen.py`.
//!
//! The drawing starts at y{y0} on the panel and is {h} rows tall, so the battery
//! bar sits over its top and the state line covers its feet. That is the same
//! compromise the capybara animation made; there is no placement that fits a
//! {h}-row picture and both captions on a 144-row glass.
//!
{note}
/// Width of every drawing in this module, in pixels.
pub const ART_W: i32 = {w};
/// Height of every drawing in this module, in pixels.
pub const ART_H: i32 = {h};
/// Bytes per row. Two pixels share each byte.
pub const ART_STRIDE: usize = {stride};

/// A drawing, as the renderer wants it: pixels and the stride they are laid out with.
pub struct Art {{
    pub data: &'static [u8],
}}

"""
PER_ART = """/// {doc}
static {name}_PIXELS: [u8; {n}] = [
{body}
];

/// {doc}
pub static {name}: Art = Art {{ data: &{name}_PIXELS }};

"""


def load_pixels(path):
    """Pull the byte array out of a file png_to_screen.py wrote."""
    text = Path(path).read_text(encoding="utf-8")
    m = re.search(r"\[u8;\s*(\d+)\s*\]\s*=\s*\[(.*?)\];", text, re.S)
    if not m:
        raise SystemExit(f"{path}: no [u8; N] = [ ... ]; array found")
    n = int(m.group(1))
    values = re.findall(r"0x([0-9a-fA-F]{2})", m.group(2))
    if len(values) != n:
        raise SystemExit(f"{path}: declared {n} bytes but found {len(values)}")
    return n, [int(v, 16) for v in values]


def format_body(values, per_line=16):
    lines = []
    for i in range(0, len(values), per_line):
        chunk = ", ".join("0x%02x" % v for v in values[i:i + per_line])
        lines.append("    " + chunk + ",")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", required=True, help="drawing for the LEFT half")
    ap.add_argument("--right", required=True, help="drawing for the RIGHT half")
    ap.add_argument("--out", default="keyboard/src/screen/art.rs")
    ap.add_argument("--y0", type=int, default=9, help="row the drawing starts at")
    ap.add_argument("--note", default="", help="extra line for the module header, "
                                              "e.g. which variant of a drawing this is")
    args = ap.parse_args()

    n_l, px_l = load_pixels(args.left)
    n_r, px_r = load_pixels(args.right)
    if n_l != n_r:
        raise SystemExit(f"size mismatch: {n_l} vs {n_r} bytes")
    stride = 36
    h = n_l // stride
    w = stride * 2

    note = ("//! " + args.note) if args.note else ""
    out = [MODULE_HEADER.format(w=w, h=h, stride=stride, y0=args.y0, note=note)]
    for name, doc, px in (
        ("LEFT_ART", "Left half.", px_l),
        ("RIGHT_ART", "Right half.", px_r),
    ):
        out.append(PER_ART.format(name=name, doc=doc, n=n_l,
                                  body=format_body(px)))
    text = "".join(out)

    Path(args.out).write_text(text, encoding="utf-8")
    print("wrote %s (%d x %d, stride %d, %d bytes per drawing)"
          % (args.out, w, h, stride, n_l))


if __name__ == "__main__":
    main()