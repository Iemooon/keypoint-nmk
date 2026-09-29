"""Decode the Vial definition this receiver was built with.

`build.rs` XZ-compresses `vial.json` into the `VIAL_KEYBOARD_DEF` constant that
the firmware serves to Vial. This reads that constant back out of the generated
file and decodes it, so "did my vial.json edit actually reach the firmware" has
a yes/no answer instead of being a guess.

Usage:
    python tools/check_vial_blob.py            # newest config_generated.rs
    python tools/check_vial_blob.py <path>     # one specific file
"""

import glob
import json
import lzma
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def newest_generated():
    pattern = os.path.join(
        ROOT, "target", "**", "build", "keypoint-nmk-receiver-*", "out", "config_generated.rs"
    )
    paths = glob.glob(pattern, recursive=True)
    if not paths:
        sys.exit("no config_generated.rs under target/ - build the receiver first")
    paths.sort(key=os.path.getmtime, reverse=True)
    return paths[0]


def decode(path):
    text = open(path, encoding="utf-8").read()
    m = re.search(r"VIAL_KEYBOARD_DEF[^=]*=\s*&\[(.*?)\];", text, re.S)
    if not m:
        sys.exit("%s: no VIAL_KEYBOARD_DEF array found" % path)
    data = bytes(int(n) for n in re.findall(r"(\d+)u8", m.group(1)))

    dec = lzma.LZMADecompressor(format=lzma.FORMAT_XZ)
    raw = dec.decompress(data)
    if not dec.eof:
        print("note: the XZ stream did not run to its end marker")
    return data, raw


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest_generated()
    data, raw = decode(path)
    doc = json.loads(raw)

    print("file          : %s" % path)
    print("xz bytes      : %d" % len(data))
    print("json bytes    : %d" % len(raw))
    print("top-level keys: %s" % ", ".join(doc.keys()))
    print("name          : %s" % doc.get("name"))
    print("vid:pid       : %s:%s" % (doc.get("vendorId"), doc.get("productId")))
    matrix = doc.get("matrix") or {}
    print("matrix        : %sx%s" % (matrix.get("rows"), matrix.get("cols")))

    if "customKeycodes" in doc:
        ck = doc["customKeycodes"]
        print("customKeycodes: PRESENT (%d entries)" % len(ck))
        for i, item in enumerate(ck):
            print("   [%d] %s" % (i, item.get("title", item.get("name"))))
    else:
        print("customKeycodes: ABSENT")

    layout = doc.get("layouts", {}).get("keymap")
    print("layout rows   : %s" % (len(layout) if layout is not None else "MISSING"))


if __name__ == "__main__":
    main()
