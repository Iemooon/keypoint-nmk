# -*- coding: utf-8 -*-
r"""Confirm art.rs carries each uploaded source verbatim, byte for byte.

Runs after make_art.py. Compares LEFT_ART_PIXELS against the left source file's
array and RIGHT_ART_PIXELS against the right one - if either half got the other's
drawing, or a stale one, this says so.

Each source file produced by png_to_screen.py declares its array under the name
given on the command line (`--name REI3` -> `pub const REI3: [u8; 4320] = [...]`).

Usage:  python tools\check_art.py <left_src.rs> <right_src.rs>
"""
import re
import sys
from pathlib import Path

ART = Path("keyboard/src/screen/art.rs")
DECL = r"(\w+)\s*:\s*\[u8;\s*\d+\s*\]\s*=\s*\[(.*?)\];"


def arrays(path):
    text = Path(path).read_text(encoding="utf-8")
    out = {}
    for name, body in re.findall(DECL, text, re.S):
        out[name] = bytes(int(v, 16) for v in re.findall(r"0x([0-9a-fA-F]{2})", body))
    if not out:
        sys.exit("no u8 array in %s" % path)
    return out


def one(path):
    got = arrays(path)
    name = sorted(got, key=lambda n: -len(got[n]))[0]
    return name, got[name]


art = arrays(ART)
ok = True
for target, src in (("LEFT_ART_PIXELS", sys.argv[1]), ("RIGHT_ART_PIXELS", sys.argv[2])):
    src_name, src_bytes = one(src)
    a = art.get(target)
    same = a == src_bytes
    ok = ok and same
    print("%-16s %5d bytes  vs %-10s %-10s -> %s"
          % (target, len(a) if a else -1, Path(src).name, src_name,
             "same" if same else "DIFFERENT"))
print("PASS" if ok else "FAIL")
