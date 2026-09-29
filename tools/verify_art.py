# -*- coding: utf-8 -*-
"""Check that each half's uf2 really carries its own drawing and not the other's.

Reads the two pixel arrays out of `keyboard/src/screen/art.rs`, then looks for long
windows of those bytes inside each uf2. The firmware copies flash to RAM with no
transformation, so a run of bytes in the image should appear verbatim - the only
wrinkle is that uf2 wraps the image in 256-byte blocks with a 512-byte record
header, so a match has to be long enough not to be swamped but short enough that
the odds of a random hit are negligible. 24 bytes is 2^-192.
"""
import re
import sys
from pathlib import Path

AR = Path("keyboard/src/screen/art.rs")
WINDOW = 24
STARTS = 400  # sample windows spread across the drawing


def arrays(path):
    text = path.read_text(encoding="utf-8")
    out = {}
    for name in ("LEFT_ART_PIXELS", "RIGHT_ART_PIXELS"):
        m = re.search(name + r"\s*:\s*\[u8;\s*(\d+)\s*\]\s*=\s*\[(.*?)\];",
                      text, re.S)
        if not m:
            sys.exit("no array %s in %s" % (name, path))
        out[name] = bytes(int(v, 16) for v in re.findall(r"0x([0-9a-fA-F]{2})",
                                                         m.group(2)))
    return out


def hits(firmware, pixels):
    """How many sampled windows of `pixels` occur verbatim in `firmware`."""
    n = len(pixels)
    step = max(1, (n - WINDOW) // STARTS)
    found = 0
    total = 0
    for off in range(0, n - WINDOW, step):
        total += 1
        if firmware.find(pixels[off:off + WINDOW]) >= 0:
            found += 1
    return found, total


def main():
    art = arrays(AR)
    rei = art["LEFT_ART_PIXELS"]      # left half shows this
    asuka = art["RIGHT_ART_PIXELS"]   # right half shows this
    print("rei   %d bytes, nonzero %d" % (len(rei), sum(1 for b in rei if b)))
    print("asuka %d bytes, nonzero %d" % (len(asuka), sum(1 for b in asuka if b)))
    print()
    ok = True
    for half, want, other in (("left", rei, asuka), ("right", asuka, rei)):
        blob = Path("keyboard/keypoint-nmk-%s.uf2" % half).read_bytes()
        wf, wt = hits(blob, want)
        of, ot = hits(blob, other)
        # `other` does not have to be near zero. A window made mostly of background
        # - 0x00 bytes - occurs in *both* drawings, so any picture with a large black
        # background scores false positives against the other half. What actually
        # matters is that this half's own drawing matches far more often than the
        # other one does. (2026-09-26: the old `of < wt / 10` rule cried SUSPECT on a
        # correct build whose left drawing is 61% black.)
        verdict = "OK" if wf > wt / 2 and of < wf / 3 else "SUSPECT"
        ok = ok and verdict == "OK"
        print("%-5s uf2 %7d B | own drawing %3d/%d windows | other %3d/%d | %s"
              % (half, len(blob), wf, wt, of, ot, verdict))
    print()
    print("PASS" if ok else "FAIL")


if __name__ == "__main__":
    main()