"""Check which acceleration rungs actually made it into the built firmware.

The two ladders in `pointer_accel.rs` are `const` tables indexed at run time, so
they have to exist in flash - but "compiles" is not evidence that a retuned value
landed, and the ladder respacing changes every rung at once. This walks the Intel
HEX products, rebuilds the image bytes, and looks for the little-endian `f32`
patterns of rungs that distinguish the new ladder from the old one.

Read the result as two questions:

  * Did the new ladder land? 0.02 (new bottom), 0.22 (the gain in use) and 0.48
    (new top) must all be present.
  * Did the old one leave? 0.05 / 0.15 / 1.20 were twentieths rungs and are absent
    from the new table. A hit means either a leftover table or a coincidence -
    check the offset before believing it.
"""

import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent

NEW = [0.02, 0.04, 0.22, 0.24, 0.46, 0.48]
OLD = [0.05, 0.15, 1.05, 1.20]


def image(path: Path) -> bytes:
    """Concatenate the data records of an Intel HEX file."""
    out = bytearray()
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line.startswith(":"):
            continue
        raw = bytes.fromhex(line[1:])
        length, kind = raw[0], raw[3]
        if kind == 0:
            out.extend(raw[4 : 4 + length])
    return bytes(out)


def report(path: Path) -> None:
    blob = image(path)
    print(f"{path.name}  ({len(blob)} bytes of image)")
    for label, rungs in (("new", NEW), ("old", OLD)):
        for value in rungs:
            at = blob.find(struct.pack("<f", value))
            where = f"0x{at:x}" if at >= 0 else "--"
            print(f"  {label}  {value:5.2f}  {where}")
    print()


def main() -> int:
    products = sorted(HERE.glob("keypoint-nmk-receiver-*.hex"))
    if not products:
        print(f"no products found under {HERE}", file=sys.stderr)
        return 1
    for path in products:
        report(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
