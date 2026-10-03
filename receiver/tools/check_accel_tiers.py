"""Check which acceleration rungs actually made it into the built firmware.

The two ladders in `pointer_accel.rs` are `const` tables indexed at run time, so
they have to exist in flash - but "compiles" is not evidence that a retuned value
landed, and a respace changes every rung at once. This walks the Intel HEX
products, rebuilds the image bytes, and looks for the little-endian `f32`
patterns of every rung the source currently defines.

The rungs are READ FROM THE SOURCE rather than typed here. The first version of
this tool hard-coded them, and a later respace left it checking a ladder that no
longer existed - a check that cannot fail is worse than no check.

Read the result as: every rung the source defines should be present in the image.
A missing rung means a stale build, a table the optimiser dropped, or a renamed
array - check the offset before believing it. `OLD` is the fingerprint of the
ladder this one replaced (twenty-four rungs, gain a fiftieth per step, cap a
tenth starting at 0.1): a hit there means either a leftover table or a
coincidence - check the offset before believing it too.
"""

import re
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
SOURCE = HERE / "src" / "pointer_accel.rs"

# Fingerprints of the previous (twenty-four-rung) ladders; none of these is in the
# current tables.
OLD = [0.02, 0.48, 0.1, 2.4]

LADDERS = ("ACCEL_GAIN_TIERS", "ACCEL_CAP_TIERS")


def rungs(name: str) -> list[float]:
    """The f32 values of one `pub const <name>: [f32; N] = [...]` array."""
    src = SOURCE.read_text()
    m = re.search(rf"pub const {name}[^=]*=\s*\[(.*?)\];", src, re.S)
    if not m:
        raise SystemExit(f"{name} not found in {SOURCE}")
    # Drop the row comments first: the cap ladder's comments repeat the value
    # ("// F2  1.1x"), and a raw number search would count each of those twice.
    body = re.sub(r"//[^\n]*", "", m.group(1))
    return [float(v) for v in re.findall(r"\d+\.\d+", body)]


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
    missing: list[float] = []
    for ladder in LADDERS:
        print(f"  {ladder} (read from {SOURCE.name}):")
        for value in rungs(ladder):
            at = blob.find(struct.pack("<f", value))
            where = f"0x{at:x}" if at >= 0 else "--"
            print(f"    {value:5.2f}  {where}")
            if at < 0:
                missing.append(value)
    print("  fingerprints of the previous ladder (expected absent):")
    for value in OLD:
        at = blob.find(struct.pack("<f", value))
        where = f"0x{at:x}" if at >= 0 else "--"
        print(f"    {value:5.2f}  {where}")
    print(f"  all rungs present: {not missing}" + (f"  MISSING: {missing}" if missing else ""))
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
