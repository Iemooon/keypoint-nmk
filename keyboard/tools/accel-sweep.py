"""Build right-half firmwares with different TrackPoint acceleration curves.

The acceleration lives in `src/board.rs` (`TrackPoint::accel_factor`):

    dist   = |dx| + |dy|                counts in this packet
    speed  = dist / gap_ms              counts per millisecond
    mult   = min(cap, exp(speed * gain))    gain = TP_ACCEL_BASE_GAIN * pct/100
    out    = dx * TP_BASE_SPEED * 1.30 * mult

`gain` is only ever `TP_ACCEL_BASE_GAIN (1.307357) * TP_ACCEL_PERCENT/100`, and
`cap` is `TP_ACCEL_MAX_MULT (1.5, from ZMK's 150 % ceiling)`. Those are the two
knobs, so those are what this script rewrites.

Why a sweep is needed at all: with the ZMK numbers the curve is pinned at its
ceiling across the whole usable range. The threshold is

    speed > ln(1.5) / 1.307357 = 0.31 counts/ms

and a stick pushed past the deadzone is already there - `dist >= 3` over a 10 ms
packet is 0.30, over 5 ms it is 0.60. So the firmware behaves like a flat
1.30 x 1.5 = 1.95 gain, and "slow is precise, fast is quick" cannot be felt.
Lowering the percentage is what opens the low end back up; raising the ceiling is
what gives the fast end somewhere to go.

Only the right half has a TrackPoint, so only that uf2 is built.

    python tools\\accel-sweep.py                    # the four below
    python tools\\accel-sweep.py name:pct:cap ...   # e.g. soft:25:2.0

`board.rs` is restored from a backup in a `finally`, as in `voltage-sweep.py`.
"""

import hashlib
import math
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BOARD = os.path.join(ROOT, "src", "board.rs")
BACKUP = os.path.join(ROOT, "src", "board.rs.accelsweep.bak")

PCT = re.compile(r"(const TP_ACCEL_PERCENT: f32 = )[\d.]+;")
CAP = re.compile(r"(const TP_ACCEL_MAX_MULT: f32 = )[\d.]+;")
BASE_GAIN = 1.307357

DEFAULT = [
    # name,          pct,  cap    what it is for
    ("accel-off",    0.0,  1.0),  # control: no curve at all, flat 1.30x
    ("accel-stock",  100.0, 1.5),  # control: what is on the keyboard now
    ("accel-gentle", 50.0, 2.5),  # opens the slow end, lets the fast end run
    ("accel-steep",  75.0, 3.0),  # same idea, more of both
]

# Speeds in counts/ms, to show what each curve does: 0.3 is a light push on a
# 10 ms packet, 1.2 is a hard shove.
SAMPLE = [0.2, 0.3, 0.5, 0.8, 1.2, 2.0]


def literal(x):
    """`%g` drops the trailing `.0`, and `1` is not an f32 literal."""
    s = "%g" % x
    return s if ("." in s or "e" in s) else s + ".0"


def build_right(env):
    for cmd in (
        ["cargo", "build", "--release", "--bin", "right"],
        ["cargo", "objcopy", "--release", "--bin", "right", "--",
         "-O", "ihex", "keypoint-nmk-right.hex"],
        ["cargo", "hex-to-uf2", "--input-path", "keypoint-nmk-right.hex",
         "--output-path", "keypoint-nmk-right.uf2", "--family", "nrf52840"],
    ):
        if subprocess.run(cmd, cwd=ROOT, env=env).returncode:
            sys.exit("failed: " + " ".join(cmd))


def digest(path):
    h = hashlib.md5()
    with open(path, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()


def parse(argv):
    if not argv:
        return DEFAULT
    cases = []
    for arg in argv:
        name, pct, cap = arg.split(":")
        cases.append((name, float(pct), float(cap)))
    return cases


def main():
    cases = parse(sys.argv[1:])
    env = dict(os.environ)
    # QMK_MSYS is this machine's arm-none-eabi-gcc (RMK's crypto build script
    # shells out to it). Prepend only when it exists, so a machine that already
    # has the toolchain on PATH - CI, for instance - is not broken by its absence.
    for _p in (r"C:\QMK_MSYS\mingw64\bin", r"C:\QMK_MSYS\opt\qmk\bin",
               r"C:\QMK_MSYS\usr\bin"):
        if os.path.isdir(_p):
            env["PATH"] = _p + os.pathsep + env["PATH"]

    src = open(BOARD, encoding="utf-8").read()
    for rx_ in (PCT, CAP):
        if len(rx_.findall(src)) != 1:
            sys.exit("expected exactly one %s in board.rs" % rx_.pattern)
    shutil.copyfile(BOARD, BACKUP)

    built = []
    try:
        for name, pct, cap in cases:
            text = PCT.sub(lambda m: m.group(1) + literal(pct) + ";", src)
            text = CAP.sub(lambda m: m.group(1) + literal(cap) + ";", text)
            open(BOARD, "w", encoding="utf-8").write(text)
            build_right(env)
            dst = os.path.join(ROOT, "keypoint-nmk-right-%s.uf2" % name)
            shutil.copyfile(os.path.join(ROOT, "keypoint-nmk-right.uf2"), dst)
            built.append((name, pct, cap, dst))
    finally:
        shutil.copyfile(BACKUP, BOARD)
        os.remove(BACKUP)
        print("board.rs restored\n")

    head = "  %-13s %5s %8s %4s" % ("name", "pct", "gain", "cap")
    head += "".join("%9s" % ("v=%.1f" % s) for s in SAMPLE)
    print(head)
    seen = {}
    for name, pct, cap, dst in built:
        gain = BASE_GAIN * pct / 100.0
        curve = "".join(
            "%9.2f" % (cap if math.exp(s * gain) > cap else math.exp(s * gain))
            for s in SAMPLE)
        print("  %-13s %5g %8.3f %4g%s" % (name, pct, gain, cap, curve))
        seen.setdefault(digest(dst), []).append(name)
    print("\n  mult is the multiplier applied to the raw count; the total gain on")
    print("  screen is 1.30 * mult (1.30 = TP_SENS_BASE + TP_SENS_STEP*100).")
    dupes = {d: ns for d, ns in seen.items() if len(ns) > 1}
    if dupes:
        sys.exit("byte-identical firmwares: %s" % dupes)
    for name, _, _, dst in built:
        print("  %-13s %10d bytes  md5 %s"
              % (name, os.path.getsize(dst), digest(dst)[:12]))


if __name__ == "__main__":
    main()
