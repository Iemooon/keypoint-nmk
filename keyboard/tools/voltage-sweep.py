"""Build both halves' firmwares, one per candidate "cell full" voltage.

Lemon sweeps these by hand: flash one, charge to full, read the percentage when
the charge light goes out, flash the next. The anchor that reads 100 % is the one
that stays.

Both halves are built for every voltage. A half only ever reads its own arm -
left.rs never constructs `Role::Right`, right.rs never constructs `Role::Left` -
so the left and right sweeps are independent even though each pass writes the
same number into both arms.

    python tools/voltage-sweep.py                  # 4.15 / 4.16 / 4.17 / 4.18 V
    python tools/voltage-sweep.py 4.14 4.15        # any list, in volts
    python tools/voltage-sweep.py --half right     # one half only: left|right|both

Run from the `keyboard/` directory. The same invocation is what CI uses
(.github/workflows/build.yml), so the online build and a local sweep produce the
identical set of images from identical code - there is no second implementation to
drift.

Note that each pass leaves `keypoint-nmk-<half>.uf2` holding that voltage's
build, so after a run the plain files hold the last voltage in the list.

The source file is restored from a backup in a `finally` block, so an interrupted
run still leaves `board.rs` as it was.
"""

import hashlib
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                      # ...\keypoint-nmk\tx
BOARD = os.path.join(ROOT, "src", "board.rs")
BACKUP = os.path.join(ROOT, "src", "board.rs.voltagesweep.bak")

# Anchored on the decimal form together with its trailing comma. The comma is
# what keeps these away from the other `Role::Left/Right =>` arms in the same
# match - the pipe tables, the names, the pointer keys - and from `capy_salt`'s
# `Role::Right => 0x9e37_79b9`, where `\d+` would otherwise start eating the
# hex digits.
ANCHOR = {
    "left": re.compile(r"(Role::Left => )(\d+),"),
    "right": re.compile(r"(Role::Right => )(\d+),"),
}


def toolchain_env():
    # RMK's crypto path wants arm-none-eabi-gcc; the only copy on this machine
    # lives in MSYS, so the build needs it on PATH.
    env = dict(os.environ)
    # QMK_MSYS is this machine's arm-none-eabi-gcc (RMK's crypto build script
    # shells out to it). Prepend only when it exists, so a machine that already
    # has the toolchain on PATH - CI, for instance - is not broken by its absence.
    for _p in (r"C:\QMK_MSYS\mingw64\bin", r"C:\QMK_MSYS\opt\qmk\bin",
               r"C:\QMK_MSYS\usr\bin"):
        if os.path.isdir(_p):
            env["PATH"] = _p + os.pathsep + env["PATH"]
    return env


def build(half, env):
    name = "keypoint-nmk-" + half
    for cmd in (
        ["cargo", "build", "--release", "--bin", half],
        ["cargo", "objcopy", "--release", "--bin", half, "--",
         "-O", "ihex", name + ".hex"],
        ["cargo", "hex-to-uf2", "--input-path", name + ".hex",
         "--output-path", name + ".uf2", "--family", "nrf52840"],
    ):
        if subprocess.run(cmd, cwd=ROOT, env=env).returncode:
            sys.exit("failed: " + " ".join(cmd))


def digest(path):
    h = hashlib.md5()
    with open(path, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()


def main():
    argv = sys.argv[1:]
    halves = ["left", "right"]
    if "--half" in argv:
        i = argv.index("--half")
        if i + 1 >= len(argv):
            sys.exit("--half takes left, right or both")
        want = argv[i + 1]
        del argv[i:i + 2]
        if want not in ("left", "right", "both"):
            sys.exit("--half takes left, right or both")
        halves = ["left", "right"] if want == "both" else [want]
    volts = [float(v) for v in argv] or [4.15, 4.16, 4.17, 4.18]

    env = toolchain_env()
    src = open(BOARD, encoding="utf-8").read()
    for half, pat in ANCHOR.items():
        if len(pat.findall(src)) != 1:
            sys.exit("expected exactly one 'Role::%s => <mv>,' in board.rs"
                     % half.capitalize())

    shutil.copyfile(BOARD, BACKUP)
    out = []
    try:
        for v in volts:
            mv = int(round(v * 1000))
            edited = ANCHOR["left"].sub(r"\g<1>%d," % mv, src)
            edited = ANCHOR["right"].sub(r"\g<1>%d," % mv, edited)
            open(BOARD, "w", encoding="utf-8").write(edited)
            for half in halves:
                build(half, env)
                dst = os.path.join(ROOT, "keypoint-nmk-%s-%.2fV.uf2" % (half, v))
                shutil.copyfile(os.path.join(ROOT, "keypoint-nmk-%s.uf2" % half),
                                dst)
                out.append((half, v, mv, mv * 1146 // 1000, dst))
    finally:
        shutil.copyfile(BACKUP, BOARD)
        os.remove(BACKUP)
        print("board.rs restored")

    print()
    seen = {}
    for half, v, mv, counts, dst in out:
        d = digest(dst)
        seen.setdefault(d, []).append("%s %.2fV" % (half, v))
        print("  %-5s %.2f V  mv=%-5d counts=%-5d  %8d bytes  md5 %s"
              % (half, v, mv, counts, os.path.getsize(dst), d[:12]))
    dupes = {d: vs for d, vs in seen.items() if len(vs) > 1}
    if dupes:
        sys.exit("two firmwares are byte-identical: %s" % dupes)
    print("\nall %d differ - each is a real change" % len(out))


if __name__ == "__main__":
    main()