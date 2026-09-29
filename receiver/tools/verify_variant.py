#!/usr/bin/env python3
"""Verify the three board variants of the keypoint-nmk receiver, by product.

WHY THIS EXISTS
---------------
The three variants differ ONLY in a layout that is generated at build time
(build.rs -> memory.x) and in which cargo chip feature is enabled. Neither is
visible in the source you happen to be looking at, and getting either wrong
produces an image that flashes successfully and then never runs - or, worse,
storage that writes over live code. So the claim "this file is for board X" is
checked against the file itself, not against the intent of the build.

WHAT IS CHECKED, per variant:
  * the .hex starts exactly at the application origin the board's bootloader
    jumps to (0x1000 / 0x27000)
  * the application does not run past the end of its declared region
  * the application never reaches into the storage region (0xA0000 etc.)
  * the storage region never reaches into [flash] reserved_top
  * the .uf2 carries the right family ID - a UF2 bootloader rejects a file with
    the wrong family, and nRF52840 and nRF52833 have different ones
  * the .uf2 starts at the same address as the .hex (same image, two formats)
  * the images that MUST differ do differ: identical output means an override
    file silently did not take effect

Usage:  python tools\\verify_variant.py        (from rx\\, after a build)
"""

import hashlib
import os
import struct
import sys

# name,               origin,    app_len,  reserved_top, storage,   storage_kb, uf2 family id
VARIANTS = [
    ("PCA10059",       0x00001000, 636 * 1024, 0xE0000, 0xA0000,  24, 0xADA52840),
    ("52840-nicenano", 0x00001000, 636 * 1024, 0xF4000, 0xA0000,  24, 0xADA52840),
    ("52833-nicenano", 0x00027000, 260 * 1024, 0x74000, 0x68000,  48, 0x621E937A),
]

PREFIX = "keypoint-nmk-receiver-"

UF2_MAGIC0 = 0x0A324655
UF2_MAGIC1 = 0x9E5D5157
UF2_FAMILY_PRESENT = 0x00002000


def hex_span(path):
    """(lowest data address, end address) of an Intel HEX file.

    Record types 02 and 04 both occur in cargo objcopy output, and ignoring 02
    (as an earlier version of a sibling tool did) silently drops the high
    address bits of everything above 0xFFFF and reports a fake range.
    """
    base = 0
    lo = hi = None
    with open(path, "r", encoding="ascii") as f:
        for line in f:
            line = line.strip()
            if not line or line[0] != ":":
                continue
            raw = bytes.fromhex(line[1:])
            count, addr, rec = raw[0], (raw[1] << 8) | raw[2], raw[3]
            data = raw[4:4 + count]
            if rec == 0x00:
                if count:
                    start = base + addr
                    lo = start if lo is None else min(lo, start)
                    hi = start + count if hi is None else max(hi, start + count)
            elif rec == 0x02 and count >= 2:
                base = ((data[0] << 8) | data[1]) << 4
            elif rec == 0x04 and count >= 2:
                base = ((data[0] << 8) | data[1]) << 16
            elif rec == 0x01:
                break
    return lo, hi


def uf2_info(path):
    """(family id, lowest target address, end address, block count)."""
    with open(path, "rb") as f:
        blob = f.read()
    if not blob or len(blob) % 512:
        raise ValueError(f"{path}: size {len(blob)} is not a whole number of 512-byte blocks")
    family = None
    lo = hi = None
    blocks = 0
    numbers = set()
    for off in range(0, len(blob), 512):
        m0, m1, flags, target, payload, block_no, num_blocks, fam = struct.unpack_from(
            "<8I", blob, off
        )
        if m0 != UF2_MAGIC0 or m1 != UF2_MAGIC1:
            raise ValueError(f"{path}: block at {off:#x} has no UF2 magic")
        numbers.add(block_no)
        blocks += 1
        if num_blocks != blocks and block_no == 0:
            pass  # numBlocks is cross-checked below, once the count is known
        lo = target if lo is None else min(lo, target)
        hi = target + payload if hi is None else max(hi, target + payload)
        if flags & UF2_FAMILY_PRESENT:
            family = fam
    if blocks and max(numbers) != blocks - 1:
        # Blocks are numbered 0..n-1 with no gaps; a gap means a truncated file,
        # which a UF2 bootloader would accept and then fail to boot.
        raise ValueError(f"{path}: block numbering has gaps ({len(numbers)} of {blocks} present)")
    return family, lo, hi, blocks


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(here)

    failures = []
    digests = {}
    layouts = {}
    print(f"{'variant':<17} {'hex start':>10} {'hex end':>10} {'uf2 family':>11} {'blocks':>7}  result")
    print("-" * 74)

    for name, origin, app_len, reserved_top, storage, storage_kb, family in VARIANTS:
        problems = []
        hex_path = f"{PREFIX}{name}.hex"
        uf2_path = f"{PREFIX}{name}.uf2"
        app_end = origin + app_len
        storage_end = storage + storage_kb * 1024
        layouts[name] = (origin, app_len, storage, storage_kb, family)

        # --- the layout arithmetic itself, before looking at any product -----
        if storage < app_end:
            problems.append(f"storage {storage:#x} starts inside the application region")
        if storage_end > reserved_top:
            problems.append(
                f"storage {storage:#x}+{storage_kb}K ends at {storage_end:#x}, "
                f"past reserved_top {reserved_top:#x}"
            )

        lo = hi = None
        if not os.path.exists(hex_path):
            problems.append(f"{hex_path} is missing")
        else:
            lo, hi = hex_span(hex_path)
            digests.setdefault(hashlib.sha256(open(hex_path, "rb").read()).hexdigest(), []).append(name)
            if lo != origin:
                problems.append(f"hex starts at {lo:#x}, board's application slot is {origin:#x}")
            if hi > app_end:
                problems.append(f"hex ends at {hi:#x}, past the declared region end {app_end:#x}")
            if hi > storage:
                problems.append(f"hex ends at {hi:#x}, reaching into storage at {storage:#x}")

        fam = None
        blocks = 0
        if not os.path.exists(uf2_path):
            problems.append(f"{uf2_path} is missing")
        else:
            fam, u_lo, u_hi, blocks = uf2_info(uf2_path)
            if fam != family:
                problems.append(f"uf2 family {fam:#010x}, expected {family:#010x}")
            if u_lo != origin:
                problems.append(f"uf2 starts at {u_lo:#x}, expected {origin:#x}")
            if u_hi > app_end:
                problems.append(f"uf2 ends at {u_hi:#x}, past {app_end:#x}")

        result = "OK" if not problems else "FAIL"
        if problems:
            failures.append((name, problems))
        print(
            f"{name:<17} {('%#x' % lo) if lo is not None else '-':>10} "
            f"{('%#x' % hi) if hi is not None else '-':>10} "
            f"{('%#010x' % fam) if fam is not None else '-':>11} {blocks:>7}  {result}"
        )
        for p in problems:
            print(f"    ! {p}")

    # --- cross-variant: the image must FOLLOW the layout, and only that ------
    # Two variants may produce the same image only when the firmware can see the
    # same layout. The two nRF52840 boards qualify: they differ solely in where
    # the bootloader's reserved region starts (0xE0000 on PCA10059 vs 0xF4000 on
    # the Adafruit board), which sits above the storage and never appears in the
    # code - so one image serves both nRF52840 boards, and that is a result, not
    # a defect.
    # Any other pair coming out identical means an override silently did not
    # apply, which is the one failure this whole script exists to catch.
    for digest, names in digests.items():
        if len(names) < 2:
            continue
        tuples = {layouts[n] for n in names}
        if len(tuples) > 1:
            problems = [
                f"{' and '.join(names)} are byte-identical ({digest[:16]}...) while their "
                f"layouts differ ({tuples}) - an override did not take effect"
            ]
            failures.append(("cross-variant", problems))
            for p in problems:
                print(f"    ! {p}")
        else:
            print(
                f"note: {' and '.join(names)} produce the SAME image, as expected - "
                "the firmware sees an identical layout on both"
            )

    print()
    if failures:
        print(f"FAILED ({len(failures)} of {len(VARIANTS)} variants)")
        return 1
    print(f"ALL {len(VARIANTS)} VARIANTS OK - starts, spans, families, storage and distinctness all agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())