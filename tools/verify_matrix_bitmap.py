"""Verify the matrix bitmap format against the layout it replaced.

WHY THIS EXISTS
---------------
Before 2026-09-17 each half's report was "one byte per row, one bit per column":
payload[row] held row `row`'s column bits. That capped a half at 8 columns,
because a byte has 8 bits - so an 8x16 keyboard could not be expressed at all.

The report is now ONE BITMAP: bit `row * COLS + col`, rows concatenated. Nothing
about the wire format changed for this keyboard, and that is the claim this
script checks rather than asserts:

  * with 8 columns, `bit / 8` is the row index and `bit % 8` is the column, so
    the bitmap IS the old row-byte layout, byte for byte. 6x8 stays 6 bytes and
    the receiver's existing packets keep arriving unchanged.

It then checks the property that made the change worth making: the round trip
through the transmitter's pack and the receiver's unpack is exact for geometries
the old layout could not express (8x16, 12x16, 16x16), up to Gazell's 32-byte
payload limit = 256 cells.

The two sides are re-implemented here exactly as keyboard/src/board.rs (set_bit / scan)
and receiver/src/gazell.rs (the unpack loop) do them. Run:  python verify_matrix_bitmap.py
"""

import random

# --- the transmitter's bit set, keyboard/src/board.rs -------------------------------

def set_bit(bitmap, r, c, cols):
    """set_bit() in keyboard/src/board.rs, with COLS passed in."""
    bit = r * cols + c
    bitmap[bit // 8] |= 1 << (bit % 8)


def scan(rows_bits, rows, cols):
    """What scan() produces: the row bytes packed into the report bitmap."""
    out = [0] * (-(-rows * cols // 8))
    for r in range(rows):
        for c in range(cols):
            if rows_bits[r] & (1 << c):
                set_bit(out, r, c, cols)
    return out


# --- the receiver's unpack, receiver/src/gazell.rs ----------------------------------

def unpack(payload, rows, cols):
    """The drain_pipe() loop: bitmap back into one byte per row."""
    out = [0] * rows
    for r in range(rows):
        bits = 0
        for c in range(cols):
            bit = r * cols + c
            if payload[bit // 8] & (1 << (bit % 8)):
                bits |= 1 << c
        out[r] = bits
    return out


# --- the layout that shipped before -------------------------------------------

def old_layout(rows_bits, rows):
    """One byte per row, one bit per column, sent verbatim."""
    return list(rows_bits)


# =============================================================================
# 1. 6x8: the bitmap is the old layout, byte for byte
# =============================================================================

R, C = 6, 8
nbytes = -(-R * C // 8)
assert nbytes == R, f"6x8 must still be {R} bytes, got {nbytes}"

cases = [[0] * R, [0xFF] * R]                      # nothing held / everything held
for r in range(R):                                 # every single key alone
    for c in range(C):
        s = [0] * R
        s[r] = 1 << c
        cases.append(s)
for r in range(R):                                 # every single row alone
    s = [0] * R
    s[r] = 0xFF
    cases.append(s)
for _ in range(200000):                            # and a large random sample
    cases.append([random.randrange(256) for _ in range(R)])

bad = [s for s in cases if scan(s, R, C) != old_layout(s, R)]
print(f"6x8 identical to the old one-byte-per-row layout: {not bad}  "
      f"({len(cases)} patterns, {len(bad)} mismatches)")
for s in bad[:5]:
    print(f"    MISMATCH rows={s} bitmap={scan(s, R, C)} old={old_layout(s, R)}")

# =============================================================================
# 2. round trip, including geometries the old layout could not express
# =============================================================================

GEOMETRIES = [(6, 8), (4, 8), (1, 8), (8, 1), (8, 8), (8, 16), (12, 16),
              (16, 16), (5, 13), (25, 8)]

print("\nround trip (transmitter pack -> receiver unpack):")
rt_fail = 0
checked = 0
for rows, cols in GEOMETRIES:
    nb = -(-rows * cols // 8)
    if nb > 32:
        print(f"  {rows:>2} x {cols:<2} -> {nb:>2} bytes  "
              f"REJECTED: build.rs refuses this (Gazell payload is 32 bytes)")
        continue
    local_fail = 0
    for _ in range(20000):
        # random cell state, so the bitmap is exercised directly
        cells = [random.randrange(2) for _ in range(rows * cols)]
        rows_bits = [0] * rows
        for r in range(rows):
            for c in range(cols):
                if cells[r * cols + c]:
                    rows_bits[r] |= 1 << c
        if unpack(scan(rows_bits, rows, cols), rows, cols) != rows_bits:
            local_fail += 1
            if local_fail <= 2:
                print(f"    FAIL {rows}x{cols}: {rows_bits}")
    checked += 20000
    rt_fail += local_fail
    note = "" if (rows, cols) != (6, 8) else "   <- this keyboard"
    print(f"  {rows:>2} x {cols:<2} -> {nb:>2} bytes  "
          f"{'exact' if local_fail == 0 else 'FAILED'}{note}")

print(f"\nround trip: {checked} cases, {rt_fail} failures")

# =============================================================================
# 3. the cell budget both build scripts enforce (256 cells = 32 bytes)
# =============================================================================

print("\ncell budget (Gazell payload limit = 32 bytes = 256 cells):")
for rows, cols in [(6, 8), (8, 8), (8, 16), (12, 16), (16, 16), (32, 8),
                   (25, 8), (16, 17), (32, 9)]:
    cells = rows * cols
    nb = -(-cells // 8)
    verdict = "ok" if nb <= 32 else "REFUSED by build.rs"
    print(f"  {rows:>2} x {cols:<2} = {cells:>3} cells -> {nb:>2} bytes  {verdict}")

ok = (not bad) and rt_fail == 0
print("\nRESULT:", "PASS" if ok else "FAIL")