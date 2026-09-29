"""Report the flash extent of an Intel HEX file, and the UF2 block count it implies.

Written for one question and kept because the question recurs: two halves built
from the same source do not always produce UF2 files of the same size, and the
reason is usually that one of them crossed a 256-byte payload boundary rather
than that one of them gained 512 bytes of code. UF2 packs 256 bytes of payload
per 512-byte block, so a one-byte growth can show up as a whole extra block.

Usage: python tools\\hex_extent.py file.hex [file.hex ...]
"""

import sys


def extent(path):
    """Return (lowest address, one past the highest address) of the data in `path`."""
    base = 0          # upper 16 bits, from a type-04 extended-linear-address record
    lo = None
    hi = 0
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            rec = bytes.fromhex(line[1:])
            count = rec[0]
            addr = (rec[1] << 8) | rec[2]
            kind = rec[3]
            if kind == 0x00:                       # data
                start = base + addr
                end = start + count
                lo = start if lo is None else min(lo, start)
                hi = max(hi, end)
            elif kind == 0x04:                     # extended linear address
                data = rec[4 : 4 + count]
                base = ((data[0] << 8) | data[1]) << 16
            elif kind == 0x02:                     # extended segment address
                data = rec[4 : 4 + count]
                base = ((data[0] << 8) | data[1]) << 4
            elif kind == 0x01:                     # end of file
                break
    return lo, hi


for path in sys.argv[1:]:
    lo, hi = extent(path)
    if lo is None:
        print(f"{path}: no data records")
        continue
    size = hi - lo
    # A UF2 block carries 256 bytes of payload, and the image is written from its
    # own base, so the block count follows the span rather than the highest address.
    blocks = -(-size // 256)
    print(
        f"{path}: 0x{lo:05X}..0x{hi:05X}  span {size} B  "
        f"-> {blocks} UF2 blocks -> {blocks * 512} B uf2"
    )
