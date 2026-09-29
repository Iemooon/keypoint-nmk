"""Report a flash span from an Intel HEX file, and (optionally) compare two
Gazell archives so the library swap is documented with numbers, not memory.
"""
import os

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

import re
import sys

def hex_span(path):
    base = 0
    lo = hi = None
    n = 0
    for line in open(path):
        line = line.strip()
        if not line.startswith(":"):
            continue
        cnt = int(line[1:3], 16)
        addr16 = int(line[3:7], 16)
        rec = int(line[7:9], 16)
        if rec == 2:
            # extended SEGMENT address: the field is a paragraph count (x16).
            # llvm-objcopy emits THIS record type, not type 04 - ignoring it
            # folds the high half of the image back down to 0x0000..0xFFFF and
            # makes the reported span nonsense.
            base = int(line[9:13], 16) << 4
        elif rec == 4:
            base = int(line[9:13], 16) << 16
        elif rec == 0:
            a = base + addr16
            lo = a if lo is None else min(lo, a)
            hi = max(hi or 0, a + cnt)
            n += cnt
    return lo, hi, n

for p in sys.argv[1:]:
    if os.path.exists(p):
        lo, hi, n = hex_span(p)
        print("%-26s flash 0x%05X..0x%05X  = %6d bytes  (%d%% of 636K app slot)"
              % (os.path.basename(p), lo, hi, n, 100 * n // (636 * 1024)))
    else:
        print("%-26s (missing)" % p)

print()
# The reference trees this tool compares against are not part of this repository.
# Set GZLL_REF to a directory holding them; the loop below already prints
# (missing) rather than failing when a path is absent.
A = os.environ.get("GZLL_REF", os.path.join(_REPO, "vendor", "gzll"))
pair = [("NEW 52840 lib (in place)", os.path.join(A, "gzll_nrf52_gcc.a")),
        ("OLD lib (backup)", os.path.join(A, "gzll_nrf52_gcc.a.bak-sdk11-gcc493-2016")),
        ("vendored 52840 archive", os.path.join(_REPO, "vendor", "gzll", "gzll_nrf52840_gcc.a"))]
print("== archive fingerprints ==")
for label, p in pair:
    if not os.path.exists(p):
        print("  %-24s (missing) %s" % (label, p))
        continue
    blob = open(p, "rb").read()
    gcc = re.search(rb"GCC: [^\x00]{0,70}", blob)
    members = re.findall(rb"nrf_[a-z_]+\.(?:c\.)?o/", blob)
    print("  %-24s %8d bytes  members=%d  %s"
          % (label, len(blob), len(set(members)),
             (gcc.group(0).decode("ascii", "replace") if gcc else "no GCC tag")))
