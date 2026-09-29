"""Check that the transmitter and the receiver agree on the report layout.

WHY THIS EXISTS
---------------
One half's report is a byte string that the transmitter writes and the receiver
reads, and each side defines its own offsets (`OFF_*` in its own board.rs) because
the two crates share no code. That is fine only as long as the two definitions
match:

  * a mismatched LENGTH is loud - `drain_pipe` rejects the packet and every key
    stops working, so it shows up immediately;
  * a mismatched OFFSET with the same total length is SILENT - for instance
    swapping two fields of equal size would leave the length correct and every
    reading wrong.

So this script reconstructs both sides' constants from source, evaluates them
against each board.toml's own geometry, and compares. Run it after touching the
`OFF_*` chain on either side:

    python verify_report_layout.py
"""

import pathlib
import re
import tomllib

ROOT = pathlib.Path(__file__).resolve().parent.parent


def read_geometry(path, rows_key, cols_key):
    with open(path, "rb") as f:
        t = tomllib.load(f)
    return int(t["matrix"][rows_key]), int(t["matrix"][cols_key])


def parse_consts(path):
    """Collect `pub const NAME: type = expr;` from a board.rs."""
    text = pathlib.Path(path).read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r"^pub const (\w+)\s*(?::\s*[^=]+?)?\s*=\s*(.+?);", text, re.M):
        out[m.group(1)] = m.group(2).strip()
    return out


def evaluate(name, consts, matrix_bytes, seen=()):
    """Evaluate a constant expression using only consts, ints and div_ceil."""
    if name in seen:
        raise ValueError(f"cycle in constants: {seen} -> {name}")
    if name == "MATRIX_BYTES":
        return matrix_bytes

    expr = consts[name]
    expr = re.sub(r"\s+as\s+\w+$", "", expr).strip()          # drop `as u32`
    expr = expr.replace(".div_ceil(8)", "")                    # handled by MATRIX_BYTES
    expr = re.sub(r"\((\d+)\s*\*\s*(\d+)\)",                  # (rows * cols)
                  lambda m: str(int(m.group(1)) * int(m.group(2))), expr)
    expr = expr.replace("/", "//")

    def sub(m):
        token = m.group(0)
        if token.isdigit():
            return token
        return str(evaluate(token, consts, matrix_bytes, seen + (name,)))

    value = eval(re.sub(r"[A-Z_][A-Z0-9_]*", sub, expr), {"__builtins__": {}}, {})
    return int(value)


# ---------------------------------------------------------------------------

tx_geom = read_geometry(ROOT / "keyboard" / "board.toml", "rows", "cols")
rx_geom = read_geometry(ROOT / "receiver" / "board.toml", "rows_per_half", "cols_per_half")

print(f"geometry: tx {tx_geom[0]}x{tx_geom[1]}, rx {rx_geom[0]}x{rx_geom[1]}"
      f"{'' if tx_geom == rx_geom else '   <-- MISMATCH'}")

sides = {}
for name, path, geom in [
    ("tx", ROOT / "keyboard" / "src" / "board.rs", tx_geom),
    ("rx", ROOT / "receiver" / "src" / "board.rs", rx_geom),
]:
    consts = parse_consts(path)
    mb = -(-(geom[0] * geom[1]) // 8)
    resolved = {}
    for c in ("MATRIX_BYTES", "OFF_ENCODER", "OFF_POINTER_BUTTONS", "OFF_POINTER_X",
              "OFF_POINTER_Y", "OFF_SEQ", "PAYLOAD_LENGTH"):
        if c not in consts:
            print(f"{name}: !! {c} is missing")
            continue
        resolved[c] = evaluate(c, consts, mb)
    sides[name] = resolved

keys = sorted(set(sides["tx"]) | set(sides["rx"]),
              key=lambda k: sides["tx"].get(k, sides["rx"].get(k, -1)))

print("\nfield                 tx   rx")
bad = []
for k in keys:
    a, b = sides["tx"].get(k), sides["rx"].get(k)
    mark = "" if a == b else "   <-- MISMATCH"
    if a != b:
        bad.append(k)
    print(f"  {k:<20} {str(a):>4} {str(b):>4}{mark}")

# field-by-field size check, straight off the offsets
print("\nderived widths (from the two sides' own offsets):")
for side in ("tx", "rx"):
    s = sides[side]
    widths = {
        "matrix": s["OFF_ENCODER"],
        "encoder": s["OFF_POINTER_BUTTONS"] - s["OFF_ENCODER"],
        "buttons": s["OFF_POINTER_X"] - s["OFF_POINTER_BUTTONS"],
        "x": s["OFF_POINTER_Y"] - s["OFF_POINTER_X"],
        "y": s["OFF_SEQ"] - s["OFF_POINTER_Y"],
        "seq": s["PAYLOAD_LENGTH"] - s["OFF_SEQ"],
    }
    print(f"  {side}: " + ", ".join(f"{k}={v}" for k, v in widths.items()))

total = sides["tx"]["PAYLOAD_LENGTH"]
print(f"\ntotal report = {total} bytes  "
      f"({'ok, Gazell allows 32' if total <= 32 else 'OVER Gazell 32-byte limit'})")

print("\nRESULT:", "PASS" if not bad and tx_geom == rx_geom and total <= 32 else "FAIL")