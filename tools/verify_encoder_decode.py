"""Verify the knob's decoder before it reaches hardware.

WHY THIS EXISTS
---------------
The knob is decoded in the transmitter (`keyboard/src/board.rs`, `Encoder::poll`) and the
result is handed to RMK on the receiver, which decodes the *same* thing again from
the count difference. Two decoders, two chances to get it wrong, and the failure
mode is the least pleasant kind: not "nothing happens" but "clockwise moves the
volume the wrong way", which you only see by turning the knob on real hardware.

So this script checks three things off hardware:

  1. the transition table in keyboard/src/board.rs is BYTE-FOR-BYTE the one in RMK's
     `ResolutionPhase` (read straight out of the RMK source, not retyped), so the
     two ends cannot disagree about what a transition means;
  2. the decode behaves like a knob: one detent is 4 transitions, a turn of N
     detents moves the count by N, reversing gives -N, and contact bounce between
     two adjacent states cancels out instead of accumulating;
  3. the count wraps at 256, which is what the receiver's difference relies on.

Run:  python verify_encoder_decode.py
"""

import math
import pathlib
import re

# Where does the rmk this project builds against actually live? Since 2026-09-29 the
# dependency is the official repository at a pinned rev, so the source sits in cargo's
# git checkout - a path no human types and no constant should hard-code. Two earlier
# versions of this line each named a personal clone directory, and the check died on
# FileNotFoundError rather than comparing anything. cargo metadata answers instead.
def _rmk_encoder_source() -> pathlib.Path:
    import json
    import subprocess
    repo = pathlib.Path(__file__).resolve().parents[1]
    r = subprocess.run(
        ["cargo", "metadata", "--format-version", "1", "--offline",
         "--manifest-path", str(repo / "receiver" / "Cargo.toml")],
        capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise SystemExit("cargo metadata failed; cannot locate rmk to compare against:\n" + r.stderr)
    for pkg in json.loads(r.stdout)["packages"]:
        if pkg["name"] == "rmk":
            return pathlib.Path(pkg["manifest_path"]).parent / "src" / "input_device" / "rotary_encoder.rs"
    raise SystemExit("no rmk package in cargo metadata")

RMK_ENCODER = _rmk_encoder_source()
if not RMK_ENCODER.is_file():
    raise SystemExit(
        "cannot compare against RMK: %s is missing.\n"
        "Find the real tree with `cargo metadata` in receiver\\ - this check is\n"
        "worthless if it silently skips the comparison." % RMK_ENCODER
    )

# --- keyboard/src/board.rs --------------------------------------------------------

# ENCODER_LUT: index = prev_a | prev_b << 1 | cur_a << 2 | cur_b << 3
LUT = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]


def rust_rem(a, b):
    """Rust's `%` truncates towards zero; Python's floors. Same result here, but
    the difference would hide a bug, so the reinterpretation is explicit."""
    return int(math.fmod(a, b))


class Encoder:
    """Encoder::poll in keyboard/src/board.rs, line for line."""

    def __init__(self, resolution=4, reverse=False):
        self.state = 0
        self.pulses = 0
        self.count = 0
        self.resolution = resolution
        self.reverse = reverse

    def poll(self, a_low, b_low):
        s = self.state & 0b11
        if a_low:
            s |= 0b0100
        if b_low:
            s |= 0b1000
        self.state = s >> 2

        if (s & 0xC) == (s & 0x3):
            return False

        step = LUT[s & 0xF]
        self.pulses += -step if self.reverse else step

        if self.pulses >= self.resolution:
            self.pulses = rust_rem(self.pulses, self.resolution)
            self.count = (self.count - 1) & 0xFF
            return True
        if self.pulses <= -self.resolution:
            self.pulses = rust_rem(self.pulses, self.resolution)
            self.count = (self.count + 1) & 0xFF
            return True
        return False


# =============================================================================
# 1. the table matches RMK's, read from the RMK source
# =============================================================================

print("RMK source:", RMK_ENCODER)
rmk_text = RMK_ENCODER.read_text(encoding="utf-8")

m = re.search(r"let mut lut = \[([^\]]*)\];", rmk_text)
if not m:
    print("  !! could not find RMK's lookup table - has it been renamed?")
    rmk_lut = None
else:
    rmk_lut = [int(x.strip()) for x in m.group(1).split(",")]
    print(f"  RMK LUT      : {rmk_lut}")
    print(f"  our LUT      : {LUT}")
    print(f"  identical    : {rmk_lut == LUT}")

# RMK's threshold direction, read from source rather than assumed: a positive
# accumulation is CounterClockwise there.
pos_ccw = "self.current_pulses >= self.resolution as i8" in rmk_text
neg_cw = "self.current_pulses <= -(self.resolution as i8)" in rmk_text
print(f"  RMK: +accumulation -> CounterClockwise : {pos_ccw}")
print(f"  RMK: -accumulation -> Clockwise        : {neg_cw}")
print("  ours: +accumulation -> count -= 1 (same direction), -accumulation -> += 1")

assert rmk_lut == LUT, "the two ends' transition tables disagree"

# =============================================================================
# 2. one detent is four transitions, and a turn of N detents moves N
# =============================================================================

# The four quadrature TRANSITIONS one detent is made of, as (a_low, b_low). The
# knob is assumed to start at (0,0) - both contacts open - and one detent is the
# full cycle back to it.
#
# Four transitions, not four states: the (0,0) the cycle ends on is where the next
# detent begins, so counting states rather than transitions is exactly how this
# script first came out one detent short.
FORWARD = [(1, 0), (1, 1), (0, 1), (0, 0)]
BACKWARD = [(0, 1), (1, 1), (1, 0), (0, 0)]


def turn(enc, detents, forward=True):
    """Turn the knob `detents` detents, one quadrature transition at a time.
    Returns, per transition, whether that transition completed a detent."""
    cycle = FORWARD if forward else BACKWARD
    moves = []
    for _ in range(abs(detents)):
        for state in cycle:
            moves.append(enc.poll(*state))
    return moves


print("\none detent = four transitions:")
enc = Encoder()
moves = turn(enc, 1)
print(f"  4 transitions -> count moved {sum(moves)} time(s), count = {enc.count}")

enc = Encoder()
turn(enc, 20)
print(f"  20 detents (= one full turn, ZMK's steps = <20>) -> count = {enc.count}")
print(f"  expected -20 modulo 256 = {(-20) & 0xFF}")

enc = Encoder()
turn(enc, 1, forward=False)
print(f"  one detent the other way -> count = {enc.count} (expected 1)")

# =============================================================================
# 3. bounce cancels instead of accumulating
# =============================================================================

print("\ncontact bounce (the failure this table's zeros are for):")
enc = Encoder()
bounced = 0
for _ in range(50):
    # rattle between two adjacent states, which is what a dirty contact does
    if enc.poll(1, 0):
        bounced += 1
    if enc.poll(0, 0):
        bounced += 1
print(f"  50 bounces between two adjacent states -> count = {enc.count} "
      f"(detents counted: {bounced})")

# both contacts changing at once is not a rotation and must not count
enc = Encoder()
simultaneous = 0
for _ in range(50):
    if enc.poll(1, 1):
        simultaneous += 1
    if enc.poll(0, 0):
        simultaneous += 1
print(f"  50x (both contacts together -> both released) -> count = {enc.count} "
      f"(detents counted: {simultaneous})")

# =============================================================================
# 4. the count wraps at 256, which the receiver's difference depends on
# =============================================================================

print("\nwrap at 256:")
enc = Encoder()
turn(enc, 20)
start = enc.count
detents = 0
for _ in range(260):                      # 260 more detents, past the wrap
    detents += sum(turn(enc, 1))
print(f"  from {start}, {detents} detents later -> count = {enc.count}")
print(f"  expected ({start} - {detents}) mod 256 = {(start - detents) & 0xFF}")

# =============================================================================
# 5. reverse is exactly a mirror image
# =============================================================================

print("\nreverse = true:")
a = Encoder()
turn(a, 7)
b = Encoder(reverse=True)
turn(b, 7, forward=False)
print(f"  7 detents forward (normal) vs 7 detents backward (reversed): "
      f"{a.count} vs {b.count} - {'same' if a.count == b.count else 'DIFFERENT'}")

# =============================================================================
# 6. end to end: what the transmitter counts is what the receiver calls it
# =============================================================================
#
# The two ends are separate code in separate crates, and the mapping between them
# is a sign convention that has to hold on BOTH sides or the knob works backwards.
# Nothing in either file enforces it, so it is checked here:
#
#   tx     : pulses >= +resolution  -> count -= 1
#            (and RMK's `ResolutionPhase` calls that accumulation CounterClockwise)
#   rx     : count - last < 0       -> CounterClockwise
#   keymap : encoder!(clockwise, counter_clockwise)
#
# so a DECREASING count has to come out as the SECOND entry of that pair.


def rx_moved(count, last):
    """The receiver's difference: `count.wrapping_sub(last) as i8`."""
    return ((count - last + 128) % 256) - 128


print("\nend to end (transmitter count -> receiver direction -> keymap slot):")
for forward, name in ((True, "FORWARD quadrature cycle "), (False, "BACKWARD quadrature cycle")):
    enc = Encoder()
    turn(enc, 1, forward=forward)
    moved = rx_moved(enc.count, 0)
    slot = "clockwise (first)" if moved > 0 else "counter_clockwise (second)"
    print(f"  {name}: count 0 -> {enc.count:>3}, receiver reads {moved:+d} "
          f"-> {'Clockwise' if moved > 0 else 'CounterClockwise'} -> {slot}")

enc = Encoder()
turn(enc, 1)
decreased_is_ccw = rx_moved(enc.count, 0) < 0
print(f"  convention check - a DECREASED count reads as CounterClockwise: "
      f"{decreased_is_ccw}  (RMK: `pulses >= resolution` is CounterClockwise)")

# A multi-detent turn must arrive as that many events, because the receiver turns
# each into one keypress.
enc = Encoder()
turn(enc, 3)
moved = rx_moved(enc.count, 0)
print(f"  a 3-detent flick arrives as moved = {moved:+d}, i.e. {abs(moved)} events "
      f"({abs(moved)} volume steps, not 1)")

print("\nRESULT:", "PASS" if rmk_lut == LUT and decreased_is_ccw else "FAIL")