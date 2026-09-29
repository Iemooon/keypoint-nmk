//! The board description, as compile-time constants.
//!
//! Everything here comes from `board.toml`, turned into Rust by `build.rs`
//! (`board_generated.rs`). This file adds only the *derived* values and the
//! invariants the rest of the firmware relies on - so a wrong number in
//! `board.toml` fails the build here rather than misbehaving on hardware.
//!
//! Nothing else in the firmware hardcodes geometry: `keymap.rs` sizes its arrays
//! from `ROW`/`COL` (so a keymap that does not match the matrix is a compile
//! error), and `gazell.rs` unpacks bits using the same `COLS_PER_HALF`.

include!(concat!(env!("OUT_DIR"), "/board_generated.rs"));

// ---------------------------------------------------------------------------
// Derived
// ---------------------------------------------------------------------------

/// Bytes in one half's report that hold the MATRIX: the cells as a single bitmap,
/// row 0 first.
///
/// `gazell.rs` unpacks it back into one byte per row - the shape `RAW` and RMK
/// want. With 8 columns the bitmap *is* the "one byte per row" layout the
/// transmitter used to send (8 columns fill exactly one byte), so widening the
/// format did not change KeyPoint's packets - and a keyboard with a bigger matrix
/// is a board.toml edit, not a protocol change. The transmitter's `board.rs`
/// carries the mirror image of this, and `tools/verify_matrix_bitmap.py` checks
/// that the two agree.
pub const MATRIX_BYTES: usize = (ROWS_PER_HALF * COLS_PER_HALF).div_ceil(8);

// ---------------------------------------------------------------------------
// The rest of the report
//
// One half's report is its matrix, then everything else that half knows. This
// layout is PROTOCOL, not board configuration, so it is written down here rather
// than in board.toml - but the transmitter's board.rs must carry the same
// offsets. The link is one-way, so a disagreement is not a degraded link, it is a
// silently misread one; what keeps the two honest is that `drain_pipe` checks the
// received length against PAYLOAD_LENGTH and drops anything that does not match.
//
// Every field is a CUMULATIVE value, never a delta, and that is the whole design:
// there is no retransmission and no back channel, so a delta lost in the air is
// lost for good, while a cumulative value repairs itself on the next packet.
//
//   offset             size  field
//   0                  MB    matrix bitmap (MB = MATRIX_BYTES)
//   OFF_ENCODER        1     encoder cumulative count, low 8 bits
//   OFF_POINTER_BUTTONS 1    pointing-device buttons
//   OFF_POINTER_X      2     pointing device X, cumulative u16 LE
//   OFF_POINTER_Y      2     pointing device Y, cumulative u16 LE
//   OFF_POINTER_MODE   1     what that device means: 0 cursor, 1 wheel
//   OFF_GAP            1     how long that displacement took, in ms
//   OFF_SEQ            1     reset marker
//
// A half fills in only what it has: the left half carries the trackpad, the right
// half the TrackPoint, and the unused fields stay zero.
//
// A half DOES state what its pointing device means, and that reverses what this
// comment used to say. It still reports raw displacement and nothing else - the
// scale, the direction and the mouse layer all live here - but whether that
// displacement is a cursor or a wheel is switched by a key that sits ON THE HALF,
// and the link runs one way, so the half is the only end that can state it.
// ---------------------------------------------------------------------------

/// Encoder cumulative count, low 8 bits. Wraps at 256; the receiver differences
/// consecutive packets, so what arrives is a *count*, not a movement event.
pub const OFF_ENCODER: usize = MATRIX_BYTES;
/// Pointing-device buttons, one bit per button.
pub const OFF_POINTER_BUTTONS: usize = OFF_ENCODER + 1;
/// Pointing device X, cumulative i16 little-endian (wrapping).
pub const OFF_POINTER_X: usize = OFF_POINTER_BUTTONS + 1;
/// Pointing device Y, cumulative i16 little-endian (wrapping).
pub const OFF_POINTER_Y: usize = OFF_POINTER_X + 2;
/// What this half's pointing device means: 0 = cursor, 1 = wheel.
///
/// The one field here that is NOT cumulative, and the reason is that it is not a
/// measurement: it is a setting, owned by the half (its mode switch) and restated in
/// every packet. Nothing needs to be differenced, and nothing needs to be repaired -
/// the next packet simply repeats it, which is also what makes the two ends agree
/// again after either of them restarts.
pub const OFF_POINTER_MODE: usize = OFF_POINTER_Y + 2;
/// How long the half's displacement took, in milliseconds, saturating at 255.
///
/// The one field here that is a MEASUREMENT and still not cumulative, and the two
/// are not in conflict: a duration cannot be summed the way a position can, because
/// summing it would make every lost packet look like a pause. It is the interval
/// between two of the DEVICE's packets, which is what the acceleration curve is a
/// function of - see `pointer_accel` for why the curve is computed here even though
/// the half is the only end that could measure the interval.
///
/// Zero means "no interval established", which the curve reads as one millisecond.
pub const OFF_GAP: usize = OFF_POINTER_MODE + 1;
/// The reset marker, in the byte that used to carry nothing at all.
///
/// A half that restarts begins every cumulative field at zero, and this one byte is
/// how it says so: the first `BOOT_MARKER_PACKETS` sends after a reset carry a
/// non-zero value here, and the receiver adopts the half's counters as a baseline
/// while it sees one instead of differencing them against the last session.
///
/// Without it the first packet of a new session reads as the knob having been turned
/// back by however far it went in the previous one. Loss measurement, if it is ever
/// wanted, needs a byte of its own - this one is spent on the marker.
pub const OFF_SEQ: usize = OFF_GAP + 1;

/// Bytes on the wire for one half: the matrix bitmap, then the fields above.
///
/// 6x8 gives 6 + 9 = 15 bytes, against Gazell's 32-byte limit. A half with a
/// bigger matrix grows by the bitmap alone; the nine trailing bytes are fixed.
pub const PAYLOAD_LENGTH: u32 = (OFF_SEQ + 1) as u32;

/// The halves are joined along ROWS, not columns.
///
/// KeyPoint halves are 6x8, and the global keymap (the one Vial's definition
/// describes, and the one the BLE dongle reports) is 12x8 with **rows 0..5 =
/// left half, rows 6..11 = right half**. RMK's split driver calls that second
/// half's `row_offset`, and `keypoint-rmk` uses `row_offset: 6` for it - so this
/// is the same shape, reached a different way.
///
/// The alternative (nmk's flat 4x12 keyboards) joins along columns because their
/// two halves are halves of one row. Deriving it wrongly here would not fail to
/// compile: it would silently swap which physical key each keymap position means,
/// which is why the two lines below carry this comment.
pub const ROW: usize = ROWS_PER_HALF * HALVES;
pub const COL: usize = COLS_PER_HALF;

/// Logical row for row `row` of the half in slot `slot` (0 = left, 1 = right).
pub const fn logical_row(slot: usize, row: usize) -> usize {
    slot * ROWS_PER_HALF + row
}

// ---------------------------------------------------------------------------
// Derived link timing
//
// Gazell's own defaults for the two values below are computed from the
// library's DEFAULT configuration, not from board.toml:
//
//   NRF_GZLL_DEFAULT_SYNC_LIFETIME
//       = 3 * NRF_GZLL_DEFAULT_CHANNEL_TABLE_SIZE(5)
//           * NRF_GZLL_DEFAULT_TIMESLOTS_PER_CHANNEL(2)
//       = 30 timeslots
//   NRF_GZLL_DEFAULT_TIMESLOTS_PER_CHANNEL_WHEN_DEVICE_OUT_OF_SYNC = 15
//
// This board uses a 6-channel table and (like the working receiver) never calls
// nrf_gzll_set_timeslots_per_channel(), so the host dwells 2 timeslots per
// channel and a full rotation is 6 * 2 = 12 timeslots = 10.8 ms at 900 us.
//
// 12 < 15 is the point: a half that has lost sync dwells 15 timeslots on a
// channel before moving on, so it ALWAYS meets the host before it gives up. Set
// the host's timeslots-per-channel to 4 and the rotation becomes 24 timeslots -
// twice the half's dwell - and a half coming back from sleep usually abandons a
// channel before the host arrives. That shows up as "the first character is
// late", and, when the attempt is lost, as "the character is missing".
//
// The two settings this affects - sync lifetime (default 30 timeslots) and
// timeslots-per-channel-when-out-of-sync (default 15) - are therefore never set
// here at all, which is also why they are not in board.toml.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Invariants
//
// These duplicate the checks build.rs already makes, deliberately: build.rs
// catches bad input early with a good message, and these make sure the *derived*
// values used here stay sane even if someone edits the generated constants or
// the derivation above.
// ---------------------------------------------------------------------------

/// One half's report is a single bitmap, so the link bounds the *cell count* and
/// not either dimension - see `MATRIX_BYTES` and the note in build.rs.
const _: () = assert!(ROWS_PER_HALF >= 1, "rows_per_half must be at least 1");
const _: () = assert!(COLS_PER_HALF >= 1, "cols_per_half must be at least 1");
const _: () = assert!(
    MATRIX_BYTES <= 32,
    "one half's matrix bitmap must fit in Gazell's 32-byte payload"
);
/// The WHOLE report has to fit, not just the matrix - and this is the bound that
/// bites first, because the trailing fields are a fixed nine bytes on top.
const _: () = assert!(
    PAYLOAD_LENGTH <= 32,
    "one half's report must fit in Gazell's 32-byte payload"
);
const _: () = assert!(
    HALVES == 2,
    "the Gazell link is two pipes: pipe 0 = left, pipe 1 = right"
);
/// RMK carries row/col as u8 in `KeyboardEvent`.
const _: () = assert!(ROW <= 255 && COL <= 255, "matrix dimensions must fit in u8");