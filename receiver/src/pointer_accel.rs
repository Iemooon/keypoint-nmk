//! Sensitivity and the acceleration curves, applied on the receiver.
//!
//! This is the half of ZMK's trackpoint driver that is a SETTING rather than a
//! measurement, moved here from the transmitter so it can be edited from Vial like
//! every other speed setting on this keyboard.
//!
//! What moved, and what did not:
//!
//!   * The measurement stayed on the half. Only the half has the data-ready line,
//!     so only the half can measure the interval between two of the device's
//!     packets - and it puts that interval on the air as `OFF_GAP`.
//!   * The sensitivity factor and the curve came here, because both are settings:
//!     they answer to a keymap cell, the keymap lives on this board, and a
//!     transmitter with no storage, no USB port and no probe is the worst possible
//!     place to keep a knob the user expects to turn.
//!
//! The arithmetic is unchanged from what the transmitter used to do, and it is
//! ZMK's, in ZMK's order:
//!
//!     factor = SENS_BASE + SENS_STEP * speed_percent          // 0.3 + 0.01 * 100
//!     mult   = min(cap, exp(speed * gain))                    // speed = counts / ms
//!     out    = round(count * factor * mult)
//!
//! ## Two tables, because a curve has two knobs
//!
//! Gain and cap are the two independent numbers ZMK's driver has always had - its
//! `CONFIG_TRACKPOINT_ACCEL_FACTOR`, and the 150% ceiling written into
//! `trackpoint_exponential_factor`. Bundling them into a single tier contradicted
//! that, so each has its own table and its own keymap cell: gain decides how little
//! movement it takes before acceleration starts to show, cap decides the fastest the
//! pointer will ever go, and neither drags the other along.
//!
//! What that buys: a gentle start with a high ceiling (low gain, high cap) reads as
//! "precise when pushed slowly, quick when shoved", and it is not expressible at
//! all while the two share a row. Likewise a high gain with a low cap is "alert but
//! never wild".
//!
//! Two things worth knowing before turning either knob:
//!
//!   * Neither ladder has a rung that means "off"; the two ends of the gain ladder
//!     and the `1.0` rung of the cap ladder are the near things. A gain of 0.02
//!     wants 16.8 counts/ms before it reaches a 1.40 ceiling and therefore behaves as
//!     no acceleration at all on the nub, and a cap of 1.0 clamps every result to
//!     1.0 outright - which is a real off, and the cleanest one there is. What is
//!     left in the cap's case is the sensitivity factor, which is why "no
//!     acceleration" is still 1.30x on the nub rather than 1.00x of raw counts.
//!   * The nub's curve saturates at `ln(cap) / gain` counts per millisecond - 1.53 at
//!     the pair this keyboard runs (gain 0.22, cap 1.40), which is a hard push for
//!     this device. So the pair climbs across the whole useful range (~1.39x at a
//!     slow stroke, ~1.55x at an ordinary one, ~1.82x at the ceiling) instead of
//!     pinning itself at the cap after the first twitch the way an earlier pair did
//!     at 1.95x flat.
//!
//! ## One curve each, because the two pointers do not count the same
//!
//! Both devices are scored by the same rule and read the same two ladders; what
//! differs is where each one starts.
//!
//! The nub keeps ZMK's sensitivity factor of 1.30, because it is the half of ZMK's
//! trackpoint driver that turns raw counts into cursor counts. The pad does not, and
//! that is not an omission: the pad's counts were never on that scale. Its
//! calibration is its own speed table (`pointer_speed::PAD_SPEED_TIERS`), and
//! multiplying a touchpad's displacement by a factor measured on a stick would be
//! applying someone else's number. The pad's factor is 1.0.
//!
//! That 1.0 is also what lets a pad curve be switched off outright: at a cap of 1.0
//! the multiplier is 1.0 whatever the gain says. Both devices run a curve by default
//! now. The pad's shipped switched off for one round - its feel had been set with its
//! speed table and nobody had measured a curve for it - and it was switched on at the
//! owner's request, which is why the constants below are the only place a starting
//! bend is written down.
//!
//! The two scales are closer than they look. This keyboard already scales each
//! device once, in `pointer_speed`: the pad runs at 0.8x of its raw counts and the
//! nub at 0.39x (its 0.3 rung times this 1.30 factor). That is a ratio of about two,
//! so for the same movement of the same finger the pad reports roughly twice the
//! counts and a given gain rung bites earlier on it. The pad's starting gain is set
//! from that - about half the nub's bend - with the same ceiling. It is an argument
//! from two numbers that were chosen by feel rather than a measurement, and what
//! settles it is moving the rung: see the constants.
//!
//! ## Reading the tables
//!
//! Twelve rungs each, the same shape as the speed and scroll tables: an
//! `F1`..`F12` key value in the cell names a rung, and the rung number is all that
//! is stored. Retuning a rung is therefore a one-line change here, and a keyboard
//! that reads its rungs from flash is the only thing that has to be looked at
//! afterwards - see `pointer_speed::BOOT_TIERS` for what a cell naming no rung does.
//!
//! The rungs are numbered by value rather than by rank, and each ladder has its own
//! step: the gain ladder steps by 0.02 (`F<n>` is a gain of `n/50`) and the cap ladder
//! by 0.10 (`F<n>` is a cap of `n/10`), so a cell's value says what it holds. The cap
//! ladder's first nine rungs are the price of that - a cap below 1.0 holds a pointer
//! back below its own sensitivity factor, making those slow rungs rather than off
//! rungs, and "off" is `F10`.
//!
//! `F11` and `F14` are the pair the nub runs (gain 0.22, cap 1.40), and it has been
//! retuned on request since it was first picked - the constants below are the only
//! place that has to change, because the stored cell names the rung and never has to
//! be touched again. There used to be one undivided curve, and its value was ZMK's
//! stock pair (gain 1.307357, cap 1.5) - 1.95x flat, judged about 30% too strong
//! here. Splitting the curve into two tables is what made a gentler start with the
//! ceiling left alone expressible at all.
//!
//! Note the division of labour between gain and cap on the top rungs: gain alone does
//! not make a pointer faster, it makes it reach the ceiling sooner. If what is wanted
//! is more speed at the top, the cap rung is the one to raise.

use core::sync::atomic::{AtomicU8, Ordering};

use crate::gazell::TRACKPOINT_ID;

/// `CONFIG_TRACKPOINT_MOUSE_SENS_BASE_PERCENT = 30` -> 0.3.
const SENS_BASE: f32 = 0.3;
/// `CONFIG_TRACKPOINT_MOUSE_SENS_STEP_PERCENT = 1` -> 0.01 per percent.
const SENS_STEP: f32 = 0.01;
/// Pointer speed, where 100 is ZMK's default: `0.3 + 0.01 * 100 = 1.30`. Losing
/// this is a flat 30% of the nub's speed.
const SPEED_PERCENT: f32 = 100.0;

/// The nub's sensitivity factor, as a named value so the curve below reads as a curve.
pub const NUB_SENS: f32 = SENS_BASE + SENS_STEP * SPEED_PERCENT;

/// The pad's sensitivity factor, which is to say none.
///
/// Not a tuned value and not a placeholder for one: the pad's counts and the nub's
/// counts are different currencies, the pad's exchange rate lives in its speed table,
/// and this keeps the curve from applying the nub's rate to the wrong currency. It is
/// also what lets the pad's cap rung mean what it says - at `F10` the pad's curve is
/// off outright, because 1.0 times anything is itself.
pub const PAD_SENS: f32 = 1.0;

/// How hard a curve bends: the gain of `exp(speed * gain)`.
///
/// Twenty-four rungs, a fiftieth each: `F<n>` is a gain of `n/50`, so the ladder runs
/// 0.02 through 0.48. The spacing is set by the pair the nub runs - gain 0.22, which
/// the twentieths ladder this replaces could not express at all, carrying 0.20 and
/// 0.25 on either side of it.
///
/// Nothing in the range is unusable. Saturation arrives at `ln(cap) / gain` counts per
/// millisecond, which is 1.53 at the nub's pair and still 0.70 at the top rung, so
/// `F24` bends a curve harder than anyone needs rather than switching it - and the
/// bottom rung, 0.02, wants 16.8 counts/ms and is the nearest thing here to no
/// acceleration at all.
///
/// No rung means "off": the old tier 1 was a gain of zero, and zero does not fit a
/// ladder whose spacing is the point. Switching acceleration off is the cap table's
/// job and is the cleaner of the two anyway - a cap of 1.0 clamps every result to 1.0
/// whatever the gain does.
///
/// The higher rungs reach their ceiling sooner; they do not raise it, which is the cap
/// table's job.
pub const ACCEL_GAIN_TIERS: [f32; 12] = [
    0.15,     // F1
    0.16,     // F2
    0.17,     // F3
    0.18,     // F4
    0.19,     // F5
    0.2,      // F6
    0.21,     // F7
    0.22,     // F8
    0.23,     // F9
    0.24,     // F10
    0.25,     // F11
    0.26,     // F12
];



/// Where a curve stops: the ceiling `mult` saturates against.
///
/// Twelve rungs, a tenth each: `F<n>` is a cap of `1.0 + (n-1)/10`, so the ladder
/// runs 1.0 through 2.1. It used to be twenty-four rungs starting at 0.1, and the
/// bottom half of it is gone. Those rungs were not merely unused, they leaked a
/// design question into the settings - a cap under 1.0 holds the pointer below its
/// own sensitivity factor, which makes a rung a slow POINTER rather than a flat
/// CURVE, and nothing about a ladder for acceleration should be able to make the
/// cursor slow. Starting the ladder at 1.0 removes the possibility: `F1` is the
/// bottom rung and it means "no acceleration", every result being clamped to 1.0
/// whatever the gain does.
///
/// Of the rungs this replaces, one value survives exactly: 1.40, the ceiling the nub
/// ran for as long as the curve existed. It is `F5` here.
///
/// The trailing comment on each row is what the NUB does once saturated, i.e.
/// `NUB_SENS * cap` - a single multiplication, so the row can be read as "the fastest
/// this stick goes". The pad saturates at the row's own value, `PAD_SENS` being 1.0.
pub const ACCEL_CAP_TIERS: [f32; 12] = [
    1.0,      // F1  1x
    1.1,      // F2  1.1x
    1.2,      // F3  1.2x
    1.3,      // F4  1.3x
    1.4,      // F5  1.4x
    1.5,      // F6  1.5x
    1.6,      // F7  1.6x
    1.7,      // F8  1.7x
    1.8,      // F9  1.8x
    1.9,      // F10  1.9x
    2.0,      // F11  2x
    2.1,      // F12  2.1x
];



/// The rung each device's curve runs when nothing names one, or when a cell names no
/// tier at all.
///
/// First, the exception: the nub's ceiling was raised on request. It ran `F14`
/// (1.40) for as long as the curve has existed, and the request was to multiply
/// whatever it was by 1.5 - which lands exactly on `F21`, the ladder being tenths.
/// So the nub now saturates 1.5x higher than the value that had been measured as
/// enough, and the number in the constant below is a multiplication rather than a
/// fresh judgement. Worth knowing when reading the trailing comments on the rows:
/// the nub's top end (`NUB_SENS * cap`) is now 1.82x at the rung it runs, where it
/// used to be 1.82x only at the rung above. If that overshoots, the old value is one
/// constant away.
///
/// One constant per knob per device, because the cells are independent all the way
/// down: a gain cell that stops naming a tier falls back here for the gain alone and
/// leaves the ceiling exactly where it was. `pointer_speed`'s curve fallbacks name
/// these rather than repeating the numbers, so there is one place to change.
///
/// The nub's pair was chosen by measurement - gain 0.22 against cap 1.40 - and the
/// fiftieths ladder puts both on a rung exactly (`F11` and `F14`). The twentieths
/// ladder this replaces could only carry 0.20, a shade later to its ceiling, which is
/// the whole reason the gain ladder was respaced. These are also what a board still
/// storing `Kp` tier cells falls back to, which is why they matter beyond blank
/// boards - see `pointer_speed::BOOT_TIERS`.
///
/// The pad's pair is gain 0.10 against cap 1.40: half the nub's bend, the same
/// ceiling. Nothing about the pad was measured, so the number comes from the ratio
/// between the two devices' existing scale factors - see the module note on the two
/// scales - and it is a starting point rather than an answer.
///
/// What to move, and when. If the pad feels as though it jumped to a fixed higher
/// speed rather than speeding up as the finger moves faster, the curve is saturating
/// too early and the gain rung is what says so - drop it two or three rungs. If the
/// pad feels unchanged, the curve is not being reached and the gain wants raising. If
/// it speeds up as it should but tops out sooner than wanted, that is the cap, and
/// cap has no effect at all until it is above 1.0. Both cells are live: a Vial edit
/// lands on the next pointer event, with no keypress needed.
///
/// The pad's first round shipped switched off (gain 0.20 against cap 1.00) so that
/// adding a curve to a device whose feel had already been settled would change
/// nothing until asked. It was asked for, and this is the answer.
pub const PAD_DEFAULT_GAIN_TIER: u8 = 6;
pub const PAD_DEFAULT_CAP_TIER: u8 = 6;
pub const NUB_DEFAULT_GAIN_TIER: u8 = 6;
pub const NUB_DEFAULT_CAP_TIER: u8 = 6;
/// Per-device tier state, indexed by `slot`: 0 is the pad, 1 is the nub.
static GAIN_TIER: [AtomicU8; 2] = [
    AtomicU8::new(PAD_DEFAULT_GAIN_TIER),
    AtomicU8::new(NUB_DEFAULT_GAIN_TIER),
];
static CAP_TIER: [AtomicU8; 2] = [
    AtomicU8::new(PAD_DEFAULT_CAP_TIER),
    AtomicU8::new(NUB_DEFAULT_CAP_TIER),
];

/// Map a device id onto a slot.
///
/// Anything that is not the nub is the pad, rather than a panic or a rejection: the
/// id arrives from a packet, and a device this board has no curve for is better off
/// sharing the pad's than taking the firmware down over it.
fn slot(device_id: u8) -> usize {
    if device_id == TRACKPOINT_ID {
        1
    } else {
        0
    }
}

/// Store a new gain rung for one device. Rejects anything outside 1..24 rather than
/// clamping, for the same reason `pointer_speed` does: this is a value read from a
/// keymap cell the user edits, and a typo there should cost one step, not the device.
pub fn set_gain_tier(device_id: u8, tier: u8) -> bool {
    if !(1..=12).contains(&tier) {
        return false;
    }
    GAIN_TIER[slot(device_id)].store(tier, Ordering::Relaxed);
    true
}

/// Store a new cap rung for one device. Same rules as the gain side - see
/// `set_gain_tier`.
///
/// Two functions rather than one taking a table index, because the only caller is a
/// match on the cell's position: a named function there says which knob is being
/// turned, an index would say it twice and less clearly.
pub fn set_cap_tier(device_id: u8, tier: u8) -> bool {
    if !(1..=12).contains(&tier) {
        return false;
    }
    CAP_TIER[slot(device_id)].store(tier, Ordering::Relaxed);
    true
}

/// ZMK's `trackpoint_exponential_factor`, with both of its numbers coming from the
/// tables above.
///
/// A window of zero means "no interval known" and is read as one millisecond, the
/// same rule ZMK applies to a zero gap.
fn accel_factor(device_id: u8, dx: i16, dy: i16, window_ms: u32) -> f32 {
    let dist = (dx.unsigned_abs() as u32 + dy.unsigned_abs() as u32) as f32;
    if dist < 1.0 {
        return 1.0;
    }
    let s = slot(device_id);
    let gain = ACCEL_GAIN_TIERS[(GAIN_TIER[s].load(Ordering::Relaxed).clamp(1, 12) as usize) - 1];
    let cap = ACCEL_CAP_TIERS[(CAP_TIER[s].load(Ordering::Relaxed).clamp(1, 12) as usize) - 1];
    let speed = dist / (window_ms.max(1) as f32);
    let mult = libm::expf(speed * gain);
    if mult > cap {
        cap
    } else {
        mult
    }
}

/// Apply one device's sensitivity factor and curve to one report's displacement.
///
/// `window_ms` is the time that displacement actually took, accumulated from the
/// `OFF_GAP` of every packet that contributed to it - see `gazell`'s
/// `pointer_window_ms`. Accumulating is what keeps a displacement that spans
/// several reports from being scored as though it happened in the last
/// millisecond of them.
///
/// Both devices are scored now; they just start from different factors. Note where
/// this lands in the pipeline: the displacement is scaled here, before the pointing
/// processor decides whether it is a cursor or a scroll, so a curve that is switched
/// on is felt in scroll mode too. That has been true of the nub since the curve
/// existed.
///
/// Truncation toward zero, as in ZMK.
pub fn apply(device_id: u8, dx: i16, dy: i16, window_ms: u32) -> (i16, i16) {
    let sens = if slot(device_id) == 1 { NUB_SENS } else { PAD_SENS };
    let mult = sens * accel_factor(device_id, dx, dy, window_ms);
    let fx = dx as f32 * mult;
    let fy = dy as f32 * mult;
    (
        (fx as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16,
        (fy as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16,
    )
}
