//! Pointer speed tiers and inversion, receiver side.
//!
//! Everything that shapes a pointer movement lives here rather than on a half,
//! because all of it depends on things only the receiver has: the keymap (which is
//! where a tier is stored and where the user changes it) and the USB path out. The
//! halves send raw displacement and nothing else.
//!
//! `PointingProcessorEvent` carries a whole `PointingMode`, so changing a speed and
//! changing between cursor and scroll will turn out to be the same operation on one
//! channel - which is why the mode is built in one place (`cursor_mode`) instead of
//! being assembled wherever it happens to be needed.
//!
//! Two tables per axis, not one. A thumb pressing a nub covers far less ground than
//! a finger on the pad, and having to retune one to match the other was a trap rather
//! than a design - so the pad and the nub keep a table each even now that the two
//! tables hold the same rungs. Identical is the current answer and not a requirement:
//! a tier is named by a key whose number is the tier, and that only means anything
//! while the same number says the same thing on both halves.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use rmk::event::{publish_event, PointingProcessorEvent};
use rmk::input_device::pointing::{PointingMode, ScrollConfig, SniperConfig};

use crate::gazell::{TRACKPAD_ID, TRACKPOINT_ID};

/// Trackpad tiers, twenty-four of them: 0.1x through 2.4x, a tenth per rung.
/// Index is tier minus one, so the tier number IS the tenth - `F8` is 0.8x.
///
/// Why twenty-four and not eight: a tier is named by a single key value, and the row
/// that used to name them had exactly eight (`Kp1`..`Kp8`) - one per tier of an
/// eight-tier table, with nothing left to grow into. `F1`..`F24` is a contiguous run
/// of exactly the right length, so the ladder can be as long as the numbering.
///
/// Each rung is its tier number divided by ten, which is the whole point of the
/// change: the cell in Vial reads as the number it means. `F10` is 1.0x - unaltered
/// counts - the old table's top (2x) is `F20`, and the top of the ladder is `F24` at
/// 2.4x. The old eight values are not all present: 0.25x, 0.333x and 0.444x were
/// rungs of a hand-spaced ladder and have no tenth to land on. They are gone rather
/// than approximated, and the rungs either side of them are a hundredth away from
/// anything that was tuned by feel.
///
/// Sniper rather than Cursor, and that is not a preference: Cursor takes integer
/// multipliers only, so 1x is its floor and every tier below 1 would be impossible.
/// Sniper divides, and its accumulator carries the remainder into the next whole
/// unit, so a slow move does not lose steps to truncation. Every rung is tenths, so
/// the divisor never exceeds ten and the accumulation stays short.
pub const PAD_SPEED_TIERS: [(u8, u8); 12] = [
    (3, 10),   // F1  0.3x
    (7, 20),   // F2  0.35x
    (2, 5),    // F3  0.4x
    (9, 20),   // F4  0.45x
    (1, 2),    // F5  0.5x
    (11, 20),  // F6  0.55x
    (3, 5),    // F7  0.6x
    (13, 20),  // F8  0.65x
    (7, 10),   // F9  0.7x
    (3, 4),    // F10 0.75x
    (4, 5),    // F11 0.8x
    (17, 20),  // F12 0.85x
];


/// TrackPoint tiers: the same twenty-four rungs as the pad's list, but NOT the same
/// values - this table was halved on request.
///
/// The history matters, because the numbers here look like a break from the scheme
/// above. The two ladders once differed by design - a nub under a thumb covers less
/// ground than a finger on the pad, so its rungs sat below the pad's throughout
/// (0.1x..0.7x, while the pad ran 0.25x..2.0x). They were then unified so that `F<n>`
/// would mean one number wherever it was written, and the nub was left with the pad's
/// ladder: 0.05x through 1.20x. That turned out to be far too fast for the nub in
/// practice, and this table is the answer - every rung at half the value it holds in
/// `PAD_SPEED_TIERS`, fortieths instead of twentieths, so the ladder now runs 0.025x
/// through 0.30x.
///
/// The consequence to keep in mind: `F<n>` is NO LONGER the same number on both
/// pointers. `F8` is 0.8x on the pad and 0.125x on the nub, and a tier cell names a
/// rung rather than a speed, so the same `F8` written in the two halves' cells means
/// two different speeds. That is the price of keeping one numbering while the values
/// diverge; if a cell ever has to be read as an absolute number, read this table, not
/// the pad's.
///
/// The halving was a single multiply, so the ladder keeps the pad's shape: pairs of
/// equal rungs, and the same oddity at the top - `F23` (0.325x) is a shade faster than
/// `F24` (0.30x). That is inherited from the table this was derived from, not a typo
/// introduced here, and the pair is left alone because the top of a ladder is a range
/// someone selects from rather than one anyone is recommended to sit on.
///
/// A rung below 1 is a slower pointer and nothing else. `Sniper` divides, and its
/// accumulator carries the remainder into the next whole unit, so the slowest rung
/// here does not quantise a slow move away: it delays it, and the steps that arrive
/// are whole. The fortieths spacing keeps the divisor at forty or under, so the
/// accumulation stays short.
///
/// What this table does NOT hold is the acceleration cap. There used to be one table
/// doing both jobs and a rung meant "a speed and a ceiling", which is why the cap's
/// scale can be told apart from this one by its being ten times larger.
pub const NUB_SPEED_TIERS: [(u8, u8); 12] = [
    (1, 5),    // F1  0.2x
    (21, 100), // F2  0.21x
    (11, 50),  // F3  0.22x
    (23, 100), // F4  0.23x
    (6, 25),   // F5  0.24x
    (1, 4),    // F6  0.25x
    (13, 50),  // F7  0.26x
    (27, 100), // F8  0.27x
    (7, 25),   // F9  0.28x
    (29, 100), // F10 0.29x
    (3, 10),   // F11 0.3x
    (31, 100), // F12 0.31x
];


/// Fallback tiers, used whenever a tier cell does not name a tier - see
/// `speed_control::SpeedController`, which is what decides when that is.
///
/// These carry the speeds this keyboard was already running when the ladders grew
/// from eight rungs to twenty-four, so a board whose stored layout still names tiers
/// the old way (`Kp1`..`Kp8`, which now name nothing at all) keeps its feel instead
/// of jumping to whichever rung the numbering happens to land on:
///
///   pad cursor   `F8`   0.8x          the old tier 5, value unchanged
///   pad scroll   `F8`   divisor 24    the old tier 5, value unchanged
///   nub cursor   `F3`   0.3x          the old tier 5, value unchanged
///   nub scroll   `F3`   divisor 55    the old tier 5, value unchanged
///
/// The cursor and scroll fallbacks landing on the same rung per device is neither
/// coincidence nor copy-paste: each scroll ladder is calibrated so that rung `n`
/// scrolls at the speed rung `n` moves, which is what lets one cell name a speed for
/// both - see `PAD_SCROLL_TIERS`.
///
/// Note what these are NOT. They are not "the value on a blank board" - they apply
/// on any board, empty storage or not, every time a cell stops naming a tier. A
/// board with nothing stored starts on them because every cell is then unset, and a
/// board whose cell has been cleared or bound to an ordinary key falls back to them
/// for exactly the same reason. That is a stronger promise than the compile-time
/// keymap layout can make, because RMK reads the keymap back from flash and only
/// falls back to that layout when there is nothing stored at all - so a cell value
/// written in `keymap.rs` is gone for good once Vial has been saved over it, while
/// these constants are consulted forever.
pub const PAD_CURSOR0: u8 = 6;
pub const NUB_CURSOR0: u8 = 6;

/// Scroll divisors, one table per device, indexed by tier minus one.
///
/// `divisor` is rmk's own name for this number and its doc comment on `ScrollConfig`
/// reads "Higher = slower": the accumulated movement is divided by it before it
/// reaches the wheel. So these tables run the opposite way from the speed tables - a
/// higher tier is a SMALLER number, and a smaller number scrolls faster.
///
/// Both are twenty-four rungs now, like the speed ladders, and each is calibrated
/// against its own pointer: rung `n` is `round(BASE / n)`, where BASE is the number
/// that makes the rung the pointer sits on scroll at the speed that pointer moves.
/// The pad's BASE is 192 and the nub's is 165, which puts the pad's `F8` on divisor
/// 24 and the nub's `F3` on divisor 55 - the two values these cells have held
/// through every earlier retune. What that buys is the point of the whole
/// renumbering: one cell names ONE speed, and a rung means the same thing whether
/// the speed table reads it or this one does.
///
/// A divisor is a whole number and these rungs are not, so the upper rungs crowd
/// together - the last few are a step apart, some of them equal. That is the
/// arithmetic showing through rather than a mistake, and it happens where the
/// pointer is fastest and the eye is least able to tell two rungs apart.
///
/// Entry 1 is BASE itself and is the slowest rung by a wide margin - divisor 192 is a
/// wheel that barely turns. That is what a ladder numbered by value looks like once
/// its top is a number someone asked for; the useful range for a thumb on the pad
/// sits nearer divisor 24.
///
/// The rungs this replaces were hand-spaced, and both tables showed it: the pad's had
/// two equal values and the nub's listed a faster rung above a slower one. All of
/// that is gone with the respacing. The two numbers named above survive exactly, at
/// the rungs that keep this keyboard's own scroll speed - which is also why the
/// scroll fallbacks (`F8` and `F3`) sit on the same rungs as the cursor fallbacks.
const PAD_SCROLL_TIERS: [u8; 12] = [
    64,   // F1
    60,   // F2
    56,   // F3
    52,   // F4
    48,   // F5
    44,   // F6
    40,   // F7
    36,   // F8
    32,   // F9
    28,   // F10
    24,   // F11
    20,   // F12
];

const NUB_SCROLL_TIERS: [u8; 12] = [
    210,  // F1
    195,  // F2
    180,  // F3
    165,  // F4
    150,  // F5
    135,  // F6
    120,  // F7
    105,  // F8
    90,   // F9
    75,   // F10
    60,   // F11
    45,   // F12
];


/// Fallback scroll rungs, matching the cursor rungs above: `F8` on the pad and `F3`
/// on the nub - the same rung as that pointer's cursor fallback.
///
/// TIER numbers, not divisors. All six fallbacks share one representation because
/// `SpeedController` treats the six cells alike and only converts to a divisor at
/// the end. A divisor here would put a value outside 1..24 into a slot that is read
/// as a tier, and `set_scroll_tier` would reject it - the scroll fallback would then
/// silently never apply, leaving whatever tier was set before.
///
/// Same standing as the cursor fallbacks - read whenever a cell stops naming a tier,
/// not merely on a blank board.
pub const PAD_SCROLL0: u8 = 6;
pub const NUB_SCROLL0: u8 = 6;

/// Fallback rungs for the two acceleration curves: gain and cap per device, each a
/// rung in `pointer_accel`'s own tables rather than anything in this file.
///
/// Four constants because each curve has two knobs and each knob is set from its own
/// keymap cell - see `pointer_accel` for why they are separate, and for the values
/// themselves: these name constants there rather than repeating the numbers, because
/// a rung that has to be written down twice is a rung that will disagree with itself.
///
/// Both pairs are the ones this keyboard runs - the nub's (0.22 against 1.40) and the
/// pad's (0.10 against 1.40). See `pointer_accel` for why the gain ladder was respaced
/// to hold the first of them, and for what the two devices' count scales say about the
/// second. The pad's curve runs by default now: it shipped switched off for one round
/// and was turned on on request, which is exactly why these values live here rather
/// than being copied into a keymap cell.
pub const PAD_GAIN0: u8 = crate::pointer_accel::PAD_DEFAULT_GAIN_TIER;
pub const PAD_CAP0: u8 = crate::pointer_accel::PAD_DEFAULT_CAP_TIER;
pub const NUB_GAIN0: u8 = crate::pointer_accel::NUB_DEFAULT_GAIN_TIER;
pub const NUB_CAP0: u8 = crate::pointer_accel::NUB_DEFAULT_CAP_TIER;

/// The eight fallback tiers, in `keymap::TIER_CELLS` order: pad cursor, pad gain,
/// pad cap, pad scroll, nub cursor, nub gain, nub cap, nub scroll.
///
/// Read whenever a cell stops naming a tier, so this is the whole answer to "what
/// speed is a cell that shows no tier" - not merely the value a blank board starts
/// on. Keeping the count and the order identical to `TIER_CELLS` is what
/// `SpeedController` relies on when it indexes both by position.
///
/// The order is the keymap's, and it follows each half along its own storage row;
/// both halves read the same way now, cursor speed then the two curve knobs then
/// scroll speed. It was not always so: the nub's scroll tier used to sit between its
/// gain and its cap, which made the last cell - a curve knob - the one a reader would
/// take for the scroll speed. Do not sort it any further than it has been sorted.
pub const BOOT_TIERS: [u8; 8] = [
    PAD_CURSOR0,
    PAD_GAIN0,
    PAD_CAP0,
    PAD_SCROLL0,
    NUB_CURSOR0,
    NUB_GAIN0,
    NUB_CAP0,
    NUB_SCROLL0,
];

static PAD_TIER: AtomicU8 = AtomicU8::new(PAD_CURSOR0);
static NUB_TIER: AtomicU8 = AtomicU8::new(NUB_CURSOR0);
// The scroll slots hold divisors, so they start from the table rather than from the
// tier number. Retuning a tier then moves the starting point with it and there is
// nothing to keep in step by hand.
static PAD_SCROLL: AtomicU8 = AtomicU8::new(PAD_SCROLL_TIERS[PAD_SCROLL0 as usize - 1]);
static NUB_SCROLL: AtomicU8 = AtomicU8::new(NUB_SCROLL_TIERS[NUB_SCROLL0 as usize - 1]);
static PAD_SCROLLING: AtomicBool = AtomicBool::new(false);
static NUB_SCROLLING: AtomicBool = AtomicBool::new(false);

fn tier_slot(device_id: u8) -> &'static AtomicU8 {
    // Anything that is not the pad is the nub (`gazell::TRACKPOINT_ID`). Spelled as
    // a default rather than a second arm because there are exactly two pointers on
    // this keyboard, and a third id would be a bug rather than a third speed.
    if device_id == TRACKPAD_ID {
        &PAD_TIER
    } else {
        &NUB_TIER
    }
}

/// One device's current tier, 1..12.
pub fn cursor_tier(device_id: u8) -> u8 {
    tier_slot(device_id).load(Ordering::Relaxed)
}

/// Set one device's tier (1..12). `false` = out of range, current value untouched.
///
/// A bad tier is rejected rather than applied, because the tier comes from a keymap
/// cell the user edits at runtime and a typo there should cost one step, not the
/// device. The bound is the ladder length, and the ladder is twelve rungs: a
/// number the tables do not have is refused, not clamped to the top.
pub fn set_cursor_tier(device_id: u8, tier: u8) -> bool {
    if !(1..=12).contains(&tier) {
        return false;
    }
    tier_slot(device_id).store(tier, Ordering::Release);
    true
}

fn scroll_slot(device_id: u8) -> &'static AtomicU8 {
    if device_id == TRACKPAD_ID {
        &PAD_SCROLL
    } else {
        &NUB_SCROLL
    }
}

/// The half's own mode switch, per device. Latched: it says what the device means
/// until the switch is pressed again, and the half restates it in every packet.
static PAD_LATCHED: AtomicBool = AtomicBool::new(false);
static NUB_LATCHED: AtomicBool = AtomicBool::new(false);

/// A thumb key held on the mouse layer, per device. Momentary by nature - the other
/// of the two things that can ask for a wheel.
static PAD_HOLD: AtomicBool = AtomicBool::new(false);
static NUB_HOLD: AtomicBool = AtomicBool::new(false);

fn scrolling_slot(device_id: u8) -> &'static AtomicBool {
    if device_id == TRACKPAD_ID {
        &PAD_SCROLLING
    } else {
        &NUB_SCROLLING
    }
}

/// One device's current scroll divisor.
pub fn scroll_div(device_id: u8) -> u8 {
    scroll_slot(device_id).load(Ordering::Relaxed)
}

/// Whether this device is currently pointing a wheel instead of a cursor.
pub fn is_scrolling(device_id: u8) -> bool {
    scrolling_slot(device_id).load(Ordering::Relaxed)
}

/// Whether the pointing device's own switch has asked for a wheel. Stated by the half
/// in every packet.
fn latch_slot(device_id: u8) -> &'static AtomicBool {
    if device_id == TRACKPAD_ID {
        &PAD_LATCHED
    } else {
        &NUB_LATCHED
    }
}

/// Whether a thumb key is currently held for this device.
fn hold_slot(device_id: u8) -> &'static AtomicBool {
    if device_id == TRACKPAD_ID {
        &PAD_HOLD
    } else {
        &NUB_HOLD
    }
}

/// Republish this device's mode if, and only if, the answer changed.
///
/// The one place where the two things that can ask for a wheel are combined: the
/// switch on the half that owns the device (a latch, restated in every packet) and the
/// thumb key held on the receiver's mouse layer (a momentary request). Either one is
/// enough, and nothing else has to know which of them won.
///
/// Republishing only on a change is what lets a held key and a latched switch coexist:
/// letting the key go while the latch still asks for a wheel publishes nothing, so the
/// scroll simply continues. It is also what keeps the packet path cheap, since that
/// path calls in here once per packet.
fn apply_mode(device_id: u8) {
    let wanted =
        hold_slot(device_id).load(Ordering::Relaxed) || latch_slot(device_id).load(Ordering::Relaxed);
    if wanted == is_scrolling(device_id) {
        return;
    }
    if wanted {
        go_scroll(device_id);
    } else {
        go_cursor(device_id);
    }
}

/// The half's own switch says this device points a wheel (`true`) or a cursor
/// (`false`).
///
/// Only records it. The packet that carries this is parsed in the radio's interrupt,
/// and the answer to it is an EVENT - see `sync_modes`, which publishes from task
/// context. Publishing here would mean publishing from an interrupt, where RMK's
/// `publish_event` is non-blocking and may silently drop: a mode change that goes
/// missing once and is never repeated is exactly the failure that looks like "the
/// switch does nothing".
pub fn set_latched_mode(device_id: u8, scroll: bool) {
    latch_slot(device_id).store(scroll, Ordering::Relaxed);
}

/// The thumb key for this device went down (`true`) or came back up (`false`).
pub fn set_held_mode(device_id: u8, held: bool) {
    hold_slot(device_id).store(held, Ordering::Relaxed);
}

/// Make the processors agree with the two switches, for both devices.
///
/// Called from the event loop, in task context, once per pass. Cheap by construction:
/// two atomic loads per device and, unless something changed since the last pass, an
/// immediate return.
pub fn sync_modes() {
    apply_mode(TRACKPAD_ID);
    apply_mode(TRACKPOINT_ID);
}

/// Set one device's scroll tier (1..24). `false` = out of range, value untouched.
///
/// Stored as the divisor the device's table gives, not as the tier: divisors are the
/// form `ScrollConfig` wants, and keeping one representation means the two cannot
/// drift. Which table is read depends on the device, so the same tier number means
/// scrolls at the speed that device's rung `n` moves.
pub fn set_scroll_tier(device_id: u8, tier: u8) -> bool {
    if !(1..=12).contains(&tier) {
        return false;
    }
    let table = if device_id == TRACKPAD_ID {
        PAD_SCROLL_TIERS
    } else {
        NUB_SCROLL_TIERS
    };
    scroll_slot(device_id).store(table[(tier - 1) as usize], Ordering::Release);
    true
}

/// Scroll mode for one device, built from its stored divisor.
///
/// X is left alone. The pad's half already negates its X counts while reading them
/// (ZMK's `SCROLL_X_DIR = -1`), and that negation IS the scroll convention rather
/// than a hardware mirror - so the horizontal wheel comes out right and must not be
/// flipped a second time here. The TrackPoint's scroll maps X straight through, as
/// in ZMK.
///
/// Y is flipped for the pad, and only for the pad: the wheel ran the wrong way on
/// that touchpad (up scrolled down). It is a SCROLL-only flip because that was the
/// symptom - the same axis in cursor mode is correct, so flipping it in the A320
/// driver on the half would have fixed the wheel and broken the pointer, and the
/// current mode is known only here. Revert = the single `invert_y` expression below.
///
/// Why the pad needs it and the nub does not: the two are read by different drivers
/// (A320 vs the PS/2 bridge), so the same wheel axis is not the same sign for both.
/// Keep this per device rather than global.
pub fn scroll_mode(device_id: u8) -> PointingMode {
    PointingMode::Scroll(ScrollConfig {
        multiplier_x: 1,
        divisor_x: scroll_div(device_id),
        multiplier_y: 1,
        divisor_y: scroll_div(device_id),
        invert_x: false,
        invert_y: device_id == TRACKPAD_ID,
    })
}

/// One device's pointer mode, built from its tier.
///
/// The inversion is per device, not global, and it is hardware rather than taste:
/// the pad's own axes come out mirrored and the nub's do not, which is why only the
/// pad's path needs `invert_x`. The BLE firmware carried the same
/// `invert_x: device_id == PAD_ID`, and dropping it is exactly what made the pad
/// move left when the finger went right.
///
/// Note this is INDEPENDENT of the negation the pad's half applies while reading
/// its X counts (ZMK's `SCROLL_X_DIR = -1`, part of the scroll convention). The two
/// negations coexist and cancel for cursor movement; removing either one flips the
/// pad's X axis on hardware.
///
/// An unknown tier falls back to 1x passthrough rather than being rejected: the tier
/// is read at runtime, so a bad value costs one step instead of the device.
pub fn cursor_mode(device_id: u8) -> PointingMode {
    let tier = (cursor_tier(device_id) as usize).saturating_sub(1);
    let table = if device_id == TRACKPAD_ID {
        &PAD_SPEED_TIERS
    } else {
        &NUB_SPEED_TIERS
    };
    let (multiplier, divisor) = table.get(tier).copied().unwrap_or((1, 1));
    PointingMode::Sniper(SniperConfig {
        multiplier,
        divisor,
        invert_x: device_id == TRACKPAD_ID,
        invert_y: false,
    })
}

/// Record whether a device is scrolling.
///
/// Written before every republish so that a republish triggered while a scroll is
/// running keeps the mode instead of silently dropping back to a cursor.
pub fn set_scrolling(device_id: u8, on: bool) {
    scrolling_slot(device_id).store(on, Ordering::Relaxed);
}

/// Point one device back at the cursor.
///
/// Published rather than set directly because the processor is moved into `run_all`
/// at startup and cannot be reached afterwards. The BLE firmware did the same.
pub fn go_cursor(device_id: u8) {
    set_scrolling(device_id, false);
    publish_event(PointingProcessorEvent {
        device_id,
        mode: cursor_mode(device_id),
    });
}

/// Point one device at the wheel. The scroll key controller calls this while its
/// thumb key is held down.
pub fn go_scroll(device_id: u8) {
    set_scrolling(device_id, true);
    publish_event(PointingProcessorEvent {
        device_id,
        mode: scroll_mode(device_id),
    });
}