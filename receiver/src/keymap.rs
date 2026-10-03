//! Factory default keymap for the KeyPoint 2.4GHz receiver.
//!
//! Logical matrix is 12 rows x 8 columns, joined along ROWS:
//!   rows 0..5  = left half  (Gazell pipe 0, channels {15, 47, 71})
//!   rows 6..11 = right half (Gazell pipe 1, channels {31, 57, 81})
//! See `board.rs::logical_row` and `gazell.rs`.
//!
//! ---------------------------------------------------------------------------
//! THIS LAYOUT IS A COPY of the BLE dongle firmware's default
//! (`keypoint-rmk-dongle/src/keymap.rs`), transcribed from
//! `keypoint-zmk-dongle/config/keypoint.keymap`. It is not what the keyboard
//! normally runs on: the real layout is owned by Vial and lives in flash, and
//! this receiver shares its keyboard ID with the BLE firmware precisely so that
//! the saved layout carries over unchanged.
//!
//! So this file matters in exactly two situations: a freshly erased board, and
//! Vial's "reset to firmware default". Keep it a sane, typing-capable layout.
//!
//! ZMK constructs not expressible here, marked inline:
//!   * `&auto_shift` is written as its tap half only.
//!   * `&bt BT_CLR` / `&bt BT_CLR_ALL` are BLE-profile keys with no meaning in
//!     a 2.4GHz-only firmware: they are written as `Transparent`, i.e. those
//!     positions fall through to the layer below instead of pretending to be
//!     something they cannot be.
//!
//! `a!(No)` on unused cells is deliberate - those matrix positions have no
//! switch, and keeping them reserved documents the physical shape.
//! ---------------------------------------------------------------------------

use rmk::types::action::{EncoderAction, KeyAction};
use rmk::types::modifier::ModifierCombination;
use rmk::{a, encoder, k, lt, mo, shifted, tg, wm};

/// Dimensions come from `board.toml` (via `crate::board`), NOT from this file.
/// The array type below is built from them, so a keymap whose literal does not
/// match the matrix dimensions is a compile error rather than a runtime
/// surprise.
pub(crate) const COL: usize = crate::board::COL;
pub(crate) const ROW: usize = crate::board::ROW;
/// The layer count is a compile-time property of the keymap array below - it is
/// also the number RMK reports to Vial, which is what decides how many layers
/// the editor offers (`DynamicKeymapGetLayerCount` answers with the keymap's own
/// `NUM_LAYER`, rmk `src/host/via/mod.rs:195`).
///
/// Ten is the ceiling: the layer number in a Vial keycode is four bits wide
/// (`keycode_convert.rs:163`), and the Vial app does not offer more than ten
/// tabs either. So this is not "some layers added", it is "all the layers the
/// protocol can name".
///
/// The BLE dongle firmware still reports 5. That is harmless and deliberate:
/// storage is keyed per cell, the .vil's content lives in layers 0..4, and those
/// carry over unchanged. It does mean that a layout edited here up to layer 9 and
/// then loaded onto the BLE firmware has its writes above layer 4 refused by
/// rmk's bounds check - the first five layers still land.
pub(crate) const NUM_LAYER: usize = 10;

/// Rotary encoders this firmware knows about: one per half, id 0 = left, id 1 =
/// right, matching the ZMK `sensors = <&left_encoder &right_encoder>` order the
/// two transmitters were built from.
///
/// This number is also what RMK reports to Vial, and Vial is what decides how many
/// encoder slots the editor shows - so it has to match the two the hardware
/// actually has.
pub(crate) const NUM_ENCODER: usize = 2;

#[rustfmt::skip]
pub const fn get_default_keymap() -> [[[KeyAction; COL]; ROW]; NUM_LAYER] {
    [
        // ==================== Layer 0: QWERTY ====================
        [
            // Left rows 0..5
            [k!(Escape),   k!(Q), k!(W), k!(E), k!(R), k!(T), a!(No), a!(No)],
            [k!(Tab),      k!(A), k!(S), k!(D), k!(F), k!(G), a!(No), a!(No)],
            [k!(LShift),   k!(Z), k!(X), k!(C), k!(V), k!(B), a!(No), a!(No)],
            [k!(LCtrl),    k!(LGui), k!(LAlt), lt!(2, Space), lt!(1, Space), mo!(3), a!(No), a!(No)],
            // (4,6) and (4,7) are `No`, and the same goes for (10,6)/(10,7) on the
            // right. These four were the diagnostic build's stall-report cells
            // (F13-F16): the report was published by the receiver, and the cell
            // only had to exist so the position could be keyed at all. The cells
            // were chosen because they are spare and no switch could reach them.
            //
            // That last part stopped being true. The transmitter now puts its
            // pointer-power and pointer-mode switches on exactly these positions,
            // and a switch press is a matrix position like any other: the receiver
            // looks it up here, and a layout that still said F13 would send F13 to
            // the host on every toggle. The diagnostic itself is switched off
            // ([diagnostics] enabled = false) and always was harmless when idle -
            // it is these leftovers that were live.
            //
            // Both ends are covered: the transmitter also clears these cells out of
            // the bitmap it transmits, so a board whose stored layout still holds
            // the old values is equally quiet. This line is what a fresh board
            // starts with.
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            // pos 18 = &mkp LCLK, pos 45 = calculator. Mouse keys need no
            // feature: `is_mouse_key()` routes them into `mouse.process`.
            //
            // The first four cells are NOT switches -- (5,0) through (5,3) have no
            // physical mapping on this board, so the matrix can never read them
            // closed and they can never emit.
            //
            // They are the pad's four tier cells, read by `SpeedController` and set
            // through Vial: (5,0) is cursor speed, (5,1) scroll, (5,2) acceleration
            // gain, (5,3) acceleration cap. The nub's four are on the right half's
            // last row - see the note there - and the two halves' cells pair by role
            // rather than by position: the eight are listed as a set in
            // `keymap::TIER_CELLS` and nowhere else.
            //
            // A tier is named by an `F` key whose number is the tier - `F6` here,
            // which is 0.55x in the pad's speed ladder - and the cells start on the
            // rungs holding the speeds this keyboard was already running;
            // `pointer_speed::BOOT_TIERS` lists the same eight values and is the copy
            // that is consulted on every board.
            //
            // All four pad cells sit on `F6`: cursor 0.55x, gain 0.20, cap 1.5,
            // scroll divisor 44. This keyboard already scales each device once - the
            // pad runs at 0.55x of its raw counts and the nub at 0.29x (its 0.22x
            // rung times the 1.30 nub factor) - so for the same movement the pad
            // reports roughly twice the counts and a given gain rung bites earlier on
            // it. The rungs are starting values taken from that ratio rather than
            // from a measurement; Vial is where they get settled, and a change lands
            // on the next pointer event with no keypress.
            //
            // These values are only what a board with empty storage starts on. RMK
            // reads the keymap back from flash, so a keyboard Vial has ever saved
            // uses its own values, and this line is then historical. Clearing the
            // cell in Vial is not the same as blanking it here - that falls back to
            // `pointer_speed::BOOT_TIERS`, not to this.
            [k!(F6), k!(F6), k!(F6), k!(F6), a!(No), a!(No), k!(MouseBtn1), k!(Calculator)],
            // Right rows 6..11
            [k!(Y),        k!(U), k!(I), k!(O), k!(P), k!(Backspace), a!(No), a!(No)],
            [k!(H),        k!(J), k!(K), k!(L), k!(Semicolon), k!(Enter), a!(No), a!(No)],
            [k!(N),        k!(M), k!(Comma), k!(Dot), k!(Up), k!(Delete), a!(No), a!(No)],
            [k!(Slash),    k!(Space), mo!(2), k!(Left), k!(Down), k!(Right), a!(No), a!(No)],
            // (10,6)/(10,7): the right half's report cells (F15/F16), cleared for
            // the same reason as (4,6)/(4,7) on the left - see the note there.
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            // pos 19 = none, pos 52 = C_MUTE.
            //
            // Same storage role as the left half's storage row, four cells of its own
            // in the same four roles and the same order: (11,0) is the nub's cursor
            // speed, (11,1) the gain of its acceleration curve, (11,2) that curve's
            // cap and (11,3) its scroll speed. `F3` is 0.22x in the speed table and
            // scroll divisor 180 in the nub's own scroll ladder; the curve reads gain
            // 0.25 (`F11`) against cap 1.5 (`F6`).
            //
            // The row's history, because the cells have moved twice: (11,1) was the
            // scroll cell, then the curve took that slot when it moved here from the
            // transmitter and scroll moved down to (11,2); then the curve's two
            // numbers were split into a cell each, and the cap took the new cell
            // (11,3) that the Vial definition gained for it. A keyboard with an
            // older layout still stored in flash therefore keeps its speed and
            // scroll, and reads a curve tier out of both (11,1) and (11,2) - all
            // four are tier cells out of the box, so Vial is where to look.
            //
            // The ladders were respaced under these cells, so a value written before
            // the respace means something different now - and the cap cell used to
            // hold `F14`, which names no rung at all and fell back to rung 6. The
            // cell now says `F6` outright, which is the value it was already running.
            [k!(F3), k!(F11), k!(F6), k!(F3), a!(No), a!(No), a!(No), a!(No)],
        ],
        // ==================== Layer 1: RAISE ====================
        [
            [a!(Transparent), k!(Kc1), k!(Kc2), k!(Kc3), k!(Kc4), k!(Kc5), a!(No), a!(No)],
            [a!(Transparent), k!(Quote), shifted!(Quote), k!(Minus), k!(Equal), k!(Enter), a!(No), a!(No)],
            [a!(Transparent), k!(Home), k!(End), shifted!(Minus), shifted!(Equal), k!(Backslash), a!(No), a!(No)],
            [a!(Transparent), wm!(Home, ModifierCombination::LCTRL), wm!(End, ModifierCombination::LCTRL), k!(Calculator), tg!(1), tg!(1), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [k!(Kc7), k!(Kc8), k!(Kc9), k!(KpPlus), k!(KpAsterisk), a!(Transparent), a!(No), a!(No)],
            [k!(Kc4), k!(Kc5), k!(Kc6), k!(KpMinus), k!(KpSlash), a!(Transparent), a!(No), a!(No)],
            [k!(Kc1), k!(Kc2), k!(Kc3), k!(Comma), a!(Transparent), a!(Transparent), a!(No), a!(No)],
            [k!(Kc0), k!(Kc0), k!(KpDot), a!(Transparent), a!(Transparent), a!(Transparent), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
        ],
        // ==================== Layer 2: LOWER ====================
        [
            [shifted!(Grave), shifted!(Kc1), shifted!(Kc2), shifted!(Kc3), shifted!(Kc4), shifted!(Kc5), a!(No), a!(No)],
            [k!(Grave), shifted!(Kc6), shifted!(Kc7), shifted!(Kc8), shifted!(Kc9), shifted!(Kc0), a!(No), a!(No)],
            [a!(No), k!(Home), k!(End), wm!(PageUp, ModifierCombination::LCTRL), wm!(PageDown, ModifierCombination::LCTRL), wm!(F4, ModifierCombination::LALT), a!(No), a!(No)],
            [a!(No), wm!(Home, ModifierCombination::LCTRL), wm!(End, ModifierCombination::LCTRL), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [k!(NumLock), k!(CapsLock), k!(Pause), k!(PrintScreen), wm!(P, ModifierCombination::LCTRL), a!(No), a!(No), a!(No)],
            [a!(No), k!(LeftBracket), k!(RightBracket), shifted!(LeftBracket), shifted!(RightBracket), a!(No), a!(No), a!(No)],
            [a!(No), shifted!(Kc9), shifted!(Kc0), wm!(PageUp, ModifierCombination::LCTRL), k!(PageUp), wm!(PageDown, ModifierCombination::LCTRL), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), k!(Home), k!(PageDown), k!(End), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
        ],
        // ==================== Layer 3: FUNC ====================
        [
            // (0,4) = &msc SCRL_UP
            [k!(F1), k!(F2), k!(F3), wm!(PageUp, ModifierCombination::LCTRL), k!(MouseWheelUp), wm!(PageDown, ModifierCombination::LCTRL), a!(No), a!(No)],
            // (1,3..5) = &msc SCRL_LEFT / SCRL_DOWN / SCRL_RIGHT
            [k!(F4), k!(F5), k!(F6), k!(MouseWheelLeft), k!(MouseWheelDown), k!(MouseWheelRight), a!(No), a!(No)],
            [k!(F7), k!(F8), k!(F9), a!(No), k!(F2), k!(F5), a!(No), a!(No)],
            // (3,3)/(3,4) were BLE profile 0 / profile 1 on the BLE firmware, and
            // (5,7) was &bt BT_CLR. There is no BLE here, so all three are
            // Transparent: the positions are kept (so the Vial layout positions
            // stay aligned) without claiming a function that does not exist.
            [k!(F10), k!(F11), k!(F12), a!(Transparent), a!(Transparent), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(Transparent)],
            [k!(Kc7), k!(Kc8), k!(Kc9), k!(KpPlus), k!(KpAsterisk), a!(Transparent), a!(No), a!(No)],
            [k!(Kc4), k!(Kc5), k!(Kc6), k!(KpMinus), k!(KpSlash), a!(Transparent), a!(No), a!(No)],
            [k!(Kc1), k!(Kc2), k!(Kc3), k!(Comma), a!(Transparent), a!(Transparent), a!(No), a!(No)],
            [k!(Kc0), k!(Kc0), k!(KpDot), a!(Transparent), a!(Transparent), a!(Transparent), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
        ],
        // ==================== Layer 4: MOUSE (auto only) ====================
        // No MO/TG key points here: rmk's auto mouse layer enters on pointer
        // motion and leaves on the idle timeout.
        //
        // Written out in full instead of leaning on `Transparent`: as the
        // highest complete layer it shadows every lower layer while up, so
        // typing here is always base-plus-thumb-buttons (same as ZMK).
        //
        // Button layout is intentional, NOT copied from ZMK's MOUSE layer -
        // each hand carries both buttons. Do not "restore" ZMK assignments:
        //
        //   left button   (3,5) and (9,0)   MouseBtn1
        //   right button  (3,3) and (9,2)   MouseBtn2
        //   middle button (5,7)             MouseBtn3
        //   scroll key    (3,4) -> pad      scroll key (9,1) -> nub
        //
        // (3,4) and (9,1) carried no keymap action even on the BLE firmware:
        // `scroll_key.rs` watched those positions there. This firmware has no
        // pointing devices yet, so nothing watches them - the cells stay empty
        // so the two step-2 features have a place to land.
        [
            // Left hand, rows 0..5 - base letters, thumb cluster turned buttons.
            [k!(Escape), k!(Q), k!(W), k!(E), k!(R), k!(T), a!(No), a!(No)],
            [k!(Tab), k!(A), k!(S), k!(D), k!(F), k!(G), a!(No), a!(No)],
            [k!(LShift), k!(Z), k!(X), k!(C), k!(V), k!(B), a!(No), a!(No)],
            [k!(LCtrl), k!(LGui), k!(LAlt), k!(MouseBtn2), a!(No), k!(MouseBtn1), a!(No), a!(No)],
            [a!(No); COL],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), k!(MouseBtn1), k!(MouseBtn3)],
            // Right hand, rows 6..11
            [k!(Y), k!(U), k!(I), k!(O), k!(P), k!(Backspace), a!(No), a!(No)],
            [k!(H), k!(J), k!(K), k!(L), k!(Semicolon), k!(Enter), a!(No), a!(No)],
            [k!(N), k!(M), k!(Comma), k!(Dot), k!(Up), k!(Delete), a!(No), a!(No)],
            [k!(MouseBtn1), a!(No), k!(MouseBtn2), k!(Left), k!(Down), k!(Right), a!(No), a!(No)],
            [a!(No); COL],
            [a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No), a!(No)],
        ],
        // ==================== Layers 5..9: reserved, blank on purpose ====================
        //
        // `No`, not `Transparent`, and the two are not interchangeable here:
        //   * Vial draws an empty layer, which is what "blank" should look like when
        //     it is opened; `Transparent` would paint every one of these 96 cells as
        //     a see-through marker.
        //   * if such a layer is ever activated, `No` means the keys do nothing,
        //     while `Transparent` would fall through to layer 0 and type letters -
        //     a layer nobody configured would quietly become a second base layer.
        //
        // Blank layers are not free. rmk's own docs say so ("Empty layers still
        // consume flash and RAM"), and the numbers are recorded below rather than
        // estimated, because the cost is the argument against ever doing this again
        // without needing it.
        //
        // What is NOT disturbed, both verified in rmk's source rather than assumed:
        //   * the layout already saved in flash - storage keys each cell by
        //     `(layer, row, col)`, so layers 0..4 read back exactly as before and
        //     layers 5..9 simply have nothing stored;
        //   * the eight tier cells - `speed_control::refresh_cells` reads them with
        //     `action_at_pos(0, ..)`, layer 0 only, so blanks above cannot change
        //     pointer speeds or scroll rates.
        [ [a!(No); COL]; ROW ],
        [ [a!(No); COL]; ROW ],
        [ [a!(No); COL]; ROW ],
        [ [a!(No); COL]; ROW ],
        [ [a!(No); COL]; ROW ],
    ]
}

/// What each knob does, per layer: `[layer][encoder id]`, each entry a
/// `(clockwise, counter-clockwise)` pair - see RMK's `encoder!` macro.
///
/// Transcribed from the BLE dongle firmware's default
/// (`keypoint-rmk-dongle/src/keymap.rs`), which is what this receiver's saved Vial
/// layout was made against. ZMK's original keymap agrees about the right knob
/// (`C_VOLUME_DOWN` / `C_VOLUME_UP`) but lists the left knob's pair the other way
/// round (`PG_DN` first) - and that is the same direction question as
/// `encoder_reverse` in each transmitter's board.toml. The pair here and that flag
/// have to be right TOGETHER; either one alone can look wrong.
///
/// The layers ZMK left without `sensor-bindings` are `a!(No)` here: RMK looks this
/// table up by the active layer and does not fall through, so an unbound layer is
/// written out rather than left out.
/// Read a keymap cell value as a tier number.
///
/// Accepts `F1`..`F12` and maps them to tiers 1..12. Any other value returns `None`,
/// and the caller reads that as "no tier named here" and falls back to its own
/// default - see `pointer_speed::BOOT_TIERS`.
///
/// Twelve rungs, where this used to be twenty-four. The two extra tables are gone
/// rather than merely unused: until a keyboard is typed on, what a tier ladder is
/// worth is how finely the useful speeds can be told apart, and past about a tenth
/// of a step a hand stops reporting the change. Half the rungs were therefore
/// steps nobody could feel, and they cost a numbering scheme that had to be read
/// against two different scales to be believed.
///
/// The function keys because a tier is named by one key value, and `F1`..`F12` is a
/// contiguous run whose numbering is the tier numbering - the key called `F<n>` names
/// rung `n`, and each ladder's row comments say what that rung is worth (the same
/// number means different speeds on the pad and the nub - see `pointer_speed`).
/// Nothing needs to see `F13`..`F24` any more: a cell still holding one of those
/// names no tier and falls back, exactly as a cell holding `Kp1`..`Kp8` from the
/// eight-rung era does.
///
/// The keypad row's other virtue is kept by construction: these eight cells have no
/// switches behind them, so no tier value can ever be typed, wherever it is set.
pub fn tier_of(action: KeyAction) -> Option<u8> {
    [
        k!(F1),
        k!(F2),
        k!(F3),
        k!(F4),
        k!(F5),
        k!(F6),
        k!(F7),
        k!(F8),
        k!(F9),
        k!(F10),
        k!(F11),
        k!(F12),
    ]
    .iter()
    .position(|a| *a == action)
    .map(|i| i as u8 + 1)
}

/// The eight tier cells, in order: left move, left gain, left cap, left scroll,
/// right move, right gain, right cap, right scroll.
///
/// These coordinates have no switches behind them, so the matrix can never read
/// them closed and they can never be typed - which is exactly what makes them
/// usable as storage. They are set through Vial like any other cell, and the value
/// to write is an `F` key whose number is the rung: `F3` for the nub's 0.22x cursor,
/// `F6` for the pad's 0.55x, `F12` for the top of either ladder (0.85x on the pad,
/// 0.31x on the nub). See `tier_of`.
///
/// On the merged 12x8 matrix the left half is rows 0..5 and the right half rows
/// 6..11, so `(5,*)` is the left half's spare row and `(11,*)` the right half's.
///
/// The ORDER is a contract with `pointer_speed::BOOT_TIERS` and with the dispatch
/// in `speed_control::refresh_cells` - the three places are indexed by position, so
/// inserting a cell in the middle without touching the other two would silently move
/// every tier after it to the wrong device:
///
///   0  (5, 0)   pad cursor speed      -> `pointer_speed::PAD_SPEED_TIERS`
///   1  (5, 1)   pad acceleration gain -> `pointer_accel::ACCEL_GAIN_TIERS`
///   2  (5, 2)   pad acceleration cap  -> `pointer_accel::ACCEL_CAP_TIERS`
///   3  (5, 3)   pad scroll speed      -> `pointer_speed::PAD_SCROLL_TIERS`
///   4  (11, 0)  nub cursor speed      -> `pointer_speed::NUB_SPEED_TIERS`
///   5  (11, 1)  nub acceleration gain -> `pointer_accel::ACCEL_GAIN_TIERS`
///   6  (11, 2)  nub acceleration cap  -> `pointer_accel::ACCEL_CAP_TIERS`
///   7  (11, 3)  nub scroll speed      -> `pointer_speed::NUB_SCROLL_TIERS`
///
/// Both halves run the same order now - cursor speed, gain, cap, scroll speed -
/// and the sameness is the point: it is the order the four roles are easiest to keep
/// straight in. It replaced one where the nub's scroll cell sat between its two curve
/// cells and its cap sat last, and that order retired itself the way orders usually
/// do. `(11,3)` read as a scroll cell while actually holding the acceleration cap, so
/// setting a scroll speed there changed how fast the cursor moved - which is exactly
/// what it looked like from the outside, a fault.
///
/// The coordinates never moved, so the cost of the swap lands on a board Vial has
/// saved rather than on this file: those four cells keep their stored values and
/// change meaning, and each half's eight cells have to be entered again once. A cell
/// left holding an old value is not ignored - it is read as whatever role now owns
/// that coordinate.
pub const TIER_CELLS: [(u8, u8); 8] = [
    (5, 0),
    (5, 1),
    (5, 2),
    (5, 3),
    (11, 0),
    (11, 1),
    (11, 2),
    (11, 3),
];

pub const fn get_default_encoder_map() -> [[EncoderAction; NUM_ENCODER]; NUM_LAYER] {
    [
        // Layer 0 (QWERTY): page up/down on the left knob, volume on the right.
        [
            encoder!(k!(PageUp), k!(PageDown)),
            encoder!(k!(KbVolumeDown), k!(KbVolumeUp)),
        ],
        // Layer 1 (RAISE): unbound, as in ZMK.
        [
            encoder!(a!(No), a!(No)),
            encoder!(a!(No), a!(No)),
        ],
        // Layer 2 (LOWER): unbound, as in ZMK.
        [
            encoder!(a!(No), a!(No)),
            encoder!(a!(No), a!(No)),
        ],
        // Layer 3 (FUNC): unbound. ZMK bound BLE keys here, which have no meaning
        // in a 2.4GHz-only firmware - the same reason BT_CLR is written as
        // Transparent in the keymap above.
        [
            encoder!(a!(No), a!(No)),
            encoder!(a!(No), a!(No)),
        ],
        // Layer 4 (MOUSE): unbound. ZMK had page up/down and volume here, but on
        // the BLE firmware this layer is auto-activated by pointer motion, and
        // that is step 2's work.
        [
            encoder!(a!(No), a!(No)),
            encoder!(a!(No), a!(No)),
        ],
        // Layers 5..9: unbound, matching the blank keymap layers above.
        //
        // This table is looked up by the ACTIVE layer and does not fall through
        // (see the note at the top of this function), so a knob that is meant to
        // keep working while one of these layers is up must be bound here too -
        // blank here means the knob does nothing on that layer, not that it keeps
        // doing whatever layer 0 says. Vial can write these pairs later; the
        // encoder count this firmware reports (`NUM_ENCODER`) is unchanged, so the
        // two slots Vial shows for each of the ten layers are the same two knobs.
        [encoder!(a!(No), a!(No)); NUM_ENCODER],
        [encoder!(a!(No), a!(No)); NUM_ENCODER],
        [encoder!(a!(No), a!(No)); NUM_ENCODER],
        [encoder!(a!(No), a!(No)); NUM_ENCODER],
        [encoder!(a!(No), a!(No)); NUM_ENCODER],
    ]
}