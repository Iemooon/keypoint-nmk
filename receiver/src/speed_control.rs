//! Tier state for the pointing devices, read from eight keymap cells.
//!
//! The tiers live in base layer cells whose key value is the tier number: `(5,0)`
//! through `(5,3)` are the trackpad's cursor speed, acceleration gain, acceleration
//! cap and scroll speed, and `(11,0)`..`(11,3)` are the same four in the same order
//! for the TrackPoint. Those eight coordinates have no switches behind them, so they
//! are read only - and being read only is exactly what makes them usable as storage.
//!
//! Four of the eight name a tier in `pointer_speed`'s tables, one cursor tier and one
//! scroll tier per device; the other four name a tier in `pointer_accel`'s two, a gain
//! and a cap per device, because a curve has two knobs. Everything else about the
//! eight is identical, which is why they are read in one loop and dispatched by
//! position.
//!
//! The cells are re-read on every keyboard event and every pointing event, and
//! written to the tier tables only when a value actually changed. A cell that names
//! no tier falls back to the fallback tiers in `pointer_speed`, so clearing a cell
//! in Vial means "the default speed" rather than "leave it as it was".
//!
//! Where the values come from matters here. RMK reads the whole keymap back from
//! flash at startup and only falls back to the compile-time layout when there is
//! nothing stored, so on a keyboard that has ever been edited with Vial these are
//! the user's tiers rather than `keymap.rs`'s. That is why this receiver shares the
//! BLE firmware's keyboard id: the layout, tiers included, carries over. The
//! fallbacks above sit outside that mechanism, which is what lets them promise
//! something the compile-time layout cannot.

use rmk::event::{KeyboardEvent, PointingEvent};
use rmk::keymap::KeyMap;
use rmk::macros::processor;

use crate::gazell::{TRACKPAD_ID, TRACKPOINT_ID};
use crate::pointer_accel::{set_cap_tier, set_gain_tier};
use crate::pointer_speed::{
    BOOT_TIERS, go_cursor, go_scroll, is_scrolling, set_cursor_tier, set_scroll_tier,
};

/// Reads the eight tier cells and copies them into the tier tables.
///
/// Subscribes to pointing events as well as keyboard ones, and that second
/// subscription is the whole reason a Vial edit needs no keypress to take effect.
/// RMK has no "configuration changed" event to listen for, and a Vial edit produces
/// no keyboard event - it writes the keymap and nothing else. The old behaviour was
/// therefore "change the cell, then press any key": correct but baffling, because
/// nothing about editing a speed setting suggests you must then type.
///
/// Pointing events close that gap for free. They arrive continuously whenever a
/// pointer moves, which is exactly what someone checking their new speed setting
/// does, and they are the only stream that is present without the user doing
/// anything deliberate. The cost is eight table lookups per event, and those are
/// skipped outright once the values compare equal.
#[processor(subscribe = [KeyboardEvent, PointingEvent])]
pub struct SpeedController<'a> {
    keymap: &'a KeyMap<'a>,
    /// The tier currently applied to each cell's table, in `TIER_CELLS` order.
    /// Comparison only, and seeded with the fallbacks so the first read of an
    /// untouched board publishes nothing.
    cells: [u8; 8],
}

impl<'a> SpeedController<'a> {
    pub fn new(keymap: &'a KeyMap<'a>) -> Self {
        Self {
            keymap,
            cells: BOOT_TIERS,
        }
    }

    /// Read the eight cells and store the tiers.
    ///
    /// `action_at_pos` is a table lookup rather than a key-state read, which is what
    /// makes it work for cells that have no switch behind them.
    ///
    /// A cell that does not name a tier falls back to `BOOT_TIERS` rather than
    /// leaving the current value in place. The difference only shows when a cell
    /// stops naming a tier - cleared in Vial, or bound to an ordinary key - and then
    /// the two rules say different things: keep whatever speed was last set, or
    /// return to the documented default. Falling back is the one that can be
    /// predicted from what is on screen in Vial, and it means a cell showing no tier
    /// always means the same speed rather than whichever tier happened to be applied
    /// before the edit.
    ///
    /// A move tier needs a republish to take effect. While a device is scrolling the
    /// scroll mode is deliberately left in place instead - the new tier applies when
    /// the key is released and `go_cursor` runs anyway, so republishing here would
    /// only interrupt a scroll in progress.
    fn refresh_cells(&mut self) {
        for (idx, (row, col)) in crate::keymap::TIER_CELLS.iter().enumerate() {
            let tier = crate::keymap::tier_of(self.keymap.action_at_pos(0, *row, *col))
                .unwrap_or(BOOT_TIERS[idx]);
            if self.cells[idx] == tier {
                continue;
            }
            self.cells[idx] = tier;
            // Dispatched by position, in `TIER_CELLS` order. The table is in the
            // keymap's doc comment as well; the two must be read together.
            match idx {
                0 => {
                    set_cursor_tier(TRACKPAD_ID, tier);
                    if !is_scrolling(TRACKPAD_ID) {
                        go_cursor(TRACKPAD_ID);
                    }
                }
                1 => {
                    set_gain_tier(TRACKPAD_ID, tier);
                }
                2 => {
                    set_cap_tier(TRACKPAD_ID, tier);
                }
                // The two curves answer to no event of their own: the next pointer
                // movement will be scored against whatever tier is in force, which
                // is also why changing one looks instant. Gain and cap are separate
                // cells and separate atomics, per device, so one of them failing to
                // name a tier does not disturb another - not the other knob and not
                // the other pointer.
                3 => {
                    set_scroll_tier(TRACKPAD_ID, tier);
                    if is_scrolling(TRACKPAD_ID) {
                        go_scroll(TRACKPAD_ID);
                    }
                }
                4 => {
                    set_cursor_tier(TRACKPOINT_ID, tier);
                    if !is_scrolling(TRACKPOINT_ID) {
                        go_cursor(TRACKPOINT_ID);
                    }
                }
                5 => {
                    set_gain_tier(TRACKPOINT_ID, tier);
                }
                6 => {
                    set_cap_tier(TRACKPOINT_ID, tier);
                }
                7 => {
                    set_scroll_tier(TRACKPOINT_ID, tier);
                    if is_scrolling(TRACKPOINT_ID) {
                        go_scroll(TRACKPOINT_ID);
                    }
                }
                // Not reachable: `TIER_CELLS` has exactly eight entries and the arms
                // above cover all eight. It is spelled as a no-op rather than
                // `unreachable!()` so that adding a ninth cell is first a
                // compile-time question (does the new cell belong in a new arm?)
                // rather than a panic on the user's desk.
                _ => {}
            }
        }
    }

    async fn on_keyboard_event(&mut self, _event: KeyboardEvent) {
        self.refresh_cells();
    }

    /// Same work, for the same reason - see the type's doc comment.
    async fn on_pointing_event(&mut self, _event: PointingEvent) {
        self.refresh_cells();
    }
}
