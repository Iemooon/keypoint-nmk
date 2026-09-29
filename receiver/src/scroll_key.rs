//! Scroll key controller: while a thumb key is held, that half points a wheel
//! instead of a cursor - the left key drives the trackpad, the right key the
//! TrackPoint. Each half keeps its own key, which works because rmk gives every
//! pointer its own `PointingProcessor` keyed by `device_id`.
//!
//! This key is now only ONE of two things that can ask for a wheel. The other is the
//! mode switch on the half itself, which latches until it is pressed again and states
//! its answer in every packet. `pointer_speed` combines the two, so nothing here has
//! to know about the other one - this module only says "the key is down" and "the key
//! is up".
//!
//! The scroll scale comes from rmk's `ScrollConfig` divisor, so no scaling is
//! applied in the driver.
//!
//! Arming checks the mouse layer; releasing does not. rmk counts only Rel X/Y as
//! pointer motion, so wheel events cannot keep layer 4 alive - and checking on every
//! event would drop the mode part way through a scroll. So it is checked at press,
//! and a scroll that started on the mouse layer runs until the key is released.

use rmk::event::{KeyboardEvent, KeyboardEventPos};
use rmk::keymap::KeyMap;
use rmk::macros::processor;

use crate::gazell::{TRACKPAD_ID, TRACKPOINT_ID};
use crate::pointer_speed::set_held_mode;

/// The auto mouse layer in keymap.rs - the only layer where the scroll keys arm.
pub const MOUSE_LAYER: u8 = 4;

/// Left-hand scroll key: ZMK position 47, left thumb row 3 col 4. Drives the pad.
///
/// Note this is the cell ZMK's MOUSE layer uses for LCLK; the left button is still
/// reachable at `(3,5)` and `(9,0)`, so nothing is lost.
pub const PAD_SCROLL_KEY: (u8, u8) = (3, 4);

/// Right-hand scroll key: ZMK position 50, right thumb row 3 col 1. Drives the nub.
///
/// On the merged 12x8 matrix the right half occupies rows 6..11, so ZMK position 50
/// lands at `(9, 1)` - position numbering is NOT `row * 8 + col`.
pub const NUB_SCROLL_KEY: (u8, u8) = (9, 1);

#[processor(subscribe = [KeyboardEvent])]
pub struct ScrollKeyController<'a> {
    keymap: &'a KeyMap<'a>,
}

impl<'a> ScrollKeyController<'a> {
    /// Needs the keymap solely to ask `active_layer()` when a scroll key goes down.
    pub fn new(keymap: &'a KeyMap<'a>) -> Self {
        Self { keymap }
    }

    async fn on_keyboard_event(&mut self, event: KeyboardEvent) {
        let KeyboardEventPos::Key(pos) = event.pos else {
            // Rotary encoder events are the other variant; not our business.
            return;
        };
        let device_id = match (pos.row, pos.col) {
            r if r == PAD_SCROLL_KEY => TRACKPAD_ID,
            r if r == NUB_SCROLL_KEY => TRACKPOINT_ID,
            _ => return,
        };

        if event.pressed {
            if self.keymap.active_layer() == MOUSE_LAYER {
                set_held_mode(device_id, true);
            }
        } else {
            // Unconditionally, and that is the change the half's mode switch forced:
            // the mode is the combination of this hold and that latch, so a release
            // here may well leave a latched scroll running. Clearing a hold that was
            // never set changes nothing, so there is no case to test for.
            set_held_mode(device_id, false);
        }
    }
}