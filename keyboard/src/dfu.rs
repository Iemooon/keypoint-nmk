//! The way into a half's bootloader without opening the keyboard.
//!
//! Four cells pressed together, named by `dfu_combo` in board.toml. The point of
//! the gesture is that a half of this keyboard hides behind a case and a battery
//! connector, so "reflash it" otherwise means taking the thing apart; the reset
//! button on the board is inside that case too.
//!
//! The order of what follows is the whole design, and each step is answering a
//! measured property of the link rather than a guess:
//!
//!   1. the four cells are recognised from `now - now`, i.e. from the raw scan
//!      bitmap and its press timestamps - NOT from the report pipeline. Nothing is
//!      withheld and nothing is re-timed: the four keycodes go to the host like any
//!      other four keys, which is what keeps this from being a key that behaves
//!      differently depending on what layer it is on.
//!   2. the half then waits for `ARM_ACKS` packets to be acknowledged. A press is
//!      only sent after `DEBOUNCE_TICKS` stable scans, and the first acknowledgement
//!      after the gesture may still belong to whatever was sent before it, so one
//!      ack is not evidence and two are.
//!   3. `grace_ms` on top of that, so the host has actually been told. The receiver
//!      debounces a change for a few milliseconds of its own and the host's report
//!      follows; this covers both. It is also what stops the gesture from leaving a
//!      held key behind on a layer the user is no longer touching.
//!   4. `release_packets` reports with nothing down, at the ordinary repeat cadence,
//!      and only then the reset. The receiver keeps a half's keys until its 500 ms
//!      link timeout - without this step the four corner cells would stay down for
//!      half a second after the half had gone, and the first round of them would
//!      arrive while the host still had them held.
//!   5. `GPREGRET = 0x57`, then a system reset. The Adafruit bootloader this board
//!      carries reads that register before it decides what to be, so the half comes
//!      back as a UF2 drive.
//!
//! The half is gone from step 4 onwards, which is why the acknowledgement in step 2
//! is the last thing that can be waited for: there is no later moment at which a
//! mistake could still be corrected.
//!
//! The one thing this deliberately does NOT do is gate on a USB cable being
//! present before the gesture is *recognised*. That check happens when the gesture
//! lands (see `usb_power_present`), and refusing to act on a recognised gesture is
//! visible in its own right: nothing happens.

use defmt::info;
use embassy_time::{Duration, Instant};

use crate::board::{self, MATRIX_BYTES, Role};

/// What the main loop should do with this pass, as far as the gesture is concerned.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Nothing unusual: report the matrix as always.
    Normal,
    /// Report "nothing is down" instead of the matrix.
    Release,
    /// Report nothing at all this pass. The four cells are physically down, but the
    /// half is leaving and what the host should be left holding is the release that
    /// was just delivered, not a fresh press.
    Silent,
    /// Everything is said. The loop must call `jump`, which does not return.
    Jump,
}

/// Nothing pressed, as a matrix bitmap.
pub const NOTHING_DOWN: [u8; MATRIX_BYTES] = [0u8; MATRIX_BYTES];

/// How many acknowledged packets make the gesture's report believed delivered.
///
/// Two rather than one, because the first packet acknowledged after the gesture
/// may be one that was already in flight: a matrix change goes out only after
/// `DEBOUNCE_TICKS` stable scans, so the half is still re-sending the previous state
/// when the press is first seen.
const ARM_ACKS: u32 = 2;

/// How long to wait for those acknowledgements before giving up on the gesture.
///
/// Generous - ten times what the cadence needs - because a half whose link has just
/// been re-acquiring is exactly the half someone is trying to flash. Bounded at all
/// because the alternative is worse than a wasted gesture: an armed half that never
/// fires is one that fires *later*, in the middle of ordinary typing, the moment the
/// link comes back.
const ARM_TIMEOUT_MS: u32 = 500;

/// The interval between two "nothing is down" reports: the ordinary repeat cadence,
/// derived rather than guessed so a change to the scan rate or the debounce window
/// keeps the two in step.
///
/// One report is 1 ms of the receiver's time (each pass is `SCAN_PERIOD_US`), and
/// the receiver commits a change it has seen sustained for its own few ms - so the
/// cadence here is not what makes the release land, the *number* of reports is.
/// Sending them at the cadence the receiver has been hearing all along keeps this
/// step indistinguishable from an ordinary release.
const RELEASE_INTERVAL_MS: u64 = (board::DEBOUNCE_TICKS as u64 * board::SCAN_PERIOD_US) / 1000;

/// The gesture, as the loop sees it.
pub struct Combo {
    /// The four cells, in this half's own rows - see `board::Role::dfu_combo`.
    cells: &'static [(usize, usize); 4],
    /// When each cell was first seen down in the current gesture, or `None` while it
    /// is up.
    pressed_at: [Option<Instant>; 4],
    phase: Phase,
}

enum Phase {
    /// Watching for the gesture.
    Idle,
    /// The gesture landed; waiting for the receiver to acknowledge the report that
    /// carries it.
    Armed { acks: u32, since: Instant },
    /// Acknowledged. Reporting normally for a little longer, so the four keycodes
    /// reach the host.
    Grace { until: Instant },
    /// Saying "nothing is down", one report per `RELEASE_INTERVAL_MS`.
    Releasing { sent: u32, next: Instant },
    /// The gesture was seen but will not be acted on - no cable, or no link - and
    /// the half is waiting for the cells to be released before it will listen again.
    Abandoned,
    /// All of it is done; the next pass resets.
    Done,
}

impl Combo {
    pub fn new(role: Role) -> Self {
        Combo {
            cells: role.dfu_combo(),
            pressed_at: [None; 4],
            phase: Phase::Idle,
        }
    }

    /// One pass. `keys` is the matrix just scanned, `acks` the transmitter's
    /// lifetime count of acknowledged packets (`gazell::progressed().0` - a count,
    /// not a flag, because "how many since the gesture" is the question).
    pub fn step(&mut self, keys: &[u8; MATRIX_BYTES], acks: u32, now: Instant) -> Step {
        if !board::DFU_ENABLED {
            return Step::Normal;
        }
        match self.phase {
            Phase::Idle => {
                self.track(keys, now);
                if self.gesture_landed() {
                    if board::DFU_REQUIRE_USB && !usb_power_present() {
                        // Said out loud rather than passed over in silence: the
                        // gesture is otherwise indistinguishable from a mis-press,
                        // and this is the one link in the chain with no other way
                        // of being observed.
                        info!(
                            "dfu: gesture ignored - no USB cable (POWER->USBREGSTATUS.VBUSEDETECT \
                             is clear); release the four cells and try again with one plugged in"
                        );
                        self.phase = Phase::Abandoned;
                    } else {
                        info!("dfu: gesture recognised on this half - waiting for the report to be \
                               acknowledged before resetting");
                        self.phase = Phase::Armed { acks, since: now };
                    }
                }
                Step::Normal
            }
            Phase::Armed { acks: base, since } => {
                // Nothing cancels this, which is deliberate: the keys are held, the
                // report has been handed to the radio, and a half that second-guessed
                // itself here would be one that sometimes flashes and sometimes does
                // not. The only way out is the timeout below.
                if acks.wrapping_sub(base) >= ARM_ACKS {
                    self.phase = Phase::Grace {
                        until: now + Duration::from_millis(board::DFU_GRACE_MS as u64),
                    };
                } else if (now - since).as_millis() as u32 >= ARM_TIMEOUT_MS {
                    info!(
                        "dfu: gesture not acknowledged within {} ms - link is not carrying \
                         packets; nothing was reset",
                        ARM_TIMEOUT_MS
                    );
                    self.phase = Phase::Abandoned;
                }
                Step::Normal
            }
            Phase::Grace { until } => {
                if now >= until {
                    self.phase = Phase::Releasing { sent: 0, next: now };
                    self.release_step(now)
                } else {
                    Step::Normal
                }
            }
            Phase::Releasing { .. } => self.release_step(now),
            Phase::Abandoned => {
                // All four up again = a new attempt is possible. Waiting for that
                // rather than re-arming on the spot: the cells are still down at the
                // moment this state is entered, and the next thing that happens must
                // not be the same gesture firing.
                if self.cells.iter().all(|c| !board::key_down(keys, c.0, c.1)) {
                    self.pressed_at = [None; 4];
                    self.phase = Phase::Idle;
                }
                Step::Normal
            }
            Phase::Done => Step::Jump,
        }
    }

    /// Where each of the four cells stands, as press timestamps.
    fn track(&mut self, keys: &[u8; MATRIX_BYTES], now: Instant) {
        for (i, cell) in self.cells.iter().enumerate() {
            if board::key_down(keys, cell.0, cell.1) {
                if self.pressed_at[i].is_none() {
                    self.pressed_at[i] = Some(now);
                }
            } else {
                self.pressed_at[i] = None;
            }
        }
    }

    /// Whether all four cells are down and went down together.
    ///
    /// Measured across the four press times rather than against `now`: a cell that
    /// has been held since before the gesture is not part of it, however long the
    /// other three have been waiting. Bounce is not a concern at this level - a
    /// contact that drops out for a scan clears its timestamp, and the four cells
    /// have to be down again within the window for the gesture to land.
    fn gesture_landed(&self) -> bool {
        if self.pressed_at.iter().any(|p| p.is_none()) {
            return false;
        }
        let mut oldest: Option<Instant> = None;
        let mut newest: Option<Instant> = None;
        for at in self.pressed_at.iter().flatten() {
            oldest = Some(match oldest {
                Some(o) => o.min(*at),
                None => *at,
            });
            newest = Some(match newest {
                Some(n) => n.max(*at),
                None => *at,
            });
        }
        match (oldest, newest) {
            (Some(old), Some(new)) => {
                (new - old).as_millis() as u32 <= board::DFU_WINDOW_MS
            }
            _ => false,
        }
    }

    /// One pass of the release phase: a report, or silence, or the end of it.
    fn release_step(&mut self, now: Instant) -> Step {
        let done;
        {
            let Phase::Releasing { sent, next } = &mut self.phase else {
                return Step::Jump;
            };
            if now < *next {
                return Step::Silent;
            }
            *next = now + Duration::from_millis(RELEASE_INTERVAL_MS);
            *sent += 1;
            done = *sent >= board::DFU_RELEASE_PACKETS;
        }
        if done {
            self.phase = Phase::Done;
        }
        Step::Release
    }
}

/// Whether a USB cable is supplying this half right now.
///
/// Read when the gesture lands rather than at boot: the cable is plugged in to
/// flash, which is exactly the moment someone presses the gesture, and a reading
/// taken at boot would be answering for hours earlier.
///
/// A chip with no USB peripheral has nothing to ask and answers "yes" - which is
/// also the answer that cannot block anything, since such a chip is not flashed
/// over USB in the first place.
#[cfg(any(feature = "nrf52840", feature = "nrf52833"))]
fn usb_power_present() -> bool {
    embassy_nrf::pac::POWER.usbregstatus().read().vbusdetect()
}

#[cfg(not(any(feature = "nrf52840", feature = "nrf52833")))]
fn usb_power_present() -> bool {
    true
}

/// The Adafruit bootloader's "come up as a UF2 drive" request.
const BOOTLOADER_MAGIC: u32 = 0x57;

/// Reset into the bootloader, and never come back.
///
/// `GPREGRET` lives in the always-on power block and survives a reset; the Adafruit
/// bootloader this board carries reads it before deciding what to be. Writing it and
/// resetting is the entire mechanism - the same register and the same value RMK's
/// `jump_to_bootloader` uses for these boards.
pub fn jump(role: Role) -> ! {
    info!(
        "dfu: {} half -> resetting into the bootloader",
        role.name()
    );
    embassy_nrf::pac::POWER
        .gpregret()
        .write_value(embassy_nrf::pac::power::regs::Gpregret(BOOTLOADER_MAGIC));
    // Diverges (`-> !`), which is what this function promised its caller.
    cortex_m::peripheral::SCB::sys_reset()
}
