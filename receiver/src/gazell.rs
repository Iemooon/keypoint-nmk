//! Gazell receiver: Nordic's proprietary 2.4GHz link, host role.
//!
//! Each half sends its matrix as one bitmap (6x8 = 48 cells = 6 bytes, bit 0 =
//! the half's first cell, row after row). This module owns the radio and
//! republishes the result as `KeyboardEvent`s - the same seam RMK's own split
//! driver publishes at - so the keymap, Vial and USB layers downstream are
//! unchanged.
//!
//! The whole design is deliberately the original receiver's
//! (`redox-w-receiver-basic`, nRF51822 + SDK 11.0.0) transcribed onto one
//! nRF52840, and then re-aimed at this keyboard's shape:
//!
//!   * the host is configured with exactly the same five calls, and NOTHING
//!     else - every parameter left unset is a library default that the working
//!     receiver also ran on. Debugging that port one setting at a time added
//!     twenty-odd variables that all had to be removed again;
//!   * the RX FIFOs are drained the instant a packet arrives (in the callback,
//!     interrupt context), which is what the original achieved by polling its
//!     FIFO every ~10 us in a bare superloop. The FIFO holds only three packets
//!     and a transmitter sends one every 5 ms (less, here: the halves sleep when
//!     idle), so a drain driven by task scheduling can let it fill - and a full
//!     FIFO means no ACK, which makes the transmitter retransmit instead of
//!     queueing new state;
//!   * the received bytes are merged into a matrix and handed on as key events,
//!     with debouncing done by RMK's own debouncer exactly as RMK's built-in
//!     matrix device does it.
//!
//! The merge is **along rows**, not columns: KeyPoint halves are 6x8 and the
//! logical matrix is 12x8 with left = rows 0..5 and right = rows 6..11 (see
//! `board.rs::logical_row`). A flat 4x12 keyboard such as nmk's would shift the
//! right half into columns instead - which is why this is stated rather than
//! assumed.
//!
//! The one difference forced by putting everything on one chip is
//! `xosc_ctl = MANUAL`: Gazell's default AUTO control powers the 32MHz crystal
//! down between timeslots, which would take USB with it. main.rs starts HFXO
//! instead.
//!
//! Linking the Nordic library
//! --------------------------
//! `gzll_nrf52840_gcc.a` from nRF5 SDK 17.1.1 is linked in via build.rs. The
//! library VERSION matters: SDK 12.3's nRF52 build services the second pipe
//! unevenly (measured: pipe 0 ~185 packets/s, pipe 1 ~92), which showed up as
//! stutter on whichever half was not on pipe 0. SDK 17.1.1's nRF52840 build does
//! not. The symbol contract is the same in both, and was read out with
//! `arm-none-eabi-nm` rather than assumed:
//!
//!   * external references it needs from us - exactly six:
//!       memcpy, memset                              (Rust/compiler_builtins)
//!       nrf_gzll_host_rx_data_ready                 (below)
//!       nrf_gzll_device_tx_success / _tx_failed     (below, no-ops in host mode)
//!       nrf_gzll_disabled                           (below, no-op)
//!   * it defines the three ISRs it needs: TIMER2_IRQHandler, RADIO_IRQHandler,
//!     SWI0_EGU0_IRQHandler. Those are C names; nrf-pac names the interrupts
//!     differently, so the three thunks below bridge them.

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use rmk::debounce::fast_debouncer::FastDebouncer;
use rmk::debounce::{DebounceState, DebouncerTrait};
use rmk::event::{Axis, AxisEvent, AxisValType, KeyboardEvent, PointingEvent};
use rmk::input_device::rotary_encoder::Direction;
use rmk::macros::input_device;
use rmk::matrix::KeyState;

use crate::board::{
    BASE_ADDRESS_0, BASE_ADDRESS_1, CHANNEL_TABLE, COL, DATARATE, DIAG_BUSY_GAP_MS,
    DIAG_BUSY_PACKETS, DIAG_ENABLED, DIAG_GAP_REPORT_MS, DIAG_GAP_SEVERE_MS, DIAG_HOLD_MS,
    DIAG_QUEUE_LEN, HALVES, LINK_TIMEOUT_MS, OFF_ENCODER, OFF_GAP, OFF_POINTER_MODE,
    OFF_POINTER_X, OFF_POINTER_Y, OFF_SEQ, PAYLOAD_LENGTH, PIPES_ENABLED, POLL_MS,
    ROW, ROWS_PER_HALF, TIMESLOT_PERIOD_US, logical_row,
};

/// Matrix positions the diagnostic reports are published at, indexed by
/// `[slot][severity]` where slot 0 = left half, 1 = right and severity 0 = "a
/// stall at least `gap_report_ms` long", 1 = "at least `gap_severe_ms`".
///
/// These four cells have no switch behind them, so the matrix can never read
/// them closed and a report can never collide with a real key. They are also
/// the four spare cells that `vial.json` DOES list, which matters more than it
/// looks: Vial's GUI only shows positions the Bluetooth layout file names, and
/// an unnamed cell cannot be assigned from the GUI at all. Choosing unnamed
/// cells (the spare columns of row 0, say) would have made the bindings
/// unreachable without erasing the keyboard's stored layout first.
///
///     (4, 6) -> F13   (4, 7) -> F14      left half
///     (10, 6) -> F15  (10, 7) -> F16     right half
///
/// Those keycodes are what `keymap.rs` puts there, and they carry no character,
/// so the reports cannot disturb typing even if the layout is left alone.
const DIAG_POS: [[(u8, u8); 2]; 2] = [[(4, 6), (4, 7)], [(10, 6), (10, 7)]];

/// Largest interval between two `pump()` calls that still lets a measured gap be
/// believed.
///
/// The first version of this diagnostic measured the gap between the packets as
/// "(now) - (when the last new packet was *observed*)", which is not the same
/// thing: if `pump()` itself is late for any reason, that lateness is added to
/// the gap as if the link had stalled. Publishing a report makes that happen -
/// a report is a real key event, so `get_event()` returns one, RMK runs it
/// through the pipeline and out of USB, and the next `pump()` can be tens of
/// milliseconds late. One true stall could therefore manufacture the next
/// report, and that one the next: a self-sustaining cascade whose rate depends
/// on host load. Two runs of the same keyboard measured 0.3 and 1.4 reports per
/// second, which is exactly the variability such a mechanism predicts - and a
/// rate far above anything the typist noticed, which is what gave it away.
///
/// The decision rule after this change: a gap is only believed when the pulse
/// that observed it was itself on time. `pump()` runs from the one-millisecond
/// poll loop, so a healthy pulse interval is ~1 ms no matter how long the link
/// has been silent; a pulse interval larger than this constant means the
/// measurement window itself is untrustworthy and the sample is dropped.
///
/// The cost is the opposite error: a stall that ends while `pump()` is late goes
/// unreported. That direction is the safe one - this build exists to prove
/// stalls happen, and an understated count still does that, while an inflated
/// one proves nothing.
const DIAG_MAX_PUMP_GAP_MS: u64 = 20;

/// After a report is published, ignore gap measurements for this long.
///
/// Publishing a report is what delays the next `pump()`, so the window right
/// after one is exactly where a cascade would start. This is belt and braces on
/// top of `DIAG_MAX_PUMP_GAP_MS`: it keeps the mechanism from feeding itself
/// even if the pulse-interval guard is ever loosened.
const DIAG_QUIET_MS: u64 = 150;

/// u32 words per half's row bytes: four rows fit in one word, so a 6-row half
/// needs two.
const WORDS_PER_HALF: usize = ROWS_PER_HALF.div_ceil(4);

/// Gazell protocol enumerations - the library's own constants, not board
/// settings:
///   nrf_gzll_mode_t     : DEVICE = 0, HOST = 1, SUSPEND = 2
///   nrf_gzll_xosc_ctl_t : AUTO = 0, MANUAL = 1
const GZLL_MODE_HOST: u32 = 1;
const GZLL_XOSC_CTL_MANUAL: u32 = 1;

/// The left half is always on pipe 0: it is the only pipe that can use
/// `base_address_0`, which is the left half's base address (pipes 1..7 all share
/// `base_address_1`, the right half's). Their address prefix bytes are the
/// library defaults, which is what the transmitters use, so they are left alone.
const PIPE_LEFT: usize = 0;
const PIPE_RIGHT: usize = 1;

/// Slot in `raw` for each half: 0 = left, 1 = right.
const SLOT_LEFT: usize = 0;
const SLOT_RIGHT: usize = 1;

/// The `raw` slot that holds `pipe`'s data.
const fn slot_of(pipe: usize) -> usize {
    if pipe == PIPE_LEFT { SLOT_LEFT } else { SLOT_RIGHT }
}

// ---------------------------------------------------------------------------
// FFI to the Nordic library
// ---------------------------------------------------------------------------

// `nrf_gzll_device_tx_info_t` is 8 bytes, which AAPCS passes via a pointer to a
// caller-made copy - hence `*const u8` rather than a by-value struct. Getting
// this wrong would corrupt the stack, so it is spelled out rather than
// "modelled". (Host mode never calls those two callbacks anyway.)
#[allow(improper_ctypes)]
unsafe extern "C" {
    fn nrf_gzll_init(mode: u32) -> bool;
    fn nrf_gzll_enable() -> bool;
    fn nrf_gzll_set_channel_table(channel_table: *mut u8, size: u32) -> bool;
    fn nrf_gzll_set_datarate(data_rate: u32) -> bool;
    fn nrf_gzll_set_timeslot_period(period_us: u32) -> bool;
    fn nrf_gzll_set_base_address_0(base_address: u32) -> bool;
    fn nrf_gzll_set_base_address_1(base_address: u32) -> bool;
    fn nrf_gzll_set_xosc_ctl(xosc_ctl: u32) -> bool;
    // nrf_gzll_set_tx_power is deliberately not declared: the host's ACK power
    // stays at the library default, as it does in the receiver this was copied
    // from. See the note in `init`.
    fn nrf_gzll_set_rx_pipes_enabled(pipes: u32) -> bool;

    fn nrf_gzll_get_rx_fifo_packet_count(pipe: u32) -> i32;
    fn nrf_gzll_fetch_packet_from_rx_fifo(pipe: u32, payload: *mut u8, length: *mut u32) -> bool;

    // Provided by the archive; called from the thunks below.
    fn TIMER2_IRQHandler();
    fn RADIO_IRQHandler();
    fn SWI0_EGU0_IRQHandler();
}

/// `nrf_gzll_set_channel_table` takes a non-const pointer, so the table has to
/// live in a mutable static. It is only read by the library.
static mut CHANNEL_TABLE_STORAGE: [u8; CHANNEL_TABLE.len()] = CHANNEL_TABLE;

/// Each half's raw row bytes, one per row, packed four-to-a-word (row 0 in the
/// low byte of word 0). Written from the RX callback (interrupt context), read
/// by the task.
///
/// A 6-row half does not fit in one word, so a report reaches this static as two
/// separate stores and a reader can catch the pair half-updated. That mixture used
/// to be survivable: RMK's debouncer made every change wait out its window, and the
/// next packet repaired the picture well inside it. The receiver now commits a
/// change on the first pass that sees it (see the debouncer field below), so the
/// read is made indivisible instead - the two loads happen with interrupts off, in
/// `pump()`. That is a handful of instructions once per packet per half, and it is
/// the whole of the "seqlock" this used to say the problem did not deserve: the
/// pair is written by the radio callback and read by a task, so one PRIMASK write
/// is genuinely sufficient here.
static RAW: [[AtomicU32; WORDS_PER_HALF]; HALVES] = [
    [const { AtomicU32::new(0) }; WORDS_PER_HALF],
    [const { AtomicU32::new(0) }; WORDS_PER_HALF],
];

/// Latest encoder count per half, as it came off the air: the transmitter's
/// cumulative detent count, low 8 bits.
///
/// Cumulative, like the matrix bitmap and for the same reason. The link has no
/// retransmission, so "the knob is at 37" repairs itself on the next packet, while
/// "the knob moved one click" would be gone for good.
static ENC_RAW: [AtomicU32; HALVES] = [const { AtomicU32::new(0) }; HALVES];

/// The pointing device's cumulative X and Y, packed as two i16s into one word, per
/// half.
///
/// Cumulative for the same reason as the knob's count and the matrix bitmap: with
/// no retransmission on this link, "the pointer is at 340" repairs itself on the
/// next packet while "the pointer moved 3" would be gone for good.
///
/// A half carrying no pointing device simply reports zero forever, which reads as
/// no movement - exactly what an untouched stick reports.
static PTR_RAW: [AtomicU32; HALVES] = [const { AtomicU32::new(0) }; HALVES];

/// How long the newest packet's displacement took, in milliseconds, per half.
///
/// The same one-way hand-off as `PTR_RAW`: written in the radio callback, read by
/// the task, which is the only context where the curve can run - it reads a
/// keymap-backed tier. A value rather than a difference, like the position it
/// arrives with, so a lost packet costs nothing.
static PTR_GAP: [AtomicU32; HALVES] = [const { AtomicU32::new(0) }; HALVES];

/// Cap on detents owed to RMK per half, in either direction.
///
/// A report normally carries 0 or 1: at 5 ms a packet against 20 detents a turn,
/// even a fast twist moves one detent every few packets. So anything past a full
/// turn is not somebody spinning the knob 300 clicks, it is something having gone
/// wrong - and bounding it keeps a glitch from turning into a flood of keypresses.
const ENCODER_PENDING_LIMIT: i16 = 20;

/// The largest difference between two packets that can be a turn: anything bigger
/// is a half that has restarted, not a hand.
///
/// The count is cumulative and a restarted half begins it at zero, so the first
/// packet after a restart differs from our baseline by however far the knob was
/// moved during the whole previous session - and reporting that moves the host's
/// volume by exactly that much. Time cannot separate the two cases: the
/// transmitters go silent whenever they are idle, so an ordinary pause is
/// indistinguishable from a power cycle. Size can: a detent is four quadrature
/// steps and each one is sent as it completes, so a real packet carries one detent
/// (two if a packet was lost), and even a flick carries only what completed since
/// the last packet.
///
/// 6 is several times the observed worst case and well below any session's net
/// turn, which is what makes this a separator rather than a threshold.
const ENCODER_MAX_DETENTS_PER_PACKET: i8 = 6;

/// Whether knob detents are reported to RMK at all.
///
/// Kept as a switch because the fault it was added for is worth being able to
/// take back out in one line: with detent reporting on, the receiver delivered no
/// key to the host at all, and only a restart brought it back.
///
/// The cause was the order of the two halves of a detent. A press reported while
/// an earlier press was still owed its release overwrites the direction that
/// release would have named, so the first press is never closed out. A position
/// left down does not merely make the knob misbehave: RMK buffers later keys
/// behind it, and the whole keyboard goes quiet. RMK's own encoder device cannot
/// do this - it alternates strictly, and waits 5 ms between the two halves. Both
/// are mirrored below.
///
///   false - detents are differenced but never handed to RMK. Not a way to run
///           the keyboard; a way to rule the knob out of a fault.
///   true  - normal operation.
const ENCODER_ENABLED: bool = true;

/// What a half writes in `OFF_SEQ` for its first packets after a reset.
///
/// Any non-zero value would do; this one is unlikely to appear by accident.
const BOOT_MARKER: u8 = 0xA5;

/// Whether the last packet from each half announced a reset.
///
/// Set in the radio callback, acted on once in the task - the same split as the
/// pointing device's mode, and for the same reason: the callback runs in an interrupt,
/// where anything that publishes can silently drop.
static BOOT_SEEN: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];

/// Packets received per half since boot, so the task can tell whether a half has
/// said anything since the last time it looked.
static RX_COUNT: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];

/// Total packets seen; used once, for the startup log line.
static RX_PACKETS: AtomicU32 = AtomicU32::new(0);

/// Take everything waiting in `pipe`'s RX FIFO into `RAW`; returns how many
/// packets were taken.
///
/// Called from the RX callback - the earliest possible moment - and again from
/// the task as a safety net.
///
/// # Safety
/// `pipe` must be a valid pipe index.
unsafe fn drain_pipe(pipe: u32) -> u32 {
    let slot = slot_of(pipe as usize);
    let mut taken = 0u32;
    unsafe {
        while nrf_gzll_get_rx_fifo_packet_count(pipe) > 0 {
            let mut payload = [0u8; PAYLOAD_LENGTH as usize];
            let mut length = PAYLOAD_LENGTH;
            if !nrf_gzll_fetch_packet_from_rx_fifo(pipe, payload.as_mut_ptr(), &mut length) {
                break;
            }
            taken += 1;
            if length != PAYLOAD_LENGTH {
                continue;
            }
            // The knob's count rides along in every report, so it needs no separate
            // path: as with the matrix, the newest value simply overwrites the last.
            ENC_RAW[slot].store(payload[OFF_ENCODER] as u32, Ordering::Relaxed);
            // The stick's position rides along the same way, packed as two i16s.
            let px = i16::from_le_bytes([payload[OFF_POINTER_X], payload[OFF_POINTER_X + 1]]);
            let py = i16::from_le_bytes([payload[OFF_POINTER_Y], payload[OFF_POINTER_Y + 1]]);
            PTR_RAW[slot].store(
                ((px as u16 as u32) << 16) | (py as u16 as u32),
                Ordering::Relaxed,
            );
            // And how long that displacement took. Restated every packet rather
            // than differenced: it is what the curve divides by, and the next packet
            // brings the interval in force with it.
            PTR_GAP[slot].store(payload[OFF_GAP] as u32, Ordering::Relaxed);
            // What this half's device means: cursor or wheel. Not a measurement, so
            // there is nothing to difference and nothing to repair - the newest value
            // simply wins, exactly like the knob's count.
            //
            // Only RECORDED here. This runs in the radio's interrupt context, and the
            // receiver's answer to it is an EVENT - see `pointer_speed::sync_modes`,
            // which publishes from the event loop instead. RMK's `publish_event` is
            // non-blocking and drops when the channel is full, which in an interrupt
            // means a mode change can go missing with nothing to repeat it.
            crate::pointer_speed::set_latched_mode(slot as u8, payload[OFF_POINTER_MODE] != 0);
            // A half that has just reset says so here. Recorded only: the task adopts
            // the half's counters as a baseline when it sees this, which is what keeps
            // the first packet of a new session from reading as a turn. See OFF_SEQ.
            if payload[OFF_SEQ] == BOOT_MARKER {
                BOOT_SEEN[slot].store(1, Ordering::Relaxed);
            }
            // Unpack the bitmap into one byte per row - the shape `RAW` and
            // everything downstream expect: bit `row * COL + col` of the report
            // is row `row`, column `col`. Reading only the columns this board has
            // is also what stops a transmitter with a wider row from leaking its
            // extra bits into a neighbouring row.
            let mut packed = [0u32; WORDS_PER_HALF];
            for row in 0..ROWS_PER_HALF {
                let mut row_bits = 0u8;
                for col in 0..COL {
                    let bit = row * COL + col;
                    if payload[bit / 8] & (1 << (bit % 8)) != 0 {
                        row_bits |= 1 << col;
                    }
                }
                packed[row / 4] |= (row_bits as u32) << (8 * (row % 4));
            }
            // Two stores (one per word): wrap in interrupt::free to prevent the task
            // from reading a half-updated matrix (one word stored, the other not yet).
            cortex_m::interrupt::free(|_| {
                for (w, value) in packed.iter().enumerate() {
                    RAW[slot][w].store(*value, Ordering::Relaxed);
                }
            });
        }
    }
    if taken > 0 {
        RX_COUNT[slot].fetch_add(taken, Ordering::Relaxed);
    }
    taken
}

// ---------------------------------------------------------------------------
// The callbacks the Nordic library requires
// ---------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub extern "C" fn nrf_gzll_host_rx_data_ready(_pipe: u32, _info: u32) {
    // Drain BOTH pipes, not just the one that notified.
    //
    // All the FIFOs share one packet pool (NRF_GZLL_CONST_MAX_TOTAL_PACKETS = 6)
    // and Gazell only acknowledges a packet it has room to store, so a pipe
    // whose notification was dropped by the library's finite callback queue
    // would sit on pool packets and starve the other pipe: that device gets no
    // ACK, keeps retransmitting, and its own TX FIFO fills.
    unsafe {
        drain_pipe(PIPE_LEFT as u32);
        drain_pipe(PIPE_RIGHT as u32);
    }
    RX_PACKETS.fetch_add(1, Ordering::Relaxed);
}

#[unsafe(no_mangle)]
pub extern "C" fn nrf_gzll_device_tx_success(_pipe: u32, _info: *const u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn nrf_gzll_device_tx_failed(_pipe: u32, _info: *const u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn nrf_gzll_disabled() {}

// ---------------------------------------------------------------------------
// Vector-table thunks: PAC interrupt name -> SDK ISR name
// ---------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub unsafe extern "C" fn TIMER2() {
    unsafe { TIMER2_IRQHandler() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn RADIO() {
    unsafe { RADIO_IRQHandler() }
}

/// The PAC names this interrupt `EGU0_SWI0`; the SDK calls it
/// `SWI0_EGU0_IRQHandler`. Same vector, two naming conventions.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn EGU0_SWI0() {
    unsafe { SWI0_EGU0_IRQHandler() }
}

// ---------------------------------------------------------------------------
// The input device
// ---------------------------------------------------------------------------

/// The pointing device id the right half's stick publishes under.
///
/// It is the half's slot, and it has to match the `device_id` its
/// `PointingProcessor` is built with - the two are compiled apart and only agree by
/// convention. 1 is the number the BLE firmware used for the TrackPoint, so a
/// keymap or mouse-layer setting that referred to it still does.
pub const TRACKPOINT_ID: u8 = 1;

/// The pointing device id the left half's pad publishes under - its slot, and the
/// number the BLE firmware used for the A320.
pub const TRACKPAD_ID: u8 = 0;

/// What this device publishes.
///
/// A keyboard and a pointing device are two separate channels into RMK: the
/// keyboard takes `KeyboardEvent` through the keymap, the pointer takes
/// `PointingEvent` through a `PointingProcessor`. This receiver owns both halves, so
/// it has both to publish. RMK's `#[input_device]` allows that with a derived
/// wrapper enum - each variant routes the event to its own type's channel, which is
/// what the subscribers on the other side are listening on.
///
/// `Clone` is not optional: the derive expands to `embassy_sync` channels, and their
/// `Sender` requires the payload to be cloneable. Both variants already are.
///
/// The derive also names `::embassy_sync` by absolute path, which is why this crate
/// carries an `embassy-sync` dependency of its own even though it never uses the
/// crate directly - without it the expansion does not resolve.
#[derive(Clone, rmk::macros::Event)]
pub enum GazellEvent {
    Keyboard(KeyboardEvent),
    Pointing(PointingEvent),
}

#[input_device(publish = GazellEvent)]
pub struct GazellMatrix {
    /// One byte per logical row: `raw[logical_row(slot, row)]`.
    raw: [u8; ROW],
    /// Per-key debounced state, maintained by RMK's own debouncer - one of the two
    /// RMK ships, so this behaves like a matrix source rather than like a
    /// hand-written approximation of one.
    ///
    /// `FastDebouncer` commits a change on the FIRST pass that sees it and then
    /// holds that key still for DEBOUNCE_THRESHOLD ms. `DefaultDebouncer`, the
    /// obvious choice, would instead make every change wait out the threshold
    /// before believing it - the wrong trade for this input. The stream is already
    /// debounced where it is measured: a half only puts a state on the air once
    /// DEBOUNCE_TICKS consecutive 1 ms scans agree, and it re-sends it every 5 ms
    /// after that (tx/src/lib.rs). A second window here therefore cannot filter
    /// anything the transmitter did not; it can only delay a change that has
    /// already been proven stable, and every key paid 5 ms for it.
    ///
    /// What is still worth keeping is the lockout half of the fast debouncer: a key
    /// whose state changes twice inside the threshold is held until the window
    /// expires, so contradictory reports cannot become two events. That is the one
    /// failure mode a pre-debounced stream cannot rule out, and it costs nothing on
    /// the leading edge.
    debouncer: FastDebouncer<ROW, COL>,
    key_states: [[KeyState; ROW]; COL],
    /// Last time a packet was seen per half, for the link timeout.
    last_seen_ms: [u64; 2],
    /// Packets seen per half, to notice activity between polls.
    packets: [u32; 2],
    announced: bool,

    // ---- the knob ----------------------------------------------------------
    /// The last encoder count seen per half, to difference against.
    encoder_last: [u8; 2],
    /// Detents a half has reported but RMK has not been told about yet, signed:
    /// the sign is which way.
    ///
    /// A turn of several detents - a flick, or a packet lost along the way - owes
    /// several events, because each one is a keypress in its own right. Reporting
    /// one and dropping the rest is exactly what makes a knob feel like it skipped.
    encoder_pending: [i16; 2],
    /// Set when a half has announced a restart, so its next count re-establishes the
    /// baseline instead of being read as a turn.
    ///
    /// The count is cumulative and a restarted half begins it at zero, while the
    /// receiver's baseline belongs to the session that just ended. The first packet of
    /// the new session therefore differs by however far the knob went in the old one,
    /// and moves the host's volume by that much - and, unlike a genuine flick, usually
    /// by little enough to pass the size test below. Size cannot separate the two
    /// cases, and neither can time, because an idle half stops sending exactly as a
    /// powered-off one does. Only the half saying so can.
    encoder_resync: [bool; 2],
    /// The direction last reported for each half, so the release that follows it
    /// can name the same direction.
    ///
    /// RMK's own encoder device reports a detent as `pressed = true` and then the
    /// end of it as `pressed = false` with the same direction; reporting only the
    /// press is not the shape its consumers are written against.
    encoder_reported: [Direction; 2],

    // ---- the pointing device -----------------------------------------------
    /// The last position seen per half, to difference against.
    pointer_last: [(i16, i16); 2],
    /// Movement owed to RMK per half: the difference between two positions, held
    /// until an event can be published. Signed; one unit is one count.
    pointer_pending: [(i16, i16); 2],
    /// Set while a half has been out of touch, so its next position only
    /// re-establishes the baseline instead of being read as movement.
    ///
    /// Without this a half that restarts comes back with its position at zero, and
    /// the difference from wherever it was looks like the stick had just been flung
    /// across the screen. The knob needs no flag of its own - its differences are
    /// judged by size instead (ENCODER_MAX_DETENTS_PER_PACKET), because a turn after
    /// a silence is still a turn while a flung stick is not.
    pointer_resync: [bool; 2],
    /// How much time the displacement owed to RMK took, per half, summed from the
    /// `OFF_GAP` of every packet that contributed to it.
    ///
    /// Summed rather than taken from the last packet, because a displacement can
    /// span several reports: the stick delivers packets as fast as the device makes
    /// them, while this loop publishes once every `POLL_MS`. Scoring the whole
    /// movement against the last report's interval would read as a flick.
    ///
    /// Reset with the pending displacement it belongs to - the two are one quantity
    /// in two units.
    pointer_window_ms: [u32; 2],

    // ---- diagnostic build (board.toml [diagnostics]) -----------------------
    /// Consecutive packets that arrived without a gap, per half. A gap is only
    /// reported once this run has been established, which is what keeps the long
    /// gap before the first keystroke after an idle period - where the
    /// transmitters were simply asleep and nothing was wrong - out of the log.
    diag_busy_run: [u32; 2],
    /// Reports waiting to be published, as matrix positions.
    diag_queue: [(u8, u8); DIAG_QUEUE_LEN],
    diag_queue_len: usize,
    /// The report currently held down, and when to release it.
    diag_held: Option<(u8, u8, u64)>,
    /// Gaps seen but not published because the queue was full.
    diag_dropped: u32,
    /// Stalls seen, per slot and severity, whether published or not.
    diag_seen: [[u32; 2]; 2],
    /// When `pump()` last ran, to tell a late pulse from a late packet - see
    /// `DIAG_MAX_PUMP_GAP_MS`.
    diag_last_pump_ms: u64,
    /// Gap measurements before this time are ignored - see `DIAG_QUIET_MS`.
    diag_quiet_until_ms: u64,
    /// Samples dropped because the pulse that took them was late.
    diag_untrusted: u32,
    /// The diagnostic's own record of when each half was last heard.
    ///
    /// Deliberately separate from `last_seen_ms`, which drives the link timeout:
    /// the diagnostic forgets its timestamps whenever a pulse cannot be trusted,
    /// and doing that to `last_seen_ms` would also disable the timeout that
    /// releases a half's keys.
    diag_last_seen_ms: [u64; 2],
}

impl GazellMatrix {
    pub fn new() -> Self {
        Self {
            raw: [0; ROW],
            debouncer: FastDebouncer::new(),
            key_states: [[KeyState::default(); ROW]; COL],
            last_seen_ms: [0; 2],
            packets: [0; 2],
            announced: false,
            encoder_last: [0; 2],
            encoder_pending: [0; 2],
            encoder_reported: [Direction::None; 2],
            pointer_last: [(0, 0); 2],
            pointer_pending: [(0, 0); 2],
            pointer_resync: [false; 2],
            pointer_window_ms: [0; 2],
            encoder_resync: [false; 2],
            diag_busy_run: [0; 2],
            diag_queue: [(0, 0); DIAG_QUEUE_LEN],
            diag_queue_len: 0,
            diag_held: None,
            diag_dropped: 0,
            diag_seen: [[0; 2]; 2],
            diag_last_pump_ms: 0,
            diag_quiet_until_ms: 0,
            diag_untrusted: 0,
            diag_last_seen_ms: [0; 2],
        }
    }

    /// Queue a report at `pos`, dropping it (and counting it) if the queue is
    /// full. Dropping is deliberate: the queue exists so that a burst does not
    /// stall the input device, and the count keeps the loss visible.
    fn diag_push(&mut self, pos: (u8, u8)) {
        if self.diag_queue_len < DIAG_QUEUE_LEN {
            self.diag_queue[self.diag_queue_len] = pos;
            self.diag_queue_len += 1;
        } else {
            self.diag_dropped += 1;
        }
    }

    /// Configure and enable the radio. Called once, from `main`, before the
    /// executor starts running this device.
    pub fn init(&mut self) {
        unsafe {
            // MANUAL crystal control: USB needs HFXO permanently, so Gazell must
            // not power it down between timeslots.
            let _ = nrf_gzll_set_xosc_ctl(GZLL_XOSC_CTL_MANUAL);

            let ok = nrf_gzll_init(GZLL_MODE_HOST);

            // ---- the original receiver's configuration, plus two experiments ----
            // redox-w-receiver-basic makes exactly the calls below in this order
            // and leaves every other parameter at the library default. Two of
            // those defaults were questioned here and both experiments were
            // reverted - the reasoning is kept at the call sites so it does not
            // have to be rediscovered:
            //
            //  * TX power. Left at the library default (0 dBm for the ACKs sent
            //    back to the halves); the 4 dBm experiment and why it was reverted
            //    are noted at the call site below.
            //
            //  * Enabled pipes. `pipes_enabled` in board.toml is back at the
            //    default (0xFF); the experiment and its withdrawal are recorded
            //    there.
            //
            // nrf_gzll_set_timeslots_per_channel() is still deliberately NOT
            // called. That one is a hard requirement rather than an inheritance:
            // the host rotation has to stay shorter than the transmitters'
            // out-of-sync dwell (6 x 2 = 12 timeslots against their 15), or a half
            // coming back from silence starts sweeping past a listening host. The
            // arithmetic is in board.rs.
            let ch_ok = nrf_gzll_set_channel_table(
                core::ptr::addr_of_mut!(CHANNEL_TABLE_STORAGE) as *mut u8,
                CHANNEL_TABLE.len() as u32,
            );
            nrf_gzll_set_datarate(DATARATE);
            nrf_gzll_set_timeslot_period(TIMESLOT_PERIOD_US);
            nrf_gzll_set_base_address_0(BASE_ADDRESS_0);
            nrf_gzll_set_base_address_1(BASE_ADDRESS_1);
            // nrf_gzll_set_tx_power is deliberately NOT called.
            //
            // redox-w-receiver-basic does not call it either, so the ACKs this host
            // sends back to the halves go out at the library default of 0 dBm
            // (NRF_GZLL_DEFAULT_TX_POWER). Setting 4 dBm here was reasoned rather
            // than measured - a stronger ACK ought to mean fewer retries on the
            // device side, and the enum's top entry is 4 dBm - but the receiver it
            // was copied from has never needed it. Reverted for the same reason as
            // pipes_enabled: the shipped firmware works on the default, and every
            // difference from it is a difference that has to earn its place.
            nrf_gzll_set_rx_pipes_enabled(PIPES_ENABLED);

            let enabled = nrf_gzll_enable();

            info!("gzll: init={} channel_table={} enabled={}", ok, ch_ok, enabled);
        }
    }

    /// Copy what the RX callback has collected into `raw` and refresh the link
    /// timeout bookkeeping.
    fn pump(&mut self) {
        let now = embassy_time::Instant::now().as_millis();

        // How long since the previous pulse. A healthy value is about one
        // millisecond (POLL_MS) regardless of how long the link has been quiet -
        // silence does not slow the loop down, it only means the loop finds no
        // new packets. Anything larger means this pulse is late, and a gap read
        // from a late pulse is the sum of a real gap and that lateness. See
        // DIAG_MAX_PUMP_GAP_MS.
        let pulse_ms = if self.diag_last_pump_ms == 0 {
            u64::MAX
        } else {
            now.saturating_sub(self.diag_last_pump_ms)
        };
        self.diag_last_pump_ms = now;

        let trusted = pulse_ms <= DIAG_MAX_PUMP_GAP_MS && now >= self.diag_quiet_until_ms;
        if DIAG_ENABLED && !trusted {
            // This pulse cannot be believed, and more importantly its *idea* of
            // when each half was last heard must not be inherited by the next
            // one - that is how a single late pulse would contaminate every gap
            // measured after it. Forgetting the timestamps turns the next packet
            // into a "first packet", which is never reported.
            self.diag_untrusted += 1;
            self.diag_last_seen_ms = [0; 2];
            self.diag_busy_run = [0; 2];
        }

        for pipe in [PIPE_LEFT, PIPE_RIGHT] {
            let slot = slot_of(pipe);
            // The callback normally drains first, in interrupt context; this is
            // the safety net for a notification that did not survive the
            // library's finite callback queue.
            unsafe {
                drain_pipe(pipe as u32);
            }
            let count = RX_COUNT[slot].load(Ordering::Relaxed);
            // A pass with no new packet must ACT on nothing. The buffers
            // below still hold the last packet, and every block below is
            // written to run once per packet - acting on them anyway is how
            // this receiver used to lose the alignment it keeps for a half
            // that has been away. `pointer_resync` is armed by the link
            // timeout and was consumed here, against the OLD position, tens
            // of milliseconds before the half came back; the knob's baseline
            // was handled the same way, so the whole turn it had accumulated
            // over the session was reported as movement on its return - the
            // volume jumping on power-up.
            if count == self.packets[slot] {
                continue;
            }
            // The diagnostic keeps its own timestamps: they are forgotten whenever a
            // pulse cannot be trusted, which must not happen to the pair that
            // drives the link timeout.
            let gap = if self.diag_last_seen_ms[slot] == 0 {
                None
            } else {
                Some(now.saturating_sub(self.diag_last_seen_ms[slot]))
            };
            self.packets[slot] = count;
            self.last_seen_ms[slot] = now;
            self.diag_last_seen_ms[slot] = now;

            // Diagnostic: publish a stall that interrupted a running stream.
            // `last_seen_ms == 0` means the link timeout already gave up on
            // this half, so this is a first packet after silence rather than
            // a stall inside a stream - and the run below is only built up by
            // packets that arrive without a gap, so it cannot fire there
            // either.
            if DIAG_ENABLED && let Some(gap) = gap {
                if gap < DIAG_BUSY_GAP_MS {
                    self.diag_busy_run[slot] = self.diag_busy_run[slot].saturating_add(1);
                } else {
                    let busy = self.diag_busy_run[slot] >= DIAG_BUSY_PACKETS;
                    self.diag_busy_run[slot] = 0;
                    if busy && gap >= DIAG_GAP_REPORT_MS {
                        let severity = usize::from(gap >= DIAG_GAP_SEVERE_MS);
                        self.diag_seen[slot][severity] += 1;
                        self.diag_push(DIAG_POS[slot][severity]);
                    }
                }
            }
            // Interrupts off for the length of two loads: the pair is one snapshot
            // and the radio callback writes it as two words. See RAW above.
            let matrix_words = cortex_m::interrupt::free(|_| {
                let mut words = [0u32; WORDS_PER_HALF];
                for (w, word) in words.iter_mut().enumerate() {
                    *word = RAW[slot][w].load(Ordering::Relaxed);
                }
                words
            });
            for (w, value) in matrix_words.iter().enumerate() {
                for i in 0..4 {
                    let row = w * 4 + i;
                    if row < ROWS_PER_HALF {
                        self.raw[logical_row(slot, row)] = ((value >> (8 * i)) & 0xFF) as u8;
                    }
                }
            }

            // The knob, read as the difference of two cumulative counts. That is
            // what makes a lost packet cost nothing: the next report still carries
            // the whole count, so the movement is simply measured a little later.
            // A half that has restarted said so in its first packets; see OFF_SEQ. Every
            // cumulative field it sends begins again at zero, so this pass adopts them
            // as baselines instead of differencing them against the last session.
            if BOOT_SEEN[slot].swap(0, Ordering::Relaxed) != 0 {
                self.encoder_resync[slot] = true;
                self.pointer_resync[slot] = true;
            }
            let count = ENC_RAW[slot].load(Ordering::Relaxed) as u8;
            // Wrapping difference, read as signed - the count is 8 bits and the two
            // ends agree that a DECREASING count is counter-clockwise, so a negative
            // difference is counter-clockwise here too.
            let resyncing = self.encoder_resync[slot];
            self.encoder_resync[slot] = false;
            // A half that has announced a restart begins its count at zero, and the
            // baseline in hand belongs to the previous session - so that first value is
            // adopted rather than differenced. What decides this is the half saying it
            // restarted, not the size of the difference.
            let moved = if resyncing {
                0
            } else {
                count.wrapping_sub(self.encoder_last[slot]) as i8
            };
            self.encoder_last[slot] = count;
            // A difference too big to be a hand is a half that has restarted: the new
            // count is adopted silently and nothing is reported. Everything else is a
            // real turn - including the first detent after a pause, which a
            // time-based rule would have thrown away, since from here a pause and a
            // power cycle look identical.
            // (`as i16` before the `abs`, so a difference of exactly i8::MIN - 128,
            // the one magnitude `i8::abs` cannot represent - is judged by its size
            // like any other restart instead of wrapping through the filter.)
            if moved != 0 && (moved as i16).abs() <= ENCODER_MAX_DETENTS_PER_PACKET as i16 {
                self.encoder_pending[slot] = (self.encoder_pending[slot] + moved as i16)
                    .clamp(-ENCODER_PENDING_LIMIT, ENCODER_PENDING_LIMIT);
            }

            // The stick, differenced the same way and for the same reason. Wrapping
            // arithmetic throughout: the position is a wrapping accumulator on the
            // transmitter too, so both ends wrap at the same place and the
            // difference stays correct across the wrap.
            //
            // A half with no stick reports zero forever, so this costs a load and a
            // subtraction and never produces movement.
            let packed = PTR_RAW[slot].load(Ordering::Relaxed);
            let pos = ((packed >> 16) as u16 as i16, (packed & 0xFFFF) as u16 as i16);
            if self.pointer_resync[slot] {
                self.pointer_resync[slot] = false;
                self.pointer_last[slot] = pos;
                self.pointer_window_ms[slot] = 0;
            } else {
                let dx = pos.0.wrapping_sub(self.pointer_last[slot].0);
                let dy = pos.1.wrapping_sub(self.pointer_last[slot].1);
                self.pointer_last[slot] = pos;
                if dx != 0 || dy != 0 {
                    self.pointer_pending[slot].0 = self.pointer_pending[slot].0.saturating_add(dx);
                    self.pointer_pending[slot].1 = self.pointer_pending[slot].1.saturating_add(dy);
                    // The clock advances only when something moves. A packet with no
                    // displacement owes no time to anything, and letting an idle half
                    // lengthen the window would make the next stroke look slower than
                    // it was.
                    self.pointer_window_ms[slot] = self
                        .pointer_window_ms[slot]
                        .saturating_add(PTR_GAP[slot].load(Ordering::Relaxed));
                }
            }
        }

        if !self.announced {
            self.announced = true;
            info!("gzll: first packets received ({} total)", RX_PACKETS.load(Ordering::Relaxed));
        }
    }

    async fn read_gazell_event(&mut self) -> GazellEvent {
        loop {
            // The pointing device's mode is applied HERE, in task context, and not
            // where the packet carrying it was parsed - that parse runs in the radio's
            // interrupt, where a publish is non-blocking and may silently drop. Once
            // per pass, and a no-op unless one of the two switches moved since the last
            // one.
            crate::pointer_speed::sync_modes();
            self.pump();

            // Diagnostic reports, before the real matrix. A report is held down
            // for `hold_ms` and then released, so the host sees a clean key
            // press. While one is being held, real keys are still published
            // below - the hold is only tens of milliseconds, and it must never
            // gate typing.
            //
            // `DIAG_ENABLED` is a compile-time constant, so this whole block
            // disappears from the shipping build.
            if DIAG_ENABLED {
                let now = embassy_time::Instant::now().as_millis();
                if let Some((row, col, release_at)) = self.diag_held {
                    if now >= release_at {
                        self.diag_held = None;
                        // Publishing is what delays the next pulse, so the
                        // measurements taken right after one are the ones a
                        // cascade would be built from - see DIAG_QUIET_MS.
                        self.diag_quiet_until_ms = now + DIAG_QUIET_MS;
                        return GazellEvent::Keyboard(KeyboardEvent::key(row, col, false));
                    }
                } else if self.diag_queue_len > 0 {
                    let (row, col) = self.diag_queue[0];
                    for i in 1..self.diag_queue_len {
                        self.diag_queue[i - 1] = self.diag_queue[i];
                    }
                    self.diag_queue_len -= 1;
                    self.diag_held = Some((row, col, now + DIAG_HOLD_MS));
                    // Covers the hold, the release, and a margin after it.
                    self.diag_quiet_until_ms = now + DIAG_HOLD_MS + DIAG_QUIET_MS;
                    return GazellEvent::Keyboard(KeyboardEvent::key(row, col, true));
                }
            }

            // Link timeout: a half that has gone quiet releases its keys, so a
            // lost final packet cannot leave a key stuck. The transmitters stop
            // sending about half a second after the last key, so this fires once at
            // the end of a normal quiet stretch - releasing keys that are already
            // up, and re-baselining the pointer for the next burst - and it is what
            // saves a half that is genuinely gone.
            let now = embassy_time::Instant::now().as_millis();
            for slot in 0..HALVES {
                if self.last_seen_ms[slot] != 0
                    && now.saturating_sub(self.last_seen_ms[slot]) > LINK_TIMEOUT_MS
                {
                    self.last_seen_ms[slot] = 0;
                    for row in 0..ROWS_PER_HALF {
                        self.raw[logical_row(slot, row)] = 0;
                    }
                    // The knob is deliberately not re-aligned here. A silence is what
                    // an idle half does between every pair of keystrokes, so aligning
                    // on it would throw away the first detent after every pause. The
                    // alignment comes from the half itself instead - see OFF_SEQ - and
                    // what is dropped here is only a burst already half-emitted when it
                    // went away.
                    self.encoder_pending[slot] = 0;
                    // Same reasoning for the stick: a half that comes back has most
                    // likely restarted, and its position starts at zero again - so
                    // the difference from wherever it was would read as the stick
                    // being flung across the desk.
                    self.pointer_resync[slot] = true;
                    self.pointer_pending[slot] = (0, 0);
                    self.pointer_window_ms[slot] = 0;
                }
            }

            // The knob comes before the keys. A detent is one event with no hold
            // state to settle, so putting it behind a busy matrix would only make
            // it arrive late.
            //
            // One event per pass, like the keys below: RMK calls this in a loop and
            // takes each call as one action, so three detents are three reports of
            // a turn - which is what makes the volume move three steps rather than
            // one.
            if ENCODER_ENABLED {
                // Strictly alternate press and release, the way RMK's own encoder
                // device does. The release has to be checked FIRST: reporting a
                // second detent while the first is still open overwrites the
                // direction, so that first press is never closed out - and a
                // position left down does not just make a knob misbehave, it fills
                // RMK's held buffer until every later key is buffered as well. That
                // is a dead keyboard, recoverable only by a restart.
                for slot in 0..HALVES {
                    if self.encoder_reported[slot] != Direction::None {
                        let direction = self.encoder_reported[slot];
                        self.encoder_reported[slot] = Direction::None;
                        // RMK's own encoder waits between the two halves of a
                        // detent, so the press and its release are never handled in
                        // the same instant.
                        embassy_time::Timer::after_millis(5).await;
                        return GazellEvent::Keyboard(KeyboardEvent::rotary_encoder(
                            slot as u8,
                            direction,
                            false,
                        ));
                    }
                    if self.encoder_pending[slot] != 0 {
                        let clockwise = self.encoder_pending[slot] > 0;
                        self.encoder_pending[slot] -= self.encoder_pending[slot].signum();
                        self.encoder_reported[slot] = if clockwise {
                            Direction::Clockwise
                        } else {
                            Direction::CounterClockwise
                        };
                        return GazellEvent::Keyboard(KeyboardEvent::rotary_encoder(
                            slot as u8,
                            self.encoder_reported[slot],
                            true,
                        ));
                    }
                }
            }

            // Produce events exactly the way RMK's own matrix device does: keep
            // per-key state and let RMK's debouncer decide when a change is
            // real, one event per pass. That debouncer is the fast one, so the
            // pass that first sees a change is the pass that publishes it - see
            // the field above for why the leading edge is committed here.
            for slot in 0..HALVES {
                for row in 0..ROWS_PER_HALF {
                    let lrow = logical_row(slot, row);
                    let value = self.raw[lrow];
                    for col in 0..COL {
                        let pressed = value & (1 << col) != 0;
                        if let DebounceState::Debounced = self.debouncer.detect_change_with_debounce(
                            lrow,
                            col,
                            pressed,
                            &self.key_states[col][lrow],
                        ) {
                            self.key_states[col][lrow].toggle_pressed();
                            return GazellEvent::Keyboard(KeyboardEvent::key(
                                lrow as u8,
                                col as u8,
                                self.key_states[col][lrow].pressed,
                            ));
                        }
                    }
                }
            }

            // ---- the pointing device --------------------------------------------
            // Last, not first. A keypress is discrete and the keymap holds one
            // place for it; a pointer position is a snapshot that keeps until the
            // next event. Deferring the stick by one pass costs nothing, deferring
            // a keypress is a typo.
            //
            // The whole owed displacement goes in one event, the way the BLE
            // firmware did it, rather than a count at a time: the receiver's job is
            // to hand over what moved, and the pointing pipeline already knows how
            // to smooth a stroke.
            for slot in 0..HALVES {
                let (dx, dy) = self.pointer_pending[slot];
                if dx == 0 && dy == 0 {
                    continue;
                }
                self.pointer_pending[slot] = (0, 0);
                let window_ms = core::mem::take(&mut self.pointer_window_ms[slot]);
                // Sensitivity and the curve, applied here rather than on the half
                // that measured the interval, so that a keymap cell can change them.
                //
                // Both devices have one now. They do not share it: the pad's factor
                // is 1.0 against the nub's 1.30, because the two count in different
                // currencies - see `pointer_accel`. The pad's fallback cap is also
                // 1.0, so a pad with no curve chosen passes through untouched.
                let (dx, dy) = crate::pointer_accel::apply(slot as u8, dx, dy, window_ms);
                return GazellEvent::Pointing(PointingEvent {
                    // A half's slot IS its device id: 0 for the left pad, 1 for the
                    // right stick. The same numbers the BLE firmware used, so the
                    // processors and the keymap's mouse layer stay as they were.
                    device_id: slot as u8,
                    axes: [
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::X,
                            value: dx,
                        },
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::Y,
                            value: dy,
                        },
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::Z,
                            value: 0,
                        },
                    ],
                });
            }

            embassy_time::Timer::after_millis(POLL_MS).await;
        }
    }
}