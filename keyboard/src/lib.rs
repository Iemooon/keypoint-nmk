//! KeyPoint 2.4GHz transmitter half.
//!
//! What one half of this keyboard does, in full:
//!
//!   1. scan its own 6x8 matrix - at `scan_hz` while keys are being used, at
//!      `idle_poll_ms` once nothing has been pressed for `active_release_ms`,
//!   2. transmit the snapshot over Nordic Gazell whenever the state has been
//!      stable for a few ticks (and keep re-sending it while it is stable),
//!   3. re-initialise the link if a packet stops making progress through the queue.
//!
//! There is no keymap here, no layers, no Vial, no USB and no BLE. The keyboard's
//! brain lives in the receiver (the sibling `../receiver` crate) - which is also true of
//! the original design this follows (`redox-w-keyboard-basic` transmits, the
//! receiver holds the keymap) and of RMK's own split topology (the central holds
//! the keymap). Keeping that split means a half stays a small, auditable piece of
//! firmware whose idle behaviour is a handful of lines rather than a scheduler's
//! property.
//!
//! Idle behaviour: shallow sleep, and why
//! --------------------------------------
//! The original transmitter (and the first version of this one) enters SYSTEMOFF
//! after half a second of inactivity. Measured on this keyboard, that was wrong,
//! and not only because of the delay:
//!
//!   * SYSTEMOFF is exited by a RESET, so the next key press paid for a full
//!     restart - MBR, the UF2 bootloader, clock setup, radio init, re-acquiring
//!     the host - tens of milliseconds before this file ran again;
//!   * the matrix is sampled as a LEVEL, not captured as an event. A tap shorter
//!     than that restart was not delayed, it was LOST: by the time the chip was
//!     running, the key had already been released, and no scan ever saw it. That
//!     is exactly "I have to press a key several times", and it only ever
//!     happened after a pause - fast typing never noticed anything, because the
//!     chip was already awake.
//!
//! So the chip now stays in System ON and simply stops being busy. What was
//! expensive was never the idle state (~1 uA for SYSTEMOFF against ~2-3 uA for
//! System ON with RAM retained) - it was scanning at 1 kHz, worth tens of uA.
//! Dropping the scan rate takes the idle current to the same order as SYSTEMOFF,
//! while a press is still caught within one poll period.
//!
//! What the original's reset also did, and had to be replaced
//! ---------------------------------------------------------
//! There is a second thing a power cycle on the way in and out of idle bought the
//! original firmware, and it is easy to miss: every return from idle re-ran the
//! entire Gazell initialisation sequence. The link therefore never had to survive
//! anything for long - whatever state the library accumulated (queue full, sync
//! lost, phase offset against the host) was thrown away at the next pause, and the
//! original could simply never exhibit a stuck link as a *permanent* condition.
//!
//! This firmware keeps the chip alive across idle periods, so nothing throws that
//! state away, and the measured symptom of that difference is blunt: after roughly
//! 30-40 keypresses on one side, that half stops producing keypresses entirely,
//! keeps the receiver showing whatever was held at the moment it stopped, and does
//! not come back until the power is switched off and on. `gazell::reset` below is
//! the replacement: when the radio refuses packets for a full second, the half
//! re-runs the same initialisation a reset would, without losing the key state
//! held in RAM.

#![no_std]

pub mod board;
pub mod dfu;
pub mod gazell;
pub mod screen;

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use embassy_time::{Duration, Instant, Timer};

use board::{
    ACTIVE_RELEASE_MS, COLS, DEBOUNCE_TICKS, IDLE_POLL_MS, KEEPALIVE_MS, MATRIX_BYTES,
    OFF_ENCODER, OFF_GAP, OFF_POINTER_MODE, OFF_POINTER_X, OFF_POINTER_Y, OFF_SEQ, PAYLOAD_LENGTH,
    POINTER_POWER_KEY_LOCKOUT_MS, RADIO_OFF_WHEN_IDLE, ROWS, Role, SCAN_HZ, SCAN_PERIOD_US,
    key_down,
};

/// How long a packet may sit in the TX queue before the link is called stuck.
///
/// One packet waiting is ordinary: it is waiting for the host to offer this pipe
/// a slot. What is not ordinary is waiting with no callback at all, because the
/// library reports both outcomes - a packet that got through, and a packet it has
/// given up on. A queue that stays occupied for this long *while saying nothing*
/// is therefore not a slow link but a stopped one, and only that case is rebuilt.
///
/// ---- 2026-09-17: 120 ms -> 500 ms, to match the original's cleanup period ----
///
/// This backstop has no counterpart in redox-w-keyboard-basic, which never
/// inspects its queue and never re-initialises anything at runtime. That is not
/// because the original is braver; it is because the original cleans the link by
/// resetting the chip:
///
///     #define INACTIVITY_THRESHOLD 500 // 0.5sec
///     ...
///     NRF_POWER->SYSTEMOFF = 1;
///
/// Half a second after the last key, the original powers off and comes back
/// through a full reset, which re-runs the entire Gazell initialisation. Any
/// wedged library state therefore has a lifetime of at most 0.5 s. This board
/// cannot do that - SYSTEMOFF on an nRF52840 with a UF2 bootloader costs tens of
/// milliseconds of restart, and because the matrix is sampled as a level rather
/// than captured as an event, a short tap during that window is lost outright - so
/// it stays in System ON and nothing ever clears the library's state. This
/// backstop is the translation of the original's cleanup into that architecture,
/// and 500 ms is the original's period.
///
/// The previous value, 120 ms, was tuned by feel: it was shortened from 300 to 150
/// to 120 because each reduction measurably shortened the stall the typist felt.
/// That is now understood to have been measuring the wrong thing. A backstop that
/// fires often is not a fast recovery, it is a link that keeps needing recovery -
/// and each firing costs a re-initialisation and a re-acquisition, which is itself
/// a gap in the packet stream. Tuning it down was making the symptom shorter and
/// the cause more frequent.
const QUEUE_STUCK_MS: u32 = 500;

// There is deliberately no separate "repeat rate" constant here.
//
// An earlier revision had one (`REPEAT_TICKS = 20`), on the argument that
// re-offering an unchanged state every 5 ms - 200 offers a second - was more than
// the link could carry, and that the repeats crowded out the states that had
// actually changed. That argument was wrong about the mechanism, and the shipped
// firmware is the proof: redox-w-keyboard-basic has a single constant for both
// cases,
//
//     #define DEBOUNCE 5
//     ...
//     if (debounce_ticks == DEBOUNCE) {
//         nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, keys_snapshot, ROWS);
//         debounce_ticks = 0;
//     }
//
// so it re-offers an unchanged state every 5 ms too, unconditionally, and it does
// not stutter. What actually crowded the queue was not the offer rate: it was a
// link that kept failing to drain, for reasons that lived in the Gazell parameters
// (see board.toml). Throttling the repeats treated the symptom and, worse, made
// the packet stream sparse enough that the receiver's gap-based measurements could
// not tell a quiet link from a stalled one.
//
// So `debounce_ticks` is the whole story again: a change is sent once the state
// has been stable for that many ticks, and an unchanged state is re-sent every
// that many ticks after it.

// There is deliberately no "link lost" detector here.
//
// An earlier revision had one (`LINK_LOST_MS = 500`): if nothing had been
// acknowledged for half a second, offer the current state again as a probe, and if
// that also went unanswered, re-initialise the link. It was added to break a real
// deadlock - a half that lost sync while nobody was typing sends nothing, and a
// half that sends nothing can never re-acquire the host, so it stayed silent until
// the power was cycled.
//
// The deadlock only exists if the packet stream stops when the keys do, and it no
// longer does: `debounce_ticks` re-offers the unchanged state every 5 ms for as
// long as the half is scanning, which is precisely what the original transmitter
// does. A keyboard with nothing held is still putting a packet on the air every
// few milliseconds, so a half that has lost sync goes looking for the host with
// traffic in hand rather than in silence. The probe was therefore fixing a problem
// this file had created for itself by throttling the repeats (see the note where
// `REPEAT_TICKS` used to be), and its second half - re-initialising after a probe
// went unanswered - interrupted a search the library was still entitled to finish.
//
// What is left for a genuinely wedged library is `QUEUE_STUCK_MS` above, which
// fires on a queue that is occupied *and* silent.

/// Bring LFCLK into a state `embassy_nrf::init` can cope with, with the wait
/// bounded. Must run before `embassy_nrf::init`.
///
/// This half is entered by a jump from the Adafruit UF2 bootloader, which has
/// already started LFCLK for its own purposes. That breaks embassy's
/// unconditional wait: per the nRF52840 PS, `TASKS_LFCLKSTART` has *no effect -
/// including no event* when the clock is already running, and embassy would spin
/// on `EVENTS_LFCLKSTARTED` forever. Stopping LFCLK first turns embassy's start
/// into a real 0 -> 1 edge, and the stop itself is waited for because a start
/// issued during it is ignored.
///
/// Deliberately NOT done here: starting HFXO. This firmware has no USB and no
/// BLE, embassy-nrf's default `hfclk_source` is the internal RC, and Gazell runs
/// with `xosc_ctl = auto`, i.e. it powers the 32 MHz crystal itself when it needs
/// it. Starting it here - which the receiver must do, for USB - would add
/// 0.4-1 ms to every boot for nothing.
fn prepare_clocks() {
    use embassy_nrf::pac::clock::vals::Lfclksrc;

    let clock = embassy_nrf::pac::CLOCK;

    clock.tasks_lfclkstop().write_value(1);
    let mut spins: u32 = 0;
    while clock.lfclkstat().read().state() {
        spins += 1;
        if spins > 20_000_000 {
            break;
        }
    }
    clock.lfclksrc().write(|w| w.set_src(Lfclksrc::Rc));
}

/// Clocks, regulators and peripheral ownership, in the order they matter.
///
/// Regulator modes are NOT the chip's reset defaults when a UF2 bootloader hands
/// over: the Adafruit bootloader switches REG1 to DC/DC before jumping into the
/// application, and DC/DC needs the external inductor that not every board
/// revision populates. A firmware that never touches the register therefore runs
/// in a mode it never chose. `[power] reg1` in board.toml decides: "keep"
/// (default) leaves it alone, "ldo" / "dcdc" force one.
pub fn init_peripherals() -> embassy_nrf::Peripherals {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P3;
    config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P3;

    if board::REG1_FORCE_LDO {
        embassy_nrf::pac::POWER.dcdcen().write(|w| w.set_dcdcen(false));
    } else if board::REG1_FORCE_DCDC {
        embassy_nrf::pac::POWER.dcdcen().write(|w| w.set_dcdcen(true));
    }

    prepare_clocks();
    embassy_nrf::init(config)
}

/// The stick's cumulative displacement, or zero for a half that carries no pointer.
fn pointer_position(pointer: &Option<board::Pointer>) -> (i16, i16) {
    pointer.as_ref().map(|p| p.position()).unwrap_or((0, 0))
}

/// Whether this pass carries a fresh press of the switch key `key`.
    ///
    /// An EDGE, not a level, and debounced by a lockout rather than by the report's
    /// own debounce. The key is read from the raw scan, whose whole purpose is to see
    /// contact bounce, so a level test would toggle several times per press; and the
    /// report's debounce is about what goes on the air, not about what the loop acts
    /// on. Two deliberate presses are two toggles, which is what a latching switch has
    /// to promise.
    fn took_press(
        keys: &[u8; MATRIX_BYTES],
        key: (usize, usize),
        was_down: &mut bool,
        locked_until: &mut Instant,
        now: Instant,
    ) -> bool {
        let down = key_down(keys, key.0, key.1);
        let fresh = down && !*was_down && now >= *locked_until;
        if fresh {
            *locked_until =
                now + embassy_time::Duration::from_millis(POINTER_POWER_KEY_LOCKOUT_MS as u64);
        }
        *was_down = down;
        fresh
    }

    /// Fill in a report from the things this half knows, and hand it to the radio.
///
/// The layout is `board.rs`'s. Everything goes through here so that adding a field
/// means changing one function instead of every call site.
fn send_report(
    pipe: u32,
    role: Role,
    matrix: &[u8; MATRIX_BYTES],
    encoder_count: u8,
    pointer: (i16, i16),
    scroll: bool,
    report: &mut [u8; PAYLOAD_LENGTH],
) {
    // The two switch keys are dropped here, on the way onto the air. They are
    // read from the raw scan a few lines up and drive the power and mode latches
    // from there, so nothing about their function changes; what changes is that
    // the host never hears about them.
    //
    // This matters because the receiver cannot tell a switch cell from any other
    // cell - it looks the position up in its keymap like everything else, and the
    // key that comes out is whatever the layout says. When those cells held a
    // spare `No` the press was already harmless; when the layout has something
    // there - and the default keymap did, left over from the diagnostic build -
    // the host gets a keystroke nobody asked for.
    //
    // Clearing in the transmitter is the only place this can be fixed, because
    // the link runs one way: nothing downstream can be told to ignore a cell.
    let mut wired = *matrix;
    for key in [
        role.pointer_power_key(),
        role.pointer_mode_key(),
    ]
    .into_iter()
    .flatten()
    {
        board::clear_bit(&mut wired, key.0, key.1);
    }
    report[..MATRIX_BYTES].copy_from_slice(&wired);
    report[OFF_ENCODER] = encoder_count;
    // Cumulative and little-endian, like the knob's count and the matrix bitmap: the
    // receiver takes differences between packets, so one lost in the air costs
    // nothing at all.
    report[OFF_POINTER_X..OFF_POINTER_X + 2].copy_from_slice(&pointer.0.to_le_bytes());
    report[OFF_POINTER_Y..OFF_POINTER_Y + 2].copy_from_slice(&pointer.1.to_le_bytes());
    // A setting rather than a measurement, so it is simply restated every time. That
    // is also what makes the two ends agree after either of them restarts.
    report[OFF_POINTER_MODE] = if scroll { 1 } else { 0 };
    // How long it has been since the previous report left this half, in
    // milliseconds. This is the interval the displacement above covers, and it is
    // the clock the receiver's acceleration curve runs on - see `OFF_GAP` for why
    // the answer has to be measured here.
    //
    // Taken in this function rather than at each call site: every report carries the
    // field whether or not it has anything in the pointer slots, and two call sites
    // that disagreed about when the clock restarted would make the interval a
    // fiction.
    //
    // Saturating at 255 rather than wrapping. The value is a duration, and a half
    // that has been idle has a genuinely long gap on its first packet; wrapping
    // would turn minutes into a few milliseconds and read as a flick. Clamping
    // makes it read as "slow", which is what the packet actually was.
    static LAST_REPORT_MS: AtomicU32 = AtomicU32::new(0);
    let now_ms = embassy_time::Instant::now().as_millis() as u32;
    let last_ms = LAST_REPORT_MS.swap(now_ms, Ordering::Relaxed);
    report[OFF_GAP] = (now_ms.saturating_sub(last_ms)).min(255) as u8;
    // The first packets after a reset say so, in the one byte of this layout that
    // carries nothing else yet.
    //
    // The receiver reads the knob as the difference of two cumulative counts, which
    // is what makes a lost packet free - but a half that restarts begins its count
    // again at zero, and the receiver still holds the count from the last session.
    // The first packet of the new session then differs by however far the knob went
    // in the old one, and the host's volume moves by that much. Nothing about the
    // timing separates the two cases: the transmitters fall silent when idle, so an
    // ordinary pause looks exactly like a power cycle.
    //
    // Counting packets rather than milliseconds is the point. A half that boots and
    // sits idle sends nothing at all, so a time window would expire long before the
    // first packet went out; counting sends keeps the marker on until the receiver
    // has certainly seen it.
    static PACKETS_SENT: AtomicU32 = AtomicU32::new(0);
    let boot_marker = PACKETS_SENT.fetch_add(1, Ordering::Relaxed) < BOOT_MARKER_PACKETS;
    report[OFF_SEQ] = if boot_marker { BOOT_MARKER } else { 0 };
    gazell::send(pipe, report);
}

/// How many packets after a reset announce it.
///
/// Sixteen sends at the 5 ms cadence is 80 ms of contact with the receiver, and the
/// marker is repeated in every one of them, so a lost packet does not lose the news.
/// Until then the receiver adopts the half's counters as a baseline instead of
/// differencing them against the previous session.
const BOOT_MARKER_PACKETS: u32 = 16;

/// What the marker looks like in the packet's sequence byte.
///
/// Any non-zero value would do; this one is unlikely to appear by accident if that
/// byte is ever given a real meaning.
const BOOT_MARKER: u8 = 0xA5;

/// Run the half. Never returns: there is no power-down path any more, so the
/// loop simply runs for as long as the board has power.
pub async fn run(
    role: Role,
    mut pins: board::Matrix,
    mut encoder: board::Encoder,
    mut pointer: Option<board::Pointer>,
) -> ! {
    let pipe = role.pipe();
    let channel_table = role.channel_table();
    gazell::init(pipe, channel_table);
    info!(
        "tx {}: {}x{} matrix, pipe {}, {} Hz scan / {} ms idle poll, release after {} ms, \
         radio off when idle: {}",
        role.name(),
        ROWS,
        COLS,
        pipe,
        SCAN_HZ,
        IDLE_POLL_MS,
        ACTIVE_RELEASE_MS,
        RADIO_OFF_WHEN_IDLE
    );

    let mut snapshot = [0u8; MATRIX_BYTES];
    // One half's report: the matrix, then the encoder and the pointing device.
    // Only the matrix is filled in so far - the trailing fields stay zero until
    // the devices behind them are read.
    let mut report = [0u8; PAYLOAD_LENGTH];
    let mut debounce: u32 = 0;
    let mut radio_on = true;
    let mut idle = false;
    let mut queue_busy_ms: u32 = 0;
    // Link-health bookkeeping: both callback counts as of the last time round the
    // loop. A queue that stays occupied while these move is a library working
    // through a backlog; one that stays occupied while these do not move is a
    // library that has stopped - see QUEUE_STUCK_MS.
    let mut progress_snapshot = gazell::progressed();
    // When anything last went onto the air, whatever put it there. Read only by the
    // idle keepalive, which asks how long the link has been quiet - see KEEPALIVE_MS.
    let mut last_offer = Instant::now();
    // When the matrix and the stick were last read.
    //
    // Timestamps, not pass counts. A pass is no longer a fixed length: the loop
    // wakes on its own tick OR on a knob edge OR on the TrackPoint's data-ready
    // line, so counting passes would make "10 ms" mean anywhere between ten and a
    // thousand wake-ups depending on what happened to be moving.
    let mut last_scan = Instant::now();
    // The stick keeps its own slower beat; see where it is polled below.
    let mut last_pointer = Instant::now();

    // --- the panel: when it is allowed to be touched, and what it shows -------
    //
    // MEASURED, 2026-09-18: a transfer that lands at the START of a typing burst
    // costs keys. The build that painted only on the active/idle transitions -
    // i.e. exactly at the edges of a burst - still dropped keys, occasionally
    // rather than always. So the damage is done at the instant a burst begins,
    // and the only transfers ever observed to be harmless are the ones that happen
    // well inside a stretch of quiet: the boot frame is painted with the radio idle
    // and cost nothing.
    //
    // So the panel is only ever touched once this half has been quiet for
    // QUIET_FOR_PAINT_MS. The screen is consequently useless exactly while you are
    // typing, which is the price of a keystroke - and keystrokes win.
    //
    // Two things are told to the panel, and they differ in exactly the way that
    // matters: the state line goes out the moment it changes (a switch key being
    // pressed, i.e. one band of 28 lines rather than a 144-line frame),
        // everything else waits for the quiet window below.
    let mut last_active = Instant::now();
    // Whether the quiet stretch we are in has been painted already.
    //
    // NOT "is the screen showing the right state": that comparison never fires
    // here, because by the time the quiet window is satisfied the half has long
    // been considered idle again and the state has settled back to exactly what was
    // last painted. What is being detected is the EDGE "a typing session ended",
    // and one paint per edge keeps the animation alive without any transfer ever
    // starting near input.
    let mut quiet_painted = false;

    // What the panel was last told about the pointing device, so a state change is
    // worth exactly one band repaint. Starts at `Off` to match both the boot frame
    // and the loop's own starting state above.
    let mut last_pointer_ui = screen::PointerUi::Off;

    // ---- the pointing device's own supply ---------------------------------
    //
    // Two ways it goes off, and they are independent on purpose:
    //
    //   * the key board.toml names. That one is a decision - "I am leaving, stop
    //     paying for the nub" - and it stays in force until the same key is pressed
    //     again. It is NOT undone by typing, or the next keystroke would undo it;
    //   * the idle timeout. That one is a policy: after `idle_off_ms` with no input
    //     at all the device is switched off, and the next key or knob turn brings it
    //     straight back, because the quiet clock is reset by exactly that activity.
    //     Being undone by typing is the point - the device is simply not powered
    //     while nobody is using it, which is what the drain complaint asks for.
    //
    // Boot starts powered. Nothing is remembered across a reset: this half has no
    // flash, and the question is worth asking again after every power cycle anyway.
    // Boot starts UNPOWERED. That is the ask (cold boot is a keyboard, not a
    // pointer), and it also means the state line's first frame is honest: the main
    // loop's opening pass sees `manual_on == false` against `pointer_power_on == true`
    // and switches the rail off before anything can be read from the device.
    let power_key = role.pointer_power_key();
    let mode_key = role.pointer_mode_key();
    let pointer_idle_off_ms = role.pointer_idle_off_ms();
    let mut manual_on = false;
    let mut power_key_was_down = false;
    let mut power_key_locked_until = Instant::now();
    let mut pointer_power_on = true;
    // What this half's pointing device means, latched by its own key and restated in
    // every packet. Cursor at boot: this half has no flash, and the receiver starts
    // there too, so the two agree without anyone having to remember anything.
    let mut pointer_scroll = false;
    let mut mode_key_was_down = false;
    let mut mode_key_locked_until = Instant::now();

    // The bootloader gesture. Owns nothing but its own state: which four cells, and
    // how far through the sequence it is - see `dfu`.
    let mut dfu = dfu::Combo::new(role);

    let mut wake = board::Wake::Tick;

    loop {
        let now = Instant::now();

        // ---- the knob: sampled on EVERY wake-up -----------------------------
        // Every wake-up means every knob edge too, which is the point: a detent is
        // four quadrature steps and a quick hand covers them in a few milliseconds,
        // so a knob sampled only on the loop's own tick drops the step after every
        // pause. Waiting on the contacts and reading them the moment either moves
        // costs nothing while nothing moves, and cannot miss a step while
        // something does.
        let turned = encoder.poll();

        // ---- the matrix: every pass while active, every idle_poll_ms while idle
        let since_scan = (now - last_scan).as_millis() as u32;
        let scan_now = !idle || turned || since_scan >= IDLE_POLL_MS;
        let keys = if scan_now {
            last_scan = now;
            pins.scan()
        } else {
            // The matrix has not been read this pass, so its last reading stands.
            // Reporting it as unchanged is what keeps an idle half from sending.
            snapshot
        };

        // ---- the stick: only when the device says so ------------------------
        // Not on a beat, and that is the point rather than an optimisation - see
        // `Wake` and `Pointer::read_due`. The acceleration curve is a function of
        // the interval between two packets, so a beat here pins that interval to
        // the beat and the curve collapses into a flat multiplier. Reading when the
        // line says there is something keeps the interval a measurement of the
        // device.
        //
        // A failure here is a dropped round and nothing more: `poll` already
        // reports "nothing moved", and the next signal brings it straight back.
        let since_pointer = (now - last_pointer).as_millis() as u32;
        let pointer_due = match pointer.as_ref() {
            Some(dev) => dev.read_due(wake, since_pointer),
            None => false,
        };
        let mut pointer_moved = false;
        if pointer_due {
            last_pointer = now;
            if let Some(stick) = pointer.as_mut() {
                pointer_moved = stick.poll().await;
            }
        }

        let busy = keys.iter().any(|k| *k != 0) || turned || pointer_moved;

        // ---- wake the radio BEFORE anything is queued into it ---------------
        // A packet handed to a disabled Gazell device has nowhere to go, and the
        // re-acquisition that follows a wake-up is measured in milliseconds, not
        // microseconds - so it starts as early as this half knows a key is down.
        if busy {
            // Any activity at all postpones the next transfer, and this timestamp
            // is what the quiet window is measured from. Pointer movement counts
            // too: it is input, and it means a hand is on the keyboard.
            //
            // It is ALSO what the idle decision below is measured from. The two
            // were separate state until 2026-09-28, and the separation was a bug:
            // the other one counted loop PASSES, and a pass is no longer a fixed
            // length now that the loop wakes on knob and pointer edges as well as
            // on its own tick. With a 2 ms idle tick, 500 passes was 1000 ms, so
            // `active_release_ms = 500` was in truth a one-second release - it kept
            // the 1 kHz scan running for twice as long as configured, which is
            // precisely the current this firmware is trying not to spend. The same
            // reasoning already converted `last_scan` and `last_pointer` to
            // timestamps (see the note where they are declared); this one was
            // missed. It is folded into `last_active` rather than given a second
            // timestamp because the two quantities are the same quantity: the
            // moment this half last saw input.
            last_active = now;
            // A fresh burst has begun, so the next quiet stretch is a new edge and
            // will get its own frame.
            quiet_painted = false;
            if !radio_on {
                gazell::set_enabled(true, pipe);
                radio_on = true;
            }
        }
        // A duration now, not a count, so `ACTIVE_RELEASE_MS` means milliseconds.
        // Zero while `busy` holds, because the branch above just set `last_active`
        // to this pass's own `now`. The cast is needed: embassy_time's `as_millis`
        // is u64 (not the u128 the standard library's is) and the generated
        // constant is u32, and Rust will not compare the two unaided.
        idle = (now - last_active).as_millis() >= ACTIVE_RELEASE_MS as u64;

        // ---- the pointing device's own supply -------------------------------
        //
        // Placed after the activity clock has been updated, so a pass that did
        // something is seen as activity by the same test that decides to switch the
        // device off. That is what makes "off after a minute of quiet, back on the
        // moment you touch the keyboard" fall out of one comparison rather than two
        // pieces of state that have to agree.
        if let Some(key) = power_key {
            if took_press(
                &keys,
                key,
                &mut power_key_was_down,
                &mut power_key_locked_until,
                now,
            ) {
                manual_on = !manual_on;
                info!(
                    "pointer power: manual switch -> {}",
                    if manual_on { "on" } else { "off" }
                );
            }
        }
        // The mode switch. Deliberately independent of the power switch: which of the
        // two a hand reaches for depends on what it wants, and neither implies the
        // other.
        if let Some(key) = mode_key {
            if took_press(
                &keys,
                key,
                &mut mode_key_was_down,
                &mut mode_key_locked_until,
                now,
            ) {
                pointer_scroll = !pointer_scroll;
                info!(
                    "pointer mode: {}",
                    if pointer_scroll { "scroll" } else { "cursor" }
                );
            }
        }
        let quiet_ms = (now - last_active).as_millis() as u32;
        let wanted = manual_on && (pointer_idle_off_ms == 0 || quiet_ms < pointer_idle_off_ms);
        if wanted != pointer_power_on {
            pointer_power_on = wanted;
            if let Some(dev) = pointer.as_mut() {
                dev.set_power(wanted);
                info!(
                    "pointer power: {} (device reports {})",
                    if wanted { "on" } else { "off" },
                    if dev.power_on() { "on" } else { "off" }
                );
            }
        }

        // ---- the bootloader gesture -----------------------------------------
        // Asked here, between everything that has a say in what goes out and the
        // sending itself: the gesture reads the raw scan - which is how it sees a
        // four-cell press the moment it happens, without waiting for the report's own
        // debounce - and then takes over the reporting entirely for the few packets
        // it needs.
        //
        // `keys` is this pass's scan, or the last one while idle. It cannot be stale
        // in a way that matters here: the cells have to be down for the gesture to
        // land, and a half with cells down is never idle.
        let dfu_step = dfu.step(&keys, gazell::progressed().0, now);
        if dfu_step == dfu::Step::Jump {
            dfu::jump(role);
        }

        // ---- send -----------------------------------------------------------
        // Whether this pass put anything on the air. The idle keepalive below
        // measures its own interval from the last pass that did.
        let mut offered = false;
        //
        // The state is sent once it has held still for `DEBOUNCE_TICKS`, and then
        // kept being sent every `DEBOUNCE_TICKS` while it stays the same. The
        // repetition is not redundant: it is what keeps the receiver's link alive
        // while a key is held (the receiver releases a half whose packets stop
        // arriving) and what repairs a packet lost in either direction. The
        // original transmitter does exactly this.
        //
        // Nothing is sent while idle with no keys down: an "all keys released"
        // packet that has already been delivered carries no information.
        if dfu_step == dfu::Step::Release {
            // Nothing down, once per report cadence, until the half is gone. The
            // receiver holds a half's keys until its 500 ms link timeout, so this is
            // what stops the four corner cells from being held after the reset - and
            // what makes the gesture look, from the host's side, like four keys
            // tapped and released.
            send_report(
                pipe,
                role,
                &dfu::NOTHING_DOWN,
                encoder.count(),
                pointer_position(&pointer),
                pointer_scroll,
                &mut report,
            );
            offered = true;
        } else if dfu_step == dfu::Step::Silent {
            // Between two of those reports: nothing at all goes out, because the
            // ordinary report would carry the four cells that are still physically
            // down and undo the release that was just sent.
        } else if turned || pointer_moved {
            // A turn or a cursor move goes out at once, without waiting for the
            // debounce below.
            //
            // That debounce exists to swallow contact bounce in the MATRIX, where
            // one noisy reading is an entire phantom keypress. Neither of these
            // needs it: the knob already refuses to count a detent until four
            // quadrature transitions have completed, and a stick packet has been
            // checked against the bridge's magic byte. Waiting anyway would be
            // worse than pointless - both produce changes every few ticks, so the
            // counter would be reset before it ever expired and every step of a
            // continuous turn or stroke would be held back until it stopped.
            //
            // `snapshot` is deliberately NOT updated here: the matrix has not
            // changed, and pretending it had would skip the debounce for a press
            // that happened to land on the same tick.
            send_report(
                pipe,
                role,
                &snapshot,
                encoder.count(),
                pointer_position(&pointer),
                pointer_scroll,
                &mut report,
            );
            offered = true;
        } else if !idle || busy {
            if keys == snapshot {
                debounce += 1;
                // One interval for both cases, exactly as in the original transmitter:
                // a change goes out once it has been stable for `debounce_ticks`,
                // and a state that has not changed goes out again every
                // `debounce_ticks` after that. There is no separate repeat rate and
                // no separate path for repeats - see the note where REPEAT_TICKS
                // used to be.
                //
                // A refusal is not remembered as "sent", so the same snapshot is
                // offered again on the next tick. That is what makes it safe to
                // offer unconditionally: nothing is lost by an offer the library
                // had no room for, and nothing is gained by withholding one.
                if debounce >= DEBOUNCE_TICKS {
                    debounce = 0;
                    send_report(
                        pipe,
                        role,
                        &snapshot,
                        encoder.count(),
                        pointer_position(&pointer),
                        pointer_scroll,
                        &mut report,
                    );
                    offered = true;
                }
            } else {
                debounce = 0;
                snapshot = keys;
            }
        } else {
            // ---- idle, nothing held: the keepalive ---------------------------------
            //
            // The reference transmitter is still offering its snapshot at this
            // point - its `handle_send` runs on every tick and an unchanged state is
            // re-sent every DEBOUNCE ticks whatever it says (see KEEPALIVE_MS in
            // board.toml). This one stopped here, and a half that stops is out of
            // sync 27 ms later, so the first key after a pause paid for a search.
            // This branch is the reference's stream at a slower rate: the same
            // packet with the same unchanged state, often enough that the library
            // never decides the device has been lost.
            //
            // A packet the library refuses is not remembered as sent, so the next
            // pass offers it again 2 ms later (idle_poll_ms). A refusal means the
            // queue is full, i.e. the link is demonstrably moving anyway.
            debounce = 0;
            if KEEPALIVE_MS > 0
                && !RADIO_OFF_WHEN_IDLE
                && (now - last_offer).as_millis() as u32 >= KEEPALIVE_MS
            {
                send_report(
                    pipe,
                    role,
                    &snapshot,
                    encoder.count(),
                    pointer_position(&pointer),
                    pointer_scroll,
                    &mut report,
                );
                offered = true;
            }
        }
        if offered {
            last_offer = now;
        }

        // ---- link health ---------------------------------------------------
        // Two questions, both answered from the library's own callbacks: a packet
        // is only believed sent when a callback says so, and `gazell::send` merely
        // hands the state over.
        //
        // Nothing here acts on a packet that is merely taking a while. A packet is
        // attempted at most `max_tx_attempts` times, one timeslot each, so up to
        // ~90 ms of work before the library reports either outcome; the failure
        // worth catching is a link that has stopped, not one that is working.
        // Cutting that work short only restarts it, and a half that restarts its
        // search every 80 ms never finishes one - which is what an earlier version
        // of this file did, and why it went quiet after a few characters.
        // A pass is one scan period, idle or not - the tick no longer stretches,
        // so the elapsed time is one millisecond wherever this is used.
        let step_ms = 1;

        let progressed = gazell::progressed();
        let moved = progressed != progress_snapshot;
        if moved {
            progress_snapshot = progressed;
        }

        // (a) The library is holding a packet and has said nothing about it for
        //     far longer than any search takes: it is not searching, it has
        //     stopped. A callback during the wait clears the clock - the library
        //     working through a backlog is slow, not dead, and gets left alone.
        if gazell::queued(pipe) {
            queue_busy_ms = queue_busy_ms.saturating_add(step_ms);
            if queue_busy_ms >= QUEUE_STUCK_MS {
                if moved {
                    queue_busy_ms = 0;
                } else {
                    info!(
                        "gzll: packet held {} ms with no callback, re-initialising the link",
                        queue_busy_ms
                    );
                    gazell::reset(pipe, channel_table);
                    queue_busy_ms = 0;
                    progress_snapshot = gazell::progressed();
                }
            }
        } else {
            queue_busy_ms = 0;
        }

        // (b) is gone. There used to be a second link-health check here - nothing
        //     acknowledged for LINK_LOST_MS, so offer a probe, and re-initialise if
        //     that went unanswered too. See the note where the constant used to be:
        //     the 5 ms repeat stream makes it unnecessary, and both of its actions
        //     (an extra packet, and an unrequested re-initialisation) were things
        //     the original transmitter never does.

        // ---- the panel: touched only in a quiet window ----------------------
        //
        // One store when a repaint is wanted, and nothing else; the painting itself
        // happens in `screen::task`. The CONDITION is what matters here - no
        // transfer is started unless this half has been completely quiet for
        // seconds, so a burst never sees one. See the declaration above for the
        // measurement that forces this.
        //
        // The signal now carries the pointing device's state; see `screen::Request`
        // for why that one thing is allowed to break the quiet rule. Everything else
        // on the panel is still only composed in a quiet window, so nothing that
        // changes WITH activity - an activity word, a layer, a link state - is drawn
        // and could not be drawn truthfully. What is left is what does not move while
        // you type: the animation, the battery, and the state of a switch you pressed
        // yourself.
        // The state line, the moment it changes - see the note above on why this one
        // breaks the quiet rule. Both `pointer_power_on` and `pointer_scroll` are
        // final by now: the switch handling above has already run this pass.
        let pointer_ui = if !pointer_power_on {
            screen::PointerUi::Off
        } else if pointer_scroll {
            screen::PointerUi::Scroll
        } else {
            screen::PointerUi::Move
        };
        if pointer_ui != last_pointer_ui {
            last_pointer_ui = pointer_ui;
            screen::UI.signal(screen::Request {
                pointer: pointer_ui,
                full: false,
            });
        }

        if !quiet_painted && (now - last_active).as_millis() as u64 >= screen::QUIET_FOR_PAINT_MS
        {
            quiet_painted = true;
            screen::UI.signal(screen::Request {
                pointer: pointer_ui,
                full: true,
            });
        }

        // ---- drop back to low power ----------------------------------------
        if idle && radio_on && RADIO_OFF_WHEN_IDLE {
            gazell::set_enabled(false, pipe);
            radio_on = false;
        }

        // ---- park until something needs doing -------------------------------
        // Three things bring the loop back: its own tick, a knob contact moving, or
        // the TrackPoint's data-ready line. Whichever comes first.
        //
        // The tick is there because a matrix has no way to announce a press - it has
        // to be driven and read - so something still has to time the scan. What the
        // other two buy is that the tick is now the ONLY thing that fires when
        // nothing is going on, and while idle it is the idle_poll_ms one. The core
        // spends the gap in `WFI` instead of coming round a thousand times a second
        // to decide there is nothing to do, and that 1 kHz loop was the single
        // largest idle current this firmware had - more than the pointer polling and
        // the matrix scan put together.
        //
        // NOTE, 2026-09-18: this path was bisected out once, on suspicion of
        // causing dropped keys, and the keys kept dropping without it. So the
        // edge wake-ups are not the fault and are back. So is the panel, but with
        // one hard rule attached: a transfer to it must never overlap input. Its
        // frames are composed only in a quiet window, seconds long, once per
        // typing session - see `QUIET_FOR_PAINT_MS` and the note at the top of
        // `screen`.
        //
        // 2026-09-28: one more rule, and it is about current rather than keys. The
        // loop parks on three futures and two of them are GPIO edges, so a pin that
        // generates its own edges makes the park return immediately, forever, at
        // full CPU speed. Nothing in here can distinguish that from a real turn.
        // `SLEEP_FLOOR_US` is slept unconditionally before the race is armed, which
        // bounds the pass rate whatever the pins do - and, because the tick is then
        // parked only for the remainder, leaves the pass length (and so the scan
        // cadence) exactly as it was. See the constant for the full argument,
        // including why the blind window it creates cannot lose an encoder step.
        Timer::after(Duration::from_micros(board::SLEEP_FLOOR_US)).await;

        let tick_us: u64 = if idle {
            IDLE_POLL_MS as u64 * 1000
        } else {
            SCAN_PERIOD_US
        };
        let rest_us = tick_us.saturating_sub(board::SLEEP_FLOOR_US);
        let stick = pointer.as_mut();
        // Recorded for the NEXT pass, which is where it is used: what woke the loop
        // is what decides whether the pointer is read. Nothing between here and the
        // top of the loop looks at anything but this one value, so carrying it
        // forward costs nothing and keeps the reading code in one piece.
        wake = board::wait_any_of(
            Timer::after(Duration::from_micros(rest_us)),
            encoder.wait_edge(),
            async {
                match stick {
                    Some(dev) => dev.wait_motion().await,
                    None => core::future::pending().await,
                }
            },
        )
        .await;
    }
}