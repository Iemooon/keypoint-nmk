//! Nordic Gazell, DEVICE role, in as little code as the library allows.
//!
//! This is the transmitting half of the link whose receiving half lives in the
//! sibling `../receiver` crate. The two must agree on the channel table, the base
//! addresses, the datarate and the timeslot period - each of those is a
//! board.toml entry on both sides, because a mismatch does not produce a
//! degraded link, it produces no link at all.
//!
//! What is deliberately NOT here: anything to do with the ACK payload.
//!
//! Gazell acknowledges every packet at the protocol level - the host sends an
//! ACK, and that is how the device knows whether to retry - but the link is
//! strictly one-way as far as *data* is concerned: no layer number, no display
//! content, no configuration ever travels from host to half. The original
//! firmware used the ACK payload to drive an LED; this keyboard's display is
//! local, so the ACK's payload slot is left empty and nothing in this firmware
//! reads it.
//!
//! Linking the Nordic library
//! --------------------------
//! The archive (nRF5 SDK 17.1.1, `gzll_nrf52840_gcc.a` by default - see
//! board.toml) is linked by build.rs. Its symbol contract was read out with
//! `arm-none-eabi-nm` rather than assumed:
//!
//!   * external references it needs from us - exactly six:
//!       memcpy, memset                              (Rust/compiler_builtins)
//!       nrf_gzll_device_tx_success / _tx_failed     (below)
//!       nrf_gzll_host_rx_data_ready                 (below, a no-op in device role)
//!       nrf_gzll_disabled                           (below, a no-op)
//!   * it defines the three ISRs it needs: TIMER2_IRQHandler, RADIO_IRQHandler,
//!     SWI0_EGU0_IRQHandler. Those are C names; nrf-pac names the interrupts
//!     differently, so the three thunks below bridge them.
//!
//! The four callbacks are defined here even though a device only ever sees two
//! of them: the archive references all four, so all four must exist to link.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::board::{
    BASE_ADDRESS_0, BASE_ADDRESS_1, CHANNEL_SELECTION_POLICY, DATARATE, MAX_TX_ATTEMPTS,
    SYNC_LIFETIME_TIMESLOTS, TIMESLOTS_PER_CHANNEL, TIMESLOTS_PER_CHANNEL_WHEN_OUT_OF_SYNC,
    TIMESLOT_PERIOD_US, XOSC_MANUAL,
};

/// Gazell protocol enumerations - the library's own constants:
///   nrf_gzll_mode_t     : DEVICE = 0, HOST = 1, SUSPEND = 2
///   nrf_gzll_xosc_ctl_t : AUTO = 0, MANUAL = 1
const GZLL_MODE_DEVICE: u32 = 0;
const GZLL_XOSC_CTL_MANUAL: u32 = 1;

// ---------------------------------------------------------------------------
// FFI
// ---------------------------------------------------------------------------

// `nrf_gzll_device_tx_info_t` is 8 bytes, which AAPCS passes via a pointer to a
// caller-made copy - hence `*const u8` rather than a by-value struct. Getting
// this wrong would corrupt the stack, so it is spelled out rather than
// "modelled".
#[allow(improper_ctypes)]
unsafe extern "C" {
    fn nrf_gzll_init(mode: u32) -> bool;
    fn nrf_gzll_enable() -> bool;
    fn nrf_gzll_disable();
    fn nrf_gzll_set_channel_table(channel_table: *mut u8, size: u32) -> bool;
    fn nrf_gzll_set_datarate(data_rate: u32) -> bool;
    fn nrf_gzll_set_timeslot_period(period_us: u32) -> bool;
    fn nrf_gzll_set_base_address_0(base_address: u32) -> bool;
    fn nrf_gzll_set_base_address_1(base_address: u32) -> bool;
    fn nrf_gzll_set_xosc_ctl(xosc_ctl: u32) -> bool;
    // uint16_t in the header, hence the cast at the call site.
    fn nrf_gzll_set_max_tx_attempts(max_tx_attempts: u16);
    fn nrf_gzll_set_timeslots_per_channel(timeslots: u32);
    fn nrf_gzll_set_timeslots_per_channel_when_device_out_of_sync(timeslots: u32) -> bool;
    fn nrf_gzll_set_sync_lifetime(lifetime: u32) -> bool;
    fn nrf_gzll_set_device_channel_selection_policy(policy: u32) -> bool;

    // FIFO management. `flush_tx_fifo` is documented as "not allowed ... when
    // Gazell is enabled", which is why every call below happens after the
    // library has reported itself disabled.
    fn nrf_gzll_is_enabled() -> bool;
    fn nrf_gzll_ok_to_add_packet_to_tx_fifo(pipe: u32) -> bool;
    fn nrf_gzll_flush_tx_fifo(pipe: u32) -> bool;
    fn nrf_gzll_get_tx_fifo_packet_count(pipe: u32) -> i32;

    fn nrf_gzll_add_packet_to_tx_fifo(
        pipe: u32,
        payload: *mut u8,
        length: u32,
    ) -> bool;

    // Provided by the archive; called from the thunks below.
    fn TIMER2_IRQHandler();
    fn RADIO_IRQHandler();
    fn SWI0_EGU0_IRQHandler();
}

/// Storage for the channel table.
///
/// `nrf_gzll_set_channel_table` takes a non-const pointer and keeps it, so the
/// table cannot be a `const` - it has to live somewhere the library may legally
/// read.
///
/// Gazell allows at most 16 channels (NRF_GZLL_CONST_MAX_CHANNEL_TABLE_SIZE),
/// which is what sizes the storage below.
const CHANNEL_TABLE_MAX: usize = 16;

static mut CHANNEL_TABLE_STORAGE: [u8; CHANNEL_TABLE_MAX] = [0; CHANNEL_TABLE_MAX];

/// Configure and enable the radio as this half.
///
/// The call sequence is the original transmitter's, in its order, with nothing
/// added: init, then the two device-side retry/dwell parameters, then the
/// channel table, the datarate, the timeslot period, the two base addresses, and
/// enable. `redox-w-keyboard-basic` sets these and leaves every other Gazell
/// parameter at the library default, so this does too.
pub fn init(pipe: u32, channel_table: &[u8]) {
    assert!(channel_table.len() <= CHANNEL_TABLE_MAX);

    unsafe {
        // Crystal control. "auto" (the default here, and what the original
        // transmitter runs on) lets Gazell power the 32 MHz crystal down between
        // timeslots, which is exactly what is wanted: nothing else in this
        // firmware needs it. It is manual-controllable only because a board with
        // USB on the same crystal has to keep it running - that is the receiver,
        // not this half.
        if XOSC_MANUAL {
            let _ = nrf_gzll_set_xosc_ctl(GZLL_XOSC_CTL_MANUAL);
        }

        let ok = nrf_gzll_init(GZLL_MODE_DEVICE);

        // Device-side link parameters. The host does NOT set these; they only
        // describe how this half retries and how long it dwells on a channel.
        nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS as u16);
        nrf_gzll_set_timeslots_per_channel(TIMESLOTS_PER_CHANNEL);
        // How long it dwells on a channel while *searching* - a different number
        // for a different problem. `TIMESLOTS_PER_CHANNEL` applies once the link is
        // up; this one decides how long a stutter lasts when it is lost, and
        // therefore whether a key pressed during the stutter is sent at all. See
        // board.toml for why this is 15 - the reference's value, after an earlier
        // arithmetic-derived 4 was withdrawn.
        nrf_gzll_set_timeslots_per_channel_when_device_out_of_sync(
            TIMESLOTS_PER_CHANNEL_WHEN_OUT_OF_SYNC,
        );
        // How long "in sync" survives without a successful packet, and where a new
        // packet starts. Both are about the same failure: the library's default sync
        // lifetime (30 timeslots, ~27 ms) is shorter than a single retry is allowed
        // to take (`max_tx_attempts` 100 timeslots, ~90 ms), so one lost packet used
        // to cost the half its place on the channel and send it back to searching.
        //
        // ORDERING IS LOAD-BEARING HERE AND FOR EVERY SETTER ABOVE THIS POINT: each
        // of them returns false once Gazell is enabled, and the return values are not
        // checked, so a call moved below `nrf_gzll_enable()` would fail silently and
        // leave the library on its default. nrf_gzll.h documents this per function
        // ("@retval false If Gazell was enabled").
        //
        // See board.toml for why the value is 500 rather than the default, and for
        // the two ceilings that stop it there - crystal drift between the two ends'
        // timeslot counters, and the fact that this counter is also how long a half
        // stays deaf after the HOST restarts.
        nrf_gzll_set_sync_lifetime(SYNC_LIFETIME_TIMESLOTS);
        nrf_gzll_set_device_channel_selection_policy(CHANNEL_SELECTION_POLICY);

        for (i, c) in channel_table.iter().enumerate() {
            CHANNEL_TABLE_STORAGE[i] = *c;
        }
        let table_ok = nrf_gzll_set_channel_table(
            core::ptr::addr_of_mut!(CHANNEL_TABLE_STORAGE) as *mut u8,
            channel_table.len() as u32,
        );

        nrf_gzll_set_datarate(DATARATE);
        nrf_gzll_set_timeslot_period(TIMESLOT_PERIOD_US);
        nrf_gzll_set_base_address_0(BASE_ADDRESS_0);
        nrf_gzll_set_base_address_1(BASE_ADDRESS_1);

        let enabled = nrf_gzll_enable();

        defmt::info!(
            "gzll: pipe={} init={} table={} enabled={}",
            pipe,
            ok,
            table_ok,
            enabled
        );
    }
}

/// Queue a *changed* snapshot for transmission - and never let it be dropped.
///
/// Returns whether the library took it. There is no separate `send_repeat`: a
/// change and a repeat take the same path, and the difference lives in the caller's
/// cadence (lib.rs re-offers a stable state every `DEBOUNCE_TICKS`).
///
/// **Why a change is not gated on the queue being empty.** The queue holds three
/// packets and the pool behind it six, shared with the other half, and Gazell only
/// acknowledges a packet it had room to store - so a full queue is a real
/// possibility. An earlier revision of this file read that as "do not offer a new
/// state while one is in flight", which is backwards: a state refused at the
/// moment the keys move is a keystroke that was never sent, and since the keys
/// keep moving it never comes back. Dropping a *repeat* costs nothing, because the
/// same state is offered again `debounce_ticks` later; dropping a *change* loses a
/// keypress. So changes always go in, and repeats go in only when there is room.
///
/// The original transmitter does this by not thinking about it - it calls
/// `add_packet` and ignores the result, letting the library refuse what will not
/// fit. That is right for changes, and it is what this function reproduces.
///
/// The caller does not gate on the return value: a refused offer is simply offered
/// again at the next cadence, which is safe because a refusal means the queue is
/// full - i.e. the link is demonstrably moving.
pub fn send(pipe: u32, payload: &mut [u8]) -> bool {
    unsafe {
        if !nrf_gzll_ok_to_add_packet_to_tx_fifo(pipe) {
            return false;
        }
        nrf_gzll_add_packet_to_tx_fifo(pipe, payload.as_mut_ptr(), payload.len() as u32)
    }
}

/// Is a packet still waiting to go out on this pipe?
///
/// One packet waiting is ordinary - it is waiting for the host to offer this pipe
/// a slot. A queue that stays occupied for a long time is the failure mode the
/// caller watches for.
pub fn queued(pipe: u32) -> bool {
    unsafe { nrf_gzll_get_tx_fifo_packet_count(pipe) > 0 }
}

/// Turn the radio off while the half is idle, and back on when a key arrives.
///
/// This is the whole of the "keep listening" question. A disabled device loses
/// sync, so the first packet after re-enabling waits 1..11 ms (one host
/// rotation) to be re-acquired - but it is not lost, the matrix is still being
/// sampled and the packet queues while sync is re-established. What it buys is
/// that the periodic receive current is not paid while nobody is typing.
///
/// The flush on the way down is not decoration, it is the bug fix. A half goes
/// idle immediately *after* offering a state, and that packet is still in the
/// FIFO - it never gets a timeslot, because the radio is off for exactly as long
/// as the half stays idle, and Gazell does not empty the FIFO when it is
/// disabled. So every idle period used to strand one more packet: after three,
/// the FIFO was full, `send` refused every state from then on, and the half went
/// silent until it was power-cycled. That is the "types fine, then suddenly locks
/// up and needs the switch flicked" symptom.
///
/// `nrf_gzll_disable` only *begins* disabling - the library reports completion by
/// calling `nrf_gzll_disabled()` (the no-op below) - and flushing while Gazell is
/// still enabled is explicitly not allowed, so this waits for the library to say
/// it is disabled first. The wait is bounded: spinning for ever inside a keyboard
/// would be a worse bug than the race it avoids.
pub fn set_enabled(on: bool, pipe: u32) -> bool {
    unsafe {
        if on {
            return nrf_gzll_enable();
        }

        nrf_gzll_disable();

        let mut spins: u32 = 0;
        while nrf_gzll_is_enabled() {
            spins += 1;
            if spins > 200_000 {
                break;
            }
        }

        // Legal here, and the point of the whole function: drop anything that
        // was offered but never sent, so the next wake-up starts with an empty
        // queue.
        let _ = nrf_gzll_flush_tx_fifo(pipe);

        true
    }
}

/// Re-initialise the link, doing what a power cycle would do.
///
/// This exists because of what the original transmitter does NOT do: it never
/// touches the link while it runs, and it does not have to, because every return
/// from idle is a chip reset - SYSTEMOFF, MBR, the bootloader, `main()`, and the
/// whole initialisation sequence below all over again. Its link state never lives
/// long enough to get stuck.
///
/// This firmware keeps the chip alive across idle periods, so nothing throws that
/// state away, and the measured consequence is blunt: after roughly 30-40
/// keypresses on one side, that half stops producing keypresses entirely, keeps
/// the receiver showing whatever key was held the moment it stopped, and does not
/// come back until the power is switched off and on. That is a link state the
/// original firmware cannot be caught in, not a bug the original avoids.
///
/// So this is that reset, without the reset: disable the radio (waiting for the
/// library to report itself disabled), flush the queue while flushing is legal,
/// and run the same initialisation sequence `init()` runs at boot. The matrix
/// state lives in RAM and is untouched, so unlike a real power cycle this does not
/// also cost the key that is being pressed at the time.
pub fn reset(pipe: u32, channel_table: &[u8]) {
    unsafe {
        nrf_gzll_disable();

        let mut spins: u32 = 0;
        while nrf_gzll_is_enabled() {
            spins += 1;
            if spins > 200_000 {
                break;
            }
        }

        let _ = nrf_gzll_flush_tx_fifo(pipe);
    }

    init(pipe, channel_table);
}

// ---------------------------------------------------------------------------
// The callbacks the Nordic library requires
// ---------------------------------------------------------------------------

/// The two callbacks a device actually gets: a packet was acknowledged, or it was
/// given up on.
///
/// The original transmitter uses `tx_success` to pull a payload out of the ACK
/// (its LED state); this firmware has no host-to-half data at all, so there is
/// nothing to read. What it does instead is count both, because "has anything
/// produced a callback recently" is the only direct evidence this side has that
/// the link is moving at all - and that is the question the main loop needs
/// answered (see `QUEUE_STUCK_MS` in lib.rs).
///
/// `tx_failed` counts too, and not as an error: it means the library did put the
/// packet on the air and got no acknowledgement. Either callback proves the
/// library reached a decision about a packet, which is what separates "slow" from
/// "stopped". Relaxed atomics are enough - the loop only ever asks whether the
/// number moved.
static TX_SUCCESSES: AtomicU32 = AtomicU32::new(0);
static TX_FAILURES: AtomicU32 = AtomicU32::new(0);

/// How many packets have been acknowledged, and how many have been given up on.
pub fn progressed() -> (u32, u32) {
    (
        TX_SUCCESSES.load(Ordering::Relaxed),
        TX_FAILURES.load(Ordering::Relaxed),
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn nrf_gzll_device_tx_success(_pipe: u32, _info: *const u8) {
    TX_SUCCESSES.fetch_add(1, Ordering::Relaxed);
}

#[unsafe(no_mangle)]
pub extern "C" fn nrf_gzll_device_tx_failed(_pipe: u32, _info: *const u8) {
    TX_FAILURES.fetch_add(1, Ordering::Relaxed);
}

/// Referenced by the archive even in device role, so it has to exist.
#[unsafe(no_mangle)]
pub extern "C" fn nrf_gzll_host_rx_data_ready(_pipe: u32, _info: u32) {}

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
/// `SWI0_EGU0_IRQHandler`. Same vector, two naming conventions. (This is the
/// name on nRF52840 and nRF52833 both; the `SWI0_EGU0` spelling belongs to
/// nRF52832-era headers.)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn EGU0_SWI0() {
    unsafe { SWI0_EGU0_IRQHandler() }
}