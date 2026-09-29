//! The board description: what build.rs generated, plus the one thing that
//! cannot be generated - the matrix scan, whose shape depends on the geometry
//! and on the diode direction.
//!
//! Nothing here hardcodes a pin, a size or a radio parameter: those live in
//! `board.toml`, reach this file through `board_generated.rs`, and a wrong value
//! there is a build error rather than firmware that misbehaves on the air.

use embassy_nrf::gpio::{Input, Output};

include!(concat!(env!("OUT_DIR"), "/board_generated.rs"));

/// One half's matrix pins.
///
/// `drive` is the side that is driven (always outputs), `read` is the side that
/// is sampled (always inputs) - which physical direction that is depends on
/// `[matrix] diode` in board.toml, and that is why the two arrays are named
/// after the electrical role rather than after rows and columns.
/// How often a half that has no data-ready line is read, in milliseconds.
///
/// Only the pad uses this, and only as a beat: it announces nothing, so something
/// has to decide when to touch the bus. The stick is read from its own line and
/// never from here - see `Pointer::read_due` for why a beat would be wrong for it.
///
/// 5 ms at 400 kHz is about 200 us of bus time per read, which is why it is not
/// simply every pass.
pub const POINTER_POLL_MS: u32 = 5;

/// Wait for whichever of two futures finishes first, dropping the other.
///
/// Hand-rolled rather than taken from `embassy-futures`, for a boring reason: the
/// copy in this machine's registry cache predates the rest of this dependency set,
/// and pulling a current one would make a build that currently works offline need
/// the network. The two-way case is all this firmware has, so the version worth
/// carrying is a dozen lines rather than a dependency.
async fn wait_either<A, B>(a: A, b: B)
where
    A: core::future::Future<Output = ()>,
    B: core::future::Future<Output = ()>,
{
    let mut a = core::pin::pin!(a);
    let mut b = core::pin::pin!(b);
    core::future::poll_fn(|cx| {
        if a.as_mut().poll(cx).is_ready() {
            return core::task::Poll::Ready(());
        }
        if b.as_mut().poll(cx).is_ready() {
            return core::task::Poll::Ready(());
        }
        core::task::Poll::Pending
    })
    .await
}

/// The minimum the loop sleeps on every pass, before it arms the edge wake-ups.
///
/// WHY THIS EXISTS. `wait_any_of` returns as soon as ANY of its three futures is
/// ready, and two of the three are GPIO edges. A pin that is not held at a defined
/// level produces edges by itself, and each one is a perfectly good reason to run
/// the whole loop body again - so a noisy contact turns "sleep until something
/// happens" into "never sleep", at full CPU speed, which on this chip is about
/// 3.3 mA. Nothing inside the loop can tell that apart from a real knob turn,
/// because from the loop's side they are the same event.
///
/// The knob's two contacts are exactly such a risk. The factory devicetree declares
/// them `GPIO_ACTIVE_HIGH | GPIO_OPEN_DRAIN` (quoted in board.toml above), and this
/// firmware configures them `Pull::None` because that is the only combination
/// measured to make the knob work at all - `Pull::Up` and `Pull::Down` were both
/// tried and both killed it. So nothing internal holds the line at a level while
/// the contact is open, and these same lines are already known to produce phantom
/// steps.
///
/// A floor turns an unbounded failure into a bounded one. With it the loop runs at
/// most one pass per SLEEP_FLOOR_US however noisy the pins are, which puts a hard
/// ceiling on CPU duty: at 500 us that is 2000 passes/s of a few microseconds each,
/// tens of uA instead of milliamps. It costs nothing when the pins are quiet,
/// because the tick is already longer than the floor and only the remainder is
/// parked after it - total pass length is unchanged, so the scan cadence (1 ms
/// active, `idle_poll_ms` idle) is exactly what it was.
///
/// WHAT IT COSTS. Between the floor and the arming of the edge futures there is a
/// window in which an edge is not a wake-up: embassy's `wait_for_any_edge` picks its
/// sense from the level at the moment it is first polled, so a change that already
/// happened is read as the starting state and does not re-trigger. That is safe
/// here rather than merely tolerable, for three separate reasons:
///
///   * `Encoder::poll` reads LEVELS and works the transition out of the pair, so a
///     level change inside the window is still seen on the next pass. Actually
///     losing a step would need two quadrature transitions inside 500 us, which at
///     `ENCODER_RESOLUTION = 4` is over a thousand detents a second;
///   * `TrackPoint::wait_motion` checks `is_low()` first and returns at once when
///     the line is already asserted, so a packet that arrived during the window is
///     read on the next pass rather than lost;
///   * the matrix has no edge source at all and runs off the tick either way.
///
/// 500 us is also about a tenth of the shortest interval anyone can feel: this
/// project's own finding is that 5 ms is below resolution and 10-20 ms is where a
/// difference becomes reliably perceptible.
///
/// Raising it saves more current under a storm and blunts the knob further;
/// lowering it does the reverse. Setting it to 0 restores the pre-2026-09-28
/// behaviour exactly, which is also the way to test whether this was worth adding.
pub const SLEEP_FLOOR_US: u64 = 500;

/// The same, for the three things the main loop watches: its own tick, the knob,
/// and the TrackPoint's data-ready line.
///
/// A separate function because the loop's borrows are what make it awkward - three
/// disjoint pieces of state, borrowed mutably at once - and spelling it out here
/// keeps that awkwardness in one place. `Timer::after` borrows nothing, so only the
/// last two matter.
pub(crate) async fn wait_any_of<A, B, C>(a: A, b: B, c: C) -> Wake
where
    A: core::future::Future<Output = ()>,
    B: core::future::Future<Output = ()>,
    C: core::future::Future<Output = ()>,
{
    let mut a = core::pin::pin!(a);
    let mut b = core::pin::pin!(b);
    let mut c = core::pin::pin!(c);
    core::future::poll_fn(|cx| {
        if a.as_mut().poll(cx).is_ready() {
            return core::task::Poll::Ready(Wake::Tick);
        }
        if b.as_mut().poll(cx).is_ready() {
            return core::task::Poll::Ready(Wake::Knob);
        }
        if c.as_mut().poll(cx).is_ready() {
            return core::task::Poll::Ready(Wake::Pointer);
        }
        core::task::Poll::Pending
    })
    .await
}

/// Which of the three things the loop was parked on brought it back.
///
/// The loop does not need to know this in general - everything it does is a fresh
/// reading either way - but the pointer is the exception, and the reason is the
/// acceleration curve. That curve is a function of the interval between two
/// packets, so the firmware must read the device when the DEVICE says it has
/// something, not on a beat of its own choosing. Reading on a beat makes the
/// interval a constant, and a constant interval turns `exp(dist / gap)` into a
/// flat multiplier - the acceleration silently becomes a fixed gain.
///
/// So the pointer is read when this says `Pointer`. The other two cases matter for
/// the halves that have no line to be woken by.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Wake {
    /// The loop's own tick: the matrix scan's turn, and nothing more.
    Tick,
    /// A knob contact moved.
    Knob,
    /// The pointing device signalled that it has something to read.
    Pointer,
}

pub struct Matrix {
    pub drive: [Output<'static>; DRIVE_PINS],
    pub read: [Input<'static>; READ_PINS],
}

impl Matrix {
    /// Read the whole matrix once, as the half's report bitmap: bit `r * COLS + c`
    /// is row `r`, column `c`, row 0 first.
    ///
    /// The loop drives one line high, waits for the level to settle, reads the
    /// other side, and drops the line again - the same order the original
    /// transmitter uses (`redox-w-keyboard-basic`, whose `read_keys()` even
    /// comments on needing a gap between driving and reading). The difference is
    /// that the settling time here is an explicit, configurable number of cycles
    /// rather than a single `nop`.
    ///
    /// The scan produces the wire format directly, so there is no packing step and
    /// no second representation to keep in step. With 8 columns the compiler folds
    /// `bit / 8` and `bit % 8` back into the row index and the column bit the
    /// original code wrote by hand, so this costs exactly what it used to.
    pub fn scan(&mut self) -> [u8; MATRIX_BYTES] {
        let mut keys = [0u8; MATRIX_BYTES];
        for (i, line) in self.drive.iter_mut().enumerate() {
            line.set_high();
            cortex_m::asm::delay(SETTLE_CYCLES);
            if DIODE_IS_COL2ROW {
                // Driving column i, reading rows.
                for (r, pin) in self.read.iter().enumerate() {
                    if pin.is_high() {
                        set_bit(&mut keys, r, i);
                    }
                }
            } else {
                // Driving row i, reading columns.
                for (c, pin) in self.read.iter().enumerate() {
                    if pin.is_high() {
                        set_bit(&mut keys, i, c);
                    }
                }
            }
            line.set_low();
        }
        keys
    }
}

/// Bytes in one half's report: the matrix as a single bitmap, row 0 first.
///
/// This is the scan's own output format and the on-air format both, so there is
/// no packing step and no second representation to keep in step.
///
/// With 8 columns the bitmap is the "one byte per row" layout the original
/// transmitter writes by hand, byte for byte (8 columns fill exactly one byte) -
/// which is why KeyPoint's packets did not change when the format became a bitmap,
/// and why a wider keyboard is now a board.toml edit rather than a protocol
/// change: 8x16 needs 16 bytes, 16x16 exactly fills Gazell's 32-byte payload.
pub const MATRIX_BYTES: usize = (ROWS * COLS).div_ceil(8);

/// Set bit `r * COLS + c` - row `r`, column `c` - in a half's report bitmap.
///
/// `#[inline(always)]` on purpose: `COLS` is a constant, so each call site folds
/// `bit / 8` into a byte index and `bit % 8` into a shift. With the usual 8 columns
/// this compiles down to exactly the `keys[row] |= 1 << col` the original
/// transmitter writes by hand.
#[inline(always)]
fn set_bit(bitmap: &mut [u8; MATRIX_BYTES], r: usize, c: usize) {
    let bit = r * COLS + c;
    bitmap[bit / 8] |= 1 << (bit % 8);
}

/// Clear the bit `set_bit` would set.
///
/// Used on the way out rather than on the way in. Every other cell of the matrix
/// is a key whose whole job is to reach the host, but two of them - whichever the
/// half acts on itself - are switches this firmware reads rather than keys it
/// forwards. They have to stay in the raw scan, because that is what the latch
/// logic watches; they must not appear in the report, because a host that sees
/// them sees a keypress and there is nothing on the other side of the link that
/// can take it back. Clearing here is the only place the half still has a say.
#[inline(always)]
pub fn clear_bit(bitmap: &mut [u8; MATRIX_BYTES], r: usize, c: usize) {
    let bit = r * COLS + c;
    bitmap[bit / 8] &= !(1 << (bit % 8));
}

// ---------------------------------------------------------------------------
// The rest of the report
//
// One half's report is its matrix, then everything else that half knows. This
// layout is PROTOCOL, not board configuration, so it is written down here rather
// than in board.toml - but the receiver's board.rs must carry the same offsets,
// and that is why its copy of this block names the same constants. The field
// table, and the reason every field is cumulative rather than a delta, are at
// that copy; the short version is that the link has no retransmission and no back
// channel, so a delta lost in the air is lost for good.
//
// A half fills in only what it has. The left half carries the trackpad, the right
// half the TrackPoint, and the fields a half does not use stay zero. A half does
// NOT decide what a pointing device means: it reports raw displacement and leaves
// the mode to the receiver, because the mode is switched by a key and the keys
// live there.
// ---------------------------------------------------------------------------

/// Encoder cumulative count, low 8 bits. Wraps at 256; the receiver differences
/// consecutive packets, so what goes out is a *count*, not a movement event.
pub const OFF_ENCODER: usize = MATRIX_BYTES;
/// Pointing-device buttons, one bit per button.
pub const OFF_POINTER_BUTTONS: usize = OFF_ENCODER + 1;
/// Pointing device X, cumulative i16 little-endian (wrapping).
pub const OFF_POINTER_X: usize = OFF_POINTER_BUTTONS + 1;
/// Pointing device Y, cumulative i16 little-endian (wrapping).
pub const OFF_POINTER_Y: usize = OFF_POINTER_X + 2;
/// What this half's pointing device should produce: 0 = cursor, 1 = wheel.
///
/// The one field on the wire that is not cumulative, and the reason is that it is not
/// a measurement: it is a SETTING, owned by this half (its mode switch) and stated in
/// every packet. Cumulative values exist so a lost packet repairs itself on the next
/// one; a setting does the same thing for free, because the next packet repeats it.
///
/// It is here rather than resolved on the receiver because the switch that changes it
/// is on this half, and the link runs one way - the receiver could never be told
/// otherwise. Nothing else about the mode is decided here: the receiver still owns
/// what "wheel" means (scale, direction, and whether a held thumb key is asking for
/// the same thing).
pub const OFF_POINTER_MODE: usize = OFF_POINTER_Y + 2;
/// How long the displacement in this packet took, in milliseconds, saturating at
/// 255.
///
/// A MEASUREMENT, and the only one on the wire that is not a cumulative value -
/// it cannot be one, because it is a duration rather than a position, and because
/// the receiver needs the interval THIS packet's displacement covers.
///
/// It is here because the acceleration curve is a function of speed, and speed
/// needs a clock. The clock that matters is the DEVICE's - the interval between
/// two of its packets, which the stick's data-ready line makes a real measurement.
/// The receiver has a clock of its own, but it measures the radio's arrival
/// schedule, not the sensor's, so timing the curve there would make a retransmit
/// look like a flick. Measuring on this side and shipping the answer keeps the two
/// identical: the receiver divides by the same number this half would have used.
///
/// Milliseconds rather than microseconds because the curve only cares about the
/// order of magnitude, and one byte leaves no room for finer units.
pub const OFF_GAP: usize = OFF_POINTER_MODE + 1;
/// Packet sequence number. Carries nothing yet - it is here so that loss can be
/// measured later without spending another byte on it then.
pub const OFF_SEQ: usize = OFF_GAP + 1;

/// Bytes on the wire for one half: the matrix bitmap, then the fields above.
///
/// 6x8 gives 6 + 9 = 15 bytes, against Gazell's 32-byte limit.
pub const PAYLOAD_LENGTH: usize = OFF_SEQ + 1;

// ---------------------------------------------------------------------------
// The knob
// ---------------------------------------------------------------------------

// The I2C bus the pointing device sits on.
//
// Defined here, once, and not in the generated per-half code: both halves are
// compiled into the same library, so a second `bind_interrupts!` for TWISPI0
// inside the other half's module would define the same interrupt symbol twice and
// the link would fail. One binding serves whichever half carries a pointer.
embassy_nrf::bind_interrupts!(pub struct Irqs {
    TWISPI0 => embassy_nrf::twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
});

/// TWIM's write buffer, which has to be `'static` and in RAM.
///
/// The TrackPoint never writes to its bus, but the A320 must: one byte points at
/// the motion register before the read. TWIM drives EasyDMA from RAM only, so that
/// byte has to live there - a `&[0x82u8]` literal would sit in flash and the
/// transaction would fail with no visible error at all. Hence a real buffer rather
/// than the empty placeholder a read-only device would have allowed.
pub fn tx_buffer() -> &'static mut [u8] {
    static CELL: static_cell::StaticCell<[u8; TX_BUFFER_LEN]> = static_cell::StaticCell::new();
    CELL.init([0; TX_BUFFER_LEN])
}

/// Enough for the longest write either device takes: one register byte.
const TX_BUFFER_LEN: usize = 8;

/// A half's pointing device, whichever it is.
///
/// The matrix and the knob look the same from the main loop on both halves; the
/// pointer does not, and pretending otherwise would mean either two optionals that
/// can never both be set or a trait object on a no-alloc target. This is the
/// smallest thing that lets the loop say "poll the pointer" without knowing which.
pub enum Pointer {
    TrackPoint(TrackPoint),
    A320(A320),
}

impl Pointer {
    /// Wait until the device has something to say, if it has a way of saying it.
    ///
    /// This is the half of the sleep story that costs nothing when idle: an await
    /// here parks the whole loop in `WFI`, so the current a pass costs is only paid
    /// when there is a reason to run one.
    ///
    /// The TrackPoint has a data-ready line and gets woken the moment it moves. The
    /// pad has one on the board too, but this firmware does not configure it - see
    /// A320 for why an unread line beats a guessed-about one - so it parks forever
    /// here and is instead read on the loop's own tick. That costs the pad at most
    /// one idle tick of latency, which is why it is acceptable.
    pub async fn wait_motion(&mut self) {
        match self {
            Pointer::TrackPoint(dev) => dev.wait_motion().await,
            Pointer::A320(dev) => dev.wait_motion().await,
        }
    }

    /// Whether a read is due, given what woke the loop and how long it has been.
    ///
    /// The two devices answer this differently, and the difference is not taste -
    /// see `Wake`.
    ///
    /// The TrackPoint is read exactly when its line says to. There is deliberately
    /// no beat underneath that, and that absence is the whole point: a beat pins the
    /// packet interval to its own period, and the acceleration curve is a function
    /// of that interval, so the curve flattens into a fixed multiplier. The one
    /// exception is the fallback, which is insurance against a dead line rather than
    /// part of normal operation - see `TP_MOTION_FALLBACK_MS` - and it can only fire
    /// once the device has gone quiet for far longer than a healthy one ever does.
    ///
    /// The pad has no line configured, so it is read on the beat instead. It does
    /// not miss the interval: it is a touchpad whose packets carry plain
    /// displacement, and there is no acceleration curve on this side at all - the
    /// BLE firmware and the ZMK original agree there is none.
    pub(crate) fn read_due(&self, wake: Wake, since_ms: u32) -> bool {
        match self {
            Pointer::TrackPoint(_) => {
                wake == Wake::Pointer || since_ms >= TP_MOTION_FALLBACK_MS as u32
            }
            Pointer::A320(_) => since_ms >= POINTER_POLL_MS,
        }
    }

    /// Read the device and fold the result into its accumulated position. Returns
    /// whether anything moved.
    pub async fn poll(&mut self) -> bool {
        match self {
            Pointer::TrackPoint(dev) => dev.poll().await,
            Pointer::A320(dev) => dev.poll().await,
        }
    }

    /// Switch this device's own supply, when the board gave it a gate.
    ///
    /// Both devices take it, because both halves of this keyboard have a rail: the
    /// nub's is P0.08 and the pad's is P0.02 - see `PointerPower`. On a half with no
    /// gate this records the intent and does nothing else, which is the honest
    /// answer rather than a false one.
    pub fn set_power(&mut self, on: bool) {
        match self {
            Pointer::TrackPoint(dev) => dev.set_power(on),
            Pointer::A320(dev) => dev.set_power(on),
        }
    }

    /// Whether this device is powered right now.
    pub fn power_on(&self) -> bool {
        match self {
            Pointer::TrackPoint(dev) => dev.power_on(),
            Pointer::A320(dev) => dev.power_on(),
        }
    }

    /// The cumulative displacement to put on the air.
    pub fn position(&self) -> (i16, i16) {
        match self {
            Pointer::TrackPoint(dev) => dev.position(),
            Pointer::A320(dev) => dev.position(),
        }
    }
}

// ---------------------------------------------------------------------------
// The pointing device's own supply
// ---------------------------------------------------------------------------

/// How long after the gate is released the device is left alone.
///
/// The nub is a PS/2-to-I2C bridge with a sensor behind it: give it power back and
/// it needs a moment before its answers mean anything. Reads inside that window are
/// dropped rather than trusted, because a device that is still starting up answers
/// with garbage - and garbage added to a CUMULATIVE counter is a cursor that jumps.
///
/// The pad is a plain register interface and most likely needs far less; the two
/// share one number on purpose, because this is a settle guess rather than a
/// measurement, and one number is one thing to correct.
pub const POINTER_POWER_SETTLE_MS: u64 = 250;

/// How long a second press of the power key is ignored.
///
/// The key is read from the raw scan, whose whole purpose is to see contact bounce;
/// without this, one physical press would toggle two or three times. Long enough to
/// swallow any bounce, short enough that two deliberate presses are two toggles.
pub const POINTER_POWER_KEY_LOCKOUT_MS: u32 = 300;

/// The MOSFET gate that switches a pointing device's own supply, when the board has
/// one.
///
/// `None` means this half's pointer has no gate and stays powered: the switch is then
/// software only - the half still stops touching the bus - but it cannot remove the
/// device's own quiescent draw, which is the entire reason a hardware switch is worth
/// having.
///
/// ACTIVE LOW, and that polarity is hardware rather than preference: it is what ZMK's
/// own `EXT_POWER` nodes carry for both halves of this keyboard (see board.toml).
pub struct PointerPower {
    gate: Option<Output<'static>>,
    on: bool,
    /// When the gate was last released, so reads can be held off until the device has
    /// come up. See `POINTER_POWER_SETTLE_MS`.
    switched_on_at: embassy_time::Instant,
}

impl PointerPower {
    pub fn new(gate: Option<Output<'static>>) -> Self {
        let mut power = Self {
            gate,
            on: false,
            switched_on_at: embassy_time::Instant::now(),
        };
        // Boot = unpowered, and it is driven here rather than left to the first pass of
        // the loop (Lemon, 2026-09-19): the pointer starts in OFFLINE, so the half is as
        // close to idle as it gets from the first instruction, instead of spending the
        // tens of milliseconds between reset and that pass with a live rail.
        //
        // ZMK's `ext_power` defaults to *on*, which is why this used to match it. That
        // default is a convenience for a display or a light; here the rail feeds a
        // pointing device whose idle current is the largest single load on this half
        // (1..3 mA, measured over a 36 h standby by Lemon), and the state the user
        // actually wants after a reset is off.
        //
        // Nothing is remembered: this half has no storage, and after a power cycle the
        // question is worth asking again anyway.
        power.drive(false);
        power
    }

    fn drive(&mut self, on: bool) {
        // ACTIVE LOW: low = the MOSFET conducts = the device has power.
        if let Some(gate) = self.gate.as_mut() {
            if on {
                gate.set_low();
            } else {
                gate.set_high();
            }
        }
    }

    pub fn is_on(&self) -> bool {
        self.on
    }

    /// Switch the device's supply.
    pub fn set(&mut self, on: bool) {
        if on == self.on {
            return;
        }
        self.on = on;
        self.drive(on);
        if on {
            self.switched_on_at = embassy_time::Instant::now();
        }
    }

    /// Powered AND past the settle window, i.e. safe to talk to.
    fn ready(&self) -> bool {
        self.on
            && embassy_time::Instant::now()
                .saturating_duration_since(self.switched_on_at)
                .as_millis()
                >= POINTER_POWER_SETTLE_MS
    }
}

/// Whether the matrix cell (row `r`, column `c`) is down in a scanned bitmap.
///
/// The read counterpart of `set_bit`, and the only way a half reads a key for itself:
/// board.toml names the cell in local coordinates, the scan fills the bitmap, and the
/// loop asks about that one cell.
///
/// An out-of-range ask answers `false` instead of panicking. build.rs already refuses
/// a power_key outside the matrix, and a firmware that panics while a hand is on the
/// keyboard is a worse failure than one that ignores a typo.
#[inline(always)]
pub fn key_down(bitmap: &[u8; MATRIX_BYTES], r: usize, c: usize) -> bool {
    if r >= ROWS || c >= COLS {
        return false;
    }
    let bit = r * COLS + c;
    bitmap[bit / 8] & (1 << (bit % 8)) != 0
}

/// A320 trackpad, left half.
///
/// Nothing like the TrackPoint on the wire: this one is a register interface. One
/// byte is written to aim at the motion register, then three come back -
/// `[status, dx, dy]`, the deltas being signed 8-bit counts.
///
/// Unlike the PS/2 bridge it does not hold its clock down while idle; it answers at
/// once with a zero packet. So the deadline here is insurance against a wedged bus
/// rather than part of normal operation, and a zero packet is the ordinary "nothing
/// moved" answer rather than an error.
///
/// Cumulative, for the same reason as everything else this half sends.
pub struct A320 {
    i2c: embassy_nrf::twim::Twim<'static>,
    x: i16,
    y: i16,
    /// The pad's supply switch, when the board has one. See `PointerPower`.
    power: PointerPower,
}

/// A320 slave address - `reg = <0x3B>` in the overlay.
const A320_ADDR: u8 = 0x3B;
/// Written first to aim the burst at the motion register.
const A320_MOTION_REG: u8 = 0x82;
/// `[status, dx, dy]`. The status byte carries touch state and is not used: nothing
/// on this keyboard reads it, and the BLE firmware ignored it too.
const A320_PACKET_LEN: usize = 3;
const A320_READ_TIMEOUT_MS: u64 = 20;
/// ZMK's Kconfig default for the pad: at or below this, it is noise.
const A320_DEADZONE: i8 = 1;
/// Ceiling on one drain pass.
///
/// The loop is already bounded by the pad answering a zero packet once it is dry,
/// so this only backstops a pad that somehow never does - which would otherwise be
/// a stuck main loop, the one failure this design must not have.
const A320_DRAIN_LIMIT: u32 = 32;

impl A320 {
    /// Park forever: the pad is read on the loop's own tick, never wakes it.
    ///
    /// The pad does have a data-ready line on the board (P0.08 - the BLE firmware's
    /// own `Input::new(p.P0_08, Pull::Up)` is the citation), and configuring it
    /// would let the pad wake the loop the way the TrackPoint's motion line does.
    /// That is deliberately not done: neither the line's polarity nor whether it
    /// holds or pulses is recorded anywhere this build can point at, and a wake
    /// source that turns out to be wrong in the "always asserted" direction is
    /// worse than having none - the loop would spin at full rate forever, which is
    /// the exact thing this change exists to stop.
    ///
    /// Parking here is honest about what is actually known. The cost is bounded and
    /// small: a stroke that begins while the loop is idle is picked up on the next
    /// tick, so the pad is at most one idle period behind, against a device whose
    /// own samples are tens of milliseconds apart.
    pub async fn wait_motion(&mut self) {
        core::future::pending::<()>().await
    }
    pub fn new(i2c: embassy_nrf::twim::Twim<'static>, power: PointerPower) -> Self {
        Self {
            i2c,
            x: 0,
            y: 0,
            power,
        }
    }

    /// Switch the pad's supply. See `Pointer::set_power`.
    pub fn set_power(&mut self, on: bool) {
        self.power.set(on);
    }

    pub fn power_on(&self) -> bool {
        self.power.is_on()
    }

    pub fn position(&self) -> (i16, i16) {
        (self.x, self.y)
    }

    /// Read one packet, deadzoned and signed, or `None` when there is nothing more.
    async fn read_packet(&mut self) -> Option<(i16, i16)> {
        use embedded_hal_async::i2c::Operation;

        // The pointer byte lives in a local on purpose - see `tx_buffer` for why it
        // cannot be a literal.
        let ptr = [A320_MOTION_REG];
        let mut buf = [0u8; A320_PACKET_LEN];
        let read = embassy_time::with_timeout(
            embassy_time::Duration::from_millis(A320_READ_TIMEOUT_MS),
            self.i2c.transaction(
                A320_ADDR,
                &mut [Operation::Write(&ptr), Operation::Read(&mut buf)],
            ),
        )
        .await;
        match read {
            Ok(Ok(())) => {}
            Ok(Err(_)) | Err(_) => return None,
        }

        let raw_x = buf[1] as i8;
        // The pad reports +Y downwards; negate it, which is where ZMK negates.
        let raw_y = (buf[2] as i8).wrapping_neg();
        let dx = if raw_x.abs() <= A320_DEADZONE { 0 } else { raw_x };
        let dy = if raw_y.abs() <= A320_DEADZONE { 0 } else { raw_y };
        if dx == 0 && dy == 0 {
            return None;
        }
        // X is negated for a different reason than the stick's: it is ZMK's scroll
        // convention (`SCROLL_X_DIR = -1`), independent of the hardware mirror
        // compensation that lives in the receiver's tier table. Removing this flips
        // one axis on hardware.
        Some((-(dx as i16), dy as i16))
    }

    /// Read the pad dry and fold everything into the accumulated displacement.
    ///
    /// A loop rather than a single read, and this is the one place the pad and the
    /// stick genuinely differ. The stick's packet is the CURRENT position, so
    /// reading it once whenever convenient is enough. The pad queues DELTAS - each
    /// packet is the movement since the one before it - so whatever is not read
    /// stays unread, and the cursor ends up trailing the finger by however much
    /// built up. The BLE firmware drains for the same reason.
    ///
    /// The loop ends on the first zero packet, which is what the pad answers when
    /// it has nothing more to give, and on any bus error.
    pub async fn poll(&mut self) -> bool {
        // Switched off, or switched on but still coming up: nothing to read. The
        // settle window is why this cannot simply be "am I powered": a device that is
        // still starting up answers, and its answers go into a cumulative counter.
        if !self.power.ready() {
            return false;
        }
        let mut moved = false;
        for _ in 0..A320_DRAIN_LIMIT {
            let Some((dx, dy)) = self.read_packet().await else {
                break;
            };
            self.x = self.x.wrapping_add(dx);
            self.y = self.y.wrapping_add(dy);
            moved = true;
        }
        moved
    }
}

// ---------------------------------------------------------------------------
// Pointing device
// ---------------------------------------------------------------------------

/// TrackPoint slave address - `reg = <0x15>` in the ZMK overlay.
const TP_ADDR: u8 = 0x15;
/// The bridge's packet is a fixed 7 bytes: byte 0 is a magic number, bytes 2 and 3
/// are the signed x and y counts.
const TP_PACKET_LEN: usize = 7;
/// `TRACKPOINT_MAGIC_BYTE0`. A packet that does not start with this is not a
/// movement report.
const TP_MAGIC: u8 = 0x50;
/// The bridge holds SCL low while it has nothing to send and TWIM waits for
/// DEVSTOP, so an unbounded read blocks forever - and here it would take the whole
/// scan loop with it. This is the deadline. A missed round is dropped.
const TP_READ_TIMEOUT_MS: u64 = 20;
/// Counts this small are drift settling rather than a finger: ZMK's
/// `TRACKPOINT_DRIFT_DEADZONE`.
const TP_DEADZONE: i8 = 2;

/// Gain and the acceleration curve USED to live here. They moved to the receiver
/// when `OFF_GAP` gave it the one thing it could not measure for itself, because
/// both are SETTINGS: the keymap that names a tier lives on the receiver, and a half
/// with no storage, no USB port and no probe is the worst possible place to keep a
/// knob the user expects to turn.
///
/// Nothing about the numbers changed in the move - `rx/src/pointer_accel.rs` carries
/// the sensitivity factor and the curve, and this half now puts the counts it read
/// on the air unchanged.

/// How long the data-ready line may stay silent before the stick is read anyway.
///
/// Insurance against a dead line, not part of normal operation: the bridge raises
/// the line for every packet, so with a working line this never expires. What it
/// buys is that a silent line - unpopulated, mis-wired, or a bridge that stopped
/// raising it - degrades to a slow pointer instead of to no pointer at all, which
/// is the one failure that would be indistinguishable from a dead device.
///
/// The value has to be long enough that a healthy device cannot reach it, and this
/// is the whole reason it is 50 ms rather than the read beat it replaced: a
/// TrackPoint reports well under 200 Hz, so 50 ms is several missed packets, while
/// a beat of a few milliseconds would be reached constantly and would - by pinning
/// the packet interval - flatten the acceleration curve. The cost of it firing by
/// mistake is only that one packet's gap stops being a measurement.
const TP_MOTION_FALLBACK_MS: u64 = 50;

/// The TrackPoint, read over I2C and accumulated.
///
/// The device is a PS/2-to-I2C bridge. It answers with that fixed packet and
/// resends the last one while the stick is still. What this half puts on the air is
/// a CUMULATIVE displacement rather than a delta, for the same reason the knob
/// sends a count: a packet lost in the air repairs itself on the next one, while a
/// delta would simply be gone.
///
/// Nothing about gain or the acceleration curve lives here any more. Both are
/// functions of the packet interval, this half is still the one that measures it -
/// but the interval now goes ON THE AIR (`OFF_GAP`) instead of being spent here, so
/// the receiver can run the curve against settings it owns. This half reads counts
/// and reports them.
///
/// Speed tiers, the curve and cursor-versus-scroll all stay on the receiver, because
/// they depend on settings the keymap owns and on keys that live on the other half -
/// and a half with no USB and no probe is the worst place to debug a pointer that
/// feels wrong.
pub struct TrackPoint {
    i2c: embassy_nrf::twim::Twim<'static>,
    /// Cumulative X and Y, wrapping on purpose - only the difference between two
    /// packets is ever used, so wrapping costs nothing.
    x: i16,
    y: i16,
    /// The bridge's data-ready line, when board.toml gives this half one.
    ///
    /// Active low with the bridge holding it up - the same electrical story as ZMK's
    /// `GPIO_ACTIVE_LOW | GPIO_PULL_UP` on this pin. `None` means this half was built
    /// without one and `poll` falls back to reading on the timer.
    ///
    /// It is here because of the acceleration curve. That curve divides distance by
    /// the gap between two packets, and a half that reads on a beat of its own
    /// invention measures its own beat, not the device's: every packet then looks
    /// like it arrived right after the last one, so every movement looks fast and the
    /// curve saturates. Reading only when the device says it has something makes the
    /// gap a real measurement.
    motion: Option<embassy_nrf::gpio::Input<'static>>,
    /// The nub's supply switch, when the board has one. See `PointerPower`.
    power: PointerPower,
}

impl TrackPoint {
    pub fn new(
        i2c: embassy_nrf::twim::Twim<'static>,
        motion: Option<embassy_nrf::gpio::Input<'static>>,
        power: PointerPower,
    ) -> Self {
        Self {
            i2c,
            x: 0,
            y: 0,
            motion,
            power,
        }
    }

    /// Switch the nub's supply. See `Pointer::set_power`.
    pub fn set_power(&mut self, on: bool) {
        self.power.set(on);
    }

    pub fn power_on(&self) -> bool {
        self.power.is_on()
    }

    /// Wait for the bridge to signal that it has a packet.
    ///
    /// The level is checked before waiting, and that check is what makes this
    /// correct whether the line holds or pulses: if the bridge is asserting it right
    /// now then there is already something to read, so there is nothing to wait for.
    /// Without that fast path a held line would produce exactly one wake-up - the
    /// edge - and every packet after it would sit unread until the loop ran for
    /// some other reason.
    ///
    /// A half built without the line parks forever; see `A320::wait_motion` for why
    /// that is the honest answer for a line nobody has characterised.
    pub async fn wait_motion(&mut self) {
        // A device that has been switched off holds nothing: the line it used to
        // drive now leads to an unpowered part, so waiting on it would be waiting on
        // a floating pin - and a pin that floats low would spin this loop at full
        // rate for the entire time the device is off, which is exactly when the
        // saving is supposed to be happening. Nothing to wait for.
        if !self.power.is_on() {
            core::future::pending::<()>().await
        }
        match &mut self.motion {
            None => core::future::pending::<()>().await,
            Some(pin) => {
                if pin.is_low() {
                    return;
                }
                pin.wait_for_any_edge().await
            }
        }
    }


    /// The cumulative displacement to put on the air.
    pub fn position(&self) -> (i16, i16) {
        (self.x, self.y)
    }

    /// Read one packet and fold it into the accumulated displacement. Returns
    /// whether anything moved, which the main loop treats exactly like a keypress:
    /// send now, and do not go idle.
    pub async fn poll(&mut self) -> bool {
        // Powered off, or powered on but still coming up: nothing to read. The
        // settle window matters here more than anywhere: the bridge's answers while
        // it starts are garbage, and garbage added to a CUMULATIVE counter is a
        // cursor that jumps - and the receiver, which has no way to tell a jump from
        // a fast stroke, would faithfully draw it.
        if !self.power.ready() {
            return false;
        }
        // Whether to read at all has already been decided by the time this runs - the
        // loop calls it because the motion line woke it, or on its own tick. What is
        // left here is the gap, and that gap is now a measurement of the device
        // rather than of the loop: see `wait_motion`.
        let mut buf = [0u8; TP_PACKET_LEN];
        let read = embassy_time::with_timeout(
            embassy_time::Duration::from_millis(TP_READ_TIMEOUT_MS),
            self.i2c.read(TP_ADDR, &mut buf),
        )
        .await;
        // A bus error or a deadline is one dropped round, not a fault - the stick
        // is read again on the next pass. ZMK drops the round the same way.
        match read {
            Ok(Ok(())) => {}
            Ok(Err(_)) | Err(_) => return false,
        }
        if buf[0] != TP_MAGIC {
            return false;
        }

        let raw_x = buf[2] as i8;
        let raw_y = buf[3] as i8;
        // Ignore the small residuals, where real drift settles.
        let dx = if raw_x.abs() <= TP_DEADZONE { 0 } else { raw_x };
        let dy = if raw_y.abs() <= TP_DEADZONE { 0 } else { raw_y };
        if dx == 0 && dy == 0 {
            return false;
        }

        // The counts go on the air unchanged. The interval they cover is measured
        // where the report is built (see `OFF_GAP`), so the receiver is given
        // everything this half used to need in order to run the curve itself.

        // Negated here, which is where ZMK does it: the bridge reports how the stick
        // is pushed, and a push to the right is a pointer move to the left until this
        // flips it. Subtraction rather than a float conversion, so there is nothing
        // left to round: what accumulates is what the device reported.
        let ix = -dx as i16;
        let iy = -dy as i16;
        self.x = self.x.wrapping_add(ix);
        self.y = self.y.wrapping_add(iy);
        true
    }
}

/// Quadrature decoder for the knob, decoded exactly the way the receiver's decoder
/// will be.
///
/// The table and the "resolution" rule below are RMK's own
/// (`rmk/src/input_device/rotary_encoder.rs`, `ResolutionPhase`): a transition
/// contributes +1 or -1 pulses, and every `ENCODER_RESOLUTION` pulses are one
/// detent of the knob. Reading it the same way here is the point - it is what makes
/// "clockwise" the same physical direction at both ends of the link, by
/// construction rather than by two implementations happening to agree.
///
/// The count is CUMULATIVE and wraps at 256. What goes on the air is the count, not
/// the movement: a packet lost in the air is lost for good, while a count repairs
/// itself on the next packet.
pub struct Encoder {
    a: Input<'static>,
    b: Input<'static>,
    /// Bits 0-1: the previous state, as `a_is_low | b_is_low << 1`.
    state: u8,
    /// Pulses accumulated towards the next detent, in `-resolution..resolution`.
    pulses: i8,
    /// Cumulative detent count, low 8 bits.
    count: u8,
    /// Swap clockwise and counter-clockwise. Per half, and not a preference: the
    /// two knobs are mounted as mirror images, so the same quadrature direction
    /// means opposite things on the two sides. board.toml carries the two values.
    reverse: bool,
}

/// RMK's transition table. The index is
/// `prev_a | prev_b << 1 | cur_a << 2 | cur_b << 3`, and the value is the pulse
/// that transition contributes. The zeros are "nothing moved" and "both contacts
/// changed at once" - the latter is not something a turning knob does, and treating
/// it as movement would count contact bounce as clicks.
const ENCODER_LUT: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

impl Encoder {
    pub fn new(a: Input<'static>, b: Input<'static>, reverse: bool) -> Self {
        Self {
            a,
            b,
            state: 0,
            pulses: 0,
            count: 0,
            reverse,
        }
    }

    /// Wait until a contact moves, so the knob can be handled without the loop running.
    ///
    /// Both contacts are watched, not just one. A detent is four quadrature
    /// transitions and either contact can be the one that moves next, so watching a
    /// single line would miss every other step. Which one fired does not matter -
    /// `poll` reads both levels and works the transition out from the pair.
    ///
    /// This is the second half of the sleep story. With the loop parked on this
    /// future the core sits in `WFI` drawing almost nothing, and a turn brings it
    /// straight back - which is what replaces sampling the knob at 1 kHz forever.
    pub async fn wait_edge(&mut self) {
        let a = &mut self.a;
        let b = &mut self.b;
        wait_either(a.wait_for_any_edge(), b.wait_for_any_edge()).await;
    }

    /// The cumulative count to put on the air.
    pub fn count(&self) -> u8 {
        self.count
    }

    /// Sample the contacts once, and move the count if that completed a detent.
    ///
    /// Returns whether the count changed - which is what lets the main loop treat a
    /// turn exactly like a keypress: send now, and do not go idle.
    ///
    /// Polling at `scan_hz` is far faster than a hand can turn a knob: at 1 kHz even
    /// a violent flick is tens of ticks per detent.
    pub fn poll(&mut self) -> bool {
        let mut s = self.state & 0b11;
        if self.a.is_low() {
            s |= 0b0100;
        }
        if self.b.is_low() {
            s |= 0b1000;
        }
        self.state = s >> 2;

        // Nothing moved, or both contacts moved at once.
        if (s & 0xC) == (s & 0x3) {
            return false;
        }

        let step = ENCODER_LUT[(s & 0xF) as usize];
        self.pulses += if self.reverse { -step } else { step };

        // RMK's threshold, including which way round it reports: there a positive
        // accumulation is counter-clockwise, so it subtracts the count here.
        if self.pulses >= ENCODER_RESOLUTION {
            self.pulses %= ENCODER_RESOLUTION;
            self.count = self.count.wrapping_sub(1);
            true
        } else if self.pulses <= -ENCODER_RESOLUTION {
            self.pulses %= ENCODER_RESOLUTION;
            self.count = self.count.wrapping_add(1);
            true
        } else {
            false
        }
    }
}

/// Which half this firmware is.
///
/// A role carries exactly two things that differ between the halves: the Gazell
/// pipe and the channel subset. Everything else - geometry, timings, the radio's
/// other parameters - is the same on both, which is why they are constants
/// rather than part of this enum.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Left,
    Right,
}

impl Role {
    pub fn pipe(self) -> u32 {
        match self {
            Role::Left => left::PIPE,
            Role::Right => right::PIPE,
        }
    }

    pub fn channel_table(self) -> &'static [u8] {
        match self {
            Role::Left => &left::CHANNEL_TABLE,
            Role::Right => &right::CHANNEL_TABLE,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Role::Left => "left",
            Role::Right => "right",
        }
    }

    /// LOCAL (row, col) of the key that switches the pointing device's supply, and
    /// how long this half tolerates idleness before cutting it by itself.
    ///
    /// Both come from the `[pointer]` table, and both are per half in the generated
    /// code even though one table describes them - which keeps the door open for two
    /// different switches without changing how the loop asks.
    pub fn pointer_power_key(self) -> Option<(usize, usize)> {
        match self {
            Role::Left => left::POINTER_POWER_KEY,
            Role::Right => right::POINTER_POWER_KEY,
        }
    }

    /// LOCAL (row, col) of the key that switches that device between cursor and wheel.
    ///
    /// Same provenance as the power key above: board.toml writes it in whole-keyboard
    /// coordinates and build.rs converts it. Latching rather than momentary - the loop
    /// toggles on the press edge, which is why an edge is the thing that is read.
    pub fn pointer_mode_key(self) -> Option<(usize, usize)> {
        match self {
            Role::Left => left::POINTER_MODE_KEY,
            Role::Right => right::POINTER_MODE_KEY,
        }
    }

    pub fn pointer_idle_off_ms(self) -> u32 {
        match self {
            Role::Left => left::POINTER_POWER_IDLE_MS,
            Role::Right => right::POINTER_POWER_IDLE_MS,
        }
    }

    /// LOCAL (row, col) of the four cells that put this half into its bootloader.
    ///
    /// Per half because each half only ever sees its own rows: board.toml writes the
    /// gesture in whole-keyboard coordinates and build.rs converts all four cells
    /// here. See `crate::dfu` for what happens once they are down together.
    pub fn dfu_combo(self) -> &'static [(usize, usize); 4] {
        match self {
            Role::Left => &left::DFU_COMBO,
            Role::Right => &right::DFU_COMBO,
        }
    }

    /// Which of the nine animation frames this half starts on.
    ///
    /// ZMK's own left/right salts for this keyboard, carried over unchanged so
    /// the two panels are out of step exactly as they were under ZMK. Mixed with
    /// the chip's DEVICEID, so it is not simply "left frame 3, right frame 5".
    pub fn capy_salt(self) -> u32 {
        match self {
            Role::Left => 0x85eb_ca6b,
            Role::Right => 0x9e37_79b9,
        }
    }

    /// The word this half's panel uses for its own pointing device.
    ///
    /// The halves do not carry the same part - the left one has the pad, the right
    /// one the stick - so their panels must not print the same word. Spelled out
    /// rather than abbreviated (Lemon, 2026-09-18); the renderer draws this row one
    /// size down from the state row. See `screen::renderers::draw_pointer`.
    pub fn pointer_label(self) -> &'static str {
        match self {
            Role::Left => "TOUCH PAD",
            Role::Right => "TRACK POINT",
        }
    }

    /// The ADC reading this half's cell sits at when its charger reports it full.
    ///
    /// Per half, and measured rather than derived, because the two halves do not
    /// agree at the top: on 2026-09-19 the right half read 96 % at the moment its
    /// charge light went out, which is 4756 counts (~4151 mV) - about 30 mV below
    /// what the left half shows in the same state. The left half's reading has been
    /// right all along, so only the right one moves off the shared value.
    ///
    /// A voltage gauge cannot see "charging finished". Once the charger terminates
    /// the cell relaxes to roughly this level and stays there, so the anchor belongs
    /// where the cell actually comes to rest, not at the 4.18 V it only holds while
    /// current is still flowing into it.
    ///
    /// Set a little under the observed reading (4151 mV), so the display still reads
    /// 100 % if the cell settles a millivolt lower once the cable comes out.
    ///
    /// 2026-09-19: the right half's value is being swept by hand. Four firmwares are
    /// built at 4.15/4.16/4.17/4.18 V and Lemon flashes them in turn to find which
    /// one reads 100 % at the moment his charge light goes out. Whatever survives
    /// that test stays, with this paragraph rewritten to say so.
    ///
    /// If this half turns out to read LOW WHEN EMPTY as well, the error is one of
    /// gain rather than of where the ceiling sits, and the fix becomes a scale on
    /// this half's reading instead of a different full anchor. Nothing so far says
    /// which it is; the empty end is the observation that would.
    pub fn battery_full_counts(self) -> i32 {
        // Cell millivolts times the same 1145.9 counts per volt of cell that
        // `renderers` documents, in integer form. Written as a product rather than as
        // a ready-made count so that the number on the right is the voltage itself -
        // the thing actually known about the hardware - and not an arithmetic result
        // to be taken on trust.
        let mv = match self {
            Role::Left => 4180,
            Role::Right => 4180,
        };
        mv * 1146 / 1000
    }
}

// ---------------------------------------------------------------------------
// Invariants, checked at compile time.
// ---------------------------------------------------------------------------

/// The matrix goes out as one bitmap, so the link bounds the *cell count* - not
/// either dimension. 256 cells = 32 bytes is Gazell's payload limit; the scan's
/// own limit is separate and is checked below.
const _: () = assert!(ROWS >= 1, "rows must be at least 1");
const _: () = assert!(COLS >= 1, "cols must be at least 1");
const _: () = assert!(
    MATRIX_BYTES <= 32,
    "the matrix bitmap must fit in Gazell's 32-byte payload"
);
/// The WHOLE report has to fit, not just the matrix - and this is the bound that
/// bites first, because the trailing fields are a fixed eight bytes on top.
const _: () = assert!(
    PAYLOAD_LENGTH <= 32,
    "one half's report must fit in Gazell's 32-byte payload"
);
/// The driven and read sides must together cover the whole matrix.
const _: () = assert!(
    (DIODE_IS_COL2ROW && DRIVE_PINS == COLS && READ_PINS == ROWS)
        || (!DIODE_IS_COL2ROW && DRIVE_PINS == ROWS && READ_PINS == COLS),
    "the diode direction and the two pin lists disagree about which side is which"
);
const _: () = assert!(DEBOUNCE_TICKS >= 1, "debounce_ticks must be at least 1");
/// The idle poll must be slower than the active scan, or "idle" would not save
/// anything; and the active window must be long enough to cover a burst of
/// typing between two pauses.
const _: () = assert!(
    IDLE_POLL_MS > 1000 / SCAN_HZ,
    "idle_poll_ms must be slower than the active scan period"
);
const _: () = assert!(
    ACTIVE_RELEASE_MS > 0,
    "active_release_ms must be at least 1 ms"
);