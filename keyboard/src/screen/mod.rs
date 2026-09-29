//! The half's own status panel.
//!
//! Three pieces, kept apart because they answer different questions:
//!
//! * `lpm009m360a` - the panel's wire protocol, carried over from the BLE
//!   firmware where it was transcribed from the ZMK driver and verified on the
//!   real glass.
//! * `capy_art` - the animation frames, likewise carried over.
//! * `renderers` - what goes on screen, which is where this build differs: a
//!   half has no link to the receiver, so the badge, layer name and battery
//!   readout of the old build have no honest source here and are gone.
//!
//! ## When it paints
//!
//! There is no timer in here, and that is deliberate. The screen is a memory
//! panel: it holds whatever was last written, costs nothing to leave alone, and
//! has no sleep command, so painting it on a schedule would burn power to
//! redraw a picture that has not changed.
//!
//! Instead the screen is driven by events the loop was going to handle anyway:
//!
//! * the moment this half goes idle, and the moment it starts working again -
//!   these are exactly the two transitions the `RUN`/`IDLE` word reports;
//! * every `REDRAW_KEYS` key events while active, so the animation gets a chance
//!   to advance and the screen is not frozen for a whole session.
//!
//! Both are rare, and both happen at moments when nothing is waiting on the SPI
//! bus. A flush is 144 column lines at 4 MHz, about 3 ms, which is longer than a
//! matrix scan period - which is why the caller paints AFTER sending, never
//! before, and why nothing paints while a key is on its way out.

pub mod capy_art;
pub mod lpm009m360a;
pub mod renderers;

pub use renderers::draw_ui;

use embassy_nrf::gpio::Pin;
use embassy_nrf::saadc::{self, AnyInput, Saadc};
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::{bind_interrupts, interrupt, peripherals, Peri};
use embassy_nrf::interrupt::InterruptExt;
use static_cell::StaticCell;

use lpm009m360a::{Lpm009m360a, FRAMEBUFFER_LEN};

// The panel hangs off SPI2 on both halves. Binding the interrupt here rather
// than in each binary keeps the two halves on one definition - they differ in
// which pins they use, not in which peripheral.
bind_interrupts!(pub struct Irqs {
    SPI2 => spim::InterruptHandler<peripherals::SPI2>;
    SAADC => saadc::InterruptHandler;
});

/// The screen as the rest of the firmware sees it: a concrete type, because it
/// has to live inside `board`'s return value and a trait object is not an option
/// on this target.
pub type Screen = Lpm009m360a<Spim<'static>, embassy_nrf::gpio::Output<'static>>;

/// The battery sense, as the rest of the firmware sees it: one channel, as the
/// concrete type because it is stored in the task rather than passed around.
pub type Battery = Saadc<'static, 1>;

/// Configure the ADC to read the battery divider.
///
/// Same hardware as the BLE firmware, and the same divider it used: the sense pin
/// is one of the two analog-capable pins on this chip (P0_02 is AIN0, P0_03 is
/// AIN1), so the panel path can read the cell directly through it.
///
/// The ADC's default configuration - gain 1/6, 0.6 V reference, 12 bits - scales
/// the divider output to the low 3000s for a healthy cell. That is well clear of
/// the 500..1000 band, which is what a board measuring VDDH instead would show;
/// the conversion in `renderers` assumes the cell case, which is this board.
///
/// THE PRIORITY HERE IS DELIBERATE AND DELIBERATELY LOW. Every interrupt on this
/// chip starts at priority 0 unless something says otherwise, and Gazell is the
/// one peripheral here with real-time deadlines - it sets no priorities of its own
/// (checked by disassembling the library), so it relies on everything else staying
/// out of its way. Nordic's own Gazell examples put every application peripheral
/// at priority 6 for exactly that reason. The ADC is only ever read in the screen
/// task during a quiet window, so putting it below Gazell costs nothing and means
/// it can never be the thing that delays a packet.
/// One random number for the boot frame, from the chip's RNG.
///
/// The picture should not look identical across two switch-ons, and nothing else
/// on this chip can arrange that: the device id is the same every time and RAM does
/// not survive a power cycle. This is the peripheral that exists for it.
///
/// Read once, at boot, inside `pins()` - the only place where the peripherals are
/// still whole - and handed to the screen task. `Rng::new_blocking` needs no
/// interrupt and no channel for a single number, so it is used synchronously and
/// dropped straight away.
pub fn boot_seed_from_rng(rng: Peri<'static, peripherals::RNG>) -> u32 {
    embassy_nrf::rng::Rng::new_blocking(rng).blocking_next_u32()
}

pub fn new_battery(adc: Peri<'static, peripherals::SAADC>, pin: AnyInput<'static>) -> Battery {
    interrupt::SAADC.set_priority(interrupt::Priority::P3);
    let config = saadc::Config::default();
    let channel = saadc::ChannelConfig::single_ended(pin);
    Saadc::new(adc, Irqs, config, [channel])
}

/// Framebuffer storage.
///
/// 1296 bytes, handed to the driver as `&'static mut` because it is far too much
/// to leave on a task's stack frame.
static FB: StaticCell<[u8; FRAMEBUFFER_LEN]> = StaticCell::new();

/// Build the panel driver from its three pins, on SPI2 at 32 MHz.
///
/// 32 MHz because the half gets eight times fewer microseconds that way: the bus is
/// occupied for one eighth as long as it was at 4 MHz, and "how long the bus is busy"
/// is the quantity the quiet-window rule below is trying to keep out of the way of a
/// keystroke. The panel was probed at 4/8/16/32 MHz and drew correctly at all four,
/// so 16 MHz is the fallback if this one ever misbehaves - that is this single line.
///
/// Mode 0 and MSB-first are the protocol, not a preference; CS is active-high,
/// so its idle level is low.
pub fn new_screen<S, M, C>(
    spi: Peri<'static, peripherals::SPI2>,
    sck: Peri<'static, S>,
    mosi: Peri<'static, M>,
    cs: Peri<'static, C>,
) -> Screen
where
    S: Pin + 'static,
    M: Pin + 'static,
    C: Pin + 'static,
{
    let mut cfg = spim::Config::default();
    cfg.frequency = spim::Frequency::M32;
    let bus = Spim::new_txonly(spi, Irqs, sck, mosi, cfg);
    let cs = embassy_nrf::gpio::Output::new(
        cs,
        embassy_nrf::gpio::Level::Low,
        embassy_nrf::gpio::OutputDrive::Standard,
    );
    Lpm009m360a::new(
        bus,
        cs,
        FB.init([0u8; FRAMEBUFFER_LEN]),
        lpm009m360a::PanelRot::R270,
    )
}

/// How long the half must be completely quiet before a repaint is worth asking
/// for, in milliseconds.
///
/// This is the whole safety mechanism, and it is not a heuristic - it follows from
/// a measurement. The build that painted on the active/idle transitions, i.e. at
/// the edges of a burst, still dropped keys. The build that painted only at boot,
/// with the radio idle, dropped none. So: transfers at the start of a burst cost
/// keystrokes, transfers deep inside quiet do not, and the difference between the
/// two is what this constant buys.
///
/// Seconds, not the 500 ms that counts as idle for radio purposes. The bar for
/// starting a transfer is "the user has visibly stopped", not "no key moved in the
/// last blink".
///
/// Raising it is free and only makes the screen lazier. LOWERING IT IS NOT SAFE on
/// its own: it narrows the gap between the last keystroke and the transfer, which
/// is the exact variable the experiments say matters. If the screen feels too
/// stale, the fix is a transfer that can abandon itself part-way, not a shorter
/// window - see the note at the bottom of this module.
pub const QUIET_FOR_PAINT_MS: u64 = 3000;

/// How many key events pass between repaints while the half is active.
///
/// NOT USED YET - see the note at the bottom of this module. The current rule is
/// stricter than a key count, and this is kept as the knob to loosen it with once
/// the strict rule has proven itself on the real keyboard.
#[allow(dead_code)]
pub const REDRAW_KEYS: u32 = 1000;

// ---------------------------------------------------------------------------
// WHEN THIS PAINTS, AND WHY IT IS THIS CAREFUL  (2026-09-18)
//
// The keyboard drops keys whenever the panel is talked to while typing. That was
// bisected down over three builds:
//
//   1. painting inline in the main loop      -> dropped keys
//   2. painting in its own task (loop never blocks) -> still dropped keys
//   3. that, with the edge wake-ups removed  -> still dropped keys
//   4. the panel wired but painted exactly once, at boot, and never again
//                                            -> NO dropped keys
//
// Step 4 is the one that settles it. The peripheral was present and configured
// the whole time - pins held, interrupt registered, clock running - and it did
// drive the bus once. What changed between 3 and 4 is only that no transfer
// happens while anyone might be typing.
//
// So the conclusion is precise: having SPI2 is harmless, TRANSFERRING on it
// collides with Gazell's RADIO. Steps 1 and 2 already rule out "the loop was
// busy" as the mechanism, so this is not a scheduling problem and cannot be
// fixed by yielding more. The only lever is to not transfer while keys are in
// play.
//
// Hence the rule this module now implements: no transfer unless the half has been
// quiet for seconds, one frame per quiet stretch, and the ask is a single store so
// the typing loop never waits on it.
//
// WHERE THE CONTROLS CAME FROM, so they are not re-litigated:
//
//   * boot frame, radio idle, never repainted ..... no drops
//   * full frames repeatedly, mid-burst ........... drops
//   * full frames only at burst edges ............. drops, occasionally
//   * full frames only deep in quiet (this) ....... untested as of this build
//
// The third line is the one that pins it down. "Edges of a burst" is a very narrow
// class of moment, and it was still enough to cost keys - so the damage is done at
// the instant a burst begins, while Gazell is re-acquiring a link it had let lapse,
// not by the transfers in aggregate and not by the loop being busy (that was ruled
// out earlier: an async task that never blocks the loop still dropped keys).
//
// The occasional-not-always character is consistent with that: only a burst whose
// first packet lands inside the 3 ms sweep is exposed, and only a tap short enough
// to be swallowed by the resulting retry gap is actually lost.
//
// WHAT RECOVERING RUN WOULD TAKE, in the order worth trying:
//
//   1. paint only the RUN/IDLE glyph instead of the whole frame. It is 7 columns,
//      ~154 us, about a twentieth of the sweep. If duration is what matters - and
//      the boot frame's 3 ms with an idle radio suggests it is - this may be
//      survivable at burst start.
//   2. shorten Gazell's re-acquisition instead, so a perturbation cannot strand the
//      link for long. `timeslots_per_channel_when_device_out_of_sync` is 15 (the
//      library default, deliberately kept to match the reference transmitter), and
//      the timeslot period sits at the documented 1 Mbit minimum of 900 us. Both
//      are knobs on how expensive it is to be interrupted at the wrong moment.
//   3. a flush that abandons itself between columns, so a burst arriving mid-sweep
//      truncates the transfer instead of waiting it out. Weak on its own: while
//      idle the loop ticks every 10 ms, so the abort would be noticed far too late
//      to matter. It only helps combined with a faster wake-up.
//
// NOT DONE, and worth knowing before loosening anything: the flush is still one
// 3 ms sweep with no way to abandon it part-way, and while the half is quiet the
// main loop only looks at the matrix every 10 ms. So the residual exposure is
// "user resumes typing inside the 3 ms after a clear pause" - rare, but not zero.
//
// The cost is that a long typing session leaves the screen frozen on its last
// frame, and that the animation advances only during breaks. That is the
// intended trade: typing has the highest priority, and this is what buying it
// looks like.
//
// NOT DONE, and worth knowing before loosening anything: the flush is still one
// 3 ms sweep rather than something that can abandon itself part-way. A key
// pressed during those 3 ms is still exposed. The quiet window makes that
// vanishingly rare, but it is not zero, and if drops ever show up again under
// heavy typing that is the first place to look.
// ---------------------------------------------------------------------------

/// Tells the screen task to compose and paint a frame.
///
/// Carries the pointing device's state, and whether a full frame is wanted.
///
/// The state is here rather than read by the task because only the main loop knows
/// it. It is one of the few things on this half that changes WHILE a hand is on the
/// keyboard - a switch key is being pressed at the moment it changes - so it is the
/// one thing that cannot simply wait for the next quiet window. It is answered with
/// the band of rows the state line lives in rather than a whole frame.
///
/// A `Signal` rather than a channel because the screen only cares about the latest
/// request: if two arrive while a flush is in flight, painting the older one would
/// be work whose result is immediately overwritten. One slot, last write wins.
pub static UI: embassy_sync::signal::Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Request,
> = embassy_sync::signal::Signal::new();

/// What this half's own pointing device is doing, as far as the half can tell.
///
/// Three states and no more: the device is either unpowered, or it is on and in one
/// of its two modes. That is the whole truth available here - the device is on this
/// half, so its switch is known locally - which is what makes it honest to print.
/// The panel spells them `OFFLINE`, `POINTER MODE` and `SCROLL MODE`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PointerUi {
    Off,
    Move,
    Scroll,
}

/// A request to the screen task.
pub struct Request {
    /// What the panel should be showing.
    pub pointer: PointerUi,
    /// Whether a full frame is also due - the animation and the battery are only
    /// worth a full transfer in a quiet window, so this is `false` for a bare state
    /// change and `true` when the quiet edge comes round.
    pub full: bool,
}

/// How many conversions are averaged into one displayed percentage.
///
/// A one-shot SAADC reading at this gain is noisy by a couple of digits and the
/// panel shows a whole number, so a single conversion would visibly wobble.
/// Averaging costs a few conversions - in the screen task, which is not on the
/// typing path at all, so the cheap fix here is also the free one.
///
/// There is deliberately no rate-limiting filter on top, unlike the BLE firmware's
/// `shown_battery`: that existed because rmk published a sample every 30 s and
/// repainted on every 1% change. Here a frame is composed at most once per typing
/// break, so there is nothing left for a filter to suppress.
const BATTERY_SAMPLES: usize = 8;

/// Average a few conversions and turn them into a percentage.
async fn read_battery_percent(battery: &mut Battery, full_counts: i32) -> Option<u8> {
    let mut buf = [0i16; 1];
    let mut sum: i32 = 0;
    for _ in 0..BATTERY_SAMPLES {
        battery.sample(&mut buf).await;
        sum += buf[0] as i32;
    }
    renderers::percent_from_raw(sum / BATTERY_SAMPLES as i32, full_counts)
}

/// Own the panel and the battery sense, and paint when told to.
///
/// `salt` seeds the animation's starting frame, and it is how the two halves differ
/// on screen: the caller hands over `Role::Left.capy_salt()` or the right one. This
/// half has no way to tell which it is, and does not need one.
///
/// (For one evening - 2026-09-26 - this parameter was `&LEFT_ART`/`&RIGHT_ART`, a
/// fixed drawing per half. Lemon called that off the same night and the animation is
/// back. The drawings are parked in `_quarantine-2026-09-26/art-pictures/`.)
///
/// `label` names this half's own pointing device, for the state line.
/// `full_counts` is where this half's cell rests when its charger says it is full -
/// see `board::Role::battery_full_counts`. It differs between the halves, which is
/// the whole reason it is a parameter.
#[embassy_executor::task]
pub async fn task(
    mut screen: Screen,
    mut battery: Battery,
    salt: u32,
    label: &'static str,
    full_counts: i32,
) {
    screen.init().await;
    // One calibration, at boot. The ADC is read a handful of times an hour, so
    // there is nothing to gain from repeating this - and it is the only part of the
    // battery path that is not a plain conversion.
    battery.calibrate().await;

    // Boot frame, painted before any signal so the panel is never showing whatever
    // its controller RAM happened to hold. The state line starts at `Off` because
    // that is what the main loop starts at: cold boot leaves the device unpowered.
    let boot_level = read_battery_percent(&mut battery, full_counts).await;
    renderers::draw_ui(&mut screen, boot_level, salt, PointerUi::Off, label);
    screen.flush().await;

    let mut painted = PointerUi::Off;
    loop {
        let request = UI.wait().await;

        // A state change is sent the moment it happens, and answered with the band
        // the state line lives in rather than a whole frame - see `flush_rows`. It
        // cannot wait for a quiet window: the change IS a keypress.
        if request.pointer != painted {
            renderers::draw_pointer(&mut screen, request.pointer, label);
            screen
                .flush_rows(renderers::POINTER_BAND.0, renderers::POINTER_BAND.1)
                .await;
            painted = request.pointer;
        }

        if !request.full {
            continue;
        }

        // Read here rather than in the main loop: this is where the frame is being
        // composed, it is the only consumer of the value, and it keeps the ADC
        // entirely off the typing path.
        let level = read_battery_percent(&mut battery, full_counts).await;
        renderers::draw_ui(&mut screen, level, salt, request.pointer, label);
        // No comparison against what is already on screen: the caller decides when
        // a repaint is worth asking for, and the framebuffer hash inside the driver
        // is what stops a redundant request from reaching the bus.
        screen.flush().await;
    }
}