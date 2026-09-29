//! KeyPoint 2.4GHz receiver: Gazell matrix in, USB HID keyboard (+ Vial) out.
//!
//! One nRF52840 (PCA10059) doing both jobs:
//!   * the two halves' 6x8 matrices arrive over Nordic Gazell (2.4GHz,
//!     strictly one-way: transmitters -> this dongle)
//!   * the host sees one USB HID keyboard, with Vial for live keymap editing
//!
//! Architecture, and why:
//!
//!   * **USB is `rmk::usb::UsbTransport`**, not a hand-written stack. It is the
//!     stack already verified to enumerate on this exact dongle
//!     (VID_1313/PID_1208 with keyboard + mouse + Vial interfaces).
//!   * **No BLE at all.** The link is one-way Gazell, so MPSL and
//!     SoftDeviceController are dropped - which also hands RADIO to Gazell and
//!     frees the timeslot machinery. Both consequences are handled below.
//!   * **Input arrives as `KeyboardEvent`s.** RMK's own split driver does exactly
//!     this (`rmk/src/split/driver.rs`: validate row/col, then publish
//!     `KeyboardEvent::key(row + row_offset, col + col_offset, pressed)`), and
//!     the `Keyboard` processor turns those into HID reports. So Gazell plugs in
//!     at the same seam a split link would, via an `InputDevice`.
//!
//! Entered by a jump from the factory bootloader
//! ---------------------------------------------
//! `nrf_bootloader_app_start` sets VTOR and branches - it does NOT reset the
//! chip - so this application inherits peripheral and clock state. Two clock
//! facts follow, and both are handled in `prepare_clocks`:
//!
//! 1. **HFXO must be running before USBD is touched.** The nRF52840 USB
//!    controller needs the 32 MHz crystal (HFINT is good to a few percent, USB
//!    wants +/-0.25%), and enabling USBD while still on HFINT wedges it so that
//!    `Bus::enable` waits forever for `EVENTCAUSE.READY`. With MPSL present this
//!    was implicit (MPSL starts HFXO for the radio); without it we must ask.
//! 2. **The waits must be bounded.** Per the nRF52840 PS, `TASKS_HFCLKSTART` /
//!    `TASKS_LFCLKSTART` have *no effect - including no event* - when the
//!    oscillator already runs. The bootloader uses USB (so HFXO may be up) and
//!    app_timer (so LFCLK is up), and `embassy_nrf::init` waits for both events
//!    unconditionally. Unbounded waits there deadlock; see the comments on
//!    `prepare_clocks`.

#![no_std]
#![no_main]

mod board;
mod gazell;
mod keymap;
mod pointer_accel;
mod pointer_speed;
mod scroll_key;
mod speed_control;
mod vial;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::{self, Driver};
use embassy_nrf::{bind_interrupts, peripherals};
use panic_probe as _;
use rmk::config::{
    AutoMouseLayerConfig, BehaviorConfig, DeviceConfig, PositionalConfig, RmkConfig,
    StorageConfig, VialConfig,
};
use rmk::host::HostService;
use rmk::input_device::pointing::{PointingProcessor, PointingProcessorConfig};
use rmk::keyboard::Keyboard;
use rmk::processor::builtin::wpm::WpmProcessor;
use rmk::storage::async_flash_wrapper;
use rmk::usb::UsbTransport;
use rmk::AutoMouseLayerRunner;
use rmk::{KeymapData, initialize_keymap_and_storage, run_all};
use vial::{VIAL_KEYBOARD_DEF, VIAL_KEYBOARD_ID};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    // Gazell will claim RADIO / TIMER2 / SWI0_EGU0 once the receiver lands; see
    // gazell.rs. Nothing else in this build uses them (no MPSL/SDC).
});

/// Bring CLOCK into a state this application and `embassy_nrf::init` can cope
/// with, with every wait bounded. Must run before anything touches USBD.
fn prepare_clocks() {
    use embassy_nrf::pac::clock::vals::{HfclkstatSrc, Lfclksrc};

    let clock = embassy_nrf::pac::CLOCK;

    // ---- 1. HFXO: required for USB -----------------------------------------
    // Only skip if genuinely already running *from the crystal*: HFCLKSTAT.STATE
    // reads 1 even when the source is HFINT, so the source is what matters.
    let st = clock.hfclkstat().read();
    if !(st.state() && st.src() == HfclkstatSrc::Xtal) {
        clock.events_hfclkstarted().write_value(0);
        clock.tasks_hfclkstart().write_value(1);
        // Bounded: if HFXO were already up this event would never arrive, and
        // an unbounded wait here is a hang before USB is ever reached.
        let mut spins: u32 = 0;
        while clock.events_hfclkstarted().read() == 0 {
            spins += 1;
            if spins > 20_000_000 {
                info!("HFXO did not report started; continuing anyway");
                break;
            }
        }
    }

    // ---- 2. LFCLK: stop it, and WAIT for the stop --------------------------
    // app_timer in the bootloader starts LFCLK and does not stop it. embassy's
    // init then does `events_lfclkstarted = 0; tasks_lfclkstart = 1; while !event`
    // and that task has no effect while LFCLK already runs from the selected
    // source - so the wait never completes. Stopping first makes embassy's start
    // a real 0 -> 1 edge. The stop must be waited out: issuing LFCLKSTART while
    // the stop is still settling is ignored.
    clock.tasks_lfclkstop().write_value(1);
    let mut spins: u32 = 0;
    while clock.lfclkstat().read().state() {
        spins += 1;
        if spins > 20_000_000 {
            info!("LFCLK did not report stopped; continuing anyway");
            break;
        }
    }
    clock.lfclksrc().write(|w| w.set_src(Lfclksrc::Rc));
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("keypoint-nmk-receiver: starting");

    // Interrupt priorities as in the upstream non-BLE nrf52840 example: USB and
    // CLOCK_POWER above the gpiote/time drivers.
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P3;
    config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P3;
    embassy_nrf::interrupt::USBD.set_priority(embassy_nrf::interrupt::Priority::P2);
    embassy_nrf::interrupt::CLOCK_POWER.set_priority(embassy_nrf::interrupt::Priority::P2);

    // Regulator modes are NOT the chip's reset defaults when a UF2 bootloader
    // hands over: the Adafruit bootloader switches REG1 to DC/DC before jumping
    // into the application, and DC/DC needs the external inductor that not every
    // nice!nano revision populates. A firmware that never touches these
    // registers therefore runs in a mode it never chose - on a board without
    // that inductor the supply can collapse the moment the radio or USB draws
    // current, and the board looks "written successfully, then completely dead".
    //
    // The values come from board.toml's `[power]` section, and the defaults
    // stated in the profiles follow RMK's own chip defaults for nRF52840
    // (`dcdc_reg0 = true`, `dcdc_reg1 = true` in
    // rmk/examples/use_config/nrf52840_ble/keyboard.toml), which is the
    // configuration known to work on these boards. "keep" leaves a register
    // alone, which is what every profile here did before the option existed.
    //
    // UICR.REGOUT0 (REG0's output voltage) is deliberately not touched: RMK warns
    // that changing it requires bootloader >= 0.10.0, and these boards ship 3V3.
    if board::REG1_FORCE_LDO {
        embassy_nrf::pac::POWER.dcdcen().write(|w| w.set_dcdcen(false));
    } else if board::REG1_FORCE_DCDC {
        embassy_nrf::pac::POWER.dcdcen().write(|w| w.set_dcdcen(true));
    }
    // REG0 (the high-voltage regulator) only exists on nRF52840 - nRF52833 has no
    // DCDCEN0 register in its PAC, which is why build.rs emits these two
    // constants for nRF52840 only.
    #[cfg(feature = "nrf52840")]
    if board::REG0_FORCE_LDO {
        // The field inside the PAC's Dcdcen0 register is still named DCDCEN.
        embassy_nrf::pac::POWER.dcdcen0().write(|w| w.set_dcdcen(false));
    } else if board::REG0_FORCE_DCDC {
        embassy_nrf::pac::POWER.dcdcen0().write(|w| w.set_dcdcen(true));
    }

    // Clocks before USB: HFXO on (bounded), LFCLK stopped and settled.
    prepare_clocks();

    let p = embassy_nrf::init(config);
    info!("peripherals up, HFXO requested");

    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

    // Internal flash holds the keymap/Vial storage. The address comes from
    // board.toml, and build.rs refuses to build if it overlaps the application
    // region in memory.x (writing storage over live code is the one mistake here
    // that fails in a confusing way) or reaches into the bootloader.
    let flash = async_flash_wrapper(Nvmc::new(p.NVMC));
    let storage_config = StorageConfig {
        start_addr: board::STORAGE_START_ADDR,
        num_sectors: board::STORAGE_NUM_SECTORS,
        // Back to `false`, which is where it belongs: the stored layout is the
        // user's, and resetting it on every boot throws away whatever was set up in
        // Vial.
        //
        // It needed to be reset exactly once. RMK reads the keymap AND the encoder
        // map back out of flash, and what was stored there - written by the BLE
        // build this one replaced, which shares the same storage - gave the encoder
        // nothing to do. One boot with `clear_layout` wrote the compiled-in map
        // over it, and the knob now runs on that; with the flag off the same map
        // simply stays in place, untouched.
        //
        // Nothing after this blanks the encoder map either: Vial does not know this
        // keyboard has encoders (`vial.json` declares none), so it never sends an
        // encoder action in the first place. If the knob is ever dead again, that
        // assumption is the first thing to check.
        clear_layout: false,
        ..Default::default()
    };

    let rmk_config = RmkConfig {
        device_config: DeviceConfig {
            vid: board::VID,
            pid: board::PID,
            manufacturer: board::MANUFACTURER,
            product_name: board::PRODUCT_NAME,
            ..DeviceConfig::default()
        },
        // Vial's unlock policy.
        //
        // `insecure: true` makes HostLock::is_unlocked() unconditionally true,
        // so nothing ever has to be unlocked: keymap edits and the Vial matrix
        // tester both work straight away. (`VialConfig::new()` hardcodes
        // `insecure: false`, which is why the struct is built directly here.)
        //
        // `unlock_keys` is nevertheless a real combination rather than empty.
        // The Vial GUI runs an unlock handshake before physical-presence actions
        // - its "enter bootloader" button above all - and this is the list of
        // positions it shows as "the keys to press". Empty means an
        // unsatisfiable prompt, 0xFF-filled key positions in the reply, and a
        // "No unlock keys provided" warning from the firmware. Because
        // `insecure` already reports the keyboard as unlocked, these keys never
        // actually have to be pressed; they only make the protocol well-formed.
        //
        // Positions are matrix (row, col) with 12 ROWS: 0-5 left half, 6-11
        // right half (KeyPoint's halves are joined along ROWS - see
        // board.rs::logical_row). Below is the left half's top row, first two
        // keys - one half on purpose, so unlocking never needs both
        // transmitters awake.
        vial_config: VialConfig {
            vial_keyboard_id: VIAL_KEYBOARD_ID,
            vial_keyboard_def: VIAL_KEYBOARD_DEF,
            unlock_keys: &[(0, 0), (0, 1)],
            insecure: true,
        },
        ..Default::default()
    };

    let mut keymap_data = KeymapData::new_with_encoder(
        keymap::get_default_keymap(),
        keymap::get_default_encoder_map(),
    );
    // NO tap-hold timings are set here, deliberately (decided 2026-09-28): the
    // firmware carries no initial values of its own, so Vial is the single source
    // of truth for tapping term, flow tap and the hold flavour.
    //
    // `BehaviorConfig::default()` is therefore what a board runs on until the
    // first Vial edit - RMK's stock values, not ours: tapping term (`
    // default_profile.hold_timeout`) 250 ms, flow tap OFF, `prior_idle_time`
    // 120 ms, mode Normal. Setting these here was tried and removed; note that it
    // would only ever have applied to a board with empty flash anyway, because
    // RMK restores the persisted `BehaviorConfig` over this at boot
    // (rmk/src/host/storage.rs:110-115 assigns `morse.prior_idle_time` and the
    // whole `morse.default_profile`).
    //
    // Field names, since they mislead and this cost a whole diagnosis:
    //   * `default_profile.hold_timeout` - the TAPPING TERM, ZMK's
    //     `tapping-term-ms`. Vial's "Tapping Term".
    //   * `prior_idle_time`              - the FLOW-TAP WINDOW, ZMK's
    //     `flow-tap-term-ms`. Vial's "Flow Tap". Read only when flow tap is on
    //     (rmk/src/config/behavior.rs; used at keyboard.rs:715).
    //   * `enable_flow_tap`              - the switch. Resolves per-key profile ->
    //     default profile -> config level (keyboard/morse.rs:410-412), so a
    //     `Some(false)` in the default profile would outrank the config flag.
    //     `MorseProfile::new`'s first argument is `unilateral_tap`, not flow tap.
    //
    // Also note keyboard.toml's `[behavior.morse]` is NOT read by this project:
    // rmk-types/build.rs turns that file into compile-time CAPACITIES only
    // (COMBO_MAX_NUM, MORSE_MAX_NUM, DEBOUNCE_THRESHOLD,
    // AUTO_MOUSE_LAYER_MAX_NUM, event subs). Timings never came from there.
    let mut behavior_config = BehaviorConfig::default();

    // Which layer the auto mouse runner enters, and when it leaves.
    //
    // This has to be set explicitly, and this is the reason the mouse layer looked
    // dead: `AutoMouseLayerConfig::default()` has `target_layer: 0`. Left alone, the
    // runner does exactly what it is told - it activates the base layer on pointer
    // motion, which changes nothing visible, so nothing ever reaches layer 4 and the
    // scroll keys never find themselves armed.
    //
    // Per device rather than one entry, because the two timeouts differ by design.
    // The pad reports ~40 packets/s while a finger is down, so 500 ms of silence is
    // clearly a stop. A nub push can be 1-2 counts in a burst with much longer gaps,
    // so the same 500 ms would drop the layer between micro-movements of a single
    // gesture; 1000 ms covers one. Both keep `threshold: 1`, the minimum, because a
    // light nub push is only ever 1-2 counts.
    //
    // `deactivate_on_key: false` is deliberate: a click must not exit the layer, or
    // a move-then-click would leave the button press landing on the typing layout
    // instead of the mouse buttons.
    behavior_config.auto_mouse_layer = {
        let mut entries = heapless::Vec::new();
        entries
            .push(AutoMouseLayerConfig {
                device_id: Some(gazell::TRACKPAD_ID),
                target_layer: scroll_key::MOUSE_LAYER,
                timeout: embassy_time::Duration::from_millis(500),
                threshold: 1,
                deactivate_on_key: false,
                extra_mouse_keys: &[],
                reset_timeout_on_key: false,
            })
            .ok();
        entries
            .push(AutoMouseLayerConfig {
                device_id: Some(gazell::TRACKPOINT_ID),
                target_layer: scroll_key::MOUSE_LAYER,
                timeout: embassy_time::Duration::from_millis(1000),
                threshold: 1,
                deactivate_on_key: false,
                extra_mouse_keys: &[],
                reset_timeout_on_key: false,
            })
            .ok();
        entries
    };

    let per_key_config = PositionalConfig::default();

    let (keymap, mut storage) = initialize_keymap_and_storage(
        &mut keymap_data,
        flash,
        &storage_config,
        &mut behavior_config,
        &per_key_config,
    )
    .await;
    info!("keymap + storage initialised");

    let mut keyboard = Keyboard::new(&keymap);
    let host_service = HostService::new(&keymap, &rmk_config);
    let mut usb_transport = UsbTransport::new(driver, rmk_config.device_config).with_host_service(&host_service);
    let mut wpm_processor = WpmProcessor::new();

    // NOTE: no hardware watchdog. RMK's `Nrf52Watchdog` is gated behind the nRF
    // BLE features (`_nrf_ble`), which this build deliberately does not enable,
    // so there is no runner to hand `p.WDT` to. If a watchdog is wanted here it
    // has to be a small local one; leaving it out is also the safer bring-up
    // choice, since a watchdog reset is indistinguishable from a crash.

    // ---------------------------------------------------------------------
    // Input source: the Gazell 2.4GHz receiver.
    //
    // `init` configures and enables the radio; from then on the Nordic library
    // is interrupt driven (TIMER2/RADIO/SWI0_EGU0) and this device only turns
    // received packets into `KeyboardEvent`s. That is the same seam RMK's own
    // split link publishes at, so everything downstream - keymap, Vial, USB - is
    // the combination already verified to work on this dongle.
    //
    // `gazell::test_matrix` was the bring-up placeholder that pulsed F13; it
    // proved the USB/keymap/report path in isolation and has been removed.
    // ---------------------------------------------------------------------
    let mut matrix = gazell::GazellMatrix::new();
    matrix.init();

    // The pointing processor lives here on the receiver, deliberately. The halves
    // put nothing but raw displacement on the air; everything that decides what a
    // movement MEANS - scaling, acceleration, and whether the stick drives the
    // cursor or the wheel - is a keyboard-side setting read from the keymap. Only
    // this side has the keymap, and only this side has USB to report through.
    //
    // Built with the same device id the right half publishes under; that number is
    // what pairs the two.
    let mut trackpoint_processor = PointingProcessor::new(
        &keymap,
        PointingProcessorConfig {
            device_id: gazell::TRACKPOINT_ID,
            ..Default::default()
        },
    );
    trackpoint_processor.set_pointing_mode(pointer_speed::cursor_mode(gazell::TRACKPOINT_ID));

    // One processor per device, because everything that shapes a movement is a
    // property of the DEVICE rather than of the pointer in general: the two need
    // different speed curves (a thumb on a nub moves far less than a finger on a
    // pad) and only one of them is the pad the scroll key is meant to drive. That
    // pairing is the `device_id`, and it has to match what the half publishes
    // under.
    let mut trackpad_processor = PointingProcessor::new(
        &keymap,
        PointingProcessorConfig {
            device_id: gazell::TRACKPAD_ID,
            ..Default::default()
        },
    );
    // `cursor_mode` carries the pad's X inversion, so this line is also what stops
    // the pad moving left when the finger goes right.
    trackpad_processor.set_pointing_mode(pointer_speed::cursor_mode(gazell::TRACKPAD_ID));

    // Enters the mouse layer (layer 4) on pointer motion and times back out when the
    // pointer goes idle. That layer is what arms the scroll keys - and it is also
    // why the layer exists in the keymap at all: without a runner nothing would ever
    // activate it, so the layer would be dead weight.
    let mut auto_mouse = AutoMouseLayerRunner::new(&keymap);
    // Turns a held thumb key into a wheel for that half's device.
    let mut scroll_controller = scroll_key::ScrollKeyController::new(&keymap);
    // Copies the six Vial-editable tier cells into the speed tables.
    let mut speed_controller = speed_control::SpeedController::new(&keymap);

    info!("entering run loop");
    run_all!(
        storage,
        usb_transport,
        wpm_processor,
        keyboard,
        matrix,
        trackpoint_processor,
        trackpad_processor,
        auto_mouse,
        scroll_controller,
        speed_controller
    )
    .await;
}
