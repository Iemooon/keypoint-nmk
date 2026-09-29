//! The LEFT half's firmware.
//!
//! Everything it does is in the library; this file only says which role this
//! image is and which half's pins it drives. The same source builds both halves
//! (see `src/right.rs`), so the two images cannot drift apart - the only
//! differences are the ones board.toml describes: the Gazell pipe and the
//! channel subset.
//!
//! Built with `cargo build --release --bin left`, then converted to a UF2 and
//! dragged onto the half's BOOT drive.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use keypoint_nmk_keyboard::board::{self, Role};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = keypoint_nmk_keyboard::init_peripherals();
    let (pins, encoder, pointer, screen, battery, boot_seed) = board::left::pins(p);

    // Which capybara frame the picture starts on: drawn from the chip's RNG in
    // `pins()`. A picture that is the same at every switch-on is the one thing this
    // is meant to avoid.
    keypoint_nmk_keyboard::screen::renderers::set_boot_seed(boot_seed);

    // The panel and the battery sense get their own task so that the typing loop
    // never waits on either - see the note in `screen::mod` for why that separation
    // is not optional on this keyboard.
    //
    // `Role::Left.capy_salt()` is what keeps this half's animation out of step with
    // the other's: the screen task is told a number, never which half it is on.
    spawner.spawn(
        keypoint_nmk_keyboard::screen::task(
            screen,
            battery,
            Role::Left.capy_salt(),
            Role::Left.pointer_label(),
            Role::Left.battery_full_counts(),
        )
        .expect("screen task spawn"),
    );

    keypoint_nmk_keyboard::run(Role::Left, pins, encoder, pointer).await
}