//! The RIGHT half's firmware.
//!
//! See `src/left.rs` - the two files are the same three lines with a different
//! role, which is the point: the halves differ in exactly two board.toml values
//! (Gazell pipe and channel subset) and nowhere else.
//!
//! Built with `cargo build --release --bin right`, then converted to a UF2 and
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
    let (pins, encoder, pointer, screen, battery, boot_seed) = board::right::pins(p);

    // Which capybara frame the picture starts on: drawn from the chip's RNG in
    // `pins()`. A picture that is the same at every switch-on is the one thing this
    // is meant to avoid.
    keypoint_nmk_keyboard::screen::renderers::set_boot_seed(boot_seed);

    // See the note in `src/left.rs` - the panel and the battery sense run in their
    // own task on both halves. The salt is the one thing that differs between the two
    // screens.
    spawner.spawn(
        keypoint_nmk_keyboard::screen::task(
            screen,
            battery,
            Role::Right.capy_salt(),
            Role::Right.pointer_label(),
            Role::Right.battery_full_counts(),
        )
        .expect("screen task spawn"),
    );

    keypoint_nmk_keyboard::run(Role::Right, pins, encoder, pointer).await
}