# keypoint-nmk — a 2.4GHz-only firmware pair for the KeyPoint keyboard

[![build](https://github.com/Iemooon/keypoint-nmk/actions/workflows/build.yml/badge.svg)](https://github.com/Iemooon/keypoint-nmk/actions/workflows/build.yml)

Two firmwares that together replace the keyboard's BLE link with a Nordic Gazell
(2.4GHz) link. No radio mode switch, no BLE, no USB on the halves: **2.4GHz only**.

```
        left half                  right half
   (nRF52840, 6x8 matrix)     (nRF52840, 6x8 matrix)
            |                          |
            |  Gazell device            |  Gazell device
            |  pipe 0, ch {15,47,71}    |  pipe 1, ch {31,57,81}
            |                          |
            +-----------+--------------+
                        |
                        v
            receiver  (nRF52840 dongle, PCA10059)
                  Gazell host, ch {15,31,47,57,71,81}
                        |
                        v
                  USB HID keyboard + Vial
```

## What is where

| Directory | What it is | Chip / delivery |
|---|---|---|
| `keyboard/` | **Transmitter half.** Scans its own 6x8 matrix, transmits the snapshot over Gazell, drops to a 10 ms poll with the radio off when idle. No keymap, no USB, no BLE, no RMK. | nRF52840 half, app at `0x1000`, flashed as **UF2** (drag onto the `BOOT` drive) |
| `receiver/` | **Receiver.** Gazell host; merges both halves into one 12x8 matrix and publishes it to RMK as key events; USB HID keyboard with Vial. | nRF52840 dongle (PCA10059), app at `0x1000`, flashed with **nRF Connect Programmer** (`.hex`) |

Both crates are independent: separate `Cargo.toml`, separate `target/`, built
separately. They share nothing at compile time — only the numbers in their
`board.toml` files, which must agree on the radio side.

## Build

Prerequisites, all of them public:

* Rust **1.98.0**, pinned per crate by `rust-toolchain.toml`. This is not
  bookkeeping: the first CI run used the runner's `stable` (1.98.1) and every image
  came out byte-different from the firmware actually flashed on the keyboard — same
  flash spans, same behaviour, different codegen. Pinning makes a green CI run mean
  "this is the same firmware".
* the `thumbv7em-none-eabihf` target (installed automatically by rustup)
* `cargo install --locked cargo-binutils@0.4.0 cargo-hex-to-uf2@0.1.2 flip-link@0.1.12`
* `arm-none-eabi-gcc` **only for `receiver`** — RMK's crypto dependency
  (`p256-cortex-m4-sys`) shells out to a C compiler. On Linux:
  `sudo apt install gcc-arm-none-eabi`. On Windows the project was developed with
  QMK_MSYS's copy; the build scripts add it to `PATH` when it is present.
* Nothing to download for the radio stack: Nordic's precompiled Gazell archive is
  vendored under `vendor/gzll/` together with its `license.txt` (see the README
  there for provenance and redistribution terms). `GZLL_DIR`/`GZLL_LIB` override it
  if you want to link a different SDK's build; the build scripts fail loudly rather
  than silently skipping the archive.

```bat
keyboard\tools\build.cmd              :: -> keypoint-nmk-left.uf2, keypoint-nmk-right.uf2
receiver\tools\build.cmd              :: -> keypoint-nmk-receiver.hex (+ .uf2)
receiver\tools\build-variants.cmd     :: -> all three receiver boards, then verifies them
```

The scripts exist to put the ARM toolchain on `PATH` and to run the
`objcopy`/`hex-to-uf2` steps in the right order; on a machine where the toolchain is
already reachable they are plain convenience.

Plain cargo works too, from inside each crate directory:

```bat
cd keyboard && cargo build --release
cargo objcopy --release --bin left -- -O ihex keypoint-nmk-left.hex
cargo hex-to-uf2 --input-path keypoint-nmk-left.hex --output-path keypoint-nmk-left.uf2 --family nrf52840
```

`.github/workflows/build.yml` runs exactly these steps on every push and publishes
the images as build artifacts.

## Reproducibility — what CI does and does not prove

CI builds the same source, with the same pinned compiler, from the same vendored
Gazell archive. It does **not** reproduce the developer's image byte for byte, and
no amount of pinning will: `rustc` embeds absolute paths of dependency sources into
panic and assertion-location strings, so the same code compiled on two hosts differs
wherever those strings sit.

Measured on the receiver image (both sides rustc 1.98.0, commit `88d9e12ae`, same rmk
checkout `f12257e`):

```
developer machine : C:\Users\lemon\.cargo\registry\src\...   (assert paths)
GitHub runner     : /home/runner/.cargo/registry/src/...     (51 hits)
result            : 162,796 B vs 162,692 B - same flash span, 104 B shorter
```

So the workflow asserts what is actually invariant and skips what is not:
`tools/verify_variant.py` checks each image's start address, chip family byte and
flash span against the board it claims to be, and that two boards sharing a layout
produce the *same* bytes — which is the property that matters when one image is
meant to flash two different 52840 boards. Hash equality against a laptop is not a
meaningful gate, and pretending otherwise would produce a red check that means
nothing.

If bit-reproducible images are ever wanted, the lever is `-C remap-path-prefix` over
the cargo home and workspace, applied identically on every machine that builds
releases - not a CI-only flag, or CI and local builds would then differ by design.

## Flashing

* **Halves (`keyboard`)**: double-tap the half's reset button (or hold it while
  plugging the half in). A `BOOT` drive appears; drag `keypoint-nmk-left.uf2`
  onto the left half and `keypoint-nmk-right.uf2` onto the right one. **The two
  images are not interchangeable** — they differ in Gazell pipe and channel
  subset, which is how the receiver tells the halves apart. Flashing the left
  image onto the right half produces a keyboard whose right half never types.
* **Dongle (`receiver`)**: nRF Connect Programmer → the dongle → *Add file* →
  `keypoint-nmk-receiver.hex` → *Write*. Application base address is `0x1000`
  (measured, not assumed: the working images are continuous from there).

## The channel plan — do not "harmonise" it

This keyboard and the other 2.4GHz keyboard in this workspace (`nmk`, a flat
4x12) must not share a channel, or they steal each other's packets when both are
on:

| | host table | left half | right half |
|---|---|---|---|
| **keypoint-nmk** (this project) | `[15, 31, 47, 57, 71, 81]` | pipe 0, `[15, 47, 71]` | pipe 1, `[31, 57, 81]` |
| `nmk` (the other keyboard) | `[13, 27, 43, 53, 67, 77]` | pipe 0, `[13, 43, 67]` | pipe 1, `[27, 53, 77]` |

The two sets share no channel. Each half transmits on its own subset, and the
receiver listens on their union — which is what makes the receiver's rotation
(6 channels x 2 timeslots = 12) shorter than a searching half's dwell per channel
(15 timeslots), so a half that has just woken up always meets the host before
moving on. That arithmetic is spelled out in `receiver/board.toml` and `receiver/src/board.rs`.

## Single direction, on purpose

Data flows **halves -> dongle only**. Gazell still acknowledges packets at the
protocol level (that is how the transmitter knows to retry), but the ACK's
payload slot is unused and nothing in either firmware reads it. There is no
layer display, no host-to-half configuration, no battery reporting back. That was
a decision, not an omission: it keeps the halves' radio work to "send my matrix"
and removes a whole class of idle wake-ups.

## Idle behaviour

* A half scans at 1000 Hz while keys are in use, and sends whenever its state has
  been stable for 5 ticks - and keeps re-sending while it stays stable. The
  repetition is what keeps the receiver's link alive while a key is held.
* Once nothing has been pressed for `active_release_ms` (200 ms) it drops to
  `idle_poll_ms` (10 ms) and switches the radio off; the first press wakes both.
* **The chip is never powered down any more.** The first version entered
  SYSTEMOFF after 0.5 s, as the original transmitter does, and that was wrong on
  this keyboard for a reason worth keeping written down: SYSTEMOFF is exited by a
  *reset*, so every key pressed after a pause paid for the MBR, the UF2
  bootloader, the clocks, the radio init and re-acquiring the host; and because
  the matrix is sampled as a *level* rather than captured as an event, a tap
  shorter than that restart was not delayed but LOST - by the time the chip was
  running again the key had already been released and no scan ever saw it.
  Measured symptom: fast typing felt perfect, and every pause cost its first
  keystroke ("I have to press a key several times").
* What made that trade look necessary was a wrong assumption about where the
  current goes. SYSTEMOFF is ~1 uA; the 1 kHz polling loop it replaced is worth
  tens of uA. Dropping to 10 ms polls with the radio off lands in the same order
  of magnitude as SYSTEMOFF, while a press is still caught within one poll period
  and the radio is already re-acquiring by the time the key is debounced.
* Dials, all in `keyboard/board.toml`: `idle_poll_ms` (detection latency against idle
  current), `active_release_ms` (how long a typing pause may last before the half
  drops back to polling), and `[gazell] radio_off_when_idle` (default true; false
  keeps the radio listening, saving the 1..11 ms re-acquisition at the cost of the
  periodic receive current).

## Vial: this receiver *is* the keyboard

`receiver/vial.json` is carried over unchanged from the BLE dongle firmware
(`keypoint-rmk-dongle`), and the Vial keyboard ID is the same
(`B9BC09B29D374CEA`), as are VID/PID (`0x1313:0x1208`). Vial identifies a
keyboard by that ID plus the matrix shape and layout, so **saved layouts keep
working** — and the logical matrix is the same 12x8 the BLE dongle reports.

The halves are joined along **rows**: left = rows 0..5, right = rows 6..11. (A
flat 4x12 keyboard such as `nmk` joins along columns instead; mixing the two up
does not fail to compile, it silently reassigns which physical key each keymap
position means.) See `receiver/src/board.rs::logical_row`.

## Porting to another keyboard

One file per side, and nothing else:

* `keyboard/board.toml` — chip, flash/RAM layout, matrix size, diode direction, both
  halves' pins, both halves' pipe + channel subset, Gazell parameters, timings.
* `receiver/board.toml` — chip, layout, matrix size, the host's channel table, USB/Vial
  identity, storage placement.

`build.rs` on each side turns that into Rust constants, the pin constructors, the
wake-up wiring and `memory.x`. A wrong number is a build error, not firmware that
misbehaves over the air: matrix size vs pin count, chip vs cargo feature, flash
region vs chip size, two halves on the same pipe, two halves sharing a channel,
`vial.json` vs board geometry, VID/PID vs board — all checked.

Chip support: `nrf52840` (default) / `nrf52833` / `nrf52832` features on `keyboard`
(`nrf52840` / `nrf52833` on `receiver`, which needs a USB controller). The Gazell
archive is selected by `[gazell] library` in `keyboard/board.toml`.

## Verification status — read this before trusting anything

**Verified here:**

* Both firmwares compile clean (no warnings on `keyboard`; `receiver` has none left that matter).
* Images are structurally sane: `tool`-checked start addresses (`0x1000`,
  i.e. the app slot, not the chip base), initial SP inside the declared RAM,
  reset vector in range, contiguous span, UF2 family `0xada52840`, app ends below
  both bootloaders' regions.
* The Gazell symbol contract is satisfied on both sides - the archive's ISR
  handlers and our four callbacks are actually linked, not merely declared
  (`arm-none-eabi-nm` on both ELFs).
* Sizes: `keyboard` 25.5 KB (2.5% of its 1020K slot), `receiver` 142.8 KB (22% of 636K).
* **Flashed and typed on (2026-09-16):** both halves link, every key in the 12x8
  matrix lands on the right HID key, and either half types on its own. Polarity,
  column order, the row-wise 12x8 merge, the channel tables and the Vial mapping
  are therefore confirmed on hardware, not only on paper.

**NOT verified:** radio range, and coexistence with the neighbouring 2.4 GHz
keyboard (`planckduos-nmk`) while both are powered, have not been measured - the
zero-intersection channel tables are the argument, not a measurement. Nor has the
idle current of the new shallow sleep been on a meter: SYSTEMOFF is ~1 uA and this
is expected to land in the tens of uA, which is small against a Li-ion's own
self-discharge, but "expected" is not "measured". Remaining suspect list if
something misbehaves: wrong diode direction or swapped pins for one half
(`keyboard/board.toml`), the two halves' images flashed the wrong way round, and only
then the channel table.

## Not in this step

The halves also carry a display, a touchpad (left), a trackpoint (right), an
encoder and battery sense. None of that is handled yet — this is the
keyboard-only step. The encoder is cheap when it comes (it can ride along as
extra matrix cells, which is how the original firmware's knob variant does it);
the pointer devices and the display need a packet-type byte in the payload, and
the display would reintroduce a host-to-half direction that was deliberately
dropped here.