# receiver — the KeyPoint 2.4GHz receiver

Gazell host in, USB HID keyboard (with Vial) out, on any of three boards:
the Nordic nRF52840 Dongle (PCA10059), an nRF52840 nice!nano, and an nRF52833
nice!nano (the board its owner calls "blue macro"). Build, flash and
channel-plan notes are in the project README one level up; this file is only
about what is specific to this crate.

Built from `nmk`'s `gazell-dongle` (same receiver role, same library, same
parameters) re-aimed at this keyboard's shape.

## What differs from the `nmk` receiver it was copied from

| | `nmk` | here |
|---|---|---|
| Logical matrix | 4 rows x 12 cols, halves joined along **columns** | 12 rows x 8 cols, halves joined along **rows** (`keypoint-rmk-dongle`'s `row_offset: 6`) |
| Per-half payload | 4 bytes (one per row) | 6 bytes (one per row) |
| Host channel table | `[13,27,43,53,67,77]` | `[15,31,47,57,71,81]` — shares no channel with `nmk` |
| Vial definition | generated from geometry | carried over verbatim from `keypoint-rmk-dongle/vial.json` |

The halves' raw bytes are stored in `RAW`, which is a pair of `AtomicU32`s per
half (6 rows do not fit in one word). A report is therefore written as two
stores; a reader can catch the pair half-updated. That is deliberate and
harmless — a report is repeated every 5 ms while any key is down, and RMK's
debouncer only accepts a change it sees sustained. See `src/gazell.rs`.

## Layout

```
board.toml             the keyboard and the link: single source of truth
boards/*.toml          per-board overrides, [chip]/[memory]/[flash]/[storage] only
build.rs               board.toml -> board_generated.rs + memory.x; checks vial.json
vial.json              the Vial definition, unchanged from the BLE dongle firmware
src/main.rs            wiring: radio in, RMK + USB out
src/board.rs           derived geometry (ROW/COL, row joining) and the link-timing notes
src/gazell.rs          Gazell FFI, RX callbacks, matrix merge, key events
src/keymap.rs          factory default keymap (12x8, 5 layers)
tools/                 build.cmd (PCA10059 only) and build-variants.cmd (all
                       three), plus the hex/uf2/family checks used to validate
                       images
```

`src/keymap.rs` matters only on a freshly erased board or after Vial's "reset to
firmware default" — the real layout lives in flash and is shared with the BLE
dongle firmware via the common keyboard ID.

## The three boards

One source tree, three layouts. What differs is only what the bootloader and the
chip decide, and it lives in one override file per board under `boards/` — except
for PCA10059, which *is* the default layout in `board.toml` and therefore gets no
override file at all. `board.toml` stays the single source of truth for the
keyboard and the link, so the matrix, the channel table, the USB identity and the
storage semantics cannot drift between the boards.

| Board | `flash_origin` | App region | Storage | Bootloader | Flash with |
|---|---|---|---|---|---|
| Nordic nRF52840 Dongle (PCA10059) | `0x1000` | 636K | `0xA0000`+24K | Nordic USB DFU at `0xE0000` | nRF Connect Programmer (`.hex`) |
| nRF52840 nice!nano | `0x1000` | 636K | `0xA0000`+24K | Adafruit UF2, nosd layout | drag the `.uf2` |
| nRF52833 nice!nano ("blue macro") | `0x27000` | 260K | `0x68000`+48K | Adafruit UF2, SoftDevice layout | drag the `.uf2` |

```
tools\build-variants.cmd        build all three, then run tools\verify_variant.py
```

The nRF52833 dongle (PCA10100 / the bare 52833 dev board with no bootloader) was
dropped on 2026-09-26 — not needed, and nothing else depends on it.

Points that matter:

* **The two nRF52840 boards produce the same image, byte for byte.** They differ
  only in where the bootloader's reserved region starts (`0xE0000` on the Nordic
  dongle,
  `0xF4000` on the Adafruit board), which sits above the storage and appears
  nowhere in the code. One image therefore serves both — and, since both storage
  regions are identical too, one Vial layout works on either.
* **`0x1000`, not `0x26000`, for the nRF52840 nice!nano.** That board's own
  `INFO_UF2.TXT` reports "SoftDevice: not found": it carries the *nosd* Adafruit
  bootloader, which starts the application at `0x1000`. A SoftDevice-layout
  image at `0x26000` flashes fine and then never runs.
* **nRF52833 is a different part, not just a smaller one**: half the flash (512K)
  and half the RAM (128K), so the storage region is re-placed rather than shrunk.
* **`reserved_top = 0x74000` on the nRF52833 nice!nano is not measured** — it is
  the top 48K by analogy with the nRF52840 board, and the storage is placed right
  up against it. Check with `nrfutil device fw-info --serial-number <sn>`; if the
  bootloader is larger than 48K, storage would write into it.
* The Vial keyboard ID is the same on all three (it identifies the keyboard, not
  the chip), but **the storage address differs on the nRF52833 boards**, so a
  `.vil` must be re-loaded once when moving between chip families. The sector
  counts differ too (24K on both nRF52840 boards — RMK's floor, kept identical on
  purpose — against the room a 512K part leaves on the nRF52833 boards).
* Flashing an nRF52840 image onto an nRF52833 is refused by the UF2 family ID
  (`0xADA52840` vs `0x621E937A`); `verify_variant.py` checks that, along with
  each image's start address and its distance from the storage region.