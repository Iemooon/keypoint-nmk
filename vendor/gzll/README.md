# Vendored Nordic Gazell archive

The firmware links one precompiled static library: Nordic Semiconductor's Gazell
2.4 GHz protocol stack. Nothing else from the nRF5 SDK is needed - every symbol the
Rust side calls is declared by hand in `keyboard/src/gzll_ffi` and
`receiver/src/gazell.rs`, so no headers are required to build.

| file | chip | bytes | md5 |
|---|---|---|---|
| `gzll_nrf52840_gcc.a` | nRF52840 | 7082428 | f38ef14a6c56a2516a8d4bf773c9d25e |
| `gzll_nrf52_gcc.a`    | nRF52833 / nRF52832 | 5826684 | 7b168072545d485f605d805f6ec597ec |
| `license.txt`         | Nordic's notice, must travel with the binaries | 1956 | d2ca93a244fde7c208164989b0e28408 |

Provenance: nRF5 SDK **17.1.1**, `components/proprietary_rf/gzll/gcc/`, unmodified.
Taken from that directory rather than rebuilt, because the archive is closed-source.

## Why 17.1.1 and not something older

The version of this library was the last standing variable in a link-quality
investigation. SDK 12.3's nRF52 build services the second Gazell pipe unevenly, which
on this keyboard looked like intermittent stalls on one half only. Both firmware sides
are therefore built against the same SDK's library. See the `[gazell]` section of
`keyboard/board.toml` for the reasoning in the place where the parameters live.

## Licence terms that apply to this directory

`license.txt` (BSD-3-style, Nordic Semiconductor) permits redistribution in binary
form provided the copyright notice, the conditions list and the disclaimer are
reproduced with the distribution - hence this file and `license.txt` sit next to the
archives. Two further conditions bind users of this repository:

  * the software may only be used with a Nordic Semiconductor integrated circuit;
  * the binaries may not be reverse engineered, decompiled, modified or disassembled.

## Overriding this copy

`build.rs` on both sides looks at `GZLL_DIR` and `GZLL_LIB` first, so a different SDK
build can be tried without editing code or this directory. The default resolves to
this folder relative to the repository root.
