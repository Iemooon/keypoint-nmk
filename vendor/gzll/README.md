# Vendored Nordic Gazell archive

The firmware links one precompiled static library: Nordic Semiconductor's Gazell
2.4 GHz protocol stack. Nothing else from the nRF5 SDK is needed - every symbol the
Rust side calls is declared by hand in `keyboard/src/gzll_ffi` and
`receiver/src/gazell.rs`, so no headers are required to build.

| file | role in this repository | bytes | md5 |
|---|---|---|---|
| `gzll_nrf52840_gcc.a` | **linked by every build** - both transmitter halves and all three receiver profiles | 7082428 | f38ef14a6c56a2516a8d4bf773c9d25e |
| `gzll_nrf52_gcc.a`    | Nordic's generic nRF52 build. **Linked by nothing** - kept so that `GZLL_LIB=` has something to point at without a download | 5826684 | 7b168072545d485f605d805f6ec597ec |
| `license.txt`         | Nordic's notice, must travel with the binaries | 1956 | d2ca93a244fde7c208164989b0e28408 |

### One archive, whatever the chip

The name reads like a restriction it does not impose: `gzll_nrf52840_gcc.a` is what
everything links, including the nRF52833 receiver profile. Neither build picks an
archive by chip - `keyboard/board.toml` names it (`library`) and `receiver/build.rs`
defaults to it - so there is no second code path in use, and the table above says so
instead of implying a mapping that does not exist.

What that pairing costs on nRF52833 silicon is **untested, in this project**: no 52833
board has been flashed with this firmware. If Nordic's generic nRF52 build is ever
wanted for it, `GZLL_LIB=gzll_nrf52_gcc.a` is the knob, and the provenance check below
reads either archive the same way.

### Removing the unused archive: what it would and would not save

`git rm gzll_nrf52_gcc.a` shrinks the checkout and the source archives GitHub generates
per tag. It does **not** shrink a clone: the blob is already in `main`'s history
(`57617b8c5ca982e85e8e52c88c106fd656898ae5`), so a clone keeps downloading it until
history is rewritten and force-pushed. Anyone doing the removal also needs to repoint
`receiver/tools/gzll_provenance.py` and `receiver/tools/gzll_syms.py`, which still name
this file.

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
