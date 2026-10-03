# keyboard — the KeyPoint 2.4GHz transmitter halves

One crate, two binaries (`left`, `right`), one flashable UF2 each. Scans a 6x8
matrix, transmits the snapshot over Nordic Gazell, and drops to a low-power poll when idle. Build
and flashing notes are in the project README one level up.

## Deliberately not built on RMK

The keyboard's brain — keymap, layers, Vial — is in the receiver, exactly as in
the original redox design and in RMK's own split topology. A half only samples,
transmits and idles, so this firmware has no keyboard framework in it at all:
just `embassy-nrf`, a Gazell FFI module, and a scan loop. Idle behaviour is then
a few lines of code rather than a property of an async runtime, and upgrading embassy
is a version bump in `Cargo.toml`.

Nothing here depends on the receiver's Rust code. The two crates share only the
numbers in their `board.toml` files.

## Layout

```
board.toml             the only board-specific file: chip, memory, matrix + pins,
                       diode direction, both halves' pipe/channel subset, radio
                       parameters, scan/debounce/idle timings
build.rs               board.toml -> board_generated.rs (constants, pin
                       constructors) + memory.x; links the Nordic
                       Gazell archive; validates chip vs cargo feature, pin counts
                       vs matrix size, and the halves' pipe/channel separation
src/lib.rs             clocks, regulator mode, the scan/send/idle loop, the
                       pointer power/mode switches, the DFU hand-off
src/board.rs           the Matrix, Encoder, Pointer and PointerPower types
src/gazell.rs          Gazell FFI (device role), the four callbacks, ISR thunks
src/dfu.rs             the four-corner bootloader gesture
src/screen/            the local status panel (driver, renderers, art) and the
                       battery readout
src/left.rs            the left image: role, screen task, call into run()
src/right.rs           the right image: the same, other role
tools/build.cmd        build both halves -> two UF2s
tools/voltage-sweep.py the battery-anchor sweep (also run by CI)
```

## The two images differ in exactly two numbers

On the radio side, that is: Gazell pipe and channel subset (`[left]` / `[right]`
in `board.toml`). Both are required to be different from each other, and the
build fails if they are not: a host tells devices apart by their on-air address,
so two halves on one pipe would overwrite each other's packets.

Everything else about the link — geometry, timings, the rest of the radio
parameters — is shared; each half's own pins and peripherals are its own section
of `board.toml`, and `left.rs`/`right.rs` differ only in the `Role` they pass.

## Idle behaviour

The half is never powered down. It scans at 1000 Hz while keys are in use; after
`active_release_ms` with nothing pressed it drops the matrix scan to
`idle_poll_ms` while the radio stays enabled (see `radio_off_when_idle`), and the
first press brings the full rate back. SYSTEMOFF was implemented first
(as the original transmitter does it) and rejected on hardware: it is exited by a
*reset*, so a press after a pause paid for a whole restart - MBR, UF2 bootloader,
clocks, radio, re-acquiring the host - and because the matrix is sampled as a
*level* rather than captured as an event, a tap shorter than that restart was not
delayed but lost outright. Measurements and reasoning are in the project README;
the dials (`idle_poll_ms`, `active_release_ms`, `keepalive_ms`,
`radio_off_when_idle`) are in `board.toml`.

> ## What keeps the link moving (2026-09-17)
> 
> Three changes, aimed at the failure the original firmware cannot be caught in
> because every return from idle resets the chip.
> 
> The first attempt at this was wrong, and the way it was wrong is worth keeping,
> because these parameters do not mean what their names suggest.
> `max_tx_attempts` is not a retry count. From the library's own constants, a
> device that has lost sync dwells 15 timeslots on each of 6 channels, so one
> sweep of the host's table costs 90 timeslots (~81 ms at a 900 us period) - that
> is how long a packet is allowed to spend looking. Setting it to 2, and adding an
> 80 ms timer that re-initialised the link whenever a packet produced no callback,
> meant the search covered 2 positions out of 90 and was restarted before it could
> finish one. On hardware that read as the half going quiet after a few
> characters.
> 
> What is in place now:
> 
> * `max_tx_attempts` is 100 (`board.toml`) - the original's value, and the value
>   that covers one full sweep. A packet owns the link for at most one timeslot
>   per attempt, but only while it is searching; a packet that is in sync costs a
>   single timeslot.
> * `timeslots_per_channel_when_out_of_sync` is 15 (`board.toml`) - the library
>   default, and the value the reference transmitter runs on. It decides how long a
>   stutter lasts when the link is lost: a searching packet dwells this long on each
>   of its three channels before moving on. An earlier value of 4 was derived from a
>   rotation arithmetic that has since been withdrawn - it was inferred, never
>   measured, and 15 clears whatever the rotation really is (see `board.toml`).
> * The main loop asks whether the library has *reached a decision* about the last
>   state it accepted. A queue occupied for `QUEUE_STUCK_MS` (500 ms) *with no
>   callback at all* is a library that has stopped rather than a link that is slow,
>   and is re-initialised in place (`gazell::reset`) - a 1..11 ms re-sync instead
>   of a power cycle. A callback during the wait clears the clock, because a
>   library working through a backlog is slow, not dead.
> * There is no link-lost detector, deliberately: the repeat stream above keeps the
>   link exercised while a key is held, so a half that has lost sync goes looking
>   for the host with traffic in hand rather than in silence. An earlier
>   `LINK_LOST_MS` probe was removed together with the repeat-throttling that had
>   made it necessary - see the notes in `src/lib.rs`.
> * `gazell::send` offers a change even while a packet is already in flight (see
>   its own note): a change refused for room is offered again on the next cadence,
>   so a busy queue delays a keystroke rather than losing it.
> 
> Measured behaviour of the old arrangement: one side went permanently silent
> after 30-40 keys and kept the receiver showing whatever was held at that moment.

## The half's other hardware

Beyond the matrix, a half drives its own encoder, its pointing device (trackpad on
the left, TrackPoint on the right), a status panel and the battery sense - all of
it implemented and all of it local. The encoder count and the pointing device's
raw displacement ride in the report; what a turn or a movement means is decided in
the receiver, which is where the keymap lives. The panel shows the half's own
state (animation, battery, pointer mode) and never anything from the host - there
is still no host-to-half data path at all (see the project README's "single
direction" section).