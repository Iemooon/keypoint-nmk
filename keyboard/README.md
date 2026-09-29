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
src/lib.rs             clocks, regulator mode, the scan/send/idle loop
src/board.rs           the Matrix type: scan() and drive_high()
src/gazell.rs          Gazell FFI (device role), the four callbacks, ISR thunks
src/left.rs            the left image:  three lines
src/right.rs           the right image: the same three lines, other role
tools/build.cmd        build both halves -> two UF2s
```

## The two images differ in exactly two numbers

Gazell pipe and channel subset (`[left]` / `[right]` in `board.toml`). Both are
required to be different from each other, and the build fails if they are not:
a host tells devices apart by their on-air address, so two halves on one pipe
would overwrite each other's packets.

Everything else — geometry, timings, the rest of the radio parameters — is
shared, which is why `left.rs` and `right.rs` are three lines apiece.

## Idle behaviour

The half is never powered down. It scans at 1000 Hz while keys are in use; after
`active_release_ms` with nothing pressed it drops to `idle_poll_ms` and switches
the radio off, and the first press wakes both. SYSTEMOFF was implemented first
(as the original transmitter does it) and rejected on hardware: it is exited by a
*reset*, so a press after a pause paid for a whole restart - MBR, UF2 bootloader,
clocks, radio, re-acquiring the host - and because the matrix is sampled as a
*level* rather than captured as an event, a tap shorter than that restart was not
delayed but lost outright. Measurements and reasoning are in the project README;
the dials (`idle_poll_ms`, `active_release_ms`, `radio_off_when_idle`) are in
`board.toml`.

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
> * `timeslots_per_channel_when_out_of_sync` is 4 (`board.toml`) rather than the
>   library default of 15. This is the one that decides how long a stutter lasts:
>   4 timeslots on each of 6 channels is 24 timeslots, ~22 ms, and 24 timeslots is
>   exactly one full turn of the receiver's rotation - the shortest sweep that can
>   still promise the packet was offered on every channel while the host was
>   listening on this pipe. At the default it was 90 timeslots, ~81 ms, which is
>   long enough to feel: while one packet sweeps, `send` refuses the states offered
>   behind it, so keys pressed during the sweep are never transmitted. Typing on
>   that felt like an occasional stutter with a dropped key - or, once the
>   receiver's own link timeout had released the held key, a repeated one.
> * The main loop asks whether the library has *reached a decision* about the last
>   state it accepted. A queue occupied for `QUEUE_STUCK_MS` (300 ms) *with no
>   callback at all* is a library that has stopped rather than a link that is slow,
>   and is re-initialised in place (`gazell::reset`) - a 1..11 ms re-sync instead
>   of a power cycle. A callback during the wait clears the clock, because a
>   library working through a backlog is slow, not dead.
> * If nothing has been *acknowledged* for `LINK_LOST_MS` (500 ms, six times the
>   worst legitimate flight time), the half sends the current state anyway and only
>   rebuilds the link if that also goes unanswered. This closes the dead end a
>   silent half used to sit in: it offered nothing, and a half that offers nothing
>   has no way to find the host again, so a half that locked up stayed locked up
>   until the power was cycled. It now recovers on its own within about a second.
> * `gazell::send` keeps at most one packet in flight (see its own note): with
>   three slots fillable, a queue that filled up refused every later state, so a
>   half went silent while the receiver kept showing a key that was long released.
> 
> Measured behaviour of the old arrangement: one side went permanently silent
> after 30-40 keys and kept the receiver showing whatever was held at that moment.

## Not yet handled

The half also has a display, a trackpad or trackpoint, an encoder and battery
sense on the KeyPoint halves. None of it is touched here: this step is the
keyboard only, and there is no host-to-half data path at all (see the project
README's "single direction" section).