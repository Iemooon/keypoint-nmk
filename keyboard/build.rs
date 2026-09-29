//! Build script for the 2.4GHz transmitter halves.
//!
//! Two jobs, and nothing else.
//!
//! **1. Turn `board.toml` into code.** One file describes the whole keyboard -
//! chip, matrix size, both halves' pins, both halves' radio parameters, timings -
//! and this script emits `$OUT_DIR/board_generated.rs` containing, for each half:
//!
//!   * the Gazell pipe and channel table to transmit on,
//!   * a `matrix_pins()` function that builds the actual embassy pin objects
//!     (pins are named in board.toml, so the code that names them has to be
//!     generated).
//!
//! plus the geometry, radio and timing constants, and `$OUT_DIR/memory.x`.
//!
//! This is what makes "port it to another keyboard" a one-file edit, and what
//! makes a wrong number a build error instead of firmware that misbehaves over
//! the air.
//!
//! **2. Wire the linker**: memory.x where flip-link and cortex-m-rt look for it,
//! `--nmagic` (our FLASH origin is 0x1000, not 64K-aligned), the two linker
//! scripts, and the closed-source Nordic Gazell archive.

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::PathBuf;

// ---------------------------------------------------------------------------
// board.toml model
// ---------------------------------------------------------------------------

struct Half {
    pipe: u32,
    channel_table: Vec<u8>,
    row_pins: Vec<Pin>,
    col_pins: Vec<Pin>,
    /// The knob's two contacts. Extra pins - the encoder is not in the matrix.
    encoder_a: Pin,
    encoder_b: Pin,
    /// Per half, because the two knobs are mounted as mirror images of each other:
    /// the same quadrature direction means opposite things on the two sides.
    encoder_reverse: bool,
    /// The pointing device, when this half has one: which driver, then the bus pins.
    /// `None` is a legitimate state - such a half simply reports zero movement.
    pointer: Option<(PointerKind, Pin, Pin, Option<Pin>)>,
    /// The MOSFET gate that switches the pointing device's own supply, when this
    /// half has one. `None` means the device is permanently powered and the switch
    /// is software only - still worth having (the half stops touching the bus), but
    /// unable to remove the device's own quiescent draw.
    ///
    /// Hardware, not a preference: the keyboard's designer confirmed that P0.08 of
    /// the right half drives the nub's supply through a MOSFET, and ZMK's board
    /// definition carries the same two gates as
    /// `EXT_POWER { control-gpios = <&gpio0 8 GPIO_ACTIVE_LOW>; }` on the right half
    /// and `<&gpio0 2 ...>` on the left. Both ACTIVE LOW: low = powered.
    pointer_power: Option<Pin>,
    /// The matrix cells that switch the pointing device's power and its mode, in this
    /// half's LOCAL coordinates (build.rs converts them from the whole-keyboard
    /// numbering board.toml uses), and how long this half may sit with no input before
    /// switching the device off by itself (0 = never).
    ///
    /// Two keys, two jobs: the power switch removes the device's supply, the mode
    /// switch decides whether its movement is a cursor or a wheel. Neither implies the
    /// other - a nub you are not using at all should cost nothing, while a nub you are
    /// driving a page with should still be powered.
    pointer_power_key: Option<(usize, usize)>,
    pointer_mode_key: Option<(usize, usize)>,
    pointer_idle_off_ms: u32,
    /// The four cells that put this half into its bootloader when pressed together,
    /// in this half's LOCAL coordinates (converted here, like the switch keys).
    ///
    /// Required, not optional, unlike the switch keys: a half with no combo is a
    /// half that can only be flashed by taking the keyboard apart, and a board.toml
    /// edit that lost the line would stay invisible until exactly that moment.
    dfu_combo: [(usize, usize); 4],
    /// The status panel, in the order the driver wants them: clock, data, then
    /// chip select (active high on this board). Driven once per boot - see the
    /// bisect note in src/screen/mod.rs.
    screen: (Pin, Pin, Pin),
    /// The battery divider's sense line, in the order the ADC wants it. Read only
    /// by the panel path - see src/screen/mod.rs.
    battery: Pin,
}

/// One pin: the embassy peripheral field name, e.g. `P0_24`, used verbatim as
/// `p.P0_24` in the generated matrix constructor.
///
/// The name is validated at parse time (port P0/P1, pin number 0..=31) so a typo
/// in board.toml fails here with a clear message instead of surfacing as a
/// confusing "no field `P2_40`" from rustc.
struct Pin {
    field: String,
}

impl Pin {
    fn parse(s: &str) -> Pin {
        // Expected shape: P<port>_<index>, e.g. P0_24 / P1_08.
        let rest = s
            .strip_prefix('P')
            .unwrap_or_else(|| panic!("board.toml: pin {s:?} must look like \"P0_24\""));
        let (port, index) = rest
            .split_once('_')
            .unwrap_or_else(|| panic!("board.toml: pin {s:?} must look like \"P0_24\""));
        let port: u8 = port
            .parse()
            .unwrap_or_else(|_| panic!("board.toml: pin {s:?} has a bad port number"));
        let index: u8 = index
            .parse()
            .unwrap_or_else(|_| panic!("board.toml: pin {s:?} has a bad pin number"));
        if port > 1 {
            panic!("board.toml: pin {s:?} - only ports P0 and P1 exist");
        }
        if index > 31 {
            panic!("board.toml: pin {s:?} - pin numbers are 0..=31");
        }
        Pin {
            field: s.to_string(),
        }
    }
}

struct Board {
    chip: String,
    flash_origin: u32,
    flash_length: u32,
    ram_origin: u32,
    ram_length: u32,

    rows: u32,
    cols: u32,
    col2row: bool,
    settle_cycles: u32,

    left: Half,
    right: Half,

    library: String,
    library_dir: String,
    datarate: u32,
    timeslot_period_us: u32,
    max_tx_attempts: u32,
    timeslots_per_channel: u32,
    timeslots_per_channel_when_out_of_sync: u32,
    sync_lifetime_timeslots: u32,
    channel_selection_policy: u32,
    base_address_0: u32,
    base_address_1: u32,
    xosc_manual: bool,
    radio_off_when_idle: bool,

    scan_hz: u32,
    debounce_ticks: u32,
    idle_poll_ms: u32,
    active_release_ms: u32,
    keepalive_ms: u32,

    reg1_ldo: bool,
    reg1_dcdc: bool,

    encoder_resolution: i8,

    /// The bootloader gesture. One policy for the whole keyboard, not per half: the
    /// two halves should answer the same four-cell press the same way, or the gesture
    /// becomes something to remember rather than something to know.
    dfu: Dfu,
}

/// `[dfu]`: the four-cell gesture that resets a half into its bootloader.
///
/// Every field here is a dial on a sequence that is otherwise fixed - see
/// `src/dfu.rs` for the sequence and why each step is in it.
struct Dfu {
    enabled: bool,
    require_usb: bool,
    window_ms: u32,
    grace_ms: u32,
    release_packets: u32,
}

impl Dfu {
    /// Off unless board.toml says otherwise.
    ///
    /// The default leans towards the harmless failure: no `[dfu]` table means no
    /// gesture, which is one more reflash rather than a keyboard that resets itself
    /// because someone typed on it.
    fn default() -> Dfu {
        Dfu {
            enabled: false,
            require_usb: true,
            window_ms: 40,
            grace_ms: 40,
            release_packets: 6,
        }
    }
}

/// `[dfu]` from board.toml, or the defaults above when the table is absent.
fn get_dfu(t: &toml::Value) -> Dfu {
    let mut dfu = Dfu::default();
    let Some(table) = t.get("dfu").and_then(|v| v.as_table()) else {
        return dfu;
    };
    let bool_at = |key: &str, into: &mut bool| {
        if let Some(v) = table.get(key) {
            *into = v
                .as_bool()
                .unwrap_or_else(|| panic!("board.toml: [dfu] {key} must be true or false"));
        }
    };
    let uint_at = |key: &str, into: &mut u32| {
        if let Some(v) = table.get(key) {
            let n = v
                .as_integer()
                .unwrap_or_else(|| panic!("board.toml: [dfu] {key} must be an integer"));
            if n < 0 {
                panic!("board.toml: [dfu] {key} must not be negative");
            }
            *into = n as u32;
        }
    };
    bool_at("enabled", &mut dfu.enabled);
    bool_at("require_usb", &mut dfu.require_usb);
    uint_at("window_ms", &mut dfu.window_ms);
    uint_at("grace_ms", &mut dfu.grace_ms);
    uint_at("release_packets", &mut dfu.release_packets);
    if dfu.window_ms == 0 && dfu.enabled {
        panic!(
            "board.toml: [dfu] window_ms = 0 with enabled = true means the four cells have to go \
             down on the same scan, which a hand cannot do. Set it to 0 only by disabling the \
             gesture."
        );
    }
    dfu
}

/// `dfu_combo` in a half's section: the four cells of the bootloader gesture, in
/// WHOLE-KEYBOARD coordinates like the switch keys above.
fn get_dfu_combo(t: &toml::Value, half: &str, rows: u32, cols: u32) -> [(usize, usize); 4] {
    const N: usize = 4;
    let table = t
        .get(half)
        .and_then(|v| v.as_table())
        .unwrap_or_else(|| panic!("board.toml: [{half}] must be a table"));
    let list = table
        .get("dfu_combo")
        .unwrap_or_else(|| {
            panic!(
                "board.toml: [{half}] dfu_combo is missing. Every half names the four cells that \
                 put it into its bootloader; there is no other way in without opening the \
                 keyboard."
            )
        })
        .as_array()
        .unwrap_or_else(|| panic!("board.toml: [{half}] dfu_combo must be an array of [row, col]"));
    if list.len() != N {
        panic!(
            "board.toml: [{half}] dfu_combo has {} entries; it needs exactly {N} (one per corner \
             of the block).",
            list.len()
        );
    }
    let mut out = [(0usize, 0usize); N];
    for (i, cell) in list.iter().enumerate() {
        out[i] = parse_cell(cell, half, "dfu_combo", rows, cols);
    }
    for i in 0..N {
        for j in 0..i {
            if out[i] == out[j] {
                panic!(
                    "board.toml: [{half}] dfu_combo lists the cell ({}, {}) twice (local row, \
                     col); four corners, four distinct cells.",
                    out[i].0, out[i].1
                );
            }
        }
    }
    out
}

fn get<'a>(t: &'a toml::Value, table: &str, key: &str) -> &'a toml::Value {
    t.get(table)
        .and_then(|s| s.get(key))
        .unwrap_or_else(|| panic!("board.toml: missing [{table}] {key}"))
}

fn get_int(t: &toml::Value, table: &str, key: &str) -> i64 {
    get(t, table, key)
        .as_integer()
        .unwrap_or_else(|| panic!("board.toml: [{table}] {key} must be an integer"))
}

fn get_bool(t: &toml::Value, table: &str, key: &str) -> bool {
    get(t, table, key)
        .as_bool()
        .unwrap_or_else(|| panic!("board.toml: [{table}] {key} must be true or false"))
}

fn get_str(t: &toml::Value, table: &str, key: &str) -> String {
    get(t, table, key)
        .as_str()
        .unwrap_or_else(|| panic!("board.toml: [{table}] {key} must be a string"))
        .to_string()
}

/// One pin, where `get_pins` takes a list. The knob's contacts are single pins.
fn get_pin(t: &toml::Value, table: &str, key: &str) -> Pin {
    Pin::parse(
        get(t, table, key)
            .as_str()
            .unwrap_or_else(|| panic!("board.toml: [{table}] {key} must be a pin name")),
    )
}

fn as_u32(v: i64, what: &str) -> u32 {
    u32::try_from(v).unwrap_or_else(|_| panic!("board.toml: {what} must fit in u32 (got {v})"))
}

/// Which pointing device a half carries.
///
/// The two are nothing alike on the wire - one is a read-only PS/2 bridge, the
/// other a register interface that has to be written before it can be read - so a
/// pin pair is not enough to describe a half's pointer. board.toml says which.
#[derive(Clone, Copy, PartialEq, Eq)]
enum PointerKind {
    /// PS/2-to-I2C bridge: fixed 7-byte packet behind a magic byte, never written.
    TrackPoint,
    /// Register interface: one pointer byte written, then a burst read.
    A320,
}

/// The pointing device's bus and driver, when board.toml gives this half one.
///
/// Optional on purpose: a half with no pointer is a half whose packets simply
/// carry zeros in those fields. All three keys must be present or none - a partial
/// set is always a mistake.
///
/// The motion pin is separate and optional on its own: a device that signals when
/// it has data gets one, a device that just answers when asked does not. See
/// `TrackPoint::sample_motion`.
fn get_pointer(t: &toml::Value, half: &str) -> Option<(PointerKind, Pin, Pin, Option<Pin>)> {
    let table = t
        .get(half)
        .and_then(|v| v.as_table())
        .unwrap_or_else(|| panic!("board.toml: [{half}] must be a table"));
    let present = [
        table.contains_key("pointer_kind"),
        table.contains_key("pointer_sda"),
        table.contains_key("pointer_scl"),
    ];
    match present {
        [false, false, false] => None,
        [true, true, true] => {
            let kind = match get_str(t, half, "pointer_kind").as_str() {
                "trackpoint" => PointerKind::TrackPoint,
                "a320" => PointerKind::A320,
                other => panic!(
                    "board.toml: [{half}] pointer_kind = {other:?} is not a known device \
                     (expected \"trackpoint\" or \"a320\")"
                ),
            };
            let motion = table
                .contains_key("pointer_motion")
                .then(|| get_pin(t, half, "pointer_motion"));
            Some((
                kind,
                get_pin(t, half, "pointer_sda"),
                get_pin(t, half, "pointer_scl"),
                motion,
            ))
        }
        _ => panic!(
            "board.toml: [{half}] pointer_kind, pointer_sda and pointer_scl must be set \
             together or left out together"
        ),
    }
}

/// The gate that switches this half's pointing device off and on, when board.toml
/// names one.
///
/// Optional because the gate is a board feature: a half whose pointer is permanently
/// powered is one where the switch can stop the bus traffic but not the device's own
/// draw, and that difference is worth being able to express rather than assume.
fn get_pointer_power(t: &toml::Value, half: &str) -> Option<Pin> {
    let table = t
        .get(half)
        .and_then(|v| v.as_table())
        .unwrap_or_else(|| panic!("board.toml: [{half}] must be a table"));
    if !table.contains_key("pointer_power") {
        return None;
    }
    if get_pointer(t, half).is_none() {
        panic!(
            "board.toml: [{half}] pointer_power is set but this half has no pointer \
             (pointer_kind/pointer_sda/pointer_scl). A gate with nothing behind it is a \
             typo, not a configuration."
        );
    }
    Some(get_pin(t, half, "pointer_power"))
}

/// One switch key from a half's section: `[row, col]` in whole-keyboard
/// coordinates. `None` when the half does not name that key at all.
fn get_pointer_key(
    t: &toml::Value,
    half: &str,
    key: &str,
    rows: u32,
    cols: u32,
) -> Option<(usize, usize)> {
    let table = t
        .get(half)
        .and_then(|v| v.as_table())
        .unwrap_or_else(|| panic!("board.toml: [{half}] must be a table"));
    let value = table.get(key)?;
    Some(parse_cell(value, half, key, rows, cols))
}

/// One `[row, col]` cell from a half's section, in WHOLE-KEYBOARD coordinates,
/// converted to that half's own rows.
///
/// Whole-keyboard rather than local because that is how a switch position is
/// written down and discussed everywhere else - the keymap, Vial, and the person
/// holding the screwdriver. The receiver joins the two halves along rows (left
/// 0..rows, right rows..2*rows - see rx/src/board.rs::logical_row), and the
/// conversion happens here, once, so nothing downstream has to know which half it
/// is running on.
///
/// Shared by the two switch keys and the four cells of the bootloader gesture:
/// they are written the same way in board.toml, and a second copy of this
/// conversion would be a second place for it to be wrong.
fn parse_cell(value: &toml::Value, half: &str, key: &str, rows: u32, cols: u32) -> (usize, usize) {
    let list = value
        .as_array()
        .unwrap_or_else(|| panic!("board.toml: [{half}] {key} must be [row, col]"));
    if list.len() != 2 {
        panic!("board.toml: [{half}] {key} must have exactly two entries (row, col)");
    }
    let n = |i: usize| {
        let v = list[i]
            .as_integer()
            .unwrap_or_else(|| panic!("board.toml: [{half}] {key} entries must be integers"));
        if v < 0 {
            panic!("board.toml: [{half}] {key} entries must not be negative");
        }
        v as u32
    };
    let (row, col) = (n(0), n(1));
    let offset = match half {
        "left" => 0,
        "right" => rows,
        other => panic!("board.toml: unknown half {other:?}"),
    };
    if row < offset || row >= offset + rows {
        panic!(
            "board.toml: [{half}] {key} = [{row}, {col}] is not a row of this half. It is in \
             whole-keyboard coordinates: the left half owns rows 0..{rows}, the right half \
             rows {rows}..{}.",
            rows * 2
        );
    }
    if col >= cols {
        panic!(
            "board.toml: [{half}] {key} = [{row}, {col}] has column {col}, but the matrix is \
             {cols} columns wide."
        );
    }
    ((row - offset) as usize, col as usize)
}

/// `[pointer] idle_off_ms`: how long this half may sit with no input before it cuts
/// the pointing device's power by itself. 0, or no table at all, disables it.
fn get_pointer_idle_ms(t: &toml::Value) -> u32 {
    let Some(table) = t.get("pointer").and_then(|v| v.as_table()) else {
        return 0;
    };
    let idle = table
        .get("idle_off_ms")
        .map(|v| {
            v.as_integer()
                .unwrap_or_else(|| panic!("board.toml: [pointer] idle_off_ms must be an integer"))
        })
        .unwrap_or(0);
    if idle < 0 {
        panic!("board.toml: [pointer] idle_off_ms must not be negative (0 disables it)");
    }
    idle as u32
}

fn get_pins(t: &toml::Value, half: &str, key: &str) -> Vec<Pin> {
    get(t, half, key)
        .as_array()
        .unwrap_or_else(|| panic!("board.toml: [{half}] {key} must be an array of pin names"))
        .iter()
        .map(|v| {
            Pin::parse(
                v.as_str()
                    .unwrap_or_else(|| panic!("board.toml: [{half}] {key} entries must be strings")),
            )
        })
        .collect()
}

fn get_channel_table(t: &toml::Value, half: &str) -> Vec<u8> {
    let channels: Vec<u8> = get(t, half, "channel_table")
        .as_array()
        .unwrap_or_else(|| panic!("board.toml: [{half}] channel_table must be an array of integers"))
        .iter()
        .map(|v| {
            let n = v.as_integer().expect("board.toml: channel must be an integer");
            u8::try_from(n).unwrap_or_else(|_| panic!("board.toml: channel {n} does not fit in u8"))
        })
        .collect();
    if channels.is_empty() || channels.len() > 16 {
        panic!(
            "board.toml: [{half}] channel_table has {} entries; Gazell allows 1..=16 \
             (NRF_GZLL_CONST_MAX_CHANNEL_TABLE_SIZE).",
            channels.len()
        );
    }
    for c in &channels {
        if *c > 100 {
            panic!("board.toml: channel {c} is outside the 2.4GHz band (0..=100)");
        }
    }
    channels
}

/// Parse `0x1000`, `636K`, `1M` or a plain decimal into a byte count.
fn parse_num(s: &str) -> Option<u32> {
    let s = s.trim();
    let (digits, mult) = if let Some(d) = s.strip_suffix(['K', 'k']) {
        (d, 1024u32)
    } else if let Some(d) = s.strip_suffix(['M', 'm']) {
        (d, 1024 * 1024)
    } else {
        (s, 1)
    };
    let digits = digits.trim();
    let base = if digits.starts_with("0x") || digits.starts_with("0X") {
        16
    } else {
        10
    };
    let digits = digits.trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(digits, base).ok().map(|v| v * mult)
}

fn get_bytes(t: &toml::Value, table: &str, key: &str) -> u32 {
    match get(t, table, key) {
        toml::Value::Integer(v) => as_u32(*v, &format!("{table}.{key}")),
        toml::Value::String(s) => parse_num(s).unwrap_or_else(|| {
            panic!(
                "board.toml: [{table}] {key} = {s:?} is not a byte count (use 0x1000 or \"1020K\")"
            )
        }),
        _ => panic!("board.toml: [{table}] {key} must be an integer or a string like \"1020K\""),
    }
}

/// `[power] reg1` - optional, one of `"keep"` (default), `"ldo"`, `"dcdc"`.
/// Same semantics as the receiver's; see its board.toml for why this is a
/// per-board decision rather than a chip default.
fn reg_mode(t: &toml::Value, key: &str) -> (bool, bool) {
    let mode = t
        .get("power")
        .and_then(|p| p.get(key))
        .map(|v| {
            v.as_str()
                .unwrap_or_else(|| panic!("board.toml: [power] {key} must be a string"))
        })
        .unwrap_or("keep");
    match mode {
        "keep" => (false, false),
        "ldo" => (true, false),
        "dcdc" => (false, true),
        other => {
            panic!("board.toml: [power] {key} = {other:?} must be \"keep\", \"ldo\" or \"dcdc\"")
        }
    }
}

fn load_board() -> Board {
    let path = env::var("BOARD_TOML").unwrap_or_else(|_| "board.toml".to_string());
    println!("cargo:rerun-if-env-changed=BOARD_TOML");
    println!("cargo:rerun-if-changed={path}");
    let text = fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read board config {path}: {e}"));
    let t: toml::Value = text.parse().expect("board.toml is not valid TOML");

    let rows = as_u32(get_int(&t, "matrix", "rows"), "matrix.rows");
    let cols = as_u32(get_int(&t, "matrix", "cols"), "matrix.cols");
    if rows == 0 {
        panic!("board.toml: [matrix] rows = 0 must be at least 1");
    }
    if cols == 0 {
        panic!("board.toml: [matrix] cols = 0 must be at least 1");
    }
    // ---- what the link limits is the CELL COUNT, not either dimension -------
    // The matrix travels as ONE BITMAP: row 0's columns first, then row 1's, and
    // so on. It is not one byte per row any more, so neither rows nor cols has a
    // bound of its own - only their product does.
    //
    //     6 x 8  ->  48 cells ->  6 bytes   (KeyPoint: byte-identical to the old
    //                                        one-byte-per-row layout, because 8
    //                                        columns exactly fill a byte)
    //     8 x 8  ->  64 cells ->  8 bytes
    //     8 x 16 -> 128 cells -> 16 bytes
    //    12 x 16 -> 192 cells -> 24 bytes
    //    16 x 16 -> 256 cells -> 32 bytes = Gazell's limit
    //
    // This is what makes a wider keyboard a board.toml edit instead of a
    // protocol change.
    let cells = rows * cols;
    let matrix_bytes = cells.div_ceil(8);
    if matrix_bytes > 32 {
        panic!(
            "board.toml: [matrix] {rows} rows x {cols} cols = {cells} cells needs {matrix_bytes} \
             bitmap bytes, and Gazell's payload limit is 32 bytes \
             (NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH). At most 256 cells fit in one report."
        );
    }

    let col2row = match get_str(&t, "matrix", "diode").to_ascii_lowercase().as_str() {
        "col2row" => true,
        "row2col" => false,
        other => panic!("board.toml: [matrix] diode = {other:?} must be \"col2row\" or \"row2col\""),
    };

    // The decoder counts quarter-steps, so a resolution of 0 would make it emit a
    // step on every single transition - contact bounce in, clicks out.
    let encoder_resolution = i8::try_from(get_int(&t, "encoder", "resolution"))
        .unwrap_or_else(|_| panic!("board.toml: [encoder] resolution must fit in i8"));
    if encoder_resolution < 1 {
        panic!("board.toml: [encoder] resolution = {encoder_resolution} must be at least 1");
    }

    let pointer_idle_off_ms = get_pointer_idle_ms(&t);
    let left = Half {
        pipe: as_u32(get_int(&t, "left", "pipe"), "left.pipe"),
        channel_table: get_channel_table(&t, "left"),
        row_pins: get_pins(&t, "left", "row_pins"),
        col_pins: get_pins(&t, "left", "col_pins"),
        encoder_a: get_pin(&t, "left", "encoder_a"),
        encoder_b: get_pin(&t, "left", "encoder_b"),
        encoder_reverse: get_bool(&t, "left", "encoder_reverse"),
        pointer: get_pointer(&t, "left"),
        pointer_power: get_pointer_power(&t, "left"),
        pointer_power_key: get_pointer_key(&t, "left", "pointer_power_key", rows, cols),
        pointer_mode_key: get_pointer_key(&t, "left", "pointer_mode_key", rows, cols),
        pointer_idle_off_ms,
        dfu_combo: get_dfu_combo(&t, "left", rows, cols),
        screen: (
            get_pin(&t, "left", "screen_sck"),
            get_pin(&t, "left", "screen_mosi"),
            get_pin(&t, "left", "screen_cs"),
        ),
        battery: get_pin(&t, "left", "battery_adc"),
    };
    let right = Half {
        pipe: as_u32(get_int(&t, "right", "pipe"), "right.pipe"),
        channel_table: get_channel_table(&t, "right"),
        row_pins: get_pins(&t, "right", "row_pins"),
        col_pins: get_pins(&t, "right", "col_pins"),
        encoder_a: get_pin(&t, "right", "encoder_a"),
        encoder_b: get_pin(&t, "right", "encoder_b"),
        encoder_reverse: get_bool(&t, "right", "encoder_reverse"),
        pointer: get_pointer(&t, "right"),
        pointer_power: get_pointer_power(&t, "right"),
        pointer_power_key: get_pointer_key(&t, "right", "pointer_power_key", rows, cols),
        pointer_mode_key: get_pointer_key(&t, "right", "pointer_mode_key", rows, cols),
        pointer_idle_off_ms,
        dfu_combo: get_dfu_combo(&t, "right", rows, cols),
        screen: (
            get_pin(&t, "right", "screen_sck"),
            get_pin(&t, "right", "screen_mosi"),
            get_pin(&t, "right", "screen_cs"),
        ),
        battery: get_pin(&t, "right", "battery_adc"),
    };

    for (name, half) in [("left", &left), ("right", &right)] {
        // The knob's contacts are read on their own schedule, so they must not
        // also be matrix lines: the scan would drive a shared pin high behind the
        // decoder's back and every reading would be wrong.
        let mut matrix_pins: Vec<&str> = half
            .row_pins
            .iter()
            .chain(half.col_pins.iter())
            .map(|p| p.field.as_str())
            .collect();
        matrix_pins.sort_unstable();
        for (label, pin) in [("encoder_a", &half.encoder_a), ("encoder_b", &half.encoder_b)] {
            if matrix_pins.contains(&pin.field.as_str()) {
                panic!(
                    "board.toml: [{name}] {label} = {:?} is also a matrix pin. The knob's \
                     contacts are read outside the scan, so a pin cannot be both.",
                    pin.field
                );
            }
        }
        if half.encoder_a.field == half.encoder_b.field {
            panic!(
                "board.toml: [{name}] encoder_a and encoder_b are the same pin ({:?}); a \
                 quadrature encoder needs two.",
                half.encoder_a.field
            );
        }

        // No cell may do two jobs. A corner of the bootloader gesture that also
        // switches the pointing device's power would make one of the two depend on
        // the other - press it as part of the gesture and the pointer dies, press it
        // alone and half the gesture is already down.
        for (label, key) in [
            ("pointer_power_key", half.pointer_power_key),
            ("pointer_mode_key", half.pointer_mode_key),
        ] {
            if let Some(cell) = key {
                if half.dfu_combo.contains(&cell) {
                    panic!(
                        "board.toml: [{name}] dfu_combo and {label} both name the cell ({}, {}) \
                         (local row, col of this half).",
                        cell.0, cell.1
                    );
                }
            }
        }

        // The battery divider is sampled by the ADC at moments the loop chooses,
        // and any pin the scan drives would make it read the drive level instead
        // of the cell. So it is kept off everything else, same as the knob.
        {
            let mut taken: Vec<(&str, &String)> = vec![
                ("encoder_a", &half.encoder_a.field),
                ("encoder_b", &half.encoder_b.field),
                ("screen_sck", &half.screen.0.field),
                ("screen_mosi", &half.screen.1.field),
                ("screen_cs", &half.screen.2.field),
            ];
            if let Some((_, sda, scl, motion)) = &half.pointer {
                taken.push(("pointer_sda", &sda.field));
                taken.push(("pointer_scl", &scl.field));
                if let Some(m) = motion {
                    taken.push(("pointer_motion", &m.field));
                }
            }
            for (label, pin) in taken {
                if half.battery.field == *pin {
                    panic!(
                        "board.toml: [{name}] battery_adc = {:?} is already {label}. The ADC \
                         samples it whenever it likes, so it cannot be a pin the scan drives \
                         or another peripheral owns.",
                        half.battery.field
                    );
                }
            }
            if matrix_pins.contains(&half.battery.field.as_str()) {
                panic!(
                    "board.toml: [{name}] battery_adc = {:?} is also a matrix pin; the scan \
                     would drive it and the ADC would read the drive level, not the cell.",
                    half.battery.field
                );
            }
        }

        // The pointing device's bus is driven outside the scan too, and an I2C
        // line also carrying a matrix line would put the bus in a state neither
        // end can make sense of.
        if let Some((_, sda, scl, motion)) = &half.pointer {
            for (label, pin) in [("pointer_sda", sda), ("pointer_scl", scl)] {
                if matrix_pins.contains(&pin.field.as_str()) {
                    panic!(
                        "board.toml: [{name}] {label} = {:?} is also a matrix pin; the bus is \
                         read outside the scan, so a pin cannot be both.",
                        pin.field
                    );
                }
                if pin.field == half.encoder_a.field || pin.field == half.encoder_b.field {
                    panic!(
                        "board.toml: [{name}] {label} = {:?} is also one of the knob's \
                         contacts.",
                        pin.field
                    );
                }
            }
            if sda.field == scl.field {
                panic!(
                    "board.toml: [{name}] pointer_sda and pointer_scl are the same pin ({:?}).",
                    sda.field
                );
            }
            // The motion line is an ordinary GPIO, so unlike the bus it could share
            // with the matrix in principle - but it is sampled every pass, and a
            // matrix drive line would make it read as active whenever that row was
            // driven. Kept out of the matrix and off the knob for the same reason
            // the bus is.
            if let Some(pin) = motion {
                for (label, other) in [("pointer_sda", sda), ("pointer_scl", scl)] {
                    if pin.field == other.field {
                        panic!(
                            "board.toml: [{name}] pointer_motion = {:?} is also {label}.",
                            pin.field
                        );
                    }
                }
                if pin.field == half.encoder_a.field || pin.field == half.encoder_b.field {
                    panic!(
                        "board.toml: [{name}] pointer_motion = {:?} is also one of the knob's \
                         contacts.",
                        pin.field
                    );
                }
                if matrix_pins.contains(&pin.field.as_str()) {
                    panic!(
                        "board.toml: [{name}] pointer_motion = {:?} is also a matrix pin; it is \
                         sampled every pass and the scan would drive it.",
                        pin.field
                    );
                }
            }
        }

        // The switch is an output this firmware drives, so it may not be anything
        // else on this half - least of all one of the pointer's own lines, where
        // driving it would fight the bus or the device's data-ready line.
        if let Some(gate) = &half.pointer_power {
            let mut taken: Vec<(&str, &String)> = vec![
                ("encoder_a", &half.encoder_a.field),
                ("encoder_b", &half.encoder_b.field),
                ("screen_sck", &half.screen.0.field),
                ("screen_mosi", &half.screen.1.field),
                ("screen_cs", &half.screen.2.field),
                ("battery_adc", &half.battery.field),
            ];
            if let Some((_, sda, scl, motion)) = &half.pointer {
                taken.push(("pointer_sda", &sda.field));
                taken.push(("pointer_scl", &scl.field));
                if let Some(m) = motion {
                    taken.push(("pointer_motion", &m.field));
                }
            }
            for (label, pin) in taken {
                if gate.field == *pin {
                    panic!(
                        "board.toml: [{name}] pointer_power = {:?} is already {label}. It is \
                         driven as an output, so it cannot also be a line the scan drives, the \
                         bus uses or the ADC reads.",
                        gate.field
                    );
                }
            }
            if matrix_pins.contains(&gate.field.as_str()) {
                panic!(
                    "board.toml: [{name}] pointer_power = {:?} is also a matrix pin; the scan \
                     drives it and so does this, and the two would fight.",
                    gate.field
                );
            }
        }

        // The two switch keys are already checked against the geometry where they are
        // parsed (in whole-keyboard coordinates, converted there to this half's own
        // rows), so there is nothing left to check here.

        if half.pipe > 7 {
            panic!("board.toml: [{name}] pipe = {} must be 0..=7", half.pipe);
        }
        if half.row_pins.len() as u32 != rows {
            panic!(
                "board.toml: [{name}] row_pins has {} entries but [matrix] rows = {rows}. Every \
                 row needs a pin, in row order (row 0 first).",
                half.row_pins.len()
            );
        }
        if half.col_pins.len() as u32 != cols {
            panic!(
                "board.toml: [{name}] col_pins has {} entries but [matrix] cols = {cols}. Every \
                 column needs a pin, in column order (column 0 first).",
                half.col_pins.len()
            );
        }
    }
    if left.pipe == right.pipe {
        panic!(
            "board.toml: both halves are on Gazell pipe {}. A host identifies a device by its \
             on-air address, so two halves sharing a pipe cannot be told apart - their packets \
             would overwrite each other. The left half must be on pipe 0 (the only pipe that can \
             carry base_address_0) and the right half on pipe 1..7.",
            left.pipe
        );
    }
    let shared: Vec<u8> = left
        .channel_table
        .iter()
        .filter(|c| right.channel_table.contains(c))
        .copied()
        .collect();
    if !shared.is_empty() {
        panic!(
            "board.toml: the two halves share channel(s) {shared:?}. Each half must transmit on \
             its own subset of the receiver's table - sharing a channel means two devices \
             contending for one frequency, which shows up as dropped keystrokes."
        );
    }

    let datarate = match get_str(&t, "gazell", "datarate").to_ascii_lowercase().as_str() {
        "250kbit" => 0,
        "1mbit" => 1,
        "2mbit" => 2,
        other => {
            panic!("board.toml: [gazell] datarate \"{other}\" must be 250kbit, 1mbit or 2mbit")
        }
    };

    let xosc_manual = match get_str(&t, "gazell", "xosc_ctl").to_ascii_lowercase().as_str() {
        "auto" => false,
        "manual" => true,
        other => panic!("board.toml: [gazell] xosc_ctl = {other:?} must be \"auto\" or \"manual\""),
    };

    let scan_hz = as_u32(get_int(&t, "timing", "scan_hz"), "timing.scan_hz");
    if scan_hz == 0 || scan_hz > 8000 {
        panic!("board.toml: [timing] scan_hz = {scan_hz} must be 1..=8000");
    }

    Board {
        chip: get_str(&t, "chip", "name"),
        flash_origin: get_bytes(&t, "memory", "flash_origin"),
        flash_length: get_bytes(&t, "memory", "flash_length"),
        ram_origin: get_bytes(&t, "memory", "ram_origin"),
        ram_length: get_bytes(&t, "memory", "ram_length"),

        rows,
        cols,
        col2row,
        settle_cycles: as_u32(get_int(&t, "matrix", "settle_cycles"), "matrix.settle_cycles"),

        left,
        right,

        library: get_str(&t, "gazell", "library"),
        library_dir: get_str(&t, "gazell", "library_dir"),
        datarate,
        timeslot_period_us: as_u32(
            get_int(&t, "gazell", "timeslot_period_us"),
            "gazell.timeslot_period_us",
        ),
        max_tx_attempts: as_u32(get_int(&t, "gazell", "max_tx_attempts"), "gazell.max_tx_attempts"),
        timeslots_per_channel: as_u32(
            get_int(&t, "gazell", "timeslots_per_channel"),
            "gazell.timeslots_per_channel",
        ),
        timeslots_per_channel_when_out_of_sync: as_u32(
            get_int(&t, "gazell", "timeslots_per_channel_when_out_of_sync"),
            "gazell.timeslots_per_channel_when_out_of_sync",
        ),
        sync_lifetime_timeslots: as_u32(
            get_int(&t, "gazell", "sync_lifetime_timeslots"),
            "gazell.sync_lifetime_timeslots",
        ),
        channel_selection_policy: as_u32(
            get_int(&t, "gazell", "channel_selection_policy"),
            "gazell.channel_selection_policy",
        ),
        base_address_0: as_u32(get_int(&t, "gazell", "base_address_0"), "gazell.base_address_0"),
        base_address_1: as_u32(get_int(&t, "gazell", "base_address_1"), "gazell.base_address_1"),
        xosc_manual,
        radio_off_when_idle: get_bool(&t, "gazell", "radio_off_when_idle"),

        scan_hz,
        debounce_ticks: as_u32(get_int(&t, "timing", "debounce_ticks"), "timing.debounce_ticks"),
        idle_poll_ms: as_u32(get_int(&t, "timing", "idle_poll_ms"), "timing.idle_poll_ms"),
        active_release_ms: as_u32(
            get_int(&t, "timing", "active_release_ms"),
            "timing.active_release_ms",
        ),
        keepalive_ms: as_u32(
            get_int(&t, "timing", "keepalive_ms"),
            "timing.keepalive_ms",
        ),

        reg1_ldo: reg_mode(&t, "reg1").0,
        reg1_dcdc: reg_mode(&t, "reg1").1,

        encoder_resolution,

        dfu: get_dfu(&t),
    }
}

// ---------------------------------------------------------------------------
// Code generation
// ---------------------------------------------------------------------------

/// Which way round the pins are: the driven side is always outputs, the read side
/// always inputs, and which pin list that is depends on the diode direction.
fn role_pins<'a>(b: &Board, half: &'a Half) -> (&'a Vec<Pin>, &'a Vec<Pin>) {
    if b.col2row {
        (&half.col_pins, &half.row_pins) // drive columns, read rows
    } else {
        (&half.row_pins, &half.col_pins) // drive rows, read columns
    }
}

fn write_half_module(b: &Board, name: &str, half: &Half, out: &mut String) {
    let (drive, read) = role_pins(b, half);

    writeln!(out, "pub mod {name} {{").unwrap();
    writeln!(
        out,
        "    /// Gazell pipe this half transmits on. Read from board.toml, not chosen here:"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the host identifies a device by its on-air address, so this is what makes the"
    )
    .unwrap();
    writeln!(out, "    /// two halves distinguishable at all.").unwrap();
    writeln!(out, "    pub const PIPE: u32 = {};", half.pipe).unwrap();
    // The pointing device's own supply switch: where the key is, and how long this
    // half tolerates idleness before it cuts the power itself. Both are read by the
    // loop through `Role`, which is how one binary serves either half.
    for (name, key) in [
        ("POINTER_POWER_KEY", half.pointer_power_key),
        ("POINTER_MODE_KEY", half.pointer_mode_key),
    ] {
        writeln!(
            out,
            "    /// LOCAL (row, col) of a switch key; board.toml writes it in"
        )
        .unwrap();
        writeln!(
            out,
            "    /// whole-keyboard coordinates and build.rs converts it."
        )
        .unwrap();
        writeln!(
            out,
            "    pub const {name}: Option<(usize, usize)> = {};",
            match key {
                Some((r, c)) => format!("Some(({r}, {c}))"),
                None => "None".to_string(),
            }
        )
        .unwrap();
    }
    writeln!(
        out,
        "    /// Zero disables it: the manual key is then the only switch."
    )
    .unwrap();
    writeln!(
        out,
        "    pub const POINTER_POWER_IDLE_MS: u32 = {};",
        half.pointer_idle_off_ms
    )
    .unwrap();
    writeln!(
        out,
        "    /// LOCAL (row, col) of the four cells that put this half into its"
    )
    .unwrap();
    writeln!(
        out,
        "    /// bootloader, converted here from board.toml's whole-keyboard numbering."
    )
    .unwrap();
    writeln!(out, "    pub const DFU_COMBO: [(usize, usize); 4] = [").unwrap();
    for (r, c) in half.dfu_combo {
        writeln!(out, "        ({r}, {c}),").unwrap();
    }
    writeln!(out, "    ];\n").unwrap();
    writeln!(out, "    pub const CHANNEL_TABLE: [u8; {}] = [", half.channel_table.len()).unwrap();
    for c in &half.channel_table {
        writeln!(out, "        {c},").unwrap();
    }
    writeln!(out, "    ];\n").unwrap();

    writeln!(
        out,
        "    /// Everything board.toml describes for this half, as embassy objects: the matrix,"
    )
    .unwrap();
    writeln!(
        out,
        "    /// then the knob's two contacts, then the pointing device if this half has one."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// One function rather than three because `Peripherals` is consumed: all of them"
    )
    .unwrap();
    writeln!(out, "    /// come out of the same `p`.").unwrap();
    writeln!(
        out,
        "    pub fn pins(p: embassy_nrf::Peripherals) -> (crate::board::Matrix, crate::board::Encoder, Option<crate::board::Pointer>, crate::screen::Screen, crate::screen::Battery, u32) {{"
    )
    .unwrap();
    writeln!(out, "        use embassy_nrf::saadc::Input as _;").unwrap();
    // Drawn before anything else touches `p`, and only the RNG field is moved out
    // of it, so every pin below is still readable. One number, read here because
    // this is the one place where the peripherals are still whole - see the note on
    // `boot_seed_from_rng`.
    writeln!(out, "        let boot_seed = crate::screen::boot_seed_from_rng(p.RNG);").unwrap();
    writeln!(out, "        (").unwrap();
    writeln!(out, "            crate::board::Matrix {{").unwrap();
    writeln!(out, "                drive: [").unwrap();
    for pin in drive {
        writeln!(
            out,
            "                    embassy_nrf::gpio::Output::new(p.{}, embassy_nrf::gpio::Level::Low, \
             embassy_nrf::gpio::OutputDrive::Standard),",
            pin.field
        )
        .unwrap();
    }
    writeln!(out, "                ],").unwrap();
    writeln!(out, "                read: [").unwrap();
    for pin in read {
        writeln!(
            out,
            "                    embassy_nrf::gpio::Input::new(p.{}, embassy_nrf::gpio::Pull::Down),",
            pin.field
        )
        .unwrap();
    }
    writeln!(out, "                ],").unwrap();
    writeln!(out, "            }},").unwrap();
    writeln!(
        out,
        "            // `Pull::None`, and that is the whole of what is known (2026-09-19)."
    )
    .unwrap();
    writeln!(
        out,
        "            //"
    )
    .unwrap();
    writeln!(
        out,
        "            // Two attempts to strengthen these lines failed identically: `Pull::Up`"
    )
    .unwrap();
    writeln!(
        out,
        "            // first and then `Pull::Down` each left the knob completely dead. A line"
    )
    .unwrap();
    writeln!(
        out,
        "            // held the wrong way would fail on one direction and work on the other,"
    )
    .unwrap();
    writeln!(
        out,
        "            // so neither of those failures says which rail the contacts reach - the"
    )
    .unwrap();
    writeln!(
        out,
        "            // reasoning that led to each of them was unfounded. What is established"
    )
    .unwrap();
    writeln!(
        out,
        "            // is only that the pins settle correctly with no internal pull, i.e. the"
    )
    .unwrap();
    writeln!(
        out,
        "            // board provides whatever the encoder needs."
    )
    .unwrap();
    writeln!(
        out,
        "            //"
    )
    .unwrap();
    writeln!(
        out,
        "            // Still open: the phantom detent (an occasional volume step when the"
    )
    .unwrap();
    writeln!(
        out,
        "            // supply rail switches). Chasing it needs data, not a third guess -"
    )
    .unwrap();
    writeln!(
        out,
        "            // either show these two pin levels on the panel and read them while"
    )
    .unwrap();
    writeln!(
        out,
        "            // turning, or drive the pins as the devicetree says ZMK does"
    )
    .unwrap();
    writeln!(
        out,
        "            // (`GPIO_OPEN_DRAIN`) and read them back."
    )
    .unwrap();
    writeln!(out, "            crate::board::Encoder::new(").unwrap();
    writeln!(
        out,
        "                embassy_nrf::gpio::Input::new(p.{}, embassy_nrf::gpio::Pull::None),",
        half.encoder_a.field
    )
    .unwrap();
    writeln!(
        out,
        "                embassy_nrf::gpio::Input::new(p.{}, embassy_nrf::gpio::Pull::None),",
        half.encoder_b.field
    )
    .unwrap();
    writeln!(out, "                {},", half.encoder_reverse).unwrap();
    writeln!(out, "            ),").unwrap();

    match &half.pointer {
        None => writeln!(out, "            None,").unwrap(),
        Some((kind, sda, scl, motion)) => {
            writeln!(out, "            {{").unwrap();
            writeln!(
                out,
                "                let mut i2c_cfg = embassy_nrf::twim::Config::default();"
            )
            .unwrap();
            match kind {
                PointerKind::TrackPoint => {
                    writeln!(
                        out,
                        "                // 400 kHz, matching the ZMK overlay. Not cosmetics: the device is"
                    )
                    .unwrap();
                    writeln!(
                        out,
                        "                // a PS/2-to-I2C bridge, and a slower bus widens the window in which"
                    )
                    .unwrap();
                    writeln!(
                        out,
                        "                // the stick can overwrite the packet being fetched."
                    )
                    .unwrap();
                }
                PointerKind::A320 => {
                    writeln!(
                        out,
                        "                // 400 kHz, matching the ZMK overlay for the pad."
                    )
                    .unwrap();
                }
            }
            writeln!(
                out,
                "                i2c_cfg.frequency = embassy_nrf::twim::Frequency::K400;"
            )
            .unwrap();
            let ctor = match kind {
                PointerKind::TrackPoint => "crate::board::TrackPoint",
                PointerKind::A320 => "crate::board::A320",
            };
            let variant = match kind {
                PointerKind::TrackPoint => "TrackPoint",
                PointerKind::A320 => "A320",
            };
            writeln!(
                out,
                "                Some(crate::board::Pointer::{variant}({ctor}::new(embassy_nrf::twim::Twim::new("
            )
            .unwrap();
            writeln!(out, "                    p.TWISPI0,").unwrap();
            writeln!(out, "                    crate::board::Irqs,").unwrap();
            writeln!(out, "                    p.{},", sda.field).unwrap();
            writeln!(out, "                    p.{},", scl.field).unwrap();
            writeln!(out, "                    i2c_cfg,").unwrap();
            writeln!(out, "                    crate::board::tx_buffer(),").unwrap();
            writeln!(out, "                ),").unwrap();
            // Only the TrackPoint takes one: the pad answers with a zero packet
            // instead of holding a data-ready line, so it has nothing to sample.
            // `None` here means this half polls on the timer, which is what the
            // driver did before the line existed.
            if *kind == PointerKind::TrackPoint {
                match motion {
                    Some(pin) => {
                        writeln!(out, "                // Active low, held up by the bridge: the same").unwrap();
                        writeln!(out, "                // flags ZMK puts on this pin.").unwrap();
                        writeln!(
                            out,
                            "                Some(embassy_nrf::gpio::Input::new(p.{}, \
                             embassy_nrf::gpio::Pull::Up)),",
                            pin.field
                        )
                        .unwrap();
                    }
                    None => writeln!(out, "                None,").unwrap(),
                }
            }
            // The device's own supply, last in both constructors' argument lists. It
            // is driven, not merely remembered: `PointerPower` owns the pin and the
            // polarity, and the loop only ever says "on" or "off".
            match &half.pointer_power {
                Some(gate) => {
                    writeln!(
                        out,
                        "                // ACTIVE LOW: low = the MOSFET conducts = powered."
                    )
                    .unwrap();
                    writeln!(
                        out,
                        "                crate::board::PointerPower::new(Some(embassy_nrf::gpio::Output::new("
                    )
                    .unwrap();
                    writeln!(out, "                    p.{},", gate.field).unwrap();
                    writeln!(out, "                    embassy_nrf::gpio::Level::Low,").unwrap();
                    writeln!(
                        out,
                        "                    embassy_nrf::gpio::OutputDrive::Standard,"
                    )
                    .unwrap();
                    writeln!(out, "                ))),").unwrap();
                }
                None => writeln!(
                    out,
                    "                crate::board::PointerPower::new(None),"
                )
                .unwrap(),
            }
            writeln!(out, "                )))").unwrap();
            writeln!(out, "            }},").unwrap();
        }
    }

    // The panel last, so the tuple's other four keep their positions. Wired but
    // deliberately almost unused - see the bisect note in src/screen/mod.rs.
    writeln!(out, "            crate::screen::new_screen(").unwrap();
    writeln!(out, "                p.SPI2,").unwrap();
    writeln!(out, "                p.{},", half.screen.0.field).unwrap();
    writeln!(out, "                p.{},", half.screen.1.field).unwrap();
    writeln!(out, "                p.{},", half.screen.2.field).unwrap();
    writeln!(out, "            ),").unwrap();
    // Then the battery, which the panel path also owns: it is read between
    // frames, never from the typing loop.
    writeln!(out, "            crate::screen::new_battery(").unwrap();
    writeln!(out, "                p.SAADC,").unwrap();
    writeln!(out, "                p.{}.degrade_saadc(),", half.battery.field).unwrap();
    writeln!(out, "            ),").unwrap();
    // Last, and for the screen task alone: it does not change what the half does,
    // only which frame the picture starts on.
    writeln!(out, "            boot_seed,").unwrap();
    writeln!(out, "        )").unwrap();
    writeln!(out, "    }}\n").unwrap();

    writeln!(out, "}}\n").unwrap();
}

fn write_generated(out_dir: &std::path::Path, b: &Board) {
    let mut s = String::new();
    s.push_str("// @generated by build.rs from board.toml - do not edit.\n\n");

    s.push_str(&format!("pub const CHIP: &str = {:?};\n", b.chip));
    s.push_str(&format!("pub const ROWS: usize = {};\n", b.rows));
    s.push_str(&format!("pub const COLS: usize = {};\n", b.cols));
    s.push_str("pub const CELLS: usize = ROWS * COLS;\n");
    s.push_str(&format!(
        "pub const DIODE_IS_COL2ROW: bool = {};\n",
        b.col2row
    ));
    s.push_str(&format!("pub const SETTLE_CYCLES: u32 = {};\n", b.settle_cycles));
    s.push_str(&format!(
        "pub const DRIVE_PINS: usize = {};\n",
        if b.col2row { b.cols } else { b.rows }
    ));
    s.push_str(&format!(
        "pub const READ_PINS: usize = {};\n",
        if b.col2row { b.rows } else { b.cols }
    ));

    s.push_str("\n// ---- Gazell device parameters ----\n");
    s.push_str(&format!("pub const DATARATE: u32 = {};\n", b.datarate));
    s.push_str(&format!(
        "pub const TIMESLOT_PERIOD_US: u32 = {};\n",
        b.timeslot_period_us
    ));
    s.push_str(&format!("pub const MAX_TX_ATTEMPTS: u32 = {};\n", b.max_tx_attempts));
    s.push_str(&format!(
        "pub const TIMESLOTS_PER_CHANNEL: u32 = {};\n",
        b.timeslots_per_channel
    ));
    s.push_str(&format!(
        "pub const TIMESLOTS_PER_CHANNEL_WHEN_OUT_OF_SYNC: u32 = {};\n",
        b.timeslots_per_channel_when_out_of_sync
    ));
    s.push_str(&format!(
        "pub const SYNC_LIFETIME_TIMESLOTS: u32 = {};\n",
        b.sync_lifetime_timeslots
    ));
    s.push_str(&format!(
        "pub const CHANNEL_SELECTION_POLICY: u32 = {};\n",
        b.channel_selection_policy
    ));
    s.push_str(&format!(
        "pub const BASE_ADDRESS_0: u32 = 0x{:08X};\n",
        b.base_address_0
    ));
    s.push_str(&format!(
        "pub const BASE_ADDRESS_1: u32 = 0x{:08X};\n",
        b.base_address_1
    ));
    s.push_str(&format!("pub const XOSC_MANUAL: bool = {};\n", b.xosc_manual));
    s.push_str(&format!(
        "pub const RADIO_OFF_WHEN_IDLE: bool = {};\n",
        b.radio_off_when_idle
    ));

    s.push_str("\n// ---- encoder ----\n");
    s.push_str(&format!(
        "pub const ENCODER_RESOLUTION: i8 = {};\n",
        b.encoder_resolution
    ));

    s.push_str("\n// ---- timing ----\n");
    s.push_str(&format!("pub const SCAN_HZ: u32 = {};\n", b.scan_hz));
    s.push_str(&format!(
        "pub const SCAN_PERIOD_US: u64 = {};\n",
        1_000_000u64 / b.scan_hz.max(1) as u64
    ));
    s.push_str(&format!("pub const DEBOUNCE_TICKS: u32 = {};\n", b.debounce_ticks));
    s.push_str(&format!("pub const IDLE_POLL_MS: u32 = {};\n", b.idle_poll_ms));
    s.push_str(&format!(
        "pub const KEEPALIVE_MS: u32 = {};\n",
        b.keepalive_ms
    ));
    s.push_str(&format!(
        "pub const ACTIVE_RELEASE_MS: u32 = {};\n",
        b.active_release_ms
    ));

    s.push_str("\n// ---- power ----\n");
    s.push_str(&format!("pub const REG1_FORCE_LDO: bool = {};\n", b.reg1_ldo));
    s.push_str(&format!("pub const REG1_FORCE_DCDC: bool = {};\n", b.reg1_dcdc));

    s.push_str("\n// ---- bootloader gesture ([dfu]) ----\n");
    // The cells themselves are per half (each half sees only its own rows), so they
    // are emitted in each half's module; the policy is one for the keyboard.
    s.push_str(&format!("pub const DFU_ENABLED: bool = {};\n", b.dfu.enabled));
    s.push_str(&format!(
        "pub const DFU_REQUIRE_USB: bool = {};\n",
        b.dfu.require_usb
    ));
    s.push_str(&format!("pub const DFU_WINDOW_MS: u32 = {};\n", b.dfu.window_ms));
    s.push_str(&format!("pub const DFU_GRACE_MS: u32 = {};\n", b.dfu.grace_ms));
    s.push_str(&format!(
        "pub const DFU_RELEASE_PACKETS: u32 = {};\n",
        b.dfu.release_packets
    ));

    s.push_str("\n// ---- per-half ----\n");
    let mut out = String::new();
    write_half_module(b, "left", &b.left, &mut out);
    write_half_module(b, "right", &b.right, &mut out);
    s.push_str(&out);

    fs::write(out_dir.join("board_generated.rs"), s).unwrap();
}

// ---------------------------------------------------------------------------

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    println!("cargo:rerun-if-changed=board.toml");
    println!("cargo:rerun-if-changed=build.rs");

    let board = load_board();

    // ---- chip <-> feature validation ---------------------------------------
    // A layout that cannot fit the chip, or a feature/board mismatch, is the
    // classic "flashes fine, never boots" mistake - so it is a build error.
    let (chip_flash, chip_ram_base, chip_ram_size) = match board.chip.as_str() {
        "nrf52840" => (1024 * 1024u32, 0x2000_0000u32, 256 * 1024u32),
        "nrf52833" => (512 * 1024, 0x2000_0000, 128 * 1024),
        "nrf52832" => (512 * 1024, 0x2000_0000, 64 * 1024),
        other => panic!(
            "board.toml: [chip] name = {other:?} is not supported by this crate's feature list \
             (nrf52840 / nrf52833 / nrf52832). Add the feature in Cargo.toml and the chip's \
             flash/RAM sizes here if you need another part."
        ),
    };
    let enabled: Vec<&str> = ["nrf52840", "nrf52833", "nrf52832"]
        .into_iter()
        .filter(|c| env::var(format!("CARGO_FEATURE_{}", c.to_uppercase())).is_ok())
        .collect();
    if enabled.len() != 1 {
        panic!(
            "exactly one chip feature must be enabled, found {enabled:?}. Use \
             `cargo build --release --features nrf52833 --no-default-features`."
        );
    }
    if enabled[0] != board.chip {
        panic!(
            "chip mismatch: cargo feature `{}` is enabled but board.toml says [chip] name = {:?}. \
             They must agree, or the linker script and the PAC describe different chips.",
            enabled[0], board.chip
        );
    }

    let flash_end = board.flash_origin + board.flash_length;
    if flash_end > chip_flash {
        panic!(
            "board.toml: [memory] flash_origin + flash_length = 0x{:X}..0x{:X} exceeds {} flash \
             (0x{:X} bytes).",
            board.flash_origin, flash_end, board.chip, chip_flash
        );
    }
    let ram_end = board.ram_origin + board.ram_length;
    if board.ram_origin < chip_ram_base || ram_end > chip_ram_base + chip_ram_size {
        panic!(
            "board.toml: [memory] RAM 0x{:X}..0x{:X} is outside {} RAM (0x{:X}..0x{:X}).",
            board.ram_origin,
            ram_end,
            board.chip,
            chip_ram_base,
            chip_ram_base + chip_ram_size
        );
    }

    write_generated(&out, &board);

    // ---- memory.x, generated so the layout is a board.toml entry -----------
    fs::write(
        out.join("memory.x"),
        format!(
            "/* @generated by build.rs from board.toml - do not edit. */\n\
             MEMORY\n{{\n\
             \x20 FLASH : ORIGIN = 0x{:08X}, LENGTH = {}K\n\
             \x20 RAM   : ORIGIN = 0x{:08X}, LENGTH = {}K\n\
             }}\n",
            board.flash_origin,
            board.flash_length / 1024,
            board.ram_origin,
            board.ram_length / 1024
        ),
    )
    .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // `--nmagic`: required because our FLASH origin (0x1000) is not 64K-aligned.
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    // ---- the closed-source Nordic Gazell archive ---------------------------
    // What it needs from us (memcpy/memset plus four nrf_gzll_* callbacks) and
    // which ISRs it defines are documented in src/gazell.rs; that contract was
    // read out with arm-none-eabi-nm rather than assumed.
    let gzll_lib = env::var("GZLL_LIB").unwrap_or_else(|_| board.library.clone());
    let gzll_dir = {
        let raw = env::var("GZLL_DIR").unwrap_or_else(|_| board.library_dir.clone());
        let p = std::path::Path::new(&raw);
        // A relative value in board.toml is resolved against the package root, so
        // "../vendor/gzll" means the same thing from a clone anywhere on earth and
        // the vendored Nordic archive is the default. GZLL_DIR still wins, which is
        // how a different SDK build of the library gets tried without code changes.
        let p = if p.is_relative() {
            std::path::Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap()).join(p)
        } else {
            p.to_path_buf()
        };
        if !p.join(&gzll_lib).is_file() {
            panic!(
                "Gazell archive '{}' not found in '{}'.\n\
                 Commit it under vendor/gzll (Nordic licence permits redistributing the\n\
                 binary alongside license.txt) or point GZLL_DIR at an nRF5 SDK\n\
                 'components/proprietary_rf/gzll/gcc' directory.",
                gzll_lib,
                p.display()
            );
        }
        p.display().to_string()
    };
    println!("cargo:rustc-link-search=native={gzll_dir}");
    // `+verbatim`: the file is `gzll_nrf52840_gcc.a`, not `libgzll_nrf52840_gcc.a`.
    println!("cargo:rustc-link-lib=static:+verbatim={gzll_lib}");
    println!("cargo:rerun-if-changed={gzll_dir}/{gzll_lib}");
    println!("cargo:rerun-if-env-changed=GZLL_DIR");
    println!("cargo:rerun-if-env-changed=GZLL_LIB");

    // "0" on its own reads like a value nobody filled in, so it is spelled out.
    // What the number controls: how often an idle half re-sends its unchanged
    // (all-released) state purely to keep Gazell in sync. At 0 the half sends
    // nothing while idle and the link lapses once sync_lifetime runs out, so the
    // first key of a burst after a pause longer than that pays a re-acquisition
    // search.
    //
    // The lapse is COMPUTED here rather than written into the string. It is
    // sync_lifetime x timeslot_period, both of which live elsewhere in board.toml,
    // and a hard-coded "27 ms" in a summary line is exactly how a line that exists
    // to prevent misunderstanding ends up causing one - this one did, within a day
    // of being written, when sync_lifetime moved off the library default.
    let lapse_ms =
        board.sync_lifetime_timeslots as u64 * board.timeslot_period_us as u64 / 1000;
    let keepalive = match board.keepalive_ms {
        0 => format!(
            "0 ms (OFF - an idle half sends nothing, so the link lapses {lapse_ms} ms \
             (sync_lifetime {} x {} us) after the last packet and the first key after \
             that pays a re-search)",
            board.sync_lifetime_timeslots, board.timeslot_period_us
        ),
        ms => format!(
            "{ms} ms (on - refreshes sync every {ms} ms, so the {lapse_ms} ms lapse \
             above can never be reached while the half has power)"
        ),
    };
    println!(
        "cargo:warning=tx: {} {}x{} halves, left pipe {} ch {:?}, right pipe {} ch {:?}, scan {} Hz, \
         idle poll {} ms, release after {} ms, radio off when idle: {}",
        board.chip,
        board.rows,
        board.cols,
        board.left.pipe,
        board.left.channel_table,
        board.right.pipe,
        board.right.channel_table,
        board.scan_hz,
        board.idle_poll_ms,
        board.active_release_ms,
        board.radio_off_when_idle
    );
    println!("cargo:warning=tx: idle keepalive: {keepalive}");

    // The switch, printed rather than left to be discovered in the generated file.
    // A gate that fails to parse is a gate that silently does nothing, so this is
    // the line that tells "wired" and "not wired" apart without a probe.
    let gate = |half: &Half| match &half.pointer_power {
        Some(pin) => pin.field.clone(),
        None => "none (software switch only)".to_string(),
    };

    // HOW TO READ THE TWO HALVES' CELL NUMBERS. board.toml writes every one of
    // these cells in WHOLE-KEYBOARD coordinates because that is the numbering Vial
    // shows and the one a person discusses a key position in: the left half owns
    // rows 0..rows, the right half rows rows..2*rows. Each half can only see its own
    // rows, so build.rs converts to LOCAL once, in parse_cell - and that is what the
    // firmware actually uses.
    //
    // These lines print BOTH, so neither number has to be converted in the reader's
    // head. The offset is board.rows, the row count of ONE half (6 here) - NOT
    // board.rows / 2, and not the joined 12-row picture the receiver sees.
    let half_rows = board.rows as usize;
    // cell is already LOCAL, so going the other way means ADDING the right half's
    // offset. Doing this backwards is exactly the trap this line exists to avoid.
    let both = |cell: (usize, usize), left: bool| {
        let (r, c) = cell;
        let global_row = if left { r } else { r + half_rows };
        format!("local ({r}, {c}) = global (row {global_row}, col {c})")
    };
    let keys = |cell: Option<(usize, usize)>, left: bool| match cell {
        Some(cell) => both(cell, left),
        None => "none".to_string(),
    };
    println!(
        "cargo:warning=tx: pointer switches: idle off after {} ms. Coordinates below are \
         TOGGLE-KEY cells, given as local (what this half's firmware sees) = global (what \
         board.toml and Vial use); the right half's global row = local row + {}.",
        board.left.pointer_idle_off_ms,
        half_rows
    );
    println!(
        "cargo:warning=tx:   pointer power switch (MOSFET gate): left {} on {}, right {} on {}",
        gate(&board.left),
        keys(board.left.pointer_power_key, true),
        gate(&board.right),
        keys(board.right.pointer_power_key, false)
    );
    println!(
        "cargo:warning=tx:   pointer mode switch (cursor <-> wheel): left {}, right {}",
        keys(board.left.pointer_mode_key, true),
        keys(board.right.pointer_mode_key, false)
    );

    // The bootloader gesture gets a line of its own. It is the one setting in this
    // file whose failure mode is "the keyboard resets while I type", and its four
    // cells are written in whole-keyboard coordinates - i.e. in a numbering the
    // person reading this line is not looking at.
    // A combo is a SET of cells, so it gets its own printer rather than the
    // single-cell one above. Same rule as the switch keys: local = global on the
    // left, and the right half's rows are the left half's row count further down.
    let combo = |cells: &[(usize, usize)], left: bool| {
        let mut s = String::new();
        for (i, cell) in cells.iter().enumerate() {
            if i > 0 {
                s.push_str(", ");
            }
            s.push_str(&both(*cell, left));
        }
        s
    };
    if board.dfu.enabled {
        println!(
            "cargo:warning=tx: dfu combo: the FOUR CORNER CELLS (a set of cell locations, NOT \
             switch keys), local = global per half; the right half's global row = local row + \
             {}.",
            half_rows
        );
        println!("cargo:warning=tx:   left  [{}]", combo(&board.left.dfu_combo, true));
        println!("cargo:warning=tx:   right [{}]", combo(&board.right.dfu_combo, false));
        println!(
            "cargo:warning=tx:   all four within {} ms, {} ms grace, {} release packets, usb \
             cable required: {}",
            board.dfu.window_ms,
            board.dfu.grace_ms,
            board.dfu.release_packets,
            board.dfu.require_usb
        );
    } else {
        println!(
            "cargo:warning=tx: dfu combo DISABLED ([dfu] enabled = false) - the reset button is \
             the only way into a half's bootloader"
        );
    }
}