//! Status screen for a half's own LPM009M360A panel (72x144 portrait).
//!
//! A half transmits and never receives, so there is nothing it could honestly
//! show about layers, keymaps, the host, or the link. Everything here is
//! something the half knows by itself:
//!
//! ```text
//! ┌──────────────┐
//! │         [█] 87│ y2 battery bar (22x11) + number, 6x13
//! │              │
//! │   (capybara  │ y9 animation, 70x120, one frame per ten minutes
//! │    frames)   │
//! │              │
//! └──────────────┘
//! ```
//!
//! The battery is the top row's whole content. It replaced an activity word
//! (`RUN`/`IDLE`) that was the screen's original point - and the replacement was
//! forced, not chosen: showing `RUN` means composing a frame the moment typing
//! starts, and that is exactly the instant a transfer costs keystrokes. See the
//! long note in `screen::mod`. The battery has no such problem, because it can be
//! read whenever the screen is allowed to be touched at all.
//!
//! What is left on screen is therefore: the animation, which advances once per
//! typing break, and the battery, which is sampled at the same moments. Nothing
//! here changes while a hand is on the keyboard.
//!
//! Coordinates are carried over from the BLE firmware, where they were tuned
//! against the real panel. Do not rescale or re-derive them from datasheet or
//! physical-glass numbers.

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use embassy_time::{Duration, Instant};

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_6X13};
use embedded_graphics::mono_font::MonoTextStyle;

use super::PointerUi;
use embedded_graphics::pixelcolor::{Rgb888, RgbColor};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;

/// The palette, in one place.
///
/// The four-bit transport is in place and works, but colour is not being spent on
/// the interface. That was tried: the battery went green/amber/red by level and
/// the state line red/cyan/yellow by state. It was judged not worth having - the
/// text was legible in white, the level is already carried by the digits and the
/// length of the bar, and colour on the small elements made the screen busier
/// without making it clearer. White ink on the same black ground as before.
///
/// The transport stays, because a drawing is where colour would earn its place, and
/// that needs the four bits whether or not the text uses them. `CAPY_INK` is the hook
/// for it: the animation is one bit deep, so it is a single ink, and a colour drawing
/// would come in as its own packed data and be drawn with its own colours rather than
/// through this constant - which is exactly what happened for one evening
/// (2026-09-26) before the animation came back.
///
/// Black ground, deliberately, and it is also why the ink is white rather than
/// something softer: the capybara is the largest thing on screen, it is line art,
/// and it wants the highest contrast the glass can give.
const PAPER: Rgb888 = Rgb888::new(0, 0, 0);

/// Ink for text and for the battery. One value, because the interface is one
/// colour; a second ink here would be a decision, not a detail.
const INK: Rgb888 = Rgb888::WHITE;

/// The capybara's ink. One value, because the animation is line art and one bit
/// deep: `blit` lights a pixel in this colour.
///
/// A colour drawing would not come through here - it arrives as its own packed data
/// and is drawn with its own colours, which is what `blit_colour` below is for. The
/// halves showed such drawings for one evening (2026-09-26); Lemon called them off
/// the same night ("放弃图片版，还原capy轮播版") and the animation is back, which is
/// what this constant is for.
const CAPY_INK: Rgb888 = Rgb888::WHITE;
/// The device-name row's face: one size down from `HEAD` (Lemon, 2026-09-18).
///
/// Same 6px cell, so the two rows sit on one grid and the name stays centred under
/// the state word - and `TRACK POINT` at 11 characters still fits a 72px panel
/// (66px), which it would not at a wider step down.
// The two faces are built per draw rather than held as constants, because their
// colour is not fixed: the battery's ink depends on the level and the state line's
// on the state. `MonoTextStyle::new` is a couple of moves, and this runs once per
// repaint, never on the typing path.

/// Baseline of the state line's lower row - the row that reads `OFFLINE`,
/// `POINTER MODE` or `SCROLL MODE`.
///
/// 143 until 2026-09-26, when Lemon asked for that row to come up a pixel - it became
/// 142. The same evening he moved it up once more, to 141, and the pitch did not
/// change, so the device-name row above it came up with it, to 128. The whole block
/// now sits two pixels higher than it started, which is what he asked for.
pub const POINTER_Y: i32 = 141;
/// Row pitch of the state line: two rows of `HEAD`, one above the other.
///
/// 14 until 2026-09-26, 13 since. The pitch did not change for its own sake: the lower
/// row moved up a pixel and 13 is what keeps the two 13 apart, so the name row above it
/// moved with it. The pair is now 141 and 128.
///
/// The line began as a single row of the 4x6 face, which is the only face that fits
/// `TKPAD ON/SCROLL` - 15 characters - on one row of a 72 px panel, and it was not
/// readable at that size (Lemon, 2026-09-18: "字符太小了，需要放大一倍"). It now uses
/// the same 6x13 face as the battery digits, so it is exactly as legible as they are,
/// and the phrase is split over two rows because it no longer fits on one: 15
/// characters at 6 px is 90 px, and the panel is 72.
const POINTER_LINE_PITCH: i32 = 13;
/// The rows the state line lives in, as a half-open range.
///
/// Also the driver's unit of work when the line changes: `flush_rows` sends exactly
/// these 28 of the panel's 144 lines - about 0.6 ms on the 4 MHz bus this was
/// measured at, against 3 ms for a whole frame; the bus runs at 32 MHz now (see
/// `new_screen`) - which is what makes it safe to update the line the moment a
/// switch key is pressed. Everything drawn between `POINTER_BAND.0` and `POINTER_BAND.1` must stay
/// inside it.
pub const POINTER_BAND: (i32, i32) = (116, 144);
const FILL_PAPER: PrimitiveStyle<Rgb888> = PrimitiveStyle::with_fill(PAPER);

// --- top-right: the battery ---------------------------------------------
//
// Geometry taken verbatim from the BLE firmware's panel, where it was tuned
// against the real glass. The bar and the digits are one chained group: the
// number is pinned to the right margin and the bar hangs off its left, so the two
// slide together as the digit count changes instead of the number jittering.
const TOP_Y: i32 = 8; //  battery bar & digits anchor here
/// The bar rides 6px above the anchor, i.e. y3..14.
const ICON_Y: i32 = TOP_Y - 6;
/// Digits sit 4px below the anchor: the 6x13 cell keeps dead space above its
/// glyphs, so this matches the optical centre of the 11px bar beside it.
const NUM_Y: i32 = TOP_Y + 4;
/// Battery number right-align edge (2px canvas margin).
const NUM_RIGHT_EDGE: i32 = 70;
const BAR_W: i32 = 22;
const BAR_H: i32 = 11;

/// Divider the sense pin sees. 2840 is not the board's ratio (the devicetree says
/// 2000 of 2820); it is the BLE firmware's deliberate nudge of the whole curve,
/// kept so a number on this panel still means what it meant there.
const ADC_DIVIDER_MEASURED: i32 = 2000;
const ADC_DIVIDER_TOTAL: i32 = 2840;

/// The two ends of the battery scale, in the units `scaled` comes out in.
///
/// 3953 counts is 3450 mV of cell and 4790 counts is 4180 mV. The empty end is
/// ZMK's (`lithium_ion_mv_to_pct` returns 0 at 3450 mV), the point where a li-ion
/// is genuinely finished rather than merely low. The full end is Lemon's: 4180 mV
/// rather than ZMK's 4200 mV, because these cells do not reach 4.2 V, and 4180 is
/// still high enough that a charged cell only sits on the ceiling while it really
/// is charged.
///
/// Replaced (2026-09-19) the BLE firmware's calibration: 4055 counts empty
/// (3539 mV), 4755 full (4149 mV), one percent per 7 counts.
const BATTERY_EMPTY_COUNTS: i32 = 3953;
    /// The full anchor is no longer a constant here: it is measured on each half and
    /// arrives as an argument. See `board::Role::battery_full_counts` - 4790 counts
    /// (4180 mV) on the left, 4750 (4145 mV) on the right.
    ///
/// The full anchor used to live here as `BATTERY_FULL_COUNTS = 4790`; it is per half
    /// now and comes in as `full_counts` - see the note above.

/// Convert a raw SAADC reading into a percentage, or `None` when the reading
/// cannot be a battery at all.
///
/// `scaled` is the cell voltage in ADC counts per volt: with the ADC's default
/// gain of 1/6 and 0.6 V reference a raw count is 1137.8 per volt at the pin, so
/// `raw * 2840 / 2000` comes to 1145.9 counts per volt of cell - which is why
/// 4180 mV lands on 4790 and 3450 mV on 3953.
///
/// Between the ends the mapping is linear, as ZMK's is: ZMK approximates the
/// discharge curve with the straight line `mV * 2 / 15 - 459` between the same
/// kind of endpoints. A straight line is what the ecosystem ships, and over this
/// span it tracks the real curve closely enough that a one-digit display cannot
/// tell. One `sdiv`, once per repaint, off the typing path.
    ///
    /// `full_counts` is the top of the scale on this half, in the same units - the one
    /// number in the battery path that is measured per half rather than shared.
///
/// Anything below 500 counts is treated as "nothing is measuring a cell" rather
/// than as an empty one: that is a grounded or floating sense line, and showing
/// 0% for it would be a confident lie.
pub fn percent_from_raw(raw: i32, full_counts: i32) -> Option<u8> {
    if raw < 500 {
        return None;
    }
    // Rounded to nearest rather than truncated: half a count of truncation is enough
    // to leave a cell sitting exactly on the full anchor one percent short of it.
    let scaled = (raw * ADC_DIVIDER_TOTAL + ADC_DIVIDER_MEASURED / 2) / ADC_DIVIDER_MEASURED;
    let pct = if scaled >= full_counts {
        100
    } else if scaled <= BATTERY_EMPTY_COUNTS {
        0
    } else {
        (scaled - BATTERY_EMPTY_COUNTS) * 100 / (full_counts - BATTERY_EMPTY_COUNTS)
    };
    Some(pct.clamp(0, 100) as u8)
}

// --- capybara animation (ZMK peripheral_status.c algorithm, ported) ------
//
// Nine 72x120 portrait frames, one frame every ten minutes, cycling forward.
// The boot frame is RANDOM: one number drawn from the chip's RNG in `pins()`,
// handed over by `set_boot_seed`. Two switch-ons must not start on the same
// picture, and the RNG is the only thing on this chip that differs between them -
// the device id is stable per chip and RAM does not survive a power cycle.
//
// (ZMK derives its boot frame from the device id, so its picture always starts the
// same way. That is the one behaviour this deviates from, deliberately. The
// per-half salt is only kept so the two halves stay apart when the draws land
// close.)
//
// Frames advance only when something draws, which on an event-driven screen
// means only when the loop was already awake for another reason. There is no
// timer behind this - see `screen::task`.

const CAPY_INTERVAL: Duration = Duration::from_secs(600);
/// 0xFF = not yet seeded.
static CAPY_FRAME: AtomicU8 = AtomicU8::new(0xFF);
static CAPY_SINCE: AtomicU32 = AtomicU32::new(0); // ms of last switch, wrapping
/// The number the boot frame is drawn from. 0 = never handed over.
static CAPY_SEED: AtomicU32 = AtomicU32::new(0);

/// Hand over one random number, which the boot frame is drawn from.
///
/// Called once from the half's `main`, with the value `pins()` read out of the RNG.
/// A static rather than a parameter because the frame is drawn from deep inside the
/// drawing path, and this happens once, before the screen task is spawned.
pub fn set_boot_seed(seed: u32) {
    CAPY_SEED.store(seed, Ordering::Relaxed);
}

/// FNV-1a over DEVICEID[0..1] (FICR @ 0x1000_0060, unique per chip).
///
/// NB: 0x1000_0100 is INFO.PART/VARIANT/... - model-common values, NOT a
/// fingerprint (was the original bug).
///
/// Only a fallback, for a build that never hands a seed over. It is stable per
/// chip, so the picture would then start on the same frame every time - the very
/// thing the seed is there to avoid.
fn device_id_hash() -> u32 {
    let mut h: u32 = 2166136261;
    for i in 0..2u32 {
        // FICR.DEVICEID[i] @ 0x1000_0060: factory-programmed unique ID, read-only.
        let w = unsafe { core::ptr::read_volatile((0x1000_0060 + i * 4) as *const u32) };
        h = (h ^ (w & 0xFF)).wrapping_mul(16777619);
        h = (h ^ (w >> 8 & 0xFF)).wrapping_mul(16777619);
        h = (h ^ (w >> 16 & 0xFF)).wrapping_mul(16777619);
        h = (h ^ (w >> 24)).wrapping_mul(16777619);
    }
    h
}

fn capy_frame(salt: u32) -> usize {
    let now_ms = Instant::now().as_millis() as u32;
    let f = CAPY_FRAME.load(Ordering::Relaxed);
    if f == 0xFF {
        let seed = CAPY_SEED.load(Ordering::Relaxed);
        let base = if seed != 0 { seed } else { device_id_hash() };
        let idx = (base ^ salt).wrapping_mul(16777619) % 9;
        CAPY_FRAME.store(idx as u8, Ordering::Relaxed);
        CAPY_SINCE.store(now_ms, Ordering::Relaxed);
        idx as usize
    } else if now_ms.wrapping_sub(CAPY_SINCE.load(Ordering::Relaxed))
        >= CAPY_INTERVAL.as_millis() as u32
    {
        let nf = ((f as usize + 1) % 9) as u8;
        CAPY_FRAME.store(nf, Ordering::Relaxed);
        CAPY_SINCE.store(now_ms, Ordering::Relaxed);
        nf as usize
    } else {
        f as usize
    }
}

fn draw_capy<D: DrawTarget<Color = Rgb888>>(d: &mut D, frame: usize) {
    // 70x120 (the generator trims the white edges), full size, at x1 - one pixel clear
    // of each side, as it was in the BLE firmware. y9 (Lemon, 2026-09-18: "图片 y设成9").
    //
    // The fixed drawings that briefly replaced this (2026-09-26, one evening) sat at
    // y13: they were sized to the glass by eye and walked down three times that night.
    // The animation is back at its own y9, which is where the ZMK firmware put it and
    // what this geometry was tuned for.
    //
    // The state line's band starts at row 116, so it covers the bottom 15 rows of the
    // picture. That is the deliberate trade: at this size there is no placement that
    // avoids it - 120 rows of picture plus 28 rows of caption do not fit on a
    // 144-row panel - and the frame is worth more than its feet. The top rows sit
    // under the battery bar and its digits, which are drawn after it and win, which
    // is where y11 put it originally.
    blit(
        d,
        &crate::screen::capy_art::CAPY_FRAMES[frame],
        crate::screen::capy_art::CAPY_W as i32,
        crate::screen::capy_art::CAPY_H as i32,
        9,
        crate::screen::capy_art::CAPY_W as i32,
        crate::screen::capy_art::CAPY_H as i32,
        1,
        9,
    );
}

fn txt<D: DrawTarget<Color = Rgb888>>(
    d: &mut D,
    s: &str,
    x: i32,
    y: i32,
    style: MonoTextStyle<'static, Rgb888>,
) {
    Text::new(s, Point::new(x, y), style).draw(d).ok();
}

/// The same string drawn twice, one pixel apart, so its strokes come out two pixels
/// wide.
///
/// That is what "bold" can mean here: embedded-graphics ships one weight per face and
/// no synthetic weight, and the panel is one bit per pixel, so the only lever is
/// where the pixels land. Horizontal only - shifting vertically as well closes the
/// 6x13 cell's row gap and the glyphs smear into a solid bar.
///
/// The old note here said the panel was one bit per pixel; it is four now, but the
/// reason for the two-pass bold is unchanged: embedded-graphics ships one weight
/// per face and no synthetic weight, so the only lever is still where the pixels
/// land.
///
/// It costs one more pass over the glyphs (a few hundred `draw_iter` calls, in the
/// screen task, never on the typing path) and widens each string by 1px.
fn txt_bold<D: DrawTarget<Color = Rgb888>>(
    d: &mut D,
    s: &str,
    x: i32,
    y: i32,
    style: MonoTextStyle<'static, Rgb888>,
) {
    txt(d, s, x, y, style);
    txt(d, s, x + 1, y, style);
}

/// 1-bit bitmap blit, MSB-first, `stride` bytes per row, rescaled to `out_w` x `out_h`.
///
/// The rescale is a box-OR, not a point sample: a destination pixel is on if ANY
/// source pixel in the box it covers is on. For line art that distinction is the
/// whole thing - point sampling a 5/6 reduction drops every sixth stroke and the
/// drawing comes out dotted, while OR keeps every stroke and merely thickens the
/// finest ones.
///
/// The source is one bit deep and stays that way: the artwork is line art, and the
/// four-bit transport is used here only to say which colour a lit pixel gets.
/// One-bit blit, the path the capybara comes through: the four-bit transport is used
/// here only to say which colour a lit pixel gets, and that colour is `CAPY_INK`.
#[allow(clippy::too_many_arguments)]
fn blit<D: DrawTarget<Color = Rgb888>>(
    d: &mut D,
    bits: &[u8],
    w: i32,
    h: i32,
    stride: usize,
    out_w: i32,
    out_h: i32,
    x0: i32,
    y0: i32,
) {
    if out_w <= 0 || out_h <= 0 || w <= 0 || h <= 0 {
        return;
    }
    for oy in 0..out_h {
        let sy0 = oy * h / out_h;
        let sy1 = (((oy + 1) * h) / out_h).max(sy0 + 1).min(h);
        for ox in 0..out_w {
            let sx0 = ox * w / out_w;
            let sx1 = (((ox + 1) * w) / out_w).max(sx0 + 1).min(w);
            let mut on = false;
            'source: for sy in sy0..sy1 {
                let row = &bits[(sy as usize) * stride..(sy as usize) * stride + stride];
                for sx in sx0..sx1 {
                    if row[sx as usize / 8] & (0x80 >> (sx as usize % 8)) != 0 {
                        on = true;
                        break 'source;
                    }
                }
            }
            if on {
                Rectangle::new(Point::new(x0 + ox, y0 + oy), Size::new(1, 1))
                    .into_styled(PrimitiveStyle::with_fill(CAPY_INK))
                    .draw(d)
                    .ok();
            }
        }
    }
}

/// One of the panel's eight values back as a colour, for drawing a colour bitmap.
///
/// These are the same eight the driver quantises towards - the table exists twice
/// on purpose, because the driver's copy goes colour-to-nibble and this one goes
/// nibble-to-colour, and folding them into one table would mean carrying the
/// reverse mapping through the transport for no reason.
#[allow(dead_code)]
fn colour_of(nibble: u8) -> Rgb888 {
    match nibble & 0x0F {
        0x2 => Rgb888::new(0, 0, 255),
        0x4 => Rgb888::new(0, 255, 0),
        0x6 => Rgb888::new(0, 255, 255),
        0x8 => Rgb888::new(255, 0, 0),
        0xA => Rgb888::new(255, 0, 255),
        0xC => Rgb888::new(255, 255, 0),
        0xE => Rgb888::WHITE,
        _ => Rgb888::BLACK,
    }
}

/// Four-bit bitmap blit, same geometry as `blit` above, in the panel's own pixel
/// format - what `tools/png_to_screen.py` produces.
///
/// The rescale picks, for each destination pixel, the first non-black source pixel
/// in its box. That is the colour analogue of `blit`'s box-OR and it is chosen for
/// the same reason: a stroke covering only part of the box still shows, and strokes
/// never average into a colour that is in neither the stroke nor the ground.
///
/// Black is treated as transparent rather than as ink, because here it IS the
/// ground. A drawing with a black background would disappear into the screen; a
/// drawing meant for this glass is drawn on nothing.
///
/// Nothing calls this: the screen shows the one-bit capybara, which `blit` draws.
/// This is the path a colour drawing arrives through - generate it with
/// `tools/png_to_screen.py`, put the array where `capy_art.rs` lives, and draw it from
/// a function shaped like `draw_capy`. One evening's worth of such drawings went
/// through it (2026-09-26) and were parked in `_quarantine-2026-09-26/art-pictures/`
/// (not part of this repository), with `make_art.py` able to rebuild them.
#[allow(dead_code, clippy::too_many_arguments)]
fn blit_colour<D: DrawTarget<Color = Rgb888>>(
    d: &mut D,
    data: &[u8],
    w: i32,
    h: i32,
    stride: usize,
    out_w: i32,
    out_h: i32,
    x0: i32,
    y0: i32,
) {
    if out_w <= 0 || out_h <= 0 || w <= 0 || h <= 0 {
        return;
    }
    for oy in 0..out_h {
        let sy0 = oy * h / out_h;
        let sy1 = (((oy + 1) * h) / out_h).max(sy0 + 1).min(h);
        for ox in 0..out_w {
            let sx0 = ox * w / out_w;
            let sx1 = (((ox + 1) * w) / out_w).max(sx0 + 1).min(w);
            let mut nibble = 0u8;
            'source: for sy in sy0..sy1 {
                let row = &data[(sy as usize) * stride..(sy as usize) * stride + stride];
                for sx in sx0..sx1 {
                    let byte = row[sx as usize / 2];
                    let value = if sx % 2 == 0 { byte >> 4 } else { byte & 0x0F };
                    if value != 0 {
                        nibble = value;
                        break 'source;
                    }
                }
            }
            if nibble != 0 {
                Rectangle::new(Point::new(x0 + ox, y0 + oy), Size::new(1, 1))
                    .into_styled(PrimitiveStyle::with_fill(colour_of(nibble)))
                    .draw(d)
                    .ok();
            }
        }
    }
}

/// Paint the whole screen.
///
/// `level` is the battery percentage, or `None` when the sense line is not
/// measuring a cell - which draws as `--` rather than as an empty battery.
///
/// `salt` seeds the animation's starting frame and must differ between the two
/// halves, or they would step in lockstep. The caller picks it, so nothing in here
/// has to know which half it is running on - the two binaries differ only in which
/// salt they hand over.
pub fn draw_ui<D: DrawTarget<Color = Rgb888>>(
    d: &mut D,
    level: Option<u8>,
    salt: u32,
    pointer: PointerUi,
    label: &str,
) {
    d.clear(PAPER).ok();
    draw_capy(d, capy_frame(salt));

    // Number pinned to the right margin, bar hanging off its left. The digits are
    // built by hand rather than with a formatter: the value is 0..=100, so there
    // are three cases and no reason to pull in a formatting stack or a string type
    // for them.
    // Digits, then a percent sign. Still built by hand rather than with a formatter:
    // the value is 0..=100, so there are three cases and no reason to pull in a
    // formatting stack or a string type for them.
    //
    // The sign counts in the width the right-align uses, which is what keeps the
    // number's right edge where it was when the sign appeared.
    let mut buf = [0u8; 4];
    let (text, digits): (&str, i32) = match level {
        Some(l) => {
            let l = l.min(100) as u32;
            let len = if l >= 100 {
                buf[..3].copy_from_slice(b"100");
                3
            } else if l >= 10 {
                buf[0] = b'0' + (l / 10) as u8;
                buf[1] = b'0' + (l % 10) as u8;
                2
            } else {
                buf[0] = b'0' + l as u8;
                1
            };
            buf[len] = b'%';
            (
                core::str::from_utf8(&buf[..len + 1]).unwrap_or("?"),
                (len + 1) as i32,
            )
        }
        // Nothing is measuring a cell. A percentage of nothing would be a confident
        // lie, and a percent sign on top of it would be one twice over, so this case
        // stays as it was.
        None => ("--", 2),
    };
    // Bold costs a pixel of width, so the number starts one pixel further left to
    // keep the same right margin - and the bar hangs off it as before, which is what
    // keeps the two from jittering apart as the digit count changes.
    let num_x = NUM_RIGHT_EDGE - digits * 6 - 1;
    // Cap ends 2px clear of the digits, one more than it used to be (Lemon,
    // 2026-09-18: "电池图标与数字之间间距加1px"). The bar moves as a unit with the
    // number, which is what keeps the two from jittering apart as the digit count
    // changes.
    let bar_x = num_x - 4 - BAR_W;

    let head = MonoTextStyle::new(&FONT_6X13, INK);
    let fill = PrimitiveStyle::with_fill(INK);

    txt_bold(d, text, num_x, NUM_Y, head);

    // Bar: outline, then the inside punched out, then the charge, then the cap.
    Rectangle::new(Point::new(bar_x, ICON_Y + 1), Size::new(BAR_W as u32, BAR_H as u32))
        .into_styled(fill)
        .draw(d)
        .ok();
    Rectangle::new(
        Point::new(bar_x + 1, ICON_Y + 2),
        Size::new((BAR_W - 2) as u32, (BAR_H - 2) as u32),
    )
    .into_styled(FILL_PAPER)
    .draw(d)
    .ok();
    if let Some(l) = level {
        // 18px of usable interior; never zero-width, so "0%" still reads as a
        // battery that is merely empty rather than as no battery at all.
        let w = (l as i32 * (BAR_W - 4) / 100).clamp(1, BAR_W - 4);
        Rectangle::new(Point::new(bar_x + 2, ICON_Y + 3), Size::new(w as u32, (BAR_H - 4) as u32))
            .into_styled(fill)
            .draw(d)
            .ok();
    }
    Rectangle::new(Point::new(bar_x + BAR_W, ICON_Y + 4), Size::new(2, 5))
        .into_styled(fill)
        .draw(d)
        .ok();

    // Last, so nothing can paint over it.
    draw_pointer(d, pointer, label);
}

/// The pointing device's state, on one line along the bottom.
///
/// `TKPAD ON/SCROLL` is 15 characters. At the 6x13 face used for the battery that
/// would need 90 px of a 72 px-wide panel, so this line gets the 4x6 face - the
/// only one that fits the whole phrase without splitting it or abbreviating it.
///
/// The band is filled with the background first. The line sits over the last rows
/// of the animation, so text drawn straight onto the picture would be legible or
/// not depending on which frame is up.
///
/// `label` names this half's own device - the two halves do not carry the same part
/// and must not print the same word.
pub fn draw_pointer<D: DrawTarget<Color = Rgb888>>(d: &mut D, pointer: PointerUi, label: &str) {
    // Spelled out (Lemon, 2026-09-18). Widths at 6px per character, against a 72px
    // panel: `SCROLL MODE` is 11 -> 67 with the bold copy, `OFFLINE` is 7 -> 43, both
    // centred. `POINTER MODE` is 12 -> 73, one pixel wider than the panel, so it is
    // placed at x0 and its last column falls off the edge - one column of the final
    // `E`, which is not visible at this size. The alternative was a face narrow
    // enough to fit it, and that would have been smaller than the name above it.
    let tail: &str = match pointer {
        PointerUi::Off => "OFFLINE",
        PointerUi::Move => "POINTER MODE",
        PointerUi::Scroll => "SCROLL MODE",
    };

    // The band is filled with the background first. The line sits over the last rows
    // of the animation, so text drawn straight onto the picture would be legible or
    // not depending on which frame is up.
    Rectangle::new(
        Point::new(0, POINTER_BAND.0),
        Size::new(72, (POINTER_BAND.1 - POINTER_BAND.0) as u32),
    )
    .into_styled(FILL_PAPER)
    .draw(d)
    .ok();

    // Two centred rows: what the device is, then what it is doing. The name is one
    // size down from the state word, and both are bold - see `txt_bold` - with the
    // centring accounting for the extra pixel that costs.
    txt_bold(
        d,
        label,
        (72 - (label.len() as i32 * 6 + 1)) / 2,
        POINTER_Y - POINTER_LINE_PITCH,
        MonoTextStyle::new(&FONT_6X10, INK),
    );
    txt_bold(
        d,
        tail,
        (72 - (tail.len() as i32 * 6 + 1)) / 2,
        POINTER_Y,
        MonoTextStyle::new(&FONT_6X13, INK),
    );
}