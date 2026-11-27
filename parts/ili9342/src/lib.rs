//! An ILI9342C screen on SPI (320x240), as a virtual part for esp-emu. The
//! CoreS3 (in StackChan and the CoreS3 Lite) has one: CS on GPIO 3, D/C on
//! GPIO 35, 3-wire SPI.
//!
//! The part reads every SPI transfer to its chip select. D/C, as it was when
//! the transfer was clocked (`SpiTransfer.pin_levels`), says what the bytes
//! are: low is a command, high is its parameters or pixels. This is the
//! command set that M5GFX, ESP-IDF's `esp_lcd` and our `tr_lcd` use:
//!
//! | Command | In the part |
//! | --- | --- |
//! | 0x01 SWRESET | Back to the reset state (sleeping, display off, black) |
//! | 0x10 / 0x11 SLPIN / SLPOUT | Sleep: the screen shows nothing |
//! | 0x28 / 0x29 DISPOFF / DISPON | Display off / on |
//! | 0x20 / 0x21 INVOFF / INVON | Inversion (see below) |
//! | 0x2A / 0x2B CASET / RASET | The window, 4 bytes each |
//! | 0x2C / 0x3C RAMWR / RAMWRC | Pixels into the window, from its corner / from where they stopped |
//! | 0x36 MADCTL | Row/column order and exchange (MY, MX, MV), and BGR |
//! | 0x3A COLMOD | 16 bits per pixel (0x55), or 18 (0x66, 3 bytes a pixel) |
//!
//! Other commands (power, gamma, timing) are taken and ignored. Reads answer
//! zeros: M5GFX reads the panel ID by toggling GPIO pins itself, not through
//! the SPI peripheral, and only logs a warning when it gets no answer.
//!
//! Two facts about the CoreS3's panel, from the drivers: it is an IPS panel
//! that shows the colors inverted unless the firmware sends INVON (M5GFX sets
//! `invert = true`, tr_lcd sends 0x21), and its subpixels are in BGR order,
//! so MADCTL's BGR bit (0x08, what M5GFX and tr_lcd send) gives the right
//! colors.
//!
//! The page reads `state()`: `on` (awake and display on), `frames` (pixel
//! windows filled), and the rows that changed since the last read: `rows` as
//! "first,last" and `pixels` as base64 of those whole rows, RGB565
//! little-endian, as the screen shows them (after inversion and BGR).

use esp_part::prelude::*;
use std::cell::Cell;

const CS: &str = "CS";
const DC: &str = "DC";

pub const WIDTH: usize = 320;
pub const HEIGHT: usize = 240;

const MADCTL_MY: u8 = 0x80;
const MADCTL_MX: u8 = 0x40;
const MADCTL_MV: u8 = 0x20;
const MADCTL_BGR: u8 = 0x08;

/// The panel inverts unless the firmware sends INVON (an IPS panel).
const PANEL_INVERTS: bool = true;
/// The panel's subpixels are BGR: MADCTL BGR = 1 gives the right colors.
const PANEL_BGR: bool = true;

struct Ili9342 {
    cs: Option<u32>,
    dc: Option<u32>,
    /// What the firmware wrote, RGB565, by memory row and column. Empty
    /// until the first pixel: the engine's part-worker.js (esp-emu 1.0.0)
    /// reads `memory.buffer` before it first calls `part_buffer()`, which
    /// builds this device, so growing the memory there would detach that
    /// buffer and fail the start.
    fb: Vec<u16>,
    cmd: u8,
    args: Vec<u8>,
    /// The window, in the firmware's (MCU) column/row space.
    xs: u16,
    xe: u16,
    ys: u16,
    ye: u16,
    /// Where the next pixel goes, in the same space.
    x: u16,
    y: u16,
    /// A pixel's first byte(s), when a transfer ends inside one.
    partial: Vec<u8>,
    madctl: u8,
    bytes_per_pixel: usize,
    inverted: bool,
    awake: bool,
    display_on: bool,
    frames: u32,
    /// Rows changed since the page last read them (first, last).
    dirty: Cell<Option<(usize, usize)>>,
}

impl Default for Ili9342 {
    fn default() -> Self {
        Self {
            cs: None,
            dc: None,
            fb: Vec::new(),
            cmd: 0,
            args: Vec::new(),
            xs: 0,
            xe: (WIDTH - 1) as u16,
            ys: 0,
            ye: (HEIGHT - 1) as u16,
            x: 0,
            y: 0,
            partial: Vec::new(),
            madctl: 0,
            bytes_per_pixel: 2,
            inverted: false,
            awake: false,
            display_on: false,
            frames: 0,
            dirty: Cell::new(Some((0, HEIGHT - 1))),
        }
    }
}

impl Ili9342 {
    fn reset(&mut self) {
        *self = Self { cs: self.cs, dc: self.dc, ..Self::default() };
    }

    fn mark(&self, first: usize, last: usize) {
        let d = match self.dirty.get() {
            Some((a, b)) => (a.min(first), b.max(last)),
            None => (first, last),
        };
        self.dirty.set(Some(d));
    }

    fn mark_all(&self) {
        self.mark(0, HEIGHT - 1);
    }

    fn command(&mut self, c: u8) {
        self.cmd = c;
        self.args.clear();
        self.partial.clear();
        match c {
            0x01 => {
                self.reset();
                self.cmd = c;
            }
            0x10 => {
                self.awake = false;
                self.mark_all();
            }
            0x11 => {
                self.awake = true;
                self.mark_all();
            }
            0x20 | 0x21 => {
                self.inverted = c == 0x21;
                self.mark_all();
            }
            0x28 | 0x29 => {
                self.display_on = c == 0x29;
                self.mark_all();
            }
            0x2C => {
                self.x = self.xs;
                self.y = self.ys;
            }
            _ => {}
        }
    }

    fn data(&mut self, bytes: &[u8]) {
        if matches!(self.cmd, 0x2C | 0x3C) {
            self.pixels(bytes);
            return;
        }
        self.args.extend_from_slice(bytes);
        let a = &self.args;
        match self.cmd {
            0x2A if a.len() >= 4 => {
                self.xs = u16::from_be_bytes([a[0], a[1]]);
                self.xe = u16::from_be_bytes([a[2], a[3]]);
            }
            0x2B if a.len() >= 4 => {
                self.ys = u16::from_be_bytes([a[0], a[1]]);
                self.ye = u16::from_be_bytes([a[2], a[3]]);
            }
            0x36 if !a.is_empty() => {
                if (self.madctl ^ a[0]) & MADCTL_BGR != 0 {
                    self.mark_all();
                }
                self.madctl = a[0];
            }
            0x3A if !a.is_empty() => {
                // DBI bits 2:0: 0b101 is 16 bits a pixel, 0b110 is 18.
                self.bytes_per_pixel = if a[0] & 0x07 == 0x06 { 3 } else { 2 };
            }
            _ => {}
        }
    }

    /// Where a pixel at the firmware's (column, row) lands in memory.
    fn place(&self, c: u16, r: u16) -> Option<(usize, usize)> {
        let (mut col, mut row) = if self.madctl & MADCTL_MV != 0 { (r as usize, c as usize) } else { (c as usize, r as usize) };
        if col >= WIDTH || row >= HEIGHT {
            return None;
        }
        if self.madctl & MADCTL_MX != 0 {
            col = WIDTH - 1 - col;
        }
        if self.madctl & MADCTL_MY != 0 {
            row = HEIGHT - 1 - row;
        }
        Some((col, row))
    }

    fn pixels(&mut self, bytes: &[u8]) {
        if self.fb.is_empty() {
            self.fb = vec![0; WIDTH * HEIGHT];
        }
        let bpp = self.bytes_per_pixel;
        let mut buf = std::mem::take(&mut self.partial);
        buf.extend_from_slice(bytes);
        let whole = buf.len() / bpp * bpp;
        let (mut first, mut last) = (usize::MAX, 0);
        for px in buf[..whole].chunks_exact(bpp) {
            let rgb565 = if bpp == 2 {
                u16::from_be_bytes([px[0], px[1]])
            } else {
                // 18 bits: the top 6 bits of each byte, R, G, B.
                ((px[0] as u16 & 0xF8) << 8) | ((px[1] as u16 & 0xFC) << 3) | (px[2] as u16 >> 3)
            };
            if let Some((col, row)) = self.place(self.x, self.y) {
                self.fb[row * WIDTH + col] = rgb565;
                first = first.min(row);
                last = last.max(row);
            }
            self.x += 1;
            if self.x > self.xe {
                self.x = self.xs;
                self.y += 1;
                if self.y > self.ye {
                    self.y = self.ys;
                    self.frames = self.frames.wrapping_add(1);
                }
            }
        }
        self.partial = buf[whole..].to_vec();
        if first <= last {
            self.mark(first, last);
        }
    }

    fn on(&self) -> bool {
        self.awake && self.display_on
    }

    /// A pixel as the screen shows it.
    fn shown(&self, raw: u16) -> u16 {
        if !self.on() {
            return 0;
        }
        let mut p = raw;
        if self.inverted != PANEL_INVERTS {
            p = !p;
        }
        if (self.madctl & MADCTL_BGR != 0) != PANEL_BGR {
            p = (p & 0x07E0) | (p >> 11) | ((p & 0x1F) << 11);
        }
        p
    }

    fn transfer(&mut self, t: &SpiTransfer) {
        if let Some(cmd) = t.cmd.as_ref().and_then(|c| c.data.last()) {
            self.command(*cmd);
        }
        if t.tx.is_empty() {
            return;
        }
        let dc_high = self.dc.is_some_and(|dc| t.pin_levels >> dc & 1 == 1);
        if dc_high {
            self.data(&t.tx);
        } else {
            for &c in &t.tx {
                self.command(c);
            }
        }
    }
}

fn base64(bytes: &[u8]) -> String {
    const ABC: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for c in bytes.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            out.push(if i <= c.len() { ABC[(n >> (18 - 6 * i) & 63) as usize] as char } else { '=' });
        }
    }
    out
}

impl Device for Ili9342 {
    const NAME: &'static str = "ili9342";
    const ON_BUS: bool = true;

    fn configure(&mut self, config: &PartConfig) {
        self.cs = config.pins.get(CS).copied();
        self.dc = config.pins.get(DC).copied();
    }

    fn claims(&self) -> Vec<Claim> {
        self.cs
            .map(|cs| Claim { what: Some(claim::What::Spi(SpiClaim { cs_pin: Some(cs), ..Default::default() })) })
            .into_iter()
            .collect()
    }

    fn on_bus(&mut self, request: &BusRequest) -> Option<BusReply> {
        let Some(bus_request::Op::Spi(t)) = &request.op else {
            return Some(BusReply { correlation_id: request.correlation_id, op: BusOp::I2c as i32, ..Default::default() });
        };
        self.transfer(t);
        // A write completed when it was sent: no answer. A read gets zeros.
        (t.rx_len > 0).then(|| BusReply {
            correlation_id: request.correlation_id,
            op: BusOp::Spi as i32,
            data: vec![0; t.rx_len as usize],
            ..Default::default()
        })
    }

    fn state(&self) -> Option<State> {
        let mut s = State::new().set("on", self.on() as u8).set("frames", self.frames);
        if let Some((first, last)) = self.dirty.take() {
            let mut bytes = Vec::with_capacity((last - first + 1) * WIDTH * 2);
            for i in first * WIDTH..(last + 1) * WIDTH {
                let raw = self.fb.get(i).copied().unwrap_or(0);
                bytes.extend_from_slice(&self.shown(raw).to_le_bytes());
            }
            s = s.set("rows", format!("{first},{last}")).set("pixels", base64(&bytes));
        }
        Some(s)
    }
}

// A whole frame is 150 KB, about 200 KB as base64 in the state; SPI
// transfers are up to 32 KB.
esp_part::export_part!(Ili9342, buffer = 262144);

#[cfg(test)]
mod tests {
    use super::*;

    const DC_PIN: u32 = 35;

    fn part() -> Ili9342 {
        let mut p = Ili9342::default();
        let pins = [("CS", 3), ("DC", DC_PIN)].map(|(n, p)| (n.to_string(), p)).into();
        p.configure(&PartConfig { pins, ..Default::default() });
        p
    }

    fn send(p: &mut Ili9342, dc: bool, tx: &[u8]) {
        let t = SpiTransfer { tx: tx.to_vec(), pin_levels: (dc as u64) << DC_PIN, ..Default::default() };
        let req = BusRequest { correlation_id: 1, op: Some(bus_request::Op::Spi(t)), ..Default::default() };
        assert_eq!(p.on_bus(&req), None, "a write gets no answer");
    }

    fn cmd(p: &mut Ili9342, c: u8, args: &[u8]) {
        send(p, false, &[c]);
        if !args.is_empty() {
            send(p, true, args);
        }
    }

    /// As M5GFX and tr_lcd leave the CoreS3 panel: awake, on, INVON, BGR.
    fn ready() -> Ili9342 {
        let mut p = part();
        cmd(&mut p, 0x11, &[]);
        cmd(&mut p, 0x36, &[MADCTL_BGR]);
        cmd(&mut p, 0x3A, &[0x55]);
        cmd(&mut p, 0x21, &[]);
        cmd(&mut p, 0x29, &[]);
        p.state(); // the page has read the first frame
        p
    }

    fn window(p: &mut Ili9342, x0: u16, x1: u16, y0: u16, y1: u16) {
        let [a, b] = x0.to_be_bytes();
        let [c, d] = x1.to_be_bytes();
        cmd(p, 0x2A, &[a, b, c, d]);
        let [a, b] = y0.to_be_bytes();
        let [c, d] = y1.to_be_bytes();
        cmd(p, 0x2B, &[a, b, c, d]);
    }

    fn decode(b64: &str) -> Vec<u16> {
        let val = |c: u8| b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/".iter().position(|&x| x == c);
        let mut bytes = Vec::new();
        for q in b64.as_bytes().chunks(4) {
            let n = q.iter().fold(0u32, |n, &c| n << 6 | val(c).unwrap_or(0) as u32);
            let pad = q.iter().filter(|&&c| c == b'=').count();
            bytes.extend_from_slice(&n.to_be_bytes()[1..4 - pad]);
        }
        bytes.chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect()
    }

    #[test]
    fn claims_its_chip_select() {
        let spi = SpiClaim { cs_pin: Some(3), ..Default::default() };
        assert_eq!(part().claims(), vec![Claim { what: Some(claim::What::Spi(spi)) }]);
    }

    #[test]
    fn dark_until_awake_and_on() {
        let mut p = part();
        window(&mut p, 0, 0, 0, 0);
        cmd(&mut p, 0x2C, &[0xFF, 0xFF]);
        assert_eq!(p.state().unwrap().get("on"), Some("0"));
        cmd(&mut p, 0x11, &[]);
        assert_eq!(p.state().unwrap().get("on"), Some("0"), "awake but display off");
        cmd(&mut p, 0x29, &[]);
        assert_eq!(p.state().unwrap().get("on"), Some("1"));
    }

    #[test]
    fn a_window_of_pixels_lands_and_only_those_rows_are_sent() {
        let mut p = ready();
        window(&mut p, 10, 11, 100, 101);
        // Red, green, blue, white, RGB565 big-endian on the wire.
        cmd(&mut p, 0x2C, &[0xF8, 0x00, 0x07, 0xE0, 0x00, 0x1F, 0xFF, 0xFF]);
        let s = p.state().unwrap();
        assert_eq!(s.get("rows"), Some("100,101"));
        assert_eq!(s.get("frames"), Some("1"));
        let px = decode(s.get("pixels").unwrap());
        assert_eq!(px.len(), 2 * WIDTH);
        assert_eq!(&px[10..12], &[0xF800, 0x07E0], "INVON + BGR: colors as sent");
        assert_eq!(&px[WIDTH + 10..WIDTH + 12], &[0x001F, 0xFFFF]);
        assert_eq!(p.state().unwrap().get("rows"), None, "nothing new");
    }

    #[test]
    fn without_invon_the_panel_inverts_and_without_bgr_red_and_blue_swap() {
        let mut p = ready();
        window(&mut p, 0, 0, 0, 0);
        cmd(&mut p, 0x2C, &[0xF8, 0x00]); // red
        cmd(&mut p, 0x20, &[]);
        assert_eq!(decode(p.state().unwrap().get("pixels").unwrap())[0], 0x07FF, "inverted red");
        cmd(&mut p, 0x21, &[]);
        cmd(&mut p, 0x36, &[0x00]);
        assert_eq!(decode(p.state().unwrap().get("pixels").unwrap())[0], 0x001F, "red shows as blue");
    }

    #[test]
    fn pixels_split_across_transfers_and_ramwrc_carries_on() {
        let mut p = ready();
        window(&mut p, 0, 2, 0, 0);
        cmd(&mut p, 0x2C, &[0x12]);
        send(&mut p, true, &[0x34]); // one pixel split over two transfers
        cmd(&mut p, 0x3C, &[0x56, 0x78, 0x9A]); // a new command drops a half pixel (0x9A)
        cmd(&mut p, 0x3C, &[0xBC, 0xDE]);
        let px = decode(p.state().unwrap().get("pixels").unwrap());
        assert_eq!(&px[0..3], &[0x1234, 0x5678, 0xBCDE]);
    }

    #[test]
    fn the_write_wraps_inside_the_window() {
        let mut p = ready();
        window(&mut p, 5, 6, 7, 7);
        cmd(&mut p, 0x2C, &[0x00, 0x01, 0x00, 0x02, 0x00, 0x03]);
        let px = decode(p.state().unwrap().get("pixels").unwrap());
        assert_eq!(&px[5..7], &[0x0003, 0x0002], "third pixel back at the corner");
    }

    #[test]
    fn madctl_mirrors_and_exchanges() {
        let mut p = ready();
        cmd(&mut p, 0x36, &[MADCTL_BGR | MADCTL_MX | MADCTL_MY]);
        window(&mut p, 0, 0, 0, 0);
        cmd(&mut p, 0x2C, &[0x12, 0x34]);
        let s = p.state().unwrap();
        assert_eq!(s.get("rows"), Some(&*format!("{},{}", HEIGHT - 1, HEIGHT - 1)));
        assert_eq!(decode(s.get("pixels").unwrap())[WIDTH - 1], 0x1234, "bottom right");
        cmd(&mut p, 0x36, &[MADCTL_BGR | MADCTL_MV]);
        window(&mut p, 3, 3, 9, 9); // column 3, row 9 exchanged: memory column 9, row 3
        cmd(&mut p, 0x2C, &[0x56, 0x78]);
        let s = p.state().unwrap();
        assert_eq!(s.get("rows"), Some("3,3"));
        assert_eq!(decode(s.get("pixels").unwrap())[9], 0x5678);
    }

    #[test]
    fn eighteen_bit_pixels() {
        let mut p = ready();
        cmd(&mut p, 0x3A, &[0x66]);
        window(&mut p, 0, 0, 0, 0);
        cmd(&mut p, 0x2C, &[0xFC, 0x00, 0xFC]); // magenta
        assert_eq!(decode(p.state().unwrap().get("pixels").unwrap())[0], 0xF81F);
    }

    #[test]
    fn swreset_clears_everything() {
        let mut p = ready();
        window(&mut p, 0, 0, 0, 0);
        cmd(&mut p, 0x2C, &[0xFF, 0xFF]);
        cmd(&mut p, 0x01, &[]);
        let s = p.state().unwrap();
        assert_eq!(s.get("on"), Some("0"));
        assert_eq!(s.get("rows"), Some(&*format!("0,{}", HEIGHT - 1)));
    }

    #[test]
    fn the_command_phase_counts_as_a_command() {
        let mut p = ready();
        window(&mut p, 0, 0, 0, 0);
        let cmd_phase = esp_part::proto::SpiPhase { bits: 8, lines: 1, data: vec![0x2C] };
        let t = SpiTransfer { cmd: Some(cmd_phase), tx: vec![0x43, 0x21], pin_levels: 1 << DC_PIN, ..Default::default() };
        p.on_bus(&BusRequest { correlation_id: 2, op: Some(bus_request::Op::Spi(t)), ..Default::default() });
        assert_eq!(decode(p.state().unwrap().get("pixels").unwrap())[0], 0x4321);
    }

    #[test]
    fn reads_get_zeros() {
        let mut p = ready();
        let t = SpiTransfer { tx: vec![0x04], rx_len: 3, ..Default::default() };
        let r = p.on_bus(&BusRequest { correlation_id: 9, op: Some(bus_request::Op::Spi(t)), ..Default::default() });
        let r = r.unwrap();
        assert_eq!((r.correlation_id, r.data), (9, vec![0, 0, 0]));
    }

    #[test]
    fn nothing_big_is_allocated_before_the_first_pixel() {
        let mut p = ready();
        assert!(p.fb.is_empty());
        assert_eq!(decode(p.state().unwrap().get("pixels").unwrap_or("")).len(), 0, "nothing new to send");
        window(&mut p, 0, 0, 0, 0);
        cmd(&mut p, 0x2C, &[0x00, 0x01]);
        assert_eq!(p.fb.len(), WIDTH * HEIGHT);
    }

    #[test]
    fn base64_matches_the_standard() {
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
    }
}
