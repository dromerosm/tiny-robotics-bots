//! A chain of WS2812 RGB LEDs on one data pin, as a virtual part for esp-emu.
//! The AtomS3 Lite has one, a WS2812C on GPIO 35.
//!
//! The part watches every edge of the pin (`GpioClaim.watch_pins`) and reads
//! each high pulse as one bit, as the LED does:
//!
//! - High for less than 0.6 us is a 0, and longer is a 1. The datasheet
//!   values are 0.4 and 0.8 us, with a margin of 0.15 us; ESP-IDF drivers often
//!   send 0.3 and 0.9 us. Both read right.
//! - A high pulse shorter than 0.1 us is noise: it is counted in `bad` and
//!   starts a new frame. One longer than 5 us is not a bit but an idle line
//!   (the pin floats high until the firmware drives it): it starts a new frame
//!   and is not counted.
//! - Low for 50 us or more is the reset: the next bit is the first bit of the
//!   first LED again. The datasheets ask for at least 50 us (older parts) to
//!   280 us (newer ones); drivers send more, so 50 us catches every one.
//!
//! Each LED takes 24 bits, green first, then red and blue, most significant bit
//! first. A real LED shows its color at the reset; this part shows it as soon
//! as its 24 bits are in, which looks the same on the page.
//!
//! The page reads `state()`: `leds` (colors as `rrggbb`, comma separated, in
//! chain order), `frames` (how many colors were set) and `bad` (noise pulses).

use esp_part::prelude::*;

const DIN: &str = "DIN";

const BIT_ONE_NS: u64 = 600; // longer high pulses are a 1
const NOISE_NS: u64 = 100; // shorter high pulses are ignored
const MAX_HIGH_NS: u64 = 5_000; // longer high pulses are not a bit
const RESET_NS: u64 = 50_000; // a low this long starts a new frame
const MAX_LEDS: usize = 64; // more than any M5Stack board has on one pin

#[derive(Default)]
struct Ws2812 {
    pin: Option<u32>,
    /// The level of the pin at the last edge, and when that edge came.
    high: bool,
    since_ns: u64,
    /// Bits of the frame so far.
    bits: usize,
    shift: u32,
    leds: Vec<u32>,
    frames: u64,
    bad: u64,
}

impl Ws2812 {
    fn edge(&mut self, high: bool, now_ns: u64) {
        if high == self.high {
            return;
        }
        let width = now_ns.saturating_sub(self.since_ns);
        self.high = high;
        self.since_ns = now_ns;
        if high {
            // The end of a low: a long one is the reset.
            if width >= RESET_NS {
                self.restart();
            }
            return;
        }
        // The end of a high pulse: one bit, noise, or an idle line.
        if width < NOISE_NS || width > MAX_HIGH_NS {
            self.bad += (width < NOISE_NS) as u64;
            self.restart();
            return;
        }
        self.bit(width >= BIT_ONE_NS);
    }

    fn restart(&mut self) {
        self.bits = 0;
        self.shift = 0;
    }

    fn bit(&mut self, one: bool) {
        self.shift = (self.shift << 1) | one as u32;
        self.bits += 1;
        if self.bits % 24 != 0 {
            return;
        }
        let index = self.bits / 24 - 1;
        if index >= MAX_LEDS {
            return;
        }
        let (g, r, b) = ((self.shift >> 16) & 0xFF, (self.shift >> 8) & 0xFF, self.shift & 0xFF);
        if self.leds.len() <= index {
            self.leds.resize(index + 1, 0);
        }
        self.leds[index] = (r << 16) | (g << 8) | b;
        self.shift = 0;
        self.frames += 1;
    }

    fn colors(&self) -> String {
        self.leds.iter().map(|c| format!("{c:06x}")).collect::<Vec<_>>().join(",")
    }
}

impl Device for Ws2812 {
    const NAME: &'static str = "ws2812";

    fn configure(&mut self, config: &PartConfig) {
        self.pin = config.pins.get(DIN).copied();
    }

    fn claims(&self) -> Vec<Claim> {
        // It only watches its pin: an LED drives nothing.
        self.pin
            .map(|pin| Claim { what: Some(claim::What::Gpio(GpioClaim { watch_pins: 1 << pin, ..Default::default() })) })
            .into_iter()
            .collect()
    }

    fn on_pins(&mut self, levels: u64, now_ns: u64) {
        if let Some(pin) = self.pin {
            self.edge(levels >> pin & 1 == 1, now_ns);
        }
    }

    fn state(&self) -> Option<State> {
        Some(State::new().set("leds", self.colors()).set("frames", self.frames).set("bad", self.bad))
    }
}

esp_part::export_part!(Ws2812, buffer = 1024);

#[cfg(test)]
mod tests {
    use super::*;

    const PIN: u32 = 35;

    fn part() -> Ws2812 {
        let mut p = Ws2812::default();
        let pins = [("DIN".to_string(), PIN)].into();
        p.configure(&PartConfig { pins, ..Default::default() });
        // The pin floats high until the firmware drives it, then goes low.
        p.on_pins(1 << PIN, 0);
        p.on_pins(0, 1_000_000);
        p
    }

    /// Send bytes as the firmware would, with high pulses of `t0`/`t1` ns in
    /// bit periods of 1.25 us, then a low of `reset` ns. Returns the end time.
    fn send(p: &mut Ws2812, mut t: u64, bytes: &[u8], t0: u64, t1: u64, reset: u64) -> u64 {
        t += reset;
        for &byte in bytes {
            for i in (0..8).rev() {
                let high = if byte >> i & 1 == 1 { t1 } else { t0 };
                p.on_pins(1 << PIN, t);
                p.on_pins(0, t + high);
                t += 1_250;
            }
        }
        t
    }

    #[test]
    fn claims_only_a_watch_on_its_pin() {
        let gpio = GpioClaim { watch_pins: 1 << PIN, ..Default::default() };
        assert_eq!(part().claims(), vec![Claim { what: Some(claim::What::Gpio(gpio)) }]);
        assert!(Ws2812::default().claims().is_empty(), "not wired: no claim");
    }

    #[test]
    fn reads_grb_with_the_atom_timing() {
        let mut p = part();
        // As atom.c: green, red, blue in pulses of 0.3 and 0.9 us.
        send(&mut p, 1_000_000, &[0x20, 0x00, 0x08], 300, 900, 300_000);
        let s = p.state().unwrap();
        assert_eq!(s.get("leds"), Some("002008"), "rrggbb");
        assert_eq!(s.get("frames"), Some("1"));
        assert_eq!(s.get("bad"), Some("0"));
    }

    #[test]
    fn reads_the_datasheet_timing_and_a_chain() {
        let mut p = part();
        let t = send(&mut p, 1_000_000, &[0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF], 400, 800, 60_000);
        assert_eq!(p.state().unwrap().get("leds"), Some("00ff00,0000ff"));
        // After a reset the first LED gets the next color.
        send(&mut p, t, &[0x00, 0x80, 0x00], 400, 800, 60_000);
        assert_eq!(p.state().unwrap().get("leds"), Some("800000,0000ff"));
    }

    #[test]
    fn a_short_gap_does_not_restart_the_chain() {
        let mut p = part();
        let t = send(&mut p, 1_000_000, &[0x01, 0x02, 0x03], 300, 900, 300_000);
        // Under 50 us of low: the next 24 bits go to the second LED.
        send(&mut p, t, &[0x04, 0x05, 0x06], 300, 900, 20_000);
        assert_eq!(p.state().unwrap().get("leds"), Some("020103,050406"));
    }

    #[test]
    fn noise_and_long_highs_are_not_bits() {
        let mut p = part();
        p.on_pins(1 << PIN, 2_000_000);
        p.on_pins(0, 2_000_050); // 50 ns: noise
        p.on_pins(1 << PIN, 2_100_000);
        p.on_pins(0, 2_200_000); // 100 us: an idle line, not counted
        assert_eq!(p.state().unwrap().get("bad"), Some("1"));
        assert_eq!(p.state().unwrap().get("leds"), Some(""));
        // A good frame after them still reads.
        send(&mut p, 2_200_000, &[0x00, 0xFF, 0x00], 300, 900, 300_000);
        assert_eq!(p.state().unwrap().get("leds"), Some("ff0000"));
    }

    #[test]
    fn a_reset_mid_frame_drops_the_partial_bits() {
        let mut p = part();
        // 16 bits, then the reset, then a full color.
        let t = send(&mut p, 1_000_000, &[0xAA, 0xA0], 300, 900, 300_000);
        send(&mut p, t, &[0x00, 0x00, 0x10], 300, 900, 300_000);
        assert_eq!(p.state().unwrap().get("leds"), Some("000010"));
    }
}
