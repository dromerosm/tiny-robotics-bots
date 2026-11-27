//! The BALA2 motor base: an STM32F030 at I2C 0x3A. M5Stack publishes no
//! register map; this one comes from M5Stack's own drivers (Arduino M5Bala2
//! `src/bala.cpp` and UiFlow2 `bala2.py`). Multi-byte values are big-endian
//! and the register pointer moves on after each byte:
//!
//! | Registers | Access | Content |
//! | --- | --- | --- |
//! | 0x00-0x03 | W | motor 1 and motor 2 speed, int16 each, -1023..1023 |
//! | 0x10-0x17 | R/W | encoder 1 and encoder 2, int32 each; a write sets them |
//! | 0x20-0x27 | W | servo 1..8 angle, 0..180 (500..2500 us) |
//! | 0x30-0x3F | W | servo 1..8 pulse, uint16 us, 500..2500, at 50 Hz |
//!
//! Other registers read back what was written; the real behavior is not
//! documented. The motors and encoders live in [`Physics`].

use crate::physics::Physics;

const SPEED: usize = 0x00;
const ENCODER: usize = 0x10;
const ANGLE: usize = 0x20;
const PULSE: usize = 0x30;

pub struct Base {
    regs: [u8; 256],
    at: u8,
    pub servo_us: [u16; 8],
}

impl Default for Base {
    fn default() -> Self {
        Self { regs: [0; 256], at: 0, servo_us: [1500; 8] }
    }
}

fn get32(b: &[u8]) -> i32 {
    i32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

impl Base {
    /// One I2C transaction. A write takes effect at its end, as the base
    /// reads a whole command at once.
    pub fn transfer(&mut self, write: &[u8], read_len: usize, phys: &mut Physics) -> Vec<u8> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg;
            let (lo, hi) = (reg as usize, reg as usize + data.len());
            for &b in data {
                self.regs[self.at as usize] = b;
                self.at = self.at.wrapping_add(1);
            }
            if !data.is_empty() {
                self.apply(lo, hi, phys);
            }
        }
        if read_len > 0 {
            // The encoders as they are now.
            let e0 = (phys.enc[0].round() as i64) as i32;
            let e1 = (phys.enc[1].round() as i64) as i32;
            self.regs[ENCODER..ENCODER + 4].copy_from_slice(&e0.to_be_bytes());
            self.regs[ENCODER + 4..ENCODER + 8].copy_from_slice(&e1.to_be_bytes());
        }
        (0..read_len)
            .map(|_| {
                let v = self.regs[self.at as usize];
                self.at = self.at.wrapping_add(1);
                v
            })
            .collect()
    }

    /// Bytes `lo..hi` were written (`hi` excluded).
    fn apply(&mut self, lo: usize, hi: usize, phys: &mut Physics) {
        let touched = |a: usize, b: usize| lo <= b && hi > a; // registers a..=b
        if touched(SPEED, SPEED + 3) {
            for w in 0..2 {
                let v = i16::from_be_bytes([self.regs[SPEED + 2 * w], self.regs[SPEED + 2 * w + 1]]);
                phys.u[w] = v.clamp(-1023, 1023);
            }
        }
        if touched(ENCODER, ENCODER + 7) {
            phys.enc[0] = get32(&self.regs[ENCODER..]) as f64;
            phys.enc[1] = get32(&self.regs[ENCODER + 4..]) as f64;
        }
        for ch in 0..8 {
            if touched(ANGLE + ch, ANGLE + ch) {
                let angle = self.regs[ANGLE + ch].min(180) as u32;
                self.servo_us[ch] = (500 + angle * 2000 / 180) as u16;
            }
            if touched(PULSE + 2 * ch, PULSE + 2 * ch + 1) {
                let us = u16::from_be_bytes([self.regs[PULSE + 2 * ch], self.regs[PULSE + 2 * ch + 1]]);
                self.servo_us[ch] = us.clamp(500, 2500);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speeds_as_bala_cpp_sends_them() {
        let (mut b, mut p) = (Base::default(), Physics::default());
        b.transfer(&[0x00, 0x01, 0xF4, 0xFE, 0x0C], 0, &mut p); // 500, -500
        assert_eq!(p.u, [500, -500]);
        b.transfer(&[0x00, 0x7F, 0xFF, 0x80, 0x00], 0, &mut p);
        assert_eq!(p.u, [1023, -1023], "clamped");
    }

    #[test]
    fn encoders_read_back_and_a_write_sets_them() {
        let (mut b, mut p) = (Base::default(), Physics::default());
        p.enc = [-1234.0, 56.0];
        let r = b.transfer(&[0x10], 8, &mut p);
        assert_eq!((get32(&r[0..]), get32(&r[4..])), (-1234, 56));
        b.transfer(&[0x10, 0, 0, 0, 0, 0, 0, 0, 0], 0, &mut p);
        assert_eq!(p.enc, [0.0, 0.0]);
    }

    #[test]
    fn servo_pulses_and_angles() {
        let (mut b, mut p) = (Base::default(), Physics::default());
        b.transfer(&[0x36, 0x06, 0xA4], 0, &mut p); // channel 4: 1700 us
        assert_eq!(b.servo_us[3], 1700);
        b.transfer(&[0x20, 90], 0, &mut p);
        assert_eq!(b.servo_us[0], 1500);
    }
}
