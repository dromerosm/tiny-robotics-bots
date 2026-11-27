//! AW9523B IO expander, address 0x58.
//!
//! - 0x00/0x01 read the pins of port 0/1: an output reads its level, an input
//!   reads the line (high from the board's pull-ups unless something drives
//!   it). Reading one also clears that port's interrupt.
//! - 0x02/0x03 output levels, 0x04/0x05 directions (1 = input, all inputs
//!   after reset), 0x06/0x07 interrupt enable (0 = enabled, the reset value),
//!   0x10 the ID (0x23), 0x7F software reset (write 0x00).
//! - INTN goes low when an enabled input changes from the value last read,
//!   and back high when that port is read. On the CoreS3 it is GPIO 21
//!   (I2C_INT), and the touch chip's INT reaches it on P1_2, so M5GFX reads
//!   the touch only while GPIO 21 is low.
//!
//! Pins on the CoreS3 (from M5Unified, M5GFX and tr_board): P0_0 touch reset,
//! P0_1 BUS_OUT_EN, P0_2 speaker enable, P1_0 camera enable, P1_1 LCD reset,
//! P1_2 touch INT (input), P1_7 BOOST_EN.

use crate::chip::{Chip, Regs8};
use esp_part::State;

const IN0: u8 = 0x00;
const OUT0: u8 = 0x02;
const DIR0: u8 = 0x04;
const INT0: u8 = 0x06;
const ID: u8 = 0x10;
const RESET: u8 = 0x7F;

pub const BUS_OUT_EN: (usize, u8) = (0, 0x02); // P0_1
pub const SPEAKER_EN: (usize, u8) = (0, 0x04); // P0_2
pub const CAMERA_EN: (usize, u8) = (1, 0x01); // P1_0
pub const LCD_RESET: (usize, u8) = (1, 0x02); // P1_1, low = in reset
pub const TOUCH_INT: (usize, u8) = (1, 0x04); // P1_2, low = touched
pub const BOOST_EN: (usize, u8) = (1, 0x80); // P1_7

pub struct Aw9523 {
    regs: Regs8,
    /// What drives each input line from outside (1 = high), by port.
    line: [u8; 2],
    /// The input values the firmware last read, and whether an enabled
    /// input changed since.
    seen: [u8; 2],
    pending: [bool; 2],
}

fn reset_regs() -> Regs8 {
    // After reset every pin is an input, and every interrupt is enabled.
    Regs8::new(&[(ID, 0x23), (DIR0, 0xFF), (DIR0 + 1, 0xFF)], &[IN0, IN0 + 1, ID])
}

impl Default for Aw9523 {
    fn default() -> Self {
        Self { regs: reset_regs(), line: [0xFF; 2], seen: [0xFF; 2], pending: [false; 2] }
    }
}

impl Aw9523 {
    fn levels(&self, port: usize) -> u8 {
        let m = &self.regs.mem;
        let dir = m[(DIR0 as usize) + port];
        (m[(OUT0 as usize) + port] & !dir) | (self.line[port] & dir)
    }

    /// Mark a port's interrupt if an enabled input no longer reads as the
    /// firmware last saw it.
    fn check(&mut self, port: usize) {
        let m = &self.regs.mem;
        let watched = m[(DIR0 as usize) + port] & !m[(INT0 as usize) + port];
        if (self.levels(port) ^ self.seen[port]) & watched != 0 {
            self.pending[port] = true;
        }
    }

    /// Something outside drives an input line.
    pub fn set_line(&mut self, (port, bit): (usize, u8), high: bool) {
        if high {
            self.line[port] |= bit;
        } else {
            self.line[port] &= !bit;
        }
        self.check(port);
    }

    /// A pin the chip drives as an output and holds high.
    pub fn output_high(&self, (port, bit): (usize, u8)) -> bool {
        let m = &self.regs.mem;
        m[(DIR0 as usize) + port] & bit == 0 && m[(OUT0 as usize) + port] & bit != 0
    }

    /// The INTN line: low while a port has an interrupt.
    pub fn intn_high(&self) -> bool {
        !(self.pending[0] || self.pending[1])
    }
}

impl Chip for Aw9523 {
    fn transfer(&mut self, write: &[u8], read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        if write == [RESET, 0x00] {
            *self = Self { line: self.line, ..Self::default() };
            return Some(Vec::new());
        }
        self.regs.write(write);
        self.check(0);
        self.check(1);
        let inputs = [self.levels(0), self.levels(1)];
        let mut cleared = [false; 2];
        let data = self.regs.read(read_len, |m, r| match r {
            IN0 | 0x01 => {
                let port = (r - IN0) as usize;
                cleared[port] = true;
                inputs[port]
            }
            _ => m[r as usize],
        });
        for port in 0..2 {
            if cleared[port] {
                self.seen[port] = inputs[port];
                self.pending[port] = false;
            }
        }
        Some(data)
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state
            .set("bus_out_en", self.output_high(BUS_OUT_EN) as u8)
            .set("boost_en", self.output_high(BOOST_EN) as u8)
            .set("speaker_en", self.output_high(SPEAKER_EN) as u8)
            .set("camera_en", self.output_high(CAMERA_EN) as u8)
            .set("lcd_reset", !self.output_high(LCD_RESET) as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(c: &mut Aw9523, write: &[u8], read_len: usize) -> Vec<u8> {
        c.transfer(write, read_len, false, 0).unwrap()
    }

    #[test]
    fn id_and_reset_state() {
        let mut c = Aw9523::default();
        assert_eq!(t(&mut c, &[ID], 1), [0x23]);
        assert_eq!(t(&mut c, &[DIR0], 2), [0xFF, 0xFF], "all inputs");
        assert_eq!(t(&mut c, &[INT0], 2), [0x00, 0x00], "all interrupts enabled");
        assert_eq!(t(&mut c, &[IN0], 2), [0xFF, 0xFF], "inputs read high");
    }

    #[test]
    fn outputs_read_their_level_and_inputs_cannot_be_written() {
        let mut c = Aw9523::default();
        t(&mut c, &[DIR0, 0xFE], 0); // P0_0 output, low
        assert_eq!(t(&mut c, &[IN0], 1), [0xFE]);
        t(&mut c, &[IN0, 0x00], 0);
        assert_eq!(t(&mut c, &[IN0], 1), [0xFE]);
    }

    #[test]
    fn touch_int_pulls_intn_low_until_port_1_is_read() {
        let mut c = Aw9523::default();
        t(&mut c, &[IN0], 2); // the firmware reads both ports once
        assert!(c.intn_high());
        c.set_line(TOUCH_INT, false);
        assert!(!c.intn_high(), "a touch raises the interrupt");
        assert_eq!(t(&mut c, &[IN0], 1), [0xFF], "port 0 alone does not clear it");
        assert!(!c.intn_high());
        assert_eq!(t(&mut c, &[IN0 + 1], 1), [0xFB]);
        assert!(c.intn_high(), "reading port 1 clears it");
        c.set_line(TOUCH_INT, true);
        assert!(!c.intn_high(), "the release is a change too");
    }

    #[test]
    fn a_disabled_interrupt_or_an_output_does_not_fire() {
        let mut c = Aw9523::default();
        t(&mut c, &[INT0 + 1, 0x04], 0); // P1_2 interrupt off
        c.set_line(TOUCH_INT, false);
        assert!(c.intn_high());
        t(&mut c, &[INT0 + 1, 0x00], 0);
        assert!(!c.intn_high(), "enabled again: P1_2 differs from the last read");
        t(&mut c, &[IN0 + 1], 1);
        assert!(c.intn_high());
        t(&mut c, &[DIR0 + 1, 0xFB], 0); // P1_2 as an output now
        c.set_line(TOUCH_INT, true);
        assert!(c.intn_high(), "an output does not fire");
    }

    #[test]
    fn software_reset_restores_the_registers() {
        let mut c = Aw9523::default();
        t(&mut c, &[DIR0, 0x00], 0);
        t(&mut c, &[RESET, 0x00], 0);
        assert_eq!(t(&mut c, &[DIR0], 1), [0xFF]);
    }
}
