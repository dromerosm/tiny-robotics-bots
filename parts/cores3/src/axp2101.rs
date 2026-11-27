//! AXP2101 power chip, address 0x34.
//!
//! What the drivers read (tr_board, M5Unified, M5GFX, StackChan):
//! - 0x00 status: bit 5 USB power present, bit 3 battery present.
//! - 0x01 status 2: bits 6:5 charge state (0 standby).
//! - 0x03 chip ID, 0x4A. M5GFX needs it to call the board a CoreS3.
//! - 0x34, 0x38, 0x3A: battery, USB and system voltage, 14 bits in mV,
//!   high byte first (as tr_board reads them).
//! - 0xA4 fuel gauge, in percent.
//! - 0x90 LDO enable: bit 7 DLDO1 (the screen's backlight), bit 2 ALDO3,
//!   bits 4/5 BLDO1/BLDO2 (camera). 0x99 is DLDO1's voltage, (mV - 500) / 100.
//!
//! Every other register stores what is written, so the drivers' read,
//! change and write steps work. It is not a model of the chip's electronics:
//! no charging, and the readings stay where they are set.

use crate::chip::{Chip, Regs8};
use esp_part::State;

const STATUS: u8 = 0x00;
const CHIP_ID: u8 = 0x03;
const VBAT: u8 = 0x34;
const VBUS: u8 = 0x38;
const VSYS: u8 = 0x3A;
const LDO_EN: u8 = 0x90;
const DLDO1_V: u8 = 0x99;
const GAUGE: u8 = 0xA4;

pub struct Axp2101 {
    regs: Regs8,
}

impl Default for Axp2101 {
    fn default() -> Self {
        let mv = |v: u16| [(v >> 8) as u8, v as u8];
        let (vbat, vbus, vsys) = (mv(4000), mv(5000), mv(5000));
        Self {
            regs: Regs8::new(
                &[
                    (STATUS, 0x28), // USB power and a battery
                    (CHIP_ID, 0x4A),
                    (VBAT, vbat[0]),
                    (VBAT + 1, vbat[1]),
                    (VBUS, vbus[0]),
                    (VBUS + 1, vbus[1]),
                    (VSYS, vsys[0]),
                    (VSYS + 1, vsys[1]),
                    (GAUGE, 80),
                ],
                &[STATUS, 0x01, CHIP_ID, VBAT, VBAT + 1, VBUS, VBUS + 1, VSYS, VSYS + 1, GAUGE],
            ),
        }
    }
}

impl Axp2101 {
    /// The backlight: DLDO1 on, and its voltage in mV.
    pub fn backlight_mv(&self) -> Option<u32> {
        let m = &self.regs.mem;
        (m[LDO_EN as usize] & 0x80 != 0).then(|| 500 + 100 * (m[DLDO1_V as usize] & 0x1F) as u32)
    }

    /// ALDO3, which (with AW9523 P1_0) powers the camera and the light sensor.
    pub fn aldo3_on(&self) -> bool {
        self.regs.mem[LDO_EN as usize] & 0x04 != 0
    }
}

impl Chip for Axp2101 {
    fn transfer(&mut self, write: &[u8], read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        self.regs.write(write);
        Some(self.regs.read(read_len, |m, r| m[r as usize]))
    }

    fn input(&mut self, name: &str, value: f64) -> bool {
        if name != "battery" {
            return false;
        }
        self.regs.mem[GAUGE as usize] = value.clamp(0.0, 100.0) as u8;
        true
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state.set("backlight_mv", self.backlight_mv().unwrap_or(0)).set("battery", self.regs.mem[GAUGE as usize])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(c: &mut Axp2101, write: &[u8], read_len: usize) -> Vec<u8> {
        c.transfer(write, read_len, false, 0).unwrap()
    }

    #[test]
    fn ids_and_readings() {
        let mut c = Axp2101::default();
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x4A]);
        assert_eq!(t(&mut c, &[STATUS], 2), [0x28, 0x00], "USB, battery, standby");
        assert_eq!(t(&mut c, &[VBAT], 2), [0x0F, 0xA0], "4000 mV");
        assert_eq!(t(&mut c, &[GAUGE], 1), [80]);
        t(&mut c, &[CHIP_ID, 0x00], 0);
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x4A], "read only");
    }

    #[test]
    fn backlight_follows_dldo1() {
        let mut c = Axp2101::default();
        assert_eq!(c.backlight_mv(), None);
        t(&mut c, &[DLDO1_V, ((3300u32 - 500) / 100) as u8], 0);
        let en = t(&mut c, &[LDO_EN], 1)[0];
        t(&mut c, &[LDO_EN, en | 0x80], 0);
        assert_eq!(c.backlight_mv(), Some(3300));
    }

    #[test]
    fn battery_can_be_set_from_the_page() {
        let mut c = Axp2101::default();
        assert!(c.input("battery", 15.0));
        assert_eq!(t(&mut c, &[GAUGE], 1), [15]);
        assert!(!c.input("touch_x", 1.0));
    }
}
