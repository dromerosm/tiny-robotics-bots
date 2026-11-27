//! LTR-553ALS light and proximity sensor, address 0x23. Only tr_board drives
//! it: M5Unified and the StackChan firmware have no driver for it.
//!
//! - 0x80 ALS_CONTR and 0x81 PS_CONTR: read and write, 0x00 (standby) at
//!   reset. Light is on when ALS_CONTR bit 0 is set, proximity when PS_CONTR
//!   bit 1 is set (both as in Linux's ltr501 driver for the LTR-559, the
//!   LTR-553's sibling). ALS_CONTR bits 4:2 are the gain: 1, 2, 4, 8, -, -,
//!   48, 96 (ESPHome's ltr_als_ps table). ALS_CONTR bit 1 is a software reset:
//!   all registers go back to their reset values and the bit reads 0 again
//!   (datasheet, unconfirmed: we have no LTR-553 register table).
//! - 0x88-0x8B light data: CH1 low, CH1 high, CH0 low, CH0 high (CH1 first,
//!   as Linux and ESPHome read it). 0x8D-0x8E proximity, 11 bits, low byte
//!   first. All zero while the sensor is in standby. The 0x8E saturation bit
//!   (bit 7) stays 0.
//! - 0x8C ALS_PS_STATUS: the gain in bits 6:4, "new light data" in bit 2 and
//!   "new proximity data" in bit 0 while each one is on (datasheet,
//!   unconfirmed). The data are there at once: no wait for a first
//!   measurement.
//! - 0x86 PART_ID 0x92 and 0x87 MANUFAC_ID 0x05, read only. ESPHome's
//!   ltr_als_ps driver lists part number 0x9, revision 0x2 for the LTR-553
//!   and checks manufacturer 0x05 (Lite-On); a probe log in nanoFramework's
//!   Ltr553AlsWa driver shows 0x92 and 0x05.
//! - Every other register stores what is written. Reset values, all
//!   datasheet, unconfirmed: 0x82 0x7F, 0x83 0x01, 0x84 0x02, 0x85 0x03,
//!   0x8F 0x08, 0x90 0xFF, 0x91 0x07, 0x97 0xFF, 0x98 0xFF.
//! - The register pointer moves on after each byte.
//!
//! Light from lux: CH0 = 1.5 x lux and CH1 = 0.5 x lux, times the gain and
//! times the integration time / 100 ms (0x85 bits 5:3: 100, 50, 200, 400,
//! 150, 250, 300, 350 ms; datasheet, unconfirmed), capped at 65535. So at
//! gain 1x and 100 ms tr_board's `(ch0 + ch1) / 2` prints the lux value. It
//! is a simple mapping for tr_board, not Lite-On's lux formula: that formula
//! would read about 3.2 times the value.
//!
//! tr_board (tr_sensors.c:102-107, :294-298) writes 0x80←01 (light on, gain
//! 1x) and 0x81←02 (proximity on), then reads 4 bytes from 0x88 and 2 bytes
//! from 0x8D. It says the real chip only answers with the camera powered
//! (AXP2101 ALDO3 and AW9523 P1_0). Here it always answers, so a firmware
//! that forgets the camera power still sees the sensor.
//!
//! Inputs `lux` (0 or more) and `proximity` (0..2047). State: `lux`,
//! `proximity`, the values the page set.

use crate::chip::{Chip, Regs8};
use esp_part::State;

const ALS_CONTR: u8 = 0x80;
const PS_CONTR: u8 = 0x81;
const ALS_MEAS_RATE: u8 = 0x85;
const PART_ID: u8 = 0x86;
const MANUFAC_ID: u8 = 0x87;
const ALS_DATA: u8 = 0x88; // to 0x8B
const STATUS: u8 = 0x8C;
const PS_DATA: u8 = 0x8D; // and 0x8E

const SW_RESET: u8 = 0x02;
const GAIN: [u32; 8] = [1, 2, 4, 8, 1, 1, 48, 96]; // 4 and 5 are reserved: taken as 1x
const INTEGRATION_MS: [u32; 8] = [100, 50, 200, 400, 150, 250, 300, 350];

pub struct Ltr553 {
    regs: Regs8,
    lux: f64,
    proximity: u16,
}

fn reset_regs() -> Regs8 {
    Regs8::new(
        &[
            (0x82, 0x7F),
            (0x83, 0x01),
            (0x84, 0x02),
            (ALS_MEAS_RATE, 0x03),
            (PART_ID, 0x92),
            (MANUFAC_ID, 0x05),
            (0x8F, 0x08),
            (0x90, 0xFF),
            (0x91, 0x07),
            (0x97, 0xFF),
            (0x98, 0xFF),
        ],
        &[PART_ID, MANUFAC_ID, 0x88, 0x89, 0x8A, 0x8B, STATUS, 0x8D, 0x8E],
    )
}

impl Default for Ltr553 {
    fn default() -> Self {
        Self { regs: reset_regs(), lux: 0.0, proximity: 0 }
    }
}

impl Ltr553 {
    fn als_on(&self) -> bool {
        self.regs.mem[ALS_CONTR as usize] & 0x01 != 0
    }

    fn ps_on(&self) -> bool {
        self.regs.mem[PS_CONTR as usize] & 0x02 != 0
    }

    /// CH1 and CH0 as the chip would count them now.
    fn channels(&self) -> (u16, u16) {
        if !self.als_on() {
            return (0, 0);
        }
        let m = &self.regs.mem;
        let gain = GAIN[(m[ALS_CONTR as usize] >> 2 & 7) as usize] as f64;
        let time = INTEGRATION_MS[(m[ALS_MEAS_RATE as usize] >> 3 & 7) as usize] as f64 / 100.0;
        let count = |share: f64| (self.lux * share * gain * time).round().clamp(0.0, 65535.0) as u16;
        (count(0.5), count(1.5))
    }
}

impl Chip for Ltr553 {
    fn transfer(&mut self, write: &[u8], read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        self.regs.write(write);
        if self.regs.mem[ALS_CONTR as usize] & SW_RESET != 0 {
            let at = self.regs.at;
            self.regs = reset_regs();
            self.regs.at = at;
        }
        let (ch1, ch0) = self.channels();
        let ps = if self.ps_on() { self.proximity } else { 0 };
        let status = (self.regs.mem[ALS_CONTR as usize] & 0x1C) << 2
            | if self.als_on() { 0x04 } else { 0 }
            | if self.ps_on() { 0x01 } else { 0 };
        Some(self.regs.read(read_len, |m, r| match r {
            ALS_DATA => ch1 as u8,
            0x89 => (ch1 >> 8) as u8,
            0x8A => ch0 as u8,
            0x8B => (ch0 >> 8) as u8,
            STATUS => status,
            PS_DATA => ps as u8,
            0x8E => (ps >> 8) as u8 & 0x07,
            _ => m[r as usize],
        }))
    }

    fn input(&mut self, name: &str, value: f64) -> bool {
        match name {
            "lux" => self.lux = value.max(0.0),
            "proximity" => self.proximity = value.clamp(0.0, 2047.0) as u16,
            _ => return false,
        }
        true
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state.set("lux", self.lux).set("proximity", self.proximity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(c: &mut Ltr553, write: &[u8], read_len: usize) -> Vec<u8> {
        c.transfer(write, read_len, false, 0).unwrap()
    }

    /// tr_board's status line (tr_sensors.c:294-298): light and proximity.
    fn tr_board_read(c: &mut Ltr553) -> (i32, i32) {
        let b = t(c, &[ALS_DATA], 4);
        let p = t(c, &[PS_DATA], 2);
        let (ch1, ch0) = (b[0] as i32 | (b[1] as i32) << 8, b[2] as i32 | (b[3] as i32) << 8);
        ((ch0 + ch1) / 2, p[0] as i32 | (p[1] as i32 & 0x07) << 8)
    }

    #[test]
    fn ids_and_standby_at_reset() {
        let mut c = Ltr553::default();
        assert_eq!(t(&mut c, &[PART_ID], 2), [0x92, 0x05]);
        assert_eq!(t(&mut c, &[ALS_CONTR], 2), [0x00, 0x00], "standby");
        c.input("lux", 300.0);
        c.input("proximity", 900.0);
        assert_eq!(t(&mut c, &[ALS_DATA], 7), [0; 7], "no data in standby");
        t(&mut c, &[PART_ID, 0x00], 0);
        assert_eq!(t(&mut c, &[PART_ID], 1), [0x92], "read only");
    }

    #[test]
    fn tr_board_setup_and_reads() {
        let mut c = Ltr553::default();
        c.input("lux", 321.0);
        c.input("proximity", 1500.0);
        // light_init(): 0x80←01, 0x81←02.
        t(&mut c, &[ALS_CONTR, 0x01], 0);
        t(&mut c, &[PS_CONTR, 0x02], 0);
        assert_eq!(t(&mut c, &[ALS_CONTR], 2), [0x01, 0x02]);
        assert_eq!(tr_board_read(&mut c), (321, 1500));
        assert_eq!(t(&mut c, &[ALS_DATA], 4), [0xA1, 0x00, 0xE2, 0x01], "CH1 161, CH0 482");
        let s = c.state(State::new(), 0);
        assert_eq!((s.get("lux"), s.get("proximity")), (Some("321"), Some("1500")));
    }

    #[test]
    fn gain_and_integration_time_scale_the_counts() {
        let mut c = Ltr553::default();
        c.input("lux", 100.0);
        t(&mut c, &[ALS_CONTR, 0x01 | 2 << 2], 0); // 4x
        assert_eq!(tr_board_read(&mut c).0, 400);
        t(&mut c, &[ALS_MEAS_RATE, 0x01 << 3 | 0x03], 0); // 50 ms
        assert_eq!(tr_board_read(&mut c).0, 200);
        c.input("lux", 1000.0);
        t(&mut c, &[ALS_CONTR, 0x01 | 7 << 2], 0); // 96x: CH0 saturates
        assert_eq!(t(&mut c, &[0x8A], 2), [0xFF, 0xFF]);
        assert_eq!(t(&mut c, &[STATUS], 1), [0x74], "gain 7, new light data");
    }

    #[test]
    fn proximity_is_11_bits_and_clamped() {
        let mut c = Ltr553::default();
        t(&mut c, &[PS_CONTR, 0x03], 0);
        c.input("proximity", 5000.0);
        assert_eq!(t(&mut c, &[PS_DATA], 2), [0xFF, 0x07]);
        c.input("proximity", -3.0);
        assert_eq!(t(&mut c, &[PS_DATA], 2), [0x00, 0x00]);
        assert_eq!(t(&mut c, &[STATUS], 1), [0x01]);
    }

    #[test]
    fn split_read_goes_on_from_the_pointer() {
        let mut c = Ltr553::default();
        c.input("lux", 1000.0); // CH1 500 = 0x01F4, CH0 1500 = 0x05DC
        t(&mut c, &[ALS_CONTR, 0x01], 0);
        assert_eq!(c.transfer(&[ALS_DATA], 2, false, 0), Some(vec![0xF4, 0x01]));
        assert_eq!(c.transfer(&[], 2, true, 1), Some(vec![0xDC, 0x05]));
    }

    #[test]
    fn software_reset_restores_standby() {
        let mut c = Ltr553::default();
        t(&mut c, &[ALS_CONTR, 0x01], 0);
        t(&mut c, &[PS_CONTR, 0x02], 0);
        t(&mut c, &[ALS_CONTR, SW_RESET], 0);
        assert_eq!(t(&mut c, &[ALS_CONTR], 2), [0x00, 0x00]);
        assert_eq!(t(&mut c, &[ALS_MEAS_RATE], 1), [0x03]);
    }

    #[test]
    fn probe_acks_and_inputs_belong_here() {
        let mut c = Ltr553::default();
        assert_eq!(c.transfer(&[], 0, false, 0), Some(vec![]));
        assert!(c.input("lux", 1.0) && c.input("proximity", 1.0));
        assert!(!c.input("time", 1.0));
    }
}
