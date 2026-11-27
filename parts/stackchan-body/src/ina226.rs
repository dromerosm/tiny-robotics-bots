//! The INA226 battery monitor in StackChan's body, at I2C address 0x41, with a
//! 10 mOhm shunt.
//!
//! Only tr_board uses it (`tr_board.c`): a probe, then 2 bytes from 0xFE that
//! must be 0x5449, then the config write `[00, 47, 27]`. Each battery read
//! takes 2 bytes from 0x02 (bus voltage, 1.25 mV per bit) and 2 bytes from 0x01
//! (shunt voltage, signed, 2.5 uV per bit; over 10 mOhm that is 0.25 mA per
//! bit, positive while the battery discharges). The official StackChan
//! firmware and M5Unified never touch it.
//!
//! The model, from TI's datasheet (SBOS547) where the code says nothing:
//!
//! - 16-bit registers, high byte first. The first written byte sets the
//!   register pointer, which stays until the next write, so a read with no
//!   write reads the last register pointed at. Two more bytes write the
//!   register. A write with only one data byte only moves the pointer.
//! - A read longer than 2 bytes goes on with the same register again (high,
//!   low, high...): the pointer does not move on (datasheet, unconfirmed). A
//!   split read (`continues`) goes on from the byte where it stopped.
//! - 0x00 config, R/W. Reset value 0x4127 (datasheet, unconfirmed: tr_board
//!   always writes 0x4727). Bit 15 resets every register to its reset value and
//!   reads back 0. The mode (bits 2:0): continuous modes follow the inputs,
//!   a triggered mode measures once when config is written, power-down holds
//!   the last values (datasheet, unconfirmed). Conversion and averaging time
//!   are not modelled: a value shows at once.
//! - 0x01 shunt and 0x02 bus, read-only, from the inputs `battery_ma` and
//!   `battery_mv`, with tr_board's scale (0.25 mA and 1.25 mV per bit).
//! - 0x03 power and 0x04 current, read-only, as the datasheet defines them:
//!   current = shunt x calibration / 2048, power = |current| x bus / 20000.
//!   Both read 0 until calibration is written (datasheet, unconfirmed: no
//!   firmware of ours reads them).
//! - 0x05 calibration, R/W, bit 15 reads 0 (datasheet, unconfirmed).
//! - 0x06 mask/enable and 0x07 alert limit, R/W storage. The alert pin and the
//!   flags are not modelled.
//! - 0xFE manufacturer ID 0x5449 (tr_board checks it). 0xFF die ID 0x2260
//!   (datasheet, unconfirmed).
//! - Any other register reads 0x0000 and ignores writes (unconfirmed).
//! - It always answers (ACK), a probe too: it runs from the battery.

pub const ADDR: u32 = 0x41;

pub const CONFIG: u8 = 0x00;
pub const SHUNT: u8 = 0x01;
pub const BUS: u8 = 0x02;
pub const POWER: u8 = 0x03;
pub const CURRENT: u8 = 0x04;
pub const CALIBRATION: u8 = 0x05;
pub const MASK: u8 = 0x06;
pub const ALERT: u8 = 0x07;
pub const MAKER: u8 = 0xFE;
pub const DIE: u8 = 0xFF;

/// tr_board's constant ("TI").
pub const MAKER_TI: u16 = 0x5449;
/// Datasheet, unconfirmed.
const DIE_ID: u16 = 0x2260;
/// Datasheet, unconfirmed.
pub const CONFIG_RESET: u16 = 0x4127;
const RST: u16 = 0x8000;

/// What the battery shows by default. 4000 mV: what the cores3 part's AXP2101
/// reports for the same battery, and near what our robot read on battery
/// (3.95 V at 82 %, tr_board README, 2026-11-01). StackChan has one 550 mAh
/// cell (M5Stack's docs).
pub const DEFAULT_MV: i32 = 4000;
/// 96 mA: our robot awake, screen at 20 %, servos powered (tr_board README,
/// 2026-11-01).
pub const DEFAULT_MA: i32 = 96;

/// The largest input that fits: 0x7FFF bus bits (40.96 V), and a signed
/// 16-bit shunt (+-8191.75 mA over 10 mOhm, so +-8191 whole mA).
const MAX_MV: f64 = 0x7FFF as f64 * 1.25;
const MAX_MA: f64 = 8191.0;

pub struct Ina226 {
    at: u8,
    /// The byte of the register the next read gives: 0 high, 1 low.
    byte: usize,
    config: u16,
    calibration: u16,
    mask: u16,
    alert: u16,
    /// The inputs: battery voltage (mV) and current (mA, positive =
    /// discharging).
    pub mv: i32,
    pub ma: i32,
    /// The last measured register values.
    shunt_raw: i16,
    bus_raw: u16,
}

impl Default for Ina226 {
    fn default() -> Self {
        let mut c = Self {
            at: 0,
            byte: 0,
            config: CONFIG_RESET,
            calibration: 0,
            mask: 0,
            alert: 0,
            mv: DEFAULT_MV,
            ma: DEFAULT_MA,
            shunt_raw: 0,
            bus_raw: 0,
        };
        c.measure(true, true);
        c
    }
}

/// The register values for a voltage and a current, with tr_board's scale.
pub fn encode(mv: i32, ma: i32) -> (u16, i16) {
    let bus = (mv as f64 / 1.25).round().clamp(0.0, 0x7FFF as f64) as u16;
    let shunt = (ma as f64 * 4.0).round().clamp(i16::MIN as f64, i16::MAX as f64) as i16;
    (bus, shunt)
}

impl Ina226 {
    fn mode(&self) -> u16 {
        self.config & 0x7
    }

    fn measure(&mut self, shunt: bool, bus: bool) {
        let (b, s) = encode(self.mv, self.ma);
        if shunt {
            self.shunt_raw = s;
        }
        if bus {
            self.bus_raw = b;
        }
    }

    /// Continuous modes (5, 6, 7) measure all the time: take the inputs now.
    fn follow(&mut self) {
        let m = self.mode();
        if m & 0x4 != 0 {
            self.measure(m & 0x1 != 0, m & 0x2 != 0);
        }
    }

    fn current(&self) -> i16 {
        let c = self.shunt_raw as i32 * self.calibration as i32 / 2048;
        c.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }

    fn reg(&mut self, reg: u8) -> u16 {
        match reg {
            CONFIG => self.config,
            SHUNT => {
                self.follow();
                self.shunt_raw as u16
            }
            BUS => {
                self.follow();
                self.bus_raw
            }
            POWER => {
                self.follow();
                let p = (self.current() as i32).unsigned_abs() * self.bus_raw as u32 / 20000;
                p.min(0xFFFF) as u16
            }
            CURRENT => {
                self.follow();
                self.current() as u16
            }
            CALIBRATION => self.calibration,
            MASK => self.mask,
            ALERT => self.alert,
            MAKER => MAKER_TI,
            DIE => DIE_ID,
            _ => 0,
        }
    }

    fn write_reg(&mut self, reg: u8, v: u16) {
        match reg {
            CONFIG if v & RST != 0 => {
                // A reset: every register back to its reset value.
                let (at, mv, ma) = (self.at, self.mv, self.ma);
                *self = Self { at, mv, ma, ..Self::default() };
                self.measure(true, true);
            }
            CONFIG => {
                self.config = v;
                let m = self.mode();
                if m != 0 && m & 0x4 == 0 {
                    self.measure(m & 0x1 != 0, m & 0x2 != 0); // triggered: once
                }
            }
            CALIBRATION => self.calibration = v & 0x7FFF,
            MASK => self.mask = v,
            ALERT => self.alert = v,
            _ => {} // read-only, or not a register
        }
    }

    /// One I2C transaction. A write sets the pointer (and writes the register
    /// with two more bytes); a read gives the pointed register, high byte
    /// first, from the byte where a split read stopped.
    pub fn transfer(&mut self, write: &[u8], read_len: usize, continues: bool) -> Vec<u8> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg;
            if data.len() >= 2 {
                self.write_reg(reg, u16::from_be_bytes([data[0], data[1]]));
            }
        }
        if !continues {
            self.byte = 0;
        }
        if read_len == 0 {
            return Vec::new();
        }
        let v = self.reg(self.at).to_be_bytes();
        (0..read_len)
            .map(|_| {
                let b = v[self.byte];
                self.byte ^= 1;
                b
            })
            .collect()
    }

    /// `battery_mv` or `battery_ma`. Returns false for any other name.
    pub fn input(&mut self, name: &str, value: f64) -> bool {
        match name {
            "mv" => self.mv = value.clamp(0.0, MAX_MV).round() as i32,
            "ma" => self.ma = value.clamp(-MAX_MA, MAX_MA).round() as i32,
            _ => return false,
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// tr_board's read16(): transmit_receive of [reg], 2 bytes, high first.
    fn read16(c: &mut Ina226, reg: u8) -> u16 {
        let b = c.transfer(&[reg], 2, false);
        u16::from_be_bytes([b[0], b[1]])
    }

    /// tr_battery_read()'s arithmetic, as C does it (integer division
    /// truncates toward zero).
    fn tr_battery(c: &mut Ina226) -> (i32, i32) {
        let bus = read16(c, BUS) as i32;
        let shunt = read16(c, SHUNT) as i16 as i32;
        (bus * 125 / 100, shunt * 2500 / 10000)
    }

    #[test]
    fn tr_board_open_and_read() {
        let mut c = Ina226::default();
        // ina226_open(): probe, maker ID, then the setup.
        assert_eq!(c.transfer(&[], 0, false), []);
        assert_eq!(read16(&mut c, MAKER), 0x5449);
        assert_eq!(c.transfer(&[CONFIG, 0x47, 0x27], 0, false), []);
        assert_eq!(read16(&mut c, CONFIG), 0x4727);
        assert_eq!(tr_battery(&mut c), (DEFAULT_MV, DEFAULT_MA));
    }

    #[test]
    fn reset_values() {
        let mut c = Ina226::default();
        assert_eq!(read16(&mut c, CONFIG), 0x4127);
        assert_eq!(read16(&mut c, DIE), 0x2260);
        assert_eq!(read16(&mut c, CALIBRATION), 0);
        assert_eq!(read16(&mut c, MASK), 0);
        assert_eq!(read16(&mut c, ALERT), 0);
        assert_eq!(read16(&mut c, 0x08), 0, "not a register");
    }

    #[test]
    fn scaling_round_trips_through_tr_board() {
        let mut c = Ina226::default();
        for (mv, ma) in [(4200, 0), (3700, 96), (3300, 390), (4120, -643), (3510, 0), (0, 8191)] {
            assert!(c.input("mv", mv as f64));
            assert!(c.input("ma", ma as f64));
            assert_eq!(tr_battery(&mut c), (mv, ma), "{mv} mV, {ma} mA");
        }
        // The raw bits: 1.25 mV and 0.25 mA (2.5 uV over 10 mOhm) per bit.
        c.input("mv", 4000.0);
        c.input("ma", -10.0);
        assert_eq!(read16(&mut c, BUS), 3200);
        assert_eq!(read16(&mut c, SHUNT), (-40i16) as u16, "charging reads negative");
        // Out of range values clamp to what the registers hold.
        c.input("mv", 99_999.0);
        c.input("ma", -99_999.0);
        assert_eq!(read16(&mut c, BUS), 0x7FFF);
        assert_eq!(read16(&mut c, SHUNT), (-8191i16 * 4) as u16);
    }

    #[test]
    fn pointer_persists_between_transactions() {
        let mut c = Ina226::default();
        c.transfer(&[MAKER], 0, false);
        assert_eq!(c.transfer(&[], 2, false), [0x54, 0x49]);
        assert_eq!(c.transfer(&[], 2, false), [0x54, 0x49], "still 0xFE");
        // Writing config leaves the pointer on config.
        c.transfer(&[CONFIG, 0x47, 0x27], 0, false);
        assert_eq!(c.transfer(&[], 2, false), [0x47, 0x27]);
        // One data byte only moves the pointer.
        c.transfer(&[ALERT, 0x12], 0, false);
        assert_eq!(c.transfer(&[], 2, false), [0, 0]);
    }

    #[test]
    fn split_and_long_reads() {
        let mut c = Ina226::default();
        // Split: one byte, then the next piece carries on.
        assert_eq!(c.transfer(&[MAKER], 1, false), [0x54]);
        assert_eq!(c.transfer(&[], 1, true), [0x49]);
        // A new read (not a continuation) starts at the high byte again.
        assert_eq!(c.transfer(&[], 1, false), [0x54]);
        // Longer than 2 bytes: the same register again.
        assert_eq!(c.transfer(&[DIE], 4, false), [0x22, 0x60, 0x22, 0x60]);
    }

    #[test]
    fn read_only_registers_ignore_writes() {
        let mut c = Ina226::default();
        for reg in [SHUNT, BUS, POWER, CURRENT, MAKER, DIE] {
            let before = read16(&mut c, reg);
            c.transfer(&[reg, 0xAB, 0xCD], 0, false);
            assert_eq!(read16(&mut c, reg), before, "register {reg:#04x}");
        }
        c.transfer(&[MASK, 0x04, 0x01], 0, false);
        c.transfer(&[ALERT, 0x12, 0x34], 0, false);
        c.transfer(&[CALIBRATION, 0xFF, 0xFF], 0, false);
        assert_eq!(read16(&mut c, MASK), 0x0401);
        assert_eq!(read16(&mut c, ALERT), 0x1234);
        assert_eq!(read16(&mut c, CALIBRATION), 0x7FFF, "bit 15 is not used");
    }

    #[test]
    fn current_and_power_follow_calibration() {
        let mut c = Ina226::default(); // 4000 mV, 96 mA: shunt 384, bus 3200
        assert_eq!(read16(&mut c, CURRENT), 0, "no calibration yet");
        assert_eq!(read16(&mut c, POWER), 0);
        // Current LSB 0.1 mA: CAL = 0.00512 / (0.0001 * 0.01) = 5120.
        c.transfer(&[CALIBRATION, 0x14, 0x00], 0, false);
        assert_eq!(read16(&mut c, CURRENT), 960, "384 * 5120 / 2048 = 96.0 mA");
        // Power LSB = 25 x current LSB = 2.5 mW: 960 * 3200 / 20000 = 153 (382 mW).
        assert_eq!(read16(&mut c, POWER), 153);
        c.input("ma", -96.0);
        assert_eq!(read16(&mut c, CURRENT) as i16, -960);
        assert_eq!(read16(&mut c, POWER), 153, "power has no sign");
    }

    #[test]
    fn config_reset_bit_and_modes() {
        let mut c = Ina226::default();
        c.transfer(&[CALIBRATION, 0x14, 0x00], 0, false);
        c.transfer(&[CONFIG, 0x47, 0x27], 0, false);
        // Power-down (mode 0) holds the last values.
        c.transfer(&[CONFIG, 0x47, 0x20], 0, false);
        c.input("mv", 3600.0);
        assert_eq!(tr_battery(&mut c).0, DEFAULT_MV, "held");
        // Bus triggered (mode 2): one bus measure on the write.
        c.transfer(&[CONFIG, 0x47, 0x22], 0, false);
        c.input("mv", 3500.0);
        assert_eq!(tr_battery(&mut c).0, 3600);
        // Back to continuous: it follows again.
        c.transfer(&[CONFIG, 0x47, 0x27], 0, false);
        assert_eq!(tr_battery(&mut c).0, 3500);
        // Bit 15: everything back to reset values, the inputs stay.
        c.transfer(&[CONFIG, 0x80, 0x00], 0, false);
        assert_eq!(read16(&mut c, CONFIG), 0x4127);
        assert_eq!(read16(&mut c, CALIBRATION), 0);
        assert_eq!(tr_battery(&mut c), (3500, DEFAULT_MA));
    }
}
