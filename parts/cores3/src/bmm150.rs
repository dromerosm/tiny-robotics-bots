//! BMM150 magnetometer, behind the BMI270's aux interface (aux address
//! 0x10). It is not on the main bus: M5Unified also probes a direct BMM150
//! at 0x10 there, and that probe must find nothing. Only M5Unified talks to
//! it; tr_board and the StackChan firmware never do.
//!
//! What is modelled (the BMI270 calls `read` and `write` for each aux byte):
//! - 0x4B power control: bit 0 powers the chip on. Bits 7 and 1 together are
//!   a soft reset, which clears itself and keeps the power bit. Powered off
//!   (suspend, the state after power-on), every register but 0x4B reads 0x00
//!   and ignores writes. M5Unified writes 0x83 (reset and power on).
//! - 0x40 chip ID, 0x32 (the value M5Unified requires).
//! - 0x4C op mode, bits 2:1: 00 normal, 01 forced (one measurement, then
//!   sleep), 11 sleep. Bits 5:3 are the ODR, stored only. Reset value 0x06
//!   (sleep). M5Unified writes 0x38 (normal, 30 Hz).
//! - 0x42-0x49 data, in the raw (not trim compensated) layout M5Unified
//!   reads: X and Y as 13 bits in [15:3], Z as 15 bits in [15:1], RHALL as
//!   14 bits in [15:2] with bit 0 of 0x48 the data ready flag. In normal
//!   mode every read measures the field again.
//! - Everything else stores what is written.
//!
//! The bit layouts, the reset values, the suspend behaviour and the soft
//! reset bits come from the datasheet, unconfirmed by any driver here. The
//! trim registers (0x5D-0x71) read 0x00 and RHALL reads 0: no driver here
//! reads them, so the model does not invent values.
//!
//! Scale: 0.3 µT per raw LSB on every axis (datasheet nominal, unconfirmed).
//! It is also what M5Unified's own scale gives back: it reads X as
//! int16 >> 2 (2 counts per raw LSB) and multiplies by 10 * 4912 / 32760
//! mG, about 0.15 µT per count (`BMI270_Class.inl` getConvertParam).

/// µT per raw LSB (datasheet nominal, unconfirmed; matches M5Unified's scale).
pub const UT_PER_LSB: f64 = 0.3;

const CHIP_ID: u8 = 0x40;
const DATA_X: u8 = 0x42;
const DATA_RHALL_LSB: u8 = 0x48;
const POWER: u8 = 0x4B;
const OPMODE: u8 = 0x4C;

const ID: u8 = 0x32;
const POWER_ON: u8 = 0x01;
const SOFT_RESET: u8 = 0x82; // bits 7 and 1 (datasheet, unconfirmed)
const OPMODE_MASK: u8 = 0x06;
const OP_NORMAL: u8 = 0x00;
const OP_FORCED: u8 = 0x02;
const OP_SLEEP: u8 = 0x06;

pub struct Bmm150 {
    mem: [u8; 256],
    /// The field on the chip's own axes, in µT.
    field: [f64; 3],
}

impl Default for Bmm150 {
    fn default() -> Self {
        Self { mem: Self::reset_regs(0), field: [0.0; 3] }
    }
}

impl Bmm150 {
    /// Register values after a reset; `power` is what 0x4B bit 0 keeps.
    fn reset_regs(power: u8) -> [u8; 256] {
        let mut mem = [0u8; 256];
        mem[CHIP_ID as usize] = ID;
        mem[POWER as usize] = power & POWER_ON;
        mem[OPMODE as usize] = OP_SLEEP; // datasheet, unconfirmed
        mem
    }

    fn powered(&self) -> bool {
        self.mem[POWER as usize] & POWER_ON != 0
    }

    fn opmode(&self) -> u8 {
        self.mem[OPMODE as usize] & OPMODE_MASK
    }

    /// The field the chip measures, on its own axes, in µT.
    pub fn set_field(&mut self, field: [f64; 3]) {
        self.field = field;
    }

    /// Encode the field into 0x42-0x49 and set data ready.
    fn measure(&mut self) {
        let raw = |ut: f64, max: i32| ((ut / UT_PER_LSB).round() as i32).clamp(-max - 1, max);
        let x = (raw(self.field[0], 4095) << 3) as u16; // 13 bits in [15:3]
        let y = (raw(self.field[1], 4095) << 3) as u16;
        let z = (raw(self.field[2], 16383) << 1) as u16; // 15 bits in [15:1]
        let rhall: u16 = 0x0001; // RHALL 0 (not modelled), bit 0 data ready
        for (i, v) in [x, y, z, rhall].into_iter().enumerate() {
            self.mem[DATA_X as usize + 2 * i] = v as u8;
            self.mem[DATA_X as usize + 2 * i + 1] = (v >> 8) as u8;
        }
    }

    /// One aux read burst of `len` bytes from `reg` on (the address moves
    /// on after each byte). In normal mode a burst that touches the data
    /// registers gets one fresh sample, so X, Y and Z belong together.
    pub fn read_burst(&mut self, reg: u8, len: usize) -> Vec<u8> {
        let end = reg as usize + len;
        if self.powered() && self.opmode() == OP_NORMAL && (reg as usize) <= (DATA_RHALL_LSB + 1) as usize && end > DATA_X as usize {
            self.measure();
        }
        (0..len).map(|i| self.read(reg.wrapping_add(i as u8))).collect()
    }

    fn read(&self, reg: u8) -> u8 {
        if reg != POWER && !self.powered() {
            return 0x00; // suspend (datasheet, unconfirmed)
        }
        self.mem[reg as usize]
    }

    /// One register write through the aux interface.
    pub fn write(&mut self, reg: u8, value: u8) {
        if reg == POWER {
            if value & SOFT_RESET == SOFT_RESET {
                self.mem = Self::reset_regs(value);
            } else {
                self.mem[POWER as usize] = value & POWER_ON;
            }
            return;
        }
        if !self.powered() || reg == CHIP_ID {
            return;
        }
        self.mem[reg as usize] = value;
        if reg == OPMODE {
            match value & OPMODE_MASK {
                OP_NORMAL => self.measure(),
                OP_FORCED => {
                    self.measure();
                    self.mem[OPMODE as usize] = (value & !OPMODE_MASK) | OP_SLEEP;
                }
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn int16(c: &mut Bmm150, reg: u8) -> i16 {
        let b = c.read_burst(reg, 2);
        i16::from_le_bytes([b[0], b[1]])
    }

    fn rd(c: &mut Bmm150, reg: u8) -> u8 {
        c.read_burst(reg, 1)[0]
    }

    #[test]
    fn id_reads_zero_until_powered_on() {
        let mut c = Bmm150::default();
        assert_eq!(rd(&mut c, CHIP_ID), 0x00, "suspend after power-on");
        c.write(OPMODE, 0x38);
        assert_eq!(rd(&mut c, OPMODE), 0x00, "writes ignored in suspend");
        c.write(POWER, 0x83); // as M5Unified: soft reset + power on
        assert_eq!(rd(&mut c, CHIP_ID), 0x32);
        assert_eq!(rd(&mut c, POWER), 0x01, "the reset bits clear themselves");
        assert_eq!(rd(&mut c, OPMODE), OP_SLEEP, "sleep after reset");
    }

    #[test]
    fn soft_reset_restores_registers_and_keeps_power() {
        let mut c = Bmm150::default();
        c.write(POWER, 0x01);
        c.write(OPMODE, 0x38);
        c.write(0x51, 0x04);
        c.write(POWER, 0x83);
        assert_eq!(rd(&mut c, OPMODE), OP_SLEEP);
        assert_eq!(rd(&mut c, 0x51), 0x00);
        assert_eq!(rd(&mut c, CHIP_ID), 0x32);
    }

    #[test]
    fn raw_layout_as_m5unified_reads_it() {
        let mut c = Bmm150::default();
        c.write(POWER, 0x83);
        c.set_field([40.0, -12.0, -30.0]);
        c.write(OPMODE, 0x38); // normal
        let (x, y, z) = (int16(&mut c, DATA_X) >> 2, int16(&mut c, DATA_X + 2) >> 2, int16(&mut c, DATA_X + 4) & !1);
        let ut = |v: i16| v as f64 * 10.0 * 4912.0 / 32760.0 / 10.0; // M5Unified's mG, then µT
        assert!((ut(x) - 40.0).abs() < 0.35, "x {}", ut(x));
        assert!((ut(y) + 12.0).abs() < 0.35, "y {}", ut(y));
        assert!((ut(z) + 30.0).abs() < 0.35, "z {}", ut(z));
        assert_eq!(rd(&mut c, DATA_RHALL_LSB) & 1, 1, "data ready");
        assert_eq!(rd(&mut c, DATA_X) & 0x07, 0, "X bits 2:0 are not data");
    }

    #[test]
    fn normal_mode_follows_the_field_and_sleep_holds_it() {
        let mut c = Bmm150::default();
        c.write(POWER, 0x01);
        c.write(OPMODE, OP_NORMAL);
        c.set_field([3.0, 0.0, 0.0]);
        assert_eq!(int16(&mut c, DATA_X) >> 3, 10);
        c.set_field([6.0, 0.0, 0.0]);
        assert_eq!(int16(&mut c, DATA_X) >> 3, 20, "normal mode measures again");
        c.write(OPMODE, OP_SLEEP);
        c.set_field([9.0, 0.0, 0.0]);
        assert_eq!(int16(&mut c, DATA_X) >> 3, 20, "sleep keeps the last sample");
    }

    #[test]
    fn forced_mode_measures_once_then_sleeps() {
        let mut c = Bmm150::default();
        c.write(POWER, 0x01);
        c.set_field([0.0, 0.0, 30.0]);
        c.write(OPMODE, OP_FORCED);
        assert_eq!(rd(&mut c, OPMODE) & OPMODE_MASK, OP_SLEEP);
        assert_eq!(int16(&mut c, DATA_X + 4) >> 1, 100);
    }
}
