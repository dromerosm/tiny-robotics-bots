//! Injoinic IP5306 power bank chip, I2C version (I2C 0x75), from Injoinic's
//! "IP5306 register file" V1.21:
//!
//! - 0x00 SYS_CTL0 0xBE, 0x01 SYS_CTL1 0x05, 0x02 SYS_CTL2 0x00, 0x20
//!   CHG_CTL0 0x02, 0x21 CHG_CTL1 0x54, 0x22 CHG_CTL2 0x01, 0x23 CHG_CTL3
//!   0x20, all read/write. 0x24 CHG_DIG_CTL0 has no reset value in the
//!   document: 0xD6, as read on our Fire.
//! - 0x70 READ0 bit 3: charging. 0x71 READ1 bit 3: full. 0x72 READ2 bit 2:
//!   light load. 0x77 READ3: key events, write 1 to clear (no key here).
//! - 0x78, the battery level in bits 7:4 (0x0 100%, 0x8 75%, 0xC 50%, 0xE
//!   25%, 0xF 0%), is not in that document; it comes from M5Stack's drivers.
//!
//! The document shows single-byte transfers only; the pointer moves on here.
//! Other registers are stored and read back.

pub struct Ip5306 {
    regs: [u8; 256],
    at: u8,
    pub charging: bool,
    pub battery: u8,
}

impl Default for Ip5306 {
    fn default() -> Self {
        let mut regs = [0u8; 256];
        for (r, v) in [(0x00, 0xBE), (0x01, 0x05), (0x20, 0x02), (0x21, 0x54), (0x22, 0x01), (0x23, 0x20), (0x24, 0xD6), (0x72, 0x04)] {
            regs[r] = v;
        }
        Self { regs, at: 0, charging: false, battery: 100 }
    }
}

impl Ip5306 {
    fn level_bits(&self) -> u8 {
        match self.battery {
            100.. => 0x00,
            75.. => 0x80,
            50.. => 0xC0,
            25.. => 0xE0,
            _ => 0xF0,
        }
    }

    pub fn transfer(&mut self, write: &[u8], read_len: usize) -> Vec<u8> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg;
            for &b in data {
                match self.at {
                    0x70..=0x72 | 0x78 => {} // read-only
                    0x77 => self.regs[0x77] &= !b,
                    r => self.regs[r as usize] = b,
                }
                self.at = self.at.wrapping_add(1);
            }
        }
        self.regs[0x70] = if self.charging { 0x08 } else { 0x00 };
        self.regs[0x71] = if self.charging && self.battery >= 100 { 0x08 } else { 0x00 };
        self.regs[0x78] = self.level_bits();
        (0..read_len)
            .map(|_| {
                let v = self.regs[self.at as usize];
                self.at = self.at.wrapping_add(1);
                v
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_and_charging_as_m5stack_power_reads_them() {
        let mut p = Ip5306::default();
        assert_eq!(p.transfer(&[0x78], 1), vec![0x00], "100%");
        p.battery = 60;
        assert_eq!(p.transfer(&[0x78], 1), vec![0xC0]);
        assert_eq!(p.transfer(&[0x70], 1), vec![0x00]);
        p.charging = true;
        assert_eq!(p.transfer(&[0x70], 1), vec![0x08]);
        assert_eq!(p.transfer(&[0x24], 1), vec![0xD6]);
    }

    #[test]
    fn control_registers_keep_what_is_written() {
        let mut p = Ip5306::default();
        p.transfer(&[0x00, 0x37], 0);
        assert_eq!(p.transfer(&[0x00], 1), vec![0x37]);
        p.transfer(&[0x70, 0xFF], 0);
        assert_eq!(p.transfer(&[0x70], 1), vec![0x00], "read-only");
    }
}
