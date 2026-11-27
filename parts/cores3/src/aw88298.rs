//! AW88298 speaker amplifier, address 0x36. Register addresses are 8 bits,
//! values 16 bits, high byte first. Every rule and reset value below is from
//! the Awinic AW88298 data sheet (V1.6, May 2022) unless a line says
//! otherwise.
//!
//! - A write is the register, then high and low byte pairs; the register
//!   moves on after each pair. A read gives the high byte, then the low byte,
//!   then the next register. A lone high byte at the end of a write is
//!   dropped. A long read that arrives in pieces (`continues`) goes on from
//!   the byte where it stopped.
//! - 0x00 reads the chip ID, 0x1852 (also in esp_codec_dev's aw88298_reg.h).
//!   Writing 0x55AA there resets every register to its reset value. Other
//!   writes to 0x00 are ignored.
//! - Read only: 0x01 SYSST (0), 0x02 SYSINT (0; it clears on read), 0x10
//!   HAGCST (0), 0x12 VDD (0x0263), 0x13 TEMP (0x0019, 25 °C), 0x14 PVDD
//!   (0x0263). These are the data sheet's reset values; nothing here changes
//!   them.
//! - Everything else reads back exactly what was written, so the drivers'
//!   read, change and write steps work. Addresses the data sheet does not
//!   list are plain storage, 0 at reset (our choice).
//! - Reset values: 03 0xEFBF, 04 0x4003 (PWDN = 1, AMPPD = 1: off), 05
//!   0x0038, 06 0x14E8, 07 0x0330, 09 0x3940, 0A 0x0030, 0B 0x01E0, 0C
//!   0x0064, 60 0x0402, 61 0x6673 (esp_codec_dev's comment says the same).
//!
//! What the drivers do (cores3-chips-spec.md section 6):
//! - esp_codec_dev (tr_board, StackChan through xiaozhi): open writes
//!   00←55AA, 04←4040, 05←0008, 06←3CC8, 0C←3064, 61←0673. set_fs reads and
//!   writes 06 (bits 7:4 sample size, 3:0 rate). Start reads 04, clears bits
//!   1:0 and sets bit 6; stop does the reverse. Mute sets 05 bit 4. Volume
//!   writes 0C = reg << 8 | 0x64.
//! - M5Unified writes only. On: 61←0673, 04←4040, 05←0008, 06←14C0|rate,
//!   0C←0064. Off: 04←4000.
//!
//! State:
//! - `amp_on`: 1 when 04 has PWDN (bit 0) = 0, AMPPD (bit 1) = 0 and I2SEN
//!   (bit 6) = 1: powered, amplifier on and taking I2S samples. Both drivers'
//!   "on" values give 1 and their "off" values give 0.
//! - `amp_volume`: the volume in dB from 0C bits 15:8, as the data sheet
//!   reads them: bits 11:8 in -0.5 dB steps and bits 15:12 in -6 dB steps
//!   (0 = full volume). Note: esp_codec_dev assumes plain -0.5 dB steps
//!   (0xC0 = -96 dB), so its volumes land elsewhere on the real chip; we show
//!   what the chip would do.

use crate::chip::Chip;
use esp_part::State;

const ID: u8 = 0x00;
const SYSCTRL: u8 = 0x04;
const HAGCCFG4: u8 = 0x0C;
const CHIP_ID: u16 = 0x1852;
const RESET_CODE: u16 = 0x55AA;
const READ_ONLY: [u8; 7] = [ID, 0x01, 0x02, 0x10, 0x12, 0x13, 0x14];
const RESET: [(u8, u16); 17] = [
    (ID, CHIP_ID),
    (0x03, 0xEFBF),
    (SYSCTRL, 0x4003),
    (0x05, 0x0038),
    (0x06, 0x14E8),
    (0x07, 0x0330),
    (0x09, 0x3940),
    (0x0A, 0x0030),
    (0x0B, 0x01E0),
    (HAGCCFG4, 0x0064),
    (0x12, 0x0263),
    (0x13, 0x0019),
    (0x14, 0x0263),
    (0x60, 0x0402),
    (0x61, 0x6673),
    (0x01, 0x0000),
    (0x02, 0x0000),
];

pub struct Aw88298 {
    regs: [u16; 256],
    at: u8,
    /// The pointer is on the low byte of `at`.
    low: bool,
}

fn reset_regs() -> [u16; 256] {
    let mut regs = [0u16; 256];
    for (r, v) in RESET {
        regs[r as usize] = v;
    }
    regs
}

impl Default for Aw88298 {
    fn default() -> Self {
        Self { regs: reset_regs(), at: 0, low: false }
    }
}

impl Aw88298 {
    pub fn amp_on(&self) -> bool {
        self.regs[SYSCTRL as usize] & 0x43 == 0x40
    }

    /// The volume in dB, from HAGCCFG4 bits 15:8.
    pub fn volume_db(&self) -> f32 {
        let vol = self.regs[HAGCCFG4 as usize] >> 8;
        // + 0.0 turns -0.0 into 0.0, so full volume prints "0".
        -6.0 * (vol >> 4) as f32 - 0.5 * (vol & 0x0F) as f32 + 0.0
    }

    fn write_reg(&mut self, r: u8, v: u16) {
        if r == ID && v == RESET_CODE {
            self.regs = reset_regs();
        } else if !READ_ONLY.contains(&r) {
            self.regs[r as usize] = v;
        }
    }
}

impl Chip for Aw88298 {
    fn transfer(&mut self, write: &[u8], read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg;
            self.low = false;
            for pair in data.chunks_exact(2) {
                self.write_reg(self.at, u16::from_be_bytes([pair[0], pair[1]]));
                self.at = self.at.wrapping_add(1);
            }
        }
        let data = (0..read_len)
            .map(|_| {
                let v = self.regs[self.at as usize].to_be_bytes()[self.low as usize];
                if self.low {
                    self.at = self.at.wrapping_add(1);
                }
                self.low = !self.low;
                v
            })
            .collect();
        Some(data)
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state.set("amp_on", self.amp_on() as u8).set("amp_volume", self.volume_db())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(c: &mut Aw88298, write: &[u8], read_len: usize) -> Vec<u8> {
        c.transfer(write, read_len, false, 0).unwrap()
    }

    /// esp_codec_dev's aw88298_write_reg: register, high byte, low byte.
    fn wr(c: &mut Aw88298, reg: u8, v: u16) {
        t(c, &[reg, (v >> 8) as u8, v as u8], 0);
    }

    /// esp_codec_dev's aw88298_read_reg: one transaction, 2 bytes back.
    fn rd(c: &mut Aw88298, reg: u8) -> u16 {
        let b = t(c, &[reg], 2);
        (b[0] as u16) << 8 | b[1] as u16
    }

    /// M5Unified's aw88298_write_reg: the value byte-swapped, so the bytes
    /// leave high first.
    fn mu_wr(c: &mut Aw88298, reg: u8, v: u16) {
        let mut w = vec![reg];
        w.extend_from_slice(&v.swap_bytes().to_le_bytes());
        t(c, &w, 0);
    }

    fn state(c: &Aw88298) -> (String, String) {
        let s = c.state(State::new(), 0);
        (s.get("amp_on").unwrap().to_string(), s.get("amp_volume").unwrap().to_string())
    }

    #[test]
    fn id_and_reset_values() {
        let mut c = Aw88298::default();
        assert_eq!(rd(&mut c, ID), 0x1852);
        assert_eq!(rd(&mut c, SYSCTRL), 0x4003);
        assert_eq!(rd(&mut c, 0x06), 0x14E8);
        assert_eq!(rd(&mut c, 0x61), 0x6673);
        assert_eq!(rd(&mut c, 0x13), 0x0019, "25 C");
        assert_eq!(state(&c), ("0".into(), "0".into()));
    }

    #[test]
    fn esp_codec_dev_open_set_fs_start_volume_mute_stop() {
        let mut c = Aw88298::default();
        wr(&mut c, SYSCTRL, 0x0000); // something left over from before
        // aw88298_open (aw88298.c:220-225).
        wr(&mut c, ID, 0x55AA);
        assert_eq!(rd(&mut c, SYSCTRL), 0x4003, "the reset restored the defaults");
        wr(&mut c, SYSCTRL, 0x4040);
        wr(&mut c, 0x05, 0x0008);
        wr(&mut c, 0x06, 0x3CC8);
        wr(&mut c, HAGCCFG4, 0x3064);
        wr(&mut c, 0x61, 0x0673);
        assert_eq!(rd(&mut c, ID), 0x1852, "the reset code is not stored");
        // set_fs: 32 bits (StackChan's TX), then 48 kHz.
        let v = rd(&mut c, 0x06);
        wr(&mut c, 0x06, (v & !0xF0) | 0xE0);
        let v = rd(&mut c, 0x06);
        wr(&mut c, 0x06, (v & !0x0F) | 0x08);
        assert_eq!(rd(&mut c, 0x06), 0x3CE8);
        // enable(true): aw88298_start.
        let v = rd(&mut c, SYSCTRL);
        wr(&mut c, SYSCTRL, (v & !0x03) | 0x40);
        assert_eq!(rd(&mut c, SYSCTRL), 0x4040);
        assert_eq!(state(&c).0, "1");
        // set_vol: reg 0x1E (esp_codec_dev means -15 dB) → 0C = 0x1E64.
        wr(&mut c, HAGCCFG4, 0x1E << 8 | 0x64);
        assert_eq!(state(&c), ("1".into(), "-13".into()), "data sheet: -6 dB - 7 dB");
        // mute: 05 bit 4 on, then off.
        let v = rd(&mut c, 0x05);
        wr(&mut c, 0x05, v | 0x10);
        assert_eq!(rd(&mut c, 0x05), 0x0018);
        let v = rd(&mut c, 0x05);
        wr(&mut c, 0x05, v & !0x10);
        // enable(false): aw88298_stop.
        let v = rd(&mut c, SYSCTRL);
        wr(&mut c, SYSCTRL, (v | 0x03) & !0x40);
        assert_eq!(rd(&mut c, SYSCTRL), 0x4003);
        assert_eq!(state(&c).0, "0");
    }

    #[test]
    fn m5unified_write_only_enable_and_disable() {
        let mut c = Aw88298::default();
        // _speaker_enabled_cb_cores3 at 48 kHz: rate index 8 → 06 = 0x14C8.
        mu_wr(&mut c, 0x61, 0x0673);
        mu_wr(&mut c, SYSCTRL, 0x4040);
        mu_wr(&mut c, 0x05, 0x0008);
        mu_wr(&mut c, 0x06, 0x14C8);
        mu_wr(&mut c, HAGCCFG4, 0x0064);
        assert_eq!(rd(&mut c, 0x06), 0x14C8);
        assert_eq!(rd(&mut c, 0x61), 0x0673);
        assert_eq!(state(&c), ("1".into(), "0".into()), "full volume");
        mu_wr(&mut c, SYSCTRL, 0x4000); // I2SEN = 0
        assert_eq!(state(&c).0, "0");
    }

    #[test]
    fn volume_follows_the_data_sheet_steps() {
        let mut c = Aw88298::default();
        for (vol, db) in [(0x00u16, "0"), (0x01, "-0.5"), (0x10, "-6"), (0xC0, "-72"), (0xFF, "-97.5")] {
            wr(&mut c, HAGCCFG4, vol << 8 | 0x64);
            assert_eq!(state(&c).1, db, "0C high byte {vol:#04x}");
        }
    }

    #[test]
    fn read_only_registers_and_other_writes_to_00() {
        let mut c = Aw88298::default();
        wr(&mut c, 0x12, 0x0000);
        wr(&mut c, ID, 0x1234);
        assert_eq!(rd(&mut c, 0x12), 0x0263);
        assert_eq!(rd(&mut c, ID), 0x1852);
        wr(&mut c, 0x30, 0xBEEF); // not in the data sheet: plain storage
        assert_eq!(rd(&mut c, 0x30), 0xBEEF);
    }

    #[test]
    fn burst_write_and_read_move_one_register_per_pair() {
        let mut c = Aw88298::default();
        t(&mut c, &[0x09, 0x11, 0x22, 0x33, 0x44, 0x55], 0); // 09, 0A, and a lone byte
        assert_eq!(t(&mut c, &[0x09], 6), [0x11, 0x22, 0x33, 0x44, 0x01, 0xE0]);
    }

    #[test]
    fn split_read_goes_on_from_the_byte_where_it_stopped() {
        let mut c = Aw88298::default();
        assert_eq!(c.transfer(&[ID], 3, false, 0), Some(vec![0x18, 0x52, 0x00]));
        assert_eq!(c.transfer(&[], 3, true, 1), Some(vec![0x00, 0x00, 0x00]), "rest of 01, then 02");
        assert_eq!(c.transfer(&[], 2, true, 2), Some(vec![0xEF, 0xBF]), "03");
    }

    #[test]
    fn probe_acks() {
        let mut c = Aw88298::default();
        assert_eq!(c.transfer(&[], 0, false, 0), Some(vec![]));
    }
}
