//! ES7210 microphone ADC, address 0x40. Byte registers with byte values.
//! Reset values and bit meanings are from the Everest ES7210 data sheet
//! (Revision 21.0, January 2023).
//!
//! - Every register, 0x00-0xFF, reads back what was written, so the drivers'
//!   read, change and write steps and esp_codec_dev's read of 0x01 (stored,
//!   then written again on start) work. Only the chip ID is read only: 0x3D =
//!   0x72, 0x3E = 0x10, 0x3F = 0x00 (version).
//! - 0x00 bit 2 (RST_REGS) resets every other register to its reset value.
//!   We reset when it is written as 1 and do not hold the registers while it
//!   stays 1 (the data sheet does not say). Both drivers start with 00←FF
//!   and clear it before anything else.
//! - The pointer moves on after each byte. The data sheet only shows
//!   one-byte accesses; the drivers make only those too.
//! - 0x0B (chip status) is plain storage: its state-machine bits do not
//!   follow the chip's state.
//! - No audio here: the samples come from the I2S side of the emulator.
//!
//! Reset values (data sheet): 00 0x32, 01 0x20, 02 0x02, 03 0x04, 04 0x01,
//! 07 0x20, 08 0x10, 09 0x40, 0A 0x40, 0D 0x09, 18/19 0xF7, 1B-1E 0xBF, 20
//! 0x26, 21 0x26, 22 0x06, 23 0x26, 40 0x80, 41/42 0x71, 4B/4C 0xFF, the
//! chip ID above, every other register 0x00.
//!
//! What the drivers do (cores3-chips-spec.md section 7):
//! - esp_codec_dev (tr_board and StackChan): open writes 00←FF, 00←41,
//!   01←3F, the timing and filter registers, reads and writes 08 bit 0, writes
//!   40←43, 41←70, 42←70, 07←20, 02←C1, selects the microphones (read and
//!   write of 43-46, 01, writes of 4B/4C and 12) and the gain (43-46 bits
//!   3:0), then reads 01 back and keeps it. Start writes that 01 value, 06←00,
//!   40←43, 47-4A←08, selects the microphones again, then 00←71, 00←41. Stop
//!   writes 47-4C←FF, 40←C0, 01←7F, 06←07.
//! - M5Unified writes one register per transaction: 00←FF, and when it turns
//!   the microphone on, a table from 00←41 to 01←14 (M5Unified.inl:1026-1068).
//!   Off is 00←FF alone.
//!
//! State `mic_on`: 1 when the chip would be converting, from the data sheet's
//! bits:
//! - 00: CSM_ON (bit 0) = 1, and RST_DIG (bit 1) and RST_REGS (bit 2) = 0;
//! - 01: MCLK_OFF (bit 0) = 0; 40: PDN_ANA (bit 7) = 0;
//! - and one ADC pair is running: for MIC1/2, RST_ADC12 (00 bit 4) = 0,
//!   ADC12_MCLK_OFF (01 bit 3) = 0 and one of PDN_MOD1/PDN_MOD2 (4B bits 3:2)
//!   = 0; for MIC3/4 the same with 00 bit 5, 01 bit 4 and 4C.
//!
//! So it is 0 at reset, after esp_codec_dev's stop and after M5Unified's
//! 00←FF, and 1 after M5Unified's table. With esp_codec_dev it is already 1
//! once `open` has selected the microphones, before `enable`: the chip is
//! awake by then, and esp_codec_dev_open always enables it right after.

use crate::chip::Chip;
use esp_part::State;

const RESET_CTL: u8 = 0x00;
const CLOCK_OFF: usize = 0x01;
const ANALOG: usize = 0x40;
const MIC12_PDN: usize = 0x4B;
const MIC34_PDN: usize = 0x4C;
const RST_REGS: u8 = 0x04;
const READ_ONLY: [u8; 3] = [0x3D, 0x3E, 0x3F];
const RESET: [(u8, u8); 28] = [
    (0x00, 0x32),
    (0x01, 0x20),
    (0x02, 0x02),
    (0x03, 0x04),
    (0x04, 0x01),
    (0x07, 0x20),
    (0x08, 0x10),
    (0x09, 0x40),
    (0x0A, 0x40),
    (0x0D, 0x09),
    (0x18, 0xF7),
    (0x19, 0xF7),
    (0x1B, 0xBF),
    (0x1C, 0xBF),
    (0x1D, 0xBF),
    (0x1E, 0xBF),
    (0x20, 0x26),
    (0x21, 0x26),
    (0x22, 0x06),
    (0x23, 0x26),
    (0x3D, 0x72),
    (0x3E, 0x10),
    (0x3F, 0x00),
    (0x40, 0x80),
    (0x41, 0x71),
    (0x42, 0x71),
    (0x4B, 0xFF),
    (0x4C, 0xFF),
];

pub struct Es7210 {
    mem: [u8; 256],
    at: u8,
}

fn reset_mem() -> [u8; 256] {
    let mut mem = [0u8; 256];
    for (r, v) in RESET {
        mem[r as usize] = v;
    }
    mem
}

impl Default for Es7210 {
    fn default() -> Self {
        Self { mem: reset_mem(), at: 0 }
    }
}

impl Es7210 {
    pub fn mic_on(&self) -> bool {
        let m = &self.mem;
        let r0 = m[RESET_CTL as usize];
        let pair12 = r0 & 0x10 == 0 && m[CLOCK_OFF] & 0x08 == 0 && m[MIC12_PDN] & 0x0C != 0x0C;
        let pair34 = r0 & 0x20 == 0 && m[CLOCK_OFF] & 0x10 == 0 && m[MIC34_PDN] & 0x0C != 0x0C;
        r0 & 0x07 == 0x01 && m[CLOCK_OFF] & 0x01 == 0 && m[ANALOG] & 0x80 == 0 && (pair12 || pair34)
    }

    fn write_reg(&mut self, r: u8, v: u8) {
        if r == RESET_CTL && v & RST_REGS != 0 {
            self.mem = reset_mem();
        }
        if !READ_ONLY.contains(&r) {
            self.mem[r as usize] = v;
        }
    }
}

impl Chip for Es7210 {
    fn transfer(&mut self, write: &[u8], read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg;
            for &v in data {
                self.write_reg(self.at, v);
                self.at = self.at.wrapping_add(1);
            }
        }
        let data = (0..read_len)
            .map(|_| {
                let v = self.mem[self.at as usize];
                self.at = self.at.wrapping_add(1);
                v
            })
            .collect();
        Some(data)
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state.set("mic_on", self.mic_on() as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIC1: u8 = 1;
    const MIC2: u8 = 2;
    const MIC3: u8 = 4;
    const MIC4: u8 = 8;

    /// esp_codec_dev's es7210 driver, call for call (es7210.c, 1.5.11),
    /// one register per transaction as audio_codec_ctrl_i2c does.
    struct Driver<'a> {
        c: &'a mut Es7210,
        mics: u8,
        gain: u8,
        off_reg: u8,
    }

    impl Driver<'_> {
        fn wr(&mut self, r: u8, v: u8) {
            assert_eq!(self.c.transfer(&[r, v], 0, false, 0), Some(vec![]));
        }
        fn rd(&mut self, r: u8) -> u8 {
            self.c.transfer(&[r], 1, false, 0).unwrap()[0]
        }
        fn update(&mut self, r: u8, bits: u8, data: u8) {
            let v = self.rd(r);
            self.wr(r, (v & !bits) | (bits & data));
        }
        fn mic_select(&mut self) {
            for i in 0..4 {
                self.update(0x43 + i, 0x10, 0x00);
            }
            self.wr(0x4B, 0xFF);
            self.wr(0x4C, 0xFF);
            for (i, mic) in [MIC1, MIC2, MIC3, MIC4].into_iter().enumerate() {
                if self.mics & mic != 0 {
                    let (clocks, power) = if i < 2 { (0x0B, 0x4B) } else { (0x15, 0x4C) };
                    self.update(0x01, clocks, 0x00);
                    self.wr(power, 0x00);
                    self.update(0x43 + i as u8, 0x10, 0x10);
                    let gain = self.gain;
                    self.update(0x43 + i as u8, 0x0F, gain);
                }
            }
            let tdm = self.mics.count_ones() >= 3;
            self.wr(0x12, if tdm { 0x02 } else { 0x00 });
        }
        fn open(&mut self) {
            for (r, v) in [(0x00, 0xFF), (0x00, 0x41), (0x01, 0x3F), (0x09, 0x30), (0x0A, 0x30), (0x23, 0x2A)] {
                self.wr(r, v);
            }
            for (r, v) in [(0x22, 0x0A), (0x20, 0x0A), (0x21, 0x2A)] {
                self.wr(r, v);
            }
            self.update(0x08, 0x01, 0x00); // slave
            for (r, v) in [(0x40, 0x43), (0x41, 0x70), (0x42, 0x70), (0x07, 0x20), (0x02, 0xC1)] {
                self.wr(r, v);
            }
            self.mic_select();
            // _es7210_set_channel_gain(0xF, 30.0): GAIN_30DB = 10.
            for (i, mic) in [MIC1, MIC2, MIC3, MIC4].into_iter().enumerate() {
                if self.mics & mic != 0 {
                    self.update(0x43 + i as u8, 0x0F, 10);
                }
            }
            self.off_reg = self.rd(0x01);
        }
        fn set_fs_16bit(&mut self) {
            let v = self.rd(0x11);
            self.wr(0x11, (v & 0x1F) | 0x60);
            let v = self.rd(0x11);
            self.wr(0x11, v & 0xFC);
        }
        fn start(&mut self) {
            let off = self.off_reg;
            self.wr(0x01, off);
            self.wr(0x06, 0x00);
            self.wr(0x40, 0x43);
            for r in 0x47..=0x4A {
                self.wr(r, 0x08);
            }
            self.mic_select();
            self.wr(0x40, 0x43);
            self.wr(0x00, 0x71);
            self.wr(0x00, 0x41);
        }
        fn stop(&mut self) {
            for r in 0x47..=0x4C {
                self.wr(r, 0xFF);
            }
            self.wr(0x40, 0xC0);
            self.wr(0x01, 0x7F);
            self.wr(0x06, 0x07);
        }
    }

    fn mic_on(c: &Es7210) -> &'static str {
        match c.state(State::new(), 0).get("mic_on") {
            Some("1") => "1",
            Some("0") => "0",
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn reset_values_and_chip_id() {
        let mut c = Es7210::default();
        assert_eq!(c.transfer(&[0x3D], 3, false, 0), Some(vec![0x72, 0x10, 0x00]));
        assert_eq!(c.transfer(&[0x00], 2, false, 0), Some(vec![0x32, 0x20]));
        assert_eq!(c.transfer(&[0x4B], 2, false, 0), Some(vec![0xFF, 0xFF]));
        c.transfer(&[0x3D, 0x00], 0, false, 0);
        assert_eq!(c.transfer(&[0x3D], 1, false, 0), Some(vec![0x72]), "read only");
        assert_eq!(mic_on(&c), "0");
    }

    #[test]
    fn esp_codec_dev_mic1_2_open_start_stop() {
        // tr_mic.c: the default selection, MIC1 + MIC2.
        let mut c = Es7210::default();
        let mut d = Driver { c: &mut c, mics: 0, gain: 0, off_reg: 0 };
        d.mics = MIC1 | MIC2;
        d.open();
        assert_eq!(d.off_reg, 0x34, "01 round trip: 3F with ADC12 clocks on");
        assert_eq!([d.rd(0x43), d.rd(0x44), d.rd(0x45)], [0x1A, 0x1A, 0x00], "on, 30 dB");
        assert_eq!([d.rd(0x4B), d.rd(0x4C), d.rd(0x12), d.rd(0x08)], [0x00, 0xFF, 0x00, 0x10]);
        d.set_fs_16bit();
        assert_eq!(d.rd(0x11), 0x60);
        d.start();
        assert_eq!(d.rd(0x01), 0x34);
        assert_eq!(mic_on(d.c), "1");
        d.stop();
        assert_eq!(mic_on(d.c), "0");
        d.start();
        assert_eq!(mic_on(d.c), "1", "starts again from the kept 01 value");
    }

    #[test]
    fn esp_codec_dev_mic1_3_and_mic1_4_use_tdm() {
        // StackChan (xiaozhi) selects MIC1-3; tr_audio.c selects MIC1-4.
        for mics in [MIC1 | MIC2 | MIC3, MIC1 | MIC2 | MIC3 | MIC4] {
            let mut c = Es7210::default();
            let mut d = Driver { c: &mut c, mics, gain: 0, off_reg: 0 };
            d.open();
            assert_eq!(d.off_reg, 0x20, "all ADC clocks on");
            assert_eq!([d.rd(0x4B), d.rd(0x4C), d.rd(0x12)], [0x00, 0x00, 0x02]);
            assert_eq!(d.rd(0x45), 0x1A);
            d.start();
            assert_eq!(mic_on(d.c), "1");
            d.stop();
            assert_eq!(mic_on(d.c), "0");
        }
    }

    #[test]
    fn mic_on_goes_off_while_the_adcs_are_held_in_reset() {
        let mut c = Es7210::default();
        let mut d = Driver { c: &mut c, mics: MIC1 | MIC2, gain: 0, off_reg: 0 };
        d.open();
        d.start();
        d.wr(0x00, 0x71); // RST_ADC34 and RST_ADC12
        assert_eq!(mic_on(d.c), "0");
        d.wr(0x00, 0x41);
        assert_eq!(mic_on(d.c), "1");
    }

    #[test]
    fn m5unified_write_only_table_then_off() {
        let mut c = Es7210::default();
        c.transfer(&[0x40, 0x00], 0, false, 0); // left over: RST_REGS must clear it
        let table: [(u8, u8); 29] = [
            (0x00, 0x41),
            (0x01, 0x1f),
            (0x06, 0x00),
            (0x07, 0x20),
            (0x08, 0x10),
            (0x09, 0x30),
            (0x0A, 0x30),
            (0x20, 0x0a),
            (0x21, 0x2a),
            (0x22, 0x0a),
            (0x23, 0x2a),
            (0x02, 0xC1),
            (0x04, 0x01),
            (0x05, 0x00),
            (0x11, 0x60),
            (0x40, 0x42),
            (0x41, 0x70),
            (0x42, 0x70),
            (0x43, 0x1B),
            (0x44, 0x1B),
            (0x45, 0x00),
            (0x46, 0x00),
            (0x47, 0x00),
            (0x48, 0x00),
            (0x49, 0x00),
            (0x4A, 0x00),
            (0x4B, 0x00),
            (0x4C, 0xFF),
            (0x01, 0x14),
        ];
        // _microphone_enabled_cb_cores3(true): 00←FF, then the table.
        c.transfer(&[0x00, 0xFF], 0, false, 0);
        assert_eq!(c.transfer(&[0x40], 1, false, 0), Some(vec![0x80]), "registers reset");
        for (r, v) in table {
            c.transfer(&[r, v], 0, false, 0);
            if r == 0x00 {
                assert_eq!(mic_on(&c), "0", "not yet: analog still powered down");
            }
        }
        assert_eq!(mic_on(&c), "1");
        for (r, v) in table.into_iter().filter(|&e| e != (0x01, 0x1f)) {
            assert_eq!(c.transfer(&[r], 1, false, 0), Some(vec![v]), "register {r:#04x}");
        }
        // _microphone_enabled_cb_cores3(false): 00←FF only.
        c.transfer(&[0x00, 0xFF], 0, false, 0);
        assert_eq!(mic_on(&c), "0");
        assert_eq!(c.transfer(&[0x00], 2, false, 0), Some(vec![0xFF, 0x20]));
    }

    #[test]
    fn high_addresses_are_harmless_storage_and_the_pointer_moves_on() {
        let mut c = Es7210::default();
        c.transfer(&[0xFE, 0x12, 0x34, 0x50], 0, false, 0); // FE, FF, then 00
        assert_eq!(c.transfer(&[0xFE], 2, false, 0), Some(vec![0x12, 0x34]));
        assert_eq!(c.transfer(&[0x00], 1, false, 0), Some(vec![0x50]));
        assert_eq!(c.transfer(&[0x41], 1, false, 0), Some(vec![0x71]), "0x50 has no RST_REGS bit");
    }

    #[test]
    fn split_read_goes_on_from_the_pointer() {
        let mut c = Es7210::default();
        assert_eq!(c.transfer(&[0x3C], 2, false, 0), Some(vec![0x00, 0x72]));
        assert_eq!(c.transfer(&[], 2, true, 1), Some(vec![0x10, 0x00]));
    }

    #[test]
    fn probe_acks() {
        let mut c = Es7210::default();
        assert_eq!(c.transfer(&[], 0, false, 0), Some(vec![]));
    }
}
