//! GC0308 camera, control interface only (SCCB), address 0x21.
//!
//! What is modelled:
//! - SCCB: 8-bit register address, 8-bit value. The drivers only use
//!   single-register access; the pointer moves on after each byte anyway
//!   and stays between transactions, so esp32-camera's fallback read (write
//!   the address, stop, then read) works.
//! - 0xFE selects the page with bits 1:0 and is the same register on every
//!   page. Writing it with bit 7 set is a soft reset: every page goes back
//!   to its reset values, and 0xFE reads 0x00 after it (bit 7 clears
//!   itself).
//! - Page 0 register 0x00 is the chip ID, 0x9B, read only.
//! - Every other register on every page is storage that reads back what was
//!   written, because the drivers do read-modify-write. Reset values other
//!   than the ID are not modelled (they read 0x00); the drivers write their
//!   own tables after a reset.
//!
//! What each driver needs:
//! - M5GFX/M5Unified board detection (`esp32s3_cores3.inl` camera_id):
//!   reads 0x00 with no page select and needs 0x9B. This decides CoreS3
//!   against CoreS3SE, and only a board with a camera can be a StackChan.
//!   After power on (or a soft reset) the page is 0, so the ID is there.
//! - tr_board (esp32-camera `gc0308.c`): detect = `FE<-00`, read 0x00, needs
//!   0x9B. Reset = `FE<-F0`, wait 80 ms, the default table (with `FE` page
//!   writes), `FE<-00`. Then read-modify-writes after page selects.
//! - StackChan (esp_cam_sensor `gc0308.c`): detect = `FE<-00`, read 0x00,
//!   needs 0x9B. Soft reset = `FE<-00`, read `FE`, write it back with bit 7
//!   set. Format tables with `FE` page writes.
//!
//! Not modelled: pixels. They travel on the DVP bus, which a wasm part
//! cannot reach. The chip always answers here. The real one needs power
//! first: AXP2101 ALDO3 and the camera enable on AW9523 P1_0.

use esp_part::State;

use crate::chip::Chip;

const CHIP_ID: u8 = 0x00;
const PAGE_SELECT: u8 = 0xFE;
/// FE bit 7: soft reset.
const SOFT_RESET: u8 = 0x80;
/// FE bits 1:0: the page.
const PAGE_MASK: u8 = 0x03;
/// GC0308_PID in esp32-camera and esp_cam_sensor, and M5GFX's id_value.
const GC0308_PID: u8 = 0x9B;

pub struct Gc0308 {
    /// Four pages of 256 registers (FE bits 1:0 can select four; the
    /// drivers use pages 0 and 1). `pages[p][0xFE]` is unused: FE is `fe`.
    pages: [[u8; 256]; 4],
    fe: u8,
    at: u8,
    /// How many times the firmware read the chip ID.
    id_reads: u32,
}

impl Default for Gc0308 {
    fn default() -> Self {
        Self { pages: [[0; 256]; 4], fe: 0, at: 0, id_reads: 0 }
    }
}

impl Gc0308 {
    fn page(&self) -> usize {
        (self.fe & PAGE_MASK) as usize
    }

    fn soft_reset(&mut self) {
        *self = Self { id_reads: self.id_reads, ..Self::default() };
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        match reg {
            PAGE_SELECT if value & SOFT_RESET != 0 => self.soft_reset(),
            PAGE_SELECT => self.fe = value,
            CHIP_ID if self.page() == 0 => {} // read only
            _ => {
                let page = self.page();
                self.pages[page][reg as usize] = value;
            }
        }
    }

    fn read_reg(&mut self, reg: u8) -> u8 {
        match reg {
            PAGE_SELECT => self.fe,
            CHIP_ID if self.page() == 0 => {
                self.id_reads += 1;
                GC0308_PID
            }
            _ => self.pages[self.page()][reg as usize],
        }
    }
}

impl Chip for Gc0308 {
    fn transfer(&mut self, write: &[u8], read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg;
            for &b in data {
                let reg = self.at;
                self.write_reg(reg, b);
                self.at = reg.wrapping_add(1);
            }
        }
        Some(
            (0..read_len)
                .map(|_| {
                    let reg = self.at;
                    self.at = reg.wrapping_add(1);
                    self.read_reg(reg)
                })
                .collect(),
        )
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state.set("camera_id_reads", self.id_reads)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(c: &mut Gc0308, write: &[u8], read_len: usize) -> Vec<u8> {
        c.transfer(write, read_len, false, 0).unwrap()
    }

    /// set_reg_bits in esp32-camera and esp_cam_sensor: read, change the
    /// bits, write back.
    fn set_bits(c: &mut Gc0308, reg: u8, offset: u8, mask: u8, value: u8) {
        let old = t(c, &[reg], 1)[0];
        let new = (old & !(mask << offset)) | ((value & mask) << offset);
        t(c, &[reg, new], 0);
    }

    #[test]
    fn probe_acks() {
        let mut c = Gc0308::default();
        assert_eq!(c.transfer(&[], 0, false, 0), Some(vec![]));
    }

    #[test]
    fn m5gfx_detect_reads_the_id_with_no_page_select() {
        let mut c = Gc0308::default();
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x9B]);
        // And again in refine, after power on.
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x9B]);
        assert_eq!(c.state(State::new(), 0).get("camera_id_reads"), Some("2"));
    }

    #[test]
    fn esp32_camera_detect_and_fallback_read() {
        let mut c = Gc0308::default();
        t(&mut c, &[PAGE_SELECT, 0x00], 0);
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x9B], "repeated-start read");
        // The fallback: write the address, stop, then a plain read.
        t(&mut c, &[CHIP_ID], 0);
        assert_eq!(t(&mut c, &[], 1), [0x9B]);
    }

    #[test]
    fn esp_cam_sensor_detect_and_soft_reset() {
        let mut c = Gc0308::default();
        t(&mut c, &[PAGE_SELECT, 0x00], 0);
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x9B]);
        t(&mut c, &[0x10, 0x55], 0);
        t(&mut c, &[PAGE_SELECT, 0x00], 0);
        set_bits(&mut c, PAGE_SELECT, 7, 0x01, 1); // gc0308_soft_reset
        assert_eq!(t(&mut c, &[PAGE_SELECT], 1), [0x00], "bit 7 cleared itself");
        assert_eq!(t(&mut c, &[0x10], 1), [0x00], "registers back to reset");
    }

    #[test]
    fn esp32_camera_reset_clears_every_page() {
        let mut c = Gc0308::default();
        t(&mut c, &[PAGE_SELECT, 0x01], 0);
        t(&mut c, &[0x53, 0x82], 0);
        t(&mut c, &[PAGE_SELECT, 0xF0], 0); // reset(): FE <- F0
        assert_eq!(t(&mut c, &[PAGE_SELECT], 1), [0x00], "page 0 after reset");
        t(&mut c, &[PAGE_SELECT, 0x01], 0);
        assert_eq!(t(&mut c, &[0x53], 1), [0x00]);
    }

    #[test]
    fn pages_hold_their_own_registers() {
        let mut c = Gc0308::default();
        t(&mut c, &[PAGE_SELECT, 0x00], 0);
        t(&mut c, &[0x24, 0xA6], 0); // OUTPUT_FMT on page 0
        t(&mut c, &[PAGE_SELECT, 0x01], 0);
        t(&mut c, &[0x24, 0x11], 0);
        set_bits(&mut c, 0x53, 7, 0x01, 1); // SUBSAMPLE_EN on page 1
        assert_eq!(t(&mut c, &[PAGE_SELECT], 1), [0x01], "FE reads back on any page");
        assert_eq!(t(&mut c, &[0x53], 1), [0x80]);
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x00], "the ID is a page 0 register");
        t(&mut c, &[PAGE_SELECT, 0x00], 0);
        assert_eq!(t(&mut c, &[0x24], 1), [0xA6]);
        set_bits(&mut c, 0x24, 0, 0x0F, 6); // set_pixformat RGB565
        assert_eq!(t(&mut c, &[0x24], 1), [0xA6]);
        set_bits(&mut c, 0x24, 0, 0x0F, 2);
        assert_eq!(t(&mut c, &[0x24], 1), [0xA2]);
    }

    #[test]
    fn chip_id_is_read_only() {
        let mut c = Gc0308::default();
        t(&mut c, &[CHIP_ID, 0x00], 0);
        assert_eq!(t(&mut c, &[CHIP_ID], 1), [0x9B]);
    }

    #[test]
    fn id_reads_survive_a_soft_reset() {
        let mut c = Gc0308::default();
        t(&mut c, &[CHIP_ID], 1);
        t(&mut c, &[PAGE_SELECT, SOFT_RESET], 0);
        t(&mut c, &[CHIP_ID], 1);
        assert_eq!(c.state(State::new(), 0).get("camera_id_reads"), Some("2"));
    }
}
