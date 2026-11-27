//! The Si12T head touch sensor in StackChan's body, at I2C address 0x68.
//!
//! What the two drivers do with it (StackChan firmware `drivers/Si12T` and
//! `hal_head_touch.cpp`; tr_board `tr_sensors.c`):
//!
//! - They write byte registers: 0x0A-0x0F (channels on), 0x09 (0x0F then
//!   0x07), 0x08 (0x22), 0x02-0x06 (sensitivity, 0x33). StackChan reads
//!   0x0A-0x0F, 0x09, 0x08 and 0x02-0x06 back, one byte at a time, and checks
//!   nothing.
//! - They poll register 0x10 (OUTPUT1), one byte: three zones of two bits,
//!   bits 1:0, 3:2 and 5:4, each 0 (none) to 3 (strong). StackChan reads it
//!   every 50 ms.
//!
//! Which zone is which: M5Stack's StackChan Arduino library gives index 0 =
//! front, 1 = middle, 2 = back, and "swiped forward" = front to back. The
//! official firmware agrees: it weighs zone 0 at -100 and zone 2 at +100, and
//! calls a rise of that value SwipeForward. tr_board does not say.
//!
//! The model:
//!
//! - 0x02-0x0F store what is written and read it back. No reset value is known
//!   from code, so they start at 0x00. What the settings do (sensitivity,
//!   sleep, the soft reset bit in 0x09) is not modelled.
//! - 0x10 reads the three zones from the inputs, bits 7:6 = 0 (channel 4, not
//!   wired on StackChan). Writes to it are ignored.
//! - 0x11 and 0x12 (OUTPUT2 and OUTPUT3 in the driver, for channels that are
//!   not wired) read 0x00 and ignore writes. Inferred from the names, not
//!   confirmed: no driver touches them.
//! - Every other address reads 0x00 and ignores writes. This matters: M5Unified
//!   probes an MPU6886 at 0x68 (it reads 0x75 and wants 0x19, 0x68 or 0x71)
//!   and a BMI270 (it reads 0x00 and wants 0x24). With 0x00 at both it finds no
//!   IMU here and goes on to the BMI270 at 0x69.
//! - The register pointer moves on after each byte, so a longer read or write
//!   goes on to the next register (unconfirmed: the drivers only move one byte
//!   at a time). It persists between transactions.
//! - It always answers (ACK), a probe too.

pub const ADDR: u32 = 0x68;

const FIRST_RW: u8 = 0x02;
const LAST_RW: u8 = 0x0F;
pub const OUTPUT1: u8 = 0x10;

/// The zones, in the order of their bits in OUTPUT1.
pub const ZONES: [&str; 3] = ["front", "middle", "back"];

#[derive(Default)]
pub struct Si12t {
    /// Registers 0x00-0x0F; only 0x02-0x0F are stored.
    regs: [u8; 16],
    at: u8,
    /// Touch intensity of each zone, 0..3, in `ZONES` order.
    pub zones: [u8; 3],
}

impl Si12t {
    fn output1(&self) -> u8 {
        self.zones.iter().enumerate().fold(0, |v, (i, &z)| v | (z & 3) << (2 * i))
    }

    fn read_reg(&self, reg: u8) -> u8 {
        match reg {
            FIRST_RW..=LAST_RW => self.regs[reg as usize],
            OUTPUT1 => self.output1(),
            _ => 0x00,
        }
    }

    /// One I2C transaction. The first written byte sets the pointer, the rest
    /// are stored from there on. A read goes on from the pointer, which is
    /// also how a split read (`continues`, no write bytes) carries on.
    pub fn transfer(&mut self, write: &[u8], read_len: usize) -> Vec<u8> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg;
            for &b in data {
                if (FIRST_RW..=LAST_RW).contains(&self.at) {
                    self.regs[self.at as usize] = b;
                }
                self.at = self.at.wrapping_add(1);
            }
        }
        (0..read_len)
            .map(|_| {
                let v = self.read_reg(self.at);
                self.at = self.at.wrapping_add(1);
                v
            })
            .collect()
    }

    /// `head_<zone>` = 0..3. Returns false if the name is not a zone.
    pub fn input(&mut self, zone: &str, value: f64) -> bool {
        let Some(i) = ZONES.iter().position(|&z| z == zone) else { return false };
        self.zones[i] = value.round().clamp(0.0, 3.0) as u8;
        true
    }

    /// The three intensities, front first: "0,3,0".
    pub fn head(&self) -> String {
        let [f, m, b] = self.zones;
        format!("{f},{m},{b}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// StackChan's si12t_i2c_write_reg: one transmit of [reg, value].
    fn wr(s: &mut Si12t, reg: u8, v: u8) {
        assert_eq!(s.transfer(&[reg, v], 0), []);
    }

    /// StackChan's si12t_i2c_read_reg and tr_board's rd(): transmit_receive
    /// of [reg], one byte.
    fn rd(s: &mut Si12t, reg: u8) -> u8 {
        let v = s.transfer(&[reg], 1);
        assert_eq!(v.len(), 1);
        v[0]
    }

    #[test]
    fn stackchan_setup_reads_back_what_it_wrote() {
        let mut s = Si12t::default();
        // si12t_setup(LOW, LEVEL_3), in its order (Si12T.cpp).
        // si12t_enable_channel: write 0A-0F = 00, then read each back.
        for reg in 0x0A..=0x0F {
            wr(&mut s, reg, 0x00);
        }
        for reg in 0x0A..=0x0F {
            assert_eq!(rd(&mut s, reg), 0x00);
        }
        // si12t_set_ctrl2: 0F then 07, read back.
        wr(&mut s, 0x09, 0x0F);
        wr(&mut s, 0x09, 0x07);
        assert_eq!(rd(&mut s, 0x09), 0x07);
        // si12t_set_ctrl1: 22, read back.
        wr(&mut s, 0x08, 0x22);
        assert_eq!(rd(&mut s, 0x08), 0x22);
        // si12t_set_sensitivity: 02-06 = 0x33; si12t_get_sensitivity reads them.
        for reg in 0x02..=0x06 {
            wr(&mut s, reg, 0x33);
        }
        for reg in 0x02..=0x06 {
            assert_eq!(rd(&mut s, reg), 0x33);
        }
        assert_eq!(rd(&mut s, 0x07), 0x00, "SENSITIVITY6 is not written");
    }

    #[test]
    fn stackchan_poll_every_50_ms_sees_the_zones() {
        let mut s = Si12t::default();
        // _head_touch_update_task: read OUTPUT1, parse three 2-bit zones.
        let parse = |v: u8| [v & 3, v >> 2 & 3, v >> 4 & 3];
        assert_eq!(rd(&mut s, OUTPUT1), 0x00, "no touch");
        assert!(s.input("front", 1.0));
        assert_eq!(parse(rd(&mut s, OUTPUT1)), [1, 0, 0]);
        // A hand from front to back: StackChan's position goes -100 to +100.
        s.input("front", 0.0);
        s.input("middle", 2.0);
        s.input("back", 3.0);
        let v = rd(&mut s, OUTPUT1);
        assert_eq!(parse(v), [0, 2, 3]);
        assert_eq!(v & 0xC0, 0, "bits 7:6 stay 0");
        assert_eq!(s.head(), "0,2,3");
        // The same read again, 50 ms later: no write needed between polls.
        assert_eq!(rd(&mut s, OUTPUT1), v);
    }

    #[test]
    fn tr_board_setup_and_poll() {
        let mut s = Si12t::default();
        // head_touch_init(): the same writes, no readbacks.
        for reg in 0x0A..=0x0F {
            wr(&mut s, reg, 0x00);
        }
        wr(&mut s, 0x09, 0x0F);
        wr(&mut s, 0x09, 0x07);
        wr(&mut s, 0x08, 0x22);
        for reg in 0x02..=0x06 {
            wr(&mut s, reg, 0x33);
        }
        // tr_head_touch(): reads 0x10 and keeps v & 0x3F.
        s.input("middle", 3.0);
        assert_eq!(rd(&mut s, OUTPUT1) & 0x3F, 3 << 2);
    }

    #[test]
    fn outputs_are_read_only_and_inputs_are_clamped() {
        let mut s = Si12t::default();
        s.input("back", 2.0);
        wr(&mut s, OUTPUT1, 0xFF);
        wr(&mut s, 0x11, 0xFF);
        wr(&mut s, 0x12, 0xFF);
        assert_eq!(rd(&mut s, OUTPUT1), 2 << 4);
        assert_eq!(rd(&mut s, 0x11), 0);
        assert_eq!(rd(&mut s, 0x12), 0);
        s.input("front", 9.0);
        s.input("middle", -4.0);
        assert_eq!(s.head(), "3,0,2");
        assert!(!s.input("left", 1.0), "not a zone");
    }

    #[test]
    fn m5unified_imu_probes_find_nothing() {
        let mut s = Si12t::default();
        s.input("front", 3.0);
        // MPU6886_Class::begin: readRegister8(0x75), wants 0x19, 0x68 or 0x71.
        assert_eq!(rd(&mut s, 0x75), 0x00);
        // BMI270_Class::begin at 0x68 (only when 0x69 fails): WhoAmI() twice, reg 0x00, wants 0x24.
        assert_eq!(rd(&mut s, 0x00), 0x00);
        assert_eq!(rd(&mut s, 0x00), 0x00);
        // Writes outside 02-0F change nothing.
        wr(&mut s, 0x75, 0x19);
        wr(&mut s, 0x00, 0x24);
        assert_eq!(rd(&mut s, 0x75), 0x00);
        assert_eq!(rd(&mut s, 0x00), 0x00);
    }

    #[test]
    fn pointer_persists_and_moves_on() {
        let mut s = Si12t::default();
        s.input("front", 1.0);
        wr(&mut s, 0x08, 0x22);
        // Set the pointer, then read in a later transaction (as a split read
        // carries on): 0x0E, 0x0F, then OUTPUT1.
        assert_eq!(s.transfer(&[0x0E], 0), []);
        assert_eq!(s.transfer(&[], 2), [0x00, 0x00]);
        assert_eq!(s.transfer(&[], 1), [0x01]);
        // A multi-byte write stores from the pointer on.
        s.transfer(&[0x02, 0x11, 0x22, 0x33], 0);
        assert_eq!(s.transfer(&[0x02], 3), [0x11, 0x22, 0x33]);
        assert_eq!(rd(&mut s, 0x08), 0x22);
    }
}
