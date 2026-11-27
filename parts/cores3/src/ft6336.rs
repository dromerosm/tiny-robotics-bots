//! FT6336U capacitive touch, address 0x38.
//!
//! What is modelled:
//! - Byte registers with a pointer that moves on after each byte, on reads
//!   and on writes. The pointer stays between transactions, so a read that
//!   goes on with no register address (M5GFX, see below) continues from
//!   where it stopped.
//! - 0x00 DEVICE_MODE: read/write storage, reset 0x00 (working mode). Other
//!   modes (factory mode) are not modelled: the touch data stays the same.
//! - 0x01 GEST_ID (always 0x00), 0x02 TD_STATUS (touch count, 0 or 1),
//!   0x03..0x08 point 1: XH (bits 7:6 event, bits 3:0 X[11:8]), XL,
//!   YH (bits 7:4 touch ID, bits 3:0 Y[11:8]), YL, weight, misc.
//!   0x09..0x0E point 2: always empty. Writes to 0x01..0x0E are ignored.
//! - 0xA3 CIPHER, 0xA6 FIRMID, 0xA8 FOCALTECH_ID: read only, see the
//!   constants. 0xA4 G_MODE and 0xA5 PWR_MODE: read/write.
//! - Every other register is read/write storage that reads back.
//!
//! Touch data is latched at the start of a transaction that writes a
//! register address, and only then. So the bytes of one read always belong
//! together, and two reads in a row with no change on the page return the
//! same bytes (M5GFX reads twice and compares).
//!
//! What each driver needs:
//! - tr_board (`tr_lcd.c` screen_is_9342e): `00<-00`, then 1 byte from 0xA6,
//!   up to 5 times. 0x12 means ILI9342E, anything else ILI9342C.
//!   `tr_sensors.c` tr_touch_read: 5 bytes from 0x02, and `A4<-00` after the
//!   first good read. A touch is TD_STATUS & 0x0F in 1..2.
//! - StackChan (`stackchan.cc` Ft6336): `00<-00`, then 6 bytes from 0xA3.
//!   Valid if A3 != 0 and A8 != 0; ILI9342E only if A6 == 0x12 and
//!   A8 == 0x11. Then 6 bytes from 0x02 every 20 ms; LVGL sees "pressed"
//!   while TD_STATUS & 0x0F is not 0. It never reads the INT line.
//! - M5GFX/M5Unified (`Touch_FT5x06.inl`): `00<-00`, 6 bytes from 0xA3,
//!   `A4<-00`, needs A8 != 0. Data: one transaction writes 0x02, reads
//!   TD_STATUS, then goes on reading `points*6-2` bytes in the same
//!   transaction (here a second piece with `continues` and no write). It
//!   reads twice and compares. The CoreS3 board reads only while GPIO 21 is
//!   low: `touched()` drives AW9523 P1_2, which drives GPIO 21 (lib.rs
//!   `sync_touch`). `sleep()` writes `A5<-03`, `wakeup()` writes `A5<-01`.
//!
//! The release, as the bus shows it: TD_STATUS goes straight from 1 to 0 at
//! the first latch after the finger lifts. Point 1 keeps the last
//! coordinates with event 1 (lift up). All three drivers decide "touched"
//! from the count in TD_STATUS alone and never read the event bits, so a
//! frame with TD_STATUS 1 and event 1 would make each of them report one
//! more press at the lift point. A count of 0 ends the touch in every
//! driver: tr_board returns false, StackChan sets LVGL to released, M5GFX
//! returns 0 points and then clears the AW9523 interrupt.
//!
//! The press: the first frames after the finger lands show event 0 (press
//! down) for one report period, then event 2 (contact).
//!
//! Not modelled: the reset line (AW9523 P0_0; the real chip does not answer
//! while it is held low), trigger mode on INT (G_MODE 1 gives pulses on the
//! real chip; here INT is a level, low while touched), gestures, the second
//! point. In hibernate (PWR_MODE 0x03) the chip stops scanning: no touch is
//! reported and INT stays high. The real chip also stops answering on I2C
//! until INT is pulsed; this one keeps answering, because M5GFX wakes it on
//! the CoreS3 with an I2C write (`A5<-01`) and cannot pulse the touch INT.

use crate::chip::{Chip, Regs8};
use esp_part::State;

const DEVICE_MODE: u8 = 0x00;
const TD_STATUS: u8 = 0x02;
const P1_XH: u8 = 0x03;
/// First and last register of the touch data (GEST_ID to P2_MISC).
const DATA_FIRST: u8 = 0x01;
const DATA_LAST: u8 = 0x0E;
const CIPHER: u8 = 0xA3;
const G_MODE: u8 = 0xA4;
const PWR_MODE: u8 = 0xA5;
const FIRMID: u8 = 0xA6;
const FOCALTECH_ID: u8 = 0xA8;

/// Chip ID of the FT6336U. Datasheet, unconfirmed on our units. The drivers
/// only need it to be nonzero (StackChan checks it).
const CIPHER_FT6336U: u8 = 0x64;
/// Firmware ID. On the CoreS3 it tells which screen panel the board has:
/// 0x10 for the ILI9342C, 0x12 for the ILI9342E (units from August 2026).
/// tr_board and StackChan pick the panel init from it (StackChan also needs
/// FOCALTECH_ID 0x11 for the E). The emulator draws an ILI9342C panel, so
/// we answer 0x10. Change both together if the emulated panel changes.
const FIRMID_ILI9342C: u8 = 0x10;
/// Panel maker ID, 0x11 as StackChan expects (`stackchan.cc`).
const FOCALTECH_ID_VALUE: u8 = 0x11;
/// PWR_MODE value for hibernate (M5GFX `sleep()`).
const HIBERNATE: u8 = 0x03;

/// The event in bits 7:6 of P1_XH.
const EVENT_DOWN: u8 = 0;
const EVENT_UP: u8 = 1;
const EVENT_CONTACT: u8 = 2;

/// How long the first contact shows event 0 (press down): one report
/// period. Unconfirmed: the datasheet gives the active period register
/// (0x88) but not the rate on the CoreS3; 10 ms is a typical 100 Hz.
const REPORT_PERIOD_NS: u64 = 10_000_000;

/// Screen size in touch pixels, from M5GFX's CoreS3 touch config
/// (`x_max` 319, `y_max` 239, no rotation).
pub const WIDTH: u16 = 320;
pub const HEIGHT: u16 = 240;

pub struct Ft6336 {
    regs: Regs8,
    /// The finger, as the page sets it.
    down: bool,
    x: u16,
    y: u16,
    /// The touch data latched at the last register address write,
    /// registers 0x01..0x0E.
    data: [u8; 14],
    /// When the first latch saw the current touch, if a touch is on.
    down_at: Option<u64>,
}

fn reset_regs() -> Regs8 {
    Regs8::new(
        &[
            (DEVICE_MODE, 0x00), // working mode
            (G_MODE, 0x00),      // polling; datasheet, unconfirmed
            (CIPHER, CIPHER_FT6336U),
            (FIRMID, FIRMID_ILI9342C),
            (FOCALTECH_ID, FOCALTECH_ID_VALUE),
        ],
        &[CIPHER, FIRMID, FOCALTECH_ID],
    )
}

impl Default for Ft6336 {
    fn default() -> Self {
        // No touch since power on: count 0, both points empty (0xFF, as
        // FocalTech chips show unused points; unconfirmed on the FT6336U).
        let mut data = [0xFF; 14];
        data[0] = 0x00; // GEST_ID
        data[1] = 0x00; // TD_STATUS
        Self { regs: reset_regs(), down: false, x: 0, y: 0, data, down_at: None }
    }
}

impl Ft6336 {
    /// Whether a finger is on the screen now and the chip is scanning. Its
    /// INT line is low then (polling mode); lib.rs puts it on AW9523 P1_2.
    pub fn touched(&self) -> bool {
        self.down && self.regs.mem[PWR_MODE as usize] != HIBERNATE
    }

    /// Take a new frame of touch data from the finger.
    fn latch(&mut self, now_ns: u64) {
        let td = (TD_STATUS - DATA_FIRST) as usize;
        let p1 = (P1_XH - DATA_FIRST) as usize;
        if self.touched() {
            let since = *self.down_at.get_or_insert(now_ns);
            let event = if now_ns.saturating_sub(since) < REPORT_PERIOD_NS { EVENT_DOWN } else { EVENT_CONTACT };
            self.data[td] = 1;
            self.data[p1] = event << 6 | (self.x >> 8) as u8;
            self.data[p1 + 1] = self.x as u8;
            self.data[p1 + 2] = (self.y >> 8) as u8; // touch ID 0
            self.data[p1 + 3] = self.y as u8;
            self.data[p1 + 4] = 0x00; // weight: not modelled
            self.data[p1 + 5] = 0x00; // misc (area): not modelled
        } else {
            self.data[td] = 0;
            if self.down_at.take().is_some() {
                // The lift: keep the last point, marked as lift up.
                self.data[p1] = EVENT_UP << 6 | (self.data[p1] & 0x0F);
            }
        }
    }
}

/// A pixel from the page, clamped to the screen.
fn pixel(value: f64, size: u16) -> u16 {
    value.round().clamp(0.0, f64::from(size - 1)) as u16
}

impl Chip for Ft6336 {
    fn transfer(&mut self, write: &[u8], read_len: usize, _continues: bool, now_ns: u64) -> Option<Vec<u8>> {
        if let Some((&reg, _)) = write.split_first() {
            self.latch(now_ns);
            self.regs.at = reg;
            for &b in &write[1..] {
                let r = self.regs.at;
                if !(DATA_FIRST..=DATA_LAST).contains(&r) {
                    self.regs.write(&[r, b]);
                } else {
                    self.regs.at = r.wrapping_add(1);
                }
            }
        }
        let data = self.data;
        Some(self.regs.read(read_len, |m, r| match r {
            DATA_FIRST..=DATA_LAST => data[(r - DATA_FIRST) as usize],
            _ => m[r as usize],
        }))
    }

    fn input(&mut self, name: &str, value: f64) -> bool {
        if !value.is_finite() {
            return matches!(name, "touch" | "touch_x" | "touch_y");
        }
        match name {
            "touch" => self.down = value >= 0.5,
            "touch_x" => self.x = pixel(value, WIDTH),
            "touch_y" => self.y = pixel(value, HEIGHT),
            _ => return false,
        }
        true
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state.set("touch", self.down as u8).set("touch_x", self.x).set("touch_y", self.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn t(c: &mut Ft6336, write: &[u8], read_len: usize, now: u64) -> Vec<u8> {
        c.transfer(write, read_len, false, now).unwrap()
    }

    /// M5GFX `_read_data`: write 0x02, read TD_STATUS, and if there are
    /// points keep reading `points*6-2` bytes in the same transaction.
    fn m5gfx_read(c: &mut Ft6336, now: u64) -> Vec<u8> {
        let mut out = c.transfer(&[TD_STATUS], 1, true, now).unwrap();
        let points = (out[0] & 0x0F).min(5) as usize;
        if points > 0 {
            out.extend(c.transfer(&[], points * 6 - 2, false, now).unwrap());
        }
        out
    }

    fn finger(c: &mut Ft6336, x: f64, y: f64) {
        assert!(c.input("touch_x", x));
        assert!(c.input("touch_y", y));
        assert!(c.input("touch", 1.0));
    }

    #[test]
    fn probe_acks() {
        let mut c = Ft6336::default();
        assert_eq!(c.transfer(&[], 0, false, 0), Some(vec![]));
    }

    #[test]
    fn tr_board_panel_id_says_ili9342c() {
        let mut c = Ft6336::default();
        t(&mut c, &[DEVICE_MODE, 0x00], 0, 0);
        assert_eq!(t(&mut c, &[FIRMID], 1, 0), [0x10], "0x10 stops the 5 tries at once");
    }

    #[test]
    fn stackchan_version_read() {
        let mut c = Ft6336::default();
        t(&mut c, &[DEVICE_MODE, 0x00], 0, 0);
        let info = t(&mut c, &[CIPHER], 6, 0);
        assert_eq!(info, [0x64, 0x00, 0x00, 0x10, 0x00, 0x11]);
        assert!(info[0] != 0 && info[5] != 0, "valid");
        assert!(!(info[3] == 0x12 && info[5] == 0x11), "not ILI9342E: ILI9342C");
    }

    #[test]
    fn m5gfx_check_init() {
        let mut c = Ft6336::default();
        t(&mut c, &[DEVICE_MODE, 0x00], 0, 0);
        let tmp = t(&mut c, &[CIPHER], 6, 0);
        t(&mut c, &[G_MODE, 0x00], 0, 0);
        assert_ne!(tmp[5], 0, "VENDID must be nonzero");
        assert_eq!(t(&mut c, &[G_MODE], 1, 0), [0x00]);
    }

    #[test]
    fn id_registers_are_read_only_and_modes_read_back() {
        let mut c = Ft6336::default();
        t(&mut c, &[CIPHER, 0x00], 0, 0);
        t(&mut c, &[FIRMID, 0x12], 0, 0);
        t(&mut c, &[FOCALTECH_ID, 0x00], 0, 0);
        t(&mut c, &[G_MODE, 0x01], 0, 0);
        t(&mut c, &[PWR_MODE, 0x01], 0, 0);
        t(&mut c, &[DEVICE_MODE, 0x40], 0, 0);
        assert_eq!(t(&mut c, &[CIPHER], 6, 0), [0x64, 0x01, 0x01, 0x10, 0x00, 0x11]);
        assert_eq!(t(&mut c, &[DEVICE_MODE], 1, 0), [0x40]);
        t(&mut c, &[TD_STATUS, 0x05], 0, 0);
        assert_eq!(t(&mut c, &[TD_STATUS], 1, 0), [0x00], "touch data cannot be written");
    }

    #[test]
    fn no_touch_reads_empty() {
        let mut c = Ft6336::default();
        assert!(!c.touched());
        assert_eq!(t(&mut c, &[TD_STATUS], 5, 0)[0], 0x00);
        assert_eq!(m5gfx_read(&mut c, 0), [0x00], "M5GFX gets 1 byte and stops");
    }

    #[test]
    fn tr_board_poll_down_move_up() {
        let mut c = Ft6336::default();
        finger(&mut c, 200.0, 100.0);
        let b = t(&mut c, &[TD_STATUS], 5, 0);
        t(&mut c, &[G_MODE, 0x00], 0, 0); // the first good read
        assert_eq!(b, [0x01, 0x00, 200, 0x00, 100], "event 0 (down), X 200, Y 100");
        finger(&mut c, 300.0, 230.0);
        let b = t(&mut c, &[TD_STATUS], 5, 20 * MS);
        assert_eq!(b, [0x01, 0x81, 44, 0x00, 230], "event 2 (contact), X 300");
        assert_eq!((b[1] as u16 & 0x0F) << 8 | b[2] as u16, 300);
        c.input("touch", 0.0);
        let b = t(&mut c, &[TD_STATUS], 5, 40 * MS);
        assert_eq!(b[0] & 0x0F, 0, "released: count 0 at once");
        assert_eq!(b[1] >> 6, EVENT_UP, "the last point is marked lift up");
    }

    #[test]
    fn stackchan_poll_down_move_up() {
        let mut c = Ft6336::default();
        finger(&mut c, 10.0, 20.0);
        let b = t(&mut c, &[TD_STATUS], 6, 0);
        assert_eq!(b, [0x01, 0x00, 10, 0x00, 20, 0x00]);
        finger(&mut c, 11.0, 21.0);
        let b = t(&mut c, &[TD_STATUS], 6, 20 * MS);
        assert_eq!((b[0] & 0x0F, b[2], b[4]), (1, 11, 21));
        c.input("touch", 0.0);
        let b = t(&mut c, &[TD_STATUS], 6, 40 * MS);
        assert_eq!(b[0] & 0x0F, 0, "LVGL released");
    }

    #[test]
    fn m5gfx_split_read_twice_equal() {
        let mut c = Ft6336::default();
        finger(&mut c, 160.0, 120.0);
        assert!(c.touched(), "INT low: M5GFX reads");
        let a = m5gfx_read(&mut c, 20 * MS);
        let b = m5gfx_read(&mut c, 20 * MS + 50_000);
        assert_eq!(a.len(), 5, "TD_STATUS plus 0x03..0x06");
        assert_eq!(a, b, "two reads in a row agree");
        assert_eq!(a, [0x01, 0x00, 160, 0x00, 120]);
        let id = a[3] >> 4;
        assert_eq!(id, 0);
        // Later frames show contact.
        let a = m5gfx_read(&mut c, 40 * MS);
        assert_eq!(a[1] >> 6, EVENT_CONTACT);
        c.input("touch", 0.0);
        assert!(!c.touched(), "INT high again");
        assert_eq!(m5gfx_read(&mut c, 60 * MS), [0x00], "0 points: M5GFX clears the AW9523 INT");
    }

    #[test]
    fn data_is_latched_only_by_a_register_address() {
        let mut c = Ft6336::default();
        finger(&mut c, 50.0, 60.0);
        let first = c.transfer(&[TD_STATUS], 1, true, 0).unwrap();
        // The page moves the finger in the middle of the transaction.
        finger(&mut c, 250.0, 200.0);
        let rest = c.transfer(&[], 4, false, 0).unwrap();
        assert_eq!(first, [0x01]);
        assert_eq!(rest, [0x00, 50, 0x00, 60], "still the frame from the start");
        assert_eq!(t(&mut c, &[], 1, 0), [0x00], "a read with no address does not latch either");
        assert_eq!(t(&mut c, &[0x03], 2, 0), [0x00, 250], "a new address does");
    }

    #[test]
    fn coordinates_are_clamped() {
        let mut c = Ft6336::default();
        finger(&mut c, 999.0, -5.0);
        assert_eq!(t(&mut c, &[0x03], 4, 0), [0x01, 0x3F, 0x00, 0x00], "X 319, Y 0");
        finger(&mut c, -1.0, 1e9);
        assert_eq!(t(&mut c, &[0x03], 4, 0), [0x00, 0x00, 0x00, 239]);
        assert!(c.input("touch_x", f64::NAN), "a bad number is ours but ignored");
        assert_eq!(c.x, 0);
    }

    #[test]
    fn second_point_is_empty() {
        let mut c = Ft6336::default();
        finger(&mut c, 1.0, 2.0);
        let b = t(&mut c, &[DATA_FIRST], 14, 0);
        assert_eq!(b[1], 1, "one point");
        assert_eq!(&b[8..], [0xFF; 6], "P2 at 0x09..0x0E");
    }

    #[test]
    fn hibernate_stops_touch_and_wakeup_restarts_it() {
        let mut c = Ft6336::default();
        finger(&mut c, 5.0, 5.0);
        t(&mut c, &[PWR_MODE, HIBERNATE], 0, 0); // M5GFX sleep()
        assert!(!c.touched());
        assert_eq!(t(&mut c, &[TD_STATUS], 1, 0), [0x00]);
        t(&mut c, &[PWR_MODE, 0x01], 0, 0); // M5GFX wakeup()
        assert!(c.touched());
        assert_eq!(t(&mut c, &[TD_STATUS], 1, 0), [0x01]);
    }

    #[test]
    fn inputs_and_state() {
        let mut c = Ft6336::default();
        assert!(!c.input("battery", 1.0));
        finger(&mut c, 12.4, 33.6);
        let s = c.state(State::new(), 0);
        assert_eq!(s.get("touch"), Some("1"));
        assert_eq!(s.get("touch_x"), Some("12"));
        assert_eq!(s.get("touch_y"), Some("34"));
    }
}
