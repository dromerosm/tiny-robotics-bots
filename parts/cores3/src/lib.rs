//! The chips on M5Stack's CoreS3 board (inside StackChan, and the CoreS3
//! Lite), as one virtual part for esp-emu. They all sit on the internal I2C
//! bus (SDA G12, SCL G11). Each chip is a module; README.md says what each one
//! models and which drivers it was checked against.
//!
//! | Address | Chip | Module |
//! | --- | --- | --- |
//! | 0x21 | GC0308 camera (control only) | `gc0308` |
//! | 0x23 | LTR-553ALS light and proximity | `ltr553` |
//! | 0x34 | AXP2101 power | `axp2101` |
//! | 0x36 | AW88298 speaker amplifier | `aw88298` |
//! | 0x38 | FT6336U touch | `ft6336` |
//! | 0x40 | ES7210 microphone ADC | `es7210` |
//! | 0x51 | BM8563 RTC | `bm8563` |
//! | 0x58 | AW9523B IO expander | `aw9523` |
//! | 0x69 | BMI270 IMU, BMM150 behind it | `bmi270` |
//!
//! The part also drives GPIO 21 (pin `INT`), the AW9523's INTN line: M5GFX
//! reads the touch only while it is low.
//!
//! The page sets values with inputs (`set:<name>=<value>`, see each module)
//! and reads `state()`.

mod aw88298;
mod aw9523;
mod axp2101;
mod bm8563;
mod bmm150;
mod bmi270;
mod chip;
mod es7210;
mod ft6336;
mod gc0308;
mod ltr553;

use chip::Chip;
use esp_part::prelude::*;

const SDA: &str = "SDA";
const SCL: &str = "SCL";
const INT: &str = "INT";

const GC0308: u32 = 0x21;
const LTR553: u32 = 0x23;
const AXP2101: u32 = 0x34;
const AW88298: u32 = 0x36;
const FT6336: u32 = 0x38;
const ES7210: u32 = 0x40;
const BM8563: u32 = 0x51;
const AW9523: u32 = 0x58;
const BMI270: u32 = 0x69;
const ADDRESSES: [u32; 9] = [GC0308, LTR553, AXP2101, AW88298, FT6336, ES7210, BM8563, AW9523, BMI270];

#[derive(Default)]
struct Cores3 {
    sda: Option<u32>,
    scl: Option<u32>,
    int: Option<u32>,
    now_ns: u64,
    /// The level we last drove on INT, if any.
    int_driven: Option<bool>,
    aw: aw9523::Aw9523,
    axp: axp2101::Axp2101,
    touch: ft6336::Ft6336,
    imu: bmi270::Bmi270,
    rtc: bm8563::Bm8563,
    light: ltr553::Ltr553,
    amp: aw88298::Aw88298,
    mic: es7210::Es7210,
    cam: gc0308::Gc0308,
}

impl Cores3 {
    fn chips(&mut self) -> [&mut dyn Chip; 9] {
        [
            &mut self.cam,
            &mut self.light,
            &mut self.axp,
            &mut self.amp,
            &mut self.touch,
            &mut self.mic,
            &mut self.rtc,
            &mut self.aw,
            &mut self.imu,
        ]
    }

    fn chip(&mut self, addr: u32) -> Option<&mut dyn Chip> {
        let at = ADDRESSES.iter().position(|&a| a == addr)?;
        Some(self.chips().into_iter().nth(at).unwrap())
    }

    /// The touch chip's INT reaches the AW9523 on P1_2, low while touched.
    fn sync_touch(&mut self) {
        let touched = self.touch.touched();
        self.aw.set_line(aw9523::TOUCH_INT, !touched);
    }

    /// The 5 V the CoreS3 gives the body: BUS_OUT_EN and BOOST_EN both on.
    fn body_5v(&self) -> bool {
        self.aw.output_high(aw9523::BUS_OUT_EN) && self.aw.output_high(aw9523::BOOST_EN)
    }
}

impl Device for Cores3 {
    const NAME: &'static str = "cores3";
    const ON_BUS: bool = true;

    fn configure(&mut self, config: &PartConfig) {
        self.sda = config.pins.get(SDA).copied();
        self.scl = config.pins.get(SCL).copied();
        self.int = config.pins.get(INT).copied();
    }

    fn claims(&self) -> Vec<Claim> {
        let mut claims: Vec<Claim> = ADDRESSES
            .map(|addr| Claim { what: Some(claim::What::I2c(I2cClaim { addr, scl_pin: self.scl, sda_pin: self.sda })) })
            .to_vec();
        if let Some(pin) = self.int {
            claims.push(drives(1 << pin));
        }
        claims
    }

    fn on_pins(&mut self, _levels: u64, now_ns: u64) {
        self.now_ns = now_ns;
    }

    fn poll(&mut self, now_ns: u64) -> Option<EmulatorMessage> {
        self.now_ns = now_ns;
        let pin = self.int?;
        let level = self.aw.intn_high();
        if self.int_driven == Some(level) {
            return None;
        }
        self.int_driven = Some(level);
        Some(drive(pin, level, now_ns))
    }

    fn on_bus(&mut self, request: &BusRequest) -> Option<BusReply> {
        let (correlation_id, op) = (request.correlation_id, BusOp::I2c as i32);
        let mut reply = BusReply { correlation_id, op, ..Default::default() };
        let Some(bus_request::Op::I2c(i2c)) = &request.op else { return Some(reply) };
        let now = self.now_ns;
        let Some(chip) = self.chip(i2c.addr) else { return Some(reply) }; // not ours: no ACK
        if let Some(data) = chip.transfer(&i2c.write, i2c.read_len as usize, i2c.continues, now) {
            reply.ack = true;
            reply.data = data;
        }
        self.sync_touch();
        Some(reply)
    }

    fn on_input(&mut self, input: &str) {
        // The SDK's set_value takes only letters, digits and `-` in a name;
        // the chips name their inputs with `_`. Both spellings work.
        let input = input.replace('_', "-");
        let Some((name, value)) = set_value(&input) else { return };
        let name = name.replace('-', "_");
        for chip in self.chips() {
            if chip.input(&name, value) {
                break;
            }
        }
        self.sync_touch();
    }

    fn state(&self) -> Option<State> {
        let mut s = State::new().set("body_5v", self.body_5v() as u8);
        let now = self.now_ns;
        // The chips' own values, in bus order.
        s = self.cam.state(s, now);
        s = self.light.state(s, now);
        s = self.axp.state(s, now);
        s = self.amp.state(s, now);
        s = self.touch.state(s, now);
        s = self.mic.state(s, now);
        s = self.rtc.state(s, now);
        s = self.aw.state(s, now);
        s = self.imu.state(s, now);
        Some(s)
    }
}

esp_part::export_part!(Cores3, buffer = 16384);

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn part() -> Cores3 {
        let mut p = Cores3::default();
        let pins = [("SDA", 12), ("SCL", 11), ("INT", 21)].map(|(n, p)| (n.to_string(), p)).into();
        p.configure(&PartConfig { pins, ..Default::default() });
        p
    }

    pub(crate) fn i2c(p: &mut Cores3, addr: u32, write: &[u8], read_len: u32) -> Option<Vec<u8>> {
        let req = BusRequest {
            correlation_id: 7,
            op: Some(bus_request::Op::I2c(I2cTransaction { addr, write: write.to_vec(), read_len, ..Default::default() })),
            ..Default::default()
        };
        let reply = p.on_bus(&req).unwrap();
        assert_eq!(reply.correlation_id, 7);
        reply.ack.then_some(reply.data)
    }

    #[test]
    fn claims_every_address_and_the_int_pin() {
        let claims = part().claims();
        assert_eq!(claims.len(), ADDRESSES.len() + 1);
        assert_eq!(claims.last(), Some(&drives(1 << 21)));
    }

    #[test]
    fn ids_answer_and_other_addresses_do_not() {
        let mut p = part();
        assert_eq!(i2c(&mut p, AW9523, &[0x10], 1), Some(vec![0x23]));
        assert_eq!(i2c(&mut p, AXP2101, &[0x03], 1), Some(vec![0x4A]));
        assert_eq!(i2c(&mut p, 0x10, &[0x40], 1), None, "M5Unified probes a direct BMM150 here");
        assert_eq!(i2c(&mut p, 0x68, &[0x75], 1), None, "and an MPU6886 here: the body's Si12T answers it");
    }

    #[test]
    fn body_5v_needs_both_pins_as_high_outputs() {
        let mut p = part();
        assert_eq!(p.state().unwrap().get("body_5v"), Some("0"));
        // As tr_board_aw9523_on(0x02, 0): level first, then direction.
        i2c(&mut p, AW9523, &[0x02, 0x02], 0);
        i2c(&mut p, AW9523, &[0x04, 0xFD], 0);
        assert_eq!(p.state().unwrap().get("bus_out_en"), Some("1"));
        assert_eq!(p.state().unwrap().get("body_5v"), Some("0"), "BOOST_EN still off");
        i2c(&mut p, AW9523, &[0x03, 0x80], 0);
        i2c(&mut p, AW9523, &[0x05, 0x7F], 0);
        assert_eq!(p.state().unwrap().get("body_5v"), Some("1"));
    }

    #[test]
    fn int_pin_is_driven_high_then_follows_intn() {
        let mut p = part();
        assert_eq!(p.poll(5), Some(drive(21, true, 5)), "released high at start");
        assert_eq!(p.poll(6), None, "nothing new");
    }

    #[test]
    fn a_touch_pulls_gpio21_low_until_the_firmware_reads_the_aw9523() {
        let mut p = part();
        i2c(&mut p, AW9523, &[0x00], 2); // M5Unified reads both ports at start
        assert_eq!(p.poll(1), Some(drive(21, true, 1)));
        p.on_input("set:touch-x=10");
        p.on_input("set:touch-y=20");
        p.on_input("set:touch=1");
        assert_eq!(p.poll(2), Some(drive(21, false, 2)), "touch INT on P1_2 -> INTN low");
        // M5GFX reads the touch, then the AW9523 ports when it finds no point.
        assert_eq!(i2c(&mut p, FT6336, &[0x02], 1), Some(vec![1]));
        i2c(&mut p, AW9523, &[0x01], 1);
        assert_eq!(p.poll(3), Some(drive(21, true, 3)), "reading port 1 releases it");
    }

    #[test]
    fn inputs_reach_the_chip_that_owns_the_name() {
        let mut p = part();
        p.on_input("set:battery=42");
        assert_eq!(i2c(&mut p, AXP2101, &[0xA4], 1), Some(vec![42]));
        p.on_input("set:touch-x=100");
        p.on_input("set:touch_y=50");
        p.on_input("set:touch=1");
        let s = p.state().unwrap();
        assert_eq!((s.get("touch_x"), s.get("touch_y"), s.get("touch")), (Some("100"), Some("50"), Some("1")), "both spellings");
        p.on_input("set:nobody=1"); // ignored
        p.on_input("press"); // not a set: ignored
    }
}
