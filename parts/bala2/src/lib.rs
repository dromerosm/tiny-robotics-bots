//! M5Stack BALA2 Fire around its ESP32, as one virtual part for esp-emu:
//! the chips on the Fire's internal I2C bus (SDA G21, SCL G22), the three
//! buttons, and how the robot moves. Each chip is a module; README.md says
//! what each one models and which drivers it was checked against.
//!
//! | Address | Chip | Module |
//! | --- | --- | --- |
//! | 0x3A | BALA2 base (STM32): motors, encoders, servos | `base` |
//! | 0x68 | MPU-6886 IMU | `mpu6886` |
//! | 0x75 | IP5306 power | `ip5306` |
//!
//! The buttons A, B and C are GPIO 39, 38 and 37 (pins `BTN_A`, `BTN_B`,
//! `BTN_C`). The board pulls them up; a press pulls one low. The part drives
//! them.
//!
//! The motors, the encoders and the IMU meet in `physics`, stepped in emulated
//! time: the firmware's motor commands move the wheels and tilt the body, and
//! the IMU feels it.
//!
//! The page sets values with inputs (`set:<name>=<value>`) and reads
//! `state()`.

mod base;
mod ip5306;
mod mpu6886;
mod physics;

use esp_part::prelude::*;
use physics::{Physics, STEP_NS};

const SDA: &str = "SDA";
const SCL: &str = "SCL";
const BUTTONS: [&str; 3] = ["BTN_A", "BTN_B", "BTN_C"];

const BASE: u32 = 0x3A;
const MPU6886: u32 = 0x68;
const IP5306: u32 = 0x75;
const ADDRESSES: [u32; 3] = [BASE, MPU6886, IP5306];

/// After a gap this long (the page paused), the model jumps instead of
/// catching up step by step.
const MAX_CATCH_UP_NS: u64 = 10_000_000_000;

#[derive(Default)]
struct Bala2 {
    sda: Option<u32>,
    scl: Option<u32>,
    button_pin: [Option<u32>; 3],
    pressed: [bool; 3],
    /// The level we last drove on each button pin, if any.
    driven: [Option<bool>; 3],
    now_ns: u64,
    /// Emulated time of the physics state.
    t_ns: u64,
    phys: Physics,
    imu: mpu6886::Mpu6886,
    base: base::Base,
    power: ip5306::Ip5306,
}

impl Bala2 {
    /// Brings the physics, and the IMU with it, up to `now_ns`.
    fn advance(&mut self, now_ns: u64) {
        self.now_ns = self.now_ns.max(now_ns);
        if self.now_ns > self.t_ns + MAX_CATCH_UP_NS {
            self.t_ns = self.now_ns - MAX_CATCH_UP_NS;
        }
        while self.t_ns + STEP_NS <= self.now_ns {
            self.t_ns += STEP_NS;
            self.phys.step();
            let (acc, gyro) = self.phys.imu();
            self.imu.feed(self.t_ns, acc, gyro);
        }
    }

    fn input(&mut self, name: &str, value: f64) -> bool {
        let on = value != 0.0;
        match name {
            "btn_a" => self.pressed[0] = on,
            "btn_b" => self.pressed[1] = on,
            "btn_c" => self.pressed[2] = on,
            "held" => self.phys.set_held(on),
            "hold_angle" => self.phys.hold_deg = value.clamp(-40.0, 40.0),
            "push" => self.phys.push(value),
            "battery" => self.power.battery = value.clamp(0.0, 100.0) as u8,
            "charging" => self.power.charging = on,
            "body_l" if value > 0.0 => self.phys.body_l = value,
            "couple" if value >= 0.0 => self.phys.couple = value,
            "motor_tau" if value > 0.0 => self.phys.motor_tau = value,
            "motor_delay_ms" if value >= 0.0 => self.phys.motor_delay_ms = value,
            "m_s_per_pwm" if value > 0.0 => self.phys.m_s_per_pwm = value,
            _ => return false,
        }
        true
    }
}

impl Device for Bala2 {
    const NAME: &'static str = "bala2";
    const ON_BUS: bool = true;

    fn configure(&mut self, config: &PartConfig) {
        self.sda = config.pins.get(SDA).copied();
        self.scl = config.pins.get(SCL).copied();
        for (i, name) in BUTTONS.iter().enumerate() {
            self.button_pin[i] = config.pins.get(*name).copied();
        }
    }

    fn claims(&self) -> Vec<Claim> {
        let mut claims: Vec<Claim> = ADDRESSES
            .map(|addr| Claim { what: Some(claim::What::I2c(I2cClaim { addr, scl_pin: self.scl, sda_pin: self.sda })) })
            .to_vec();
        let mask = self.button_pin.iter().flatten().fold(0u64, |m, &p| m | 1 << p);
        if mask != 0 {
            claims.push(drives(mask));
        }
        claims
    }

    fn on_pins(&mut self, _levels: u64, now_ns: u64) {
        self.advance(now_ns);
    }

    fn poll(&mut self, now_ns: u64) -> Option<EmulatorMessage> {
        self.advance(now_ns);
        // One button edge per poll: released is high (pulled up), pressed low.
        for i in 0..3 {
            let Some(pin) = self.button_pin[i] else { continue };
            let level = !self.pressed[i];
            if self.driven[i] != Some(level) {
                self.driven[i] = Some(level);
                return Some(drive(pin, level, now_ns));
            }
        }
        None
    }

    fn on_bus(&mut self, request: &BusRequest) -> Option<BusReply> {
        let (correlation_id, op) = (request.correlation_id, BusOp::I2c as i32);
        let mut reply = BusReply { correlation_id, op, ..Default::default() };
        let Some(bus_request::Op::I2c(i2c)) = &request.op else { return Some(reply) };
        let now = self.now_ns;
        self.advance(now);
        let (write, len) = (&i2c.write[..], i2c.read_len as usize);
        reply.data = match i2c.addr {
            BASE => self.base.transfer(write, len, &mut self.phys),
            MPU6886 => self.imu.transfer(write, len, now),
            IP5306 => self.power.transfer(write, len),
            _ => return Some(reply), // not ours: no ACK
        };
        reply.ack = true;
        Some(reply)
    }

    fn on_input(&mut self, input: &str) {
        // The SDK's set_value takes only letters, digits and `-` in a name;
        // the inputs are named with `_`. Both spellings work.
        let input = input.replace('_', "-");
        let Some((name, value)) = set_value(&input) else { return };
        self.input(&name.replace('-', "_"), value);
    }

    fn state(&self) -> Option<State> {
        let p = &self.phys;
        let s = State::new()
            .set("t", format!("{:.3}", self.t_ns as f64 / 1e9))
            .set("tilt", format!("{:.2}", p.tilt_deg()))
            .set("rate", format!("{:.1}", p.rate_dps()))
            .set("x", format!("{:.4}", p.position()))
            .set("v_l", format!("{:.3}", p.v[0]))
            .set("v_r", format!("{:.3}", p.v[1]))
            .set("u_l", p.u[0])
            .set("u_r", p.u[1])
            .set("enc_l", p.enc[0].round() as i64)
            .set("enc_r", p.enc[1].round() as i64)
            .set("heading", format!("{:.1}", p.heading_deg()))
            .set("held", p.held as u8)
            .set("servo4", self.base.servo_us[3])
            .set("btn_a", self.pressed[0] as u8)
            .set("btn_b", self.pressed[1] as u8)
            .set("btn_c", self.pressed[2] as u8)
            .set("imu_awake", self.imu.awake() as u8)
            .set("imu_fifo", self.imu.fifo_count())
            .set("battery", self.power.battery)
            .set("charging", self.power.charging as u8);
        Some(s)
    }
}

esp_part::export_part!(Bala2, buffer = 16384);

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn part() -> Bala2 {
        let mut p = Bala2::default();
        let pins = [("SDA", 21), ("SCL", 22), ("BTN_A", 39), ("BTN_B", 38), ("BTN_C", 37)].map(|(n, p)| (n.to_string(), p)).into();
        p.configure(&PartConfig { pins, ..Default::default() });
        p
    }

    fn i2c(p: &mut Bala2, addr: u32, write: &[u8], read_len: u32) -> Option<Vec<u8>> {
        let req = BusRequest {
            correlation_id: 3,
            op: Some(bus_request::Op::I2c(I2cTransaction { addr, write: write.to_vec(), read_len, ..Default::default() })),
            ..Default::default()
        };
        let reply = p.on_bus(&req).unwrap();
        assert_eq!(reply.correlation_id, 3);
        reply.ack.then_some(reply.data)
    }

    #[test]
    fn claims_three_addresses_and_the_button_pins() {
        let c = part().claims();
        assert_eq!(c.len(), 4);
        assert_eq!(c[3], drives(1 << 37 | 1 << 38 | 1 << 39));
    }

    #[test]
    fn chips_answer_and_others_do_not() {
        let mut p = part();
        assert_eq!(i2c(&mut p, MPU6886, &[0x75], 1), Some(vec![0x19]));
        assert_eq!(i2c(&mut p, IP5306, &[0x78], 1), Some(vec![0x00]));
        assert_eq!(i2c(&mut p, BASE, &[], 0), Some(vec![]), "probe");
        assert_eq!(i2c(&mut p, 0x6C, &[0x30], 1), None, "M5Stack's library looks for an SH200Q here");
        assert_eq!(i2c(&mut p, 0x29, &[0x01], 1), None, "no ToF unit");
    }

    #[test]
    fn buttons_start_released_and_a_press_pulls_low() {
        let mut p = part();
        let mut seen = vec![];
        while let Some(m) = p.poll(1) {
            seen.push(m);
        }
        assert_eq!(seen, vec![drive(39, true, 1), drive(38, true, 1), drive(37, true, 1)]);
        p.on_input("set:btn_b=1");
        assert_eq!(p.poll(2), Some(drive(38, false, 2)));
        assert_eq!(p.poll(3), None);
        p.on_input("set:btn-b=0");
        assert_eq!(p.poll(4), Some(drive(38, true, 4)));
    }

    #[test]
    fn a_motor_command_turns_the_encoders_in_emulated_time() {
        let mut p = part();
        p.on_input("set:held=1");
        i2c(&mut p, BASE, &[0x00, 0x01, 0xF4, 0x01, 0xF4], 0); // 500, 500
        p.poll(1000 * MS);
        let r = i2c(&mut p, BASE, &[0x10], 8).unwrap();
        let left = i32::from_be_bytes([r[0], r[1], r[2], r[3]]);
        assert!(left < -500, "counts down going forward: {left}");
    }

    #[test]
    fn lying_on_its_front_the_imu_reads_45_degrees() {
        let mut p = part();
        for (r, v) in [(0x6B, 0x80), (0x6B, 0x01), (0x1C, 0x10), (0x1B, 0x18), (0x1A, 0x01), (0x19, 0x01)] {
            i2c(&mut p, MPU6886, &[r, v], 0);
        }
        p.poll(20 * MS);
        let b = i2c(&mut p, MPU6886, &[0x3B], 6).unwrap();
        let ay = i16::from_be_bytes([b[2], b[3]]) as f64;
        let az = i16::from_be_bytes([b[4], b[5]]) as f64;
        let tilt = ay.atan2(az).to_degrees();
        assert!((tilt - 45.0).abs() < 1.5, "tilt {tilt}");
    }

    #[test]
    fn inputs_and_state() {
        let mut p = part();
        p.on_input("set:hold_angle=-5");
        p.on_input("set:battery=40");
        p.on_input("set:nobody=1"); // ignored
        let s = p.state().unwrap();
        assert_eq!(s.get("battery"), Some("40"));
        assert_eq!(s.get("tilt"), Some("45.00"));
        assert_eq!(i2c(&mut p, IP5306, &[0x78], 1), Some(vec![0xE0]));
    }
}
