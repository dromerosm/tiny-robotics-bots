//! How the robot moves: a two-wheeled inverted pendulum, stepped in 0.5 ms
//! steps of emulated time. Ported from `sim_step()` in
//! examples/bala2-fire/06-balance, with the numbers its README measured on
//! our robot:
//!
//! - Each motor: a speed that grows from PWM 31 to 0.44 m/s at PWM 500 (with
//!   the robot's weight), reached after a 20 ms delay with an 80 ms lag
//!   (test 11). A stopped wheel only starts at PWM 55 (example 05's dead
//!   band); a turning one keeps going above 31.
//! - The body: the motors set the wheels' speed against the body, `v_rel`
//!   (the encoders count it), so over the floor the axle moves at
//!   `v_rel + r theta'`. With that constraint the equation of motion is
//!
//!   `L(theta) theta'' = g sin(theta) - (cos(theta) + c) a_rel + r sin(theta) theta'^2`
//!
//!   where `a_rel` is the wheels' acceleration against the body, r = 22.5 mm
//!   and `L(theta) = L - 2 r (1 - cos(theta))` is the pendulum length about
//!   the floor contact. L = 91 mm is what the fall tests measured with the
//!   motors holding the wheels (`a_rel` = 0, lambda = 10.4 1/s). The c term
//!   is the motor's reaction torque on the body, `M r / (m d)`: about 0.35
//!   with an estimated center of mass d = 65 mm over the axle.
//! - It lies on the floor at 45 degrees on its front and 42 on its back.
//! - Wheels 45 mm across, 0.077 m apart, 420 encoder counts per turn.
//! - A hand can hold it (`held`): off the floor, the wheels spin in the air,
//!   and the hand turns the body to `hold_deg` at up to 120 deg/s, so the
//!   gyro sees the move. Let go, it stands on the floor at that angle.
//!
//! The IMU sits 60 mm over the axle. Z points up when the robot stands, X
//! along the axle (to the left of "forward", a positive motor command) and
//! Y backward. It gets the specific force (gravity plus the body's own
//! acceleration) and the body's rates in those axes.

use std::f64::consts::PI;

const G: f64 = 9.81;
const WHEEL_R: f64 = 0.0225;
const TRACK_M: f64 = 0.077;
const IMU_R: f64 = 0.06;
pub const BODY_L: f64 = 0.091;
pub const COUPLE: f64 = 0.35;
pub const MOTOR_TAU: f64 = 0.08;
pub const MOTOR_DELAY_MS: f64 = 20.0;
pub const M_S_PER_PWM: f64 = 0.44 / (500.0 - 31.0);
const DEAD_BAND: i32 = 55;
const PWM_OFFSET: i32 = 31;
pub const M_PER_COUNT: f64 = 2.0 * PI * WHEEL_R / 420.0;
const LIE_FRONT: f64 = 45.0 * PI / 180.0;
const LIE_BACK: f64 = -42.0 * PI / 180.0;
const HAND_RATE: f64 = 120.0 * PI / 180.0;
const HAND_ACC: f64 = 20.0;
pub const STEP_NS: u64 = 500_000;
const DT: f64 = STEP_NS as f64 / 1e9;
const MAX_DELAY_STEPS: usize = 400; // 200 ms
const RAD2DEG: f64 = 180.0 / PI;

pub struct Physics {
    pub th: f64,       // tilt, rad, + = leans forward
    pub thd: f64,      // tilt rate
    thdd: f64,
    pub v: [f64; 2],   // wheel speed against the body, m/s, motor 1 and 2
    pub x: [f64; 2],   // distance over the floor, m
    pub enc: [f64; 2], // encoder counts
    pub psi: f64,      // heading, rad, + = left
    ground_a: f64,     // axle acceleration over the floor
    pub u: [i16; 2],   // motor commands, -1023..1023
    u_hist: [[i16; 2]; MAX_DELAY_STEPS],
    u_pos: usize,
    pub held: bool,
    pub hold_deg: f64,
    // The model's numbers, to try others from the page.
    pub body_l: f64,
    pub couple: f64,
    pub motor_tau: f64,
    pub motor_delay_ms: f64,
    pub m_s_per_pwm: f64,
}

impl Default for Physics {
    fn default() -> Self {
        Self {
            th: LIE_FRONT, // it starts lying on its front
            thd: 0.0,
            thdd: 0.0,
            v: [0.0; 2],
            x: [0.0; 2],
            enc: [0.0; 2],
            psi: 0.0,
            ground_a: 0.0,
            u: [0; 2],
            u_hist: [[0; 2]; MAX_DELAY_STEPS],
            u_pos: 0,
            held: false,
            hold_deg: 0.0,
            body_l: BODY_L,
            couple: COUPLE,
            motor_tau: MOTOR_TAU,
            motor_delay_ms: MOTOR_DELAY_MS,
            m_s_per_pwm: M_S_PER_PWM,
        }
    }
}

impl Physics {
    /// The speed a command asks for. A stopped wheel needs DEAD_BAND to
    /// start; a turning one keeps going down to PWM_OFFSET.
    fn motor_speed(&self, u: i16, v_now: f64) -> f64 {
        let u = u as i32;
        let mag = u.abs();
        let turning = v_now.abs() > 0.002 && (v_now > 0.0) == (u > 0);
        if mag <= PWM_OFFSET || (mag < DEAD_BAND && !turning) {
            return 0.0;
        }
        let v = self.m_s_per_pwm * (mag - PWM_OFFSET) as f64;
        if u > 0 { v } else { -v }
    }

    pub fn step(&mut self) {
        // The motor driver acts on the command 20 ms late.
        self.u_hist[self.u_pos] = self.u;
        let delay = ((self.motor_delay_ms * 2.0).round() as usize).clamp(1, MAX_DELAY_STEPS);
        let old = (self.u_pos + MAX_DELAY_STEPS + 1 - delay) % MAX_DELAY_STEPS;
        self.u_pos = (self.u_pos + 1) % MAX_DELAY_STEPS;
        let tau = self.motor_tau.max(0.001);
        let mut a = 0.0;
        for w in 0..2 {
            let aw = (self.motor_speed(self.u_hist[old][w], self.v[w]) - self.v[w]) / tau;
            self.v[w] += aw * DT;
            self.enc[w] -= self.v[w] * DT / M_PER_COUNT; // a positive speed counts down
            a += aw / 2.0;
        }

        if self.held {
            let target = self.hold_deg / RAD2DEG;
            let mut rate = ((target - self.th) * 8.0).clamp(-HAND_RATE, HAND_RATE);
            rate = rate.clamp(self.thd - HAND_ACC * DT, self.thd + HAND_ACC * DT);
            self.thdd = (rate - self.thd) / DT;
            self.thd = rate;
            self.th += rate * DT;
            self.ground_a = 0.0;
            return;
        }

        let th = self.th;
        let len = (self.body_l - 2.0 * WHEEL_R * (1.0 - th.cos())).max(0.01);
        self.thdd = (G * th.sin() - (th.cos() + self.couple) * a + WHEEL_R * th.sin() * self.thd * self.thd) / len;
        self.thd += self.thdd * DT;
        self.th += self.thd * DT;
        // The floor.
        if self.th >= LIE_FRONT {
            self.th = LIE_FRONT;
            if self.thd > 0.0 {
                self.thd = 0.0;
                self.thdd = 0.0;
            }
        } else if self.th <= LIE_BACK {
            self.th = LIE_BACK;
            if self.thd < 0.0 {
                self.thd = 0.0;
                self.thdd = 0.0;
            }
        }
        // Over the floor the axle rolls with the body as well.
        self.ground_a = a + WHEEL_R * self.thdd;
        for w in 0..2 {
            self.x[w] += (self.v[w] + WHEEL_R * self.thd) * DT;
        }
        self.psi += (self.v[0] - self.v[1]) / TRACK_M * DT;
    }

    /// What the IMU feels: specific force (g) and rates (deg/s), in its axes.
    pub fn imu(&self) -> ([f64; 3], [f64; 3]) {
        let (th, thd, thdd) = (self.th, self.thd, self.thdd);
        let roll = if self.held { 0.0 } else { WHEEL_R * thd };
        let vm = (self.v[0] + self.v[1]) / 2.0 + roll;
        let psid = (self.v[0] - self.v[1]) / TRACK_M;
        let f_fwd = self.ground_a + IMU_R * (thdd * th.cos() - thd * thd * th.sin());
        let f_up = IMU_R * (-thdd * th.sin() - thd * thd * th.cos()) + G;
        let acc = [vm * psid / G, (-f_fwd * th.cos() + f_up * th.sin()) / G, (f_fwd * th.sin() + f_up * th.cos()) / G];
        let gyro = [thd * RAD2DEG, psid * th.sin() * RAD2DEG, psid * th.cos() * RAD2DEG];
        (acc, gyro)
    }

    /// A hand catches it (no faster than it turns it) or lets go gently.
    pub fn set_held(&mut self, on: bool) {
        self.held = on;
        self.thd = if on { self.thd.clamp(-HAND_RATE, HAND_RATE) } else { 0.0 };
    }

    /// A push: adds this much (deg/s) to the tilt rate.
    pub fn push(&mut self, dps: f64) {
        if !self.held {
            self.thd += dps.clamp(-200.0, 200.0) / RAD2DEG;
        }
    }

    pub fn tilt_deg(&self) -> f64 {
        self.th * RAD2DEG
    }

    pub fn rate_dps(&self) -> f64 {
        self.thd * RAD2DEG
    }

    pub fn position(&self) -> f64 {
        (self.x[0] + self.x[1]) / 2.0
    }

    pub fn heading_deg(&self) -> f64 {
        self.psi * RAD2DEG
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(p: &mut Physics, ms: u64) {
        for _ in 0..ms * 2 {
            p.step();
        }
    }

    #[test]
    fn lying_still_it_feels_gravity_at_45_degrees() {
        let p = Physics::default();
        let (acc, gyro) = p.imu();
        assert!((acc[1] - 45f64.to_radians().sin()).abs() < 1e-9);
        assert!((acc[2] - 45f64.to_radians().cos()).abs() < 1e-9);
        assert_eq!(gyro, [0.0; 3]);
    }

    #[test]
    fn a_fall_from_upright_grows_at_the_measured_rate() {
        let mut p = Physics::default();
        p.th = 1f64.to_radians();
        run(&mut p, 100);
        // lambda = sqrt(g / L) = 10.4 1/s: about e^1.04 = 2.8 times in 100 ms
        // (cosh for a start at rest: 1.6), well before the floor.
        let grow = p.tilt_deg();
        assert!(grow > 1.4 && grow < 2.0, "tilt after 100 ms: {grow}");
    }

    #[test]
    fn a_command_moves_the_wheels_after_the_delay_and_counts_down() {
        let mut p = Physics::default();
        p.held = true;
        p.u = [500, 500];
        run(&mut p, 19);
        assert_eq!(p.v, [0.0, 0.0], "nothing in the first 20 ms");
        run(&mut p, 600);
        assert!((p.v[0] - 0.44).abs() < 0.01, "0.44 m/s at PWM 500: {}", p.v[0]);
        assert!(p.enc[0] < -100.0);
        assert_eq!(p.position(), 0.0, "held: the wheels spin in the air");
    }

    #[test]
    fn a_stopped_wheel_needs_55_and_a_turning_one_goes_on_above_31() {
        let mut p = Physics::default();
        p.held = true;
        p.u = [40, 40];
        run(&mut p, 300);
        assert_eq!(p.v[0], 0.0);
        p.u = [60, 60];
        run(&mut p, 300);
        p.u = [40, 40];
        run(&mut p, 600);
        assert!(p.v[0] > 0.005, "still turning: {}", p.v[0]);
    }

    #[test]
    fn the_hand_picks_it_up_to_the_hold_angle() {
        let mut p = Physics::default();
        p.set_held(true);
        run(&mut p, 2000);
        assert!(p.tilt_deg().abs() < 0.1, "{}", p.tilt_deg());
        p.set_held(false);
        assert_eq!(p.thd, 0.0);
    }
}
