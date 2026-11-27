//! TDK InvenSense MPU-6886 6-axis IMU (I2C 0x68), from its datasheet
//! (DS-000193 rev 1.1, section 9):
//!
//! - WHO_AM_I (0x75) = 0x19. PWR_MGMT_1 (0x6B) resets to 0x41 (asleep),
//!   CONFIG (0x1A) to 0x80, every other register to 0. DEVICE_RESET
//!   (PWR_MGMT_1 bit 7) restores them all.
//! - Output data rate: 1 kHz / (1 + SMPLRT_DIV) while the low-pass filter is
//!   on (GYRO_CONFIG FCHOICE_B = 0 and CONFIG DLPF_CFG 1 to 6), 8 kHz with
//!   DLPF_CFG 0 or 7, 32 kHz with FCHOICE_B set.
//! - Full scales (FS_SEL): 131, 65.5, 32.8, 16.4 LSB per deg/s and 16384,
//!   8192, 4096, 2048 LSB per g. Temperature: TEMP_OUT = (T - 25 C) * 326.8.
//! - The data registers 0x3B to 0x48 (accel, temperature, gyro, big-endian)
//!   take a new sample at the output data rate. INT_STATUS (0x3A) sets
//!   DATA_RDY_INT (bit 0) on each sample and FIFO_OFLOW_INT (bit 4) on an
//!   overflow; reading INT_STATUS clears both.
//! - A 1024-byte FIFO: FIFO_EN (0x23) GYRO_FIFO_EN (bit 4) and ACCEL_FIFO_EN
//!   (bit 3), with USER_CTRL (0x6A) FIFO_EN (bit 6). A packet is the accel
//!   (6 bytes) if on, the temperature (2) whenever accel or gyro is on, then
//!   the gyro (6) if on. When full, CONFIG FIFO_MODE (bit 6) 0 drops the
//!   oldest bytes and 1 drops the new ones. USER_CTRL FIFO_RST (bit 2)
//!   empties it. FIFO_COUNTH/L at 0x72/0x73, data at FIFO_R_W (0x74); a long
//!   read of FIFO_R_W stays on FIFO_R_W. An empty FIFO reads 0xFF.
//! - Sleep (PWR_MGMT_1 bit 6) stops the samples.
//!
//! Not modelled: the offset registers (stored only), self-test, wake on
//! motion, the INT pin and the low-pass filter itself.
//!
//! The chip adds a fixed zero-rate offset and white noise of about the
//! datasheet's noise density, so a firmware has something to calibrate.

const SMPLRT_DIV: usize = 0x19;
const CONFIG: usize = 0x1A;
const GYRO_CONFIG: usize = 0x1B;
const ACCEL_CONFIG: usize = 0x1C;
const FIFO_EN: usize = 0x23;
const INT_STATUS: usize = 0x3A;
const ACCEL_XOUT_H: usize = 0x3B;
const GYRO_ZOUT_L: usize = 0x48;
const SIGNAL_PATH_RESET: usize = 0x68;
const USER_CTRL: usize = 0x6A;
const PWR_MGMT_1: usize = 0x6B;
const FIFO_COUNTH: usize = 0x72;
const FIFO_COUNTL: usize = 0x73;
const FIFO_R_W: usize = 0x74;
const WHO_AM_I: usize = 0x75;

const FIFO_MODE: u8 = 0x40;
const GYRO_FIFO: u8 = 0x10;
const ACCEL_FIFO: u8 = 0x08;
const INT_FIFO_OFLOW: u8 = 0x10;
const INT_DATA_RDY: u8 = 0x01;
const USER_FIFO_EN: u8 = 0x40;
const USER_FIFO_RST: u8 = 0x04;
const USER_SIG_COND_RST: u8 = 0x01;
const DEVICE_RESET: u8 = 0x80;
const SLEEP: u8 = 0x40;

pub const ID: u8 = 0x19;
const FIFO_SIZE: usize = 1024;
const NS: u64 = 1_000_000_000;

/// Zero-rate offset (deg/s) and zero-g offset (g) of this unit, inside the
/// datasheet's typical +-1 deg/s and +-25 mg. Noise: 0.004 deg/s/sqrt(Hz)
/// and 100 ug/sqrt(Hz) over the default 176 Hz and 218 Hz filters.
const GYRO_BIAS_DPS: [f64; 3] = [-0.61, 0.37, 0.24];
const ACC_BIAS_G: [f64; 3] = [0.012, -0.008, 0.015];
const GYRO_NOISE_DPS: f64 = 0.07;
const ACC_NOISE_G: f64 = 0.0019;
const TEMP_C: f64 = 31.0;

const GYRO_LSB: [f64; 4] = [131.0, 65.5, 32.8, 16.4];
const ACC_LSB: [f64; 4] = [16384.0, 8192.0, 4096.0, 2048.0];

pub struct Mpu6886 {
    regs: [u8; 128],
    at: usize,
    fifo: [u8; FIFO_SIZE],
    head: usize,
    count: usize,
    next_sample_ns: u64,
    rng: u32,
    /// Motion in the chip's axes: specific force (g) and rate (deg/s).
    acc_g: [f64; 3],
    gyro_dps: [f64; 3],
}

impl Default for Mpu6886 {
    fn default() -> Self {
        let mut m = Self {
            regs: [0; 128],
            at: 0,
            fifo: [0; FIFO_SIZE],
            head: 0,
            count: 0,
            next_sample_ns: 0,
            rng: 0x6886,
            acc_g: [0.0, 0.0, 1.0],
            gyro_dps: [0.0; 3],
        };
        m.reset(0);
        m
    }
}

fn put16(out: &mut [u8], v: f64) {
    let x = v.round().clamp(-32768.0, 32767.0) as i16;
    out[0] = (x >> 8) as u8;
    out[1] = x as u8;
}

impl Mpu6886 {
    fn reset(&mut self, now_ns: u64) {
        self.regs = [0; 128];
        self.regs[CONFIG] = 0x80; // bit 7 is reserved and resets to 1
        self.regs[PWR_MGMT_1] = 0x41;
        self.regs[WHO_AM_I] = ID;
        self.head = 0;
        self.count = 0;
        self.next_sample_ns = now_ns;
    }

    pub fn awake(&self) -> bool {
        self.regs[PWR_MGMT_1] & SLEEP == 0
    }

    pub fn fifo_on(&self) -> bool {
        self.regs[USER_CTRL] & USER_FIFO_EN != 0
    }

    fn period_ns(&self) -> u64 {
        let dlpf = self.regs[CONFIG] & 7;
        if self.regs[GYRO_CONFIG] & 3 != 0 {
            NS / 32_000
        } else if dlpf == 0 || dlpf == 7 {
            NS / 8_000
        } else {
            NS * (1 + self.regs[SMPLRT_DIV] as u64) / 1_000
        }
    }

    /// Unit-variance noise: xorshift32, then a sum of four uniforms.
    fn noise(&mut self) -> f64 {
        let mut sum = 0.0;
        for _ in 0..4 {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 17;
            self.rng ^= self.rng << 5;
            sum += self.rng as f64 / 4294967296.0 - 0.5;
        }
        sum * 1.732_050_8
    }

    fn fifo_push(&mut self, data: &[u8]) {
        for &b in data {
            if self.count == FIFO_SIZE {
                self.regs[INT_STATUS] |= INT_FIFO_OFLOW;
                if self.regs[CONFIG] & FIFO_MODE != 0 {
                    return;
                }
                self.head = (self.head + 1) % FIFO_SIZE;
                self.count -= 1;
            }
            self.fifo[(self.head + self.count) % FIFO_SIZE] = b;
            self.count += 1;
        }
    }

    fn sample(&mut self) {
        let a = ACC_LSB[((self.regs[ACCEL_CONFIG] >> 3) & 3) as usize];
        let g = GYRO_LSB[((self.regs[GYRO_CONFIG] >> 3) & 3) as usize];
        let mut out = [0u8; 14];
        for i in 0..3 {
            let acc = (self.acc_g[i] + ACC_BIAS_G[i] + ACC_NOISE_G * self.noise()) * a;
            put16(&mut out[2 * i..], acc);
            let rate = (self.gyro_dps[i] + GYRO_BIAS_DPS[i] + GYRO_NOISE_DPS * self.noise()) * g;
            put16(&mut out[8 + 2 * i..], rate);
        }
        put16(&mut out[6..], (TEMP_C - 25.0) * 326.8);
        self.regs[ACCEL_XOUT_H..=GYRO_ZOUT_L].copy_from_slice(&out);
        self.regs[INT_STATUS] |= INT_DATA_RDY;

        let en = self.regs[FIFO_EN] & (GYRO_FIFO | ACCEL_FIFO);
        if self.fifo_on() && en != 0 {
            if en & ACCEL_FIFO != 0 {
                self.fifo_push(&out[0..6]);
            }
            self.fifo_push(&out[6..8]);
            if en & GYRO_FIFO != 0 {
                self.fifo_push(&out[8..14]);
            }
        }
    }

    /// The motion at emulated time `now_ns`, in the chip's axes. Takes every
    /// sample due up to then. Call with rising times.
    pub fn feed(&mut self, now_ns: u64, acc_g: [f64; 3], gyro_dps: [f64; 3]) {
        self.acc_g = acc_g;
        self.gyro_dps = gyro_dps;
        let period = self.period_ns();
        if !self.awake() {
            self.next_sample_ns = now_ns + period;
            return;
        }
        // After a long gap the FIFO would be full anyway.
        if now_ns > self.next_sample_ns + NS {
            self.next_sample_ns = now_ns - NS;
        }
        while self.next_sample_ns <= now_ns {
            self.sample();
            self.next_sample_ns += period;
        }
    }

    fn write_reg(&mut self, reg: usize, v: u8, now_ns: u64) {
        match reg {
            PWR_MGMT_1 if v & DEVICE_RESET != 0 => self.reset(now_ns),
            PWR_MGMT_1 => {
                if !self.awake() && v & SLEEP == 0 {
                    self.next_sample_ns = now_ns;
                }
                self.regs[reg] = v;
            }
            USER_CTRL => {
                if v & USER_FIFO_RST != 0 {
                    self.head = 0;
                    self.count = 0;
                }
                if v & USER_SIG_COND_RST != 0 {
                    self.regs[ACCEL_XOUT_H..=GYRO_ZOUT_L].fill(0);
                }
                self.regs[reg] = v & !(USER_FIFO_RST | USER_SIG_COND_RST);
            }
            SIGNAL_PATH_RESET => {} // self-clearing
            FIFO_R_W => self.fifo_push(&[v]),
            SMPLRT_DIV | CONFIG | GYRO_CONFIG => {
                self.regs[reg] = v;
                // The new rate starts from now.
                self.next_sample_ns = now_ns + self.period_ns();
            }
            INT_STATUS | ACCEL_XOUT_H..=GYRO_ZOUT_L | FIFO_COUNTH | FIFO_COUNTL | WHO_AM_I => {}
            _ => self.regs[reg] = v,
        }
    }

    fn read_reg(&mut self) -> u8 {
        let reg = self.at;
        let v = match reg {
            FIFO_COUNTH => (self.count >> 8) as u8,
            FIFO_COUNTL => self.count as u8,
            FIFO_R_W => {
                if self.count == 0 {
                    return 0xFF; // the pointer stays on FIFO_R_W
                }
                let b = self.fifo[self.head];
                self.head = (self.head + 1) % FIFO_SIZE;
                self.count -= 1;
                return b;
            }
            INT_STATUS => {
                let v = self.regs[INT_STATUS];
                self.regs[INT_STATUS] = 0;
                v
            }
            _ => self.regs[reg],
        };
        self.at = (self.at + 1) % 128;
        v
    }

    /// One I2C transaction: the first written byte is the register, the rest
    /// are written from there; then `read_len` bytes are read. A later piece
    /// of a split read (nothing written) goes on from where the last stopped.
    pub fn transfer(&mut self, write: &[u8], read_len: usize, now_ns: u64) -> Vec<u8> {
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg as usize % 128;
            for &b in data {
                let r = self.at;
                if r != FIFO_R_W {
                    self.at = (self.at + 1) % 128;
                }
                self.write_reg(r, b, now_ns);
            }
        }
        (0..read_len).map(|_| self.read_reg()).collect()
    }

    pub fn fifo_count(&self) -> usize {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn awake_500hz() -> Mpu6886 {
        let mut m = Mpu6886::default();
        // As M5Stack's MPU6886::Init: reset, wake, +-8 g, +-2000 deg/s, 500 Hz.
        for (r, v) in [(0x6B, 0x00), (0x6B, 0x80), (0x6B, 0x01), (0x1C, 0x10), (0x1B, 0x18), (0x1A, 0x01), (0x19, 0x01)] {
            m.transfer(&[r, v], 0, 0);
        }
        m
    }

    #[test]
    fn identity_and_reset_values() {
        let mut m = Mpu6886::default();
        assert_eq!(m.transfer(&[0x75], 1, 0), vec![0x19]);
        assert_eq!(m.transfer(&[0x6B], 1, 0), vec![0x41]);
        assert_eq!(m.transfer(&[0x1A], 1, 0), vec![0x80]);
    }

    #[test]
    fn asleep_it_takes_no_samples() {
        let mut m = Mpu6886::default();
        m.feed(10 * MS, [0.0, 0.0, 1.0], [0.0; 3]);
        assert_eq!(m.transfer(&[0x3B], 6, 10 * MS), vec![0; 6]);
    }

    #[test]
    fn data_registers_read_one_g_on_z_at_8g_scale() {
        let mut m = awake_500hz();
        m.feed(10 * MS, [0.0, 0.0, 1.0], [0.0; 3]);
        let b = m.transfer(&[0x3B], 14, 10 * MS);
        let az = i16::from_be_bytes([b[4], b[5]]) as f64;
        assert!((az - 4096.0).abs() < 4096.0 * 0.05, "az {az}");
        let gx = i16::from_be_bytes([b[8], b[9]]) as f64 / 16.4;
        assert!(gx.abs() < 1.5, "gyro offset within +-1 deg/s plus noise: {gx}");
    }

    #[test]
    fn fifo_fills_with_14_byte_packets_at_500_hz() {
        let mut m = awake_500hz();
        // As M5Stack's setFIFOEnable(true) and RestFIFO().
        m.transfer(&[0x23, 0x18], 0, 0);
        m.transfer(&[0x6A, 0x40], 0, 0);
        let v = m.transfer(&[0x6A], 1, 0)[0];
        m.transfer(&[0x6A, v | 0x04], 0, 0);
        m.feed(10 * MS, [0.0, 0.0, 1.0], [0.0; 3]);
        let c = m.transfer(&[0x72], 2, 10 * MS);
        let count = (c[0] as usize) << 8 | c[1] as usize;
        assert_eq!(count, 5 * 14, "10 ms at 500 Hz");
        let b = m.transfer(&[0x74], 14, 10 * MS);
        assert!(i16::from_be_bytes([b[4], b[5]]) > 3500, "accel Z first");
        assert_eq!(m.fifo_count(), 4 * 14);
    }

    #[test]
    fn a_full_fifo_drops_the_oldest_and_flags_it() {
        let mut m = awake_500hz();
        m.transfer(&[0x23, 0x18], 0, 0);
        m.transfer(&[0x6A, 0x40], 0, 0);
        m.feed(500 * MS, [0.0, 0.0, 1.0], [0.0; 3]);
        assert_eq!(m.fifo_count(), 1024);
        assert_eq!(m.transfer(&[0x3A], 1, 500 * MS)[0] & 0x10, 0x10);
        assert_eq!(m.transfer(&[0x3A], 1, 500 * MS)[0], 0, "read clears");
    }

    #[test]
    fn an_empty_fifo_reads_ff() {
        let mut m = awake_500hz();
        assert_eq!(m.transfer(&[0x74], 2, 0), vec![0xFF, 0xFF]);
    }
}
