//! BMI270 IMU at 0x69, with the BMM150 magnetometer (`bmm150`) behind its
//! aux interface.
//!
//! Checked against three drivers, which must all work unchanged:
//! - tr_board (`tr_sensors.c`): config in 64-byte chunks, polls 0x21, reads
//!   12 bytes from 0x0C as ±8 g / ±2000 °/s, temperature from 0x22.
//! - M5Unified (`BMI270_Class.inl`): chip ID twice, the whole config in one
//!   8193-byte write, the BMM150 through aux manual mode, then aux data mode;
//!   reads 0x1D and, if a drdy bit is set, 20 bytes from 0x04.
//! - The StackChan firmware (Bosch BMI270 SensorAPI, `bmi2.c`): soft reset
//!   that must ACK, config in 32-byte bursts, one read of 0x21 that must be
//!   1, ACC/GYR config with an EVENT check, then 24 bytes from 0x03.
//!
//! What is modelled:
//! - Registers auto-increment on reads and writes, except two that trap the
//!   pointer (the SensorAPI's note on `bmi2_get_regs`): 0x5E (INIT_DATA) on
//!   writes and 0x26 (FIFO_DATA) on reads.
//! - Config upload: 0x5B bits 3:0 and 0x5C give the word address; bytes
//!   written to 0x5E go to that word and every 2 bytes move it on by one,
//!   whatever the burst size (2..8192 bytes, also split into separate
//!   writes). INIT_CTRL (0x59) bit 0 set after every one of the 4096 words
//!   was written makes INTERNAL_STATUS (0x21) read 0x01 (init ok). The
//!   content is not checked.
//! - CMD (0x7E): 0xB6 soft reset (ACKed, then every register back to its
//!   reset value and the upload forgotten). Other commands are ignored.
//! - PWR_CONF (0x7C) and PWR_CTRL (0x7D) store what is written. Data only
//!   flows once the config is loaded, and only for the sensors enabled in
//!   PWR_CTRL: bit 2 accel, bit 1 gyro, bit 3 temperature, bit 0 aux.
//!   Disabled data registers keep their last value. Advanced power save
//!   does not stop the data (the SensorAPI keeps it on while it reads).
//! - Accel (0x0C-0x11) scaled by ACC_RANGE (0x41: ±2/4/8/16 g), gyro
//!   (0x12-0x17) by GYR_RANGE (0x43: ±2000/1000/500/250/125 °/s), int16
//!   little-endian. Each new transaction (not a `continues` piece) takes a
//!   fresh, coherent sample of everything.
//! - Data ready at the ODR set in ACC_CONF/GYR_CONF/AUX_CONF bits 3:0
//!   (100 Hz × 2^(odr - 8)): it sets STATUS (0x03) bits 7/6/5 and
//!   INT_STATUS_1 (0x1D) bits 7/6/5. Reading 0x1D clears what it read.
//!   Reading the matching data registers clears the STATUS bit.
//! - Sensortime (0x18-0x1A): 24 bits, one tick per 39.0625 µs of emulated
//!   time (`BMI2_SENSORTIME_RESOLUTION` in the SensorAPI's bmi2_defs.h).
//! - Temperature (0x22-0x23): 25 °C as 0 = 23 °C, 512 LSB/K (the formula
//!   tr_board and M5Unified use); 0x8000 while it is disabled.
//! - ERR_REG (0x02) reads 0 and EVENT (0x1B) reads 0x01 (power-on detected,
//!   no config error in bits 4:2). Invalid configs are not detected.
//! - STATUS bit 4 (cmd_rdy) is always 1, bit 2 (aux busy) always 0: aux
//!   operations finish at once.
//! - Aux interface, with IF_CONF (0x6B) bit 5 on and AUX_DEV_ID (0x4B) =
//!   0x10 << 1: in manual mode (AUX_IF_CONF 0x4C bit 7) writing AUX_RD_ADDR
//!   (0x4D) reads 1/2/6/8 bytes (0x4C bits 3:2) into 0x04.., and writing
//!   AUX_WR_ADDR (0x4E) writes AUX_WR_DATA (0x4F) to the BMM150. In data
//!   mode (0x4C bit 7 clear, PWR_CTRL bit 0 set) every sample copies
//!   1/2/6/8 bytes (0x4C bits 1:0) from the BMM150 at 0x4D into 0x04...
//!   Any other aux address finds nothing: the bytes stay as they were.
//! - Everything else is storage (feature pages 0x2F/0x30-0x3F read zeros,
//!   so the gyro cross-sensitivity the SensorAPI reads is 0).
//!
//! Orientation. The page sets `roll`, `pitch` (degrees, the board's tilt),
//! `yaw` (degrees, compass heading) and `gyro_x/y/z` (°/s). Axes are the
//! BMI270's own, which are the axes M5Unified reports on the CoreS3 and
//! StackChan: it applies no accel or gyro fix there, only Y and Z inverted
//! on the magnetometer (`IMU_Class.inl`, the CoreS3/StackChan case at
//! 0x69). Flat and screen up the accel reads (0, 0, +1 g). The board turns
//! by yaw about up, then pitch about its Y, then roll about its X, so the
//! accel reads (-sin p, sin r cos p, cos r cos p) g and
//! roll = atan2(ay, az), pitch = atan2(-ax, hypot(ay, az)) give the inputs
//! back. Which physical edge of the board is +X was not checked on
//! hardware. The gyro inputs go to the sensor axes as given; they are not
//! integrated into the angles.
//!
//! Magnetic field: 40 µT towards magnetic north and 30 µT down (about 50 µT
//! at 37° dip, a mid-latitude value), turned with the board. At yaw 0 the
//! board's +X points north; yaw grows clockwise seen from above, so the
//! heading is atan2(my, mx) on the board axes when flat. The BMM150 gets it
//! on its own axes, (x, -y, -z) of the board, so M5Unified's Y and Z
//! inversion brings it back to the board's axes.
//!
//! Reset values marked "datasheet, unconfirmed" come from the BMI270
//! datasheet, not from driver code.

use crate::bmm150::Bmm150;
use crate::chip::{Chip, Regs8};
use esp_part::State;

const CHIP_ID: u8 = 0x00;
const ERR_REG: u8 = 0x02;
const STATUS: u8 = 0x03;
const AUX_DATA: u8 = 0x04;
const ACC_DATA: u8 = 0x0C;
const GYR_DATA: u8 = 0x12;
const SENSORTIME: u8 = 0x18;
const EVENT: u8 = 0x1B;
const INT_STATUS_0: u8 = 0x1C;
const INT_STATUS_1: u8 = 0x1D;
const INTERNAL_STATUS: u8 = 0x21;
const TEMPERATURE: u8 = 0x22;
const FIFO_DATA: u8 = 0x26;
const ACC_CONF: u8 = 0x40;
const ACC_RANGE: u8 = 0x41;
const GYR_CONF: u8 = 0x42;
const GYR_RANGE: u8 = 0x43;
const AUX_CONF: u8 = 0x44;
const AUX_DEV_ID: u8 = 0x4B;
const AUX_IF_CONF: u8 = 0x4C;
const AUX_RD_ADDR: u8 = 0x4D;
const AUX_WR_ADDR: u8 = 0x4E;
const AUX_WR_DATA: u8 = 0x4F;
const INIT_CTRL: u8 = 0x59;
const INIT_ADDR_0: u8 = 0x5B;
const INIT_ADDR_1: u8 = 0x5C;
const INIT_DATA: u8 = 0x5E;
const IF_CONF: u8 = 0x6B;
const PWR_CONF: u8 = 0x7C;
const PWR_CTRL: u8 = 0x7D;
const CMD: u8 = 0x7E;

const ID: u8 = 0x24; // all three drivers require it
const SOFT_RESET: u8 = 0xB6; // all three drivers send it
const STATUS_CMD_RDY: u8 = 0x10; // datasheet, unconfirmed
const INIT_OK: u8 = 0x01; // the SensorAPI's BMI2_CONFIG_LOAD_SUCCESS
const INIT_ERR: u8 = 0x02; // datasheet, unconfirmed
const IF_AUX_EN: u8 = 0x20; // M5Unified's auxSetupMode
const AUX_MANUAL: u8 = 0x80;
const BMM150_AUX_ADDR: u8 = 0x10; // M5Unified: auxSetupMode(0x10)

/// The config file is 8192 bytes in all three drivers: 4096 words.
const CONFIG_WORDS: usize = 4096;

/// 39.0625 µs per sensortime tick (SensorAPI `BMI2_SENSORTIME_RESOLUTION`).
const SENSORTIME_NS_X2: u64 = 78125; // 2 × 39062.5 ns

/// The chip's temperature, °C (no input sets it).
const TEMP_C: f64 = 25.0;

/// Earth's field in µT, as (north, west, up): 40 µT north, 30 µT down.
const EARTH_FIELD_UT: [f64; 3] = [40.0, 0.0, -30.0];

/// The three data sources, as (PWR_CTRL bit, CONF register, drdy bit in
/// STATUS and INT_STATUS_1, first data register).
#[derive(Clone, Copy)]
enum Sensor {
    Acc,
    Gyr,
    Aux,
}
const SENSORS: [Sensor; 3] = [Sensor::Acc, Sensor::Gyr, Sensor::Aux];

impl Sensor {
    fn pwr_bit(self) -> u8 {
        match self {
            Sensor::Acc => 0x04,
            Sensor::Gyr => 0x02,
            Sensor::Aux => 0x01,
        }
    }
    fn conf(self) -> u8 {
        match self {
            Sensor::Acc => ACC_CONF,
            Sensor::Gyr => GYR_CONF,
            Sensor::Aux => AUX_CONF,
        }
    }
    /// drdy_acc/gyr/aux in STATUS and INT_STATUS_1 (bits 7/6/5, as
    /// M5Unified reads 0x1D).
    fn drdy(self) -> u8 {
        match self {
            Sensor::Acc => 0x80,
            Sensor::Gyr => 0x40,
            Sensor::Aux => 0x20,
        }
    }
    fn data(self) -> std::ops::RangeInclusive<u8> {
        match self {
            Sensor::Acc => ACC_DATA..=ACC_DATA + 5,
            Sensor::Gyr => GYR_DATA..=GYR_DATA + 5,
            Sensor::Aux => AUX_DATA..=AUX_DATA + 7,
        }
    }
}

/// Registers no write may change.
const READ_ONLY: &[u8] = &[
    CHIP_ID, 0x01, ERR_REG, STATUS, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
    0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, EVENT, INT_STATUS_0, INT_STATUS_1, 0x1F, 0x20, INTERNAL_STATUS,
    TEMPERATURE, 0x23, 0x24, 0x25, FIFO_DATA,
];

fn reset_regs() -> Regs8 {
    Regs8::new(
        &[
            (CHIP_ID, ID),
            (STATUS, STATUS_CMD_RDY),
            (EVENT, 0x01),            // por_detected (datasheet, unconfirmed)
            (TEMPERATURE + 1, 0x80),  // 0x8000: no value yet (datasheet, unconfirmed)
            (ACC_CONF, 0xA8),         // 100 Hz, avg4, perf (datasheet, unconfirmed)
            (ACC_RANGE, 0x02),        // ±8 g: tr_board and M5Unified rely on it
            (GYR_CONF, 0xA9),         // 200 Hz (datasheet, unconfirmed)
            (GYR_RANGE, 0x08),        // ±2000 °/s as the drivers assume; bit 3 ois_range (datasheet, unconfirmed)
            (AUX_CONF, 0x46),         // 25 Hz (datasheet, unconfirmed)
            (AUX_DEV_ID, 0x20),       // datasheet, unconfirmed
            (AUX_IF_CONF, 0x83),      // datasheet, unconfirmed
            (AUX_RD_ADDR, 0x42),      // datasheet, unconfirmed
            (AUX_WR_ADDR, 0x4C),      // datasheet, unconfirmed
            (AUX_WR_DATA, 0x02),      // datasheet, unconfirmed
            (PWR_CONF, 0x03),         // nonzero: M5Unified polls it after the reset (value: datasheet, unconfirmed)
        ],
        READ_ONLY,
    )
}

/// Bytes per aux burst for a 2-bit length code (datasheet, unconfirmed;
/// M5Unified uses code 0 for 1 byte and code 3 for its 8-byte data mode).
fn burst_len(code: u8) -> usize {
    [1, 2, 6, 8][(code & 0x03) as usize]
}

/// Sample period of an ODR code, 100 Hz × 2^(odr - 8) (datasheet,
/// unconfirmed; 0x08 = 100 Hz is BMI2_ACC_ODR_100HZ in the SensorAPI).
fn period_ns(conf: u8) -> Option<u64> {
    match conf & 0x0F {
        0 | 14 | 15 => None,
        odr @ 1..=8 => Some(10_000_000 << (8 - odr)),
        odr => Some(10_000_000 >> (odr - 8)),
    }
}

/// Rotate `v` by angle `a` (radians) about X, Y or Z (right-handed).
fn rot(axis: usize, a: f64, v: [f64; 3]) -> [f64; 3] {
    let (s, c) = a.sin_cos();
    let [x, y, z] = v;
    match axis {
        0 => [x, c * y - s * z, s * y + c * z],
        1 => [c * x + s * z, y, -s * x + c * z],
        _ => [c * x - s * y, s * x + c * y, z],
    }
}

fn le(v: i16) -> [u8; 2] {
    v.to_le_bytes()
}

fn scale(value: f64, lsb_per_unit: f64) -> i16 {
    (value * lsb_per_unit).round().clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

pub struct Bmi270 {
    regs: Regs8,
    /// The config upload: word pointer, a byte waiting for its pair, and
    /// which words were written.
    cfg_word: u16,
    cfg_half: bool,
    /// One bit per config word written. A fixed array, not a Vec: the part
    /// must not allocate when it is built (see `ili9342`'s note on the
    /// engine's part-worker.js).
    cfg_done: [u64; CONFIG_WORDS / 64],
    /// When each sensor's next sample is due (None while it is off).
    next_ns: [Option<u64>; 3],
    roll: f64,
    pitch: f64,
    yaw: f64,
    gyro: [f64; 3],
    mag: Bmm150,
}

impl Default for Bmi270 {
    fn default() -> Self {
        let mut c = Self {
            regs: reset_regs(),
            cfg_word: 0,
            cfg_half: false,
            cfg_done: [0; CONFIG_WORDS / 64],
            next_ns: [None; 3],
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            gyro: [0.0; 3],
            mag: Bmm150::default(),
        };
        c.update_field();
        c
    }
}

impl Bmi270 {
    fn reg(&self, r: u8) -> u8 {
        self.regs.mem[r as usize]
    }

    fn set(&mut self, r: u8, v: u8) {
        self.regs.mem[r as usize] = v;
    }

    /// The config is loaded: INTERNAL_STATUS message is "init ok".
    pub fn initialized(&self) -> bool {
        self.reg(INTERNAL_STATUS) & 0x0F == INIT_OK
    }

    /// The accel full scale in g, from ACC_RANGE.
    pub fn acc_range_g(&self) -> u32 {
        2 << (self.reg(ACC_RANGE) & 0x03)
    }

    fn gyr_range_dps(&self) -> f64 {
        match self.reg(GYR_RANGE) & 0x07 {
            r @ 0..=4 => 2000.0 / (1 << r) as f64,
            _ => 125.0, // reserved codes (datasheet, unconfirmed)
        }
    }

    /// Gravity's reaction on the sensor axes, in g.
    fn accel_g(&self) -> [f64; 3] {
        self.to_board([0.0, 0.0, 1.0])
    }

    /// A world vector (north, west, up) on the board's axes.
    fn to_board(&self, world: [f64; 3]) -> [f64; 3] {
        let (r, p, y) = (self.roll.to_radians(), self.pitch.to_radians(), self.yaw.to_radians());
        // Board to world is Rz(-yaw)·Ry(pitch)·Rx(roll); this is its inverse.
        rot(0, -r, rot(1, -p, rot(2, y, world)))
    }

    /// Give the BMM150 the field on its own axes: (x, -y, -z) of the board.
    fn update_field(&mut self) {
        let [x, y, z] = self.to_board(EARTH_FIELD_UT);
        self.mag.set_field([x, -y, -z]);
    }

    fn aux_ready(&self) -> bool {
        self.reg(IF_CONF) & IF_AUX_EN != 0 && self.reg(AUX_DEV_ID) >> 1 == BMM150_AUX_ADDR
    }

    /// Aux manual read: AUX_RD_ADDR was written.
    fn aux_manual_read(&mut self) {
        if !self.aux_ready() {
            return;
        }
        let len = burst_len(self.reg(AUX_IF_CONF) >> 2);
        let data = self.mag.read_burst(self.reg(AUX_RD_ADDR), len);
        self.regs.mem[AUX_DATA as usize..AUX_DATA as usize + len].copy_from_slice(&data);
    }

    /// Aux manual write: AUX_WR_ADDR was written.
    fn aux_manual_write(&mut self) {
        if self.aux_ready() {
            self.mag.write(self.reg(AUX_WR_ADDR), self.reg(AUX_WR_DATA));
        }
    }

    fn aux_data_mode(&self) -> bool {
        self.reg(AUX_IF_CONF) & AUX_MANUAL == 0 && self.aux_ready()
    }

    fn enabled(&self, s: Sensor) -> bool {
        let on = self.initialized() && self.reg(PWR_CTRL) & s.pwr_bit() != 0;
        match s {
            Sensor::Aux => on && self.aux_data_mode(),
            _ => on,
        }
    }

    fn soft_reset(&mut self) {
        self.regs = reset_regs();
        self.cfg_word = 0;
        self.cfg_half = false;
        self.cfg_done = [0; CONFIG_WORDS / 64];
        self.next_ns = [None; 3];
    }

    /// One byte the master wrote to register `r`. Returns false to stop
    /// taking bytes (after a soft reset).
    fn write_byte(&mut self, r: u8, v: u8) -> bool {
        match r {
            CMD => {
                if v == SOFT_RESET {
                    self.soft_reset();
                    return false;
                }
                // 0xB0 FIFO flush and the rest: nothing to do here.
            }
            INIT_DATA => {
                // Two bytes per word; the pointer stays on 0x5E.
                if self.cfg_half {
                    let word = self.cfg_word as usize;
                    if word < CONFIG_WORDS {
                        self.cfg_done[word / 64] |= 1 << (word % 64);
                    }
                    self.cfg_word = self.cfg_word.wrapping_add(1);
                }
                self.cfg_half = !self.cfg_half;
            }
            INIT_ADDR_0 | INIT_ADDR_1 => {
                self.set(r, v);
                self.cfg_word = (self.reg(INIT_ADDR_0) & 0x0F) as u16 | (self.reg(INIT_ADDR_1) as u16) << 4;
                self.cfg_half = false;
            }
            INIT_CTRL => {
                self.set(r, v);
                if v & 0x01 != 0 {
                    let full = self.cfg_done.iter().all(|&w| w == u64::MAX);
                    self.set(INTERNAL_STATUS, if full { INIT_OK } else { INIT_ERR });
                }
            }
            AUX_RD_ADDR => {
                self.set(r, v);
                if self.reg(AUX_IF_CONF) & AUX_MANUAL != 0 {
                    self.aux_manual_read();
                }
            }
            AUX_WR_ADDR => {
                self.set(r, v);
                if self.reg(AUX_IF_CONF) & AUX_MANUAL != 0 {
                    self.aux_manual_write();
                }
            }
            _ if READ_ONLY.contains(&r) => {}
            _ => self.set(r, v),
        }
        true
    }

    /// Data ready for every sensor whose sample time has come.
    fn tick(&mut self, now_ns: u64) {
        for (i, s) in SENSORS.into_iter().enumerate() {
            let period = period_ns(self.reg(s.conf()));
            let (true, Some(period)) = (self.enabled(s), period) else {
                self.next_ns[i] = None;
                continue;
            };
            match self.next_ns[i] {
                None => self.next_ns[i] = Some(now_ns + period),
                Some(due) if now_ns >= due => {
                    self.set(STATUS, self.reg(STATUS) | s.drdy());
                    self.set(INT_STATUS_1, self.reg(INT_STATUS_1) | s.drdy());
                    self.next_ns[i] = Some(due + period * ((now_ns - due) / period + 1));
                }
                Some(_) => {}
            }
        }
    }

    /// A fresh sample of everything that is on, and the sensortime.
    fn sample(&mut self, now_ns: u64) {
        let ticks = (now_ns * 2 / SENSORTIME_NS_X2) as u32;
        let st = ticks.to_le_bytes();
        self.regs.mem[SENSORTIME as usize..SENSORTIME as usize + 3].copy_from_slice(&st[..3]);
        if self.enabled(Sensor::Acc) {
            let lsb_per_g = 32768.0 / self.acc_range_g() as f64;
            let a = self.accel_g();
            for (i, v) in a.into_iter().enumerate() {
                let at = ACC_DATA as usize + 2 * i;
                self.regs.mem[at..at + 2].copy_from_slice(&le(scale(v, lsb_per_g)));
            }
        }
        if self.enabled(Sensor::Gyr) {
            let lsb_per_dps = 32768.0 / self.gyr_range_dps();
            for (i, v) in self.gyro.into_iter().enumerate() {
                let at = GYR_DATA as usize + 2 * i;
                self.regs.mem[at..at + 2].copy_from_slice(&le(scale(v, lsb_per_dps)));
            }
        }
        if self.initialized() {
            let t = if self.reg(PWR_CTRL) & 0x08 != 0 { scale(TEMP_C - 23.0, 512.0) } else { i16::MIN };
            self.regs.mem[TEMPERATURE as usize..TEMPERATURE as usize + 2].copy_from_slice(&le(t));
        }
        if self.enabled(Sensor::Aux) {
            let len = burst_len(self.reg(AUX_IF_CONF));
            let data = self.mag.read_burst(self.reg(AUX_RD_ADDR), len);
            self.regs.mem[AUX_DATA as usize..AUX_DATA as usize + len].copy_from_slice(&data);
        }
    }

    /// One byte read from register `r`, with its side effects.
    fn read_byte(&mut self, r: u8) -> u8 {
        let v = match r {
            FIFO_DATA => 0x80, // empty FIFO (datasheet, unconfirmed); no FIFO is modelled
            CMD => 0x00,
            _ => self.reg(r),
        };
        if r == INT_STATUS_0 || r == INT_STATUS_1 {
            self.set(r, 0); // clear on read
        }
        for s in SENSORS {
            if s.data().contains(&r) {
                // Reading the data clears its STATUS drdy bit (datasheet, unconfirmed).
                self.set(STATUS, self.reg(STATUS) & !s.drdy());
            }
        }
        v
    }
}

impl Chip for Bmi270 {
    fn transfer(&mut self, write: &[u8], read_len: usize, continues: bool, now_ns: u64) -> Option<Vec<u8>> {
        if let Some((&reg, data)) = write.split_first() {
            self.regs.at = reg;
            for &b in data {
                let r = self.regs.at;
                if !self.write_byte(r, b) {
                    break;
                }
                if r != INIT_DATA {
                    self.regs.at = r.wrapping_add(1);
                }
            }
        }
        if !continues {
            self.tick(now_ns);
            self.sample(now_ns);
        }
        let out = (0..read_len)
            .map(|_| {
                let r = self.regs.at;
                let v = self.read_byte(r);
                if r != FIFO_DATA {
                    self.regs.at = r.wrapping_add(1);
                }
                v
            })
            .collect();
        Some(out)
    }

    fn input(&mut self, name: &str, value: f64) -> bool {
        match name {
            "roll" => self.roll = value,
            "pitch" => self.pitch = value,
            "yaw" => self.yaw = value,
            "gyro_x" => self.gyro[0] = value,
            "gyro_y" => self.gyro[1] = value,
            "gyro_z" => self.gyro[2] = value,
            _ => return false,
        }
        self.update_field();
        true
    }

    fn state(&self, state: State, _now_ns: u64) -> State {
        state
            .set("imu_roll", self.roll)
            .set("imu_pitch", self.pitch)
            .set("imu_yaw", self.yaw)
            .set("imu_init", self.initialized() as u8)
            .set("imu_acc_range_g", self.acc_range_g())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    /// A stand-in for Bosch's 8 KB config file (the content is not checked).
    fn config() -> Vec<u8> {
        (0..8192u32).map(|i| (i * 7 + 3) as u8).collect()
    }

    fn t(c: &mut Bmi270, write: &[u8], read_len: usize, now: u64) -> Vec<u8> {
        c.transfer(write, read_len, false, now).expect("ACK")
    }

    fn rd(c: &mut Bmi270, reg: u8, now: u64) -> u8 {
        t(c, &[reg], 1, now)[0]
    }

    fn i16s(b: &[u8]) -> Vec<i16> {
        b.chunks(2).map(|p| i16::from_le_bytes([p[0], p[1]])).collect()
    }

    /// tr_board's motion_init, byte for byte (tr_sensors.c).
    fn tr_board_init(c: &mut Bmi270, now: u64) {
        assert_eq!(rd(c, CHIP_ID, now), 0x24);
        t(c, &[CMD, SOFT_RESET], 0, now);
        t(c, &[PWR_CONF, 0x00], 0, now);
        t(c, &[INIT_CTRL, 0x00], 0, now);
        let cfg = config();
        for i in (0..cfg.len()).step_by(64) {
            t(c, &[INIT_ADDR_0, ((i / 2) & 0x0F) as u8, ((i / 2) >> 4) as u8], 0, now);
            let mut buf = vec![INIT_DATA];
            buf.extend_from_slice(&cfg[i..i + 64]);
            t(c, &buf, 0, now);
        }
        t(c, &[INIT_CTRL, 0x01], 0, now);
        let mut status = 0;
        for i in 0..20 {
            status = rd(c, INTERNAL_STATUS, now + (i + 1) * 10 * MS);
            if status & 1 != 0 {
                break;
            }
        }
        assert_eq!(status & 0x01, 0x01, "tr_board needs bit 0");
        t(c, &[PWR_CTRL, 0x0E], 0, now);
    }

    /// M5Unified's BMI270_Class::begin, up to the BMM150 (BMI270_Class.inl).
    fn m5unified_init(c: &mut Bmi270, now: u64) -> bool {
        assert_eq!(rd(c, CHIP_ID, now), 0x24);
        assert_eq!(rd(c, CHIP_ID, now), 0x24, "read twice");
        t(c, &[CMD, SOFT_RESET], 0, now);
        assert_ne!(rd(c, PWR_CONF, now + 3 * MS), 0, "M5Unified polls 0x7C until nonzero");
        t(c, &[PWR_CONF, 0x00], 0, now);
        t(c, &[INIT_ADDR_0, 0x00, 0x00], 0, now);
        let mut buf = vec![INIT_DATA];
        buf.extend(config());
        assert_eq!(buf.len(), 8193);
        t(c, &buf, 0, now);
        t(c, &[INIT_CTRL, 0x01], 0, now);
        t(c, &[0x58, 0xFF], 0, now);
        assert_ne!(rd(c, INTERNAL_STATUS, now + MS), 0);
        // auxSetupMode(0x10)
        t(c, &[IF_CONF, 0x20], 0, now);
        t(c, &[PWR_CONF, 0x00], 0, now);
        t(c, &[PWR_CTRL, 0x0E], 0, now);
        t(c, &[AUX_IF_CONF, 0x80], 0, now);
        t(c, &[AUX_DEV_ID, 0x10 << 1], 0, now);
        // auxWriteRegister8(0x4B, 0x83)
        t(c, &[AUX_WR_DATA, 0x83], 0, now);
        t(c, &[AUX_WR_ADDR, 0x4B], 0, now);
        assert_eq!(rd(c, STATUS, now) & 0x04, 0, "aux not busy");
        // auxReadRegister8(0x40), twice
        let mut who = 0;
        for _ in 0..2 {
            t(c, &[AUX_IF_CONF, 0x80], 0, now);
            t(c, &[AUX_RD_ADDR, 0x40], 0, now);
            assert_eq!(rd(c, STATUS, now) & 0x04, 0);
            who = rd(c, AUX_DATA, now);
        }
        if who != 0x32 {
            return false;
        }
        t(c, &[AUX_WR_DATA, 0x38], 0, now); // BMM150 normal, 30 Hz
        t(c, &[AUX_WR_ADDR, 0x4C], 0, now);
        t(c, &[AUX_IF_CONF, 0x4F], 0, now);
        t(c, &[AUX_RD_ADDR, 0x42], 0, now);
        t(c, &[PWR_CTRL, 0x0F], 0, now);
        true
    }

    /// M5Unified's getImuRawData: 0x1D, then 20 bytes from 0x04 if a drdy
    /// bit is set. Returns (intstat, accel, gyro, mag) with its decoding.
    fn m5unified_read(c: &mut Bmi270, now: u64) -> (u8, [i16; 3], [i16; 3], [i16; 3]) {
        let intstat = rd(c, INT_STATUS_1, now);
        if intstat & 0xE0 == 0 {
            return (intstat, [0; 3], [0; 3], [0; 3]);
        }
        let b = i16s(&t(c, &[AUX_DATA], 20, now));
        let mag = [b[0] >> 2, b[1] >> 2, ((b[2] as u16) & 0xFFFE) as i16];
        (intstat, [b[4], b[5], b[6]], [b[7], b[8], b[9]], mag)
    }

    /// The StackChan firmware: bmi270_init, then BMI270::begin's accel and
    /// gyro config and sensor enable (bmi2.c, bmi270.c, bmi270.cpp), with
    /// aps_status tracked as the SensorAPI does.
    fn bosch_init(c: &mut Bmi270, now: u64) -> u64 {
        let mut now = now;
        let mut aps = true; // bmi2_sec_init: the chip starts in power save
        // bmi2_set_regs: single-byte writes to reg + n while aps is on.
        let set_regs = |c: &mut Bmi270, aps: bool, reg: u8, data: &[u8], now: u64| {
            if aps {
                for (n, &b) in data.iter().enumerate() {
                    t(c, &[reg + n as u8, b], 0, now);
                }
            } else {
                let mut buf = vec![reg];
                buf.extend_from_slice(data);
                t(c, &buf, 0, now);
            }
        };
        // bmi2_set_adv_power_save: read-modify-write of 0x7C bit 0.
        let set_aps = |c: &mut Bmi270, aps: &mut bool, on: bool, now: u64| {
            let v = (rd(c, PWR_CONF, now) & !1) | on as u8;
            set_regs(c, *aps, PWR_CONF, &[v], now);
            *aps = on;
        };
        assert_eq!(rd(c, CHIP_ID, now), 0x24, "else BMI2_E_DEV_NOT_FOUND");
        // bmi2_soft_reset: must ACK.
        c.transfer(&[CMD, SOFT_RESET], 0, false, now).expect("soft reset ACK");
        now += 2 * MS; // and aps_status stays ENABLE, as after the reset
        // write_config_file
        set_aps(c, &mut aps, false, now);
        let v = rd(c, INIT_CTRL, now) & !1;
        set_regs(c, aps, INIT_CTRL, &[v], now);
        let cfg = config();
        for index in (0..cfg.len()).step_by(32) {
            set_regs(c, aps, INIT_ADDR_0, &[((index / 2) & 0x0F) as u8, ((index / 2) >> 4) as u8], now);
            set_regs(c, aps, INIT_DATA, &cfg[index..index + 32], now);
        }
        let v = rd(c, INIT_CTRL, now) | 1;
        set_regs(c, aps, INIT_CTRL, &[v], now);
        set_aps(c, &mut aps, true, now);
        // bmi2_get_internal_status: 20 ms, then one read.
        now += 20 * MS;
        assert_eq!(rd(c, INTERNAL_STATUS, now) & 0x0F, 1, "else BMI2_E_CONFIG_LOAD");
        // bmi2_get_gyro_cross_sense: feature page, then 16 bytes from 0x30.
        set_regs(c, aps, 0x2F, &[0x00], now);
        assert_eq!(t(c, &[0x30], 16, now).len(), 16);
        // Accel: get config, then set it with aps off and check EVENT.
        let acc = t(c, &[ACC_CONF], 2, now);
        assert_eq!(acc, [0xA8, 0x02]);
        set_aps(c, &mut aps, false, now);
        set_regs(c, aps, ACC_CONF, &[0xA8, 0x00], now); // 100 Hz, avg4, perf, ±2 g
        assert_eq!(rd(c, EVENT, now) & 0x1C, 0, "else BMI2_E_ACC_INVALID_CFG");
        set_aps(c, &mut aps, true, now);
        // Gyro: ois_range comes back from the register as read.
        let gyr = t(c, &[GYR_CONF], 2, now);
        set_aps(c, &mut aps, false, now);
        set_regs(c, aps, GYR_CONF, &[0xE9, gyr[1] & 0x08], now); // 100 Hz, normal, perf, ±2000
        assert_eq!(rd(c, EVENT, now) & 0x1C, 0, "else BMI2_E_GYRO_INVALID_CFG");
        set_aps(c, &mut aps, true, now);
        // bmi2_sensor_enable: read-modify-write of 0x7D, acc and gyr bits.
        let v = rd(c, PWR_CTRL, now) | 0x06;
        set_regs(c, aps, PWR_CTRL, &[v], now);
        assert!(aps, "the SensorAPI leaves power save on");
        now
    }

    #[test]
    fn chip_id_and_probe() {
        let mut c = Bmi270::default();
        assert_eq!(c.transfer(&[], 0, false, 0), Some(vec![]), "a probe is ACKed");
        assert_eq!(rd(&mut c, CHIP_ID, 0), 0x24);
        assert_eq!(rd(&mut c, CHIP_ID, 0), 0x24);
        t(&mut c, &[CHIP_ID, 0x00], 0, 0);
        assert_eq!(rd(&mut c, CHIP_ID, 0), 0x24, "read only");
    }

    #[test]
    fn tr_board_upload_in_64_byte_chunks_and_reads_8g() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0);
        assert!(c.initialized());
        // tr_board reads 12 bytes from 0x0C as ±8 g / ±2000 °/s.
        let v = i16s(&t(&mut c, &[ACC_DATA], 12, 300 * MS));
        assert_eq!(v, [0, 0, 4096, 0, 0, 0], "flat: +1 g on Z at ±8 g");
        let temp = i16s(&t(&mut c, &[TEMPERATURE], 2, 300 * MS))[0];
        assert_eq!(23.0 + temp as f64 / 512.0, 25.0);
    }

    #[test]
    fn tr_board_sleep_and_wake() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0);
        t(&mut c, &[PWR_CTRL, 0x00], 0, 0);
        t(&mut c, &[PWR_CONF, 0x01], 0, 0);
        c.input("roll", 90.0);
        let v = i16s(&t(&mut c, &[ACC_DATA], 6, 400 * MS));
        assert_eq!(v, [0, 0, 4096], "off: the last sample stays");
        t(&mut c, &[PWR_CONF, 0x00], 0, 0);
        t(&mut c, &[PWR_CTRL, 0x0E], 0, 0);
        let v = i16s(&t(&mut c, &[ACC_DATA], 6, 400 * MS));
        assert_eq!(v, [0, 4096, 0], "on again: roll 90 puts 1 g on Y");
    }

    #[test]
    fn m5unified_bmm150_bring_up_and_data_mode() {
        let mut c = Bmi270::default();
        assert!(m5unified_init(&mut c, 0), "BMM150 found through aux");
        let (intstat, _, _, _) = m5unified_read(&mut c, 0);
        assert_eq!(intstat & 0xE0, 0, "no sample yet");
        let (intstat, acc, gyro, mag) = m5unified_read(&mut c, 50 * MS);
        assert_eq!(intstat & 0xE0, 0xE0, "acc, gyr and aux ready");
        assert_eq!(acc, [0, 0, 4096], "M5Unified assumes ±8 g: az = +1 g");
        assert_eq!(gyro, [0, 0, 0]);
        // M5Unified: mag × 10·4912/32760 mG, then Y and Z inverted on CoreS3.
        let ut = |v: i16| v as f64 * 4912.0 / 32760.0;
        let (mx, my, mz) = (ut(mag[0]), -ut(mag[1]), -ut(mag[2]));
        assert!((mx - 40.0).abs() < 0.35 && my.abs() < 0.35 && (mz + 30.0).abs() < 0.35, "{mx} {my} {mz}");
    }

    #[test]
    fn m5unified_heading_follows_yaw() {
        let mut c = Bmi270::default();
        assert!(m5unified_init(&mut c, 0));
        for yaw in [0.0, 45.0, 90.0, -120.0] {
            c.input("yaw", yaw);
            let (_, _, _, mag) = m5unified_read(&mut c, 1000 * MS + yaw.abs() as u64 * 100 * MS);
            let heading = (-(mag[1] as f64)).atan2(mag[0] as f64).to_degrees();
            assert!((heading - yaw).abs() < 1.0, "yaw {yaw}: heading {heading}");
        }
    }

    #[test]
    fn aux_needs_aux_enabled_and_the_bmm150_address() {
        let mut c = Bmi270::default();
        assert!(m5unified_init(&mut c, 0));
        t(&mut c, &[AUX_IF_CONF, 0x80], 0, 0);
        t(&mut c, &[AUX_DEV_ID, 0x13 << 1], 0, 0);
        t(&mut c, &[AUX_RD_ADDR, 0x40], 0, 0);
        assert_ne!(rd(&mut c, AUX_DATA, 0), 0x32, "nobody at 0x13");
        t(&mut c, &[AUX_DEV_ID, 0x10 << 1], 0, 0);
        t(&mut c, &[AUX_RD_ADDR, 0x40], 0, 0);
        assert_eq!(rd(&mut c, AUX_DATA, 0), 0x32);
    }

    #[test]
    fn aux_manual_read_burst_lengths() {
        let mut c = Bmi270::default();
        assert!(m5unified_init(&mut c, 0));
        t(&mut c, &[AUX_IF_CONF, 0x80 | (2 << 2)], 0, 0); // 6 bytes
        t(&mut c, &[AUX_RD_ADDR, 0x42], 0, 0);
        let b = i16s(&t(&mut c, &[AUX_DATA], 6, 0));
        assert_eq!(b[0] >> 3, (40.0 / 0.3f64).round() as i16, "X raw");
        assert_eq!(b[2] >> 1, (30.0 / 0.3f64).round() as i16, "Z raw: the BMM150's Z is the board's -Z");
    }

    #[test]
    fn bosch_init_upload_status_event_and_config_readback() {
        let mut c = Bmi270::default();
        let now = bosch_init(&mut c, 0);
        assert_eq!(t(&mut c, &[ACC_CONF], 4, now), [0xA8, 0x00, 0xE9, 0x08]);
        assert_eq!(rd(&mut c, PWR_CTRL, now), 0x06);
        assert_eq!(rd(&mut c, PWR_CONF, now), 0x03);
        // bmi2_get_sensor_data: 24 bytes from 0x03, 10 ms later (100 Hz).
        let now = now + 10 * MS;
        let d = t(&mut c, &[STATUS], 24, now);
        assert_eq!(d[0] & 0xC0, 0xC0, "drdy acc and gyr");
        assert_eq!(d[0] & 0x10, 0x10, "cmd_rdy");
        let v = i16s(&d[9..21]);
        assert_eq!(v, [0, 0, 16384, 0, 0, 0], "±2 g: 1 g is 16384");
        let st = d[21] as u64 | (d[22] as u64) << 8 | (d[23] as u64) << 16;
        assert_eq!(st, now * 2 / 78125, "sensortime in 39.0625 µs ticks");
        assert_eq!(d[0] & 0xC0, 0xC0);
        assert_eq!(rd(&mut c, STATUS, now) & 0xC0, 0, "reading the data cleared STATUS drdy");
    }

    #[test]
    fn bosch_gyro_scaling_at_2000_dps() {
        let mut c = Bmi270::default();
        let now = bosch_init(&mut c, 0);
        c.input("gyro_x", 100.0);
        c.input("gyro_z", -2500.0);
        let d = t(&mut c, &[STATUS], 24, now + 10 * MS);
        let g = i16s(&d[15..21]);
        assert_eq!(g, [1638, 0, i16::MIN], "16.384 LSB per °/s, clamped");
    }

    #[test]
    fn upload_in_single_byte_writes() {
        // Every write one byte: 0x5B and 0x5C apart, then [0x5E, b] each.
        let mut c = Bmi270::default();
        let cfg = config();
        for index in (0..cfg.len()).step_by(32) {
            t(&mut c, &[INIT_ADDR_0, ((index / 2) & 0x0F) as u8], 0, 0);
            t(&mut c, &[INIT_ADDR_1, ((index / 2) >> 4) as u8], 0, 0);
            for &b in &cfg[index..index + 32] {
                t(&mut c, &[INIT_DATA, b], 0, 0);
            }
        }
        t(&mut c, &[INIT_CTRL, 0x01], 0, 0);
        assert_eq!(rd(&mut c, INTERNAL_STATUS, 0), 0x01);
    }

    #[test]
    fn config_writes_stay_on_0x5e() {
        let mut c = Bmi270::default();
        t(&mut c, &[INIT_DATA, 1, 2, 3, 4, 5, 6], 0, 0);
        assert_eq!(t(&mut c, &[0x5F], 2, 0), [0, 0], "0x5F and on untouched");
        assert_eq!(c.cfg_word, 3, "three words");
    }

    #[test]
    fn incomplete_upload_is_an_init_error() {
        let mut c = Bmi270::default();
        let mut buf = vec![INIT_DATA];
        buf.extend(&config()[..8190]); // one word short
        t(&mut c, &[INIT_ADDR_0, 0, 0], 0, 0);
        t(&mut c, &buf, 0, 0);
        t(&mut c, &[INIT_CTRL, 0x01], 0, 0);
        assert_eq!(rd(&mut c, INTERNAL_STATUS, 0), INIT_ERR);
        assert!(!c.initialized());
        t(&mut c, &[PWR_CTRL, 0x0E], 0, 0);
        assert_eq!(t(&mut c, &[ACC_DATA], 6, 100 * MS), [0; 6], "no data without the config");
    }

    #[test]
    fn scaling_at_2g_and_8g() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0);
        c.input("pitch", 30.0); // ax = -sin 30° = -0.5 g
        let ax = |c: &mut Bmi270| i16s(&t(c, &[ACC_DATA], 2, 0))[0];
        assert_eq!(ax(&mut c), -2048, "±8 g: 4096 LSB/g");
        t(&mut c, &[ACC_RANGE, 0x00], 0, 0);
        assert_eq!(ax(&mut c), -8192, "±2 g: 16384 LSB/g");
        assert_eq!(c.state(State::new(), 0).get("imu_acc_range_g"), Some("2"));
        t(&mut c, &[ACC_RANGE, 0x03], 0, 0);
        assert_eq!(ax(&mut c), -1024, "±16 g");
    }

    #[test]
    fn gyro_range_scaling() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0);
        c.input("gyro_y", 100.0);
        let gy = |c: &mut Bmi270| i16s(&t(c, &[GYR_DATA + 2], 2, 0))[0];
        assert_eq!(gy(&mut c), 1638, "±2000 °/s");
        t(&mut c, &[GYR_RANGE, 0x04], 0, 0);
        assert_eq!(gy(&mut c), 26214, "±125 °/s: 262.144 LSB per °/s");
    }

    #[test]
    fn roll_and_pitch_come_back_from_the_accel() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0);
        t(&mut c, &[ACC_RANGE, 0x00], 0, 0);
        for (roll, pitch) in [(20.0, -35.0), (-60.0, 10.0), (170.0, 45.0)] {
            c.input("roll", roll);
            c.input("pitch", pitch);
            let a: Vec<f64> = i16s(&t(&mut c, &[ACC_DATA], 6, 0)).into_iter().map(|v| v as f64).collect();
            let r = a[1].atan2(a[2]).to_degrees();
            let p = (-a[0]).atan2(a[1].hypot(a[2])).to_degrees();
            assert!((r - roll).abs() < 0.1 && (p - pitch).abs() < 0.1, "{roll},{pitch} -> {r},{p}");
        }
    }

    #[test]
    fn int_status_1_clears_on_read_and_comes_back_at_the_odr() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0); // acc 100 Hz, gyr 200 Hz (reset values)
        assert_eq!(rd(&mut c, INT_STATUS_1, 1000 * MS) & 0xC0, 0xC0, "samples were due");
        assert_eq!(rd(&mut c, INT_STATUS_1, 1004 * MS) & 0xC0, 0, "not yet");
        assert_eq!(rd(&mut c, INT_STATUS_1, 1005 * MS) & 0xC0, 0x40, "gyro at 200 Hz");
        assert_eq!(rd(&mut c, INT_STATUS_1, 1005 * MS), 0, "cleared by the read");
        assert_eq!(rd(&mut c, INT_STATUS_1, 1010 * MS) & 0xC0, 0xC0, "both at 10 ms");
        assert_eq!(rd(&mut c, INT_STATUS_1, 1010 * MS), 0);
        t(&mut c, &[ACC_CONF, 0xAA], 0, 1010 * MS); // 400 Hz
        assert_eq!(rd(&mut c, INT_STATUS_1, 1020 * MS) & 0x80, 0x80);
        assert_eq!(rd(&mut c, INT_STATUS_1, 1022 * MS) & 0x80, 0, "still the old due time");
        assert_eq!(rd(&mut c, INT_STATUS_1, 1023 * MS) & 0x80, 0x80, "then every 2.5 ms");
    }

    #[test]
    fn data_only_for_enabled_sensors() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0);
        t(&mut c, &[PWR_CTRL, 0x04], 0, 0); // accel only
        c.input("gyro_x", 50.0);
        let v = i16s(&t(&mut c, &[ACC_DATA], 12, 0));
        assert_eq!(v, [0, 0, 4096, 0, 0, 0], "gyro off: stays 0");
        assert_eq!(i16s(&t(&mut c, &[TEMPERATURE], 2, 0)), [i16::MIN], "temperature off: 0x8000");
    }

    #[test]
    fn soft_reset_restores_defaults() {
        let mut c = Bmi270::default();
        tr_board_init(&mut c, 0);
        t(&mut c, &[ACC_RANGE, 0x00, 0x42], 0, 0);
        t(&mut c, &[IF_CONF, 0x20], 0, 0);
        assert_eq!(c.transfer(&[CMD, SOFT_RESET], 0, false, 0), Some(vec![]), "ACKed");
        assert_eq!(t(&mut c, &[ACC_CONF], 4, 0), [0xA8, 0x02, 0xA9, 0x08]);
        assert_eq!(rd(&mut c, INTERNAL_STATUS, 0), 0x00);
        assert_eq!(rd(&mut c, PWR_CTRL, 0), 0x00);
        assert_eq!(rd(&mut c, PWR_CONF, 0), 0x03);
        assert_eq!(rd(&mut c, IF_CONF, 0), 0x00);
        assert_eq!(rd(&mut c, EVENT, 0), 0x01);
        let s = c.state(State::new(), 0);
        assert_eq!(s.get("imu_init"), Some("0"));
        assert_eq!(s.get("imu_acc_range_g"), Some("8"));
        // The upload is forgotten too: INIT_CTRL alone is not enough.
        t(&mut c, &[INIT_CTRL, 0x01], 0, 0);
        assert_eq!(rd(&mut c, INTERNAL_STATUS, 0), INIT_ERR);
    }

    #[test]
    fn split_reads_continue_from_the_pointer() {
        let mut c = Bmi270::default();
        let now = bosch_init(&mut c, 0);
        c.input("roll", 30.0);
        c.input("gyro_z", 10.0);
        let whole = t(&mut c, &[STATUS], 24, now + 10 * MS);
        let mut c2 = Bmi270::default();
        let now = bosch_init(&mut c2, 0);
        c2.input("roll", 30.0);
        c2.input("gyro_z", 10.0);
        let mut parts = t(&mut c2, &[STATUS], 5, now + 10 * MS);
        c2.input("roll", -80.0); // changes between pieces must not tear the sample
        parts.extend(c2.transfer(&[], 7, true, now + 11 * MS).unwrap());
        parts.extend(c2.transfer(&[], 12, true, now + 12 * MS).unwrap());
        assert_eq!(parts, whole);
    }

    #[test]
    fn fifo_data_traps_the_read_pointer() {
        let mut c = Bmi270::default();
        assert_eq!(t(&mut c, &[FIFO_DATA], 3, 0), [0x80, 0x80, 0x80]);
    }

    #[test]
    fn inputs_and_state() {
        let mut c = Bmi270::default();
        for (name, v) in [("roll", 1.5), ("pitch", -2.0), ("yaw", 270.0), ("gyro_x", 1.0), ("gyro_y", 2.0), ("gyro_z", 3.0)] {
            assert!(c.input(name, v), "{name}");
        }
        assert!(!c.input("battery", 1.0));
        let s = c.state(State::new(), 0);
        assert_eq!(s.get("imu_roll"), Some("1.5"));
        assert_eq!(s.get("imu_pitch"), Some("-2"));
        assert_eq!(s.get("imu_yaw"), Some("270"));
        assert_eq!(s.get("imu_init"), Some("0"));
        assert_eq!(s.get("imu_acc_range_g"), Some("8"));
    }

    #[test]
    fn through_the_part_on_the_bus() {
        use crate::tests::{i2c, part};
        use esp_part::Device;
        let mut p = part();
        assert_eq!(i2c(&mut p, 0x69, &[], 0), Some(vec![]), "the StackChan bus scan finds it");
        assert_eq!(i2c(&mut p, 0x69, &[CHIP_ID], 1), Some(vec![0x24]));
        p.on_input("set:pitch=12.5");
        assert_eq!(p.state().unwrap().get("imu_pitch"), Some("12.5"));
    }
}
