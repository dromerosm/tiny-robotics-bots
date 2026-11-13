//! StackChan's body as a virtual part for esp-emu: the PY32 IO expander that
//! switches the servo power, and the two FEETECH SCS0009 head servos on the
//! serial bus.
//!
//! The firmware talks to it as it talks to the robot:
//!
//! - I2C, address 0x6F: the PY32. Register 0x02 reads the version (0x41, as on
//!   our robot). Bit 0 of register 0x05 (with bit 0 of 0x03, "output") is
//!   VM_EN, the servo power. Every other register only stores what is written.
//! - UART, 1 Mbaud: the servos, ID 1 (yaw) and ID 2 (pitch). They speak the
//!   SCS protocol: `FF FF id len inst params... checksum`. They answer only
//!   while VM_EN is on, as on the robot.
//!
//! The protocol and the register map follow FEETECH's SCSCL driver, as used by
//! M5Stack's StackChan-BSP. A move takes the time the firmware asks for, so
//! the head turns in emulated time. The page reads the angles from `state()`.

use esp_part::prelude::*;

/// The part's own pin names. The board wires them, for StackChan:
/// `{ TX: 7, RX: 6, SDA: 12, SCL: 11 }`. The part's TX goes to the chip's RX.
const TX: &str = "TX";
const RX: &str = "RX";
const SDA: &str = "SDA";
const SCL: &str = "SCL";

// ---------------------------------------------------------------- PY32

const PY32_ADDR: u32 = 0x6F;
const PY32_VERSION_REG: usize = 0x02;
const PY32_VERSION: u8 = 0x41;
const PY32_MODE_L: usize = 0x03; // bit set: the pin is an output
const PY32_OUT_L: usize = 0x05;
const VM_EN: u8 = 0x01;
// The 12 side LEDs: colors in LED memory (RGB565, low byte first), shown when
// bit 6 of LED_CFG is written.
const PY32_LED_CFG: usize = 0x24;
const PY32_LED_RAM: usize = 0x30;
const LED_SHOW: u8 = 0x40;
const LEDS: usize = 12;

// ---------------------------------------------------------------- servos

const YAW: u8 = 1;
const PITCH: u8 = 2;
const BROADCAST: u8 = 0xFE;

const INST_PING: u8 = 0x01;
const INST_READ: u8 = 0x02;
const INST_WRITE: u8 = 0x03;
const INST_REG_WRITE: u8 = 0x04;
const INST_ACTION: u8 = 0x05;
const INST_SYNC_WRITE: u8 = 0x83;

const REG_ID: usize = 5;
const REG_TORQUE: usize = 40;
const REG_GOAL: usize = 42; // position, time (ms), speed: 2 bytes each, high byte first
const REG_POSITION: usize = 56; // position, speed, load: 2 bytes each; voltage, temperature
const REG_MOVING: usize = 66;
const MEM: usize = 70;

const MAX_RAW: i32 = 1023;
/// The fastest the servo turns when no time and no speed are given, in steps
/// per second: about 0.15 s for 60 degrees (192 steps).
const MAX_SPEED: f64 = 1280.0;
/// What the servos report: 5.0 V and 30 degrees C.
const VOLTS_X10: u8 = 50;
const CELSIUS: u8 = 30;

/// Where the firmware's zero is by default (StackChan-BSP, tr_board), and the
/// scale: 3.2 steps per degree. Only used for the angles in `state()`.
const ZERO_YAW: i32 = 460;
const ZERO_PITCH: i32 = 620;
const STEPS_PER_DEG: f64 = 3.2;

struct Servo {
    id: u8,
    mem: [u8; MEM],
    /// The move under way: from `from` to `to`, starting at `t0` (ns), for `dur` ns.
    from: f64,
    to: f64,
    t0: u64,
    dur: u64,
    /// A REG_WRITE that waits for ACTION.
    pending: Option<(usize, Vec<u8>)>,
}

impl Servo {
    fn new(id: u8, raw: i32) -> Self {
        let mut mem = [0u8; MEM];
        mem[REG_ID] = id;
        Self { id, mem, from: raw as f64, to: raw as f64, t0: 0, dur: 0, pending: None }
    }

    fn torque(&self) -> bool {
        self.mem[REG_TORQUE] != 0
    }

    fn position(&self, now: u64) -> f64 {
        if self.dur == 0 || now >= self.t0 + self.dur {
            return self.to;
        }
        let k = now.saturating_sub(self.t0) as f64 / self.dur as f64;
        self.from + (self.to - self.from) * k
    }

    fn moving(&self, now: u64) -> bool {
        self.dur > 0 && now < self.t0 + self.dur && self.from != self.to
    }

    /// Stop where it is (torque off, or power off).
    fn hold_here(&mut self, now: u64) {
        let p = self.position(now);
        self.from = p;
        self.to = p;
        self.dur = 0;
    }

    fn read(&self, addr: usize, n: usize, now: u64) -> Vec<u8> {
        let mut mem = self.mem;
        let raw = self.position(now).round() as u16;
        let speed = if self.moving(now) {
            ((self.to - self.from).abs() / (self.dur as f64 / 1e9)).round() as u16
        } else {
            0
        };
        mem[REG_POSITION..REG_POSITION + 2].copy_from_slice(&raw.to_be_bytes());
        mem[REG_POSITION + 2..REG_POSITION + 4].copy_from_slice(&speed.to_be_bytes());
        mem[REG_POSITION + 4] = 0; // load
        mem[REG_POSITION + 5] = 0;
        mem[REG_POSITION + 6] = VOLTS_X10;
        mem[REG_POSITION + 7] = CELSIUS;
        mem[REG_MOVING] = self.moving(now) as u8;
        (addr..addr + n).map(|a| mem.get(a).copied().unwrap_or(0)).collect()
    }

    fn write(&mut self, addr: usize, data: &[u8], now: u64) {
        let torque_was = self.torque();
        for (i, &b) in data.iter().enumerate() {
            if let Some(m) = self.mem.get_mut(addr + i) {
                *m = b;
            }
        }
        let touches = |reg: usize| addr <= reg + 1 && reg < addr + data.len();
        if touches(REG_GOAL) {
            self.start_move(now);
        } else if torque_was && !self.torque() {
            self.hold_here(now);
        }
    }

    /// A goal was written. Writing a goal also turns the torque on, as the
    /// servo does: tr_board never writes the torque register before a move.
    fn start_move(&mut self, now: u64) {
        let word = |r: usize| u16::from_be_bytes([self.mem[r], self.mem[r + 1]]);
        let goal = (word(REG_GOAL) as i32).clamp(0, MAX_RAW) as f64;
        let ms = word(REG_GOAL + 2) as u64;
        let speed = word(REG_GOAL + 4) as f64;
        let here = self.position(now);
        self.mem[REG_TORQUE] = 1;
        let dist = (goal - here).abs();
        self.dur = if ms > 0 {
            ms * 1_000_000
        } else {
            let sps = if speed > 0.0 { speed.min(MAX_SPEED) } else { MAX_SPEED };
            (dist / sps * 1e9) as u64
        };
        self.from = here;
        self.to = goal;
        self.t0 = now;
    }

    fn degrees(&self, now: u64) -> f64 {
        let zero = if self.id == YAW { ZERO_YAW } else { ZERO_PITCH };
        (self.position(now) - zero as f64) / STEPS_PER_DEG
    }
}

struct Body {
    tx: Option<u32>,
    rx: Option<u32>,
    sda: Option<u32>,
    scl: Option<u32>,
    py32: [u8; 256],
    py32_at: u8,
    servos: [Servo; 2],
    /// Emulated time, from the last frame.
    now: u64,
    /// Bytes from the chip not parsed yet, and answers not sent yet.
    rx_buf: Vec<u8>,
    tx_buf: Vec<u8>,
    powered: bool,
    /// The colors the LEDs show, as RGB565, from the last show.
    leds: [u16; LEDS],
}

impl Default for Body {
    fn default() -> Self {
        let mut py32 = [0u8; 256];
        py32[PY32_VERSION_REG] = PY32_VERSION;
        // The head starts looking ahead: yaw 0, pitch 45 degrees.
        let pitch = ZERO_PITCH + (45.0 * STEPS_PER_DEG) as i32;
        Self {
            tx: None,
            rx: None,
            sda: None,
            scl: None,
            py32,
            py32_at: 0,
            servos: [Servo::new(YAW, ZERO_YAW), Servo::new(PITCH, pitch)],
            now: 0,
            rx_buf: Vec::new(),
            tx_buf: Vec::new(),
            powered: false,
            leds: [0; LEDS],
        }
    }
}

fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0u8, |s, b| s.wrapping_add(*b))
}

/// An answer: `FF FF id len error data... checksum`.
fn status(id: u8, data: &[u8]) -> Vec<u8> {
    let mut p = vec![0xFF, 0xFF, id, data.len() as u8 + 2, 0];
    p.extend_from_slice(data);
    let sum = checksum(&p[2..]);
    p.push(sum);
    p
}

impl Body {
    /// Emulated time from a frame. It starts again from zero when the chip
    /// resets (the parts stay attached). The body keeps its power and its
    /// servos stay where they are, so a move under way ends at once.
    fn set_now(&mut self, now_ns: u64) {
        if now_ns + 1_000_000 < self.now {
            let old = self.now;
            for s in &mut self.servos {
                let p = s.position(old);
                s.from = p;
                s.to = p;
                s.t0 = now_ns;
                s.dur = 0;
            }
            self.now = now_ns;
        } else {
            self.now = self.now.max(now_ns);
        }
    }

    /// VM_EN, as the PY32 drives it. Turning the power off resets the servos:
    /// the torque goes off and they stay where they are.
    fn update_power(&mut self) {
        let on = self.py32[PY32_MODE_L] & VM_EN != 0 && self.py32[PY32_OUT_L] & VM_EN != 0;
        if self.powered && !on {
            let now = self.now;
            for s in &mut self.servos {
                s.hold_here(now);
                s.mem[REG_TORQUE] = 0;
            }
        }
        self.powered = on;
    }

    fn servo(&mut self, id: u8) -> Option<&mut Servo> {
        self.servos.iter_mut().find(|s| s.id == id)
    }

    /// Take every whole packet out of `rx_buf`. Bytes before `FF FF` are
    /// noise, and a packet with a bad checksum is dropped from its first byte.
    fn parse(&mut self) {
        loop {
            match self.rx_buf.windows(2).position(|w| w == [0xFF, 0xFF]) {
                Some(at) => drop(self.rx_buf.drain(..at)),
                None => {
                    // Keep a last 0xFF: it may start the next packet.
                    let keep = self.rx_buf.last() == Some(&0xFF);
                    self.rx_buf.clear();
                    if keep {
                        self.rx_buf.push(0xFF);
                    }
                    return;
                }
            }
            if self.rx_buf.len() < 4 {
                return;
            }
            let len = self.rx_buf[3] as usize;
            if len < 2 {
                self.rx_buf.drain(..1);
                continue;
            }
            if self.rx_buf.len() < 4 + len {
                return;
            }
            let packet: Vec<u8> = self.rx_buf.drain(..4 + len).collect();
            if checksum(&packet[2..3 + len]) != packet[3 + len] {
                // Not a packet: start again after its first byte.
                let mut rest = packet[1..].to_vec();
                rest.extend_from_slice(&self.rx_buf);
                self.rx_buf = rest;
                continue;
            }
            self.packet(packet[2], packet[4], &packet[5..3 + len]);
        }
    }

    fn packet(&mut self, id: u8, inst: u8, params: &[u8]) {
        if !self.powered {
            return; // no power: the servos hear nothing
        }
        let now = self.now;
        if id == BROADCAST {
            match inst {
                INST_SYNC_WRITE if params.len() >= 2 => {
                    let (addr, n) = (params[0] as usize, params[1] as usize);
                    for chunk in params[2..].chunks(n + 1) {
                        if chunk.len() == n + 1 {
                            if let Some(s) = self.servo(chunk[0]) {
                                s.write(addr, &chunk[1..], now);
                            }
                        }
                    }
                }
                INST_WRITE if !params.is_empty() => {
                    for s in &mut self.servos {
                        s.write(params[0] as usize, &params[1..], now);
                    }
                }
                INST_ACTION => {
                    for s in &mut self.servos {
                        if let Some((addr, data)) = s.pending.take() {
                            s.write(addr, &data, now);
                        }
                    }
                }
                _ => {}
            }
            return; // a broadcast is never answered
        }
        let Some(s) = self.servo(id) else { return };
        let answer = match inst {
            INST_PING => status(id, &[]),
            INST_READ if params.len() >= 2 => {
                status(id, &s.read(params[0] as usize, params[1] as usize, now))
            }
            INST_WRITE if !params.is_empty() => {
                s.write(params[0] as usize, &params[1..], now);
                status(id, &[])
            }
            INST_REG_WRITE if !params.is_empty() => {
                s.pending = Some((params[0] as usize, params[1..].to_vec()));
                status(id, &[])
            }
            INST_ACTION => {
                if let Some((addr, data)) = s.pending.take() {
                    s.write(addr, &data, now);
                }
                status(id, &[])
            }
            _ => return,
        };
        self.tx_buf.extend_from_slice(&answer);
    }

    fn py32_bus(&mut self, i2c: &I2cTransaction) -> Vec<u8> {
        if let Some((&reg, data)) = i2c.write.split_first() {
            self.py32_at = reg;
            let mut show = false;
            for &b in data {
                if self.py32_at as usize != PY32_VERSION_REG {
                    self.py32[self.py32_at as usize] = b;
                }
                show |= self.py32_at as usize == PY32_LED_CFG && b & LED_SHOW != 0;
                self.py32_at = self.py32_at.wrapping_add(1);
            }
            if show {
                for (i, led) in self.leds.iter_mut().enumerate() {
                    let at = PY32_LED_RAM + 2 * i;
                    *led = u16::from_le_bytes([self.py32[at], self.py32[at + 1]]);
                }
            }
            self.update_power();
        }
        (0..i2c.read_len)
            .map(|_| {
                let v = self.py32[self.py32_at as usize];
                self.py32_at = self.py32_at.wrapping_add(1);
                v
            })
            .collect()
    }
}

impl Device for Body {
    const NAME: &'static str = "stackchan-body";
    const ON_BUS: bool = true;
    const ON_MESSAGE: bool = true;

    fn configure(&mut self, config: &PartConfig) {
        self.tx = config.pins.get(TX).copied();
        self.rx = config.pins.get(RX).copied();
        self.sda = config.pins.get(SDA).copied();
        self.scl = config.pins.get(SCL).copied();
    }

    fn claims(&self) -> Vec<Claim> {
        let i2c = I2cClaim { addr: PY32_ADDR, scl_pin: self.scl, sda_pin: self.sda };
        vec![serial(self.tx, self.rx), Claim { what: Some(claim::What::I2c(i2c)) }]
    }

    fn on_pins(&mut self, _levels: u64, now_ns: u64) {
        self.set_now(now_ns);
    }

    fn on_bus(&mut self, request: &BusRequest) -> Option<BusReply> {
        let (correlation_id, op) = (request.correlation_id, BusOp::I2c as i32);
        let mut reply = BusReply { correlation_id, op, ..Default::default() };
        let Some(bus_request::Op::I2c(i2c)) = &request.op else { return Some(reply) };
        if i2c.addr != PY32_ADDR {
            return Some(reply);
        }
        reply.data = self.py32_bus(i2c);
        reply.ack = true;
        Some(reply)
    }

    fn on_uart(&mut self, data: &[u8]) {
        self.rx_buf.extend_from_slice(data);
    }

    /// Asked after every frame: parse what came in at this time, and send the
    /// answers.
    fn poll(&mut self, now_ns: u64) -> Option<EmulatorMessage> {
        self.set_now(now_ns);
        if !self.rx_buf.is_empty() {
            self.parse();
        }
        if self.tx_buf.is_empty() {
            return None;
        }
        Some(transmit(&std::mem::take(&mut self.tx_buf), self.now))
    }

    fn state(&self) -> Option<State> {
        let [yaw, pitch] = &self.servos;
        let now = self.now;
        Some(
            State::new()
                .set("power", self.powered as u8)
                .set("yaw", format!("{:.1}", yaw.degrees(now)))
                .set("pitch", format!("{:.1}", pitch.degrees(now)))
                .set("yaw_raw", yaw.position(now).round() as i32)
                .set("pitch_raw", pitch.position(now).round() as i32)
                .set("yaw_torque", yaw.torque() as u8)
                .set("pitch_torque", pitch.torque() as u8)
                .set("moving", (yaw.moving(now) || pitch.moving(now)) as u8)
                .set("leds", self.led_colors()),
        )
    }
}

impl Body {
    /// The shown colors as `rrggbb` (8 bits each), LED 0 first, comma
    /// separated. The LEDs only light with the body's 5 V, which comes from
    /// the CoreS3 (the cores3 part); the page puts the two together.
    fn led_colors(&self) -> String {
        let hex = |c: u16| {
            let (r, g, b) = ((c >> 11) & 31, (c >> 5) & 63, c & 31);
            let (r, g, b) = ((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2));
            format!("{r:02x}{g:02x}{b:02x}")
        };
        self.leds.iter().map(|&c| hex(c)).collect::<Vec<_>>().join(",")
    }
}

// The chip's serial data arrives in pieces of up to 128 bytes, and the state
// (with 12 LED colors) is about 200 bytes.
esp_part::export_part!(Body, buffer = 512);

#[cfg(test)]
mod tests {
    use super::*;

    fn body() -> Body {
        let mut b = Body::default();
        let pins = [("TX", 7), ("RX", 6), ("SDA", 12), ("SCL", 11)]
            .map(|(n, p)| (n.to_string(), p))
            .into();
        b.configure(&PartConfig { pins, ..Default::default() });
        b
    }

    fn i2c(b: &mut Body, write: &[u8], read_len: u32) -> Vec<u8> {
        let req = I2cTransaction { addr: PY32_ADDR, write: write.to_vec(), read_len, ..Default::default() };
        b.py32_bus(&req)
    }

    fn power_on(b: &mut Body) {
        i2c(b, &[PY32_MODE_L as u8, 0x01], 0);
        i2c(b, &[PY32_OUT_L as u8, 0x01], 0);
    }

    fn packet(id: u8, inst: u8, params: &[u8]) -> Vec<u8> {
        let mut p = vec![0xFF, 0xFF, id, params.len() as u8 + 2, inst];
        p.extend_from_slice(params);
        let sum = checksum(&p[2..]);
        p.push(sum);
        p
    }

    /// Send bytes at time `t` ns, return the answer.
    fn send(b: &mut Body, bytes: &[u8], t: u64) -> Vec<u8> {
        b.on_uart(bytes);
        match b.poll(t).and_then(|m| m.payload) {
            Some(esp_part::Payload::UartRx(rx)) => rx.data,
            None => Vec::new(),
            Some(other) => panic!("not a UartRx: {other:?}"),
        }
    }

    #[test]
    fn py32_answers_its_version_in_two_transfers() {
        let mut b = body();
        assert_eq!(i2c(&mut b, &[0x02], 0), []);
        assert_eq!(i2c(&mut b, &[], 1), [0x41]);
        i2c(&mut b, &[0x02, 0x00], 0);
        assert_eq!(i2c(&mut b, &[0x02], 1), [0x41], "the version can't be written");
    }

    #[test]
    fn servos_are_silent_without_power() {
        let mut b = body();
        assert_eq!(send(&mut b, &packet(YAW, INST_PING, &[]), 1000), []);
        power_on(&mut b);
        assert_eq!(send(&mut b, &packet(YAW, INST_PING, &[]), 2000), status(YAW, &[]));
        assert_eq!(send(&mut b, &packet(3, INST_PING, &[]), 3000), [], "no servo 3");
    }

    #[test]
    fn a_timed_move_takes_its_time() {
        let mut b = body();
        power_on(&mut b);
        // Yaw to raw 556 (30 degrees) in 1000 ms, as tr_servo_move sends it.
        let goal = [REG_GOAL as u8, 0x02, 0x2C, 0x03, 0xE8, 0, 0];
        assert_eq!(send(&mut b, &packet(YAW, INST_WRITE, &goal), 0), status(YAW, &[]));
        let read = packet(YAW, INST_READ, &[REG_POSITION as u8, 8]);
        let half = send(&mut b, &read, 500_000_000);
        assert_eq!(u16::from_be_bytes([half[5], half[6]]), 508, "halfway at 500 ms");
        let done = send(&mut b, &read, 1_000_000_000);
        assert_eq!(u16::from_be_bytes([done[5], done[6]]), 556);
        assert_eq!(done[11], VOLTS_X10);
        let torque = send(&mut b, &packet(YAW, INST_READ, &[REG_TORQUE as u8, 1]), 1_000_000_001);
        assert_eq!(torque[5], 1, "a goal turns the torque on");
        assert_eq!(b.state().unwrap().get("yaw"), Some("30.0"));
    }

    #[test]
    fn torque_off_stops_the_move_and_power_off_resets_torque() {
        let mut b = body();
        power_on(&mut b);
        let goal = [REG_GOAL as u8, 0x02, 0x2C, 0x03, 0xE8, 0, 0];
        send(&mut b, &packet(YAW, INST_WRITE, &goal), 0);
        send(&mut b, &packet(YAW, INST_WRITE, &[REG_TORQUE as u8, 0]), 500_000_000);
        b.poll(2_000_000_000);
        assert_eq!(b.state().unwrap().get("yaw_raw"), Some("508"), "stopped halfway");
        send(&mut b, &packet(PITCH, INST_WRITE, &[REG_TORQUE as u8, 1]), 2_000_000_000);
        i2c(&mut b, &[PY32_OUT_L as u8, 0x00], 0);
        assert_eq!(b.state().unwrap().get("pitch_torque"), Some("0"));
        assert_eq!(b.state().unwrap().get("power"), Some("0"));
    }

    #[test]
    fn packets_in_pieces_noise_and_bad_checksums() {
        let mut b = body();
        power_on(&mut b);
        let ping = packet(PITCH, INST_PING, &[]);
        assert_eq!(send(&mut b, &[0x00, 0x13], 1), []);
        assert_eq!(send(&mut b, &ping[..3], 2), []);
        assert_eq!(send(&mut b, &ping[3..], 3), status(PITCH, &[]));
        let mut bad = ping.clone();
        *bad.last_mut().unwrap() ^= 1;
        assert_eq!(send(&mut b, &bad, 4), []);
        assert_eq!(send(&mut b, &[ping.clone(), ping].concat(), 5).len(), 12, "two answers");
    }

    #[test]
    fn sync_write_moves_both_and_is_not_answered() {
        let mut b = body();
        power_on(&mut b);
        // Both to raw 500, at speed 0 (fastest).
        let params = [REG_GOAL as u8, 6, YAW, 0x01, 0xF4, 0, 0, 0, 0, PITCH, 0x01, 0xF4, 0, 0, 0, 0];
        assert_eq!(send(&mut b, &packet(BROADCAST, INST_SYNC_WRITE, &params), 0), []);
        b.poll(1_000_000_000);
        assert_eq!(b.state().unwrap().get("yaw_raw"), Some("500"));
        assert_eq!(b.state().unwrap().get("pitch_raw"), Some("500"));
    }

    #[test]
    fn a_chip_reset_restarts_time_and_moves_still_work() {
        let mut b = body();
        power_on(&mut b);
        let goal = [REG_GOAL as u8, 0x02, 0x2C, 0x03, 0xE8, 0, 0]; // yaw 30 deg in 1 s
        send(&mut b, &packet(YAW, INST_WRITE, &goal), 20_000_000_000);
        b.poll(20_500_000_000);
        // The chip resets halfway: time starts again near zero.
        b.poll(5_000_000);
        assert_eq!(b.state().unwrap().get("yaw_raw"), Some("508"), "stays where it was");
        assert_eq!(b.state().unwrap().get("power"), Some("1"), "the body keeps its power");
        let back = [REG_GOAL as u8, 0x01, 0xCC, 0x01, 0xF4, 0, 0]; // raw 460 in 500 ms
        send(&mut b, &packet(YAW, INST_WRITE, &back), 10_000_000);
        b.poll(510_000_000);
        assert_eq!(b.state().unwrap().get("yaw_raw"), Some("460"));
        assert_eq!(b.state().unwrap().get("moving"), Some("0"));
    }

    #[test]
    fn side_leds_show_on_the_show_bit() {
        let mut b = body();
        let off = "000000,".repeat(11) + "000000";
        assert_eq!(b.state().unwrap().get("leds"), Some(off.as_str()));
        // As tr_led_show(): LED 0 red, LED 11 blue, in one transfer from 0x30.
        let mut ram = vec![PY32_LED_RAM as u8];
        ram.extend_from_slice(&[0x00, 0xF8]);
        ram.extend_from_slice(&[0u8; 20]);
        ram.extend_from_slice(&[0x1F, 0x00]);
        i2c(&mut b, &ram, 0);
        assert_eq!(b.state().unwrap().get("leds"), Some(off.as_str()), "nothing until the show bit");
        i2c(&mut b, &[PY32_LED_CFG as u8], 0);
        let cfg = i2c(&mut b, &[], 1)[0];
        i2c(&mut b, &[PY32_LED_CFG as u8, cfg | LED_SHOW], 0);
        let shown = b.state().unwrap().get("leds").unwrap().to_string();
        assert!(shown.starts_with("ff0000,000000"));
        assert!(shown.ends_with(",0000ff"));
    }

    #[test]
    fn claims_the_serial_line_and_the_py32() {
        let claims = body().claims();
        assert_eq!(claims.len(), 2);
        let uart = UartClaim { tx_pin: Some(6), rx_pin: Some(7) };
        assert_eq!(claims[0], Claim { what: Some(claim::What::Uart(uart)) });
    }
}
