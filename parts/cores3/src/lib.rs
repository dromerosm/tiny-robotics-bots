//! The chips on StackChan's CoreS3 board that our firmware needs to drive the
//! body, as a virtual part for esp-emu. Both are on the internal I2C bus.
//!
//! - AW9523 IO expander, address 0x58. Registers 0x02/0x03 are the output
//!   levels, 0x04/0x05 the directions (1 = input), 0x00/0x01 read the pins,
//!   0x10 is the ID (0x23). P0_1 is BUS_OUT_EN and P1_7 is BOOST_EN: with
//!   both as outputs and high, the body gets 5 V (for the side LEDs).
//! - AXP2101 power chip, address 0x34. Register 0x00 says USB power is
//!   present (bit 5), 0x03 is the chip ID (0x4A), 0xA4 the battery (80 %).
//!
//! Every other register stores what is written. This is enough for tr_board
//! to switch the body's 5 V; it is not a model of the chips' electronics.

use esp_part::prelude::*;

const SDA: &str = "SDA";
const SCL: &str = "SCL";

const AW9523: u32 = 0x58;
const AXP2101: u32 = 0x34;

const AW_IN0: usize = 0x00;
const AW_OUT0: usize = 0x02;
const AW_DIR0: usize = 0x04; // 1 = input
const AW_ID: usize = 0x10;
const BUS_OUT_EN: (usize, u8) = (0, 0x02); // port 0, pin 1
const BOOST_EN: (usize, u8) = (1, 0x80); // port 1, pin 7

/// A chip seen as 256 registers with an address pointer that moves on after
/// each byte, as both chips do.
struct Regs {
    mem: [u8; 256],
    at: u8,
    /// Registers that can't be written (IDs, status).
    read_only: &'static [usize],
}

impl Regs {
    fn new(init: &[(usize, u8)], read_only: &'static [usize]) -> Self {
        let mut mem = [0u8; 256];
        for &(r, v) in init {
            mem[r] = v;
        }
        Self { mem, at: 0, read_only }
    }

    fn transaction(&mut self, i2c: &I2cTransaction, read: impl Fn(&[u8; 256], usize) -> u8) -> Vec<u8> {
        if let Some((&reg, data)) = i2c.write.split_first() {
            self.at = reg;
            for &b in data {
                if !self.read_only.contains(&(self.at as usize)) {
                    self.mem[self.at as usize] = b;
                }
                self.at = self.at.wrapping_add(1);
            }
        }
        (0..i2c.read_len)
            .map(|_| {
                let v = read(&self.mem, self.at as usize);
                self.at = self.at.wrapping_add(1);
                v
            })
            .collect()
    }
}

/// What the AW9523 reads on a pin: the level it drives on an output, and
/// high on an input (the pull-ups on the board).
fn aw_read(mem: &[u8; 256], reg: usize) -> u8 {
    match reg {
        AW_IN0 | 0x01 => {
            let port = reg - AW_IN0;
            let dir = mem[AW_DIR0 + port];
            (mem[AW_OUT0 + port] & !dir) | dir
        }
        _ => mem[reg],
    }
}

struct Cores3 {
    sda: Option<u32>,
    scl: Option<u32>,
    aw: Regs,
    axp: Regs,
}

impl Default for Cores3 {
    fn default() -> Self {
        Self {
            sda: None,
            scl: None,
            // After reset every AW9523 pin is an input.
            aw: Regs::new(&[(AW_ID, 0x23), (AW_DIR0, 0xFF), (AW_DIR0 + 1, 0xFF)], &[AW_IN0, 0x01, AW_ID]),
            // USB power present, chip ID, battery at 80 %.
            axp: Regs::new(&[(0x00, 0x20), (0x01, 0x00), (0x03, 0x4A), (0xA4, 0x50)], &[0x00, 0x03]),
        }
    }
}

impl Cores3 {
    fn aw_pin(&self, (port, bit): (usize, u8)) -> bool {
        self.aw.mem[AW_DIR0 + port] & bit == 0 && self.aw.mem[AW_OUT0 + port] & bit != 0
    }

    /// The 5 V the CoreS3 gives the body: BUS_OUT_EN and BOOST_EN both on.
    fn body_5v(&self) -> bool {
        self.aw_pin(BUS_OUT_EN) && self.aw_pin(BOOST_EN)
    }
}

impl Device for Cores3 {
    const NAME: &'static str = "cores3";
    const ON_BUS: bool = true;

    fn configure(&mut self, config: &PartConfig) {
        self.sda = config.pins.get(SDA).copied();
        self.scl = config.pins.get(SCL).copied();
    }

    fn claims(&self) -> Vec<Claim> {
        [AW9523, AXP2101]
            .map(|addr| Claim { what: Some(claim::What::I2c(I2cClaim { addr, scl_pin: self.scl, sda_pin: self.sda })) })
            .to_vec()
    }

    fn on_bus(&mut self, request: &BusRequest) -> Option<BusReply> {
        let (correlation_id, op) = (request.correlation_id, BusOp::I2c as i32);
        let mut reply = BusReply { correlation_id, op, ..Default::default() };
        let Some(bus_request::Op::I2c(i2c)) = &request.op else { return Some(reply) };
        reply.data = match i2c.addr {
            AW9523 => self.aw.transaction(i2c, aw_read),
            AXP2101 => self.axp.transaction(i2c, |m, r| m[r]),
            _ => return Some(reply), // not ours: no ACK
        };
        reply.ack = true;
        Some(reply)
    }

    fn state(&self) -> Option<State> {
        Some(
            State::new()
                .set("body_5v", self.body_5v() as u8)
                .set("bus_out_en", self.aw_pin(BUS_OUT_EN) as u8)
                .set("boost_en", self.aw_pin(BOOST_EN) as u8),
        )
    }
}

esp_part::export_part!(Cores3, buffer = 256);

#[cfg(test)]
mod tests {
    use super::*;

    fn part() -> Cores3 {
        let mut p = Cores3::default();
        let pins = [("SDA", 12), ("SCL", 11)].map(|(n, p)| (n.to_string(), p)).into();
        p.configure(&PartConfig { pins, ..Default::default() });
        p
    }

    fn i2c(p: &mut Cores3, addr: u32, write: &[u8], read_len: u32) -> Option<Vec<u8>> {
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
    fn ids_answer_and_other_addresses_do_not() {
        let mut p = part();
        assert_eq!(i2c(&mut p, AW9523, &[0x10], 1), Some(vec![0x23]));
        assert_eq!(i2c(&mut p, AXP2101, &[0x03], 1), Some(vec![0x4A]));
        assert_eq!(i2c(&mut p, AXP2101, &[0x00], 1), Some(vec![0x20]), "USB power present");
        assert_eq!(i2c(&mut p, 0x40, &[0x00], 1), None);
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
        // tr_board_aw9523_off: read the level, clear the bit.
        let v = i2c(&mut p, AW9523, &[0x02], 1).unwrap()[0];
        i2c(&mut p, AW9523, &[0x02, v & !0x02], 0);
        assert_eq!(p.state().unwrap().get("body_5v"), Some("0"));
    }

    #[test]
    fn input_registers_follow_the_pins() {
        let mut p = part();
        assert_eq!(i2c(&mut p, AW9523, &[0x00], 2), Some(vec![0xFF, 0xFF]), "inputs read high");
        i2c(&mut p, AW9523, &[0x04, 0xFE], 0); // P0_0 output, low
        assert_eq!(i2c(&mut p, AW9523, &[0x00], 1), Some(vec![0xFE]));
        i2c(&mut p, AW9523, &[0x00, 0x00], 0);
        assert_eq!(i2c(&mut p, AW9523, &[0x00], 1), Some(vec![0xFE]), "the input register can't be written");
    }
}
