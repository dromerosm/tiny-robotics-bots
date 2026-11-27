//! BMI270 IMU at 0x69, with the BMM150 magnetometer behind its aux interface. STUB: not modelled yet, it does not answer (NACK).

use crate::chip::Chip;

#[derive(Default)]
pub struct Bmi270;

impl Chip for Bmi270 {
    fn transfer(&mut self, _write: &[u8], _read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        None
    }
}
