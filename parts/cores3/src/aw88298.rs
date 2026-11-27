//! AW88298 speaker amplifier, address 0x36. STUB: not modelled yet, it does not answer (NACK).

use crate::chip::Chip;

#[derive(Default)]
pub struct Aw88298;

impl Chip for Aw88298 {
    fn transfer(&mut self, _write: &[u8], _read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        None
    }
}
