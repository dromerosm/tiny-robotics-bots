//! FT6336U touch, address 0x38. STUB: not modelled yet, it does not answer (NACK).

use crate::chip::Chip;

#[derive(Default)]
pub struct Ft6336;

impl Chip for Ft6336 {
    fn transfer(&mut self, _write: &[u8], _read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        None
    }
}

impl Ft6336 {
    /// Whether a finger is on the screen now (drives AW9523 P1_2 low).
    pub fn touched(&self) -> bool {
        false
    }
}
