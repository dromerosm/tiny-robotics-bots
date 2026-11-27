//! LTR-553ALS light and proximity sensor, address 0x23. STUB: not modelled yet, it does not answer (NACK).

use crate::chip::Chip;

#[derive(Default)]
pub struct Ltr553;

impl Chip for Ltr553 {
    fn transfer(&mut self, _write: &[u8], _read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        None
    }
}
