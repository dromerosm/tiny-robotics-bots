//! BM8563 RTC, address 0x51. STUB: not modelled yet, it does not answer (NACK).

use crate::chip::Chip;

#[derive(Default)]
pub struct Bm8563;

impl Chip for Bm8563 {
    fn transfer(&mut self, _write: &[u8], _read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        None
    }
}
