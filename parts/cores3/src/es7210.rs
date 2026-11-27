//! ES7210 microphone ADC, address 0x40. STUB: not modelled yet, it does not answer (NACK).

use crate::chip::Chip;

#[derive(Default)]
pub struct Es7210;

impl Chip for Es7210 {
    fn transfer(&mut self, _write: &[u8], _read_len: usize, _continues: bool, _now_ns: u64) -> Option<Vec<u8>> {
        None
    }
}
