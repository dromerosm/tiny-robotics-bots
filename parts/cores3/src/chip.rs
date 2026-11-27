//! What every chip model in this part has in common.

use esp_part::State;

/// One chip on the CoreS3's internal I2C bus.
///
/// The part routes each I2C transaction to the chip at its address. A chip
/// keeps its own register pointer between transactions, as the real ones do,
/// so a long read that the controller splits into pieces (`continues`, see
/// `I2cTransaction` in esp_cosim.proto) goes on from where it stopped.
pub trait Chip {
    /// One I2C transaction to this chip, at emulated time `now_ns`.
    ///
    /// - `write`: every byte the master clocked out. Usually the register
    ///   address first. Empty for a probe (`read_len` 0 too) or for a later
    ///   piece of a split read (`continues`).
    /// - `read_len`: how many bytes the master reads back.
    ///
    /// Returns the bytes read (exactly `read_len` of them), or `None` for a
    /// NACK: the chip is not there or not powered.
    fn transfer(&mut self, write: &[u8], read_len: usize, continues: bool, now_ns: u64) -> Option<Vec<u8>>;

    /// A value set from the page: `set:<name>=<value>` without the `set:`.
    /// Returns true if the name was this chip's.
    fn input(&mut self, _name: &str, _value: f64) -> bool {
        false
    }

    /// Add what the page should see, in the chip's own words.
    fn state(&self, state: State, _now_ns: u64) -> State {
        state
    }
}

/// 256 byte registers and a pointer that moves on after each byte, the most
/// common layout. `read_only` registers ignore writes.
pub struct Regs8 {
    pub mem: [u8; 256],
    pub at: u8,
    read_only: &'static [u8],
}

impl Regs8 {
    pub fn new(init: &[(u8, u8)], read_only: &'static [u8]) -> Self {
        let mut mem = [0u8; 256];
        for &(r, v) in init {
            mem[r as usize] = v;
        }
        Self { mem, at: 0, read_only }
    }

    /// The first written byte sets the pointer; the rest are stored from
    /// there on. Returns the register the write started at, if any.
    pub fn write(&mut self, write: &[u8]) -> Option<u8> {
        let (&reg, data) = write.split_first()?;
        self.at = reg;
        for &b in data {
            if !self.read_only.contains(&self.at) {
                self.mem[self.at as usize] = b;
            }
            self.at = self.at.wrapping_add(1);
        }
        Some(reg)
    }

    /// Read `len` bytes from the pointer on, through `read` (which can
    /// compute a register instead of returning the stored byte).
    pub fn read(&mut self, len: usize, mut read: impl FnMut(&[u8; 256], u8) -> u8) -> Vec<u8> {
        (0..len)
            .map(|_| {
                let v = read(&self.mem, self.at);
                self.at = self.at.wrapping_add(1);
                v
            })
            .collect()
    }
}
