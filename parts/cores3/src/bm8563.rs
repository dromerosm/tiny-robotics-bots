//! BM8563 RTC, address 0x51. The BM8563 is a copy of NXP's PCF8563: same
//! registers, same behaviour. The rules below come from the PCF8563 data
//! sheet (NXP, Rev. 11.1, January 2026) unless a line says otherwise.
//!
//! - 16 registers, 00-0F. The register pointer moves on after each byte read
//!   or written and wraps from 0F to 00. A pointer above 0F is taken modulo 16
//!   (our choice: the data sheet does not say).
//! - 02-08 seconds, minutes, hours, day, weekday, month, year, in BCD. They
//!   tick once per emulated second, from `now_ns`. Bit 7 of 02 is VL ("the
//!   clock lost power"): set at power-on, cleared when the firmware writes 02
//!   with bit 7 clear. Bit 7 of 07 is the century bit C. It toggles when the
//!   year goes from 99 to 00, and all three drivers read C = 1 as 19xx, so the
//!   clock covers 1900-2099. February has 29 days when the year register is a
//!   multiple of 4, 00 included. The weekday counts 0-6 and moves on at
//!   midnight; the chip never computes it from the date.
//! - Writing a time register does not reset the sub-second phase (only the
//!   STOP bit does). The counters are frozen during one access, so a long read
//!   that arrives in pieces (`continues`) reads one consistent time.
//! - 00 STOP (bit 5) stops the clock. The first tick after STOP is released
//!   comes 0.5078 s later, as in the data sheet.
//! - 01: AF (bit 3) and TF (bit 2) are cleared by writing 0 and kept by
//!   writing 1. TI_TP, AIE and TIE are stored.
//! - 09-0C alarm (bit 7 = disabled). AF is set when the time ticks into a
//!   minute where every enabled field matches. Writing the time or the alarm
//!   registers does not set it.
//! - 0E timer control (TE bit 7, source clock bits 1:0: 4096 Hz, 64 Hz,
//!   1 Hz, 1/60 Hz) and 0F countdown value n. TF is set at the end of every
//!   countdown; reading 0F gives the current count; n = 0 stops the timer.
//! - 0D CLKOUT control is storage.
//! - Reset values: 00 = 08, 01 = 00, VL = 1, alarms disabled (80), 0D = 80,
//!   0E = 03. The data sheet leaves the time and 0F undefined at power-on. We
//!   start at 2000-01-01 00:00:00, a Saturday (weekday 6), with 0F = 00,
//!   because M5Unified rejects a time that is not valid BCD or a valid date.
//! - Bits the data sheet marks unused, or "must write 0", read as 0 here. On
//!   the real chip they are undefined. The drivers mask them.
//! - INT is not wired: no pin goes low. The firmware sees AF and TF in 01.
//!
//! What the drivers do (cores3-chips-spec.md section 4):
//! - tr_board: reads 7 bytes from 02 and masks them (no setup).
//! - M5Unified: `begin` writes 00←00 twice and 0E←03; `setSystemTimeFromRtc`
//!   reads 7 bytes from 02 and rejects bad BCD or a bad date; `disableIRQ`
//!   writes 09←80 80 80 80 in one burst, then 0E←00 and 01←00. Its alarm and
//!   timer helpers rely on the AF/TF write-0-to-clear rule.
//! - StackChan: the same `begin`, a 7-byte read from 02 at boot, and a 7-byte
//!   write from 02 after an NTP sync.
//!
//! Input `time`: Unix seconds (UTC). The clock then reads that time, counted
//! from the latest emulated time the chip has seen (the input carries no time
//! of its own), and VL is cleared: a clock that kept time on its battery.
//! Times outside 1900-2099 are ignored. State: `rtc`, "YYYY-MM-DD hh:mm:ss".

use crate::chip::Chip;
use esp_part::State;

const NS: i64 = 1_000_000_000;
/// 1900 to 2099 in the chip's calendar: 200 years, a leap year every 4.
const CYCLE_S: i64 = (200 * 365 + 50) * 86_400;
/// The bits each register keeps (02 without VL, which is kept apart).
const MASK: [u8; 16] = [0xA8, 0x13, 0x7F, 0x7F, 0x3F, 0x3F, 0x07, 0x9F, 0xFF, 0xFF, 0xBF, 0xBF, 0x87, 0x83, 0x83, 0xFF];
/// The first tick after STOP is released, in ns (data sheet: 0.507813 s).
const FIRST_TICK_NS: i64 = 507_813_000;

const CTRL1: u8 = 0x00;
const CTRL2: u8 = 0x01;
const SECONDS: u8 = 0x02;
const ALARM: usize = 0x09;
const TIMER_CTRL: u8 = 0x0E;
const TIMER: u8 = 0x0F;
const STOP: u8 = 0x20;
const AF: u8 = 0x08;
const TF: u8 = 0x04;

pub struct Bm8563 {
    /// Registers 00-0F as stored. 01 keeps the flags; 0F the loaded count.
    regs: [u8; 16],
    vl: bool,
    at: u8,
    /// The emulated time of the last 1 Hz tick: the clock's phase.
    tick_ns: i64,
    /// When the countdown timer started, and how many countdowns it has
    /// already turned into TF.
    timer_ns: i64,
    timer_done: u64,
    /// The latest emulated time the chip has seen, for the `time` input.
    last_ns: u64,
}

impl Default for Bm8563 {
    fn default() -> Self {
        let mut regs = [0u8; 16];
        regs[CTRL1 as usize] = 0x08;
        regs[0x05] = 0x01; // day 1
        regs[0x06] = 6; // Saturday
        regs[0x07] = 0x01; // January, C = 0: 20xx
        regs[ALARM..ALARM + 4].copy_from_slice(&[0x80; 4]);
        regs[0x0D] = 0x80;
        regs[TIMER_CTRL as usize] = 0x03;
        Self { regs, vl: true, at: 0, tick_ns: 0, timer_ns: 0, timer_done: 0, last_ns: 0 }
    }
}

fn bcd(v: u8) -> i64 {
    (v >> 4) as i64 * 10 + (v & 0x0F) as i64
}

fn to_bcd(v: i64) -> u8 {
    ((v / 10) << 4 | v % 10) as u8
}

/// Days from 1900-01-01 to January 1st of year `y` (years since 1900).
fn days_before_year(y: i64) -> i64 {
    y * 365 + (y + 3) / 4
}

fn month_days(y: i64, m: i64) -> i64 {
    match m {
        2 if y % 4 == 0 => 29,
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

/// The time registers as seconds since 1900-01-01 00:00:00. A value outside
/// the calendar (a bad month, day 31 in April) is carried over, so it comes
/// out normalized after the next tick. The real chip's behaviour there is
/// undefined.
fn total(r: &[u8; 16]) -> i64 {
    let y = bcd(r[8]) + if r[7] & 0x80 != 0 { 0 } else { 100 };
    let m = bcd(r[7] & 0x1F).clamp(1, 12);
    let days = days_before_year(y) + (1..m).map(|k| month_days(y, k)).sum::<i64>() + bcd(r[5]) - 1;
    days * 86_400 + bcd(r[4]) * 3600 + bcd(r[3]) * 60 + bcd(r[2])
}

/// Seconds since 1900 (within the 200-year cycle) to (year since 1900,
/// month, day, hour, minute, second).
fn fields(t: i64) -> [i64; 6] {
    let t = t.rem_euclid(CYCLE_S);
    let (mut days, s) = (t / 86_400, t % 86_400);
    // Blocks of 4 years, each starting with a leap year.
    let mut y = 4 * (days / 1461);
    days %= 1461;
    if days >= 366 {
        days -= 366;
        y += 1 + days / 365;
        days %= 365;
    }
    let mut m = 1;
    while days >= month_days(y, m) {
        days -= month_days(y, m);
        m += 1;
    }
    [y, m, days + 1, s / 3600, s / 60 % 60, s % 60]
}

/// Store a time (seconds since 1900) and a weekday in the time registers.
fn store(r: &mut [u8; 16], t: i64, weekday: i64) {
    let [y, m, d, h, mi, s] = fields(t);
    r[2] = to_bcd(s);
    r[3] = to_bcd(mi);
    r[4] = to_bcd(h);
    r[5] = to_bcd(d);
    r[6] = weekday.rem_euclid(7) as u8;
    r[7] = to_bcd(m) | if y < 100 { 0x80 } else { 0 };
    r[8] = to_bcd(y % 100);
}

/// The time registers `secs` seconds later. The weekday moves on once per
/// midnight crossed.
fn advance(r: &mut [u8; 16], secs: i64) {
    if secs <= 0 {
        return;
    }
    let t0 = total(r);
    let t1 = t0 + secs;
    let midnights = t1.div_euclid(86_400) - t0.div_euclid(86_400);
    store(r, t1, (r[6] & 0x07) as i64 + midnights);
}

/// Unix seconds (UTC, proleptic Gregorian) to (year, month, day), from
/// Howard Hinnant's `civil_from_days`.
fn civil(unix_days: i64) -> (i64, i64, i64) {
    let z = unix_days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + (m <= 2) as i64, m, d)
}

impl Bm8563 {
    fn running(&self) -> bool {
        self.regs[CTRL1 as usize] & STOP == 0
    }

    fn timer_running(&self) -> bool {
        self.regs[TIMER_CTRL as usize] & 0x80 != 0 && self.regs[TIMER as usize] != 0
    }

    /// Whole countdown-clock periods since the timer started.
    fn timer_ticks(&self, now: i64) -> u64 {
        // 4096 Hz, 64 Hz, 1 Hz, 1/60 Hz as (numerator, denominator).
        let (num, den) = [(4096, 1), (64, 1), (1, 1), (1, 60)][(self.regs[TIMER_CTRL as usize] & 3) as usize];
        let elapsed = (now - self.timer_ns).max(0) as u128;
        (elapsed * num / (NS as u128 * den)) as u64
    }

    /// Ticks the clock would have made by `now`.
    fn ticks(&self, now: i64) -> i64 {
        if self.running() && now >= self.tick_ns + NS {
            (now - self.tick_ns) / NS
        } else {
            0
        }
    }

    /// Whether every enabled alarm field matches a time and weekday (and at
    /// least one is enabled).
    fn alarm_matches(&self, t: i64, weekday: i64) -> bool {
        let [_, _, d, h, mi, _] = fields(t);
        let now = [to_bcd(mi), to_bcd(h), to_bcd(d), weekday.rem_euclid(7) as u8];
        let masks = [0x7F, 0x3F, 0x3F, 0x07];
        let alarm = &self.regs[ALARM..ALARM + 4];
        alarm.iter().any(|a| a & 0x80 == 0)
            && (0..4).all(|i| alarm[i] & 0x80 != 0 || alarm[i] & masks[i] == now[i])
    }

    /// Set AF if the time ticks into a matching minute within the next `n`
    /// seconds. The minute, hour, day and weekday only change on a minute.
    fn check_alarm(&mut self, n: i64) {
        if self.regs[CTRL2 as usize] & AF != 0 || self.regs[ALARM..ALARM + 4].iter().all(|a| a & 0x80 != 0) {
            return;
        }
        let t0 = total(&self.regs);
        let wd0 = (self.regs[6] & 0x07) as i64;
        let weekday = |t: i64| wd0 + t.div_euclid(86_400) - t0.div_euclid(86_400);
        let mut before = self.alarm_matches(t0, wd0);
        let mut at = 60 - t0.rem_euclid(60);
        // Any alarm a calendar allows comes round within 400 days.
        let last = n.min(400 * 86_400);
        while at <= last {
            let t = t0 + at;
            let matches = self.alarm_matches(t, weekday(t));
            if matches && !before {
                self.regs[CTRL2 as usize] |= AF;
                return;
            }
            before = matches;
            at += 60;
        }
    }

    /// Bring the clock and the timer up to `now`.
    fn sync(&mut self, now: i64) {
        let n = self.ticks(now);
        if n > 0 {
            self.check_alarm(n);
            advance(&mut self.regs, n);
            self.tick_ns += n * NS;
        }
        if self.timer_running() {
            let done = self.timer_ticks(now) / self.regs[TIMER as usize] as u64;
            if done > self.timer_done {
                self.regs[CTRL2 as usize] |= TF;
                self.timer_done = done;
            }
        }
    }

    fn restart_timer(&mut self, now: i64) {
        self.timer_ns = now;
        self.timer_done = 0;
    }

    fn write_reg(&mut self, r: u8, v: u8, now: i64) {
        match r {
            CTRL1 => {
                let was_stopped = !self.running();
                self.regs[0] = v & MASK[0];
                if was_stopped && self.running() {
                    self.tick_ns = now + FIRST_TICK_NS - NS;
                }
            }
            // AF and TF: a logic AND, so writing 1 keeps a flag.
            CTRL2 => self.regs[1] = (v & 0x13) | (self.regs[1] & v & (AF | TF)),
            SECONDS => {
                self.vl = v & 0x80 != 0;
                self.regs[2] = v & 0x7F;
            }
            TIMER_CTRL => {
                let old = self.regs[r as usize];
                self.regs[r as usize] = v & MASK[r as usize];
                if v & 0x80 != 0 && (old & 0x80 == 0 || (old ^ v) & 0x03 != 0) {
                    self.restart_timer(now);
                }
            }
            TIMER => {
                self.regs[r as usize] = v;
                self.restart_timer(now);
            }
            _ => self.regs[r as usize] = v & MASK[r as usize],
        }
    }

    fn read_reg(&self, r: u8, now: i64) -> u8 {
        match r {
            SECONDS => self.regs[2] | (self.vl as u8) << 7,
            TIMER if self.timer_running() => {
                let n = self.regs[TIMER as usize] as u64;
                (n - self.timer_ticks(now) % n) as u8
            }
            _ => self.regs[r as usize],
        }
    }

    /// The date and time the clock shows at `now`, as text.
    pub fn text(&self, now_ns: u64) -> String {
        let mut r = self.regs;
        advance(&mut r, self.ticks(now_ns as i64));
        let [y, m, d, h, mi, s] = fields(total(&r));
        format!("{:04}-{:02}-{:02} {:02}:{:02}:{:02}", 1900 + y, m, d, h, mi, s)
    }
}

impl Chip for Bm8563 {
    fn transfer(&mut self, write: &[u8], read_len: usize, continues: bool, now_ns: u64) -> Option<Vec<u8>> {
        self.last_ns = self.last_ns.max(now_ns);
        if write.is_empty() && read_len == 0 {
            return Some(Vec::new()); // a probe
        }
        let now = now_ns as i64;
        // The counters are frozen for the whole access: a later piece of a
        // split read sees the time of the first.
        if !(continues && write.is_empty()) {
            self.sync(now);
        }
        if let Some((&reg, data)) = write.split_first() {
            self.at = reg & 0x0F;
            for &v in data {
                self.write_reg(self.at, v, now);
                self.at = (self.at + 1) & 0x0F;
            }
        }
        let data = (0..read_len)
            .map(|_| {
                let v = self.read_reg(self.at, now);
                self.at = (self.at + 1) & 0x0F;
                v
            })
            .collect();
        Some(data)
    }

    fn input(&mut self, name: &str, value: f64) -> bool {
        if name != "time" {
            return false;
        }
        let secs = value.floor();
        let unix = secs as i64;
        let (y, m, d) = civil(unix.div_euclid(86_400));
        if !(1900..=2099).contains(&y) {
            return true; // ours, but the chip cannot show it
        }
        let y = y - 1900;
        let days = days_before_year(y) + (1..m).map(|k| month_days(y, k)).sum::<i64>() + d - 1;
        let weekday = (unix.div_euclid(86_400) + 4).rem_euclid(7); // 1970-01-01 was a Thursday
        store(&mut self.regs, days * 86_400 + unix.rem_euclid(86_400), weekday);
        self.vl = false;
        // The value's fraction of a second sets the phase of the next tick.
        self.tick_ns = self.last_ns as i64 - ((value - secs) * NS as f64) as i64;
        true
    }

    fn state(&self, state: State, now_ns: u64) -> State {
        state.set("rtc", self.text(now_ns))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: u64 = 1_000_000_000;

    fn t(c: &mut Bm8563, write: &[u8], read_len: usize, now: u64) -> Vec<u8> {
        c.transfer(write, read_len, false, now).unwrap()
    }

    /// The seven time registers for a date, as the drivers write them.
    fn time_regs(y: i64, mo: i64, d: i64, wd: u8, h: i64, mi: i64, s: i64) -> [u8; 7] {
        [to_bcd(s), to_bcd(mi), to_bcd(h), to_bcd(d), wd, to_bcd(mo) | if y < 2000 { 0x80 } else { 0 }, to_bcd(y % 100)]
    }

    /// tr_board's decoding (tr_sensors.c:268-272), VL left out.
    fn tr_board(b: &[u8]) -> String {
        let bcd = |v: u8| (v >> 4) as i32 * 10 + (v & 0x0F) as i32;
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
            bcd(b[6]) + if b[5] & 0x80 != 0 { 1900 } else { 2000 },
            bcd(b[5] & 0x1F),
            bcd(b[3] & 0x3F),
            bcd(b[2] & 0x3F),
            bcd(b[1] & 0x7F),
            bcd(b[0] & 0x7F)
        )
    }

    /// Set the time with StackChan's 7-byte write from 02 at `now`.
    fn set(c: &mut Bm8563, regs: [u8; 7], now: u64) {
        let mut w = vec![SECONDS];
        w.extend_from_slice(&regs);
        assert_eq!(t(c, &w, 0, now), [] as [u8; 0]);
    }

    fn read7(c: &mut Bm8563, now: u64) -> Vec<u8> {
        t(c, &[SECONDS], 7, now)
    }

    /// M5Unified's getDateTime checks (PCF8563_Class.inl:30-85).
    fn m5unified_valid(b: &[u8]) -> bool {
        let valid = |v: u8| (v & 0x0F) <= 9 && (v >> 4) <= 9;
        let (s, mi, h, d, mo, y) = (b[0] & 0x7F, b[1] & 0x7F, b[2] & 0x3F, b[3] & 0x3F, b[5] & 0x1F, b[6]);
        if ![s, mi, h, d, mo, y].iter().all(|&v| valid(v)) {
            return false;
        }
        let (s, mi, h, d, mo) = (bcd(s), bcd(mi), bcd(h), bcd(d), bcd(mo));
        let year = bcd(y) + if b[5] & 0x80 != 0 { 1900 } else { 2000 };
        let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
        let dim = [31, if leap { 29 } else { 28 }, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
        s <= 59 && mi <= 59 && h <= 23 && (1..=12).contains(&mo) && d >= 1 && d <= dim[mo as usize - 1]
    }

    #[test]
    fn power_on_state_is_a_valid_date_with_vl_set() {
        let mut c = Bm8563::default();
        let b = read7(&mut c, 0);
        assert_eq!(b, [0x80, 0x00, 0x00, 0x01, 0x06, 0x01, 0x00]);
        assert!(m5unified_valid(&b), "M5Unified accepts it");
        assert_eq!(tr_board(&b), "2000-01-01 00:00:00");
        assert_eq!(t(&mut c, &[CTRL1], 2, 0), [0x08, 0x00]);
        assert_eq!(t(&mut c, &[0x09], 7, 0), [0x80, 0x80, 0x80, 0x80, 0x80, 0x03, 0x00]);
        assert_eq!(c.state(State::new(), 0).get("rtc"), Some("2000-01-01 00:00:00"));
    }

    #[test]
    fn probe_acks_without_moving_the_pointer() {
        let mut c = Bm8563::default();
        t(&mut c, &[0x05], 0, 0);
        assert_eq!(c.transfer(&[], 0, false, 0), Some(vec![]));
        assert_eq!(c.transfer(&[], 1, false, 0), Some(vec![0x01]), "a plain read goes on from 05");
    }

    #[test]
    fn m5unified_begin_read_and_disable_irq() {
        let mut c = Bm8563::default();
        // begin(): 00←00 twice, 0E←03 (PCF8563_Class.inl:13-23).
        t(&mut c, &[0x00, 0x00], 0, 0);
        t(&mut c, &[0x00, 0x00], 0, 0);
        t(&mut c, &[0x0E, 0x03], 0, 0);
        // setSystemTimeFromRtc(): 7 bytes from 02, validated.
        let b = read7(&mut c, 3 * S);
        assert!(m5unified_valid(&b));
        assert_eq!(tr_board(&b), "2000-01-01 00:00:03");
        // disableIRQ(): 09←80 80 80 80 as one burst, 0E←00, 01←00.
        t(&mut c, &[0x09, 0x80, 0x80, 0x80, 0x80], 0, 3 * S);
        t(&mut c, &[0x0E, 0x00], 0, 3 * S);
        t(&mut c, &[0x01, 0x00], 0, 3 * S);
        assert_eq!(t(&mut c, &[0x09], 6, 3 * S), [0x80, 0x80, 0x80, 0x80, 0x80, 0x00]);
        assert_eq!(t(&mut c, &[0x01], 1, 3 * S), [0x00]);
    }

    #[test]
    fn stackchan_write_then_read_keeps_running() {
        let mut c = Bm8563::default();
        // NTP sync: 2026-11-27 (a Friday, tm_wday 5) 13:06:40 UTC.
        set(&mut c, time_regs(2026, 11, 27, 5, 13, 6, 40), 10 * S);
        let b = read7(&mut c, 10 * S);
        assert_eq!(b, [0x40, 0x06, 0x13, 0x27, 0x05, 0x11, 0x26], "exact readback, VL cleared");
        assert_eq!(tr_board(&read7(&mut c, 12 * S)), "2026-11-27 13:06:42");
        assert_eq!(c.state(State::new(), 95 * S).get("rtc"), Some("2026-11-27 13:08:05"));
    }

    #[test]
    fn writing_the_time_keeps_the_sub_second_phase() {
        let mut c = Bm8563::default();
        set(&mut c, time_regs(2026, 1, 1, 4, 0, 0, 0), S / 2); // half-way into a second
        assert_eq!(read7(&mut c, S - 1)[0], 0x00);
        assert_eq!(read7(&mut c, S)[0], 0x01, "the tick comes at the old phase");
    }

    #[test]
    fn tr_board_masks_see_the_vl_bit_apart() {
        let mut c = Bm8563::default();
        let b = read7(&mut c, 61 * S);
        assert_eq!(b[0] & 0x80, 0x80, "not set: it lost power");
        assert_eq!(tr_board(&b), "2000-01-01 00:01:01");
    }

    #[test]
    fn vl_is_cleared_only_by_writing_the_seconds() {
        let mut c = Bm8563::default();
        t(&mut c, &[0x03, 0x10], 0, 0); // minutes only
        assert_eq!(t(&mut c, &[SECONDS], 1, 0), [0x80]);
        t(&mut c, &[SECONDS, 0x00], 0, 0);
        assert_eq!(t(&mut c, &[SECONDS], 1, 0), [0x00]);
    }

    #[test]
    fn rollovers_minute_hour_day_month_year_century() {
        let cases: [([u8; 7], &str, u8); 6] = [
            (time_regs(2026, 3, 5, 4, 10, 59, 59), "2026-03-05 11:00:00", 4),
            (time_regs(2026, 4, 30, 4, 23, 59, 59), "2026-05-01 00:00:00", 5),
            (time_regs(2026, 12, 31, 4, 23, 59, 59), "2027-01-01 00:00:00", 5),
            (time_regs(1999, 12, 31, 5, 23, 59, 59), "2000-01-01 00:00:00", 6),
            // 2099 to "1900": C toggles, and every driver reads C = 1 as 19xx.
            (time_regs(2099, 12, 31, 4, 23, 59, 59), "1900-01-01 00:00:00", 5),
            (time_regs(2026, 6, 30, 2, 12, 34, 59), "2026-06-30 12:35:00", 2),
        ];
        for (regs, after, weekday) in cases {
            let mut c = Bm8563::default();
            set(&mut c, regs, 0);
            let b = read7(&mut c, S);
            assert_eq!(tr_board(&b), after);
            assert_eq!(b[4], weekday, "{after}");
            assert!(m5unified_valid(&b) || after.starts_with("1900"), "{after}");
        }
        let mut c = Bm8563::default();
        set(&mut c, time_regs(1999, 12, 31, 5, 23, 59, 59), 0);
        assert_eq!(read7(&mut c, S)[5], 0x01, "C cleared on 99 to 00");
    }

    #[test]
    fn leap_days() {
        let mut c = Bm8563::default();
        set(&mut c, time_regs(2024, 2, 28, 3, 23, 59, 59), 0);
        assert_eq!(tr_board(&read7(&mut c, S)), "2024-02-29 00:00:00");
        assert_eq!(tr_board(&read7(&mut c, 86_401 * S)), "2024-03-01 00:00:00");
        let mut c = Bm8563::default();
        set(&mut c, time_regs(2023, 2, 28, 2, 23, 59, 59), 0);
        assert_eq!(tr_board(&read7(&mut c, S)), "2023-03-01 00:00:00");
        let mut c = Bm8563::default();
        set(&mut c, time_regs(2000, 2, 28, 1, 23, 59, 59), 0);
        assert_eq!(tr_board(&read7(&mut c, S)), "2000-02-29 00:00:00", "year 00 is a leap year");
    }

    #[test]
    fn a_long_gap_counts_every_second() {
        let mut c = Bm8563::default();
        set(&mut c, time_regs(2024, 1, 1, 1, 0, 0, 0), 0);
        // 366 days (2024 is a leap year) plus 1 h 2 min 3 s.
        let later = (366 * 86_400 + 3723) as u64 * S;
        let b = read7(&mut c, later);
        assert_eq!(tr_board(&b), "2025-01-01 01:02:03");
        assert_eq!(b[4], 3, "Monday + 366 days = Wednesday");
    }

    #[test]
    fn split_read_with_continues_reads_one_frozen_time() {
        let mut c = Bm8563::default();
        set(&mut c, time_regs(2026, 12, 31, 4, 23, 59, 59), 0);
        let first = c.transfer(&[SECONDS], 3, false, S - 1).unwrap();
        assert_eq!(first, [0x59, 0x59, 0x23]);
        // The rest arrives after the tick, but the access is still the same.
        let rest = c.transfer(&[], 4, true, S + 1).unwrap();
        assert_eq!(rest, [0x31, 0x04, 0x12, 0x26]);
        // The pending tick lands at the next access.
        assert_eq!(tr_board(&read7(&mut c, S + 2)), "2027-01-01 00:00:00");
    }

    #[test]
    fn pointer_wraps_from_0f_to_00() {
        let mut c = Bm8563::default();
        assert_eq!(t(&mut c, &[0x0E], 4, 0), [0x03, 0x00, 0x08, 0x00]);
        t(&mut c, &[0x0F, 0x05, 0x00], 0, 0); // 0F then 00
        assert_eq!(t(&mut c, &[0x00], 1, 0), [0x00]);
    }

    #[test]
    fn stop_freezes_the_clock_and_restarts_half_a_second_later() {
        let mut c = Bm8563::default();
        t(&mut c, &[CTRL1, STOP], 0, S / 3);
        assert_eq!(read7(&mut c, 10 * S)[0] & 0x7F, 0x00, "frozen");
        t(&mut c, &[CTRL1, 0x00], 0, 10 * S);
        assert_eq!(read7(&mut c, 10 * S + 507_000_000)[0] & 0x7F, 0x00);
        assert_eq!(read7(&mut c, 10 * S + 508_000_000)[0] & 0x7F, 0x01, "first tick after 0.5078 s");
        assert_eq!(read7(&mut c, 11 * S + 508_000_000)[0] & 0x7F, 0x02);
    }

    #[test]
    fn af_and_tf_clear_on_0_and_stay_on_1() {
        let mut c = Bm8563::default();
        c.regs[1] = AF | TF | 0x03;
        t(&mut c, &[CTRL2, 0x1F], 0, 0);
        assert_eq!(t(&mut c, &[CTRL2], 1, 0), [0x1F]);
        t(&mut c, &[CTRL2, 0x0B], 0, 0); // TF ← 0, AF ← 1
        assert_eq!(t(&mut c, &[CTRL2], 1, 0), [0x0B]);
        t(&mut c, &[CTRL2, 0x0C], 0, 0); // a flag cannot be set by writing 1
        assert_eq!(t(&mut c, &[CTRL2], 1, 0), [0x08]);
    }

    #[test]
    fn m5unified_alarm_fires_on_the_minute_and_clears() {
        let mut c = Bm8563::default();
        set(&mut c, time_regs(2026, 11, 27, 5, 7, 58, 30), 0);
        // setAlarmIRQ(07:59): disable AIE, alarm regs in one burst, clear AF,
        // then AIE on (PCF8563_Class.inl), each a read then a write of 01.
        let c2 = t(&mut c, &[CTRL2], 1, 0)[0];
        t(&mut c, &[CTRL2, (c2 & 0x11) | 0x0C], 0, 0);
        t(&mut c, &[0x09, 0x59, 0x07, 0x80, 0x80], 0, 0);
        let c2 = t(&mut c, &[CTRL2], 1, 0)[0];
        t(&mut c, &[CTRL2, (c2 & 0x11) | 0x04], 0, 0);
        let c2 = t(&mut c, &[CTRL2], 1, 0)[0];
        t(&mut c, &[CTRL2, (c2 & 0x11) | 0x0E], 0, 0);
        // getIRQstatus(): 01 & 0x0C.
        assert_eq!(t(&mut c, &[CTRL2], 1, 29 * S)[0] & 0x0C, 0, "07:58:59");
        assert_eq!(t(&mut c, &[CTRL2], 1, 30 * S)[0] & 0x0C, AF, "07:59:00");
        // clearIRQ(): bitOff(01, 0x0C).
        let v = t(&mut c, &[CTRL2], 1, 31 * S)[0];
        t(&mut c, &[CTRL2, v & !0x0C], 0, 31 * S);
        assert_eq!(t(&mut c, &[CTRL2], 1, 89 * S)[0], 0x02, "still 07:59: no second alarm");
        assert_eq!(t(&mut c, &[CTRL2], 1, 2 * 86_400 * S)[0] & AF, AF, "07:59 the next day");
    }

    #[test]
    fn alarm_is_not_set_by_writing_a_matching_time() {
        let mut c = Bm8563::default();
        t(&mut c, &[0x09, 0x30, 0x80, 0x80, 0x80], 0, 0); // minute 30
        set(&mut c, time_regs(2026, 1, 1, 4, 9, 30, 0), 0);
        assert_eq!(t(&mut c, &[CTRL2], 1, 10 * S)[0] & AF, 0);
        assert_eq!(t(&mut c, &[CTRL2], 1, 3600 * S)[0] & AF, AF, "10:30");
    }

    #[test]
    fn m5unified_timer_counts_down_and_sets_tf() {
        let mut c = Bm8563::default();
        // setTimerIRQ(3000): stop, load 3, start at 1 Hz, TIE on.
        let v = (t(&mut c, &[CTRL2], 1, 0)[0] & 0x13) | 0x08;
        t(&mut c, &[0x0E, 0x03], 0, 0);
        t(&mut c, &[CTRL2, v & !0x01], 0, 0);
        t(&mut c, &[0x0F, 3], 0, 0);
        t(&mut c, &[0x0E, 0x82], 0, 0);
        t(&mut c, &[CTRL2, (v | 0x01) & !0x80], 0, 0);
        assert_eq!(t(&mut c, &[0x0F], 1, S + 1), [2]);
        assert_eq!(t(&mut c, &[CTRL2], 1, 2 * S + 1), [0x01], "no TF yet");
        assert_eq!(t(&mut c, &[CTRL2], 1, 3 * S), [0x05], "TF at the end of the countdown");
        assert_eq!(t(&mut c, &[0x0F], 1, 3 * S), [3], "reloaded");
        // Stop: 0E←03, then TIE off with TF cleared.
        t(&mut c, &[0x0E, 0x03], 0, 4 * S);
        t(&mut c, &[CTRL2, 0x08], 0, 4 * S);
        assert_eq!(t(&mut c, &[CTRL2], 1, 100 * S), [0x00]);
    }

    #[test]
    fn timer_at_4096_hz() {
        let mut c = Bm8563::default();
        t(&mut c, &[0x0E, 0x80, 0x80], 0, 0); // on at 4096 Hz, 128 counts: 31.25 ms
        assert_eq!(t(&mut c, &[CTRL2], 1, 31_000_000)[0] & TF, 0);
        assert_eq!(t(&mut c, &[CTRL2], 1, 31_250_000)[0] & TF, TF);
    }

    #[test]
    fn time_input_sets_the_clock_from_unix_seconds() {
        let mut c = Bm8563::default();
        t(&mut c, &[SECONDS], 1, 5 * S); // the chip has seen t = 5 s
        assert!(c.input("time", 1_735_689_599.0)); // 2024-12-31 23:59:59 UTC, a Tuesday
        let b = read7(&mut c, 5 * S);
        assert_eq!(b, [0x59, 0x59, 0x23, 0x31, 0x02, 0x12, 0x24], "VL cleared");
        assert_eq!(tr_board(&read7(&mut c, 6 * S)), "2025-01-01 00:00:00");
        assert!(c.input("time", 951_782_400.5)); // 2000-02-29 00:00:00.5, a Tuesday
        assert_eq!(read7(&mut c, 6 * S)[3..5], [0x29, 0x02]);
        assert_eq!(read7(&mut c, 6 * S + S / 2)[0], 0x01, "the fraction sets the phase");
        assert!(c.input("time", -2_300_000_000.0), "1897: ours, but ignored");
        assert!(!c.input("lux", 1.0));
    }
}
