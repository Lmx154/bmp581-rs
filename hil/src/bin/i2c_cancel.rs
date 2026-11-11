//! Board measurement: what a cancelled (dropped) async I2C read does on embassy-rp 0.10.0, and which
//! recovery restores correct operation. BMP581 alone on I2C1 (SDA GPIO22, SCL GPIO23, 4.7k pull-ups),
//! address 0x46. Only READ transactions are cancelled; no register is written during the cancel phases.
#![no_std]
#![no_main]

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Waker};

use defmt_rtt as _;
use embassy_embedded_hal::SetConfig;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Flex, Pull};
use embassy_rp::i2c::{self, AbortReason, I2c};
use embassy_rp::pac;
use embassy_rp::peripherals::{I2C1, PIN_22, PIN_23};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

macro_rules! check {
    ($fails:ident, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "PASS" $(, $arg)*);
        } else {
            $fails += 1;
            defmt::error!($fmt, "FAIL" $(, $arg)*);
        }
    };
}

const ADDR: u8 = bmp581::DEFAULT_ADDRESS;

// The ONLY register pointers used in the cancel phases (cancelled reads, checking reads, recovery
// reads). BMP581 datasheet BST-BMP581-DS004-13, register map p. 50; all "R", no read side effect:
//   CHIP_ID 0x01: R, reset 0x50 (section 7.1, p. 51).
//   TEMP_DATA_XLSB 0x1D: R, reset 0x7F (7.11, p. 56). A 6-byte read from it covers TEMP_DATA_LSB 0x1E,
//   TEMP_DATA_MSB 0x1F, PRESS_DATA_XLSB 0x20 (7.12-7.14, p. 56), PRESS_DATA_LSB 0x21 and
//   PRESS_DATA_MSB 0x22 (7.15-7.16, p. 57): all R, reset 0x7F.
// Left out: CHIP_STATUS 0x11 (clear-on-read flag, p. 51), INT_STATUS 0x27 (clear-on-read, p. 58),
// FIFO_DATA 0x29 (pops the FIFO), CMD 0x7E, the NVM registers and every configuration register.
const PTR_ID: u8 = 0x01;
const PTR_DATA: u8 = 0x1D;
const ID: [u8; 1] = [0x50];

// Every read/write configuration register (p. 50) plus the read-only FIFO_COUNT, STATUS, OSR_EFF.
// Dumped with plain reads before the first cancel and after every block of cancels.
const DUMP: [u8; 19] = [
    0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x28, 0x2B, 0x2C, 0x2D, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38,
];
const NDUMP: usize = DUMP.len();

const SPEEDS: [u32; 3] = [400_000, 100_000, 1_000_000];
const RECS: [&str; 5] = ["a none", "b id-read", "c set_config", "d rebuild", "e busclear"];
const STEPS: u32 = 20;
const PER_STEP: u32 = 50;
const NREF: usize = 1000;
const DEADLINE: Duration = Duration::from_secs(95);
const MAX_CALLS: u32 = 13;

type Bus<'a> = I2c<'a, I2C1, i2c::Async>;

struct Hw {
    i2c: Peri<'static, I2C1>,
    scl: Peri<'static, PIN_23>,
    sda: Peri<'static, PIN_22>,
}

fn cfg(freq: u32) -> i2c::Config {
    let mut c = i2c::Config::default();
    c.frequency = freq;
    c.sda_pullup = false;
    c.scl_pullup = false;
    c
}

impl Hw {
    fn bus(&mut self, freq: u32) -> Bus<'_> {
        I2c::new_async(
            self.i2c.reborrow(),
            self.scl.reborrow(),
            self.sda.reborrow(),
            Irqs,
            cfg(freq),
        )
    }
}

/// (SDA high, SCL high) from the pad inputs; works while the pins belong to the I2C block.
fn lines() -> (bool, bool) {
    let v = pac::SIO.gpio_in(0).read();
    (v & (1 << 22) != 0, v & (1 << 23) != 0)
}

fn spin(us: u64) {
    let t = Instant::now();
    while t.elapsed().as_micros() < us {}
}

#[derive(Clone, Copy, PartialEq, defmt::Format)]
enum Outcome {
    Ok,
    Nack,
    Arb,
    Other,
    Wrong,
    Timeout,
}

#[derive(Default, Clone, Copy)]
struct Tally {
    dropped: u32,
    completed: u32,
    reads: u32,
    bad: u32,
    bad_after_completed: u32,
    nack: u32,
    arb: u32,
    other: u32,
    wrong: u32,
    timeout: u32,
    heal_max: u32,
    idle_low: u32,
    rec_low: u32,
    pulses: u32,
    rec_fail: u32,
    rec_us_sum: u64,
    rec_us_max: u32,
    rec_n: u32,
    unhealed: u32,
    restore_bad: u32,
    first_wrong_len: [u8; 2],
    first_wrong: [[u8; 6]; 2],
    first_other: u32,
}

impl Tally {
    fn merge(&mut self, o: &Tally) {
        self.dropped += o.dropped;
        self.completed += o.completed;
        self.reads += o.reads;
        self.bad += o.bad;
        self.bad_after_completed += o.bad_after_completed;
        self.nack += o.nack;
        self.arb += o.arb;
        self.other += o.other;
        self.wrong += o.wrong;
        self.timeout += o.timeout;
        self.heal_max = self.heal_max.max(o.heal_max);
        self.idle_low += o.idle_low;
        self.rec_low += o.rec_low;
        self.pulses += o.pulses;
        self.rec_fail += o.rec_fail;
        self.rec_us_sum += o.rec_us_sum;
        self.rec_us_max = self.rec_us_max.max(o.rec_us_max);
        self.rec_n += o.rec_n;
        self.unhealed += o.unhealed;
        self.restore_bad += o.restore_bad;
        for k in 0..2 {
            if self.first_wrong_len[k] == 0 && o.first_wrong_len[k] != 0 {
                self.first_wrong_len[k] = o.first_wrong_len[k];
                self.first_wrong[k] = o.first_wrong[k];
            }
        }
        if self.first_other == 0 {
            self.first_other = o.first_other;
        }
    }
    fn rec_mean(&self) -> u32 {
        (self.rec_us_sum / self.rec_n.max(1) as u64) as u32
    }
}

/// One plain read with its own timeout, classified against `exp`.
async fn check_read(
    bus: &mut Bus<'_>,
    ptr: u8,
    exp: &[u8],
    tmo: Duration,
    t: &mut Tally,
) -> Outcome {
    let mut buf = [0u8; 6];
    let n = exp.len();
    let r = with_timeout(tmo, AsyncI2c::write_read(bus, ADDR, &[ptr], &mut buf[..n])).await;
    let o = match r {
        Err(_) => Outcome::Timeout,
        Ok(Err(i2c::Error::Abort(AbortReason::NoAcknowledge))) => Outcome::Nack,
        Ok(Err(i2c::Error::Abort(AbortReason::ArbitrationLoss))) => Outcome::Arb,
        Ok(Err(e)) => {
            if t.first_other == 0 {
                t.first_other = match e {
                    i2c::Error::Abort(AbortReason::Other(c)) => c,
                    i2c::Error::Abort(AbortReason::TxNotEmpty(c)) => 0x1_0000 | c as u32,
                    _ => 0xFFFF_FFFF,
                };
            }
            Outcome::Other
        }
        Ok(Ok(())) if buf[..n] == *exp => Outcome::Ok,
        Ok(Ok(())) => Outcome::Wrong,
    };
    match o {
        Outcome::Ok => {}
        Outcome::Nack => t.nack += 1,
        Outcome::Arb => t.arb += 1,
        Outcome::Other => t.other += 1,
        Outcome::Timeout => t.timeout += 1,
        Outcome::Wrong => {
            t.wrong += 1;
            let k = (ptr == PTR_DATA) as usize;
            if t.first_wrong_len[k] == 0 {
                t.first_wrong_len[k] = n as u8;
                t.first_wrong[k] = buf;
            }
        }
    }
    o
}

/// CHIP_ID+REV_ID (2 bytes, so a stale CHIP_ID byte cannot pass), data, data, then alternating until two in a row are correct (at most MAX_CALLS calls).
/// Returns false when the bus did not come back.
async fn check_seq(
    bus: &mut Bus<'_>,
    exp: &[u8; 8],
    tmo: Duration,
    t: &mut Tally,
    after_completed: bool,
) -> bool {
    let (mut i, mut run, mut last_bad) = (0u32, 0u32, 0u32);
    loop {
        let (ptr, exp): (u8, &[u8]) = match i {
            0 => (PTR_ID, &exp[..2]),
            1 | 2 => (PTR_DATA, &exp[2..]),
            _ if i % 2 == 1 => (PTR_ID, &exp[..2]),
            _ => (PTR_DATA, &exp[2..]),
        };
        let o = check_read(bus, ptr, exp, tmo, t).await;
        i += 1;
        t.reads += 1;
        if o == Outcome::Ok {
            run += 1;
        } else {
            run = 0;
            last_bad = i;
            t.bad += 1;
            if after_completed {
                t.bad_after_completed += 1;
            }
        }
        if i >= 3 && run >= 2 {
            t.heal_max = t.heal_max.max(last_bad);
            return true;
        }
        if i >= MAX_CALLS {
            t.heal_max = t.heal_max.max(MAX_CALLS);
            return false;
        }
    }
}

/// Starts a 6-byte data read, polls it in a tight loop and drops it `d_us` after the first poll.
/// Returns true when it completed before that point (no cancel happened).
fn cancel_after(bus: &mut Bus<'_>, d_us: u64) -> bool {
    let mut buf = [0u8; 6];
    let mut cx = Context::from_waker(Waker::noop());
    let t0 = Instant::now();
    let mut fut = pin!(AsyncI2c::write_read(bus, ADDR, &[PTR_DATA], &mut buf));
    loop {
        if fut.as_mut().poll(&mut cx).is_ready() {
            return true;
        }
        if t0.elapsed().as_micros() >= d_us {
            return false;
        }
    }
}

/// Bus clear with the pins as GPIO: SCL pulses only while SDA reads low (at most nine), then a
/// START+STOP made with SDA alone while SCL is high. With SDA high no clock is given, so a device
/// that is receiving can never be clocked into a data byte. Returns (SDA was low, SCL was low, pulses).
fn bus_clear(hw: &mut Hw) -> (bool, bool, u8) {
    let mut scl = Flex::new(hw.scl.reborrow());
    let mut sda = Flex::new(hw.sda.reborrow());
    scl.set_pull(Pull::None);
    sda.set_pull(Pull::None);
    scl.set_low();
    sda.set_low();
    scl.set_as_input();
    sda.set_as_input();
    spin(5);
    let (sda_low, scl_low) = (sda.is_low(), scl.is_low());
    let mut pulses = 0u8;
    while sda.is_low() && pulses < 9 {
        scl.set_as_output();
        spin(5);
        scl.set_as_input();
        spin(5);
        pulses += 1;
    }
    if sda.is_high() && scl.is_high() {
        sda.set_as_output();
        spin(5);
        sda.set_as_input();
        spin(5);
    }
    (sda_low, scl_low, pulses)
}

async fn dump(bus: &mut Bus<'_>, out: &mut [u8; NDUMP]) -> Result<(), u8> {
    for (i, &r) in DUMP.iter().enumerate() {
        let mut b = [0u8; 1];
        match with_timeout(
            Duration::from_millis(50),
            AsyncI2c::write_read(bus, ADDR, &[r], &mut b),
        )
        .await
        {
            Ok(Ok(())) => out[i] = b[0],
            _ => return Err(r),
        }
    }
    Ok(())
}

fn first_diff(a: &[u8; NDUMP], b: &[u8; NDUMP]) -> Option<(u8, u8, u8)> {
    (0..NDUMP).find(|&i| a[i] != b[i]).map(|i| (DUMP[i], a[i], b[i]))
}

#[derive(PartialEq, defmt::Format)]
enum Stop {
    RegisterChanged,
    Unresponsive,
    DumpFailed,
    Deadline,
}

/// Three CHIP_ID reads in a row must be correct.
async fn guard(bus: &mut Bus<'_>, tmo: Duration) -> bool {
    let mut t = Tally::default();
    for _ in 0..3 {
        if check_read(bus, PTR_ID, &ID, tmo, &mut t).await != Outcome::Ok {
            return false;
        }
    }
    true
}

/// Reference timings: 1-byte and 6-byte write_read, NREF each. Returns (worst6, bad reads).
async fn reference(bus: &mut Bus<'_>, freq: u32, exp: &[u8; 8]) -> (u32, u32) {
    let mut d1 = [0u16; NREF];
    let mut d6 = [0u16; NREF];
    let mut t = Tally::default();
    let tmo = Duration::from_millis(20);
    for i in 0..NREF {
        let t0 = Instant::now();
        let o1 = check_read(bus, PTR_ID, &ID, tmo, &mut t).await;
        d1[i] = t0.elapsed().as_micros().min(65535) as u16;
        let t0 = Instant::now();
        let o6 = check_read(bus, PTR_DATA, &exp[2..], tmo, &mut t).await;
        d6[i] = t0.elapsed().as_micros().min(65535) as u16;
        t.bad += (o1 != Outcome::Ok) as u32 + (o6 != Outcome::Ok) as u32;
    }
    d1.sort_unstable();
    d6.sort_unstable();
    defmt::info!(
        "[STAT] ref {=u32} kHz: 1-byte write_read median {=u16} us worst {=u16} us; 6-byte median {=u16} us worst {=u16} us; {=u32} of {=u32} reads not correct (wrong {=u32} nack {=u32} arb {=u32} other {=u32} timeout {=u32})",
        freq / 1000,
        d1[NREF / 2],
        d1[NREF - 1],
        d6[NREF / 2],
        d6[NREF - 1],
        t.bad,
        2 * NREF as u32,
        t.wrong,
        t.nack,
        t.arb,
        t.other,
        t.timeout
    );
    (d6[NREF - 1] as u32, t.bad)
}

/// Cancels at d = 0 and records the controller state the drop leaves behind.
async fn diag(bus: &mut Bus<'_>, freq: u32, exp: &[u8; 8], tmo: Duration) {
    let r = pac::I2C1;
    let (mut tx_min, mut tx_max, mut rx_min, mut rx_max) = (99u8, 0u8, 99u8, 0u8);
    let (mut stop, mut tx_left, mut idle_max, mut done) = (0u32, 0u32, 0u32, 0u32);
    let mut t = Tally::default();
    for _ in 0..10 {
        if cancel_after(bus, 0) {
            done += 1;
        }
        let tx = r.ic_txflr().read().txflr();
        let t0 = Instant::now();
        while r.ic_status().read().mst_activity() && t0.elapsed().as_micros() < 5000 {}
        idle_max = idle_max.max(t0.elapsed().as_micros() as u32);
        spin(20);
        let rx = r.ic_rxflr().read().rxflr();
        stop += r.ic_raw_intr_stat().read().stop_det() as u32;
        tx_left += (r.ic_txflr().read().txflr() != 0) as u32;
        tx_min = tx_min.min(tx);
        tx_max = tx_max.max(tx);
        rx_min = rx_min.min(rx);
        rx_max = rx_max.max(rx);
        check_seq(bus, exp, tmo, &mut t, false).await;
    }
    defmt::info!(
        "[STAT] diag {=u32} kHz, drop at d=0 (10x, {=u32} completed first): TX FIFO right after drop {=u8}..{=u8} entries; controller idle after <= {=u32} us; then RX FIFO {=u8}..{=u8} bytes, TX FIFO not empty {=u32}/10, STOP_DET set {=u32}/10; following checking reads not correct {=u32} of {=u32}",
        freq / 1000,
        done,
        tx_min,
        tx_max,
        idle_max,
        rx_min,
        rx_max,
        tx_left,
        stop,
        t.bad,
        t.reads
    );
}

fn print_tally(freq: u32, rec: usize, d: i32, t: &Tally) {
    defmt::info!(
        "[STAT] {=u32} kHz {=str} d={=i32}: dropped {=u32} completed {=u32}; checks {=u32} not correct {=u32} (after completed {=u32}; wrong {=u32} nack {=u32} arb {=u32} other {=u32} timeout {=u32}); worst calls to correct {=u32}; line low idle {=u32} at recovery {=u32}; pulses {=u32}; rec read failed {=u32}; rec cost mean {=u32} max {=u32} us; not self-healed in {=u32} calls {=u32} (wrong after 2 ms idle restore {=u32})",
        freq / 1000,
        RECS[rec],
        d,
        t.dropped,
        t.completed,
        t.reads,
        t.bad,
        t.bad_after_completed,
        t.wrong,
        t.nack,
        t.arb,
        t.other,
        t.timeout,
        t.heal_max,
        t.idle_low,
        t.rec_low,
        t.pulses,
        t.rec_fail,
        t.rec_mean(),
        t.rec_us_max,
        MAX_CALLS,
        t.unhealed,
        t.restore_bad
    );
    for k in 0..2 {
        if t.first_wrong_len[k] != 0 {
            defmt::info!(
                "[INFO]   first wrong read: pointer {=u8:#04x} returned {=[u8]:#04x}",
                [PTR_ID, PTR_DATA][k],
                t.first_wrong[k][..t.first_wrong_len[k] as usize]
            );
        }
    }
    if t.first_other != 0 {
        defmt::info!("[INFO]   first other error code {=u32:#x}", t.first_other);
    }
}

/// One sweep: STEPS delays from 0 to `span_us`, PER_STEP cancels each, recovery `rec` after each
/// cancel, the checking reads, then the guard reads and the register dump after every step.
#[allow(clippy::too_many_arguments)]
async fn sweep(
    hw: &mut Hw,
    freq: u32,
    rec: usize,
    span_us: u32,
    tmo: Duration,
    exp: &[u8; 8],
    base: &[u8; NDUMP],
    start: Instant,
    total: &mut Tally,
) -> Result<(), Stop> {
    let mut bus = hw.bus(freq);
    let mut stop = None;
    let mut blocks = 0u32;
    'steps: for step in 0..STEPS {
        if start.elapsed() > DEADLINE {
            stop = Some(Stop::Deadline);
            break;
        }
        let d = span_us as u64 * step as u64 / (STEPS - 1) as u64;
        let mut t = Tally::default();
        for _ in 0..PER_STEP {
            let completed = cancel_after(&mut bus, d);
            if completed {
                t.completed += 1;
            } else {
                t.dropped += 1;
            }
            let r0 = Instant::now();
            match rec {
                0 => {}
                1 => {
                    let mut b = [0u8; 1];
                    match with_timeout(tmo, AsyncI2c::write_read(&mut bus, ADDR, &[PTR_ID], &mut b))
                        .await
                    {
                        Ok(Ok(())) => {}
                        _ => t.rec_fail += 1,
                    }
                }
                2 => {
                    let _ = bus.set_config(&cfg(freq));
                }
                3 => {
                    drop(bus);
                    bus = hw.bus(freq);
                    let (sda, scl) = lines();
                    t.rec_low += (!sda || !scl) as u32;
                }
                _ => {
                    drop(bus);
                    let (sl, cl, n) = bus_clear(hw);
                    t.rec_low += (sl || cl) as u32;
                    t.pulses += n as u32;
                    bus = hw.bus(freq);
                }
            }
            if rec != 0 {
                let us = r0.elapsed().as_micros() as u32;
                t.rec_us_sum += us as u64;
                t.rec_us_max = t.rec_us_max.max(us);
                t.rec_n += 1;
            }
            let errs0 = t.nack + t.arb + t.other + t.timeout;
            let mut healed = check_seq(&mut bus, exp, tmo, &mut t, completed).await;
            let (sda, scl) = lines();
            t.idle_low += (!sda || !scl) as u32;
            if !healed && t.nack + t.arb + t.other + t.timeout == errs0 {
                // Every call was acknowledged and completed; only the data was wrong. That is HAL
                // state, not the device: let the bus go idle (no bus traffic) and check again.
                t.unhealed += 1;
                Timer::after_millis(2).await;
                let mut r = Tally::default();
                healed = check_seq(&mut bus, exp, tmo, &mut r, false).await;
                t.restore_bad += r.bad;
            }
            if !healed {
                print_tally(freq, rec, d as i32, &t);
                defmt::error!(
                    "[FAIL] {=u32} kHz {=str} d={=u64} us: device not answering correctly after {=u32} calls (lines SDA {=bool} SCL {=bool})",
                    freq / 1000,
                    RECS[rec],
                    d,
                    MAX_CALLS,
                    sda,
                    scl
                );
                stop = Some(Stop::Unresponsive);
                total.merge(&t);
                break 'steps;
            }
        }
        if rec == 0 || t.bad != 0 || t.idle_low != 0 || t.rec_low != 0 || t.rec_fail != 0 {
            print_tally(freq, rec, d as i32, &t);
        }
        total.merge(&t);
        blocks += 1;
        // Register check after every block: guard reads (read-only pointer) first, then the dump.
        if !guard(&mut bus, tmo).await {
            defmt::error!(
                "[FAIL] {=u32} kHz {=str} d={=u64}: guard CHIP_ID reads failed before the dump",
                freq / 1000,
                RECS[rec],
                d
            );
            stop = Some(Stop::Unresponsive);
            break;
        }
        let mut now = [0u8; NDUMP];
        if let Err(reg) = dump(&mut bus, &mut now).await {
            defmt::error!(
                "[FAIL] register dump read of {=u8:#04x} failed; ending without further bus traffic",
                reg
            );
            return Err(Stop::DumpFailed);
        }
        if let Some((reg, old, new)) = first_diff(base, &now) {
            defmt::error!(
                "[FAIL] REGISTER CHANGED at {=u32} kHz {=str} d={=u64} us: reg {=u8:#04x} was {=u8:#04x} now {=u8:#04x}",
                freq / 1000,
                RECS[rec],
                d,
                reg,
                old,
                new
            );
            return Err(Stop::RegisterChanged);
        }
    }
    defmt::info!(
        "[INFO] {=u32} kHz {=str}: register dump identical to the first after each of {=u32} blocks",
        freq / 1000,
        RECS[rec],
        blocks
    );
    match stop {
        None => Ok(()),
        Some(Stop::Unresponsive) => {
            // Envelope 6: try each recovery once, record which works, end the run.
            let mut t = Tally::default();
            let mut b = [0u8; 1];
            let _ = with_timeout(tmo, AsyncI2c::write_read(&mut bus, ADDR, &[PTR_ID], &mut b)).await;
            let ok_b = guard(&mut bus, tmo).await;
            let _ = bus.set_config(&cfg(freq));
            let ok_c = guard(&mut bus, tmo).await;
            drop(bus);
            bus = hw.bus(freq);
            let ok_d = guard(&mut bus, tmo).await;
            drop(bus);
            let (sl, cl, n) = bus_clear(hw);
            bus = hw.bus(freq);
            let ok_e = guard(&mut bus, tmo).await;
            t.pulses = n as u32;
            defmt::error!(
                "[STAT] unresponsive: after b id-read {=bool}, c set_config {=bool}, d rebuild {=bool}, e busclear {=bool} (SDA low {=bool}, SCL low {=bool}, pulses {=u32})",
                ok_b,
                ok_c,
                ok_d,
                ok_e,
                sl,
                cl,
                t.pulses
            );
            Err(Stop::Unresponsive)
        }
        Some(s) => Err(s),
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let start = Instant::now();
    let mut hw = Hw {
        i2c: p.I2C1,
        scl: p.PIN_23,
        sda: p.PIN_22,
    };
    let mut failures = 0u32;
    let tmo0 = Duration::from_millis(20);

    // Soft reset through the driver: before the cancel phases, and only here.
    // exp[0..2]: CHIP_ID, REV_ID; exp[2..8]: the six data registers in their reset state.
    let mut exp = [0u8; 8];
    let mut base = [0u8; NDUMP];
    {
        let mut baro = bmp581::Bmp581::new(hw.bus(400_000), ADDR);
        // A 2-byte write takes about 70 us; if 100 ms pass, the bus is hung and the run ends here
        // with no further bus traffic.
        let Ok(r) = with_timeout(
            Duration::from_millis(100),
            baro.write_register(bmp581::reg::CMD, bmp581::reg::CMD_SOFT_RESET),
        )
        .await
        else {
            defmt::error!("[FAIL] soft reset write did not complete within 100 ms");
            hil_common::exit(1)
        };
        Timer::after_millis(5).await;
        let id = with_timeout(tmo0, baro.read_register(bmp581::reg::CHIP_ID)).await;
        check!(
            failures,
            matches!(id, Ok(Ok(0x50))),
            "[{=str}] soft reset (write result ok {=bool}), CHIP_ID afterwards {}",
            r.is_ok(),
            defmt::Debug2Format(&id)
        );
        let mut bus = baro.release();
        let ok = matches!(
            with_timeout(tmo0, AsyncI2c::write_read(&mut bus, ADDR, &[PTR_DATA], &mut exp[2..])).await,
            Ok(Ok(()))
        ) && matches!(
            with_timeout(tmo0, AsyncI2c::write_read(&mut bus, ADDR, &[PTR_ID], &mut exp[..2])).await,
            Ok(Ok(()))
        ) && exp[0] == 0x50;
        check!(
            failures,
            ok && guard(&mut bus, tmo0).await && dump(&mut bus, &mut base).await.is_ok(),
            "[{=str}] CHIP_ID, REV_ID, reset-state data {=[u8]:#04x} and first register dump {=[u8]:#04x}",
            exp,
            base
        );
        if failures != 0 {
            hil_common::exit(failures)
        }

        // Positive controls: the classifier and the dump comparison can fail.
        let mut t = Tally::default();
        let o = check_read(&mut bus, PTR_ID, &[0x51], tmo0, &mut t).await;
        check!(
            failures,
            o == Outcome::Wrong && t.wrong == 1,
            "[{=str}] control: CHIP_ID checked against 0x51 classified {} (wrong count {=u32})",
            o,
            t.wrong
        );
        let mut altered = base;
        altered[17] ^= 0x01;
        check!(
            failures,
            first_diff(&base, &altered) == Some((0x37, base[17], base[17] ^ 0x01)),
            "[{=str}] control: dump comparison reports an altered ODR_CONFIG copy"
        );
    }

    let mut stopped: Option<Stop> = None;
    // Recovery d (rebuild alone) can leave SDA held low (run 1), which ends the run: it goes last.
    let mut d_args: Option<(u32, Duration)> = None;
    'speeds: for &freq in SPEEDS.iter() {
        if start.elapsed() > DEADLINE {
            stopped = Some(Stop::Deadline);
            break;
        }
        let (worst6, tmo) = {
            let mut bus = hw.bus(freq);
            let (worst6, bad) = reference(&mut bus, freq, &exp).await;
            let tmo = Duration::from_micros((worst6 as u64 * 10).max(2000));
            if bad != 0 {
                if freq == 400_000 {
                    check!(failures, false, "[{=str}] reference reads at 400 kHz all correct");
                } else {
                    defmt::info!(
                        "[INFO] {=u32} kHz: reference reads not all correct, no cancel sweep at this speed",
                        freq / 1000
                    );
                }
                continue;
            }
            diag(&mut bus, freq, &exp, tmo).await;
            let mut now = [0u8; NDUMP];
            let ok = guard(&mut bus, tmo).await;
            if !ok || dump(&mut bus, &mut now).await.is_err() || first_diff(&base, &now).is_some() {
                defmt::error!(
                    "[FAIL] after the diag block: guard ok {=bool}, dump {=[u8]:#04x}",
                    ok,
                    now
                );
                hil_common::exit(failures + 1)
            }
            (worst6, tmo)
        };
        let span = worst6 * 115 / 100;
        defmt::info!(
            "[INFO] {=u32} kHz: cancel delays 0..{=u32} us in {=u32} steps, {=u32} cancels per step, checking-read timeout {=u64} us",
            freq / 1000,
            span,
            STEPS,
            PER_STEP,
            tmo.as_micros()
        );
        if freq == 400_000 {
            d_args = Some((span, tmo));
        }
        for rec in [0usize, 1, 2, 4] {
            let mut total = Tally::default();
            let r = sweep(&mut hw, freq, rec, span, tmo, &exp, &base, start, &mut total).await;
            print_tally(freq, rec, -1, &total);
            if rec != 0 && r.is_ok() {
                check!(
                    failures,
                    total.bad == 0,
                    "[{=str}] {=u32} kHz {=str}: {=u32} of {=u32} checking reads wrong or failed after the recovery",
                    freq / 1000,
                    RECS[rec],
                    total.bad,
                    total.reads
                );
            }
            if let Err(s) = r {
                stopped = Some(s);
                break 'speeds;
            }
        }
    }

    if let (None, Some((span, tmo))) = (&stopped, d_args) {
        let mut total = Tally::default();
        let r = sweep(&mut hw, 400_000, 3, span, tmo, &exp, &base, start, &mut total).await;
        print_tally(400_000, 3, -1, &total);
        if let Err(s) = r {
            stopped = Some(s);
        } else {
            check!(
                failures,
                total.bad == 0,
                "[{=str}] 400 kHz d rebuild: {=u32} of {=u32} checking reads wrong or failed after the recovery",
                total.bad,
                total.reads
            );
        }
    }

    match stopped {
        Some(Stop::RegisterChanged) | Some(Stop::DumpFailed) => {
            hil_common::exit(failures + 1);
        }
        Some(Stop::Unresponsive) => failures += 1,
        Some(Stop::Deadline) => defmt::info!("[INFO] deadline reached, remaining phases skipped"),
        None => {}
    }

    // Final state: the device answers and the registers equal the first dump.
    let mut bus = hw.bus(400_000);
    let ok = guard(&mut bus, tmo0).await;
    let mut now = [0u8; NDUMP];
    let same = ok && dump(&mut bus, &mut now).await.is_ok() && first_diff(&base, &now).is_none();
    check!(
        failures,
        ok && same,
        "[{=str}] end: device answers {=bool}, register dump equal to the first {=bool} ({=[u8]:#04x}); run took {=u64} ms",
        ok,
        same,
        now,
        start.elapsed().as_millis()
    );
    hil_common::exit(failures)
}
