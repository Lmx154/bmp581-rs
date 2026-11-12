//! On-board pass/fail test of contract rules A4g, A4h and A4i for the bmp581 driver: a cancelled I2C read
//! is followed by `recover()` and no read is wrong afterwards. BMP581 alone on I2C1 (SDA GPIO22, SCL
//! GPIO23, 4.7k pull-ups), address 0x46.
//!
//! Only READS of read-only registers are cancelled: `Bmp581::read` (6 bytes from TEMP_DATA_XLSB 0x1D, the
//! six data registers) and a 2-byte read from CHIP_ID 0x01 (CHIP_ID, REV_ID). No write is ever cancelled.
//!
//! How a read is judged wrong (the true pressure is not known): the identity read must return
//! [0x50, REV_ID of init]; a data read must decode to a pressure within P_TOL of the mean of 200 reads
//! taken before any cancel, a temperature within T_TOL, and not the reset value. The fault measured on
//! this HAL leaves received bytes of the cancelled frame in the receive FIFO, so the next read returns
//! stale bytes at a shifted position: an identity read gets data bytes, a data read gets identity bytes
//! or shifted data (both far outside the tolerance). Every checking round reads identity, data,
//! identity, data, so a leftover of either kind lands on a read of the other kind. C1 shows that this
//! judge sees the fault when no recovery is made.
#![no_std]
#![no_main]

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use bmp581::{Bmp581, Config, OutputDataRate, Oversampling, RawSample, reg};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Flex, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::pac;
use embassy_rp::peripherals::{I2C1, PIN_22, PIN_23};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

macro_rules! check {
    ($fails:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "PASS" $(, $arg)*);
        } else {
            $fails += 1;
            defmt::error!($fmt, "FAIL" $(, $arg)*);
        }
    };
}

const ADDR: u8 = bmp581::DEFAULT_ADDRESS;

/// Every read/write configuration register plus the read-only FIFO_COUNT, STATUS and OSR_EFF. None is
/// clear-on-read. Dumped with plain reads before the first cancel and after every block of cancels.
const DUMP: [u8; 19] = [
    0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x28, 0x2B, 0x2C, 0x2D, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38,
];
const ND: usize = DUMP.len();

const P_TOL: f32 = 50.0;
const T_TOL: f32 = 2.0;
const PER_BLOCK: u32 = 50;
const NPHASE: u32 = 8;
const RD_TMO: Duration = Duration::from_millis(20);
const REC_TMO: Duration = Duration::from_millis(100);
const RUN_LIMIT: Duration = Duration::from_secs(200);
const C1_N: u32 = 200;
const C2_BLOCKS: u32 = 20;
const C3_BLOCKS: u32 = 10;
const C4_TRIES: u32 = 60;

type Bus<'a> = I2c<'a, I2C1, i2c::Async>;
type Baro<'a> = Bmp581<Bus<'a>>;

struct Hw {
    i2c: Peri<'static, I2C1>,
    scl: Peri<'static, PIN_23>,
    sda: Peri<'static, PIN_22>,
}

impl Hw {
    fn bus(&mut self, freq: u32) -> Bus<'_> {
        let mut c = i2c::Config::default();
        c.frequency = freq;
        c.sda_pullup = false;
        c.scl_pullup = false;
        I2c::new_async(
            self.i2c.reborrow(),
            self.scl.reborrow(),
            self.sda.reborrow(),
            Irqs,
            c,
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

fn abs(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
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

/// Polls `fut` in a tight loop with a no-op waker and drops it `d_us` after the first poll.
/// None: it was dropped before it finished (a cancel). Some: it finished first.
fn cancel<F: Future>(fut: F, d_us: u64) -> Option<F::Output> {
    let mut cx = Context::from_waker(Waker::noop());
    let t0 = Instant::now();
    let mut fut = pin!(fut);
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return Some(v);
        }
        if t0.elapsed().as_micros() >= d_us {
            return None;
        }
    }
}

fn hang(what: &str, fails: u32) -> ! {
    defmt::error!(
        "[FAIL] {=str}: no answer within its bound; the sensor or the bus is stuck, stopping with no further bus traffic (a power cycle may be needed)",
        what
    );
    hil_common::exit(fails + 1)
}

fn first_diff(a: &[u8; ND], b: &[u8; ND]) -> Option<(u8, u8, u8)> {
    (0..ND)
        .find(|&i| a[i] != b[i])
        .map(|i| (DUMP[i], a[i], b[i]))
}

#[derive(Clone, Copy)]
struct Refs {
    p: f32,
    t: f32,
    rev: u8,
}

impl Refs {
    fn id_ok(&self, b: &[u8; 2]) -> bool {
        *b == [reg::CHIP_ID_VALUE, self.rev]
    }
    fn data_ok(&self, b: &[u8; 6]) -> bool {
        let s = RawSample::decode(b);
        !s.is_reset_value()
            && abs(s.pascals() - self.p) <= P_TOL
            && abs(s.celsius() - self.t) <= T_TOL
    }
}

enum Rd {
    Ok,
    Wrong([u8; 6], u8),
    Bus,
    Timeout,
}

#[derive(Default, Clone, Copy)]
struct Tally {
    dropped_d: u32,
    dropped_i: u32,
    completed: u32,
    rec_fail: u32,
    rec_n: u32,
    rec_sum_us: u64,
    rec_max_us: u32,
    reads: u32,
    wrong: u32,
    bus: u32,
    blocks: u32,
    dump_equal: u32,
    dump_changed: u32,
    dump_fail: u32,
    bus_errors_driver: u32,
}

struct Setup {
    base: [u8; ND],
    span_d: u32,
    span_i: u32,
}

struct Ctx<'a> {
    baro: Baro<'a>,
    refs: Refs,
    fails: u32,
    start: Instant,
}

fn show(first: &mut bool, kind: &str, d: u64, what: &str, idx: u32, r: &Rd) {
    if !*first {
        return;
    }
    *first = false;
    match r {
        Rd::Wrong(b, n) => defmt::error!(
            "[INFO] first failing case: cancelled {=str} at d={=u64} us, checking read {=u32} ({=str}) returned {=[u8]:#04x}",
            kind,
            d,
            idx,
            what,
            b[..*n as usize]
        ),
        _ => defmt::error!(
            "[INFO] first failing case: cancelled {=str} at d={=u64} us, checking read {=u32} ({=str}) bus error",
            kind,
            d,
            idx,
            what
        ),
    }
}

impl<'a> Ctx<'a> {
    fn new(bus: Bus<'a>, start: Instant) -> Self {
        Self {
            baro: Bmp581::new(bus, ADDR),
            refs: Refs {
                p: 0.0,
                t: 0.0,
                rev: 0,
            },
            fails: 0,
            start,
        }
    }

    async fn init(&mut self, cfg: &Config) -> Result<(), ()> {
        match with_timeout(Duration::from_millis(500), self.baro.init(&mut Delay, cfg)).await {
            Err(_) => hang("init", self.fails),
            Ok(Err(e)) => {
                defmt::error!("[INFO] init failed: {}", defmt::Debug2Format(&e));
                Err(())
            }
            Ok(Ok(info)) => {
                self.refs.rev = info.revision;
                Ok(())
            }
        }
    }

    async fn rd_id(&mut self) -> Rd {
        let mut b = [0u8; 2];
        match with_timeout(RD_TMO, self.baro.read_registers(reg::CHIP_ID, &mut b)).await {
            Err(_) => Rd::Timeout,
            Ok(Err(_)) => Rd::Bus,
            Ok(Ok(())) if self.refs.id_ok(&b) => Rd::Ok,
            Ok(Ok(())) => {
                let mut o = [0u8; 6];
                o[..2].copy_from_slice(&b);
                Rd::Wrong(o, 2)
            }
        }
    }

    async fn rd_data(&mut self) -> Rd {
        let mut b = [0u8; 6];
        match with_timeout(
            RD_TMO,
            self.baro.read_registers(reg::TEMP_DATA_XLSB, &mut b),
        )
        .await
        {
            Err(_) => Rd::Timeout,
            Ok(Err(_)) => Rd::Bus,
            Ok(Ok(())) if self.refs.data_ok(&b) => Rd::Ok,
            Ok(Ok(())) => Rd::Wrong(b, 6),
        }
    }

    /// Counts the read in `t`; true when it was right. A timeout ends the run.
    fn tally(&self, t: &mut Tally, r: &Rd) -> bool {
        match r {
            Rd::Ok => true,
            Rd::Wrong(..) => {
                t.wrong += 1;
                false
            }
            Rd::Bus => {
                t.bus += 1;
                false
            }
            Rd::Timeout => hang("checking read", self.fails),
        }
    }

    async fn dump(&mut self) -> Result<[u8; ND], u8> {
        let mut out = [0u8; ND];
        for (i, &r) in DUMP.iter().enumerate() {
            match with_timeout(RD_TMO, self.baro.read_register(r)).await {
                Ok(Ok(v)) => out[i] = v,
                Err(_) => hang("register dump", self.fails),
                _ => return Err(r),
            }
        }
        Ok(out)
    }

    /// Identity, data, identity, data after a cancel (and its recovery, if any).
    async fn check4(&mut self, t: &mut Tally, first: &mut bool, kind: &str, d: u64) {
        for i in 0..4u32 {
            let id = i % 2 == 0;
            let r = if id {
                self.rd_id().await
            } else {
                self.rd_data().await
            };
            t.reads += 1;
            if !self.tally(t, &r) {
                show(
                    first,
                    kind,
                    d,
                    if id {
                        "CHIP_ID+REV_ID"
                    } else {
                        "data, 6 bytes"
                    },
                    i,
                    &r,
                );
            }
        }
    }

    /// Init with `cfg`, reference, 300 uncancelled checking rounds (timed), oracle controls, baseline
    /// dump. None when a check failed.
    async fn setup(&mut self, cfg: &Config, label: &str) -> Option<Setup> {
        let f0 = self.fails;
        let inited = self.init(cfg).await;
        check!(
            self.fails,
            inited.is_ok(),
            "[{=str}] {=str}: init to the test configuration (normal 240 Hz, x1/x1)",
            label
        );
        inited.ok()?;
        Timer::after_millis(100).await;
        let v = with_timeout(RD_TMO, self.baro.verify()).await;
        check!(
            self.fails,
            matches!(v, Ok(Ok(()))),
            "[{=str}] {=str}: verify() after init",
            label
        );

        let mut first = [0u8; 6];
        let (mut p0, mut t0) = (0f32, 0f32);
        let (mut sp, mut st, mut got) = (0f32, 0f32, 0u32);
        for i in 0..200 {
            let mut b = [0u8; 6];
            if let Ok(Ok(())) = with_timeout(
                RD_TMO,
                self.baro.read_registers(reg::TEMP_DATA_XLSB, &mut b),
            )
            .await
            {
                let s = RawSample::decode(&b);
                if !s.is_reset_value() {
                    if got == 0 {
                        first = b;
                        p0 = s.pascals();
                        t0 = s.celsius();
                    }
                    sp += s.pascals() - p0;
                    st += s.celsius() - t0;
                    got += 1;
                }
            }
            let _ = i;
            Timer::after_micros(500).await;
        }
        check!(
            self.fails,
            got == 200,
            "[{=str}] {=str}: 200 reference reads returned samples ({=u32})",
            label,
            got
        );
        if got == 0 {
            return None;
        }
        self.refs.p = p0 + sp / got as f32;
        self.refs.t = t0 + st / got as f32;
        defmt::info!(
            "[STAT] {=str}: reference pressure {=f32} Pa, temperature {=f32} degC, REV_ID {=u8:#04x}, tolerance {=f32} Pa and {=f32} degC",
            label,
            self.refs.p,
            self.refs.t,
            self.refs.rev,
            P_TOL,
            T_TOL
        );

        let mut t = Tally::default();
        let (mut wi, mut wd, mut si, mut sd) = (0u32, 0u32, 0u64, 0u64);
        for _ in 0..300 {
            let a = Instant::now();
            let r = self.rd_id().await;
            let ui = a.elapsed().as_micros() as u32;
            self.tally(&mut t, &r);
            let a = Instant::now();
            let r = self.rd_data().await;
            let ud = a.elapsed().as_micros() as u32;
            self.tally(&mut t, &r);
            wi = wi.max(ui);
            wd = wd.max(ud);
            si += ui as u64;
            sd += ud as u64;
        }
        defmt::info!(
            "[STAT] {=str}: uncancelled 2-byte read mean {=u64} us worst {=u32} us; 6-byte read mean {=u64} us worst {=u32} us",
            label,
            si / 300,
            wi,
            sd / 300,
            wd
        );
        check!(
            self.fails,
            t.wrong == 0 && t.bus == 0,
            "[{=str}] {=str}: 600 uncancelled reads all right (wrong {=u32}, bus errors {=u32})",
            label,
            t.wrong,
            t.bus
        );

        let stale = [
            reg::CHIP_ID_VALUE,
            self.refs.rev,
            first[0],
            first[1],
            first[2],
            first[3],
        ];
        check!(
            self.fails,
            self.refs.data_ok(&first)
                && !self.refs.data_ok(&stale)
                && !self.refs.id_ok(&[reg::CHIP_ID_VALUE, self.refs.rev ^ 1])
                && !self.refs.id_ok(&[first[0], first[1]]),
            "[{=str}] {=str}: control, the judge accepts a real data read and rejects stale bytes ({=[u8]:#04x}) and a wrong identity",
            label,
            stale
        );

        let base = self.dump().await;
        check!(
            self.fails,
            base.is_ok(),
            "[{=str}] {=str}: baseline register dump taken",
            label
        );
        let base = base.ok()?;
        if self.fails != f0 {
            return None;
        }
        Some(Setup {
            base,
            span_d: wd * 115 / 100 + 5,
            span_i: wi * 115 / 100 + 5,
        })
    }

    /// C1: cancels with NO recovery, then six checking reads back to back. Returns false when the judge
    /// saw no wrong read.
    async fn c1(&mut self, s: &Setup, freq: u32) -> bool {
        let (mut n, mut with_wrong, mut worst, mut completed, mut unhealed) =
            ([0u32; 2], [0u32; 2], 0u32, 0u32, 0u32);
        let mut t = Tally::default();
        let mut first_shown = false;
        for i in 0..C1_N {
            let k = (i % 2) as usize;
            let span = if k == 0 { s.span_d } else { s.span_i } as u64;
            let d = span * (i / 2) as u64 / (C1_N / 2 - 1) as u64;
            let dropped = if k == 0 {
                cancel(self.baro.read(), d).is_none()
            } else {
                let mut b = [0u8; 2];
                cancel(self.baro.read_registers(reg::CHIP_ID, &mut b), d).is_none()
            };
            let mut w = 0u32;
            for j in 0..6u32 {
                let r = if j % 2 == 0 {
                    self.rd_id().await
                } else {
                    self.rd_data().await
                };
                t.reads += 1;
                if !self.tally(&mut t, &r) {
                    w += 1;
                    if dropped && !first_shown {
                        first_shown = true;
                        show(
                            &mut true,
                            if k == 0 { "data read" } else { "id read" },
                            d,
                            "no recovery",
                            j,
                            &r,
                        );
                    }
                }
            }
            if dropped {
                n[k] += 1;
                with_wrong[k] += (w > 0) as u32;
                worst = worst.max(w);
            } else {
                completed += 1;
            }
            // Restore without the driver's recover(): 3 ms with no bus traffic, one discarded read,
            // then rounds until identity and data are right.
            Timer::after_millis(3).await;
            let _ = self.rd_id().await;
            let mut ok = false;
            for _ in 0..4 {
                if matches!(self.rd_id().await, Rd::Ok) && matches!(self.rd_data().await, Rd::Ok) {
                    ok = true;
                    break;
                }
            }
            if !ok {
                unhealed += 1;
                Timer::after_millis(10).await;
            }
        }
        let wrong_cancels = with_wrong[0] + with_wrong[1];
        defmt::info!(
            "[STAT] C1 {=u32} kHz, no recovery: cancelled data reads {=u32} (followed by a wrong read: {=u32}), cancelled id reads {=u32} (followed by a wrong read: {=u32}), completed first {=u32}, most wrong reads after one cancel {=u32} of 6, wrong reads {=u32} of {=u32}, bus errors {=u32}, not restored by 3 ms quiet {=u32}",
            freq,
            n[0],
            with_wrong[0],
            n[1],
            with_wrong[1],
            completed,
            worst,
            t.wrong,
            t.reads,
            t.bus,
            unhealed
        );
        check!(
            self.fails,
            wrong_cancels >= 1,
            "[{=str}] C1 positive control at {=u32} kHz: {=u32} of {=u32} cancels without recovery were followed by a wrong read",
            freq,
            wrong_cancels,
            n[0] + n[1]
        );
        wrong_cancels >= 1
    }

    /// One sweep: blocks of PER_BLOCK cancels, kinds alternating by block, `recover()` after each cancel,
    /// the checking round, and the register dump after each block. Ends at the first block with a failure.
    async fn sweep(&mut self, s: &Setup, freq: u32, blocks: u32, min_dropped: u32) {
        let mut t = Tally::default();
        let mut first = true;
        let errs0 = self.baro.stats().bus_errors;
        'b: for b in 0..blocks {
            let id = b % 2 == 1;
            let span = if id { s.span_i } else { s.span_d } as u64;
            let kind = if id { "id read" } else { "data read" };
            let mut bad = false;
            for j in 0..PER_BLOCK {
                if self.start.elapsed() > RUN_LIMIT {
                    hang("run time limit", self.fails);
                }
                let d = span * (j * NPHASE + (b / 2) % NPHASE) as u64
                    / ((PER_BLOCK - 1) * NPHASE + NPHASE - 1) as u64;
                let res = if id {
                    let mut buf = [0u8; 2];
                    cancel(self.baro.read_registers(reg::CHIP_ID, &mut buf), d)
                        .map(|r| r.map(|_| ()).is_ok())
                } else {
                    cancel(self.baro.read(), d).map(|r| r.is_ok())
                };
                match res {
                    None if id => t.dropped_i += 1,
                    None => t.dropped_d += 1,
                    Some(ok) => {
                        t.completed += 1;
                        if !ok {
                            t.bus += 1;
                            bad = true;
                        }
                    }
                }
                let r0 = Instant::now();
                let r = with_timeout(REC_TMO, self.baro.recover(&mut Delay)).await;
                let us = r0.elapsed().as_micros() as u32;
                t.rec_n += 1;
                t.rec_sum_us += us as u64;
                t.rec_max_us = t.rec_max_us.max(us);
                match r {
                    Err(_) => hang("recover()", self.fails),
                    Ok(Err(e)) => {
                        t.rec_fail += 1;
                        bad = true;
                        if first {
                            first = false;
                            defmt::error!(
                                "[INFO] first failing case: cancelled {=str} at d={=u64} us, recover() returned {}",
                                kind,
                                d,
                                defmt::Debug2Format(&e)
                            );
                        }
                    }
                    Ok(Ok(())) => {}
                }
                let w0 = t.wrong + t.bus;
                self.check4(&mut t, &mut first, kind, d).await;
                bad |= t.wrong + t.bus != w0;
            }
            t.blocks += 1;
            match self.dump().await {
                Err(r) => {
                    t.dump_fail += 1;
                    defmt::error!("[INFO] register dump read of {=u8:#04x} failed", r);
                    break 'b;
                }
                Ok(now) => {
                    if let Some((r, old, new)) = first_diff(&s.base, &now) {
                        t.dump_changed += 1;
                        defmt::error!(
                            "[INFO] first register change: block {=u32} ({=str}), reg {=u8:#04x} was {=u8:#04x} now {=u8:#04x}",
                            b,
                            kind,
                            r,
                            old,
                            new
                        );
                        break 'b;
                    }
                    t.dump_equal += 1;
                }
            }
            if bad {
                break 'b;
            }
        }
        t.bus_errors_driver = self.baro.stats().bus_errors - errs0;
        let dropped = t.dropped_d + t.dropped_i;
        defmt::info!(
            "[STAT] sweep {=u32} kHz: cancelled {=u32} (data {=u32}, id {=u32}), completed before the drop {=u32}; recover() mean {=u64} us worst {=u32} us over {=u32} calls",
            freq,
            dropped,
            t.dropped_d,
            t.dropped_i,
            t.completed,
            t.rec_sum_us / t.rec_n.max(1) as u64,
            t.rec_max_us,
            t.rec_n
        );
        let ok = t.rec_fail == 0
            && t.wrong == 0
            && t.bus == 0
            && t.bus_errors_driver == 0
            && t.dump_changed == 0
            && t.dump_fail == 0
            && t.dump_equal == blocks
            && dropped >= min_dropped;
        check!(
            self.fails,
            ok,
            "[{=str}] sweep {=u32} kHz: {=u32} cancels, recover() failed {=u32}, wrong checking reads {=u32} of {=u32}, bus errors {=u32} (driver count {=u32}), register dumps equal to the baseline {=u32} of {=u32} (changed {=u32}), minimum cancels {=u32}",
            freq,
            dropped,
            t.rec_fail,
            t.wrong,
            t.reads,
            t.bus,
            t.bus_errors_driver,
            t.dump_equal,
            blocks,
            t.dump_changed,
            min_dropped
        );
    }
}

/// C4. Returns the failures it added. Everything here is bounded.
async fn c4(hw: &mut Hw, cfg: &Config, refs: Refs, start: Instant) -> u32 {
    let mut fails = 0u32;
    const F: u32 = 400_000;
    let mut baro = Bmp581::new(hw.bus(F), ADDR);
    let ok = matches!(
        with_timeout(Duration::from_millis(500), baro.init(&mut Delay, cfg)).await,
        Ok(Ok(_))
    );
    if !ok {
        check!(fails, false, "[{=str}] C4: init before the attempts");
        return fails;
    }
    Timer::after_millis(50).await;
    let mut stuck = None;
    let mut tries = 0u32;
    for a in 0..C4_TRIES {
        if start.elapsed() > RUN_LIMIT {
            hang("run time limit", fails);
        }
        tries += 1;
        let d = 60 + a as u64 * 4;
        let mut buf = [0u8; 6];
        let _ = cancel(baro.read_registers(reg::TEMP_DATA_XLSB, &mut buf), d);
        // Rebuild the I2C object at once, with the frame still running.
        drop(baro.release());
        baro = Bmp581::new(hw.bus(F), ADDR);
        spin(100);
        let (sda, scl) = lines();
        if !sda || !scl {
            Timer::after_millis(2).await;
            let (sda2, scl2) = lines();
            if !sda2 || !scl2 {
                stuck = Some((a, d, sda2, scl2, "a line held low"));
                break;
            }
        }
        Timer::after_millis(3).await;
        let mut good = 0u32;
        for _ in 0..3 {
            let mut b = [0u8; 2];
            match with_timeout(RD_TMO, baro.read_registers(reg::CHIP_ID, &mut b)).await {
                Ok(Ok(())) if refs.id_ok(&b) => good += 1,
                _ => good = 0,
            }
        }
        if good < 2 {
            let (sda2, scl2) = lines();
            stuck = Some((a, d, sda2, scl2, "reads fail with both lines high"));
            break;
        }
    }
    let Some((a, d, sda, scl, how)) = stuck else {
        defmt::info!(
            "[SKIP] C4: {=u32} rebuilds of the I2C object right after a cancelled read (d=60..{=u32} us, 400 kHz) left no line low and every read answered; the stuck line could not be provoked on purpose",
            tries,
            60 + (C4_TRIES - 1) * 4
        );
        return fails;
    };
    defmt::info!(
        "[STAT] C4: provoked at attempt {=u32} (d={=u64} us): SDA high {=bool}, SCL high {=bool} ({=str})",
        a,
        d,
        sda,
        scl,
        how
    );
    let mut failed = 0u32;
    for _ in 0..3 {
        let mut b = [0u8; 2];
        match with_timeout(RD_TMO, baro.read_registers(reg::CHIP_ID, &mut b)).await {
            Ok(Ok(())) if refs.id_ok(&b) => {}
            _ => failed += 1,
        }
    }
    check!(
        fails,
        failed >= 1,
        "[{=str}] C4: with the line stuck, {=u32} of 3 identity reads failed or hung (20 ms bound each)",
        failed
    );
    drop(baro.release());
    let (sl, cl, pulses) = bus_clear(hw);
    defmt::info!(
        "[STAT] C4: bus clear: SDA low at start {=bool}, SCL low at start {=bool}, {=u8} SCL pulses",
        sl,
        cl,
        pulses
    );
    let mut baro = Bmp581::new(hw.bus(F), ADDR);
    Timer::after_millis(3).await;
    let (sda, scl) = lines();
    let mut ids = 0u32;
    for _ in 0..3 {
        let mut b = [0u8; 2];
        if let Ok(Ok(())) = with_timeout(RD_TMO, baro.read_registers(reg::CHIP_ID, &mut b)).await
            && refs.id_ok(&b)
        {
            ids += 1;
        }
    }
    let cfgd = matches!(
        with_timeout(Duration::from_millis(500), baro.configure(&mut Delay, cfg)).await,
        Ok(Ok(_))
    );
    let rec = with_timeout(REC_TMO, baro.recover(&mut Delay)).await;
    let rec_ok = matches!(rec, Ok(Ok(())));
    Timer::after_millis(20).await;
    let (mut good, mut bad) = (0u32, 0u32);
    for _ in 0..200 {
        let mut b = [0u8; 6];
        match with_timeout(RD_TMO, baro.read_registers(reg::TEMP_DATA_XLSB, &mut b)).await {
            Ok(Ok(())) if refs.data_ok(&b) => good += 1,
            _ => bad += 1,
        }
        Timer::after_micros(500).await;
    }
    check!(
        fails,
        sda && scl && ids == 3 && cfgd && rec_ok && good == 200 && bad == 0,
        "[{=str}] C4: after the bus clear lines high (SDA {=bool}, SCL {=bool}), identity reads right {=u32} of 3, configure ok {=bool}, recover() ok {=bool}, good data reads {=u32} of 200 (bad {=u32})",
        sda,
        scl,
        ids,
        cfgd,
        rec_ok,
        good,
        bad
    );
    fails
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
    let default = Config::default();
    let test = Config::new()
        .with_pressure_oversampling(Oversampling::X1)
        .with_temperature_oversampling(Oversampling::X1)
        .with_odr(OutputDataRate::Hz240);
    let mut fails = 0u32;
    let mut bus_errors = 0u32;
    let mut refs = Refs {
        p: 0.0,
        t: 0.0,
        rev: 0,
    };
    let base0: Option<[u8; ND]>;
    let mut c1_seen = false;
    let mut setup0: Option<Setup> = None;

    // C0, C1, C2 at 400 kHz.
    {
        let mut c = Ctx::new(hw.bus(400_000), start);
        // The default configuration first: its register dump is the one C5 must meet again.
        let ok = c.init(&default).await.is_ok();
        Timer::after_millis(100).await;
        let v = with_timeout(RD_TMO, c.baro.verify()).await;
        let d0 = c.dump().await;
        check!(
            c.fails,
            ok && matches!(v, Ok(Ok(()))) && d0.is_ok(),
            "[{=str}] C0 default configuration: init, verify() and register dump {=[u8]:#04x}",
            d0.unwrap_or([0; ND])
        );
        base0 = d0.ok();
        if c.fails == 0 {
            setup0 = c.setup(&test, "C0 400 kHz").await;
        }
        if let Some(s) = &setup0 {
            refs = c.refs;
            c1_seen = c.c1(s, 400).await;
            if c1_seen {
                c.sweep(s, 400, C2_BLOCKS, 500).await;
            } else {
                defmt::info!(
                    "[SKIP] C2: the positive control saw no wrong read, so a clean sweep would prove nothing"
                );
            }
        }
        fails += c.fails;
        bus_errors += c.baro.stats().bus_errors;
    }
    if fails != 0 || setup0.is_none() {
        hil_common::exit(fails + setup0.is_none() as u32);
    }

    // C3.
    for freq in [100_000u32, 1_000_000] {
        let mut c = Ctx::new(hw.bus(freq), start);
        let label = if freq == 100_000 {
            "C3 100 kHz"
        } else {
            "C3 1000 kHz"
        };
        if let Some(s) = c.setup(&test, label).await {
            let same = first_diff(&s.base, &setup0.as_ref().map_or([0; ND], |x| x.base)).is_none();
            check!(
                c.fails,
                same,
                "[{=str}] {=str}: baseline register dump equal to the 400 kHz baseline",
                label
            );
            c.sweep(&s, freq / 1000, C3_BLOCKS, 200).await;
        }
        fails += c.fails;
        bus_errors += c.baro.stats().bus_errors;
        if fails != 0 {
            hil_common::exit(fails);
        }
    }

    // C4.
    fails += c4(&mut hw, &test, refs, start).await;
    if fails != 0 {
        hil_common::exit(fails);
    }

    // C5.
    {
        let mut c = Ctx::new(hw.bus(400_000), start);
        let ok = c.init(&default).await.is_ok();
        Timer::after_millis(100).await;
        let v = with_timeout(RD_TMO, c.baro.verify()).await;
        let now = c.dump().await;
        let same = matches!((&now, &base0), (Ok(n), Some(b)) if first_diff(b, n).is_none());
        check!(
            c.fails,
            ok && matches!(v, Ok(Ok(()))) && same,
            "[{=str}] C5 end: default configuration init ok {=bool}, verify() ok, register dump equal to the default one {=bool}",
            ok,
            same
        );
        let ints = with_timeout(RD_TMO, c.baro.interrupt_status()).await;
        let st = c.baro.stats();
        bus_errors += st.bus_errors;
        defmt::info!(
            "[INFO] health: driver bus errors over the run {=u32}, power-on resets seen {=u32}, power-on flag in INT_STATUS now {=bool} (read ok {=bool}), run took {=u64} ms, C1 saw the fault {=bool}",
            bus_errors,
            st.power_on_resets,
            matches!(&ints, Ok(Ok(s)) if s.power_on_reset),
            matches!(&ints, Ok(Ok(_))),
            start.elapsed().as_millis(),
            c1_seen
        );
        fails += c.fails;
    }
    hil_common::exit(fails)
}
