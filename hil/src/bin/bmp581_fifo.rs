//! On-board check of the bmp581 FIFO: stream, overflow, flush, threshold and full interrupts, frame
//! kinds, decimation and the three datasheet points. Async I2C1 at 400 kHz, INT on GPIO21 counted by a
//! PWM slice (hardware edge count) while the GPIO interrupt wakes the task.
#![no_std]
#![no_main]

use core::future::Future;
use core::pin::pin;
use core::task::Poll;

use bmp581::{
    Bmp581, Config, Error, FifoConfig, FifoData, FifoDecimation, FifoFrame, FifoMode, InterruptPin,
    InterruptSources, OutputDataRate, Oversampling, PowerMode, RawSample, fifo_frames,
};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C1;
use embassy_rp::pwm::{self, ChannelBPin, InputMode, Pwm, Slice};
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

const EDGE_TIMEOUT: Duration = Duration::from_millis(250);
/// Largest step between consecutive frames, pressure. bmp581_smoke measured a whole-run span of 4.4 Pa
/// (std 0.67 Pa) at the 240 Hz x1/x1 setting; a step above 10 Pa is more than twice that whole span,
/// so a frame decoded from shifted or foreign bytes (steps of kPa) cannot hide under it.
const STEP_P_PA: f32 = 10.0;
/// Largest step between consecutive frames, temperature: smoke's span at this setting was 0.04 degC.
const STEP_T_C: f32 = 0.1;
/// Plausibility bounds, the ones bmp581_smoke uses.
const P_MIN: f32 = 60_000.0;
const P_MAX: f32 = 110_000.0;
const T_MIN: f32 = 10.0;
const T_MAX: f32 = 50.0;

type Baro<'d> = Bmp581<I2c<'d, I2C1, i2c::Async>>;

struct DataReady<'d> {
    input: Input<'d>,
    pwm: Pwm<'d>,
    last: u16,
    wraps: u64,
}

impl<'d> DataReady<'d> {
    fn new<T: Slice, P: ChannelBPin<T> + Pin>(slice: Peri<'d, T>, pin: Peri<'d, P>) -> Self {
        // SAFETY: the Input only arms and awaits GPIO edge interrupts, the PWM slice only counts.
        let input = Input::new(unsafe { pin.clone_unchecked() }, Pull::None);
        let pwm = Pwm::new_input(
            slice,
            pin,
            Pull::None,
            InputMode::RisingEdge,
            pwm::Config::default(),
        );
        let last = pwm.counter();
        Self {
            input,
            pwm,
            last,
            wraps: 0,
        }
    }

    fn count(&mut self) -> u64 {
        let c = self.pwm.counter();
        if c < self.last {
            self.wraps += 1;
        }
        self.last = c;
        (self.wraps << 16) | c as u64
    }

    /// Waits until the hardware edge count exceeds `seen`; returns the count, `None` on timeout.
    /// The timeout wraps the pin wait only, never a bus call.
    async fn wait_beyond(&mut self, seen: u64, timeout: Duration) -> Option<u64> {
        let wait = async {
            loop {
                let mut edge = pin!(self.input.wait_for_rising_edge());
                let r = core::future::poll_fn(|cx| {
                    let fired = edge.as_mut().poll(cx).is_ready();
                    let c = {
                        let c = self.pwm.counter();
                        if c < self.last {
                            self.wraps += 1;
                        }
                        self.last = c;
                        (self.wraps << 16) | c as u64
                    };
                    if c > seen {
                        Poll::Ready(Some(c))
                    } else if fired {
                        Poll::Ready(None)
                    } else {
                        Poll::Pending
                    }
                })
                .await;
                if let Some(c) = r {
                    return c;
                }
            }
        };
        with_timeout(timeout, wait).await.ok()
    }
}

fn absf(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

/// What the decoded frames of a run looked like.
struct Acc {
    frames: u32,
    empties: u32,
    partials: u32,
    wrong_kind: u32,
    invalid: u32,
    step_viol: u32,
    p_min: f32,
    p_max: f32,
    t_min: f32,
    t_max: f32,
    max_dp: f32,
    max_dt: f32,
    prev_p: Option<f32>,
    prev_t: Option<f32>,
    p_sum: f64,
    t_sum: f64,
    p_n: u32,
    t_n: u32,
}

impl Acc {
    fn new() -> Self {
        Self {
            frames: 0,
            empties: 0,
            partials: 0,
            wrong_kind: 0,
            invalid: 0,
            step_viol: 0,
            p_min: f32::MAX,
            p_max: f32::MIN,
            t_min: f32::MAX,
            t_max: f32::MIN,
            max_dp: 0.0,
            max_dt: 0.0,
            prev_p: None,
            prev_t: None,
            p_sum: 0.0,
            t_sum: 0.0,
            p_n: 0,
            t_n: 0,
        }
    }

    fn pressure(&mut self, p: f32) {
        if !(P_MIN..=P_MAX).contains(&p) {
            self.invalid += 1;
        }
        if let Some(prev) = self.prev_p {
            let d = absf(p - prev);
            self.max_dp = self.max_dp.max(d);
            if d > STEP_P_PA {
                self.step_viol += 1;
            }
        }
        self.prev_p = Some(p);
        self.p_min = self.p_min.min(p);
        self.p_max = self.p_max.max(p);
        self.p_sum += p as f64;
        self.p_n += 1;
    }

    fn temperature(&mut self, t: f32) {
        if !(T_MIN..=T_MAX).contains(&t) {
            self.invalid += 1;
        }
        if let Some(prev) = self.prev_t {
            let d = absf(t - prev);
            self.max_dt = self.max_dt.max(d);
            if d > STEP_T_C {
                self.step_viol += 1;
            }
        }
        self.prev_t = Some(t);
        self.t_min = self.t_min.min(t);
        self.t_max = self.t_max.max(t);
        self.t_sum += t as f64;
        self.t_n += 1;
    }

    fn feed(&mut self, buf: &[u8], data: FifoData) {
        for f in fifo_frames(buf, data) {
            match f {
                FifoFrame::PressureTemperature(s) if data == FifoData::PressureAndTemperature => {
                    self.frames += 1;
                    if s.is_reset_value() {
                        self.invalid += 1;
                    }
                    self.pressure(s.pascals());
                    self.temperature(s.celsius());
                }
                FifoFrame::Pressure(s) if data == FifoData::Pressure => {
                    self.frames += 1;
                    self.pressure(s.pascals());
                }
                FifoFrame::Temperature(s) if data == FifoData::Temperature => {
                    self.frames += 1;
                    self.temperature(s.celsius());
                }
                FifoFrame::Empty(_) => self.empties += 1,
                FifoFrame::Partial(_) => self.partials += 1,
                _ => self.wrong_kind += 1,
            }
        }
    }

    fn p_mean(&self) -> f32 {
        (self.p_sum / self.p_n.max(1) as f64) as f32
    }

    fn t_mean(&self) -> f32 {
        (self.t_sum / self.t_n.max(1) as f64) as f32
    }
}

#[derive(Default)]
struct Io {
    bus_errors: u32,
    full_flags: u32,
    size_mismatch: u32,
    calls: u32,
}

/// Reads until a call finds the FIFO empty (bounded); returns the frames read.
async fn drain(baro: &mut Baro<'_>, data: FifoData, mut acc: Option<&mut Acc>, io: &mut Io) -> u32 {
    let mut buf = [0u8; 15];
    let n = if data == FifoData::PressureAndTemperature {
        12
    } else {
        15
    };
    let mut total = 0u32;
    for _ in 0..64 {
        match baro.fifo_read(&mut buf[..n]).await {
            Ok(r) => {
                io.calls += 1;
                if r.full {
                    io.full_flags += 1;
                }
                if r.bytes != usize::from(r.frames) * data.frame_bytes() {
                    io.size_mismatch += 1;
                }
                if r.frames == 0 {
                    break;
                }
                total += u32::from(r.frames);
                if let Some(a) = acc.as_deref_mut() {
                    a.feed(&buf[..r.bytes], data);
                }
            }
            Err(_) => {
                io.bus_errors += 1;
                break;
            }
        }
    }
    total
}

struct Stream {
    frames: u32,
    edges: u64,
    us: u64,
    timeouts: u32,
    acc: Acc,
    io: Io,
}

/// Reads the FIFO after every data-ready edge for `window`; the edge count of the INT pin is the
/// reference. The backlog present at the start is read first and not counted.
async fn stream(
    baro: &mut Baro<'_>,
    dr: &mut DataReady<'_>,
    data: FifoData,
    window: Duration,
) -> Stream {
    let mut s = Stream {
        frames: 0,
        edges: 0,
        us: 0,
        timeouts: 0,
        acc: Acc::new(),
        io: Io::default(),
    };
    drain(baro, data, None, &mut s.io).await;
    let c0 = dr.count();
    let mut seen = c0;
    let t0 = Instant::now();
    while t0.elapsed() < window {
        let Some(c) = dr.wait_beyond(seen, EDGE_TIMEOUT).await else {
            s.timeouts += 1;
            break;
        };
        seen = c;
        s.frames += drain(baro, data, Some(&mut s.acc), &mut s.io).await;
    }
    let c1 = dr.count();
    s.us = t0.elapsed().as_micros().max(1);
    s.edges = c1 - c0;
    s
}

fn check_stream(fails: &mut u32, name: &str, s: &Stream, decim: u32, data: FifoData) {
    let secs = s.us as f32 / 1.0e6;
    let fps = s.frames as f32 / secs;
    let exp = s.edges as f32 / decim as f32 / secs;
    defmt::info!(
        "[STAT] {=str}: {=u32} frames in {=u32} ms, {=f32} frames/s, edge rate / {=u32} = {=f32} /s, {=u64} edges, {=u32} fifo_read calls, bus errors {=u32}, timeouts {=u32}",
        name,
        s.frames,
        (s.us / 1000) as u32,
        fps,
        decim,
        exp,
        s.edges,
        s.io.calls,
        s.io.bus_errors,
        s.timeouts
    );
    let a = &s.acc;
    defmt::info!(
        "[STAT] {=str}: pressure mean {=f32} Pa ({=f32}..{=f32}), temperature mean {=f32} degC ({=f32}..{=f32}), max step {=f32} Pa {=f32} degC",
        name,
        a.p_mean(),
        a.p_min,
        a.p_max,
        a.t_mean(),
        a.t_min,
        a.t_max,
        a.max_dp,
        a.max_dt
    );
    let rate_ok = absf(fps - exp) <= 0.02 * exp && s.frames > 10;
    check!(
        *fails,
        rate_ok,
        "[{=str}] {=str}: {=f32} frames/s vs {=f32} /s expected from the INT pin, within 2 %",
        name,
        fps,
        exp
    );
    check!(
        *fails,
        a.wrong_kind == 0 && a.partials == 0 && a.invalid == 0 && s.io.size_mismatch == 0,
        "[{=str}] {=str}: every frame decodes ({=u32} wrong kind, {=u32} partial, {=u32} invalid, {=u32} size mismatches)",
        name,
        a.wrong_kind,
        a.partials,
        a.invalid,
        s.io.size_mismatch
    );
    check!(
        *fails,
        a.empties == 0,
        "[{=str}] {=str}: {=u32} empty frames in the stream, expected 0",
        name,
        a.empties
    );
    let plausible = match data {
        FifoData::PressureAndTemperature => {
            a.p_n > 0
                && (P_MIN..=P_MAX).contains(&a.p_min)
                && a.p_max <= P_MAX
                && a.t_min >= T_MIN
                && a.t_max <= T_MAX
        }
        FifoData::Pressure => a.p_n > 0 && a.p_min >= P_MIN && a.p_max <= P_MAX,
        FifoData::Temperature => a.t_n > 0 && a.t_min >= T_MIN && a.t_max <= T_MAX,
    };
    check!(
        *fails,
        plausible,
        "[{=str}] {=str}: plausible, bounds of bmp581_smoke (pressure 60000..110000 Pa, temperature 10..50 degC)",
        name
    );
    check!(
        *fails,
        a.step_viol == 0,
        "[{=str}] {=str}: {=u32} steps above {=f32} Pa / {=f32} degC between consecutive frames, expected 0",
        name,
        a.step_viol,
        STEP_P_PA,
        STEP_T_C
    );
    check!(
        *fails,
        s.io.bus_errors == 0 && s.timeouts == 0,
        "[{=str}] {=str}: {=u32} bus errors, {=u32} timeouts, expected 0",
        name,
        s.io.bus_errors,
        s.timeouts
    );
}

/// Polls `FIFO_COUNT` every 5 ms for at most `bound_ms` until it is at least `at_least`.
async fn wait_count(baro: &mut Baro<'_>, at_least: u8, bound_ms: u32) -> Option<u8> {
    for _ in 0..bound_ms / 5 {
        if let Ok(c) = baro.fifo_count().await
            && c >= at_least
        {
            return Some(c);
        }
        Timer::after_millis(5).await;
    }
    None
}

/// Reads `FIFO_COUNT` `n` times, 5 ms apart, without reading frames: lowest and highest value seen.
async fn count_range(baro: &mut Baro<'_>, n: u32) -> (u8, u8) {
    let (mut lo, mut hi) = (255u8, 0u8);
    for _ in 0..n {
        if let Ok(c) = baro.fifo_count().await {
            lo = lo.min(c);
            hi = hi.max(c);
        }
        Timer::after_millis(5).await;
    }
    (lo, hi)
}

fn fifo(mode: FifoMode, data: FifoData, threshold: u8, dec: FifoDecimation) -> FifoConfig {
    FifoConfig::new()
        .with_mode(mode)
        .with_data(data)
        .with_threshold(threshold)
        .with_decimation(dec)
}

/// 240 Hz, 1x/1x: the highest rate `Config::validate` allows in normal mode.
fn fast(f: FifoConfig) -> Config {
    Config::new()
        .with_pressure_oversampling(Oversampling::X1)
        .with_temperature_oversampling(Oversampling::X1)
        .with_odr(OutputDataRate::Hz240)
        .with_fifo(f)
}

async fn setup(fails: &mut u32, baro: &mut Baro<'_>, name: &str, cfg: &Config) -> bool {
    match baro.init(&mut Delay, cfg).await {
        Ok(_) => true,
        Err(e) => {
            *fails += 1;
            defmt::error!(
                "[FAIL] {=str}: init failed: {}",
                name,
                defmt::Debug2Format(&e)
            );
            false
        }
    }
}

/// A 6-byte FIFO frame as a plausible measurement, or `None`.
fn valid_pt(b: &[u8]) -> Option<(f32, f32)> {
    let arr = <[u8; 6]>::try_from(b).ok()?;
    if arr.iter().all(|&x| x == 0x7F) {
        return None;
    }
    let s = RawSample::decode(&arr);
    let (p, t) = (s.pascals(), s.celsius());
    ((P_MIN..=P_MAX).contains(&p) && (T_MIN..=T_MAX).contains(&t) && !s.is_reset_value())
        .then_some((p, t))
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut fails = 0u32;

    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    cfg.sda_pullup = false;
    cfg.scl_pullup = false;
    let bus = I2c::new_async(
        p.I2C1.reborrow(),
        p.PIN_23.reborrow(),
        p.PIN_22.reborrow(),
        Irqs,
        cfg,
    );
    let mut baro = Bmp581::new(bus, bmp581::DEFAULT_ADDRESS);
    let mut delay = Delay;
    let mut dr = DataReady::new(p.PWM_SLICE2.reborrow(), p.PIN_21.reborrow());
    let pt = FifoData::PressureAndTemperature;
    let none = FifoDecimation::By1;
    let mut flush_ok = false;

    // F1. Stream at the highest rate: normal mode 240 Hz, 1x/1x. Data-ready stays on the INT pin in the
    // same configuration, so the reference is the pin's edge count over the same window.
    defmt::info!("[INFO] F1 stream: normal 240 Hz x1/x1, P+T, data-ready on INT, 5 s");
    let f1 = fast(fifo(FifoMode::Stream, pt, 0, none));
    if setup(&mut fails, &mut baro, "F1", &f1).await {
        let before = baro.stats();
        let s = stream(&mut baro, &mut dr, pt, Duration::from_millis(5000)).await;
        check_stream(&mut fails, "F1", &s, 1, pt);
        let after = baro.stats();
        check!(
            fails,
            s.io.full_flags == 0 && after.fifo_full == before.fifo_full,
            "[{=str}] F1: FIFO never full while reading ({=u32} full flags, stats.fifo_full {=u32} -> {=u32})",
            s.io.full_flags,
            before.fifo_full,
            after.fifo_full
        );
        check!(
            fails,
            after.bus_errors == before.bus_errors
                && after.fifo_frames - before.fifo_frames >= s.frames,
            "[{=str}] F1: stats.bus_errors {=u32} -> {=u32}, stats.fifo_frames +{=u32} for {=u32} counted frames",
            before.bus_errors,
            after.bus_errors,
            after.fifo_frames - before.fifo_frames,
            s.frames
        );
        check!(
            fails,
            baro.verify().await.is_ok(),
            "[{=str}] F1: verify Ok after the run"
        );
    }

    // F1b. The same in continuous mode (driver does not restrict it with the FIFO): beyond the 240 Hz
    // ceiling of normal mode, about 496 Hz per bmp581_smoke.
    defmt::info!("[INFO] F1b stream: continuous x1/x1, P+T, 3 s");
    let f1b = fast(fifo(FifoMode::Stream, pt, 0, none)).with_power_mode(PowerMode::Continuous);
    if setup(&mut fails, &mut baro, "F1b", &f1b).await {
        let s = stream(&mut baro, &mut dr, pt, Duration::from_millis(3000)).await;
        check_stream(&mut fails, "F1b", &s, 1, pt);
    }

    // F2. Overflow, stream mode: stop reading for 500 ms at 240 Hz (about 120 frames into 15).
    // The chip's streaming FIFO stops at the depth minus one, 15 P+T frames, with the FIFO-full flag set
    // (log bmp581_fifo-20261111-180444, F2 stream and F4 full: FIFO_COUNT 15 for 20 reads over 100 ms; the
    // datasheet says 16). fifo_read reports that as full (FifoRead::full), the exact flag being
    // INT_STATUS.fifo_full.
    defmt::info!("[INFO] F2 overflow, stream mode: no reads for 500 ms at 240 Hz");
    let f2 = fast(fifo(FifoMode::Stream, pt, 0, none)).with_interrupt(
        InterruptPin::new().with_sources(InterruptSources::new().with_fifo_full(true)),
    );
    if setup(&mut fails, &mut baro, "F2", &f2).await {
        let mut io = Io::default();
        let s0 = baro.stats();
        let c0 = dr.count();
        Timer::after_millis(500).await;
        let produced = dr.count() - c0;
        let count = baro.fifo_count().await.unwrap_or(255);
        let st = baro.interrupt_status().await;
        defmt::info!(
            "[STAT] F2 stream: {=u64} measurements in 500 ms, FIFO_COUNT {=u8} (measured streaming limit 15 P+T frames, datasheet depth 16), about {=u64} lost",
            produced,
            count,
            produced.saturating_sub(u64::from(count))
        );
        check!(
            fails,
            count == 15,
            "[{=str}] F2 stream: FIFO_COUNT {=u8}, expected the measured streaming limit 15 (datasheet depth 16)",
            count
        );
        check!(
            fails,
            st.as_ref().is_ok_and(|s| s.fifo_full),
            "[{=str}] F2 stream: INT_STATUS fifo_full reported ({=bool})",
            st.as_ref().is_ok_and(|s| s.fifo_full)
        );
        let (lo, hi) = count_range(&mut baro, 20).await;
        defmt::info!(
            "[STAT] F2 stream: 20 further FIFO_COUNT reads over 100 ms with the FIFO still unread: lowest {=u8}, highest {=u8}",
            lo,
            hi
        );
        let mut acc = Acc::new();
        // The first fifo_read of the unread FIFO must report it full: the driver's contract is that the
        // loss the chip reports is returned and counted (a full FIFO unreported fails this check).
        let mut first = [0u8; 12];
        let r1 = baro.fifo_read(&mut first).await;
        let s1 = baro.stats();
        let (r1_full, r1_frames, r1_bytes) = r1
            .as_ref()
            .map_or((false, 0, 0), |r| (r.full, u32::from(r.frames), r.bytes));
        check!(
            fails,
            r1_full && s1.fifo_full - s0.fifo_full == 2,
            "[{=str}] F2 stream: first fifo_read full {=bool}, stats.fifo_full +{=u32} (expected true, 2: one INT_STATUS read, one fifo_read)",
            r1_full,
            s1.fifo_full - s0.fifo_full
        );
        acc.feed(&first[..r1_bytes], pt);
        let n = r1_frames + drain(&mut baro, pt, Some(&mut acc), &mut io).await;
        check!(
            fails,
            (15..=17).contains(&n)
                && acc.frames == n
                && acc.empties == 0
                && acc.invalid == 0
                && acc.partials == 0
                && acc.step_viol == 0
                && io.bus_errors == 0,
            "[{=str}] F2 stream: backlog of {=u32} frames (expected 15, a frame or two more may arrive during the read) reads cleanly ({=u32} decoded, {=u32} empty, {=u32} invalid, {=u32} big steps, {=u32} bus errors)",
            n,
            acc.frames,
            acc.empties,
            acc.invalid,
            acc.step_viol,
            io.bus_errors
        );
        let s = stream(&mut baro, &mut dr, pt, Duration::from_millis(1500)).await;
        check_stream(&mut fails, "F2 resume", &s, 1, pt);
    }

    // F2, stop-on-full. P+T: the count reaches the datasheet depth 16 (log bmp581_fifo-20261111-180444).
    defmt::info!("[INFO] F2 overflow, stop on full: no reads for 500 ms, then 300 ms more");
    let f2s = fast(fifo(FifoMode::StopOnFull, pt, 0, none));
    if setup(&mut fails, &mut baro, "F2s", &f2s).await {
        let mut io = Io::default();
        Timer::after_millis(500).await;
        let c1 = baro.fifo_count().await.unwrap_or(255);
        Timer::after_millis(300).await;
        let c2 = baro.fifo_count().await.unwrap_or(255);
        defmt::info!(
            "[STAT] F2 stop-on-full: FIFO_COUNT {=u8} at 500 ms, {=u8} at 800 ms (datasheet depth 16 P+T frames)",
            c1,
            c2
        );
        check!(
            fails,
            c1 == 16 && c2 == 16,
            "[{=str}] F2 stop-on-full: count stops at the depth: {=u8} then {=u8}, expected 16 and 16",
            c1,
            c2
        );
        let mut acc = Acc::new();
        let n = drain(&mut baro, pt, Some(&mut acc), &mut io).await;
        check!(
            fails,
            io.full_flags >= 1,
            "[{=str}] F2 stop-on-full: fifo_read full flag seen {=u32}x, expected at least once",
            io.full_flags
        );
        check!(
            fails,
            n >= 16
                && acc.frames == n
                && acc.empties == 0
                && acc.invalid == 0
                && io.bus_errors == 0,
            "[{=str}] F2 stop-on-full: {=u32} frames read, {=u32} decoded, {=u32} empty, {=u32} invalid, {=u32} bus errors",
            n,
            acc.frames,
            acc.empties,
            acc.invalid,
            io.bus_errors
        );
        Timer::after_millis(50).await;
        let again = baro.fifo_count().await.unwrap_or(0);
        check!(
            fails,
            again >= 5,
            "[{=str}] F2 stop-on-full: writing resumes after the read, FIFO_COUNT {=u8} after 50 ms, expected at least 5",
            again
        );
    }

    // F2, stop-on-full, pressure-only frames.
    defmt::info!(
        "[INFO] F2 overflow, stop on full, pressure-only frames: no reads for 500 ms, then 300 ms more"
    );
    let f2p = fast(fifo(FifoMode::StopOnFull, FifoData::Pressure, 0, none));
    if setup(&mut fails, &mut baro, "F2p", &f2p).await {
        Timer::after_millis(500).await;
        let c1 = baro.fifo_count().await.unwrap_or(255);
        Timer::after_millis(300).await;
        let c2 = baro.fifo_count().await.unwrap_or(255);
        defmt::info!(
            "[STAT] F2 stop-on-full pressure-only: FIFO_COUNT {=u8} at 500 ms, {=u8} at 800 ms",
            c1,
            c2
        );
        // Measured in log bmp581_fifo-20261111-181709: the count stops at 32, the datasheet depth.
        check!(
            fails,
            c1 == 32 && c2 == 32,
            "[{=str}] F2 stop-on-full pressure-only: count stops at the depth: {=u8} then {=u8}, expected 32 and 32",
            c1,
            c2
        );
        let s0 = baro.stats();
        let mut buf = [0u8; 15];
        let r = baro.fifo_read(&mut buf).await;
        let full = r.as_ref().is_ok_and(|r| r.full);
        check!(
            fails,
            full && baro.stats().fifo_full - s0.fifo_full == 1,
            "[{=str}] F2 stop-on-full pressure-only: fifo_read full {=bool}, stats.fifo_full +1 expected",
            full
        );
    }

    // F3. Flush, at 50 Hz 16x/2x so that no frame can arrive within the 4 ms the flush ends with.
    defmt::info!("[INFO] F3 flush: 50 Hz default oversampling, stream then stop on full");
    for (label, mode, wait_ms) in [
        ("stream", FifoMode::Stream, 250u64),
        ("stop-on-full", FifoMode::StopOnFull, 600u64),
    ] {
        let f3 = Config::new().with_fifo(fifo(mode, pt, 0, none));
        if !setup(&mut fails, &mut baro, "F3", &f3).await {
            continue;
        }
        Timer::after_millis(wait_ms).await;
        let pre = baro.fifo_count().await.unwrap_or(0);
        let r = baro.fifo_flush(&mut delay).await;
        let post = baro.fifo_count().await.unwrap_or(255);
        let t0 = Instant::now();
        Timer::after_millis(150).await;
        let later = baro.fifo_count().await.unwrap_or(0);
        defmt::info!(
            "[STAT] F3 {=str}: count before flush {=u8}, right after {=u8}, 150 ms later {=u8} ({=u32} ms measured)",
            label,
            pre,
            post,
            later,
            t0.elapsed().as_millis() as u32
        );
        check!(
            fails,
            r.is_ok() && pre >= 5 && post == 0,
            "[{=str}] F3 {=str}: fifo_flush Ok, FIFO_COUNT {=u8} -> {=u8}, expected >= 5 -> 0",
            label,
            pre,
            post
        );
        if label == "stream" {
            flush_ok = r.is_ok() && post == 0;
        }
        check!(
            fails,
            later >= 3,
            "[{=str}] F3 {=str}: stream restarts, FIFO_COUNT {=u8} 150 ms after the flush, expected at least 3",
            label,
            later
        );
        check!(
            fails,
            baro.verify().await.is_ok(),
            "[{=str}] F3 {=str}: verify Ok after the flush",
            label
        );
        if label == "stream" {
            let s = stream(&mut baro, &mut dr, pt, Duration::from_millis(1000)).await;
            check_stream(&mut fails, "F3 restart", &s, 1, pt);
        }
    }

    // F4. Threshold interrupt: N = 8 frames at 50 Hz, only that source on the pin.
    const N: u8 = 8;
    defmt::info!("[INFO] F4 threshold interrupt: N = 8 frames, 50 Hz, only fifo_threshold on INT");
    let f4 = Config::new()
        .with_fifo(fifo(FifoMode::Stream, pt, N, none))
        .with_interrupt(
            InterruptPin::new().with_sources(InterruptSources::none().with_fifo_threshold(true)),
        );
    if setup(&mut fails, &mut baro, "F4", &f4).await {
        let mut io = Io::default();
        let c0 = dr.count();
        let t0 = Instant::now();
        let first = dr.wait_beyond(c0, Duration::from_millis(1500)).await;
        let t_first = t0.elapsed().as_millis() as u32;
        let count = baro.fifo_count().await.unwrap_or(0);
        let st = baro.interrupt_status().await;
        let thr_flag = st.as_ref().is_ok_and(|s| s.fifo_threshold);
        defmt::info!(
            "[STAT] F4 first edge after {=u32} ms (8 frames at 50 Hz about 160 ms plus the first conversion), FIFO_COUNT {=u8}",
            t_first,
            count
        );
        check!(
            fails,
            first.is_some_and(|c| c - c0 == 1) && count >= N && thr_flag,
            "[{=str}] F4 first edge: edges {=u64}, FIFO_COUNT {=u8} (expected >= 8), INT_STATUS fifo_threshold {=bool}",
            first.map_or(0, |c| c - c0),
            count,
            thr_flag
        );
        let seen = first.unwrap_or(c0);
        Timer::after_millis(600).await;
        let extra = dr.count() - seen;
        let c_end = baro.fifo_count().await.unwrap_or(0);
        defmt::info!(
            "[STAT] F4 no reads after the first edge, flag cleared: {=u64} more edges in 600 ms, FIFO_COUNT {=u8}",
            extra,
            c_end
        );
        drain(&mut baro, pt, None, &mut io).await;
        // Cycles: wait for the edge, check, drain below the threshold.
        const CYCLES: u32 = 10;
        let mut seen = dr.count();
        let base = seen;
        let (mut frames, mut low_count, mut no_flag, mut cycles) = (0u32, 0u32, 0u32, 0u32);
        let mut min_c = 255u8;
        let mut max_c = 0u8;
        let mut timeouts = 0u32;
        let mut edge_at_wake = 0u64;
        for _ in 0..CYCLES {
            let Some(c) = dr.wait_beyond(seen, Duration::from_millis(1000)).await else {
                timeouts += 1;
                break;
            };
            edge_at_wake = c - base;
            cycles += 1;
            let n = baro.fifo_count().await.unwrap_or(0);
            min_c = min_c.min(n);
            max_c = max_c.max(n);
            if n < N {
                low_count += 1;
            }
            if !baro
                .interrupt_status()
                .await
                .is_ok_and(|s| s.fifo_threshold)
            {
                no_flag += 1;
            }
            frames += drain(&mut baro, pt, None, &mut io).await;
            seen = dr.count();
        }
        defmt::info!(
            "[STAT] F4 cycles: {=u32} wakes, {=u64} edges, {=u32} frames read, FIFO_COUNT at the edge {=u8}..{=u8}",
            cycles,
            edge_at_wake,
            frames,
            min_c,
            max_c
        );
        check!(
            fails,
            timeouts == 0 && cycles == CYCLES && edge_at_wake == u64::from(CYCLES),
            "[{=str}] F4 cycles: {=u32} edges for {=u32} wakes, expected {=u32} each, {=u32} timeouts",
            edge_at_wake as u32,
            cycles,
            CYCLES,
            timeouts
        );
        check!(
            fails,
            low_count == 0 && no_flag == 0,
            "[{=str}] F4 cycles: {=u32} edges with FIFO_COUNT below {=u8}, {=u32} without INT_STATUS fifo_threshold",
            low_count,
            N,
            no_flag
        );
        check!(
            fails,
            frames >= cycles * u32::from(N)
                && frames <= cycles * (u32::from(N) + 2)
                && io.bus_errors == 0,
            "[{=str}] F4 cycles: {=u32} frames read for {=u32} edges of N = {=u8}, expected {=u32}..{=u32}, {=u32} bus errors",
            frames,
            cycles,
            N,
            cycles * u32::from(N),
            cycles * (u32::from(N) + 2),
            io.bus_errors
        );
    }

    // F4, FIFO-full interrupt on the pin. Streaming: the interrupt fires with the count at 15 (log
    // bmp581_fifo-20261111-180444, F4 full: edge after 293 ms, FIFO_COUNT 15, flag set).
    defmt::info!("[INFO] F4 full interrupt: only fifo_full on INT, stream, 50 Hz, no reads");
    let f4f = Config::new()
        .with_fifo(fifo(FifoMode::Stream, pt, 0, none))
        .with_interrupt(
            InterruptPin::new().with_sources(InterruptSources::none().with_fifo_full(true)),
        );
    if setup(&mut fails, &mut baro, "F4f", &f4f).await {
        let c0 = dr.count();
        let t0 = Instant::now();
        let first = dr.wait_beyond(c0, Duration::from_millis(2000)).await;
        let t_first = t0.elapsed().as_millis() as u32;
        let count = baro.fifo_count().await.unwrap_or(0);
        let st = baro.interrupt_status().await;
        let full_flag = st.as_ref().is_ok_and(|s| s.fifo_full);
        defmt::info!(
            "[STAT] F4 full: first edge after {=u32} ms (15 frames at 50 Hz about 300 ms plus the first conversion), FIFO_COUNT {=u8}",
            t_first,
            count
        );
        check!(
            fails,
            first.is_some_and(|c| c - c0 == 1) && count == 15 && full_flag,
            "[{=str}] F4 full: edges {=u64}, FIFO_COUNT {=u8} (expected the measured streaming limit 15), INT_STATUS fifo_full {=bool}",
            first.map_or(0, |c| c - c0),
            count,
            full_flag
        );
        let seen = first.unwrap_or(c0);
        Timer::after_millis(60).await;
        defmt::info!(
            "[STAT] F4 full: FIFO_COUNT 60 ms after the edge {=u8}",
            baro.fifo_count().await.unwrap_or(0)
        );
        Timer::after_millis(940).await;
        defmt::info!(
            "[STAT] F4 full: {=u64} more edges in the next 1 s with the FIFO overflowing and the flag cleared",
            dr.count() - seen
        );
    }

    // F4, FIFO-full interrupt on the pin, stop-on-full.
    defmt::info!("[INFO] F4 full interrupt, stop-on-full: only fifo_full on INT, 50 Hz, no reads");
    let f4g = Config::new()
        .with_fifo(fifo(FifoMode::StopOnFull, pt, 0, none))
        .with_interrupt(
            InterruptPin::new().with_sources(InterruptSources::none().with_fifo_full(true)),
        );
    if setup(&mut fails, &mut baro, "F4g", &f4g).await {
        let c0 = dr.count();
        let t0 = Instant::now();
        let first = dr.wait_beyond(c0, Duration::from_millis(2000)).await;
        let t_first = t0.elapsed().as_millis() as u32;
        let count = baro.fifo_count().await.unwrap_or(0);
        let st = baro.interrupt_status().await;
        let full_flag = st.as_ref().is_ok_and(|s| s.fifo_full);
        Timer::after_millis(60).await;
        let later = baro.fifo_count().await.unwrap_or(0);
        defmt::info!(
            "[STAT] F4 full stop-on-full: first edge after {=u32} ms, FIFO_COUNT {=u8}, 60 ms later {=u8}",
            t_first,
            count,
            later
        );
        check!(
            fails,
            first.is_some_and(|c| c - c0 == 1) && count == 16 && full_flag,
            "[{=str}] F4 full stop-on-full: edges {=u64}, FIFO_COUNT {=u8} (expected 16), INT_STATUS fifo_full {=bool}",
            first.map_or(0, |c| c - c0),
            count,
            full_flag
        );
        let seen = first.unwrap_or(c0);
        Timer::after_millis(1000).await;
        defmt::info!(
            "[STAT] F4 full stop-on-full: {=u64} more edges in the next 1 s",
            dr.count() - seen
        );
    }

    // F5. Contents and decimation. Streaming FIFO of one quantity: the count stops at 31, not the
    // datasheet's 32 (logs bmp581_fifo-20261111-180444 and bmp581_fifo-20261111-181709, F5: lowest 31,
    // highest 31 over 20 reads), and fifo_read reports that as full.
    for (label, data) in [
        ("pressure", FifoData::Pressure),
        ("temperature", FifoData::Temperature),
    ] {
        defmt::info!("[INFO] F5 {=str}-only frames, 240 Hz x1/x1", label);
        let c = fast(fifo(FifoMode::Stream, data, 0, none));
        if !setup(&mut fails, &mut baro, "F5", &c).await {
            continue;
        }
        Timer::after_millis(400).await;
        let depth = baro.fifo_count().await.unwrap_or(0);
        let (lo, hi) = count_range(&mut baro, 20).await;
        defmt::info!(
            "[STAT] F5 {=str}: 20 further FIFO_COUNT reads over 100 ms, unread: lowest {=u8}, highest {=u8}",
            label,
            lo,
            hi
        );
        let mut buf = [0u8; 15];
        let s_before = baro.stats();
        let r = baro.fifo_read(&mut buf).await;
        let s_after = baro.stats();
        let (frames, bytes) = r.as_ref().map_or((0, 0), |r| (r.frames, r.bytes));
        let r_full = r.as_ref().is_ok_and(|r| r.full);
        let mut a = Acc::new();
        a.feed(&buf[..bytes], data);
        defmt::info!(
            "[STAT] F5 {=str}: depth {=u8} frames (measured streaming limit 31, datasheet 32), first read {=u8} frames {=usize} bytes, first bytes {=[u8]:x}",
            label,
            depth,
            frames,
            bytes,
            &buf[..6]
        );
        check!(
            fails,
            depth == 31
                && frames == 5
                && bytes == 15
                && a.frames == 5
                && a.wrong_kind == 0
                && a.invalid == 0
                && a.empties == 0,
            "[{=str}] F5 {=str}: depth {=u8} (expected the measured streaming limit 31), read {=u8} frames in {=usize} bytes (expected 5 in 15), {=u32} decoded as the right kind",
            label,
            depth,
            frames,
            bytes,
            a.frames
        );
        check!(
            fails,
            r_full && s_after.fifo_full - s_before.fifo_full == 1,
            "[{=str}] F5 {=str}: fifo_read at the streaming limit reports full {=bool}, stats.fifo_full +1 expected",
            label,
            r_full
        );
        let s = stream(&mut baro, &mut dr, data, Duration::from_millis(2000)).await;
        check_stream(&mut fails, label, &s, 1, data);
    }
    defmt::info!("[INFO] F5 decimation by 4, P+T, 240 Hz x1/x1, 3 s");
    let c = fast(fifo(FifoMode::Stream, pt, 0, FifoDecimation::By4));
    if setup(&mut fails, &mut baro, "F5d", &c).await {
        let s = stream(&mut baro, &mut dr, pt, Duration::from_millis(3000)).await;
        check_stream(&mut fails, "F5 decimation 4", &s, 4, pt);
    }

    // F6. The three points that rest on the datasheet, at 1 Hz so that frames arrive one at a time.
    defmt::info!("[INFO] F6 datasheet points, 1 Hz 16x/2x, stream P+T");
    let f6 =
        Config::new()
            .with_odr(OutputDataRate::Hz1)
            .with_fifo(fifo(FifoMode::Stream, pt, 0, none));
    if setup(&mut fails, &mut baro, "F6", &f6).await {
        // Point 1: a burst from FIFO_DATA stays at that register and returns consecutive frames.
        let mut b = [0u8; 12];
        let c = wait_count(&mut baro, 2, 4000).await;
        let r = baro.read_registers(bmp581::reg::FIFO_DATA, &mut b).await;
        let left = baro.fifo_count().await.unwrap_or(255);
        let (v1, v2) = (valid_pt(&b[..6]), valid_pt(&b[6..]));
        defmt::info!(
            "[STAT] F6 point 1: count {=u8} -> 12-byte burst {=[u8]:x} -> count {=u8}",
            c.unwrap_or(0),
            &b[..],
            left
        );
        check!(
            fails,
            c.is_some() && r.is_ok() && v1.is_some() && v2.is_some() && left == 0,
            "[{=str}] F6 point 1 (burst returns consecutive frames): 12 bytes = {=bool} + {=bool} valid frames, count after {=u8}, expected true, true, 0",
            v1.is_some(),
            v2.is_some(),
            left
        );
        // Past the last frame the chip returns the empty frame.
        let c = wait_count(&mut baro, 1, 2500).await;
        let r = baro.read_registers(bmp581::reg::FIFO_DATA, &mut b).await;
        let left = baro.fifo_count().await.unwrap_or(255);
        let empty = b[6..].iter().all(|&x| x == 0x7F);
        defmt::info!(
            "[STAT] F6 past the end: count {=u8} -> 12-byte burst {=[u8]:x} -> count {=u8}",
            c.unwrap_or(0),
            &b[..],
            left
        );
        check!(
            fails,
            c.is_some() && r.is_ok() && valid_pt(&b[..6]).is_some() && empty && left == 0,
            "[{=str}] F6 past the last frame: first frame valid ({=bool}), second 6 bytes all 0x7F ({=bool}), count after {=u8}",
            valid_pt(&b[..6]).is_some(),
            empty,
            left
        );
        // Point 3: a frame read incompletely is sent again.
        let c = wait_count(&mut baro, 1, 2500).await;
        let mut part = [0u8; 4];
        let r1 = baro.read_registers(bmp581::reg::FIFO_DATA, &mut part).await;
        let mid = baro.fifo_count().await.unwrap_or(255);
        let mut whole = [0u8; 6];
        let r2 = baro
            .read_registers(bmp581::reg::FIFO_DATA, &mut whole)
            .await;
        let left = baro.fifo_count().await.unwrap_or(255);
        defmt::info!(
            "[STAT] F6 point 3: count {=u8}, 4 bytes {=[u8]:x}, count {=u8}, then 6 bytes {=[u8]:x}, count {=u8}",
            c.unwrap_or(0),
            &part[..],
            mid,
            &whole[..],
            left
        );
        check!(
            fails,
            c.is_some()
                && r1.is_ok()
                && r2.is_ok()
                && mid == 1
                && whole[..4] == part
                && valid_pt(&whole).is_some()
                && left == 0,
            "[{=str}] F6 point 3 (incomplete frame is sent again): count after 4 bytes {=u8} (expected 1), re-read starts with the same 4 bytes ({=bool}), full frame valid ({=bool}), count after {=u8} (expected 0)",
            mid,
            whole[..4] == part,
            valid_pt(&whole).is_some(),
            left
        );
    }
    check!(
        fails,
        flush_ok,
        "[{=str}] F6 point 2 (a flush empties the FIFO): see F3 stream, count 0 right after fifo_flush ({=bool})",
        flush_ok
    );

    // F7. Default configuration and the end.
    defmt::info!("[INFO] F7 default configuration");
    let d = Config::default();
    if setup(&mut fails, &mut baro, "F7", &d).await {
        Timer::after_millis(100).await;
        check!(
            fails,
            baro.verify().await.is_ok(),
            "[{=str}] F7: verify Ok with the default configuration"
        );
        check!(
            fails,
            matches!(baro.fifo_count().await, Err(Error::FifoOff)),
            "[{=str}] F7: fifo_count is Err(FifoOff) with the FIFO off"
        );
        let idle = baro.read_register(bmp581::reg::FIFO_DATA).await;
        check!(
            fails,
            matches!(idle, Ok(0x7F)),
            "[{=str}] F7: FIFO_DATA of a disabled FIFO reads 0x7F ({=u8:#04x})",
            idle.unwrap_or(0)
        );
        let st = baro.status().await;
        let is = baro.interrupt_status().await;
        let s = baro.stats();
        defmt::info!(
            "[INFO] health: STATUS {=u8:#04x}, power-on resets {=u32}, bus errors {=u32}, fifo_full events {=u32}, fifo frames {=u32}, INT_STATUS data_ready {=bool}",
            st.as_ref().map_or(0, |s| s.raw),
            s.power_on_resets,
            s.bus_errors,
            s.fifo_full,
            s.fifo_frames,
            is.as_ref().is_ok_and(|i| i.data_ready)
        );
        check!(
            fails,
            st.as_ref().is_ok_and(|s| s.core_ready && !s.nvm_error)
                && s.power_on_resets == 0
                && s.bus_errors == 0,
            "[{=str}] F7 health: core ready, no NVM error, {=u32} power-on resets, {=u32} bus errors in the whole run",
            s.power_on_resets,
            s.bus_errors
        );
    }

    hil_common::exit(fails)
}
