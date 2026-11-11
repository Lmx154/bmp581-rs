//! On-board check of the bmp581 driver: async I2C1 at 400 kHz, data-ready on GPIO21, then the
//! blocking front-end once.
#![no_std]
#![no_main]

use core::future::Future;
use core::pin::pin;
use core::task::Poll;

use bmp581::{Bmp581, Config, Error, OutputDataRate, Oversampling, PowerMode};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C1;
use embassy_rp::pwm::{self, ChannelBPin, InputMode, Pwm, Slice};
use embassy_time::{Delay, Duration, Instant, with_timeout};
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

const WINDOW: Duration = Duration::from_millis(2000);
const EDGE_TIMEOUT: Duration = Duration::from_millis(250);

/// The INT pin used twice: a PWM slice counts every rising edge in hardware, independent of what
/// the application reads, and the GPIO interrupt wakes the waiting task.
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

#[derive(Default)]
struct Run {
    reads: u32,
    edges: u64,
    timeouts: u32,
    window_us: u64,
    read_sum_us: u64,
    read_max_us: u32,
    bus_errors: u32,
    p_first: i32,
    p_sum: f64,
    p_sumsq: f64,
    t_sum: f64,
    p_min: f32,
    p_max: f32,
    t_min: f32,
    t_max: f32,
}

impl Run {
    fn rate_hz(&self) -> f32 {
        if self.window_us == 0 {
            0.0
        } else {
            (self.edges.saturating_sub(1)) as f32 * 1.0e6 / self.window_us as f32
        }
    }
    fn mean_read_us(&self) -> u32 {
        if self.reads == 0 {
            0
        } else {
            (self.read_sum_us / self.reads as u64) as u32
        }
    }
    fn p_mean(&self) -> f32 {
        (self.p_first as f64 / 64.0 + self.p_sum / self.reads.max(1) as f64) as f32
    }
    fn t_mean(&self) -> f32 {
        (self.t_sum / self.reads.max(1) as f64) as f32
    }
    fn p_std(&self) -> f32 {
        let n = self.reads.max(1) as f64;
        let var = self.p_sumsq / n - (self.p_sum / n) * (self.p_sum / n);
        sqrt(var.max(0.0)) as f32
    }
    fn plausible(&self) -> bool {
        self.reads > 0
            && self.p_min >= 60_000.0
            && self.p_max <= 110_000.0
            && self.t_min >= 10.0
            && self.t_max <= 50.0
    }
}

fn sqrt(x: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut g = if x > 1.0 { x } else { 1.0 };
    for _ in 0..60 {
        g = 0.5 * (g + x / g);
    }
    g
}

type Baro<'d> = Bmp581<I2c<'d, I2C1, i2c::Async>>;

fn init_error<E: core::fmt::Debug>(e: &Error<E>) -> defmt::Debug2Format<'_, Error<E>> {
    defmt::Debug2Format(e)
}

/// One sample per data-ready edge for `WINDOW`, with the hardware edge count as the reference.
async fn acquire(baro: &mut Baro<'_>, dr: &mut DataReady<'_>) -> Run {
    let mut run = Run {
        p_min: f32::MAX,
        p_max: f32::MIN,
        t_min: f32::MAX,
        t_max: f32::MIN,
        ..Run::default()
    };
    let c0 = dr.count();
    let mut seen = c0;
    let start = Instant::now();
    let mut first: Option<(u64, Instant)> = None;
    let mut last: Option<(u64, Instant)> = None;
    while start.elapsed() < WINDOW {
        let Some(c) = dr.wait_beyond(seen, EDGE_TIMEOUT).await else {
            run.timeouts += 1;
            break;
        };
        let now = Instant::now();
        seen = c;
        first.get_or_insert((c, now));
        last = Some((c, now));
        let t0 = Instant::now();
        let r = baro.read().await;
        let us = t0.elapsed().as_micros() as u32;
        match r {
            Ok(s) => {
                run.read_sum_us += us as u64;
                run.read_max_us = run.read_max_us.max(us);
                if run.reads == 0 {
                    run.p_first = s.pressure;
                }
                let p = s.pascals();
                let t = s.celsius();
                let d = (s.pressure - run.p_first) as f64 / 64.0;
                run.p_sum += d;
                run.p_sumsq += d * d;
                run.t_sum += t as f64;
                run.p_min = run.p_min.min(p);
                run.p_max = run.p_max.max(p);
                run.t_min = run.t_min.min(t);
                run.t_max = run.t_max.max(t);
                run.reads += 1;
            }
            Err(_) => run.bus_errors += 1,
        }
    }
    run.edges = seen - c0;
    if let (Some((_, t1)), Some((_, t2))) = (first, last) {
        run.window_us = (t2 - t1).as_micros().max(1);
    }
    run
}

fn report(failures: &mut u32, name: &str, run: &Run, nominal_hz: f32, strict: bool) {
    let mut fails = 0u32;
    let missed = run.edges as i64 - run.reads as i64;
    let rate = run.rate_hz();
    defmt::info!(
        "[INFO] {=str}: {=u32} samples, {=u64} data-ready edges, {=i64} missed, rate {=f32} Hz (nominal {=f32}), read mean {=u32} us worst {=u32} us, bus errors {=u32}, timeouts {=u32}",
        name,
        run.reads,
        run.edges,
        missed,
        rate,
        nominal_hz,
        run.mean_read_us(),
        run.read_max_us,
        run.bus_errors,
        run.timeouts
    );
    defmt::info!(
        "[STAT] {=str}: pressure mean {=f32} Pa std {=f32} Pa, temperature mean {=f32} degC",
        name,
        run.p_mean(),
        run.p_std(),
        run.t_mean()
    );
    check!(
        fails,
        run.reads > 10 && run.bus_errors == 0 && run.timeouts == 0,
        "[{=str}] {=str}: acquired {=u32} samples, {=u32} bus errors, {=u32} timeouts",
        name,
        run.reads,
        run.bus_errors,
        run.timeouts
    );
    check!(
        fails,
        run.plausible(),
        "[{=str}] {=str}: plausible (pressure {=f32}..{=f32} Pa, temperature {=f32}..{=f32} degC)",
        name,
        run.p_min,
        run.p_max,
        run.t_min,
        run.t_max
    );
    if strict {
        check!(
            fails,
            missed == 0,
            "[{=str}] {=str}: no data-ready edge missed ({=i64})",
            name,
            missed
        );
        check!(
            fails,
            rate > nominal_hz * 0.9 && rate < nominal_hz * 1.1,
            "[{=str}] {=str}: rate {=f32} Hz within 10 % of nominal {=f32} Hz",
            name,
            rate,
            nominal_hz
        );
    }
    *failures += fails;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut failures = 0u32;

    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    cfg.sda_pullup = false;
    cfg.scl_pullup = false;
    let bus = I2c::new_async(
        p.I2C1.reborrow(),
        p.PIN_23.reborrow(),
        p.PIN_22.reborrow(),
        Irqs,
        cfg.clone(),
    );
    let mut baro = Bmp581::new(bus, bmp581::DEFAULT_ADDRESS);
    let mut delay = Delay;
    let mut dr = DataReady::new(p.PWM_SLICE2.reborrow(), p.PIN_21.reborrow());

    // 1. Init and identity.
    let c50 = Config::default();
    match baro.init(&mut delay, &c50).await {
        Ok(_) => defmt::info!("[PASS] init: chip id, NVM and configuration accepted"),
        Err(e) => {
            defmt::error!("[FAIL] init: {}", init_error(&e));
            hil_common::exit(1)
        }
    }
    match (
        baro.read_register(bmp581::reg::CHIP_ID).await,
        baro.read_register(bmp581::reg::REV_ID).await,
    ) {
        (Ok(id), Ok(rev)) => {
            check!(
                failures,
                id == 0x50,
                "[{=str}] identity: CHIP_ID {=u8:#04x}",
                id
            );
            defmt::info!("[INFO] REV_ID {=u8:#04x}", rev);
        }
        _ => check!(failures, false, "[{=str}] identity: register read failed"),
    }

    // 2a. Normal mode, 50 Hz.
    let run = acquire(&mut baro, &mut dr).await;
    report(&mut failures, "normal 50 Hz", &run, c50.odr.hz(), true);
    defmt::info!(
        "[STAT] normal 50 Hz: pressure std over the run {=f32} Pa",
        run.p_std()
    );

    // 2b. Normal mode, fastest rate at the lowest oversampling.
    let fast = Config::new()
        .with_pressure_oversampling(Oversampling::X1)
        .with_temperature_oversampling(Oversampling::X1)
        .with_odr(OutputDataRate::Hz240);
    match baro.init(&mut delay, &fast).await {
        Ok(_) => {
            let run = acquire(&mut baro, &mut dr).await;
            report(
                &mut failures,
                "normal 240 Hz x1/x1",
                &run,
                fast.odr.hz(),
                true,
            );
        }
        Err(e) => check!(
            failures,
            false,
            "[{=str}] init 240 Hz x1/x1 failed: {}",
            init_error(&e)
        ),
    }

    // 2c. Continuous mode at the lowest oversampling: the rate is whatever the chip reaches.
    let cont = fast.with_power_mode(PowerMode::Continuous);
    match baro.init(&mut delay, &cont).await {
        Ok(_) => {
            let run = acquire(&mut baro, &mut dr).await;
            report(&mut failures, "continuous x1/x1", &run, 0.0, false);
        }
        Err(e) => check!(
            failures,
            false,
            "[{=str}] init continuous failed: {}",
            init_error(&e)
        ),
    }

    // Negative control: 240 Hz with 128x oversampling is not a valid combination.
    let bad = Config::new()
        .with_pressure_oversampling(Oversampling::X128)
        .with_temperature_oversampling(Oversampling::X128)
        .with_odr(OutputDataRate::Hz240);
    let r = baro.init(&mut delay, &bad).await;
    check!(
        failures,
        matches!(r, Err(Error::InvalidConfig(_))),
        "[{=str}] invalid 240 Hz x128/x128 rejected with InvalidConfig"
    );

    // 3. Forced mode: trigger, wait for data-ready, read.
    let forced = Config::new().with_power_mode(PowerMode::Forced);
    match baro.init(&mut delay, &forced).await {
        Ok(_) => {
            let (mut sum, mut max, mut min, mut n, mut bad_n) = (0u64, 0u64, u64::MAX, 0u32, 0u32);
            let (mut p_last, mut t_last) = (0.0f32, 0.0f32);
            for _ in 0..10 {
                let seen = dr.count();
                let t0 = Instant::now();
                if baro.trigger_forced().await.is_err() {
                    bad_n += 1;
                    continue;
                }
                if dr
                    .wait_beyond(seen, Duration::from_millis(250))
                    .await
                    .is_none()
                {
                    bad_n += 1;
                    continue;
                }
                let us = t0.elapsed().as_micros();
                match baro.read().await {
                    Ok(s) => {
                        sum += us;
                        max = max.max(us);
                        min = min.min(us);
                        n += 1;
                        p_last = s.pascals();
                        t_last = s.celsius();
                    }
                    Err(_) => bad_n += 1,
                }
            }
            defmt::info!(
                "[INFO] forced x16/x2: trigger to data-ready {=u64} us mean, {=u64} min, {=u64} max over {=u32} runs",
                sum / n.max(1) as u64,
                min,
                max,
                n
            );
            check!(
                failures,
                n == 10 && bad_n == 0,
                "[{=str}] forced: {=u32} of 10 measurements completed, {=u32} failed",
                n,
                bad_n
            );
            check!(
                failures,
                (60_000.0..=110_000.0).contains(&p_last) && (10.0..=50.0).contains(&t_last),
                "[{=str}] forced: plausible (pressure {=f32} Pa, temperature {=f32} degC)",
                p_last,
                t_last
            );
            match baro.interrupt_status().await {
                Ok(st) => check!(
                    failures,
                    st.data_ready,
                    "[{=str}] forced: INT_STATUS data_ready set after a measurement ({=bool})",
                    st.data_ready
                ),
                Err(_) => check!(failures, false, "[{=str}] forced: INT_STATUS read failed"),
            }
        }
        Err(e) => check!(
            failures,
            false,
            "[{=str}] init forced failed: {}",
            init_error(&e)
        ),
    }

    // 5. Blocking front-end over the blocking driver of the same peripheral.
    drop(baro.release());
    drop(dr);
    let bus = I2c::new_blocking(
        p.I2C1.reborrow(),
        p.PIN_23.reborrow(),
        p.PIN_22.reborrow(),
        cfg,
    );
    let mut b = bmp581::blocking::Bmp581::new(bus, bmp581::DEFAULT_ADDRESS);
    match b.init(&mut Delay, &Config::default()) {
        Ok(_) => {
            embassy_time::Timer::after_millis(60).await;
            match b.read() {
                Ok(s) => {
                    let (pa, c) = (s.pascals(), s.celsius());
                    defmt::info!(
                        "[INFO] blocking: pressure {=f32} Pa, temperature {=f32} degC",
                        pa,
                        c
                    );
                    check!(
                        failures,
                        (60_000.0..=110_000.0).contains(&pa) && (10.0..=50.0).contains(&c),
                        "[{=str}] blocking: init and read, plausible"
                    );
                }
                Err(e) => check!(
                    failures,
                    false,
                    "[{=str}] blocking read failed: {}",
                    init_error(&e)
                ),
            }
        }
        Err(e) => check!(
            failures,
            false,
            "[{=str}] blocking init failed: {}",
            init_error(&e)
        ),
    }

    hil_common::exit(failures)
}
