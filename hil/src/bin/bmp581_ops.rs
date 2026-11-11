//! On-board test of the bmp581 driver operations: verify, recover, configure, the typed health reads,
//! and three open points (power-mode read-back in slow normal mode, pad-drive and DSP_CONFIG
//! read-back, effective oversampling with pressure off). Findings are reported, not fixed.
#![no_std]
#![no_main]

use core::future::Future;
use core::pin::pin;
use core::task::Poll;

use bmp581::{
    Bmp581, Config, Error, FifoConfig, FifoData, FifoMode, IirFilter, InterruptDrive,
    InterruptMode, InterruptPin, InterruptPolarity, InterruptSources, OutOfRange, OutOfRangeCount,
    OutputDataRate, Oversampling, PowerMode,
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
    ($fails:ident, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "PASS" $(, $arg)*);
        } else {
            $fails += 1;
            defmt::error!($fmt, "FAIL" $(, $arg)*);
        }
    };
}

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

    /// Waits until the hardware edge count exceeds `seen`; `None` on timeout. The timeout wraps the
    /// wait for the pin edge only.
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

fn eh<E: core::fmt::Debug>(e: &Error<E>) -> defmt::Debug2Format<'_, Error<E>> {
    defmt::Debug2Format(e)
}

fn plausible(p: f32, t: f32) -> bool {
    (80_000.0..=110_000.0).contains(&p) && (10.0..=50.0).contains(&t)
}

async fn sample(baro: &mut Baro<'_>) -> Option<(f32, f32)> {
    baro.read()
        .await
        .ok()
        .map(|s| (s.pascals(), s.celsius()))
}

/// `verify()` with the mismatch details printed. True when Ok.
async fn ver(baro: &mut Baro<'_>, what: &str) -> bool {
    match baro.verify().await {
        Ok(()) => true,
        Err(Error::ConfigMismatch(m)) => {
            defmt::error!(
                "[STAT] {=str}: verify mismatch register {=u8:#04x} mask {=u8:#04x} expected {=u8:#04x} found {=u8:#04x}",
                what,
                m.register,
                m.mask,
                m.expected,
                m.found
            );
            false
        }
        Err(e) => {
            defmt::error!("[STAT] {=str}: verify error {}", what, eh(&e));
            false
        }
    }
}

async fn show(baro: &mut Baro<'_>, what: &str) {
    if let Ok(s) = baro.status().await {
        defmt::info!("[STAT] {=str}: status {}", what, s);
    }
    if let Ok(e) = baro.effective_oversampling().await {
        defmt::info!("[STAT] {=str}: effective oversampling {}", what, e);
    }
}

fn configs() -> [(&'static str, Config); 12] {
    let fast = Config::new()
        .with_pressure_oversampling(Oversampling::X1)
        .with_temperature_oversampling(Oversampling::X1)
        .with_odr(OutputDataRate::Hz240);
    let fifo = FifoConfig::new()
        .with_mode(FifoMode::Stream)
        .with_data(FifoData::PressureAndTemperature)
        .with_threshold(4);
    let window = OutOfRange::new()
        .with_reference_pa(100_000)
        .with_range_pa(255)
        .with_count(OutOfRangeCount::Three);
    let pin_odl = InterruptPin::new()
        .with_mode(InterruptMode::Latched)
        .with_polarity(InterruptPolarity::ActiveLow)
        .with_drive(InterruptDrive::OpenDrain);
    [
        ("default", Config::new()),
        ("240 Hz 1x/1x", fast),
        (
            "10 Hz 64x/4x IIR 7/7",
            Config::new()
                .with_pressure_oversampling(Oversampling::X64)
                .with_temperature_oversampling(Oversampling::X4)
                .with_odr(OutputDataRate::Hz10)
                .with_pressure_iir(IirFilter::Coeff7)
                .with_temperature_iir(IirFilter::Coeff7),
        ),
        ("latched low open-drain", Config::new().with_interrupt(pin_odl)),
        ("standby end", Config::new().with_power_mode(PowerMode::Standby)),
        ("continuous 1x/1x", fast.with_power_mode(PowerMode::Continuous)),
        ("forced end", Config::new().with_power_mode(PowerMode::Forced)),
        (
            "fifo stream P+T thr 4",
            Config::new().with_fifo(fifo).with_interrupt(
                InterruptPin::new().with_sources(InterruptSources::new().with_fifo_threshold(true)),
            ),
        ),
        (
            "out-of-range window",
            Config::new()
                .with_out_of_range(Some(window))
                .with_interrupt(
                    InterruptPin::new().with_sources(InterruptSources::new().with_out_of_range(true)),
                ),
        ),
        (
            "everything",
            Config::new()
                .with_pressure_iir(IirFilter::Coeff3)
                .with_temperature_iir(IirFilter::Coeff3)
                .with_fifo(fifo)
                .with_out_of_range(Some(window))
                .with_interrupt(pin_odl.with_sources(
                    InterruptSources::new()
                        .with_fifo_threshold(true)
                        .with_out_of_range(true),
                )),
        ),
        (
            "forced 64x/4x IIR 31",
            Config::new()
                .with_pressure_oversampling(Oversampling::X64)
                .with_temperature_oversampling(Oversampling::X4)
                .with_pressure_iir(IirFilter::Coeff31)
                .with_power_mode(PowerMode::Forced),
        ),
        ("default again", Config::new()),
    ]
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut fails = 0u32;
    let t_start = Instant::now();

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
    let mut baro: Baro = Bmp581::new(bus, bmp581::DEFAULT_ADDRESS);
    let mut delay = Delay;
    let mut dr = DataReady::new(p.PWM_SLICE2.reborrow(), p.PIN_21.reborrow());
    let default = Config::new();

    // ---- Check 1: verify across configurations ----
    defmt::info!("[INFO] check 1: verify across configurations");
    let list = configs();
    let mut ok1 = 0u32;
    let mut n1 = 0u32;
    for (i, (name, c)) in list.iter().enumerate() {
        let t0 = Instant::now();
        let r = if i == 0 {
            baro.init(&mut delay, c).await
        } else {
            baro.configure(&mut delay, c).await
        };
        let us = t0.elapsed().as_micros() as u32;
        n1 += 1;
        match r {
            Ok(info) => {
                defmt::info!(
                    "[STAT] {=str}: applied in {=u32} us, effective {} / {}",
                    *name,
                    us,
                    info.pressure_oversampling,
                    info.temperature_oversampling
                );
            }
            Err(e) => {
                fails += 1;
                defmt::error!("[FAIL] {=str}: apply failed: {}", *name, eh(&e));
                continue;
            }
        }
        let a = ver(&mut baro, name).await;
        show(&mut baro, name).await;
        if *name == "forced end" || *name == "forced 64x/4x IIR 31" {
            let seen = dr.count();
            let t0 = Instant::now();
            let trig = baro.trigger_forced().await.is_ok();
            let got = dr.wait_beyond(seen, Duration::from_millis(500)).await.is_some();
            let us = t0.elapsed().as_micros() as u32;
            let s = sample(&mut baro).await;
            let good = trig && got && s.is_some_and(|(p, t)| plausible(p, t));
            let (pp, tt) = s.unwrap_or((0.0, 0.0));
            check!(
                fails,
                good,
                "[{=str}] {=str}: forced measurement, trigger {=bool} edge {=bool} in {=u32} us, {=f32} Pa {=f32} degC",
                *name,
                trig,
                got,
                us,
                pp,
                tt
            );
            let a2 = ver(&mut baro, name).await;
            if !a2 {
                fails += 1;
            }
        }
        Timer::after_millis(200).await;
        let b = ver(&mut baro, name).await;
        if a && b {
            ok1 += 1;
        }
        check!(
            fails,
            a && b,
            "[{=str}] {=str}: verify Ok right after {=bool}, 200 ms later {=bool}",
            *name,
            a,
            b
        );
    }
    defmt::info!("[STAT] check 1: {=u32} of {=u32} configurations verified Ok both times", ok1, n1);

    // ---- Check 2: the three open points ----
    defmt::info!("[INFO] check 2a: slow normal mode, power-mode read-back");
    let mut a_ok_all = true;
    let mut a_false = 0u32;
    for (name, odr, span_ms, nominal) in [
        ("4 Hz", OutputDataRate::Hz4, 2500u64, 4u32),
        ("1 Hz", OutputDataRate::Hz1, 2500, 1),
        ("0.125 Hz", OutputDataRate::Hz0_125, 10_000, 0),
    ] {
        let _ = nominal;
        let c = Config::new().with_odr(odr);
        if let Err(e) = baro.configure(&mut delay, &c).await {
            fails += 1;
            a_ok_all = false;
            defmt::error!("[FAIL] 2a {=str}: configure failed {}", name, eh(&e));
            continue;
        }
        let interval = span_ms / 50;
        let mut raw = [0u8; 50];
        let mut ok = 0u32;
        let mut shown = 0u32;
        let t0 = Instant::now();
        for slot in raw.iter_mut() {
            *slot = baro.read_register(bmp581::reg::ODR_CONFIG).await.unwrap_or(0xEE);
            match baro.verify().await {
                Ok(()) => ok += 1,
                Err(Error::ConfigMismatch(m)) => {
                    if shown < 6 {
                        shown += 1;
                        defmt::error!(
                            "[STAT] 2a {=str}: mismatch at {=u32} ms: register {=u8:#04x} mask {=u8:#04x} expected {=u8:#04x} found {=u8:#04x} (raw ODR_CONFIG just before {=u8:#04x})",
                            name,
                            t0.elapsed().as_millis() as u32,
                            m.register,
                            m.mask,
                            m.expected,
                            m.found,
                            *slot
                        );
                    }
                }
                Err(e) => {
                    defmt::error!("[STAT] 2a {=str}: verify error {}", name, eh(&e));
                }
            }
            Timer::after_millis(interval).await;
        }
        let span = t0.elapsed().as_millis() as u32;
        defmt::info!(
            "[STAT] 2a {=str}: verify Ok {=u32} of 50 over {=u32} ms; expected ODR_CONFIG {=u8:#04x}; raw ODR_CONFIG per call {=[u8]:x}",
            name,
            ok,
            span,
            c.odr_config(PowerMode::Normal),
            raw
        );
        if ok != 50 {
            a_ok_all = false;
            a_false += 50 - ok;
        }
    }
    check!(
        fails,
        a_ok_all,
        "[{=str}] 2a: verify raises no false alarm in slow normal mode ({=u32} mismatches)",
        a_false
    );

    defmt::info!("[INFO] check 2b: interrupt pin drive/polarity/mode");
    let mut b_ok = true;
    for mode in [InterruptMode::Pulsed, InterruptMode::Latched] {
        for pol in [InterruptPolarity::ActiveLow, InterruptPolarity::ActiveHigh] {
            for drive in [InterruptDrive::PushPull, InterruptDrive::OpenDrain] {
                let c = Config::new().with_interrupt(
                    InterruptPin::new()
                        .with_mode(mode)
                        .with_polarity(pol)
                        .with_drive(drive),
                );
                if let Err(e) = baro.configure(&mut delay, &c).await {
                    fails += 1;
                    b_ok = false;
                    defmt::error!("[FAIL] 2b: configure failed {}", eh(&e));
                    continue;
                }
                let r1 = ver(&mut baro, "2b").await;
                Timer::after_millis(50).await;
                let r2 = ver(&mut baro, "2b").await;
                let ic = baro.read_register(bmp581::reg::INT_CONFIG).await.unwrap_or(0xEE);
                let dc = baro.read_register(bmp581::reg::DSP_CONFIG).await.unwrap_or(0xEE);
                defmt::info!(
                    "[STAT] 2b {} {} {}: verify {=bool}/{=bool}; INT_CONFIG wrote {=u8:#04x} read {=u8:#04x}; DSP_CONFIG wrote {=u8:#04x} read {=u8:#04x}",
                    mode,
                    pol,
                    drive,
                    r1,
                    r2,
                    c.int_config(),
                    ic,
                    c.dsp_config(),
                    dc
                );
                if !(r1 && r2) {
                    b_ok = false;
                }
            }
        }
    }
    check!(
        fails,
        b_ok,
        "[{=str}] 2b: verify Ok for all eight interrupt pin combinations"
    );

    defmt::info!("[INFO] check 2c: temperature only, normal mode");
    let mut c_ok = true;
    for osr in [Oversampling::X1, Oversampling::X16, Oversampling::X128] {
        let c = Config::new()
            .with_pressure_enabled(false)
            .with_pressure_oversampling(osr)
            .with_temperature_oversampling(Oversampling::X2)
            .with_odr(OutputDataRate::Hz10);
        match baro.configure(&mut delay, &c).await {
            Ok(info) => {
                let r1 = ver(&mut baro, "2c").await;
                Timer::after_millis(150).await;
                let r2 = ver(&mut baro, "2c").await;
                let raw = baro.read_register(bmp581::reg::OSR_EFF).await.unwrap_or(0xEE);
                let eff = baro.effective_oversampling().await;
                if let Ok(e) = eff {
                    defmt::info!(
                        "[STAT] 2c applied P {} T {}; OSR_EFF raw {=u8:#04x}: valid {=bool} P {} T {}; info P {} T {}; verify {=bool}/{=bool}",
                        osr,
                        Oversampling::X2,
                        raw,
                        e.valid,
                        e.pressure,
                        e.temperature,
                        info.pressure_oversampling,
                        info.temperature_oversampling,
                        r1,
                        r2
                    );
                }
                if !(r1 && r2) {
                    c_ok = false;
                }
            }
            Err(e) => {
                fails += 1;
                c_ok = false;
                defmt::error!("[FAIL] 2c: configure failed {}", eh(&e));
            }
        }
    }
    check!(
        fails,
        c_ok,
        "[{=str}] 2c: verify Ok with pressure off at 1x, 16x, 128x"
    );

    // ---- Check 3: positive control ----
    defmt::info!("[INFO] check 3: positive control");
    match baro.configure(&mut delay, &default).await {
        Ok(_) => {
            let reg = bmp581::reg::INT_SOURCE;
            let mut before = baro.read_register(reg).await.unwrap_or(0xEE);
            let mut wrote = before ^ 0x02;
            let w = baro.write_register(reg, wrote).await.is_ok();
            let mut after = baro.read_register(reg).await.unwrap_or(0xEE);
            defmt::info!(
                "[STAT] 3: normal mode: INT_SOURCE before {=u8:#04x}, wrote {=u8:#04x} ok {=bool}, read back {=u8:#04x}",
                before,
                wrote,
                w,
                after
            );
            if after != wrote {
                let sb = Config::new().with_power_mode(PowerMode::Standby);
                let _ = baro.configure(&mut delay, &sb).await;
                before = baro.read_register(reg).await.unwrap_or(0xEE);
                wrote = before ^ 0x02;
                let w = baro.write_register(reg, wrote).await.is_ok();
                after = baro.read_register(reg).await.unwrap_or(0xEE);
                defmt::info!(
                    "[STAT] 3: standby fallback: INT_SOURCE before {=u8:#04x}, wrote {=u8:#04x} ok {=bool}, read back {=u8:#04x}",
                    before,
                    wrote,
                    w,
                    after
                );
            }
            let r = baro.verify().await;
            let hit = matches!(r, Err(Error::ConfigMismatch(m)) if m.register == reg);
            if let Err(Error::ConfigMismatch(m)) = r {
                defmt::info!(
                    "[STAT] 3: verify reported register {=u8:#04x} mask {=u8:#04x} expected {=u8:#04x} found {=u8:#04x}",
                    m.register,
                    m.mask,
                    m.expected,
                    m.found
                );
            }
            check!(
                fails,
                hit,
                "[{=str}] 3: verify reports exactly the changed register INT_SOURCE"
            );
            let rc = baro.configure(&mut delay, &default).await.is_ok();
            let ok = ver(&mut baro, "3 after configure").await;
            check!(
                fails,
                rc && ok,
                "[{=str}] 3: configure restores, verify Ok ({=bool}/{=bool})",
                rc,
                ok
            );
        }
        Err(e) => {
            fails += 1;
            defmt::error!("[FAIL] 3: configure failed {}", eh(&e));
        }
    }

    // ---- Check 4: configure on a running device ----
    defmt::info!("[INFO] check 4: configure while streaming");
    {
        let immediate = sample(&mut baro).await;
        if let Some((p, t)) = immediate {
            defmt::info!("[STAT] 4: sample read at once after configure to default: {=f32} Pa {=f32} degC", p, t);
        }
        let seen = dr.count();
        let _ = dr.wait_beyond(seen, Duration::from_millis(100)).await;
        let before_s = sample(&mut baro).await;
        let c0 = dr.count();
        let t0 = Instant::now();
        Timer::after_millis(1000).await;
        let n0 = dr.count() - c0;
        let us0 = t0.elapsed().as_micros() as f32;
        let rate0 = n0 as f32 * 1.0e6 / us0;
        let fast = Config::new()
            .with_pressure_oversampling(Oversampling::X1)
            .with_temperature_oversampling(Oversampling::X1)
            .with_odr(OutputDataRate::Hz240);
        let tc = Instant::now();
        let r = baro.configure(&mut delay, &fast).await;
        let conf_us = tc.elapsed().as_micros() as u32;
        let c1 = dr.count();
        let t1 = Instant::now();
        Timer::after_millis(1000).await;
        let n1 = dr.count() - c1;
        let us1 = t1.elapsed().as_micros() as f32;
        let rate1 = n1 as f32 * 1.0e6 / us1;
        let after_s = sample(&mut baro).await;
        let v = ver(&mut baro, "4").await;
        let tc2 = Instant::now();
        let r2 = baro.configure(&mut delay, &default).await;
        let conf2_us = tc2.elapsed().as_micros() as u32;
        defmt::info!(
            "[STAT] 4: configure 50->240 Hz took {=u32} us, configure back took {=u32} us; edges {=u64} in 1 s before (rate {=f32} Hz), {=u64} after (rate {=f32} Hz)",
            conf_us,
            conf2_us,
            n0,
            rate0,
            n1,
            rate1
        );
        check!(
            fails,
            r.is_ok() && r2.is_ok(),
            "[{=str}] 4: both configure calls Ok ({=bool}/{=bool})",
            r.is_ok(),
            r2.is_ok()
        );
        check!(
            fails,
            rate0 > 50.056 * 0.88 && rate0 < 50.056 * 1.12,
            "[{=str}] 4: before rate {=f32} Hz within 12 % of 50.056",
            rate0
        );
        check!(
            fails,
            rate1 > 240.0 * 0.88 && rate1 < 240.0 * 1.12,
            "[{=str}] 4: after rate {=f32} Hz within 12 % of 240",
            rate1
        );
        check!(fails, v, "[{=str}] 4: verify Ok after configure ({=bool})", v);
        check!(
            fails,
            before_s.is_some_and(|(p, t)| plausible(p, t))
                && after_s.is_some_and(|(p, t)| plausible(p, t)),
            "[{=str}] 4: pressure and temperature plausible before and after"
        );
        if let (Some((p0, t0)), Some((p1, t1))) = (before_s, after_s) {
            defmt::info!(
                "[STAT] 4: before {=f32} Pa {=f32} degC, after {=f32} Pa {=f32} degC",
                p0,
                t0,
                p1,
                t1
            );
        }
    }

    // ---- Check 5: recover ----
    defmt::info!("[INFO] check 5: recover");
    {
        let t0 = Instant::now();
        let r = baro.recover(&mut delay).await;
        let us = t0.elapsed().as_micros() as u32;
        defmt::info!("[STAT] 5: recover in normal mode took {=u32} us", us);
        check!(fails, r.is_ok(), "[{=str}] 5: recover Ok in normal mode ({=bool})", r.is_ok());
        if let Err(e) = &r {
            defmt::error!("[STAT] 5: recover error {}", eh(e));
        }
        let v = ver(&mut baro, "5 normal").await;
        let c0 = dr.count();
        Timer::after_millis(300).await;
        let n = dr.count() - c0;
        let s = sample(&mut baro).await;
        check!(
            fails,
            v && n >= 10 && s.is_some_and(|(p, t)| plausible(p, t)),
            "[{=str}] 5: verify Ok ({=bool}) and still streaming after recover ({=u64} edges in 300 ms)",
            v,
            n
        );

        let sb = Config::new().with_power_mode(PowerMode::Standby);
        let rc = baro.configure(&mut delay, &sb).await.is_ok();
        let t0 = Instant::now();
        let r = baro.recover(&mut delay).await;
        let us = t0.elapsed().as_micros() as u32;
        defmt::info!("[STAT] 5: recover in standby took {=u32} us", us);
        check!(
            fails,
            rc && r.is_ok(),
            "[{=str}] 5: recover Ok in standby ({=bool})",
            r.is_ok()
        );
        if let Err(e) = &r {
            defmt::error!("[STAT] 5: recover error {}", eh(e));
        }
        let v = ver(&mut baro, "5 standby").await;
        check!(fails, v, "[{=str}] 5: verify Ok after standby recover ({=bool})", v);
    }

    // ---- Check 6: health reads ----
    defmt::info!("[INFO] check 6: health reads");
    let seen6 = dr.count();
    match baro.init(&mut delay, &default).await {
        Ok(_) => {
            let early = baro.read().await;
            check!(
                fails,
                matches!(early, Err(Error::NoSample)),
                "[{=str}] 6: read right after init into normal mode is NoSample ({=bool})",
                matches!(early, Err(Error::NoSample))
            );
            let edge6 = dr.wait_beyond(seen6, Duration::from_millis(500)).await.is_some();
            let first = sample(&mut baro).await;
            check!(
                fails,
                edge6 && first.is_some_and(|(p, t)| plausible(p, t)),
                "[{=str}] 6: after the first data-ready edge ({=bool}) the read is plausible, {=f32} Pa {=f32} degC",
                edge6,
                first.map_or(0.0, |v| v.0),
                first.map_or(0.0, |v| v.1)
            );
            match baro.status().await {
                Ok(s) => {
                    defmt::info!("[STAT] 6: status in normal mode {}", s);
                    check!(
                        fails,
                        s.core_ready && !s.nvm_ready && !s.nvm_error,
                        "[{=str}] 6: normal mode status core ready {=bool} nvm ready {=bool} (clear while measuring) nvm error {=bool}",
                        s.core_ready,
                        s.nvm_ready,
                        s.nvm_error
                    );
                }
                Err(e) => check!(fails, false, "[{=str}] 6: status failed {}", eh(&e)),
            }
            let sbc = Config::new().with_power_mode(PowerMode::Standby);
            let _ = baro.configure(&mut delay, &sbc).await;
            if let Ok(s) = baro.status().await {
                defmt::info!("[STAT] 6: status in standby {}", s);
                check!(
                    fails,
                    s.core_ready && s.nvm_ready && !s.nvm_error,
                    "[{=str}] 6: standby status core ready {=bool} nvm ready {=bool} (set in standby) nvm error {=bool}",
                    s.core_ready,
                    s.nvm_ready,
                    s.nvm_error
                );
            }
            let _ = baro.configure(&mut delay, &default).await;
            let (mut first_set, mut second_set, mut errs) = (0u32, 0u32, 0u32);
            for _ in 0..10 {
                Timer::after_millis(30).await;
                match (baro.interrupt_status().await, baro.interrupt_status().await) {
                    (Ok(a), Ok(b)) => {
                        if a.data_ready {
                            first_set += 1;
                        }
                        if b.data_ready {
                            second_set += 1;
                        }
                        if a.power_on_reset || b.power_on_reset {
                            defmt::error!("[STAT] 6: power-on flag seen {} {}", a, b);
                        }
                    }
                    _ => errs += 1,
                }
            }
            defmt::info!(
                "[STAT] 6: interrupt_status data_ready: first read set {=u32}/10, second read right after set {=u32}/10, errors {=u32}",
                first_set,
                second_set,
                errs
            );
            check!(
                fails,
                first_set == 10 && second_set <= 1 && errs == 0,
                "[{=str}] 6: data ready seen on the first read, cleared by it"
            );
        }
        Err(e) => {
            fails += 1;
            defmt::error!("[FAIL] 6: init failed {}", eh(&e));
        }
    }

    // ---- Check 7: verify during activity ----
    defmt::info!("[INFO] check 7: verify during streaming at 240 Hz");
    let fast = Config::new()
        .with_pressure_oversampling(Oversampling::X1)
        .with_temperature_oversampling(Oversampling::X1)
        .with_odr(OutputDataRate::Hz240);
    match baro.configure(&mut delay, &fast).await {
        Ok(_) => {
            Timer::after_millis(50).await;
            let mut not_ok = 0u32;
            let c0 = dr.count();
            let t0 = Instant::now();
            for _ in 0..200 {
                if baro.verify().await.is_err() {
                    not_ok += 1;
                }
            }
            let n = dr.count() - c0;
            let us = t0.elapsed().as_micros() as f32;
            let rate = n as f32 * 1.0e6 / us;
            defmt::info!(
                "[STAT] 7: 200 verify calls in {=f32} us ({=f32} us each), {=u64} edges, rate {=f32} Hz, not Ok {=u32}",
                us,
                us / 200.0,
                n,
                rate,
                not_ok
            );
            check!(
                fails,
                not_ok == 0,
                "[{=str}] 7: every verify returned Ok ({=u32} not Ok)",
                not_ok
            );
            check!(
                fails,
                rate > 240.0 * 0.88 && rate < 240.0 * 1.12,
                "[{=str}] 7: edge rate {=f32} Hz stays at the configured 240 Hz",
                rate
            );
            // Flag not swallowed: clear, wait for an edge, verify at once, then the flag must be set.
            let (mut set, mut tries) = (0u32, 0u32);
            for _ in 0..20 {
                let _ = baro.interrupt_status().await;
                let seen = dr.count();
                if dr.wait_beyond(seen, Duration::from_millis(100)).await.is_none() {
                    continue;
                }
                tries += 1;
                let v = baro.verify().await.is_ok();
                let st = baro.interrupt_status().await;
                if v && st.is_ok_and(|s| s.data_ready) {
                    set += 1;
                }
            }
            defmt::info!(
                "[STAT] 7: edge, verify, interrupt_status: data ready still set {=u32} of {=u32}",
                set,
                tries
            );
            check!(
                fails,
                tries >= 15 && set == tries,
                "[{=str}] 7: data-ready flag observable after verify ({=u32}/{=u32})",
                set,
                tries
            );
        }
        Err(e) => {
            fails += 1;
            defmt::error!("[FAIL] 7: configure failed {}", eh(&e));
        }
    }

    // ---- Check 6b: stats after everything on the main driver ----
    let st = baro.stats();
    defmt::info!("[STAT] 6b: stats after checks 1 to 7 {}", st);
    check!(
        fails,
        st.bus_errors == 0 && st.power_on_resets == 0,
        "[{=str}] 6b: no bus error ({=u32}) and no power-on reset counted ({=u32})",
        st.bus_errors,
        st.power_on_resets
    );

    // ---- Check 5b: recover on a fresh object after configure only ----
    defmt::info!("[INFO] check 5b: recover on a fresh driver, configure only");
    let bus = baro.release();
    let mut fresh: Baro = Bmp581::new(bus, bmp581::DEFAULT_ADDRESS);
    match fresh.configure(&mut delay, &default).await {
        Ok(_) => {
            let r = fresh.recover(&mut delay).await;
            if let Err(e) = &r {
                defmt::error!("[STAT] 5b: recover error {}", eh(e));
            }
            check!(
                fails,
                r.is_ok(),
                "[{=str}] 5b: recover Ok on a fresh object with only configure ({=bool})",
                r.is_ok()
            );
        }
        Err(e) => {
            fails += 1;
            defmt::error!("[FAIL] 5b: configure failed {}", eh(&e));
        }
    }

    // ---- Final: full init to default, verify Ok ----
    let bus = fresh.release();
    let mut baro: Baro = Bmp581::new(bus, bmp581::DEFAULT_ADDRESS);
    match baro.init(&mut delay, &default).await {
        Ok(_) => {
            let v = ver(&mut baro, "final").await;
            check!(fails, v, "[{=str}] final: init to default, verify Ok ({=bool})", v);
        }
        Err(e) => {
            fails += 1;
            defmt::error!("[FAIL] final: init failed {}", eh(&e));
        }
    }
    defmt::info!(
        "[INFO] run took {=u32} ms",
        t_start.elapsed().as_millis() as u32
    );
    hil_common::exit(fails)
}
