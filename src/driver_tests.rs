//! Driver tests over the fake chip of `testbus`: the bus bytes, the validity state (contract A3), failures
//! at every transaction (D3), drops at every await (A2, D4), the chip's own validity flag, the blocking
//! front-end (A11) and `Send` (A5).

use crate::testbus::{BusFault, Polled, Shared, TestBus, TestDelay, d, poll_then_drop, r, run, w};
use crate::*;

const ADDR: u8 = DEFAULT_ADDRESS;
type Drv = Bmp581<TestBus>;

fn rig() -> (Shared, Drv, TestDelay) {
    let s = Shared::new();
    let b = Bmp581::new(s.bus(), ADDR);
    let dl = s.delay();
    (s, b, dl)
}

#[derive(Clone, Copy)]
enum Method {
    Init,
    Configure,
}

async fn call(
    b: &mut Drv,
    dl: &mut TestDelay,
    m: Method,
    c: &Config,
) -> Result<Info, Error<BusFault>> {
    match m {
        Method::Init => b.init(dl, c).await,
        Method::Configure => b.configure(dl, c).await,
    }
}

fn sample() -> RawSample {
    RawSample::decode(&testbus::DATA)
}

/// After a failed or dropped call: not configured, `read` says so without touching the bus, and a
/// complete call then succeeds and configures.
fn assert_recovers(s: &Shared, b: &mut Drv, dl: &mut TestDelay, m: Method, c: &Config, at: &str) {
    assert!(b.config().is_none(), "{at}");
    s.arm(None, None);
    assert_eq!(run(b.read()), Err(Error::NotConfigured), "{at}");
    assert_eq!(s.transactions(), 0, "{at}");
    run(call(b, dl, m, c)).unwrap();
    assert_eq!(b.config(), Some(c), "{at}");
    assert_eq!(run(b.read()), Ok(sample()), "{at}");
}

fn counts(m: Method, c: &Config) -> (usize, u32) {
    let (s, mut b, mut dl) = rig();
    s.arm(None, None);
    run(call(&mut b, &mut dl, m, c)).unwrap();
    (s.transactions(), s.awaits())
}

fn sweep_failures(m: Method, c: &Config) {
    let (n, _) = counts(m, c);
    for configured_before in [false, true] {
        for k in 1..=n {
            let (s, mut b, mut dl) = rig();
            if configured_before {
                run(b.init(&mut dl, &Config::default())).unwrap();
            }
            s.arm(Some(k), None);
            let res = run(call(&mut b, &mut dl, m, c));
            assert_eq!(res, Err(Error::Bus(BusFault::Injected)), "k {k}");
            assert_recovers(&s, &mut b, &mut dl, m, c, &std::format!("fail {k}"));
        }
    }
}

fn sweep_drops(m: Method, c: &Config) {
    let (_, n) = counts(m, c);
    for configured_before in [false, true] {
        for k in 1..=n {
            let (s, mut b, mut dl) = rig();
            if configured_before {
                run(b.init(&mut dl, &Config::default())).unwrap();
            }
            s.arm(None, Some(k));
            assert_eq!(
                poll_then_drop(call(&mut b, &mut dl, m, c)),
                Polled::Cancelled,
                "await {k}"
            );
            assert_recovers(&s, &mut b, &mut dl, m, c, &std::format!("drop {k}"));
        }
        let (s, mut b, mut dl) = rig();
        s.arm(None, Some(n + 1));
        assert!(matches!(
            poll_then_drop(call(&mut b, &mut dl, m, c)),
            Polled::Done(Ok(_))
        ));
    }
}

#[test]
fn new_starts_not_configured_without_bus_access() {
    let (s, b, _) = rig();
    assert_eq!(b.config(), None);
    assert_eq!(s.transactions(), 0);
}

#[test]
fn config_is_the_applied_configuration_once_configured() {
    let (_, mut b, mut dl) = rig();
    assert_eq!(b.config(), None);
    let c = Config::default().with_odr(OutputDataRate::Hz25_005);
    run(b.init(&mut dl, &c)).unwrap();
    assert_eq!(b.config(), Some(&c));
}

#[test]
fn release_returns_the_bus() {
    let (s, b, _) = rig();
    let mut bus = b.release();
    s.arm(None, None);
    let mut buf = [0u8; 1];
    embedded_hal::i2c::I2c::write_read(&mut bus, ADDR, &[reg::CHIP_ID], &mut buf).unwrap();
    assert_eq!(buf, [0x50]);
}

#[test]
fn init_golden_trace_for_default_config() {
    // Datasheet pages: reset command 7.36 p67 and 4.3.10 p18 (not acknowledged), wait 2 ms at most
    // (Table 4, p11); CHIP_ID 7.1 and REV_ID 7.2 p51; STATUS 7.22 p58; INT_STATUS 7.21 p58 (0x10 = power-on
    // flag, 4.3.9 p18); OSR_CONFIG 7.33 p64 (press_en, osr_p 16x, osr_t 2x = 0x61); ODR_CONFIG 7.34
    // p64-65 (50.056 Hz = code 0x0F, standby 0x3C, normal 0x3D); DSP_CONFIG 7.27 p61 (0x30); DSP_IIR
    // 7.28 p62 (0x31); INT_SOURCE 7.6 p54 (0x15, data ready); INT_CONFIG 7.5 p52-53 (0x14, pulsed, push-pull,
    // active high, enabled); OSR_EFF 7.35 p66 (0x38, read in standby, after the standby rate write and before
    // the mode write: valid, osr_p 16x, osr_t 2x = 0xA1).
    let (s, mut b, mut dl) = rig();
    s.arm(None, None);
    let info = run(b.init(&mut dl, &Config::default())).unwrap();
    assert_eq!(
        s.log(),
        [
            w(0x7E, 0xB6),
            d(3_000),
            r(0x01, &[0x50, 0x32]),
            r(0x28, &[0x02]),
            r(0x27, &[0x10]),
            w(0x36, 0x61),
            w(0x37, 0x3C),
            r(0x38, &[0xA1]),
            w(0x30, 0x03),
            w(0x31, 0x00),
            w(0x15, 0x01),
            w(0x14, 0x3A),
            r(0x27, &[0x00]),
            w(0x37, 0x3D),
        ]
    );
    assert_eq!((s.transactions(), s.awaits(), s.violations()), (13, 14, 0));
    let want = Info {
        chip_id: 0x50,
        revision: 0x32,
        pressure_oversampling: Oversampling::X16,
        temperature_oversampling: Oversampling::X2,
        output_period_us: 19_978,
        conversion_time_us: 11_500,
    };
    assert_eq!(info, want);
    assert_eq!(b.config(), Some(&Config::default()));
}

#[test]
fn read_is_one_transaction_of_six_bytes() {
    // Data registers 0x1D to 0x22 (7.11 to 7.16, p56-57); contract E1 and A4e.
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.arm(None, None);
    assert_eq!(run(b.read()), Ok(sample()));
    assert_eq!(s.log(), [r(0x1D, &testbus::DATA)]);
    assert_eq!((s.transactions(), s.awaits(), s.violations()), (1, 1, 0));
    assert_eq!(sample().celsius(), 25.0);
    assert_eq!(sample().pascals(), 27_136.0);
}

fn put_data(s: &Shared, bytes: [u8; 6]) {
    let at = usize::from(reg::TEMP_DATA_XLSB);
    s.chip().regs[at..at + 6].copy_from_slice(&bytes);
}

const MARKER: [u8; 3] = [0x7F, 0x7F, 0x7F];
const TEMP_25: [u8; 3] = [0x00, 0x00, 0x19];
const PRESS_27136: [u8; 3] = [0x00, 0x80, 0x1A];

fn bytes(t: [u8; 3], p: [u8; 3]) -> [u8; 6] {
    let [t0, t1, t2] = t;
    let [p0, p1, p2] = p;
    [t0, t1, t2, p0, p1, p2]
}

#[test]
fn read_returns_no_sample_for_the_reset_value_in_either_half_or_both() {
    // The data registers reset to 0x7F in every byte (7.11 to 7.16, p56-57); measured on the board,
    // 2026-11-10: a read before the first conversion returned 130557.98 Pa with Ok.
    assert_eq!(0x7F_7F7Fu32 as f32 / 64.0, 130_557.984);
    for (t, p) in [(MARKER, PRESS_27136), (TEMP_25, MARKER), (MARKER, MARKER)] {
        let (s, mut b, mut dl) = rig();
        run(b.init(&mut dl, &Config::default())).unwrap();
        put_data(&s, bytes(t, p));
        s.arm(None, None);
        assert_eq!(run(b.read()), Err(Error::NoSample), "{t:?} {p:?}");
        assert_eq!(s.log(), [r(0x1D, &bytes(t, p))]);
        assert_eq!((s.transactions(), s.awaits(), s.violations()), (1, 1, 0));
        assert!(b.config().is_some());
        assert_eq!(b.stats().bus_errors, 0);
        // The marker goes away with the first conversion; the same driver then reads normally.
        put_data(&s, testbus::DATA);
        assert_eq!(run(b.read()), Ok(sample()));
    }
}

#[test]
fn read_returns_a_value_one_lsb_from_the_marker() {
    for neighbour in [
        [0x7E, 0x7F, 0x7F],
        [0x80, 0x7F, 0x7F],
        [0x7F, 0x7F, 0x7E],
        [0x7F, 0x7F, 0x80],
    ] {
        for (t, p) in [
            (neighbour, PRESS_27136),
            (TEMP_25, neighbour),
            (neighbour, neighbour),
        ] {
            let (s, mut b, mut dl) = rig();
            run(b.init(&mut dl, &Config::default())).unwrap();
            put_data(&s, bytes(t, p));
            s.arm(None, None);
            assert_eq!(run(b.read()), Ok(RawSample::decode(&bytes(t, p))));
            assert_eq!(s.transactions(), 1);
        }
    }
    // Neighbours as raw counts: 0x7F7F7E and 0x7F7F80.
    let low = RawSample::decode(&bytes([0x7E, 0x7F, 0x7F], [0x80, 0x7F, 0x7F]));
    assert_eq!((low.temperature, low.pressure), (0x7F_7F7E, 0x7F_7F80));
    assert!(!low.is_reset_value());
}

#[test]
fn read_with_pressure_off_judges_only_the_temperature() {
    let c = Config::default().with_pressure_enabled(false);
    for (t, p, want) in [
        (TEMP_25, MARKER, Ok(())),
        (MARKER, PRESS_27136, Err(Error::NoSample)),
        (MARKER, MARKER, Err(Error::NoSample)),
        (TEMP_25, PRESS_27136, Ok(())),
    ] {
        let (s, mut b, mut dl) = rig();
        run(b.init(&mut dl, &c)).unwrap();
        put_data(&s, bytes(t, p));
        s.arm(None, None);
        assert_eq!(run(b.read()).map(|_| ()), want, "{t:?} {p:?}");
        assert_eq!(s.transactions(), 1);
    }
    // After configure back to pressure on, the pressure half is judged again.
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &c)).unwrap();
    run(b.configure(&mut dl, &Config::default())).unwrap();
    put_data(&s, bytes(TEMP_25, MARKER));
    assert_eq!(run(b.read()), Err(Error::NoSample));
}

#[test]
fn reset_value_predicates_are_pure_and_judge_each_half() {
    let both = RawSample::decode(&bytes(MARKER, MARKER));
    let t = RawSample::decode(&bytes(MARKER, PRESS_27136));
    let p = RawSample::decode(&bytes(TEMP_25, MARKER));
    assert!(
        both.is_reset_value()
            && both.temperature_is_reset_value()
            && both.pressure_is_reset_value()
    );
    assert!(t.is_reset_value() && t.temperature_is_reset_value() && !t.pressure_is_reset_value());
    assert!(p.is_reset_value() && !p.temperature_is_reset_value() && p.pressure_is_reset_value());
    assert!(!sample().is_reset_value());
}

#[test]
fn blocking_read_returns_no_sample_too() {
    let s = Shared::new();
    let mut bb = blocking::Bmp581::new(s.bus(), ADDR);
    let mut dl = s.delay();
    bb.init(&mut dl, &Config::default()).unwrap();
    put_data(&s, bytes(MARKER, MARKER));
    assert_eq!(bb.read(), Err(Error::NoSample));
}

#[test]
fn read_failure_and_drop_keep_the_driver_configured() {
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.arm(Some(1), None);
    assert_eq!(run(b.read()), Err(Error::Bus(BusFault::Injected)));
    assert!(b.config().is_some());
    s.arm(None, Some(1));
    assert_eq!(poll_then_drop(b.read()), Polled::Cancelled);
    assert!(b.config().is_some());
    s.arm(None, None);
    assert_eq!(run(b.read()), Ok(sample()));
}

#[test]
fn read_before_configuration_is_not_configured() {
    let (s, mut b, _) = rig();
    s.arm(None, None);
    assert_eq!(run(b.read()), Err(Error::NotConfigured));
    assert_eq!(run(b.interrupt_status()), Err(Error::NotConfigured));
    assert_eq!(run(b.trigger_forced()), Err(Error::NotConfigured));
    assert_eq!(s.transactions(), 0);
}

#[test]
fn init_failure_at_every_transaction_leaves_not_configured() {
    sweep_failures(Method::Init, &Config::default());
}

#[test]
fn init_drop_at_every_await_leaves_not_configured() {
    sweep_drops(Method::Init, &Config::default());
    sweep_drops(
        Method::Init,
        &Config::default().with_power_mode(PowerMode::Forced),
    );
    sweep_drops(
        Method::Init,
        &Config::default().with_power_mode(PowerMode::Continuous),
    );
}

#[test]
fn init_wrong_id_nvm_errors_and_missing_chip() {
    let (s, mut b, mut dl) = rig();
    s.chip().chip_id = 0x51;
    assert_eq!(
        run(b.init(&mut dl, &Config::default())),
        Err(Error::WrongId { found: 0x51 })
    );
    assert_eq!(run(b.read()), Err(Error::NotConfigured));

    let (s, mut b, mut dl) = rig();
    s.chip().status = 0;
    assert_eq!(
        run(b.init(&mut dl, &Config::default())),
        Err(Error::NvmNotReady)
    );
    s.chip().status = reg::STATUS_NVM_RDY | reg::STATUS_NVM_ERR;
    assert_eq!(
        run(b.init(&mut dl, &Config::default())),
        Err(Error::NvmError)
    );
    assert!(b.config().is_none());

    // The other address is not acknowledged by the fake: the reset is tolerated, the identity read is not.
    let s = Shared::new();
    let mut b = Bmp581::new(s.bus(), ALTERNATE_ADDRESS);
    let mut dl = s.delay();
    assert_eq!(
        run(b.init(&mut dl, &Config::default())),
        Err(Error::Bus(BusFault::NoAck))
    );
}

#[test]
fn init_invalid_config_touches_nothing_and_clears_the_state() {
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.arm(None, None);
    let bad = Config::new().with_odr(OutputDataRate::Hz240);
    let res = run(b.init(&mut dl, &bad));
    assert!(matches!(
        res,
        Err(Error::InvalidConfig(
            ConfigError::OdrTooHighForOversampling { .. }
        ))
    ));
    assert_eq!(s.transactions(), 0);
    assert_eq!(b.config(), None);
    assert_eq!(run(b.read()), Err(Error::NotConfigured));
    let res = run(b.configure(&mut dl, &bad));
    assert!(matches!(res, Err(Error::InvalidConfig(_))));
    assert_eq!(s.transactions(), 0);
}

#[test]
fn configure_golden_trace_for_default_config() {
    // The in-place order of 4.3.7 (p18): standby, wait tstandby (2.5 ms, Table 4 p11), every register
    // written whole (FIFO_SEL 0x18 and FIFO_CONFIG 0x16 7.9 and 7.7 p54-55, OOR_THR_P 0x32 0x33 and
    // OOR_RANGE 0x34 and OOR_CONFIG 0x35 7.29 to 7.32 p62-63), then 4.7.3.2 (p28): sources off, read
    // INT_STATUS, pin, sources. No reset, no identity read. OSR_EFF 7.35 p66, read in standby before the
    // filters are written and the mode is set.
    let (s, mut b, mut dl) = rig();
    s.arm(None, None);
    let info = run(b.configure(&mut dl, &Config::default())).unwrap();
    assert_eq!(
        s.log(),
        [
            w(0x37, 0x3C),
            d(2_500),
            w(0x36, 0x61),
            w(0x37, 0x3C),
            r(0x38, &[0xA1]),
            w(0x30, 0x03),
            w(0x31, 0x00),
            w(0x18, 0x00),
            w(0x16, 0x00),
            w(0x32, 0x00),
            w(0x33, 0x00),
            w(0x34, 0x00),
            w(0x35, 0x00),
            w(0x15, 0x00),
            r(0x27, &[0x00]),
            w(0x14, 0x3A),
            w(0x15, 0x01),
            r(0x27, &[0x00]),
            w(0x37, 0x3D),
        ]
    );
    assert_eq!((s.transactions(), s.awaits(), s.violations()), (18, 19, 0));
    assert_eq!((info.chip_id, info.revision), (0, 0));
    assert_eq!(info.pressure_oversampling, Oversampling::X16);
    assert_eq!(info.temperature_oversampling, Oversampling::X2);
    assert_eq!(b.config(), Some(&Config::default()));
    s.arm(None, None);
    assert_eq!(run(b.read()), Ok(sample()));
    assert_eq!((s.transactions(), s.awaits()), (1, 1));
}

#[test]
fn configure_failure_at_every_transaction_leaves_not_configured() {
    sweep_failures(Method::Configure, &Config::default());
    sweep_failures(
        Method::Configure,
        &Config::default().with_power_mode(PowerMode::Forced),
    );
}

#[test]
fn configure_drop_at_every_await_leaves_not_configured() {
    sweep_drops(Method::Configure, &Config::default());
    sweep_drops(
        Method::Configure,
        &Config::default().with_power_mode(PowerMode::Forced),
    );
    sweep_drops(
        Method::Configure,
        &Config::default().with_power_mode(PowerMode::Continuous),
    );
}

#[test]
fn configure_replaces_the_applied_configuration() {
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    let c = Config::default()
        .with_power_mode(PowerMode::Forced)
        .with_odr(OutputDataRate::Hz25_005);
    let info = run(b.configure(&mut dl, &c)).unwrap();
    assert_eq!(b.config(), Some(&c));
    assert_eq!(info.output_period_us, 0);
    // Forced mode: the chip stays in standby (the plan has no mode write) and echoes the request.
    assert_eq!(s.reg(reg::ODR_CONFIG) & 0b11, 0);
}

#[test]
fn trigger_forced_writes_the_forced_mode_with_the_applied_rate() {
    let (s, mut b, mut dl) = rig();
    let c = Config::default().with_power_mode(PowerMode::Forced);
    run(b.init(&mut dl, &c)).unwrap();
    s.arm(None, None);
    assert_eq!(run(b.trigger_forced()), Ok(()));
    assert_eq!(s.log(), [w(0x37, 0x3E)]);
    assert_eq!(s.awaits(), 1);
}

#[test]
fn interrupt_status_reads_and_clears_int_status() {
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.chip().regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_DRDY;
    s.arm(None, None);
    let st = run(b.interrupt_status()).unwrap();
    assert!(st.data_ready && !st.power_on_reset);
    assert_eq!(s.log(), [r(0x27, &[0x01])]);
    assert_eq!(s.reg(reg::INT_STATUS), 0);
}

#[test]
fn read_register_and_read_registers_are_one_write_read() {
    // Contract A4e: ONE write_read(address, &[register], buf), whatever the length.
    let (s, mut b, _) = rig();
    s.arm(None, None);
    assert_eq!(run(b.read_register(reg::CHIP_ID)), Ok(0x50));
    let mut two = [0u8; 2];
    run(b.read_registers(reg::CHIP_ID, &mut two)).unwrap();
    assert_eq!(two, [0x50, 0x32]);
    assert_eq!(s.log(), [r(0x01, &[0x50]), r(0x01, &[0x50, 0x32])]);
    assert_eq!((s.transactions(), s.violations()), (2, 0));
}

#[test]
fn write_register_is_one_write_outside_the_validity_state() {
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.arm(None, None);
    assert_eq!(run(b.write_register(reg::DSP_IIR, 0x12)), Ok(()));
    assert_eq!(s.log(), [w(0x31, 0x12)]);
    assert!(b.config().is_some());
}

fn refuse_16_2(_pressure: bool, osr_p: u8, osr_t: u8, _odr: u8) -> bool {
    !(osr_p == 4 && osr_t == 1)
}

fn refuse_all(_pressure: bool, _osr_p: u8, _osr_t: u8, _odr: u8) -> bool {
    false
}

/// The default configuration against a chip that refuses 16x/2x at every rate.
fn rejecting_rig(configured_before: bool) -> (Shared, Drv, TestDelay) {
    let (s, mut b, mut dl) = rig();
    if configured_before {
        run(b.init(&mut dl, &Config::default())).unwrap();
    }
    s.chip().rule = refuse_16_2;
    (s, b, dl)
}

fn standby_rate() -> u8 {
    Config::default().odr_config(PowerMode::Standby)
}

#[test]
fn init_chip_rejects_the_pair_in_standby_and_stays_not_configured() {
    // V3: in standby the effective oversampling echoes the request (osr_p 16x, osr_t 2x = 0x21); only the
    // flag tells. The check sits before the mode write: the last transaction is the OSR_EFF read and the
    // rate register still holds standby.
    let (s, mut b, mut dl) = rig();
    s.chip().rule = refuse_16_2;
    s.arm(None, None);
    assert_eq!(
        run(b.init(&mut dl, &Config::default())),
        Err(Error::ChipRejectsRate { osr_eff: 0x21 })
    );
    assert_eq!(
        s.log().last(),
        Some(&r(0x38, &[0x21])),
        "no transaction follows the OSR_EFF read"
    );
    assert_eq!((s.transactions(), s.awaits(), s.violations()), (7, 8, 0));
    assert_eq!(s.reg(reg::ODR_CONFIG), standby_rate());
    assert_eq!(s.reg(reg::OSR_CONFIG), 0x61);
    assert_eq!(b.config(), None);
    s.arm(None, None);
    assert_eq!(run(b.read()), Err(Error::NotConfigured));
    assert_eq!(s.transactions(), 0);
    assert_eq!(run(b.read_register(reg::OSR_EFF)), Ok(0x21));
}

#[test]
fn configure_chip_rejects_the_pair_in_standby_and_stays_not_configured() {
    // The chip runs in normal mode at 25 Hz; configure puts it in standby first and leaves it there.
    let (s, mut b, mut dl) = rig();
    run(b.init(
        &mut dl,
        &Config::default().with_odr(OutputDataRate::Hz25_005),
    ))
    .unwrap();
    s.chip().rule = refuse_16_2;
    s.arm(None, None);
    assert_eq!(
        run(b.configure(&mut dl, &Config::default())),
        Err(Error::ChipRejectsRate { osr_eff: 0x21 })
    );
    assert_eq!(s.log().last(), Some(&r(0x38, &[0x21])));
    assert_eq!((s.transactions(), s.awaits(), s.violations()), (4, 5, 0));
    assert_eq!(s.reg(reg::ODR_CONFIG), standby_rate());
    assert_eq!(b.config(), None);
    assert_eq!(run(b.read()), Err(Error::NotConfigured));
    // The pair the chip accepts: configure again.
    s.chip().rule = testbus::nominal_table;
    run(b.configure(&mut dl, &Config::default())).unwrap();
    assert_eq!(b.config(), Some(&Config::default()));
    assert_eq!(s.reg(reg::ODR_CONFIG), standby_rate() | 1);
    // At 160 Hz and above the echo of the request is 1x/1x: the byte is 0x00 with the flag clear.
    let c = Config::new()
        .with_pressure_oversampling(Oversampling::X1)
        .with_temperature_oversampling(Oversampling::X1)
        .with_odr(OutputDataRate::Hz240);
    s.chip().rule = refuse_all;
    assert_eq!(
        run(b.configure(&mut dl, &c)),
        Err(Error::ChipRejectsRate { osr_eff: 0x00 })
    );
    assert_eq!(s.reg(reg::ODR_CONFIG), c.odr_config(PowerMode::Standby));
}

#[test]
fn rejection_failure_at_every_transaction_leaves_not_configured() {
    for m in [Method::Init, Method::Configure] {
        for configured_before in [false, true] {
            let (s, mut b, mut dl) = rejecting_rig(configured_before);
            s.arm(None, None);
            let c = Config::default();
            assert_eq!(
                run(call(&mut b, &mut dl, m, &c)),
                Err(Error::ChipRejectsRate { osr_eff: 0x21 })
            );
            let n = s.transactions();
            for k in 1..=n {
                let (s, mut b, mut dl) = rejecting_rig(configured_before);
                s.arm(Some(k), None);
                let res = run(call(&mut b, &mut dl, m, &c));
                assert_eq!(res, Err(Error::Bus(BusFault::Injected)), "k {k}");
                s.chip().rule = testbus::nominal_table;
                assert_recovers(&s, &mut b, &mut dl, m, &c, &std::format!("fail {k}"));
            }
        }
    }
}

#[test]
fn rejection_drop_at_every_await_leaves_not_configured() {
    for m in [Method::Init, Method::Configure] {
        for configured_before in [false, true] {
            let (s, mut b, mut dl) = rejecting_rig(configured_before);
            s.arm(None, None);
            let c = Config::default();
            assert_eq!(
                run(call(&mut b, &mut dl, m, &c)),
                Err(Error::ChipRejectsRate { osr_eff: 0x21 })
            );
            let n = s.awaits();
            for k in 1..=n {
                let (s, mut b, mut dl) = rejecting_rig(configured_before);
                s.arm(None, Some(k));
                assert_eq!(
                    poll_then_drop(call(&mut b, &mut dl, m, &c)),
                    Polled::Cancelled,
                    "await {k}"
                );
                s.chip().rule = testbus::nominal_table;
                assert_recovers(&s, &mut b, &mut dl, m, &c, &std::format!("drop {k}"));
            }
            let (s, mut b, mut dl) = rejecting_rig(configured_before);
            s.arm(None, Some(n + 1));
            assert!(matches!(
                poll_then_drop(call(&mut b, &mut dl, m, &c)),
                Polled::Done(Err(Error::ChipRejectsRate { osr_eff: 0x21 }))
            ));
            assert_eq!(s.reg(reg::ODR_CONFIG), standby_rate());
        }
    }
}

#[test]
fn outside_normal_mode_the_flag_is_not_read_and_info_reports_the_applied_oversampling() {
    // A chip that refuses every pair: the flag is not acted on, and OSR_EFF is not read at all.
    for mode in [PowerMode::Standby, PowerMode::Forced, PowerMode::Continuous] {
        let (s, mut b, mut dl) = rig();
        s.chip().rule = refuse_all;
        let c = Config::default().with_power_mode(mode);
        let info = run(b.init(&mut dl, &c)).unwrap();
        assert_eq!(info.output_period_us, 0);
        assert_eq!(
            (info.pressure_oversampling, info.temperature_oversampling),
            (Oversampling::X16, Oversampling::X2),
            "{mode:?}"
        );
        assert_eq!(b.config(), Some(&c));
        let reads_osr_eff = s.log().iter().any(|e| {
            matches!(e, testbus::Event::I2c { ops, .. }
                if ops.first() == Some(&testbus::Op::Write(std::vec![reg::OSR_EFF])))
        });
        assert!(!reads_osr_eff, "{mode:?}");
    }
    // The way to ask the chip stays: continuous shows the substitution 4x/4x.
    let (s, mut b, mut dl) = rig();
    s.chip().rule = refuse_all;
    run(b.init(
        &mut dl,
        &Config::default().with_power_mode(PowerMode::Continuous),
    ))
    .unwrap();
    let e = run(b.effective_oversampling()).unwrap();
    assert_eq!(
        (e.valid, e.pressure, e.temperature),
        (false, Oversampling::X4, Oversampling::X4)
    );
}

#[test]
fn info_conversion_time_follows_the_effective_oversampling_and_pressure_off() {
    let (_, mut b, mut dl) = rig();
    let c = Config::new()
        .with_pressure_enabled(false)
        .with_temperature_oversampling(Oversampling::X8)
        .with_odr(OutputDataRate::Hz100_299);
    let info = run(b.init(&mut dl, &c)).unwrap();
    assert_eq!(info.conversion_time_us, 2_100);
    assert_eq!(info.output_period_us, 9_970);
    assert_eq!(info.temperature_oversampling, Oversampling::X8);
}

#[test]
fn fake_chip_follows_the_measured_validity_behaviour() {
    // V1 and V3 (measured on the board, 2026-11-10): evaluated at the write, flag clear when invalid, in
    // standby the oversampling echoes the request, outside standby it is replaced.
    let (s, mut b, _) = rig();
    s.chip().rule = refuse_all;
    let osr = Config::default().osr_config();
    run(b.write_register(reg::OSR_CONFIG, osr)).unwrap();
    run(b.write_register(reg::ODR_CONFIG, 0)).unwrap();
    assert_eq!(run(b.read_register(reg::OSR_EFF)), Ok(0x21));
    run(b.write_register(reg::ODR_CONFIG, (4 << 2) | 1)).unwrap();
    assert_eq!(run(b.read_register(reg::OSR_EFF)), Ok(0x09));
    run(b.write_register(reg::ODR_CONFIG, (5 << 2) | 1)).unwrap();
    assert_eq!(run(b.read_register(reg::OSR_EFF)), Ok(0x12));
    s.chip().rule = testbus::nominal_table;
    run(b.write_register(reg::ODR_CONFIG, (15 << 2) | 1)).unwrap();
    assert_eq!(run(b.read_register(reg::OSR_EFF)), Ok(0xA1));
}

#[test]
fn blocking_parity_traces_and_results_are_identical() {
    let c = Config::default();
    let forced = Config::default().with_power_mode(PowerMode::Forced);

    let (s, mut b, mut dl) = rig();
    s.arm(None, None);
    let a_init = run(b.init(&mut dl, &c));
    let a_read = run(b.read());
    let a_cfg = run(b.configure(&mut dl, &forced));
    let a_trig = run(b.trigger_forced());
    let a_status = run(b.interrupt_status());
    let a_log = s.log();

    let s = Shared::new();
    let mut bb = blocking::Bmp581::new(s.bus(), ADDR);
    let mut dl = s.delay();
    s.arm(None, None);
    let b_init = bb.init(&mut dl, &c);
    assert_eq!(bb.config(), Some(&c));
    let b_read = bb.read();
    let b_cfg = bb.configure(&mut dl, &forced);
    let b_trig = bb.trigger_forced();
    let b_status = bb.interrupt_status();
    assert_eq!(bb.config(), Some(&forced));
    assert_eq!(
        (a_init, a_read, a_cfg, a_trig, a_status),
        (b_init, b_read, b_cfg, b_trig, b_status)
    );
    assert_eq!(a_log, s.log());
    assert!(a_log.len() > 30);
    // Not configured: the blocking front-end says so too.
    let s = Shared::new();
    let mut bb = blocking::Bmp581::new(s.bus(), ADDR);
    assert_eq!(bb.read(), Err(Error::NotConfigured));
    assert_eq!(bb.config(), None);
}

fn assert_send<T: Send>(_: &T) {}

#[test]
fn futures_and_driver_are_send() {
    let (_, mut b, mut dl) = rig();
    let c = Config::default();
    assert_send(&b);
    assert_send(&b.init(&mut dl, &c));
    assert_send(&b.configure(&mut dl, &c));
    assert_send(&b.read());
    assert_send(&b.trigger_forced());
    assert_send(&b.interrupt_status());
    assert_send(&b.read_register(0));
    assert_send(&b.write_register(0, 0));
    assert_send(&b.read_registers(0, &mut []));
    assert_send(&b.fifo_count());
    assert_send(&b.fifo_read(&mut []));
    assert_send(&b.fifo_flush(&mut dl));
    assert_send(&fifo_frames(&[], FifoData::Pressure));
}
