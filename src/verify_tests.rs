//! Tests of `verify`, `recover`, the status reads and the counters, over the fake chip of `testbus`.

use crate::testbus::{BusFault, Polled, Shared, TestBus, TestDelay, d, poll_then_drop, r, run};
use crate::*;

const ADDR: u8 = DEFAULT_ADDRESS;
type Drv = Bmp581<TestBus>;
type Res<T> = Result<T, Error<BusFault>>;

fn rig() -> (Shared, Drv, TestDelay) {
    let s = Shared::new();
    let b = Bmp581::new(s.bus(), ADDR);
    let dl = s.delay();
    (s, b, dl)
}

fn up(c: &Config) -> (Shared, Drv, TestDelay) {
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, c)).unwrap();
    s.arm(None, None);
    (s, b, dl)
}

fn refuse_all(_pressure: bool, _osr_p: u8, _osr_t: u8, _odr: u8) -> bool {
    false
}

fn unlike_configs() -> [Config; 7] {
    [
        Config::default(),
        Config::default()
            .with_power_mode(PowerMode::Forced)
            .with_odr(OutputDataRate::Hz1)
            .with_pressure_oversampling(Oversampling::X1)
            .with_temperature_oversampling(Oversampling::X1)
            .with_pressure_iir(IirFilter::Coeff15)
            .with_temperature_iir(IirFilter::Coeff3)
            .with_fifo(
                FifoConfig::new()
                    .with_mode(FifoMode::Stream)
                    .with_data(FifoData::PressureAndTemperature)
                    .with_threshold(3)
                    .with_decimation(FifoDecimation::By4),
            )
            .with_interrupt(
                InterruptPin::new()
                    .with_sources(
                        InterruptSources::new()
                            .with_fifo_full(true)
                            .with_fifo_threshold(true),
                    )
                    .with_mode(InterruptMode::Latched)
                    .with_polarity(InterruptPolarity::ActiveLow)
                    .with_drive(InterruptDrive::OpenDrain),
            ),
        Config::default()
            .with_power_mode(PowerMode::Standby)
            .with_out_of_range(Some(
                OutOfRange::new()
                    .with_reference_pa(100_000)
                    .with_range_pa(30)
                    .with_count(OutOfRangeCount::Fifteen),
            ))
            .with_interrupt(
                InterruptPin::new().with_sources(InterruptSources::new().with_out_of_range(true)),
            ),
        Config::default()
            .with_power_mode(PowerMode::Continuous)
            .with_odr(OutputDataRate::Hz1)
            .with_pressure_enabled(false)
            .with_temperature_oversampling(Oversampling::X4)
            .with_fifo(
                FifoConfig::new()
                    .with_mode(FifoMode::StopOnFull)
                    .with_data(FifoData::Temperature)
                    .with_threshold(5),
            ),
        Config::default()
            .with_odr(OutputDataRate::Hz10)
            .with_pressure_iir(IirFilter::Coeff63)
            .with_temperature_iir(IirFilter::Coeff7)
            .with_iir_taps(
                IirTaps::new()
                    .with_data_pressure(IirTap::Unfiltered)
                    .with_fifo_temperature(IirTap::Filtered)
                    .with_out_of_range_pressure(IirTap::Filtered),
            )
            .with_iir_flush_forced(true),
        Config::default()
            .with_odr(OutputDataRate::Hz25_005)
            .with_pressure_enabled(false)
            .with_temperature_oversampling(Oversampling::X1),
        Config::default()
            .with_odr(OutputDataRate::Hz5)
            .with_pressure_oversampling(Oversampling::X64)
            .with_temperature_oversampling(Oversampling::X8),
    ]
}

fn owned_regs(c: &Config) -> std::vec::Vec<u8> {
    c.owned_registers()
        .entries()
        .iter()
        .map(|o| o.reg)
        .collect()
}

fn mismatch(register: u8, mask: u8, expected: u8, found: u8) -> Res<()> {
    Err(Error::ConfigMismatch(Mismatch {
        register,
        mask,
        expected,
        found,
    }))
}

#[test]
fn verify_owned_entries_for_the_default_config_follow_the_images() {
    // Independent literals. Masks: INT_CONFIG 7.5 p52-53, INT_SOURCE 7.6 p54, FIFO_CONFIG 7.7 p54,
    // FIFO_SEL 7.9 p55, DSP_CONFIG 7.27 p61, DSP_IIR 7.28 p62, OOR 7.29 to 7.32 p63, OSR_CONFIG 7.33 p64,
    // ODR_CONFIG 7.34 p64-65, OSR_EFF 7.35 p66.
    let got: std::vec::Vec<(u8, u8, u8)> = Config::default()
        .owned_registers()
        .entries()
        .iter()
        .map(|o| (o.reg, o.mask, o.expected))
        .collect();
    assert_eq!(
        got,
        std::vec![
            (0x14, 0xFF, 0x3A),
            (0x15, 0x0F, 0x01),
            (0x16, 0x3F, 0x00),
            (0x18, 0x1F, 0x00),
            (0x30, 0xFF, 0x03),
            (0x31, 0x3F, 0x00),
            (0x32, 0xFF, 0x00),
            (0x33, 0xFF, 0x00),
            (0x34, 0xFF, 0x00),
            (0x35, 0xC1, 0x00),
            (0x36, 0x7F, 0x61),
            (0x37, 0xFF, 0x3D),
            (0x38, 0xBF, 0xA1),
        ]
    );
}

#[test]
fn verify_power_mode_field_is_owned_except_in_forced_mode_and_the_validity_entry_only_in_normal() {
    for (mode, odr_mask, len) in [
        (PowerMode::Standby, 0xFF, 12),
        (PowerMode::Normal, 0xFF, 13),
        (PowerMode::Forced, 0xFC, 12),
        (PowerMode::Continuous, 0xFF, 12),
    ] {
        let list = Config::default().with_power_mode(mode).owned_registers();
        let odr = list.entries().iter().find(|o| o.reg == 0x37).unwrap();
        assert_eq!(odr.mask, odr_mask, "{mode:?}");
        assert_eq!(list.entries().len(), len, "{mode:?}");
        assert_eq!(odr.expected, (0x3C | mode.code()) & odr_mask, "{mode:?}");
    }
}

#[test]
fn verify_default_golden_trace() {
    let (s, mut b, _) = up(&Config::default());
    assert_eq!(run(b.verify()), Ok(()));
    assert_eq!(
        s.log(),
        [
            r(0x14, &[0x3A, 0x01, 0x00, 0x00, 0x00]),
            r(
                0x30,
                &[0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x3D, 0xA1]
            ),
        ]
    );
    assert_eq!((s.transactions(), s.awaits(), s.violations()), (2, 2, 0));
}

#[test]
fn verify_forced_golden_trace_has_no_validity_entry() {
    let (s, mut b, _) = up(&Config::default().with_power_mode(PowerMode::Forced));
    assert_eq!(run(b.verify()), Ok(()));
    assert_eq!(
        s.log(),
        [
            r(0x14, &[0x3A, 0x01, 0x00, 0x00, 0x00]),
            r(0x30, &[0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x3C]),
        ]
    );
}

#[test]
fn verify_not_configured_sends_nothing() {
    let (s, mut b, mut dl) = rig();
    assert_eq!(run(b.verify()), Err(Error::NotConfigured));
    assert_eq!(s.transactions(), 0);
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.arm(None, None);
    s.chip().regs[0x14] = 0;
    run(b.configure(&mut dl, &Config::default())).unwrap();
    assert_eq!(run(b.verify()), Ok(()));
    // A failed configure leaves it not configured.
    s.arm(Some(2), None);
    assert!(run(b.configure(&mut dl, &Config::default())).is_err());
    s.arm(None, None);
    assert_eq!(run(b.verify()), Err(Error::NotConfigured));
    assert_eq!(s.transactions(), 0);
}

#[test]
fn verify_is_ok_after_init_and_after_configure_in_the_documented_count() {
    for c in unlike_configs() {
        let (s, mut b, mut dl) = up(&c);
        assert_eq!(run(b.verify()), Ok(()), "{c:?}");
        assert_eq!((s.transactions(), s.awaits()), (2, 2), "{c:?}");
        for other in unlike_configs() {
            run(b.configure(&mut dl, &other)).unwrap();
            s.arm(None, None);
            assert_eq!(run(b.verify()), Ok(()), "{other:?} after {c:?}");
            assert_eq!(s.transactions(), 2);
        }
    }
}

#[test]
fn verify_forced_mode_reads_ok_while_the_chip_runs_its_measurement_and_after() {
    let (s, mut b, _) = up(&Config::default().with_power_mode(PowerMode::Forced));
    run(b.trigger_forced()).unwrap();
    assert_eq!(s.reg(reg::ODR_CONFIG) & 0b11, 2);
    assert_eq!(run(b.verify()), Ok(()));
    s.chip().regs[usize::from(reg::ODR_CONFIG)] &= !0b11;
    assert_eq!(run(b.verify()), Ok(()));
}

#[test]
fn verify_detects_every_masked_bit_and_ignores_every_other_bit() {
    for c in unlike_configs() {
        let owned = c.owned_registers();
        for o in owned.entries() {
            for bit in 0..8u8 {
                let (s, mut b, _) = up(&c);
                s.chip().regs[usize::from(o.reg)] ^= 1 << bit;
                let found = s.reg(o.reg);
                if o.mask & (1 << bit) != 0 {
                    assert_eq!(
                        run(b.verify()),
                        mismatch(o.reg, o.mask, o.expected, found),
                        "{:#x} bit {bit}",
                        o.reg
                    );
                    // The first window (0x14..0x18) then its re-read; the second window is not read.
                    assert_eq!(s.transactions(), if o.reg < 0x30 { 2 } else { 3 });
                } else {
                    assert_eq!(run(b.verify()), Ok(()), "{:#x} bit {bit}", o.reg);
                    assert_eq!(s.transactions(), 2);
                }
            }
        }
        let regs = owned_regs(&c);
        for addr in (0..0x80u8).filter(|a| !regs.contains(a)) {
            for bit in 0..8u8 {
                let (s, mut b, _) = up(&c);
                s.chip().regs[usize::from(addr)] ^= 1 << bit;
                assert_eq!(run(b.verify()), Ok(()), "{addr:#x} bit {bit}");
                assert_eq!(s.transactions(), 2);
            }
        }
    }
}

#[test]
fn verify_reports_a_pair_the_chip_rejects_behind_the_drivers_back() {
    let (s, mut b, _) = up(&Config::default());
    s.chip().rule = refuse_all;
    // The application rewrites the same rate; the chip re-evaluates and swaps the oversampling.
    run(b.write_register(reg::ODR_CONFIG, 0x3D)).unwrap();
    s.arm(None, None);
    assert_eq!(run(b.verify()), mismatch(0x38, 0xBF, 0xA1, 0x12));
    assert_eq!(b.config(), Some(&Config::default()));
    // Only the flag clear, the oversampling as applied: still reported.
    s.chip().regs[usize::from(reg::OSR_EFF)] = 0x21;
    assert_eq!(run(b.verify()), mismatch(0x38, 0xBF, 0xA1, 0x21));
    // The same fake in forced mode: the entry does not exist, verify is Ok.
    let (s, mut b, _) = up(&Config::default().with_power_mode(PowerMode::Forced));
    s.chip().rule = refuse_all;
    run(b.write_register(reg::ODR_CONFIG, 0x3C)).unwrap();
    assert_eq!(run(b.verify()), Ok(()));
}

#[test]
fn verify_one_off_corrupted_read_is_ok_with_one_extra_transaction() {
    for (addr, xor, window) in [(0x37u8, 0x04u8, 2usize), (0x15, 0x01, 1), (0x38, 0x80, 2)] {
        let (s, mut b, _) = up(&Config::default());
        s.chip().glitch = Some((addr, xor));
        assert_eq!(run(b.verify()), Ok(()), "{addr:#x}");
        assert_eq!((s.transactions(), s.awaits()), (3, 3));
        let log = s.log();
        let good = s.reg(addr);
        assert_eq!(log.get(window), Some(&r(addr, &[good])));
        assert_eq!(s.chip().glitch, None);
    }
}

#[test]
fn verify_bus_error_on_its_single_re_read_is_returned_and_changes_nothing() {
    // A one-off bad byte in the second window makes verify read that register again alone: transaction 3.
    let c = Config::default();
    let (s, mut b, _) = up(&c);
    s.chip().glitch = Some((0x37, 0x04));
    s.arm(Some(3), None);
    let regs = s.chip().regs;
    assert_eq!(run(b.verify()), Err(Error::Bus(BusFault::Injected)));
    assert_eq!(s.transactions(), 3);
    assert_eq!(s.log().len(), 2);
    assert_eq!(b.config(), Some(&c));
    assert_eq!(b.stats().bus_errors, 1);
    assert_eq!(s.chip().regs, regs);
    s.arm(None, None);
    assert_eq!(run(b.verify()), Ok(()));
    assert_eq!(s.transactions(), 2);
    assert_eq!(b.stats().bus_errors, 1);
}

#[test]
fn recover_after_configure_only_does_not_compare_the_revision() {
    let (s, mut b, mut dl) = rig();
    let info = run(b.configure(&mut dl, &Config::default())).unwrap();
    assert_eq!(info.revision, 0);
    // A revision the driver has never seen is accepted: nothing to compare it with.
    s.chip().regs[0x02] = 0x99;
    s.arm(None, None);
    assert_eq!(run(b.recover(&mut dl)), Ok(()));
    assert_eq!(s.transactions(), 4);
    assert_eq!(s.log().get(2), Some(&r(0x01, &[0x50, 0x99])));
    // A wrong chip id is still caught.
    s.chip().regs[0x01] = 0x51;
    assert_eq!(run(b.recover(&mut dl)), Err(Error::WrongId { found: 0x51 }));
    // After init the revision is compared.
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.chip().regs[0x02] = 0x99;
    assert_eq!(run(b.recover(&mut dl)), Err(Error::WrongId { found: 0x99 }));
}

#[test]
fn verify_reads_no_clear_on_read_register_and_keeps_an_interrupt_flag() {
    let c = Config::default();
    let (s, mut b, _) = up(&c);
    s.chip().regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_POR | reg::INT_STATUS_DRDY;
    for _ in 0..2 {
        assert_eq!(run(b.verify()), Ok(()));
        assert_eq!(run(b.recover(&mut s.delay())), Ok(()));
    }
    assert_eq!(s.reg(reg::INT_STATUS), 0x11);
    for e in s.log() {
        if let testbus::Event::I2c { ops, .. } = e {
            for op in ops {
                if let testbus::Op::Write(bytes) = op {
                    assert_eq!(bytes.len(), 1, "no write to the device");
                    assert_ne!(bytes.first(), Some(&reg::INT_STATUS));
                }
            }
        }
    }
    for c in unlike_configs() {
        let (s, mut b, _) = up(&c);
        assert_eq!(run(b.verify()), Ok(()));
        for e in s.log() {
            if let testbus::Event::I2c { ops, .. } = e
                && let (Some(testbus::Op::Write(p)), Some(testbus::Op::Read(v))) =
                    (ops.first(), ops.get(1))
            {
                let start = usize::from(p[0]);
                assert!(!(start..start + v.len()).contains(&usize::from(reg::INT_STATUS)));
            }
        }
    }
    let st = run(b.interrupt_status()).unwrap();
    assert!(st.power_on_reset && st.data_ready);
}

#[test]
fn verify_and_recover_never_change_the_state_in_any_outcome() {
    let c = unlike_configs()[4];
    let outcomes: [fn(&Shared); 4] = [
        |_| {},
        |s| s.chip().regs[0x36] ^= 1,
        |s| s.chip().regs[0x02] ^= 1,
        |s| s.chip().regs[0x01] ^= 1,
    ];
    for tweak in outcomes {
        let (s, mut b, mut dl) = up(&c);
        tweak(&s);
        let before = s.chip().regs;
        let v = run(b.verify());
        assert_eq!(b.config(), Some(&c));
        let rc = run(b.recover(&mut dl));
        assert_eq!(b.config(), Some(&c));
        assert_eq!(s.chip().regs, before);
        let _ = (v, rc);
    }
    // Bus errors.
    for k in 1..=2 {
        let (s, mut b, _) = up(&c);
        s.arm(Some(k), None);
        assert_eq!(run(b.verify()), Err(Error::Bus(BusFault::Injected)));
        assert_eq!(b.config(), Some(&c));
    }
    // Not configured stays not configured.
    let (_, mut b, mut dl) = rig();
    assert_eq!(run(b.recover(&mut dl)), Err(Error::NotConfigured));
    assert_eq!(b.config(), None);
}

#[test]
fn recover_default_golden_trace() {
    let (s, mut b, mut dl) = up(&Config::default());
    assert_eq!(run(b.recover(&mut dl)), Ok(()));
    assert_eq!(
        s.log(),
        [
            d(2_000),
            r(0x01, &[0x50]),
            r(0x01, &[0x50, 0x32]),
            r(0x14, &[0x3A, 0x01, 0x00, 0x00, 0x00]),
            r(
                0x30,
                &[0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x3D, 0xA1]
            ),
        ]
    );
    assert_eq!((s.transactions(), s.awaits(), s.violations()), (4, 5, 0));
}

#[test]
fn recover_not_configured_checks_the_identity_then_says_so() {
    let (s, mut b, mut dl) = rig();
    assert_eq!(run(b.recover(&mut dl)), Err(Error::NotConfigured));
    assert_eq!(
        s.log(),
        [d(2_000), r(0x01, &[0x50]), r(0x01, &[0x50, 0x32])]
    );
    assert_eq!(s.transactions(), 2);
    s.chip().regs[0x01] = 0x51;
    assert_eq!(run(b.recover(&mut dl)), Err(Error::WrongId { found: 0x51 }));
}

#[test]
fn recover_wrong_identity_or_wrong_revision_is_the_wrong_id_error() {
    let (s, mut b, mut dl) = up(&Config::default());
    s.chip().regs[0x01] = 0x51;
    s.arm(None, None);
    assert_eq!(run(b.recover(&mut dl)), Err(Error::WrongId { found: 0x51 }));
    assert_eq!(s.transactions(), 2);
    s.chip().regs[0x01] = 0x50;
    s.chip().regs[0x02] = 0x33;
    s.arm(None, None);
    assert_eq!(run(b.recover(&mut dl)), Err(Error::WrongId { found: 0x33 }));
    assert_eq!(s.transactions(), 2);
    assert_eq!(b.config(), Some(&Config::default()));
    // A stale byte on the second read only: the discarded read does not hide it.
    s.chip().regs[0x02] = 0x32;
    s.arm(None, None);
    s.chip().glitch = Some((0x02, 0x01));
    assert_eq!(run(b.recover(&mut dl)), Err(Error::WrongId { found: 0x33 }));
    assert_eq!(run(b.recover(&mut dl)), Ok(()));
}

#[test]
fn recover_reports_a_mismatch_after_the_identity_check() {
    let (s, mut b, mut dl) = up(&Config::default());
    s.chip().regs[0x36] ^= 0x08;
    s.arm(None, None);
    assert_eq!(run(b.recover(&mut dl)), mismatch(0x36, 0x7F, 0x61, 0x69));
    assert_eq!(s.transactions(), 5);
}

#[derive(Clone, Copy, Debug)]
enum Q {
    Verify,
    Recover,
    Status,
    Eff,
    Int,
}

#[derive(Debug, PartialEq, Eq)]
enum Out {
    Unit,
    Status(Status),
    Eff(EffectiveOversampling),
    Int(IntStatus),
}

async fn call(b: &mut Drv, dl: &mut TestDelay, q: Q) -> Res<Out> {
    match q {
        Q::Verify => b.verify().await.map(|()| Out::Unit),
        Q::Recover => b.recover(dl).await.map(|()| Out::Unit),
        Q::Status => b.status().await.map(Out::Status),
        Q::Eff => b.effective_oversampling().await.map(Out::Eff),
        Q::Int => b.interrupt_status().await.map(Out::Int),
    }
}

fn setup(c: Option<&Config>) -> (Shared, Drv, TestDelay) {
    let (s, b, dl) = match c {
        Some(c) => up(c),
        None => rig(),
    };
    s.chip().regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_DRDY;
    (s, b, dl)
}

#[test]
fn verify_recover_and_the_status_reads_survive_a_bus_error_and_a_cancel_anywhere() {
    let forced = Config::default().with_power_mode(PowerMode::Forced);
    let setups = [Some(Config::default()), Some(forced), None];
    for q in [Q::Verify, Q::Recover, Q::Status, Q::Eff, Q::Int] {
        for c in &setups {
            let (s, mut b, mut dl) = setup(c.as_ref());
            s.arm(None, None);
            let want = run(call(&mut b, &mut dl, q));
            let (n, awaits) = (s.transactions(), s.awaits());
            let stats = b.stats();
            for k in 1..=n {
                let (s, mut b, mut dl) = setup(c.as_ref());
                s.arm(Some(k), None);
                assert_eq!(
                    run(call(&mut b, &mut dl, q)),
                    Err(Error::Bus(BusFault::Injected)),
                    "{q:?} fail {k}"
                );
                assert_eq!(b.config(), c.as_ref(), "{q:?} fail {k}");
                assert_eq!(b.stats().bus_errors, 1, "{q:?} fail {k}");
                s.arm(None, None);
                assert_eq!(run(call(&mut b, &mut dl, q)), want, "{q:?} again {k}");
            }
            for k in 1..=awaits {
                let (s, mut b, mut dl) = setup(c.as_ref());
                s.arm(None, Some(k));
                assert_eq!(
                    poll_then_drop(call(&mut b, &mut dl, q)),
                    Polled::Cancelled,
                    "{q:?} await {k}"
                );
                assert_eq!(b.config(), c.as_ref(), "{q:?} drop {k}");
                s.arm(None, None);
                assert_eq!(run(call(&mut b, &mut dl, q)), want, "{q:?} again {k}");
            }
            let (s, mut b, mut dl) = setup(c.as_ref());
            s.arm(None, Some(awaits + 1));
            assert!(matches!(
                poll_then_drop(call(&mut b, &mut dl, q)),
                Polled::Done(_)
            ));
            assert_eq!(stats.bus_errors, 0);
        }
    }
}

#[test]
fn status_reads_the_status_register_in_one_transaction_and_decodes_every_bit() {
    let (s, mut b, _) = rig();
    s.arm(None, None);
    let st = run(b.status()).unwrap();
    assert_eq!(s.log(), [r(0x28, &[0x02])]);
    assert_eq!((s.transactions(), s.awaits()), (1, 1));
    assert!(st.nvm_ready && !st.core_ready && !st.nvm_error && !st.nvm_command_error);
    assert_eq!(st.raw, 0x02);
    for (bit, pick) in [
        (0u8, (|x: Status| x.core_ready) as fn(Status) -> bool),
        (1, |x| x.nvm_ready),
        (2, |x| x.nvm_error),
        (3, |x| x.nvm_command_error),
        (4, |x| x.boot_error_corrected),
        (7, |x| x.crack_check_passed),
    ] {
        s.chip().regs[usize::from(reg::STATUS)] = 1 << bit;
        let st = run(b.status()).unwrap();
        assert!(pick(st), "bit {bit}");
        assert_eq!(st.raw, 1 << bit);
    }
    s.chip().regs[usize::from(reg::STATUS)] = 0x60;
    assert_eq!(run(b.status()).unwrap().raw, 0x60);
    assert_eq!(Status::default(), Status::from_raw(0));
    // Not clear-on-read.
    assert_eq!(run(b.status()).unwrap().raw, 0x60);
}

#[test]
fn effective_oversampling_reads_osr_eff_in_one_transaction_configured_or_not() {
    let (s, mut b, mut dl) = rig();
    s.arm(None, None);
    assert_eq!(
        run(b.effective_oversampling()),
        Ok(EffectiveOversampling::default())
    );
    assert_eq!(s.log(), [r(0x38, &[0x00])]);
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.arm(None, None);
    let e = run(b.effective_oversampling()).unwrap();
    assert_eq!(s.log(), [r(0x38, &[0xA1])]);
    assert_eq!(
        (e.valid, e.pressure, e.temperature),
        (true, Oversampling::X16, Oversampling::X2)
    );
    s.chip().rule = refuse_all;
    run(b.write_register(reg::ODR_CONFIG, 0x3D)).unwrap();
    let e = run(b.effective_oversampling()).unwrap();
    assert_eq!(
        (e.valid, e.pressure, e.temperature),
        (false, Oversampling::X4, Oversampling::X4)
    );
}

#[test]
fn interrupt_status_reports_every_bit_and_the_power_on_flag_once() {
    let (s, mut b, _) = up(&Config::default());
    assert_eq!(b.stats(), Stats::default());
    for (bit, pick) in [
        (0u8, (|x: IntStatus| x.data_ready) as fn(IntStatus) -> bool),
        (1, |x| x.fifo_full),
        (2, |x| x.fifo_threshold),
        (3, |x| x.out_of_range),
        (4, |x| x.power_on_reset),
    ] {
        s.chip().regs[usize::from(reg::INT_STATUS)] = 1 << bit;
        s.arm(None, None);
        let st = run(b.interrupt_status()).unwrap();
        assert!(pick(st), "bit {bit}");
        assert_eq!(s.log(), [r(0x27, &[1 << bit])]);
        assert_eq!(s.reg(reg::INT_STATUS), 0);
    }
    assert_eq!(b.stats().power_on_resets, 1);
    s.chip().regs[usize::from(reg::INT_STATUS)] = 0x1F;
    let st = run(b.interrupt_status()).unwrap();
    assert!(st.data_ready && st.fifo_full && st.fifo_threshold && st.out_of_range);
    assert_eq!(b.stats().power_on_resets, 2);
    let st = run(b.interrupt_status()).unwrap();
    assert!(!st.power_on_reset);
    assert_eq!(b.stats().power_on_resets, 2);
}

#[test]
fn stats_count_a_reset_seen_once_and_not_the_one_init_causes() {
    let (s, mut b, mut dl) = rig();
    run(b.init(&mut dl, &Config::default())).unwrap();
    assert_eq!(b.stats(), Stats::default());
    // A brown-out while running: the chip resets itself and sets the flag.
    s.chip().regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_POR;
    assert!(run(b.interrupt_status()).unwrap().power_on_reset);
    assert_eq!(b.stats().power_on_resets, 1);
    assert_eq!(
        run(b.interrupt_status()).map(|s| s.power_on_reset),
        Ok(false)
    );
    assert_eq!(b.stats().power_on_resets, 1);
    // Not configured: no read, nothing counted.
    let (_, mut b2, _) = rig();
    assert_eq!(run(b2.interrupt_status()), Err(Error::NotConfigured));
    assert_eq!(b2.stats(), Stats::default());
}

#[test]
fn stats_count_bus_errors_where_returned_and_saturate() {
    let (s, mut b, mut dl) = rig();
    s.arm(Some(3), None);
    assert!(run(b.init(&mut dl, &Config::default())).is_err());
    assert_eq!(b.stats().bus_errors, 1);
    s.arm(None, None);
    assert!(run(b.write_register(reg::ODR_CONFIG, 0)).is_ok());
    assert_eq!(b.stats().bus_errors, 1);
    s.arm(Some(1), None);
    assert!(run(b.write_register(reg::ODR_CONFIG, 0)).is_err());
    assert!(run(b.read_register(reg::CHIP_ID)).is_ok());
    assert_eq!(b.stats().bus_errors, 2);
    b.stats.bus_errors = u32::MAX;
    b.stats.power_on_resets = u32::MAX;
    s.arm(Some(1), None);
    assert!(run(b.read_register(reg::CHIP_ID)).is_err());
    assert_eq!(b.stats().bus_errors, u32::MAX);
    run(b.init(&mut dl, &Config::default())).unwrap();
    s.chip().regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_POR;
    run(b.interrupt_status()).unwrap();
    assert_eq!(b.stats().power_on_resets, u32::MAX);
}

#[test]
fn verify_and_recover_and_the_status_reads_through_blocking_match_the_async_driver() {
    let c = Config::default();
    let (s, mut b, mut dl) = rig();
    s.arm(None, None);
    let a = (
        run(b.init(&mut dl, &c)),
        run(b.verify()),
        run(b.status()),
        run(b.effective_oversampling()),
        run(b.recover(&mut dl)),
        {
            s.chip().regs[0x36] ^= 1;
            run(b.verify())
        },
        run(b.recover(&mut dl)),
        b.stats(),
    );
    let log = s.log();
    let s = Shared::new();
    let mut bb = blocking::Bmp581::new(s.bus(), ADDR);
    let mut dl = s.delay();
    s.arm(None, None);
    let bl = (
        bb.init(&mut dl, &c),
        bb.verify(),
        bb.status(),
        bb.effective_oversampling(),
        bb.recover(&mut dl),
        {
            s.chip().regs[0x36] ^= 1;
            bb.verify()
        },
        bb.recover(&mut dl),
        bb.stats(),
    );
    assert!(a.1.is_ok() && a.4.is_ok() && a.5.is_err());
    assert_eq!(a, bl);
    assert_eq!(log, s.log());
    let s = Shared::new();
    let mut bb = blocking::Bmp581::new(s.bus(), ADDR);
    assert_eq!(bb.verify(), Err(Error::NotConfigured));
}

fn assert_send<T: Send>(_: &T) {}

#[test]
fn verify_and_recover_futures_are_send() {
    let (_, mut b, mut dl) = rig();
    assert_send(&b.verify());
    assert_send(&b.recover(&mut dl));
    assert_send(&b.status());
    assert_send(&b.effective_oversampling());
    assert_send(&b.interrupt_status());
}
