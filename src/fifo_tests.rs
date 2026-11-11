//! FIFO tests: the pure frame decoder, the three driver methods over the fake FIFO of `testbus`, loss
//! counters, failures and drops at every transaction and await, blocking parity.

use proptest::prelude::*;
use std::vec::Vec;

use crate::testbus::{
    BusFault, Event, Polled, Shared, TestBus, TestDelay, d, poll_then_drop, r, run, w,
};
use crate::*;

const ADDR: u8 = DEFAULT_ADDRESS;
type Drv = Bmp581<TestBus>;

const KINDS: [FifoData; 3] = [
    FifoData::PressureAndTemperature,
    FifoData::Temperature,
    FifoData::Pressure,
];

fn cfg(data: FifoData, mode: FifoMode) -> Config {
    Config::default().with_fifo(FifoConfig::new().with_mode(mode).with_data(data))
}

fn rig(c: &Config) -> (Shared, Drv, TestDelay) {
    let s = Shared::new();
    let mut b = Bmp581::new(s.bus(), ADDR);
    let mut dl = s.delay();
    run(b.init(&mut dl, c)).unwrap();
    (s, b, dl)
}

/// A frame of kind `data` that is never the empty frame.
fn frame(data: FifoData, i: u8) -> Vec<u8> {
    let mut f = std::vec![i, 0x01, 0x02, i, 0x03, 0x04];
    f.truncate(data.frame_bytes());
    f
}

fn decode(data: FifoData, bytes: &[u8]) -> Vec<FifoFrame<'_>> {
    fifo_frames(bytes, data).collect()
}

fn enc(v: i32) -> [u8; 3] {
    let b = v.to_le_bytes();
    [b[0], b[1], b[2]]
}

/// Test-only encoder: the bytes an item stands for.
fn encode(item: &FifoFrame<'_>) -> Vec<u8> {
    match item {
        FifoFrame::PressureTemperature(s) => [enc(s.temperature), enc(s.pressure)].concat(),
        FifoFrame::Temperature(t) => enc(t.temperature).to_vec(),
        FifoFrame::Pressure(p) => enc(p.pressure).to_vec(),
        FifoFrame::Empty(raw) | FifoFrame::Partial(raw) => raw.to_vec(),
    }
}

#[test]
fn fifo_frames_golden_pressure_temperature_frame_has_temperature_first() {
    // Table 11, p23-24: temperature XLSB, LSB, MSB, then pressure XLSB, LSB, MSB. 25 degC is 25 * 65536 =
    // 0x190000; 100 000 Pa is 100 000 * 64 = 0x61A800.
    let bytes = [0x00, 0x00, 0x19, 0x00, 0xA8, 0x61];
    let items = decode(FifoData::PressureAndTemperature, &bytes);
    let [FifoFrame::PressureTemperature(s)] = items[..] else {
        panic!("{items:?}");
    };
    assert_eq!((s.temperature, s.pressure), (0x19_0000, 0x61_A800));
    assert_eq!((s.celsius(), s.pascals()), (25.0, 100_000.0));
}

#[test]
fn fifo_frames_golden_single_value_frames() {
    // Table 12 and 13, p24: three bytes, XLSB first.
    let t = decode(FifoData::Temperature, &[0x00, 0x00, 0x19]);
    let [FifoFrame::Temperature(t)] = t[..] else {
        panic!()
    };
    assert_eq!((t.temperature, t.celsius()), (0x19_0000, 25.0));
    let p = decode(FifoData::Pressure, &[0x00, 0xA8, 0x61]);
    let [FifoFrame::Pressure(p)] = p[..] else {
        panic!()
    };
    assert_eq!((p.pressure, p.pascals()), (0x61_A800, 100_000.0));
    // The kind decides, not the bytes.
    let t = decode(FifoData::Temperature, &[1, 2, 3]);
    assert_eq!(
        t[..],
        [FifoFrame::Temperature(TemperatureFrame {
            temperature: 0x03_0201
        })]
    );
}

#[test]
fn fifo_frames_sign_handling_at_the_extremes() {
    let cases: [([u8; 3], i32); 4] = [
        ([0xFF, 0xFF, 0x7F], 8_388_607),
        ([0x00, 0x00, 0x80], -8_388_608),
        ([0xFF, 0xFF, 0xFF], -1),
        ([0x00, 0x00, 0x00], 0),
    ];
    for (b, v) in cases {
        assert_eq!(
            decode(FifoData::Temperature, &b)[..],
            [FifoFrame::Temperature(TemperatureFrame { temperature: v })]
        );
        assert_eq!(
            decode(FifoData::Pressure, &b)[..],
            [FifoFrame::Pressure(PressureFrame { pressure: v })]
        );
        let pt = [b, b].concat();
        assert_eq!(
            decode(FifoData::PressureAndTemperature, &pt)[..],
            [FifoFrame::PressureTemperature(RawSample {
                temperature: v,
                pressure: v
            })]
        );
    }
}

#[test]
fn fifo_frames_empty_frame_is_a_whole_chunk_of_0x7f() {
    // Table 14, p24 and 4.6.3, p23: an empty read returns 0x7F.
    for data in KINDS {
        let n = data.frame_bytes();
        let bytes = std::vec![0x7F; 2 * n];
        assert_eq!(
            decode(data, &bytes)[..],
            [FifoFrame::Empty(&bytes[..n]), FifoFrame::Empty(&bytes[n..])]
        );
    }
    // One byte off 0x7F is a measurement: 0x7F7F7E is a pressure and temperature half.
    let mut bytes = [0x7F; 6];
    bytes[3] = 0x7E;
    let items = decode(FifoData::PressureAndTemperature, &bytes);
    assert_eq!(
        items[..],
        [FifoFrame::PressureTemperature(RawSample {
            temperature: 0x7F_7F7F,
            pressure: 0x7F_7F7E
        })]
    );
    // Half empty frame: the other half is a real one, nothing is dropped.
    let half = [0x00, 0x00, 0x19, 0x7F, 0x7F, 0x7F];
    let [FifoFrame::PressureTemperature(s)] = decode(FifoData::PressureAndTemperature, &half)[..]
    else {
        panic!()
    };
    assert!(s.pressure_is_reset_value() && !s.temperature_is_reset_value());
}

#[test]
fn fifo_frames_trailing_bytes_come_out_once_with_their_bytes() {
    let mut bytes = frame(FifoData::PressureAndTemperature, 5);
    bytes.extend_from_slice(&[0x7F, 0x7F, 0x7F, 0x7F]);
    let items = decode(FifoData::PressureAndTemperature, &bytes);
    assert_eq!(items.len(), 2);
    assert_eq!(items[1], FifoFrame::Partial(&[0x7F; 4]));
    assert_eq!(
        decode(FifoData::Temperature, &[1, 2])[..],
        [FifoFrame::Partial(&[1, 2])]
    );
    assert_eq!(
        decode(FifoData::Pressure, &[9])[..],
        [FifoFrame::Partial(&[9])]
    );
    assert!(decode(FifoData::Pressure, &[]).is_empty());
}

#[test]
fn fifo_frames_conversions_are_those_of_the_register_sample() {
    let s = RawSample {
        temperature: -123_456,
        pressure: 6_543_210,
    };
    assert_eq!(
        TemperatureFrame {
            temperature: s.temperature
        }
        .celsius(),
        s.celsius()
    );
    assert_eq!(
        PressureFrame {
            pressure: s.pressure
        }
        .pascals(),
        s.pascals()
    );
}

fn kind() -> impl Strategy<Value = FifoData> {
    proptest::sample::select(KINDS.to_vec())
}

proptest! {
    #[test]
    fn fifo_frames_never_panics_and_accounts_for_every_byte(
        bytes in proptest::collection::vec(any::<u8>(), 0..200),
        data in kind(),
    ) {
        let size = data.frame_bytes();
        let items = decode(data, &bytes);
        let total: usize = items.iter().map(|i| encode(i).len()).sum();
        prop_assert_eq!(total, bytes.len());
        prop_assert_eq!(items.len(), bytes.len().div_ceil(size));
        prop_assert_eq!(fifo_frames(&bytes, data).size_hint(), (items.len(), Some(items.len())));
        // The items put back byte for byte are the input; a Partial is only ever last.
        let back: Vec<u8> = items.iter().flat_map(encode).collect();
        prop_assert_eq!(&back, &bytes);
        for (i, item) in items.iter().enumerate() {
            if let FifoFrame::Partial(raw) = item {
                prop_assert!(i + 1 == items.len() && raw.len() < size && !raw.is_empty());
            }
        }
    }

    #[test]
    fn fifo_frames_round_trip_through_a_test_encoder(
        values in proptest::collection::vec((-8_388_608i32..8_388_608, -8_388_608i32..8_388_608), 0..40),
        data in kind(),
    ) {
        let mut bytes = Vec::new();
        let mut expected = Vec::new();
        for (a, b) in values {
            match data {
                FifoData::PressureAndTemperature => {
                    prop_assume!(!(a == 0x7F_7F7F && b == 0x7F_7F7F));
                    bytes.extend_from_slice(&[enc(a), enc(b)].concat());
                    expected.push(FifoFrame::PressureTemperature(RawSample { temperature: a, pressure: b }));
                }
                FifoData::Temperature => {
                    prop_assume!(a != 0x7F_7F7F);
                    bytes.extend_from_slice(&enc(a));
                    expected.push(FifoFrame::Temperature(TemperatureFrame { temperature: a }));
                }
                FifoData::Pressure => {
                    prop_assume!(a != 0x7F_7F7F);
                    bytes.extend_from_slice(&enc(a));
                    expected.push(FifoFrame::Pressure(PressureFrame { pressure: a }));
                }
            }
        }
        prop_assert_eq!(decode(data, &bytes), expected);
    }

    #[test]
    fn fifo_read_never_splits_a_frame_and_decodes_to_what_was_pushed(
        data in kind(),
        waiting_seed in 0usize..=32,
        len in 0usize..=120,
    ) {
        let waiting = waiting_seed.min(usize::from(data.capacity_frames()));
        let size = data.frame_bytes();
        let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
        let pushed: Vec<Vec<u8>> = (0..waiting).map(|i| frame(data, i as u8)).collect();
        for f in &pushed { s.push_frame(f); }
        let mut buf = std::vec![0xAAu8; len];
        s.arm(None, None);
        let res = run(b.fifo_read(&mut buf));
        if len < size {
            prop_assert_eq!(res, Err(Error::BufferTooSmall));
            prop_assert_eq!(s.transactions(), 0);
        } else {
            let n = waiting.min(len / size);
            let got = res.unwrap();
            prop_assert_eq!(got.bytes, n * size);
            prop_assert_eq!(usize::from(got.frames), n);
            prop_assert_eq!(usize::from(got.waiting), waiting - n);
            prop_assert_eq!(got.full, waiting == usize::from(data.capacity_frames()));
            prop_assert_eq!(s.transactions(), if n == 0 { 1 } else { 2 });
            prop_assert!(buf[got.bytes..].iter().all(|&x| x == 0xAA));
            let want: Vec<FifoFrame<'_>> = pushed.iter().take(n).flat_map(|f| decode(data, f)).collect();
            prop_assert_eq!(decode(data, &buf[..got.bytes]), want);
            prop_assert_eq!(s.fifo_len(), waiting - n);
            prop_assert_eq!(s.violations(), 0);
        }
    }
}

#[test]
fn fifo_read_golden_trace_with_frames_waiting_is_two_transactions() {
    // 4.6.2, p23 (FIFO_COUNT, 7.8, p55), then a burst from FIFO_DATA (4.6.3, p23; 7.23, p59).
    let data = FifoData::PressureAndTemperature;
    let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
    let (f0, f1) = (frame(data, 1), frame(data, 2));
    s.push_frame(&f0);
    s.push_frame(&f1);
    let mut buf = [0u8; 64];
    s.arm(None, None);
    let got = run(b.fifo_read(&mut buf)).unwrap();
    assert_eq!(
        (got.bytes, got.frames, got.waiting, got.full),
        (12, 2, 0, false)
    );
    assert_eq!(s.log(), [r(0x17, &[2]), r(0x29, &[f0, f1].concat())]);
    assert_eq!(s.fifo_len(), 0);
}

#[test]
fn fifo_read_golden_trace_with_nothing_waiting_is_one_transaction() {
    let (s, mut b, _dl) = rig(&cfg(FifoData::Pressure, FifoMode::StopOnFull));
    let mut buf = [0x55u8; 9];
    s.arm(None, None);
    assert_eq!(run(b.fifo_read(&mut buf)), Ok(FifoRead::default()));
    assert_eq!(s.log(), [r(0x17, &[0])]);
    assert_eq!(buf, [0x55; 9]);
}

#[test]
fn fifo_read_whole_fifo_is_one_burst_of_96_bytes() {
    for data in KINDS {
        let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
        for i in 0..data.capacity_frames() {
            s.push_frame(&frame(data, i));
        }
        let mut buf = [0u8; 200];
        s.arm(None, None);
        let got = run(b.fifo_read(&mut buf)).unwrap();
        assert_eq!(
            (got.bytes, got.frames, got.waiting, got.full),
            (96, data.capacity_frames(), 0, true)
        );
        let log = s.log();
        assert_eq!(log.len(), 2);
        assert!(
            matches!(&log[1], Event::I2c { ops, .. } if matches!(&ops[1], testbus::Op::Read(v) if v.len() == 96))
        );
    }
}

#[test]
fn fifo_read_limits_by_the_buffer_and_leaves_the_rest_waiting() {
    let data = FifoData::PressureAndTemperature;
    let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
    for i in 0..5 {
        s.push_frame(&frame(data, i));
    }
    // 17 bytes: two whole frames (12 bytes), the 5 left over are not read.
    let mut buf = [0u8; 17];
    let got = run(b.fifo_read(&mut buf)).unwrap();
    assert_eq!((got.bytes, got.frames, got.waiting), (12, 2, 3));
    assert_eq!(s.fifo_len(), 3);
    let got = run(b.fifo_read(&mut buf)).unwrap();
    assert_eq!((got.frames, got.waiting), (2, 1));
    assert_eq!(
        decode(data, &buf[..got.bytes])[0],
        decode(data, &frame(data, 2))[0]
    );
}

#[test]
fn fifo_read_reports_full_once_per_call_and_counts_it() {
    let data = FifoData::PressureAndTemperature;
    let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
    for i in 0..16 {
        s.push_frame(&frame(data, i));
    }
    let mut one = [0u8; 6];
    let got = run(b.fifo_read(&mut one)).unwrap();
    assert!(got.full && got.waiting == 15);
    assert_eq!(b.stats().fifo_full, 1);
    // Not full any more: 15 frames wait.
    let got = run(b.fifo_read(&mut one)).unwrap();
    assert!(!got.full);
    assert_eq!((b.stats().fifo_full, b.stats().fifo_frames), (1, 2));
    // A count the chip cannot produce is clamped to the capacity and reads as full.
    s.chip().count_forced = Some(40);
    let mut big = [0u8; 200];
    let got = run(b.fifo_read(&mut big)).unwrap();
    assert!(got.full && got.frames == 16 && got.waiting == 0);
}

#[test]
fn fifo_full_is_counted_by_the_interrupt_status_read_and_counters_saturate() {
    let data = FifoData::Temperature;
    let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
    s.chip().regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_FIFO_FULL;
    assert!(run(b.interrupt_status()).unwrap().fifo_full);
    assert_eq!(b.stats().fifo_full, 1);
    assert!(!run(b.interrupt_status()).unwrap().fifo_full);
    assert_eq!(b.stats().fifo_full, 1);
    b.stats.fifo_full = u32::MAX;
    b.stats.fifo_frames = u32::MAX - 1;
    for i in 0..32 {
        s.push_frame(&frame(data, i));
    }
    let mut buf = [0u8; 9];
    assert!(run(b.fifo_read(&mut buf)).unwrap().full);
    assert_eq!(
        (b.stats().fifo_full, b.stats().fifo_frames),
        (u32::MAX, u32::MAX)
    );
}

#[test]
fn fifo_read_after_overflow_returns_what_the_chip_kept() {
    let data = FifoData::Pressure;
    for (mode, first) in [(FifoMode::Stream, 5u8), (FifoMode::StopOnFull, 0)] {
        let (s, mut b, _dl) = rig(&cfg(data, mode));
        for i in 0..37 {
            s.push_frame(&frame(data, i));
        }
        let mut buf = [0u8; 200];
        let got = run(b.fifo_read(&mut buf)).unwrap();
        assert!(got.full && got.frames == 32);
        // Stream keeps the newest 32 (frames 5..37), stop on full keeps the oldest (0..32).
        assert_eq!(
            decode(data, &buf[..3])[0],
            decode(data, &frame(data, first))[0]
        );
    }
}

#[test]
fn fifo_count_is_one_transaction_of_one_byte() {
    let data = FifoData::PressureAndTemperature;
    let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
    for i in 0..3 {
        s.push_frame(&frame(data, i));
    }
    s.arm(None, None);
    assert_eq!(run(b.fifo_count()), Ok(3));
    assert_eq!(s.log(), [r(0x17, &[3])]);
}

#[test]
fn fake_fifo_bursts_stay_at_the_port_and_pad_with_the_empty_frame() {
    // 4.6.3, p23: whole frames pop, an incompletely read one is sent again, the rest is 0x7F.
    let data = FifoData::PressureAndTemperature;
    let (s, mut b, _dl) = rig(&cfg(data, FifoMode::Stream));
    let f = frame(data, 7);
    s.push_frame(&f);
    let mut four = [0u8; 4];
    run(b.read_registers(reg::FIFO_DATA, &mut four)).unwrap();
    assert_eq!(four[..], f[..4]);
    assert_eq!(s.fifo_len(), 1);
    let mut nine = [0u8; 9];
    run(b.read_registers(reg::FIFO_DATA, &mut nine)).unwrap();
    assert_eq!(nine[..6], f[..]);
    assert_eq!(nine[6..], [0x7F; 3]);
    assert_eq!(s.fifo_len(), 0);
    let items = decode(data, &nine);
    assert!(matches!(
        items[..],
        [FifoFrame::PressureTemperature(_), FifoFrame::Partial(_)]
    ));
}

#[test]
fn fifo_methods_error_before_any_bus_access() {
    let data = FifoData::PressureAndTemperature;
    // Not configured.
    let s = Shared::new();
    let mut b = Bmp581::new(s.bus(), ADDR);
    let mut dl = s.delay();
    s.arm(None, None);
    assert_eq!(run(b.fifo_count()), Err(Error::NotConfigured));
    assert_eq!(run(b.fifo_read(&mut [0u8; 64])), Err(Error::NotConfigured));
    assert_eq!(run(b.fifo_read(&mut [])), Err(Error::NotConfigured));
    assert_eq!(run(b.fifo_flush(&mut dl)), Err(Error::NotConfigured));
    assert_eq!((s.transactions(), s.log().len()), (0, 0));
    // FIFO off in the applied configuration.
    let (s, mut b, mut dl) = rig(&Config::default());
    s.arm(None, None);
    assert_eq!(run(b.fifo_count()), Err(Error::FifoOff));
    assert_eq!(run(b.fifo_read(&mut [0u8; 64])), Err(Error::FifoOff));
    assert_eq!(run(b.fifo_flush(&mut dl)), Err(Error::FifoOff));
    assert_eq!((s.transactions(), s.log().len()), (0, 0));
    assert!(b.config().is_some());
    // A buffer shorter than one frame.
    for (kind, short) in [
        (data, 5),
        (FifoData::Temperature, 2),
        (FifoData::Pressure, 0),
    ] {
        let (s, mut b, _dl) = rig(&cfg(kind, FifoMode::Stream));
        s.push_frame(&frame(kind, 1));
        s.arm(None, None);
        assert_eq!(
            run(b.fifo_read(&mut std::vec![0u8; short])),
            Err(Error::BufferTooSmall)
        );
        assert_eq!(s.transactions(), 0);
        assert_eq!(s.fifo_len(), 1);
    }
    // The state after a failed init is not configured.
    let (s, mut b, mut dl) = rig(&cfg(data, FifoMode::Stream));
    s.arm(Some(3), None);
    assert!(run(b.configure(&mut dl, &cfg(data, FifoMode::Stream))).is_err());
    s.arm(None, None);
    assert_eq!(run(b.fifo_read(&mut [0u8; 64])), Err(Error::NotConfigured));
    assert_eq!(s.transactions(), 0);
}

#[derive(Clone, Copy, Debug)]
enum M {
    Count,
    Read,
    Flush,
}

#[derive(Debug, PartialEq)]
enum Out {
    Count(u8),
    Read(FifoRead, Vec<u8>),
    Flush,
}

async fn call(b: &mut Drv, dl: &mut TestDelay, m: M) -> Result<Out, Error<BusFault>> {
    match m {
        M::Count => b.fifo_count().await.map(Out::Count),
        M::Read => {
            let mut buf = [0xAAu8; 64];
            let got = b.fifo_read(&mut buf).await?;
            Ok(Out::Read(got, buf[..got.bytes].to_vec()))
        }
        M::Flush => b.fifo_flush(dl).await.map(|()| Out::Flush),
    }
}

fn loaded(c: &Config) -> (Shared, Drv, TestDelay) {
    let (s, b, dl) = rig(c);
    for i in 0..3 {
        s.push_frame(&frame(c.fifo.data, i + 1));
    }
    (s, b, dl)
}

/// After a failed or dropped call: reads and counts simply repeat; a flush left the driver not configured
/// (contract A3), so `configure` comes first. Then the call gives the right result.
fn assert_repeats(
    s: &Shared,
    b: &mut Drv,
    dl: &mut TestDelay,
    m: M,
    c: &Config,
    want: &Out,
    at: &str,
) {
    s.arm(None, None);
    if matches!(m, M::Flush) && b.config().is_none() {
        assert_eq!(run(b.fifo_flush(dl)), Err(Error::NotConfigured), "{at}");
        run(b.configure(dl, c)).unwrap();
    } else {
        assert_eq!(b.config(), Some(c), "{at}");
    }
    assert_eq!(&run(call(b, dl, m)).unwrap(), want, "{at}");
    if matches!(m, M::Flush) {
        assert_eq!(s.fifo_len(), 0, "{at}");
        assert_eq!(run(b.verify()), Ok(()), "{at}");
    }
}

#[test]
fn fifo_methods_survive_a_bus_error_at_every_transaction_and_a_drop_at_every_await() {
    // The fake fails or pends a transaction before it takes effect, so no frame is taken by a failed
    // read; on a real bus the frames of a read that failed part-way may be gone (documented).
    for data in KINDS {
        let c = cfg(data, FifoMode::Stream);
        for m in [M::Count, M::Read, M::Flush] {
            let (s, mut b, mut dl) = loaded(&c);
            s.arm(None, None);
            let want = run(call(&mut b, &mut dl, m)).unwrap();
            let (n, awaits) = (s.transactions(), s.awaits());
            assert!(n >= 1 && awaits >= 1);
            for k in 1..=n {
                let (s, mut b, mut dl) = loaded(&c);
                s.arm(Some(k), None);
                assert_eq!(
                    run(call(&mut b, &mut dl, m)),
                    Err(Error::Bus(BusFault::Injected)),
                    "{m:?} {k}"
                );
                assert_eq!(b.stats().bus_errors, 1);
                assert_repeats(
                    &s,
                    &mut b,
                    &mut dl,
                    m,
                    &c,
                    &want,
                    &std::format!("{m:?} fail {k}"),
                );
            }
            for k in 1..=awaits {
                let (s, mut b, mut dl) = loaded(&c);
                s.arm(None, Some(k));
                assert_eq!(
                    poll_then_drop(call(&mut b, &mut dl, m)),
                    Polled::Cancelled,
                    "{m:?} {k}"
                );
                assert_repeats(
                    &s,
                    &mut b,
                    &mut dl,
                    m,
                    &c,
                    &want,
                    &std::format!("{m:?} drop {k}"),
                );
            }
        }
    }
}

#[test]
fn fifo_flush_golden_trace_and_leaves_the_applied_configuration() {
    // 4.6.4, p25: FIFO_SEL changes only in standby and flush the FIFO; treconf 3 ms, treconf_deep 4 ms
    // (Table 4, p11).
    let data = FifoData::PressureAndTemperature;
    let c = cfg(data, FifoMode::Stream);
    let (s, mut b, mut dl) = loaded(&c);
    let (s2, mut b2, mut dl2) = rig(&c);
    s2.arm(None, None);
    run(b2.configure(&mut dl2, &c)).unwrap();
    let configure_log = s2.log();
    s.arm(None, None);
    assert_eq!(run(b.fifo_flush(&mut dl)), Ok(()));
    let log = s.log();
    let standby = c.odr_config(PowerMode::Standby);
    assert_eq!(log[..4], [w(0x37, standby), d(2_500), w(0x18, 0), d(4_000)]);
    assert_eq!(log[4..log.len() - 1], configure_log[..]);
    assert_eq!(log[log.len() - 1], d(4_000));
    // 18 transactions and 1 delay of `configure`, plus 2 writes and 3 delays: 20 transactions, 4 delays.
    assert_eq!(log.len(), 24);
    assert_eq!(s.fifo_len(), 0);
    assert_eq!(b.config(), Some(&c));
    assert_eq!(run(b.verify()), Ok(()));
    assert_eq!(s.reg(reg::ODR_CONFIG), c.odr_config(PowerMode::Normal));
    assert_eq!(run(b.fifo_count()), Ok(0));
    assert_eq!(s.violations(), 0);
}

#[test]
fn fifo_flush_empties_the_fifo_in_every_kind_mode_and_power_mode() {
    for data in KINDS {
        for mode in [FifoMode::Stream, FifoMode::StopOnFull] {
            for power in [PowerMode::Normal, PowerMode::Forced] {
                let c = cfg(data, mode).with_power_mode(power);
                let (s, mut b, mut dl) = loaded(&c);
                assert_eq!(s.fifo_len(), 3);
                run(b.fifo_flush(&mut dl)).unwrap();
                assert_eq!(s.fifo_len(), 0, "{data:?} {mode:?} {power:?}");
                assert_eq!(run(b.verify()), Ok(()));
                assert_eq!(b.config(), Some(&c));
                let mut buf = [0u8; 12];
                assert_eq!(run(b.fifo_read(&mut buf)), Ok(FifoRead::default()));
            }
        }
    }
}

#[test]
fn fifo_blocking_parity_traces_and_results_are_identical() {
    let data = FifoData::PressureAndTemperature;
    let c = cfg(data, FifoMode::Stream);
    let (s, mut b, mut dl) = loaded(&c);
    s.arm(None, None);
    let mut buf = [0u8; 8];
    let a = (
        run(b.fifo_count()),
        run(b.fifo_read(&mut buf)),
        run(b.fifo_flush(&mut dl)),
        run(b.fifo_count()),
    );
    let a_buf = buf;
    let a_log = s.log();

    let s = Shared::new();
    let mut bb = blocking::Bmp581::new(s.bus(), ADDR);
    let mut dl = s.delay();
    bb.init(&mut dl, &c).unwrap();
    for i in 0..3 {
        s.push_frame(&frame(data, i + 1));
    }
    s.arm(None, None);
    let mut buf = [0u8; 8];
    let bl = (
        bb.fifo_count(),
        bb.fifo_read(&mut buf),
        bb.fifo_flush(&mut dl),
        bb.fifo_count(),
    );
    assert_eq!((a, a_buf), (bl, buf));
    assert_eq!(a_log, s.log());
    assert!(a_log.len() > 15);

    let s = Shared::new();
    let mut bb = blocking::Bmp581::new(s.bus(), ADDR);
    let mut dl = s.delay();
    assert_eq!(bb.fifo_count(), Err(Error::NotConfigured));
    assert_eq!(bb.fifo_read(&mut buf), Err(Error::NotConfigured));
    assert_eq!(bb.fifo_flush(&mut dl), Err(Error::NotConfigured));
}
