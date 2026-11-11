//! Host tests of the pure layer: register images against hand-computed bytes, validation, the plan
//! and the helpers. Expected values are written out as numbers worked from the datasheet
//! (BST-BMP581-DS004-13, printed pages in the comments), not computed from the `reg` constants.

use crate::calc::*;
use crate::plan::{MAX_STEPS, STANDBY_WAIT_US};
use crate::*;
use proptest::prelude::*;
use std::vec::Vec;

const ALL_OSR: [Oversampling; 8] = [
    Oversampling::X1,
    Oversampling::X2,
    Oversampling::X4,
    Oversampling::X8,
    Oversampling::X16,
    Oversampling::X32,
    Oversampling::X64,
    Oversampling::X128,
];
const ALL_IIR: [IirFilter; 8] = [
    IirFilter::Bypass,
    IirFilter::Coeff1,
    IirFilter::Coeff3,
    IirFilter::Coeff7,
    IirFilter::Coeff15,
    IirFilter::Coeff31,
    IirFilter::Coeff63,
    IirFilter::Coeff127,
];
const ALL_ODR: [OutputDataRate; 32] = [
    OutputDataRate::Hz240,
    OutputDataRate::Hz218_537,
    OutputDataRate::Hz199_111,
    OutputDataRate::Hz179_2,
    OutputDataRate::Hz160,
    OutputDataRate::Hz149_333,
    OutputDataRate::Hz140,
    OutputDataRate::Hz129_855,
    OutputDataRate::Hz120,
    OutputDataRate::Hz110_164,
    OutputDataRate::Hz100_299,
    OutputDataRate::Hz89_6,
    OutputDataRate::Hz80,
    OutputDataRate::Hz70,
    OutputDataRate::Hz60,
    OutputDataRate::Hz50_056,
    OutputDataRate::Hz45_025,
    OutputDataRate::Hz40,
    OutputDataRate::Hz35,
    OutputDataRate::Hz30,
    OutputDataRate::Hz25_005,
    OutputDataRate::Hz20,
    OutputDataRate::Hz15,
    OutputDataRate::Hz10,
    OutputDataRate::Hz5,
    OutputDataRate::Hz4,
    OutputDataRate::Hz3,
    OutputDataRate::Hz2,
    OutputDataRate::Hz1,
    OutputDataRate::Hz0_5,
    OutputDataRate::Hz0_25,
    OutputDataRate::Hz0_125,
];
const ALL_MODES: [PowerMode; 4] = [
    PowerMode::Standby,
    PowerMode::Normal,
    PowerMode::Forced,
    PowerMode::Continuous,
];
const ALL_DEC: [FifoDecimation; 8] = [
    FifoDecimation::By1,
    FifoDecimation::By2,
    FifoDecimation::By4,
    FifoDecimation::By8,
    FifoDecimation::By16,
    FifoDecimation::By32,
    FifoDecimation::By64,
    FifoDecimation::By128,
];

/// Independent copy of the rates of 7.34 (p65) in millihertz, in code order.
const ODR_MHZ: [u32; 32] = [
    240_000, 218_537, 199_111, 179_200, 160_000, 149_333, 140_000, 129_855, 120_000, 110_164,
    100_299, 89_600, 80_000, 70_000, 60_000, 50_056, 45_025, 40_000, 35_000, 30_000, 25_005,
    20_000, 15_000, 10_000, 5_000, 4_000, 3_000, 2_000, 1_000, 500, 250, 125,
];

/// Independent literal copy of Table 7 (p17) in Hz: rows osr_p 1x..128x, columns osr_t 1x..128x.
const TABLE7_HZ: [[u32; 8]; 8] = [
    [240, 240, 240, 240, 200, 130, 80, 40],
    [240, 240, 240, 220, 180, 120, 70, 40],
    [220, 220, 200, 180, 140, 100, 70, 40],
    [140, 140, 130, 120, 100, 80, 50, 35],
    [80, 80, 80, 70, 70, 50, 45, 30],
    [45, 45, 40, 40, 40, 35, 30, 20],
    [20, 20, 20, 20, 20, 20, 15, 15],
    [10, 10, 10, 10, 10, 10, 10, 5],
];
/// Independent literal copy of Table 8 (p17) in Hz, columns osr_t 1x..128x.
const TABLE8_HZ: [u32; 8] = [240, 240, 240, 240, 200, 130, 80, 40];

/// The rate code (in millihertz) nearest to a printed nominal rate in Hz: the datasheet table is read
/// as nominal values (p17), so 100 is 100.299, 50 is 50.056, 45 is 45.025, 220 is 218.537 (7.34, p65).
fn nearest_code_mhz(hz: u32) -> u32 {
    let want = i64::from(hz) * 1000;
    ODR_MHZ
        .iter()
        .copied()
        .min_by_key(|&m| (i64::from(m) - want).abs())
        .unwrap()
}

pub(crate) fn table_allows(pressure: bool, p: usize, t: usize, odr_code: usize) -> bool {
    let max_hz = if pressure {
        TABLE7_HZ[p][t]
    } else {
        TABLE8_HZ[t]
    };
    ODR_MHZ[odr_code] <= nearest_code_mhz(max_hz)
}

#[test]
fn table_reading_is_nominal() {
    for (hz, mhz) in [
        (220, 218_537),
        (200, 199_111),
        (180, 179_200),
        (130, 129_855),
        (100, 100_299),
        (50, 50_056),
        (45, 45_025),
        (240, 240_000),
        (40, 40_000),
        (5, 5_000),
    ] {
        assert_eq!(nearest_code_mhz(hz), mhz);
    }
    // The cells that the nominal reading changed (pressure on): (osr_p, osr_t, new rate).
    for (p, t, new) in [
        (2, 5, OutputDataRate::Hz100_299),
        (3, 4, OutputDataRate::Hz100_299),
        (3, 6, OutputDataRate::Hz50_056),
        (4, 5, OutputDataRate::Hz50_056),
        (4, 6, OutputDataRate::Hz45_025),
        (5, 0, OutputDataRate::Hz45_025),
        (5, 1, OutputDataRate::Hz45_025),
    ] {
        assert_eq!(max_odr(true, ALL_OSR[p], ALL_OSR[t]), new, "p {p} t {t}");
    }
}

// ---------------------------------------------------------------------------------------------
// Register images
// ---------------------------------------------------------------------------------------------

#[test]
fn image_default_bytes() {
    let c = Config::default();
    // 7.33 (p64): press_en bit 6 = 0x40, osr_p 16x = 0b100 << 3 = 0x20, osr_t 2x = 0b001.
    assert_eq!(c.osr_config(), 0x61);
    // 7.34 (p64-65): odr 0xF (50.056 Hz) << 2 = 0x3C, pwr_mode in bits 1:0.
    assert_eq!(c.odr_config(PowerMode::Standby), 0x3C);
    assert_eq!(c.odr_config(PowerMode::Normal), 0x3D);
    assert_eq!(c.odr_config(PowerMode::Forced), 0x3E);
    assert_eq!(c.odr_config(PowerMode::Continuous), 0x3F);
    // 7.27 (p61): reset 0x03. 7.28 (p62): 0.
    assert_eq!(c.dsp_config(), 0x03);
    assert_eq!(c.dsp_iir(), 0x00);
    // 7.6 (p54): drdy_data_reg_en bit 0.
    assert_eq!(c.int_source(), 0x01);
    // 7.5 (p52-53): pad drive 3 in 7:4 = 0x30, int_en 0x08, int_pol 0x02; pulsed, push-pull.
    assert_eq!(c.int_config(), 0x3A);
    // 7.7 (p54), 7.9 (p55), 7.29 to 7.32 (p63): reset values.
    assert_eq!(c.fifo_config(), 0x00);
    assert_eq!(c.fifo_sel(), 0x00);
    assert_eq!(
        [
            c.oor_thr_p_lsb(),
            c.oor_thr_p_msb(),
            c.oor_range(),
            c.oor_config()
        ],
        [0; 4]
    );
}

#[test]
fn image_osr_config() {
    // 7.33 (p64): temperature only, osr_t 128x = 0b111.
    let c = Config::new()
        .with_pressure_enabled(false)
        .with_pressure_oversampling(Oversampling::X1)
        .with_temperature_oversampling(Oversampling::X128);
    assert_eq!(c.osr_config(), 0x07);
    // press_en, osr_p 128x = 0b111 << 3 = 0x38, osr_t 64x = 0b110.
    let c = Config::new()
        .with_pressure_oversampling(Oversampling::X128)
        .with_temperature_oversampling(Oversampling::X64);
    assert_eq!(c.osr_config(), 0x40 | 0x38 | 0x06);
}

#[test]
fn image_dsp_iir_and_dsp_config() {
    // 7.28 (p62): set_iir_p = 127 (0b111) in bits 5:3 = 0x38, set_iir_t = 63 (0b110) in bits 2:0.
    let c = Config::new()
        .with_pressure_iir(IirFilter::Coeff127)
        .with_temperature_iir(IirFilter::Coeff63);
    assert_eq!(c.dsp_iir(), 0x3E);
    // 7.27 (p61): Auto data taps: shdw_sel_iir_p bit 5 = 0x20, shdw_sel_iir_t bit 3 = 0x08, over 0x03.
    assert_eq!(c.dsp_config(), 0x2B);
    // Pressure filter only.
    let c = Config::new().with_pressure_iir(IirFilter::Coeff1);
    assert_eq!(c.dsp_iir(), 0x08);
    assert_eq!(c.dsp_config(), 0x23);
    // Every tap filtered with the filters bypassed, and the forced flush (bit 2): 0x03 + 0xF8 + 0x04.
    let all = IirTaps::new()
        .with_data_pressure(IirTap::Filtered)
        .with_data_temperature(IirTap::Filtered)
        .with_fifo_pressure(IirTap::Filtered)
        .with_fifo_temperature(IirTap::Filtered)
        .with_out_of_range_pressure(IirTap::Filtered);
    let c = Config::new().with_iir_taps(all).with_iir_flush_forced(true);
    assert_eq!(c.dsp_config(), 0xFF);
    // Each tap alone: fifo_sel_iir_t bit 4, fifo_sel_iir_p bit 6, oor_sel_iir_p bit 7.
    let one = |t: IirTaps| Config::new().with_iir_taps(t).dsp_config();
    assert_eq!(
        one(IirTaps::new().with_fifo_temperature(IirTap::Filtered)),
        0x13
    );
    assert_eq!(
        one(IirTaps::new().with_fifo_pressure(IirTap::Filtered)),
        0x43
    );
    assert_eq!(
        one(IirTaps::new().with_out_of_range_pressure(IirTap::Filtered)),
        0x83
    );
    // Unfiltered wins over a filter that is on.
    let c = Config::new()
        .with_pressure_iir(IirFilter::Coeff3)
        .with_temperature_iir(IirFilter::Coeff3)
        .with_iir_taps(
            IirTaps::new()
                .with_data_pressure(IirTap::Unfiltered)
                .with_data_temperature(IirTap::Unfiltered),
        );
    assert_eq!(c.dsp_config(), 0x03);
}

#[test]
fn image_interrupt() {
    // 7.6 (p54): drdy 0x01, fifo_full 0x02, fifo_ths 0x04, oor_p 0x08.
    let src =
        |s: InterruptSources| Config::new().with_interrupt(InterruptPin::new().with_sources(s));
    let none = InterruptSources::none();
    assert_eq!(src(none).int_source(), 0x00);
    assert_eq!(src(none.with_fifo_full(true)).int_source(), 0x02);
    assert_eq!(src(none.with_fifo_threshold(true)).int_source(), 0x04);
    assert_eq!(src(none.with_out_of_range(true)).int_source(), 0x08);
    assert_eq!(src(none.with_data_ready(true)).int_source(), 0x01);
    assert_eq!(
        src(InterruptSources::new()
            .with_fifo_full(true)
            .with_fifo_threshold(true)
            .with_out_of_range(true))
        .int_source(),
        0x0F
    );
    // 7.5 (p52-53): mode bit 0, pol bit 1, od bit 2, en bit 3, over the pad drive 0x30.
    let pin = |p: InterruptPin| Config::new().with_interrupt(p).int_config();
    assert_eq!(
        pin(InterruptPin::new()
            .with_mode(InterruptMode::Latched)
            .with_polarity(InterruptPolarity::ActiveLow)
            .with_drive(InterruptDrive::OpenDrain)),
        0x3D
    );
    assert_eq!(
        pin(InterruptPin::new().with_mode(InterruptMode::Latched)),
        0x3B
    );
    assert_eq!(
        pin(InterruptPin::new()
            .with_polarity(InterruptPolarity::ActiveLow)
            .with_enabled(false)),
        0x30
    );
    assert_eq!(pin(InterruptPin::off()), 0x32);
}

#[test]
fn image_fifo() {
    // 7.7 (p54): threshold bits 4:0, fifo_mode bit 5. 7.9 (p55): frame_sel bits 1:0, dec_sel bits 4:2.
    let f = FifoConfig::new()
        .with_mode(FifoMode::Stream)
        .with_data(FifoData::PressureAndTemperature)
        .with_threshold(15);
    let c = Config::new().with_fifo(f);
    assert_eq!((c.fifo_config(), c.fifo_sel()), (0x0F, 0x03));
    let f = FifoConfig::new()
        .with_mode(FifoMode::StopOnFull)
        .with_data(FifoData::Pressure)
        .with_threshold(31)
        .with_decimation(FifoDecimation::By128);
    let c = Config::new().with_fifo(f);
    // 0x20 | 0x1F ; frame 0b10 | (7 << 2).
    assert_eq!((c.fifo_config(), c.fifo_sel()), (0x3F, 0x1E));
    let f = FifoConfig::new()
        .with_mode(FifoMode::Stream)
        .with_data(FifoData::Temperature)
        .with_threshold(1)
        .with_decimation(FifoDecimation::By4);
    let c = Config::new().with_fifo(f);
    assert_eq!((c.fifo_config(), c.fifo_sel()), (0x01, 0x09));
    // Off: the other FIFO settings are not written.
    let f = FifoConfig::new()
        .with_threshold(9)
        .with_decimation(FifoDecimation::By8);
    let c = Config::new().with_fifo(f);
    assert_eq!((c.fifo_config(), c.fifo_sel()), (0x00, 0x00));
}

#[test]
fn image_out_of_range() {
    // 4.7.2.3 (p26) worked example: 97150 Pa = 0x17B7E, window 50 Pa = 0x32. 7.32 (p63): oor_thr_p_16
    // bit 0, cnt_lim bits 7:6 (0b11 = 15 samples, 0b01 = 3 samples).
    let w = OutOfRange::new()
        .with_reference_pa(97_150)
        .with_range_pa(50)
        .with_count(OutOfRangeCount::Fifteen);
    let c = Config::new().with_out_of_range(Some(w));
    assert_eq!(c.oor_thr_p_lsb(), 0x7E);
    assert_eq!(c.oor_thr_p_msb(), 0x7B);
    assert_eq!(c.oor_range(), 0x32);
    assert_eq!(c.oor_config(), 0xC1);
    let c = Config::new().with_out_of_range(Some(w.with_count(OutOfRangeCount::Three)));
    assert_eq!(c.oor_config(), 0x41);
    let w = OutOfRange::new()
        .with_reference_pa(0x1_FFFF)
        .with_range_pa(255)
        .with_count(OutOfRangeCount::Seven);
    let c = Config::new().with_out_of_range(Some(w));
    assert_eq!(
        [
            c.oor_thr_p_lsb(),
            c.oor_thr_p_msb(),
            c.oor_range(),
            c.oor_config()
        ],
        [0xFF, 0xFF, 0xFF, 0x81]
    );
}

// ---------------------------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------------------------

#[test]
fn validate_default_and_every_error() {
    assert_eq!(Config::default().validate(), Ok(()));
    assert_eq!(Config::new(), Config::default());

    // Table 7 (p17): osr_p 16x, osr_t 2x allows 80 Hz at most.
    let c = Config::new().with_odr(OutputDataRate::Hz240);
    assert_eq!(
        c.validate(),
        Err(ConfigError::OdrTooHighForOversampling {
            max: OutputDataRate::Hz80
        })
    );
    // Table 8 (p17): temperature only, osr_t 128x allows 40 Hz.
    let c = Config::new()
        .with_pressure_enabled(false)
        .with_temperature_oversampling(Oversampling::X128)
        .with_odr(OutputDataRate::Hz50_056);
    assert_eq!(
        c.validate(),
        Err(ConfigError::OdrTooHighForOversampling {
            max: OutputDataRate::Hz40
        })
    );

    // 4.6.1 (p23): at most 15 in PT mode, 31 otherwise.
    let fifo = FifoConfig::new().with_mode(FifoMode::Stream);
    let c = Config::new().with_fifo(fifo.with_threshold(16));
    assert_eq!(
        c.validate(),
        Err(ConfigError::FifoThresholdTooHigh { max: 15 })
    );
    let c = Config::new().with_fifo(fifo.with_data(FifoData::Pressure).with_threshold(32));
    assert_eq!(
        c.validate(),
        Err(ConfigError::FifoThresholdTooHigh { max: 31 })
    );
    let c = Config::new().with_fifo(fifo.with_data(FifoData::Pressure).with_threshold(31));
    assert_eq!(c.validate(), Ok(()));

    // 4.4.1 (p18): a pressure FIFO makes the chip measure pressure.
    let c = Config::new()
        .with_pressure_enabled(false)
        .with_fifo(fifo.with_data(FifoData::Pressure));
    assert_eq!(c.validate(), Err(ConfigError::FifoPressureWithoutPressure));
    let c = Config::new()
        .with_pressure_enabled(false)
        .with_fifo(fifo.with_data(FifoData::Temperature));
    assert_eq!(c.validate(), Ok(()));

    // 4.7.2.1 (p26): FIFO interrupts only occur with the FIFO enabled.
    let src = InterruptSources::none();
    let pin = |s| InterruptPin::new().with_sources(s);
    let c = Config::new().with_interrupt(pin(src.with_fifo_full(true)));
    assert_eq!(c.validate(), Err(ConfigError::FifoInterruptWithoutFifo));
    let c = Config::new().with_interrupt(pin(src.with_fifo_threshold(true)));
    assert_eq!(c.validate(), Err(ConfigError::FifoInterruptWithoutFifo));
    // 4.6.1 (p23): threshold 0 disables the threshold.
    let c = Config::new()
        .with_fifo(fifo)
        .with_interrupt(pin(src.with_fifo_threshold(true)));
    assert_eq!(
        c.validate(),
        Err(ConfigError::FifoThresholdInterruptWithoutThreshold)
    );
    let c = Config::new()
        .with_fifo(fifo.with_threshold(4))
        .with_interrupt(pin(src.with_fifo_threshold(true).with_fifo_full(true)));
    assert_eq!(c.validate(), Ok(()));

    // 4.7.2.3 (p26): out of range needs a window and pressure.
    let c = Config::new().with_interrupt(pin(src.with_out_of_range(true)));
    assert_eq!(
        c.validate(),
        Err(ConfigError::OutOfRangeInterruptWithoutWindow)
    );
    let w = OutOfRange::new()
        .with_reference_pa(100_000)
        .with_range_pa(50);
    let c = Config::new()
        .with_pressure_enabled(false)
        .with_out_of_range(Some(w))
        .with_interrupt(pin(src.with_out_of_range(true)));
    assert_eq!(
        c.validate(),
        Err(ConfigError::OutOfRangeInterruptWithoutPressure)
    );
    let c = Config::new()
        .with_out_of_range(Some(w))
        .with_interrupt(pin(src.with_out_of_range(true)));
    assert_eq!(c.validate(), Ok(()));
    // The reference has 17 bits (p26).
    let c = Config::new().with_out_of_range(Some(w.with_reference_pa(0x2_0000)));
    assert_eq!(c.validate(), Err(ConfigError::OutOfRangeReferenceTooHigh));
    let c = Config::new().with_out_of_range(Some(w.with_reference_pa(0x1_FFFF)));
    assert_eq!(c.validate(), Ok(()));
}

#[test]
fn validate_odr_limit_only_in_normal_mode() {
    // 4.3.4 (p17): "This action of alignment and check is done in NORMAL mode only."
    for mode in [PowerMode::Standby, PowerMode::Forced, PowerMode::Continuous] {
        let c = Config::new()
            .with_pressure_oversampling(Oversampling::X128)
            .with_temperature_oversampling(Oversampling::X128)
            .with_odr(OutputDataRate::Hz240)
            .with_power_mode(mode);
        assert_eq!(c.validate(), Ok(()));
    }
}

#[test]
fn validate_table_matches_literal_datasheet_table() {
    for pressure in [true, false] {
        for (p, &osr_p) in ALL_OSR.iter().enumerate() {
            for (t, &osr_t) in ALL_OSR.iter().enumerate() {
                for (i, &odr) in ALL_ODR.iter().enumerate() {
                    let c = Config::new()
                        .with_pressure_enabled(pressure)
                        .with_pressure_oversampling(osr_p)
                        .with_temperature_oversampling(osr_t)
                        .with_odr(odr)
                        .with_power_mode(PowerMode::Normal);
                    assert_eq!(
                        c.validate().is_ok(),
                        table_allows(pressure, p, t, i),
                        "pressure {pressure} p {p} t {t} odr {i}"
                    );
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------------------------

fn w(reg: u8, value: u8) -> Step {
    Step::Write { reg, value }
}

fn r(reg: u8) -> Step {
    Step::Read { reg }
}

const CHECK: Step = Step::CheckRateValid;

#[test]
fn plan_default_after_reset_is_literal_list() {
    // Registers: INT_STATUS 0x27, OSR_CONFIG 0x36, ODR_CONFIG 0x37, DSP_CONFIG 0x30, DSP_IIR 0x31,
    // INT_SOURCE 0x15, INT_CONFIG 0x14. This is the sequence `init` makes after its identity and NVM
    // checks (blocking_init_then_read in tests.rs), the two INT_STATUS reads included. It differs
    // from `init` only in what it leaves out: the soft reset, its wait, the CHIP_ID and STATUS
    // checks stay in `init`. The OSR_EFF check is the plan step after the standby ODR_CONFIG write.
    let plan = Config::default().plan(Apply::AfterReset).unwrap();
    assert_eq!(
        plan.steps(),
        [
            r(0x27),
            w(0x36, 0x61),
            w(0x37, 0x3C),
            CHECK,
            w(0x30, 0x03),
            w(0x31, 0x00),
            w(0x15, 0x01),
            w(0x14, 0x3A),
            r(0x27),
            w(0x37, 0x3D),
        ]
    );
    assert_eq!(plan.len(), 10);
    assert!(!plan.is_empty());
    assert_eq!(plan.iter().count(), 10);
    assert_eq!((&plan).into_iter().count(), 10);
}

#[test]
fn plan_default_in_place_is_literal_list() {
    // 4.3.7 (p18): standby, then wait tstandby (2.5 ms, p11) before writing; 4.7.3.2 (p28): sources off,
    // read INT_STATUS, then the mode.
    let plan = Config::default().plan(Apply::InPlace).unwrap();
    assert_eq!(STANDBY_WAIT_US, 2_500);
    assert_eq!(
        plan.steps(),
        [
            w(0x37, 0x3C),
            Step::DelayUs(2_500),
            w(0x36, 0x61),
            w(0x37, 0x3C),
            CHECK,
            w(0x30, 0x03),
            w(0x31, 0x00),
            w(0x18, 0x00),
            w(0x16, 0x00),
            w(0x32, 0x00),
            w(0x33, 0x00),
            w(0x34, 0x00),
            w(0x35, 0x00),
            w(0x15, 0x00),
            r(0x27),
            w(0x14, 0x3A),
            w(0x15, 0x01),
            r(0x27),
            w(0x37, 0x3D),
        ]
    );
    assert_eq!(plan.len(), 19);
    assert!(plan.len() <= MAX_STEPS);
}

#[test]
fn plan_with_fifo_out_of_range_and_forced() {
    let fifo = FifoConfig::new()
        .with_mode(FifoMode::Stream)
        .with_threshold(8);
    let w_oor = OutOfRange::new()
        .with_reference_pa(97_150)
        .with_range_pa(50)
        .with_count(OutOfRangeCount::Fifteen);
    let c = Config::new()
        .with_fifo(fifo)
        .with_out_of_range(Some(w_oor))
        .with_power_mode(PowerMode::Forced)
        .with_interrupt(
            InterruptPin::new().with_sources(
                InterruptSources::none()
                    .with_fifo_threshold(true)
                    .with_out_of_range(true),
            ),
        );
    let plan = c.plan(Apply::AfterReset).unwrap();
    assert_eq!(
        plan.steps(),
        [
            r(0x27),
            w(0x36, 0x61),
            w(0x37, 0x3C),
            w(0x30, 0x03),
            w(0x31, 0x00),
            w(0x18, 0x03),
            w(0x16, 0x08),
            w(0x32, 0x7E),
            w(0x33, 0x7B),
            w(0x34, 0x32),
            w(0x35, 0xC1),
            w(0x15, 0x0C),
            w(0x14, 0x3A),
            r(0x27),
        ]
    );
}

#[test]
fn plan_refuses_an_invalid_config_and_overflows_loudly() {
    let bad = Config::new().with_odr(OutputDataRate::Hz240);
    assert!(matches!(
        bad.plan(Apply::AfterReset),
        Err(ConfigError::OdrTooHighForOversampling { .. })
    ));
    let mut p = Plan::empty();
    assert!(p.is_empty());
    for i in 0..MAX_STEPS {
        assert_eq!(p.push(Step::DelayUs(i as u32)), Ok(()));
    }
    assert_eq!(p.push(Step::DelayUs(99)), Err(ConfigError::PlanOverflow));
    assert_eq!(p.len(), MAX_STEPS);
    assert_eq!(p.steps().last(), Some(&Step::DelayUs(MAX_STEPS as u32 - 1)));
}

// ---------------------------------------------------------------------------------------------
// Helpers and const evaluation
// ---------------------------------------------------------------------------------------------

#[test]
fn helpers_conversion_time_period_and_fifo() {
    // Table 4 (p12), typical: pressure 16x 10.4 ms + temperature 2x 1.1 ms.
    assert_eq!(
        conversion_time_us(Oversampling::X16, Oversampling::X2),
        11_500
    );
    assert_eq!(
        conversion_time_us(Oversampling::X1, Oversampling::X1),
        2_000
    );
    assert_eq!(
        conversion_time_us(Oversampling::X128, Oversampling::X128),
        101_200
    );
    assert_eq!(pressure_conversion_time_us(Oversampling::X64), 40_400);
    assert_eq!(temperature_conversion_time_us(Oversampling::X32), 5_800);
    // Nominal periods: 1e9 / rate in mHz, rounded (7.34, p65).
    assert_eq!(OutputDataRate::Hz240.period_us(), 4_167);
    assert_eq!(OutputDataRate::Hz218_537.period_us(), 4_576);
    assert_eq!(OutputDataRate::Hz100_299.period_us(), 9_970);
    assert_eq!(OutputDataRate::Hz50_056.period_us(), 19_978);
    assert_eq!(OutputDataRate::Hz25_005.period_us(), 39_992);
    assert_eq!(OutputDataRate::Hz1.period_us(), 1_000_000);
    assert_eq!(OutputDataRate::Hz0_5.period_us(), 2_000_000);
    assert_eq!(OutputDataRate::Hz0_125.period_us(), 8_000_000);
    // 4.6 (p22) and 4.6.3 (p23), Tables 11 to 13 (p23-24).
    assert_eq!(FifoData::PressureAndTemperature.capacity_frames(), 16);
    assert_eq!(FifoData::Pressure.capacity_frames(), 32);
    assert_eq!(FifoData::Temperature.capacity_frames(), 32);
    assert_eq!(FifoData::PressureAndTemperature.frame_bytes(), 6);
    assert_eq!(FifoData::Pressure.frame_bytes(), 3);
    assert_eq!(FifoData::Temperature.frame_bytes(), 3);
    assert_eq!(FifoData::PressureAndTemperature.max_threshold(), 15);
    assert_eq!(FifoData::Pressure.max_threshold(), 31);
    // Table 7 and 8 lookups.
    assert_eq!(
        max_odr(true, Oversampling::X16, Oversampling::X2),
        OutputDataRate::Hz80
    );
    assert_eq!(
        max_odr(false, Oversampling::X128, Oversampling::X1),
        OutputDataRate::Hz240
    );
}

const CONST_CONFIG: Config = Config::new()
    .with_pressure_oversampling(Oversampling::X1)
    .with_temperature_oversampling(Oversampling::X1)
    .with_odr(OutputDataRate::Hz240)
    .with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::StopOnFull)
            .with_data(FifoData::Pressure)
            .with_threshold(31),
    )
    .with_out_of_range(Some(
        OutOfRange::new()
            .with_reference_pa(97_150)
            .with_range_pa(50)
            .with_count(OutOfRangeCount::Fifteen),
    ));
const CONST_OSR: u8 = CONST_CONFIG.osr_config();
const CONST_ODR: u8 = CONST_CONFIG.odr_config(PowerMode::Normal);
const CONST_FIFO: (u8, u8) = (CONST_CONFIG.fifo_config(), CONST_CONFIG.fifo_sel());
const CONST_OOR: u8 = CONST_CONFIG.oor_config();
const CONST_PERIOD: u32 = OutputDataRate::Hz240.period_us();

#[test]
fn const_evaluated_config() {
    // 0x40 | 0 | 0 ; odr 0 | normal ; 0x20|0x1F, frame 0b10 ; 0xC1 ; 4167 us.
    assert_eq!(CONST_OSR, 0x40);
    assert_eq!(CONST_ODR, 0x01);
    assert_eq!(CONST_FIFO, (0x3F, 0x02));
    assert_eq!(CONST_OOR, 0xC1);
    assert_eq!(CONST_PERIOD, 4_167);
    assert_eq!(CONST_CONFIG.validate(), Ok(()));
}

// ---------------------------------------------------------------------------------------------
// Properties
// ---------------------------------------------------------------------------------------------

fn pick<T: Clone + core::fmt::Debug + 'static>(items: &'static [T]) -> impl Strategy<Value = T> {
    proptest::sample::select(items)
}

fn any_pin() -> impl Strategy<Value = InterruptPin> {
    (
        any::<[bool; 4]>(),
        any::<bool>(),
        any::<bool>(),
        any::<bool>(),
        any::<bool>(),
    )
        .prop_map(|(s, latched, high, od, en)| {
            InterruptPin::new()
                .with_sources(
                    InterruptSources::none()
                        .with_data_ready(s[0])
                        .with_fifo_full(s[1])
                        .with_fifo_threshold(s[2])
                        .with_out_of_range(s[3]),
                )
                .with_mode(if latched {
                    InterruptMode::Latched
                } else {
                    InterruptMode::Pulsed
                })
                .with_polarity(if high {
                    InterruptPolarity::ActiveHigh
                } else {
                    InterruptPolarity::ActiveLow
                })
                .with_drive(if od {
                    InterruptDrive::OpenDrain
                } else {
                    InterruptDrive::PushPull
                })
                .with_enabled(en)
        })
}

fn any_tap() -> impl Strategy<Value = IirTap> {
    pick(&[IirTap::Auto, IirTap::Unfiltered, IirTap::Filtered])
}

fn any_fifo() -> impl Strategy<Value = FifoConfig> {
    (
        pick(&[FifoMode::Off, FifoMode::Stream, FifoMode::StopOnFull]),
        pick(&[
            FifoData::Temperature,
            FifoData::Pressure,
            FifoData::PressureAndTemperature,
        ]),
        any::<u8>(),
        pick(&ALL_DEC),
    )
        .prop_map(|(m, d, t, dec)| {
            FifoConfig::new()
                .with_mode(m)
                .with_data(d)
                .with_threshold(t)
                .with_decimation(dec)
        })
}

fn any_window() -> impl Strategy<Value = Option<OutOfRange>> {
    proptest::option::of(
        (
            0u32..0x4_0000,
            any::<u8>(),
            pick(&[
                OutOfRangeCount::One,
                OutOfRangeCount::Three,
                OutOfRangeCount::Seven,
                OutOfRangeCount::Fifteen,
            ]),
        )
            .prop_map(|(r, g, c)| {
                OutOfRange::new()
                    .with_reference_pa(r)
                    .with_range_pa(g)
                    .with_count(c)
            }),
    )
}

fn any_config() -> impl Strategy<Value = Config> {
    (
        (
            pick(&ALL_OSR),
            pick(&ALL_OSR),
            any::<bool>(),
            pick(&ALL_ODR),
            pick(&ALL_IIR),
            pick(&ALL_IIR),
        ),
        (
            (any_tap(), any_tap(), any_tap(), any_tap(), any_tap()),
            any::<bool>(),
            pick(&ALL_MODES),
        ),
        (any_pin(), any_fifo(), any_window()),
    )
        .prop_map(
            |((op, ot, pe, odr, ip, it), ((a, b, c, d, e), flush, mode), (pin, fifo, win))| {
                Config::new()
                    .with_pressure_oversampling(op)
                    .with_temperature_oversampling(ot)
                    .with_pressure_enabled(pe)
                    .with_odr(odr)
                    .with_pressure_iir(ip)
                    .with_temperature_iir(it)
                    .with_iir_taps(
                        IirTaps::new()
                            .with_data_pressure(a)
                            .with_data_temperature(b)
                            .with_fifo_pressure(c)
                            .with_fifo_temperature(d)
                            .with_out_of_range_pressure(e),
                    )
                    .with_iir_flush_forced(flush)
                    .with_power_mode(mode)
                    .with_interrupt(pin)
                    .with_fifo(fifo)
                    .with_out_of_range(win)
            },
        )
}

/// The rules of the datasheet written once more, independently of `validate`, with the literal tables.
fn model_ok(c: &Config) -> bool {
    let fifo_on = c.fifo.mode != FifoMode::Off;
    let fifo_p = fifo_on && c.fifo.data != FifoData::Temperature;
    let cap_thr = if c.fifo.data == FifoData::PressureAndTemperature {
        15
    } else {
        31
    };
    let s = c.interrupt.sources;
    let mut ok = true;
    ok &= !(fifo_p && !c.pressure_enabled);
    ok &= !(fifo_on && c.fifo.threshold > cap_thr);
    ok &= !((s.fifo_full || s.fifo_threshold) && !fifo_on);
    ok &= !(s.fifo_threshold && c.fifo.threshold == 0);
    ok &= !(s.out_of_range && (c.out_of_range.is_none() || !c.pressure_enabled));
    ok &= c.out_of_range.is_none_or(|w| w.reference_pa < 0x2_0000);
    if c.power_mode == PowerMode::Normal {
        let p = ALL_OSR
            .iter()
            .position(|&o| o == c.pressure_oversampling)
            .unwrap();
        let t = ALL_OSR
            .iter()
            .position(|&o| o == c.temperature_oversampling)
            .unwrap();
        let i = ALL_ODR.iter().position(|&o| o == c.odr).unwrap();
        ok &= table_allows(c.pressure_enabled, p, t, i);
    }
    ok
}

proptest! {
    #[test]
    fn prop_images_keep_reserved_bits_at_reset(c in any_config()) {
        // Reset values and reserved fields: 7.33 (p64) bit 7; 7.34 (p64-65) deep_dis reset 0 (bit 7);
        // 7.27 (p61) comp_pt_en reset 0b11; 7.28 (p62) bits 7:6; 7.5 (p52) pad drive reset 3 (bits 7:4);
        // 7.6 (p54) bits 7:4; 7.7 (p54) bits 7:6; 7.9 (p55) bits 7:5; 7.32 (p63) bits 5:1.
        prop_assert_eq!(c.osr_config() & 0x80, 0);
        for m in ALL_MODES {
            prop_assert_eq!(c.odr_config(m) & 0x80, 0);
            prop_assert_eq!(c.odr_config(m) & 0b11, m.code());
            prop_assert_eq!(c.odr_config(m) >> 2, c.odr.code());
        }
        prop_assert_eq!(c.dsp_config() & 0b11, 0b11);
        prop_assert_eq!(c.dsp_iir() & 0xC0, 0);
        prop_assert_eq!(c.int_config() & 0xF0, 0x30);
        prop_assert_eq!(c.int_source() & 0xF0, 0);
        prop_assert_eq!(c.fifo_config() & 0xC0, 0);
        prop_assert_eq!(c.fifo_sel() & 0xE0, 0);
        prop_assert_eq!(c.oor_config() & 0x3E, 0);
    }

    #[test]
    fn prop_validate_matches_the_model(c in any_config()) {
        prop_assert_eq!(c.validate().is_ok(), model_ok(&c));
    }

    #[test]
    fn prop_plan_exists_exactly_when_valid_and_fits(c in any_config()) {
        for apply in [Apply::AfterReset, Apply::InPlace] {
            match c.plan(apply) {
                Ok(p) => {
                    prop_assert!(c.validate().is_ok());
                    prop_assert!(p.len() <= MAX_STEPS);
                    // The last write of the power mode is the last step unless forced; no register is
                    // written in the plan with a different image than the config's.
                    let steps: Vec<Step> = p.steps().to_vec();
                    prop_assert!(steps.contains(&w(0x36, c.osr_config())));
                    // The validity check exists in normal mode only, right after the standby rate write.
                    let at = steps.iter().position(|s| *s == CHECK);
                    prop_assert_eq!(at.is_some(), c.power_mode == PowerMode::Normal);
                    if let Some(at) = at {
                        prop_assert_eq!(steps[at - 1], w(0x37, c.odr_config(PowerMode::Standby)));
                    }
                    if c.power_mode != PowerMode::Forced {
                        prop_assert_eq!(steps.last(), Some(&w(0x37, c.odr_config(c.power_mode))));
                    }
                }
                Err(e) => {
                    prop_assert!(c.validate().is_err());
                    prop_assert_eq!(Err(e), c.validate());
                }
            }
        }
    }
}
