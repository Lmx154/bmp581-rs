//! The configuration types and their validation. Pure code, no bus access. The register images are in
//! `image`, the write plan in `plan`, the helpers in `calc`.

/// Oversampling ratio, `OSR_CONFIG.osr_p` and `osr_t`, same codes for both (7.33, p64).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Oversampling {
    /// 1x.
    X1 = 0,
    /// 2x.
    X2 = 1,
    /// 4x.
    X4 = 2,
    /// 8x.
    X8 = 3,
    /// 16x.
    X16 = 4,
    /// 32x.
    X32 = 5,
    /// 64x.
    X64 = 6,
    /// 128x.
    X128 = 7,
}

impl Oversampling {
    /// Register code (3 bits).
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// The ratio for the low three bits of `code`; the upper bits are ignored.
    pub(crate) const fn from_code(code: u8) -> Self {
        match code & 0b111 {
            0 => Self::X1,
            1 => Self::X2,
            2 => Self::X4,
            3 => Self::X8,
            4 => Self::X16,
            5 => Self::X32,
            6 => Self::X64,
            _ => Self::X128,
        }
    }
}

/// Output data rate, `ODR_CONFIG.odr` (7.34, p65). Used in normal mode; the name is the nominal rate in Hz.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum OutputDataRate {
    /// 240.000 Hz, code 0x00.
    Hz240 = 0x00,
    /// 218.537 Hz, code 0x01.
    Hz218_537 = 0x01,
    /// 199.111 Hz, code 0x02.
    Hz199_111 = 0x02,
    /// 179.200 Hz, code 0x03.
    Hz179_2 = 0x03,
    /// 160.000 Hz, code 0x04.
    Hz160 = 0x04,
    /// 149.333 Hz, code 0x05.
    Hz149_333 = 0x05,
    /// 140.000 Hz, code 0x06.
    Hz140 = 0x06,
    /// 129.855 Hz, code 0x07.
    Hz129_855 = 0x07,
    /// 120.000 Hz, code 0x08.
    Hz120 = 0x08,
    /// 110.164 Hz, code 0x09.
    Hz110_164 = 0x09,
    /// 100.299 Hz, code 0x0A.
    Hz100_299 = 0x0A,
    /// 89.600 Hz, code 0x0B.
    Hz89_6 = 0x0B,
    /// 80.000 Hz, code 0x0C.
    Hz80 = 0x0C,
    /// 70.000 Hz, code 0x0D.
    Hz70 = 0x0D,
    /// 60.000 Hz, code 0x0E.
    Hz60 = 0x0E,
    /// 50.056 Hz, code 0x0F.
    Hz50_056 = 0x0F,
    /// 45.025 Hz, code 0x10.
    Hz45_025 = 0x10,
    /// 40.000 Hz, code 0x11.
    Hz40 = 0x11,
    /// 35.000 Hz, code 0x12.
    Hz35 = 0x12,
    /// 30.000 Hz, code 0x13.
    Hz30 = 0x13,
    /// 25.005 Hz, code 0x14.
    Hz25_005 = 0x14,
    /// 20.000 Hz, code 0x15.
    Hz20 = 0x15,
    /// 15.000 Hz, code 0x16.
    Hz15 = 0x16,
    /// 10.000 Hz, code 0x17.
    Hz10 = 0x17,
    /// 5.000 Hz, code 0x18.
    Hz5 = 0x18,
    /// 4.000 Hz, code 0x19.
    Hz4 = 0x19,
    /// 3.000 Hz, code 0x1A.
    Hz3 = 0x1A,
    /// 2.000 Hz, code 0x1B.
    Hz2 = 0x1B,
    /// 1.000 Hz, code 0x1C.
    Hz1 = 0x1C,
    /// 0.500 Hz, code 0x1D.
    Hz0_5 = 0x1D,
    /// 0.250 Hz, code 0x1E.
    Hz0_25 = 0x1E,
    /// 0.125 Hz, code 0x1F.
    Hz0_125 = 0x1F,
}

impl OutputDataRate {
    /// Register code (5 bits).
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Nominal rate in hertz, as listed in the datasheet.
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz240 => 240.0,
            Self::Hz218_537 => 218.537,
            Self::Hz199_111 => 199.111,
            Self::Hz179_2 => 179.2,
            Self::Hz160 => 160.0,
            Self::Hz149_333 => 149.333,
            Self::Hz140 => 140.0,
            Self::Hz129_855 => 129.855,
            Self::Hz120 => 120.0,
            Self::Hz110_164 => 110.164,
            Self::Hz100_299 => 100.299,
            Self::Hz89_6 => 89.6,
            Self::Hz80 => 80.0,
            Self::Hz70 => 70.0,
            Self::Hz60 => 60.0,
            Self::Hz50_056 => 50.056,
            Self::Hz45_025 => 45.025,
            Self::Hz40 => 40.0,
            Self::Hz35 => 35.0,
            Self::Hz30 => 30.0,
            Self::Hz25_005 => 25.005,
            Self::Hz20 => 20.0,
            Self::Hz15 => 15.0,
            Self::Hz10 => 10.0,
            Self::Hz5 => 5.0,
            Self::Hz4 => 4.0,
            Self::Hz3 => 3.0,
            Self::Hz2 => 2.0,
            Self::Hz1 => 1.0,
            Self::Hz0_5 => 0.5,
            Self::Hz0_25 => 0.25,
            Self::Hz0_125 => 0.125,
        }
    }
}

/// IIR filter coefficient, `DSP_IIR.set_iir_p` and `set_iir_t` (7.28, p62; Table 10, p21).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum IirFilter {
    /// Filter off.
    Bypass = 0,
    /// Coefficient 1.
    Coeff1 = 1,
    /// Coefficient 3.
    Coeff3 = 2,
    /// Coefficient 7.
    Coeff7 = 3,
    /// Coefficient 15.
    Coeff15 = 4,
    /// Coefficient 31.
    Coeff31 = 5,
    /// Coefficient 63.
    Coeff63 = 6,
    /// Coefficient 127.
    Coeff127 = 7,
}

impl IirFilter {
    /// Register code (3 bits).
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Power mode, `ODR_CONFIG.pwr_mode` (7.34, p64-65).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PowerMode {
    /// No measurement.
    Standby = 0,
    /// Periodic measurement at the output data rate.
    Normal = 1,
    /// One measurement, then back to standby. See [`Bmp581::trigger_forced`](crate::Bmp581::trigger_forced).
    Forced = 2,
    /// Back-to-back measurement, the output data rate is ignored (datasheet "non-stop").
    Continuous = 3,
}

impl PowerMode {
    /// Register code (2 bits).
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Which of the two points of the IIR filter a data path takes its value from (7.27, p61).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IirTap {
    /// After the filter exactly when the filter of that channel is not bypassed. What `init` has always done
    /// for the data registers.
    #[default]
    Auto,
    /// Before the filter (bit clear).
    Unfiltered,
    /// After the filter (bit set).
    Filtered,
}

/// The source selection of each data path, `DSP_CONFIG` bits 7:3 (7.27, p61; 4.4.4, p22).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct IirTaps {
    /// Pressure data register (`shdw_sel_iir_p`).
    pub data_pressure: IirTap,
    /// Temperature data register (`shdw_sel_iir_t`).
    pub data_temperature: IirTap,
    /// Pressure in the FIFO (`fifo_sel_iir_p`).
    pub fifo_pressure: IirTap,
    /// Temperature in the FIFO (`fifo_sel_iir_t`).
    pub fifo_temperature: IirTap,
    /// Pressure the out-of-range check looks at (`oor_sel_iir_p`).
    pub out_of_range_pressure: IirTap,
}

impl IirTaps {
    /// The data registers [`IirTap::Auto`] (what `init` has always done), the FIFO and the out-of-range
    /// check [`IirTap::Unfiltered`] (the reset value, which `init` has always left).
    pub const fn new() -> Self {
        Self {
            data_pressure: IirTap::Auto,
            data_temperature: IirTap::Auto,
            fifo_pressure: IirTap::Unfiltered,
            fifo_temperature: IirTap::Unfiltered,
            out_of_range_pressure: IirTap::Unfiltered,
        }
    }

    /// Sets `data_pressure`.
    pub const fn with_data_pressure(mut self, v: IirTap) -> Self {
        self.data_pressure = v;
        self
    }

    /// Sets `data_temperature`.
    pub const fn with_data_temperature(mut self, v: IirTap) -> Self {
        self.data_temperature = v;
        self
    }

    /// Sets `fifo_pressure`.
    pub const fn with_fifo_pressure(mut self, v: IirTap) -> Self {
        self.fifo_pressure = v;
        self
    }

    /// Sets `fifo_temperature`.
    pub const fn with_fifo_temperature(mut self, v: IirTap) -> Self {
        self.fifo_temperature = v;
        self
    }

    /// Sets `out_of_range_pressure`.
    pub const fn with_out_of_range_pressure(mut self, v: IirTap) -> Self {
        self.out_of_range_pressure = v;
        self
    }
}

impl Default for IirTaps {
    fn default() -> Self {
        Self::new()
    }
}

/// INT pin mode, `INT_CONFIG.int_mode` (7.5, p52; 4.7.3.2, p27-28).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptMode {
    /// A pulse at each FALSE to TRUE change of a condition.
    Pulsed,
    /// Asserted as long as a condition holds.
    Latched,
}

/// INT pin polarity, `INT_CONFIG.int_pol` (7.5, p52).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptPolarity {
    /// Active low.
    ActiveLow,
    /// Active high.
    ActiveHigh,
}

/// INT pin output stage, `INT_CONFIG.int_od` (7.5, p53).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptDrive {
    /// Push-pull.
    PushPull,
    /// Open drain.
    OpenDrain,
}

/// The conditions routed to the INT pin and to `INT_STATUS`, `INT_SOURCE` (7.6, p54; 4.7.1, p25).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct InterruptSources {
    /// New data in the data registers (`drdy_data_reg_en`).
    pub data_ready: bool,
    /// FIFO full (`fifo_full_en`).
    pub fifo_full: bool,
    /// FIFO at or above its threshold (`fifo_ths_en`).
    pub fifo_threshold: bool,
    /// Pressure out of range (`oor_p_en`).
    pub out_of_range: bool,
}

impl InterruptSources {
    /// Data-ready only.
    pub const fn new() -> Self {
        Self {
            data_ready: true,
            fifo_full: false,
            fifo_threshold: false,
            out_of_range: false,
        }
    }

    /// No source.
    pub const fn none() -> Self {
        Self {
            data_ready: false,
            fifo_full: false,
            fifo_threshold: false,
            out_of_range: false,
        }
    }

    /// Sets `data_ready`.
    pub const fn with_data_ready(mut self, v: bool) -> Self {
        self.data_ready = v;
        self
    }

    /// Sets `fifo_full`.
    pub const fn with_fifo_full(mut self, v: bool) -> Self {
        self.fifo_full = v;
        self
    }

    /// Sets `fifo_threshold`.
    pub const fn with_fifo_threshold(mut self, v: bool) -> Self {
        self.fifo_threshold = v;
        self
    }

    /// Sets `out_of_range`.
    pub const fn with_out_of_range(mut self, v: bool) -> Self {
        self.out_of_range = v;
        self
    }

    /// True when at least one source is on.
    pub const fn any(&self) -> bool {
        self.data_ready || self.fifo_full || self.fifo_threshold || self.out_of_range
    }
}

impl Default for InterruptSources {
    fn default() -> Self {
        Self::new()
    }
}

/// The INT pin: what drives it and how (7.5, p52-53; 4.7.3, p27).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct InterruptPin {
    /// The conditions. They set their `INT_STATUS` bit whether or not the pin is enabled.
    pub sources: InterruptSources,
    /// Pulsed or latched.
    pub mode: InterruptMode,
    /// Active level.
    pub polarity: InterruptPolarity,
    /// Push-pull or open drain.
    pub drive: InterruptDrive,
    /// `INT_CONFIG.int_en`: the sources reach the pin.
    pub enabled: bool,
}

impl InterruptPin {
    /// Data-ready, pulsed, active high, push-pull, pin enabled: what `init` has always configured.
    pub const fn new() -> Self {
        Self {
            sources: InterruptSources::new(),
            mode: InterruptMode::Pulsed,
            polarity: InterruptPolarity::ActiveHigh,
            drive: InterruptDrive::PushPull,
            enabled: true,
        }
    }

    /// No source and the pin disabled.
    pub const fn off() -> Self {
        Self {
            sources: InterruptSources::none(),
            enabled: false,
            ..Self::new()
        }
    }

    /// Sets `sources`.
    pub const fn with_sources(mut self, v: InterruptSources) -> Self {
        self.sources = v;
        self
    }

    /// Sets `mode`.
    pub const fn with_mode(mut self, v: InterruptMode) -> Self {
        self.mode = v;
        self
    }

    /// Sets `polarity`.
    pub const fn with_polarity(mut self, v: InterruptPolarity) -> Self {
        self.polarity = v;
        self
    }

    /// Sets `drive`.
    pub const fn with_drive(mut self, v: InterruptDrive) -> Self {
        self.drive = v;
        self
    }

    /// Sets `enabled`.
    pub const fn with_enabled(mut self, v: bool) -> Self {
        self.enabled = v;
        self
    }
}

impl Default for InterruptPin {
    fn default() -> Self {
        Self::new()
    }
}

/// FIFO operation, `FIFO_SEL.fifo_frame_sel` and `FIFO_CONFIG.fifo_mode` (7.7, p54; 4.6.1, p22-23).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// FIFO not enabled. The other FIFO settings are not written.
    Off,
    /// Streaming: on overflow the oldest frame is dropped.
    Stream,
    /// Stop on full: on overflow new frames are not written.
    StopOnFull,
}

/// What a FIFO frame holds, `FIFO_SEL.fifo_frame_sel` (7.9, p55; Tables 11 to 13, p23-24).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoData {
    /// Temperature only, code 1.
    Temperature,
    /// Pressure only, code 2.
    Pressure,
    /// Pressure and temperature, code 3.
    PressureAndTemperature,
}

/// FIFO decimation, `FIFO_SEL.fifo_dec_sel`: one frame in 2^n is written (7.9, p55; 4.6.1, p23).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FifoDecimation {
    /// Every frame, n = 0.
    By1 = 0,
    /// One in 2.
    By2 = 1,
    /// One in 4.
    By4 = 2,
    /// One in 8.
    By8 = 3,
    /// One in 16.
    By16 = 4,
    /// One in 32.
    By32 = 5,
    /// One in 64.
    By64 = 6,
    /// One in 128.
    By128 = 7,
}

impl FifoDecimation {
    /// Register code (3 bits).
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// The FIFO (7.7 to 7.9, p54-55; 4.6, p22-25).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoConfig {
    /// Off, stream or stop on full.
    pub mode: FifoMode,
    /// What a frame holds.
    pub data: FifoData,
    /// Threshold in frames, 0 for none. At most 15 for pressure and temperature frames, 31 for the others.
    pub threshold: u8,
    /// Decimation.
    pub decimation: FifoDecimation,
}

impl FifoConfig {
    /// FIFO off.
    pub const fn new() -> Self {
        Self {
            mode: FifoMode::Off,
            data: FifoData::PressureAndTemperature,
            threshold: 0,
            decimation: FifoDecimation::By1,
        }
    }

    /// Sets `mode`.
    pub const fn with_mode(mut self, v: FifoMode) -> Self {
        self.mode = v;
        self
    }

    /// Sets `data`.
    pub const fn with_data(mut self, v: FifoData) -> Self {
        self.data = v;
        self
    }

    /// Sets `threshold`.
    pub const fn with_threshold(mut self, v: u8) -> Self {
        self.threshold = v;
        self
    }

    /// Sets `decimation`.
    pub const fn with_decimation(mut self, v: FifoDecimation) -> Self {
        self.decimation = v;
        self
    }
}

impl Default for FifoConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Samples out of range in a row before the interrupt, `OOR_CONFIG.cnt_lim` (7.32, p63).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum OutOfRangeCount {
    /// 1 sample, code 0.
    One = 0,
    /// 3 samples, code 1.
    Three = 1,
    /// 7 samples, code 2.
    Seven = 2,
    /// 15 samples, code 3.
    Fifteen = 3,
}

impl OutOfRangeCount {
    /// Register code (2 bits).
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// The out-of-range window: out of range when the pressure is above `reference_pa + range_pa` or
/// below `reference_pa - range_pa` (4.7.2.3, p26; 7.29 to 7.32, p63).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct OutOfRange {
    /// Reference pressure in pascals, 17 bits (at most [`OutOfRange::MAX_REFERENCE_PA`]).
    pub reference_pa: u32,
    /// Half width of the window in pascals, 0 to 255.
    pub range_pa: u8,
    /// Samples out of range in a row.
    pub count: OutOfRangeCount,
}

impl OutOfRange {
    /// Largest reference pressure the 17-bit field holds (4.7.2.3, p26).
    pub const MAX_REFERENCE_PA: u32 = 0x1_FFFF;

    /// Reference 0 Pa, range 0 Pa, count 1: the register reset values.
    pub const fn new() -> Self {
        Self {
            reference_pa: 0,
            range_pa: 0,
            count: OutOfRangeCount::One,
        }
    }

    /// Sets `reference_pa`.
    pub const fn with_reference_pa(mut self, v: u32) -> Self {
        self.reference_pa = v;
        self
    }

    /// Sets `range_pa`.
    pub const fn with_range_pa(mut self, v: u8) -> Self {
        self.range_pa = v;
        self
    }

    /// Sets `count`.
    pub const fn with_count(mut self, v: OutOfRangeCount) -> Self {
        self.count = v;
        self
    }
}

impl Default for OutOfRange {
    fn default() -> Self {
        Self::new()
    }
}

/// Measurement, filter, FIFO, out-of-range and interrupt configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Pressure oversampling.
    pub pressure_oversampling: Oversampling,
    /// Temperature oversampling.
    pub temperature_oversampling: Oversampling,
    /// `OSR_CONFIG.press_en`. Clear for temperature-only measurement (4.4.1, p18).
    pub pressure_enabled: bool,
    /// Output data rate, used in normal mode.
    pub odr: OutputDataRate,
    /// Pressure IIR filter.
    pub pressure_iir: IirFilter,
    /// Temperature IIR filter.
    pub temperature_iir: IirFilter,
    /// Where each data path takes its value from relative to the filters.
    pub iir_taps: IirTaps,
    /// `DSP_CONFIG.iir_flush_forced_en`: flush the filter at each forced measurement (4.4.4, p21).
    pub iir_flush_forced: bool,
    /// Mode `init` ends in. `Forced` leaves the chip in standby; call `trigger_forced` per measurement.
    pub power_mode: PowerMode,
    /// The INT pin.
    pub interrupt: InterruptPin,
    /// The FIFO.
    pub fifo: FifoConfig,
    /// The out-of-range window, or none.
    pub out_of_range: Option<OutOfRange>,
}

impl Config {
    /// Pressure 16x, temperature 2x, 50.056 Hz, no filter, normal mode, data-ready interrupt on,
    /// FIFO and out-of-range off.
    pub const fn new() -> Self {
        Self {
            pressure_oversampling: Oversampling::X16,
            temperature_oversampling: Oversampling::X2,
            pressure_enabled: true,
            odr: OutputDataRate::Hz50_056,
            pressure_iir: IirFilter::Bypass,
            temperature_iir: IirFilter::Bypass,
            iir_taps: IirTaps::new(),
            iir_flush_forced: false,
            power_mode: PowerMode::Normal,
            interrupt: InterruptPin::new(),
            fifo: FifoConfig::new(),
            out_of_range: None,
        }
    }

    /// Sets `pressure_oversampling`.
    pub const fn with_pressure_oversampling(mut self, v: Oversampling) -> Self {
        self.pressure_oversampling = v;
        self
    }

    /// Sets `temperature_oversampling`.
    pub const fn with_temperature_oversampling(mut self, v: Oversampling) -> Self {
        self.temperature_oversampling = v;
        self
    }

    /// Sets `pressure_enabled`.
    pub const fn with_pressure_enabled(mut self, v: bool) -> Self {
        self.pressure_enabled = v;
        self
    }

    /// Sets `odr`.
    pub const fn with_odr(mut self, v: OutputDataRate) -> Self {
        self.odr = v;
        self
    }

    /// Sets `pressure_iir`.
    pub const fn with_pressure_iir(mut self, v: IirFilter) -> Self {
        self.pressure_iir = v;
        self
    }

    /// Sets `temperature_iir`.
    pub const fn with_temperature_iir(mut self, v: IirFilter) -> Self {
        self.temperature_iir = v;
        self
    }

    /// Sets `iir_taps`.
    pub const fn with_iir_taps(mut self, v: IirTaps) -> Self {
        self.iir_taps = v;
        self
    }

    /// Sets `iir_flush_forced`.
    pub const fn with_iir_flush_forced(mut self, v: bool) -> Self {
        self.iir_flush_forced = v;
        self
    }

    /// Sets `power_mode`.
    pub const fn with_power_mode(mut self, v: PowerMode) -> Self {
        self.power_mode = v;
        self
    }

    /// Sets `interrupt`.
    pub const fn with_interrupt(mut self, v: InterruptPin) -> Self {
        self.interrupt = v;
        self
    }

    /// Sets `fifo`.
    pub const fn with_fifo(mut self, v: FifoConfig) -> Self {
        self.fifo = v;
        self
    }

    /// Sets `out_of_range`.
    pub const fn with_out_of_range(mut self, v: Option<OutOfRange>) -> Self {
        self.out_of_range = v;
        self
    }

    /// True when the FIFO is on.
    pub const fn fifo_enabled(&self) -> bool {
        !matches!(self.fifo.mode, FifoMode::Off)
    }

    /// True when the FIFO carries pressure.
    pub const fn fifo_carries_pressure(&self) -> bool {
        self.fifo_enabled() && !matches!(self.fifo.data, FifoData::Temperature)
    }

    /// Checks the combination against the rules of the datasheet. The register images do not check.
    ///
    /// The output data rate against the oversampling is checked in [`PowerMode::Normal`] only: that
    /// is the only mode in which the datasheet states the limit (4.3.4, p17).
    pub fn validate(&self) -> Result<(), ConfigError> {
        let fifo = &self.fifo;
        if self.fifo_carries_pressure() && !self.pressure_enabled {
            return Err(ConfigError::FifoPressureWithoutPressure);
        }
        if self.fifo_enabled() {
            let max = fifo.data.max_threshold();
            if fifo.threshold > max {
                return Err(ConfigError::FifoThresholdTooHigh { max });
            }
        }
        let sources = &self.interrupt.sources;
        if (sources.fifo_full || sources.fifo_threshold) && !self.fifo_enabled() {
            return Err(ConfigError::FifoInterruptWithoutFifo);
        }
        if sources.fifo_threshold && fifo.threshold == 0 {
            return Err(ConfigError::FifoThresholdInterruptWithoutThreshold);
        }
        if sources.out_of_range {
            if self.out_of_range.is_none() {
                return Err(ConfigError::OutOfRangeInterruptWithoutWindow);
            }
            if !self.pressure_enabled {
                return Err(ConfigError::OutOfRangeInterruptWithoutPressure);
            }
        }
        if let Some(w) = &self.out_of_range
            && w.reference_pa > OutOfRange::MAX_REFERENCE_PA
        {
            return Err(ConfigError::OutOfRangeReferenceTooHigh);
        }
        if matches!(self.power_mode, PowerMode::Normal) {
            let max = crate::calc::max_odr(
                self.pressure_enabled,
                self.pressure_oversampling,
                self.temperature_oversampling,
            );
            if self.odr.code() < max.code() {
                return Err(ConfigError::OdrTooHighForOversampling { max });
            }
        }
        Ok(())
    }
}

impl Default for Config {
    /// [`Config::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// Why a [`Config`] was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// In normal mode the output data rate is above what the oversampling allows (Table 7 or 8, p17).
    OdrTooHighForOversampling {
        /// The fastest rate allowed.
        max: OutputDataRate,
    },
    /// The FIFO threshold is above what the frame type allows (4.6.1, p23).
    FifoThresholdTooHigh {
        /// The largest threshold.
        max: u8,
    },
    /// The FIFO carries pressure while `pressure_enabled` is clear: the chip would measure pressure anyway
    /// (4.4.1, p18).
    FifoPressureWithoutPressure,
    /// A FIFO interrupt source with the FIFO off (4.7.2.1, p26).
    FifoInterruptWithoutFifo,
    /// The FIFO threshold source with threshold 0, which disables it (4.6.1, p23).
    FifoThresholdInterruptWithoutThreshold,
    /// The out-of-range source without a window.
    OutOfRangeInterruptWithoutWindow,
    /// The out-of-range source with pressure measurement off.
    OutOfRangeInterruptWithoutPressure,
    /// The out-of-range reference does not fit its 17 bits (4.7.2.3, p26).
    OutOfRangeReferenceTooHigh,
    /// A plan would not fit in its step array.
    PlanOverflow,
}
