//! Typed configuration and its encoders to register values. Pure code, no bus access.

use crate::reg;

/// Oversampling ratio, `OSR_CONFIG.osr_p` and `osr_t` (same codes for both).
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
}

/// Output data rate, `ODR_CONFIG.odr`. Used in normal mode; the name is the nominal rate in Hz.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[allow(missing_docs)]
pub enum OutputDataRate {
    Hz240 = 0x00,
    Hz218_537 = 0x01,
    Hz199_111 = 0x02,
    Hz179_2 = 0x03,
    Hz160 = 0x04,
    Hz149_333 = 0x05,
    Hz140 = 0x06,
    Hz129_855 = 0x07,
    Hz120 = 0x08,
    Hz110_164 = 0x09,
    Hz100_299 = 0x0A,
    Hz89_6 = 0x0B,
    Hz80 = 0x0C,
    Hz70 = 0x0D,
    Hz60 = 0x0E,
    Hz50_056 = 0x0F,
    Hz45_025 = 0x10,
    Hz40 = 0x11,
    Hz35 = 0x12,
    Hz30 = 0x13,
    Hz25_005 = 0x14,
    Hz20 = 0x15,
    Hz15 = 0x16,
    Hz10 = 0x17,
    Hz5 = 0x18,
    Hz4 = 0x19,
    Hz3 = 0x1A,
    Hz2 = 0x1B,
    Hz1 = 0x1C,
    Hz0_5 = 0x1D,
    Hz0_25 = 0x1E,
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

/// IIR filter coefficient, `DSP_IIR.set_iir_p` and `set_iir_t`.
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

/// Power mode, `ODR_CONFIG.pwr_mode`.
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

/// Measurement and interrupt configuration written by `init`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Pressure oversampling.
    pub pressure_oversampling: Oversampling,
    /// Temperature oversampling.
    pub temperature_oversampling: Oversampling,
    /// Output data rate, used in normal mode.
    pub odr: OutputDataRate,
    /// Pressure IIR filter.
    pub pressure_iir: IirFilter,
    /// Temperature IIR filter.
    pub temperature_iir: IirFilter,
    /// Mode `init` ends in. `Forced` leaves the chip in standby; call `trigger_forced` per measurement.
    pub power_mode: PowerMode,
    /// Route data-ready to the INT pin (push-pull, active high, pulsed).
    pub data_ready_interrupt: bool,
}

impl Default for Config {
    /// Pressure 16x, temperature 2x, 50.056 Hz, no filter, normal mode, data-ready interrupt on.
    fn default() -> Self {
        Self {
            pressure_oversampling: Oversampling::X16,
            temperature_oversampling: Oversampling::X2,
            odr: OutputDataRate::Hz50_056,
            pressure_iir: IirFilter::Bypass,
            temperature_iir: IirFilter::Bypass,
            power_mode: PowerMode::Normal,
            data_ready_interrupt: true,
        }
    }
}

impl Config {
    /// `OSR_CONFIG` value, pressure measurement enabled.
    pub const fn osr_config(&self) -> u8 {
        reg::OSR_CONFIG_PRESS_EN
            | (self.pressure_oversampling.code() << reg::OSR_CONFIG_OSR_P_SHIFT)
            | self.temperature_oversampling.code()
    }

    /// `ODR_CONFIG` value for this rate and the given power mode.
    pub const fn odr_config(&self, mode: PowerMode) -> u8 {
        (self.odr.code() << reg::ODR_CONFIG_ODR_SHIFT) | mode.code()
    }

    /// `DSP_CONFIG` value: a data register holds the filtered value when its filter is on.
    pub const fn dsp_config(&self) -> u8 {
        let mut v = reg::DSP_CONFIG_RESET;
        if !matches!(self.temperature_iir, IirFilter::Bypass) {
            v |= reg::DSP_CONFIG_SHDW_SEL_IIR_T;
        }
        if !matches!(self.pressure_iir, IirFilter::Bypass) {
            v |= reg::DSP_CONFIG_SHDW_SEL_IIR_P;
        }
        v
    }

    /// `DSP_IIR` value.
    pub const fn dsp_iir(&self) -> u8 {
        (self.pressure_iir.code() << reg::DSP_IIR_P_SHIFT) | self.temperature_iir.code()
    }

    /// `INT_SOURCE` value: data-ready only, or nothing.
    pub const fn int_source(&self) -> u8 {
        if self.data_ready_interrupt {
            reg::INT_SOURCE_DRDY_EN
        } else {
            0
        }
    }

    /// `INT_CONFIG` value: pulsed, active high, push-pull, pad drive 3, enable as configured.
    pub const fn int_config(&self) -> u8 {
        let v = reg::INT_CONFIG_ACTIVE_HIGH | (3 << reg::INT_CONFIG_PAD_DRV_SHIFT);
        if self.data_ready_interrupt {
            v | reg::INT_CONFIG_EN
        } else {
            v
        }
    }
}
