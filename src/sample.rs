//! Raw measurement and interrupt status decoding. Pure code, no bus access.

use crate::Oversampling;
use crate::reg;

/// One temperature and pressure measurement as the registers hold it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSample {
    /// Signed 24-bit temperature, 1/65536 degC per LSB.
    pub temperature: i32,
    /// Signed 24-bit pressure, 1/64 Pa per LSB.
    pub pressure: i32,
}

/// The 24-bit value the data registers hold at reset: every byte 0x7F (7.11 to 7.16, p56-57), as raw 0x7F7F7F.
const RESET_RAW: i32 = 0x7F_7F7F;

pub(crate) const fn sign_extend_24(b0: u8, b1: u8, b2: u8) -> i32 {
    (((b2 as i32) << 24) | ((b1 as i32) << 16) | ((b0 as i32) << 8)) >> 8
}

impl RawSample {
    /// Decodes the six bytes at `TEMP_DATA_XLSB` (0x1D): temperature then pressure, each little endian.
    pub const fn decode(buf: &[u8; 6]) -> Self {
        let [t0, t1, t2, p0, p1, p2] = *buf;
        Self {
            temperature: sign_extend_24(t0, t1, t2),
            pressure: sign_extend_24(p0, p1, p2),
        }
    }

    /// Whether the raw pressure equals the data registers' reset value 0x7F7F7F: "no conversion has finished
    /// yet", not a measurement (see [`crate::Bmp581::read`]). Pure; no bus access.
    pub const fn pressure_is_reset_value(&self) -> bool {
        self.pressure == RESET_RAW
    }

    /// Whether the raw temperature equals the data registers' reset value 0x7F7F7F: "no conversion has
    /// finished yet", not a measurement (see [`crate::Bmp581::read`]). Pure; no bus access.
    pub const fn temperature_is_reset_value(&self) -> bool {
        self.temperature == RESET_RAW
    }

    /// Whether the raw pressure or the raw temperature is the data registers' reset value 0x7F7F7F. With
    /// pressure measurement switched off the pressure half may be that value legitimately;
    /// [`crate::Bmp581::read`] then judges only the temperature.
    pub const fn is_reset_value(&self) -> bool {
        self.pressure_is_reset_value() || self.temperature_is_reset_value()
    }

    /// Pressure in pascals.
    pub fn pascals(&self) -> f32 {
        self.pressure as f32 / 64.0
    }

    /// Temperature in degrees Celsius.
    pub fn celsius(&self) -> f32 {
        self.temperature as f32 / 65536.0
    }
}

/// `INT_STATUS` flags (7.21, p58): one field per defined bit 4:0; bits 7:5 are reserved. The register
/// clears when read: [`crate::Bmp581::interrupt_status`] returns each flag once. The power-on flag
/// is the only sign of a brown-out or reset while the chip runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IntStatus {
    /// A new measurement is in the data registers.
    pub data_ready: bool,
    /// FIFO full.
    pub fifo_full: bool,
    /// FIFO threshold reached.
    pub fifo_threshold: bool,
    /// Pressure out of range.
    pub out_of_range: bool,
    /// Power-on or soft reset completed.
    pub power_on_reset: bool,
}

impl IntStatus {
    /// Decodes the register value.
    pub const fn from_raw(v: u8) -> Self {
        Self {
            data_ready: v & reg::INT_STATUS_DRDY != 0,
            fifo_full: v & reg::INT_STATUS_FIFO_FULL != 0,
            fifo_threshold: v & reg::INT_STATUS_FIFO_THS != 0,
            out_of_range: v & reg::INT_STATUS_OOR_P != 0,
            power_on_reset: v & reg::INT_STATUS_POR != 0,
        }
    }
}

/// `STATUS` decoded (7.22, p58) with the raw byte. Not clear-on-read.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Status {
    /// `status_core_rdy`, bit 0: the digital core domain is accessible.
    pub core_ready: bool,
    /// `status_nvm_rdy`, bit 1: the device is ready for NVM operations.
    pub nvm_ready: bool,
    /// `status_nvm_err`, bit 2: an NVM error (power transition, timeout, charge pump, or 2+ ECC errors
    /// at the last boot-load).
    pub nvm_error: bool,
    /// `status_nvm_cmd_err`, bit 3: an NVM or boot command error.
    pub nvm_command_error: bool,
    /// `status_boot_err_corrected`, bit 4: ECC corrected an error at the last boot-load.
    pub boot_error_corrected: bool,
    /// `st_crack_pass`, bit 7: the crack check passed.
    pub crack_check_passed: bool,
    /// The whole register, reserved bits 6:5 included.
    pub raw: u8,
}

impl Status {
    /// Decodes the register value.
    pub const fn from_raw(v: u8) -> Self {
        Self {
            core_ready: v & reg::STATUS_CORE_RDY != 0,
            nvm_ready: v & reg::STATUS_NVM_RDY != 0,
            nvm_error: v & reg::STATUS_NVM_ERR != 0,
            nvm_command_error: v & reg::STATUS_NVM_CMD_ERR != 0,
            boot_error_corrected: v & reg::STATUS_BOOT_ERR_CORRECTED != 0,
            crack_check_passed: v & reg::STATUS_ST_CRACK_PASS != 0,
            raw: v,
        }
    }
}

/// `OSR_EFF` decoded (7.35, p66): what the chip actually uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct EffectiveOversampling {
    /// `odr_is_valid`, bit 7: the rate and oversampling pair is valid. Otherwise the chip uses
    /// default oversampling.
    pub valid: bool,
    /// `osr_p_eff`, bits 5:3.
    pub pressure: Oversampling,
    /// `osr_t_eff`, bits 2:0.
    pub temperature: Oversampling,
}

impl Default for EffectiveOversampling {
    fn default() -> Self {
        Self::from_raw(0)
    }
}

impl EffectiveOversampling {
    /// Decodes the register value.
    pub const fn from_raw(v: u8) -> Self {
        Self {
            valid: v & reg::OSR_EFF_ODR_IS_VALID != 0,
            pressure: Oversampling::from_code(v >> reg::OSR_EFF_OSR_P_SHIFT),
            temperature: Oversampling::from_code(v & reg::OSR_EFF_OSR_T_MASK),
        }
    }
}
