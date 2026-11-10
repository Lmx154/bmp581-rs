//! Raw measurement and interrupt status decoding. Pure code, no bus access.

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

const fn sign_extend_24(b0: u8, b1: u8, b2: u8) -> i32 {
    (((b2 as i32) << 24) | ((b1 as i32) << 16) | ((b0 as i32) << 8)) >> 8
}

impl RawSample {
    /// Decodes the six bytes at `TEMP_DATA_XLSB` (0x1D): temperature then pressure, each little endian.
    pub const fn decode(buf: &[u8; 6]) -> Self {
        Self {
            temperature: sign_extend_24(buf[0], buf[1], buf[2]),
            pressure: sign_extend_24(buf[3], buf[4], buf[5]),
        }
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

/// `INT_STATUS` flags. The register clears when read.
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
