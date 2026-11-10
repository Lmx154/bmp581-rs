//! Driver for the Bosch BMP581 barometric pressure sensor over I2C.
//!
//! Built on the `embedded-hal` 1.0 traits only: no executor, no HAL, no pin, no timer. The driver
//! configures the chip's INT output; the application waits on the pin however it likes.
//! [`Bmp581`] is the one implementation (async). [`blocking::Bmp581`] runs the same code over a
//! blocking bus. The device knowledge ([`reg`], [`Config`], [`RawSample`]) is pure and needs no bus.
//!
//! Units: [`RawSample::pascals`] is pascals, [`RawSample::celsius`] is degrees Celsius.
//!
//! Async:
//! ```ignore
//! let mut baro = bmp581::Bmp581::new(i2c, bmp581::DEFAULT_ADDRESS);
//! baro.init(&mut delay, &bmp581::Config::default()).await?;
//! int_pin.wait_for_rising_edge().await;
//! let sample = baro.read().await?;
//! let (pa, deg_c) = (sample.pascals(), sample.celsius());
//! ```
//!
//! Blocking:
//! ```ignore
//! let mut baro = bmp581::blocking::Bmp581::new(i2c, bmp581::DEFAULT_ADDRESS);
//! baro.init(&mut delay, &bmp581::Config::default())?;
//! let sample = baro.read()?;
//! ```
#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub mod blocking;
mod config;
pub mod reg;
mod sample;

use embedded_hal::i2c::{Error as _, ErrorKind};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

pub use config::{Config, IirFilter, OutputDataRate, Oversampling, PowerMode};
pub use sample::{IntStatus, RawSample};

/// 7-bit address with the address pin low.
pub const DEFAULT_ADDRESS: u8 = 0x46;
/// 7-bit address with the address pin high.
pub const ALTERNATE_ADDRESS: u8 = 0x47;

/// Soft reset duration is 2 ms at most; wait a little longer.
const RESET_WAIT_US: u32 = 3_000;

/// Driver error. `E` is the bus error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// The bus failed.
    Bus(E),
    /// `CHIP_ID` is not 0x50.
    WrongId {
        /// What the chip returned.
        found: u8,
    },
    /// `STATUS.status_nvm_rdy` is clear after reset.
    NvmNotReady,
    /// `STATUS.status_nvm_err` is set.
    NvmError,
    /// The chip rejected the ODR and oversampling combination (`OSR_EFF.odr_is_valid` clear).
    InvalidConfig,
}

/// BMP581 driver over an async `embedded_hal_async::i2c::I2c`.
pub struct Bmp581<I2C> {
    i2c: I2C,
    address: u8,
    odr_config: u8,
}

impl<I2C: I2c> Bmp581<I2C> {
    /// Wraps the bus. `address` is [`DEFAULT_ADDRESS`] or [`ALTERNATE_ADDRESS`]. No bus access.
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            odr_config: 0,
        }
    }

    /// Returns the bus.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Soft-resets the chip, checks identity and NVM, and writes `config`.
    ///
    /// Order: reset (a missing acknowledge is tolerated), wait, `CHIP_ID`, `STATUS`, `INT_STATUS` (clears the
    /// reset flag, which would otherwise hold INT high once enabled), then oversampling and rate in standby,
    /// filters, interrupt, and the power mode last. With [`PowerMode::Forced`] the chip
    /// stays in standby until [`trigger_forced`](Self::trigger_forced). The ODR and oversampling
    /// combination is checked for [`PowerMode::Normal`] only; the chip ignores the rate otherwise.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<(), Error<I2C::Error>> {
        match self.write_register(reg::CMD, reg::CMD_SOFT_RESET).await {
            Err(Error::Bus(e)) if matches!(e.kind(), ErrorKind::NoAcknowledge(_)) => {}
            r => r?,
        }
        delay.delay_us(RESET_WAIT_US).await;

        let id = self.read_register(reg::CHIP_ID).await?;
        if id != reg::CHIP_ID_VALUE {
            return Err(Error::WrongId { found: id });
        }
        let status = self.read_register(reg::STATUS).await?;
        if status & reg::STATUS_NVM_RDY == 0 {
            return Err(Error::NvmNotReady);
        }
        if status & reg::STATUS_NVM_ERR != 0 {
            return Err(Error::NvmError);
        }

        self.read_register(reg::INT_STATUS).await?;
        self.odr_config = config.odr_config(PowerMode::Standby);
        self.write_register(reg::OSR_CONFIG, config.osr_config())
            .await?;
        self.write_register(reg::ODR_CONFIG, self.odr_config)
            .await?;
        if config.power_mode == PowerMode::Normal
            && self.read_register(reg::OSR_EFF).await? & reg::OSR_EFF_ODR_IS_VALID == 0
        {
            return Err(Error::InvalidConfig);
        }
        self.write_register(reg::DSP_CONFIG, config.dsp_config())
            .await?;
        self.write_register(reg::DSP_IIR, config.dsp_iir()).await?;
        self.write_register(reg::INT_SOURCE, config.int_source())
            .await?;
        self.write_register(reg::INT_CONFIG, config.int_config())
            .await?;
        self.read_register(reg::INT_STATUS).await?;
        if config.power_mode != PowerMode::Forced {
            self.write_register(reg::ODR_CONFIG, config.odr_config(config.power_mode))
                .await?;
        }
        Ok(())
    }

    /// Reads temperature and pressure in one `write_read` transaction.
    pub async fn read(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_registers(reg::TEMP_DATA_XLSB, &mut buf).await?;
        Ok(RawSample::decode(&buf))
    }

    /// Starts one forced-mode measurement with the oversampling and filters from `init`.
    /// The chip returns to standby when done; data-ready then fires.
    pub async fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
        let v = (self.odr_config & !0b11) | PowerMode::Forced.code();
        self.write_register(reg::ODR_CONFIG, v).await
    }

    /// Reads `INT_STATUS`, which clears it.
    pub async fn interrupt_status(&mut self) -> Result<IntStatus, Error<I2C::Error>> {
        Ok(IntStatus::from_raw(
            self.read_register(reg::INT_STATUS).await?,
        ))
    }

    /// Reads one register.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut b = [0u8; 1];
        self.read_registers(reg, &mut b).await?;
        Ok(b[0])
    }

    /// Writes one register.
    pub async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[reg, value])
            .await
            .map_err(Error::Bus)
    }

    /// Reads consecutive registers starting at `reg`, in one transaction.
    pub async fn read_registers(
        &mut self,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .await
            .map_err(Error::Bus)
    }
}

#[cfg(test)]
mod tests;
