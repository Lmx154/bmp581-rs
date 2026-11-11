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
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

#[cfg(test)]
extern crate std;

pub mod blocking;
pub mod calc;
mod config;
mod fifo;
mod image;
pub mod plan;
pub mod reg;
mod sample;

use embedded_hal::i2c::{Error as _, ErrorKind};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

pub use config::{
    Config, ConfigError, FifoConfig, FifoData, FifoDecimation, FifoMode, IirFilter, IirTap,
    IirTaps, InterruptDrive, InterruptMode, InterruptPin, InterruptPolarity, InterruptSources,
    OutOfRange, OutOfRangeCount, OutputDataRate, Oversampling, PowerMode,
};
pub use fifo::{FifoFrame, FifoFrames, PressureFrame, TemperatureFrame, fifo_frames};
pub use plan::{Apply, Plan, Step};
pub use sample::{EffectiveOversampling, IntStatus, RawSample, Status};

/// 7-bit address with the address pin low.
pub const DEFAULT_ADDRESS: u8 = 0x46;
/// 7-bit address with the address pin high.
pub const ALTERNATE_ADDRESS: u8 = 0x47;

/// Soft reset duration is 2 ms at most; wait a little longer.
const RESET_WAIT_US: u32 = 3_000;

/// Quiet time of [`Bmp581::recover`]: after a cancelled read, later reads return wrong data until the
/// bus has been quiet for 2 ms (measured on the board, 2026-11-10; contract A4g).
/// Longest read `verify` makes: the largest read that is one fully queued frame (contract A4e).
const VERIFY_WINDOW: u8 = 15;

const RECOVER_QUIET_US: u32 = 2_000;

/// Flush completion: `treconf_deep`, 4 ms at most, which covers `treconf`, 3 ms (Table 4, p11; 4.6.4, p25).
const FLUSH_WAIT_US: u32 = 4_000;

/// Driver error. `E` is the bus error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
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
    /// The [`Config`] breaks a rule of the datasheet; nothing was written to the chip.
    InvalidConfig(ConfigError),
    /// The chip itself rejects the output data rate and oversampling of a [`PowerMode::Normal`]
    /// configuration: `OSR_EFF.odr_is_valid` is clear (7.35, p66; 4.3.4, p17). Had the device entered
    /// normal mode it would keep the rate and replace the oversampling. It does not enter it: the flag is
    /// read in standby, before the power-mode write.
    ///
    /// State after this error: the DEVICE is in standby, with the new `OSR_CONFIG` and the standby
    /// `ODR_CONFIG` written and the steps after them not run (the filters, the FIFO, the interrupt are
    /// whatever they were, or at reset values after [`Bmp581::init`]). The DRIVER is not configured:
    /// [`Bmp581::read`], [`Bmp581::interrupt_status`] and [`Bmp581::trigger_forced`] return
    /// [`Error::NotConfigured`]; the raw register methods still work. The application calls
    /// [`Bmp581::configure`] or [`Bmp581::init`] with a pair the chip accepts.
    ChipRejectsRate {
        /// The `OSR_EFF` byte the chip returned.
        osr_eff: u8,
    },
    /// The driver is not configured: no [`Bmp581::init`] or [`Bmp581::configure`] has completed since the
    /// last one started or failed.
    NotConfigured,
    /// A register the applied configuration owns did not read back as configured, twice in a row.
    /// Returned by [`Bmp581::verify`] and [`Bmp581::recover`]; the driver is unchanged and still
    /// configured.
    ConfigMismatch(Mismatch),
    /// The data registers still hold their reset value 0x7F7F7F in the temperature or, with pressure
    /// measurement on, in the pressure: no conversion has finished since the configuration (measured on
    /// the board, 2026-11-10: such a read returned 130 557.98 Pa). Returned by [`Bmp581::read`]. Wait for
    /// the data-ready edge, or [`Info::conversion_time_us`], and read again.
    NoSample,
    /// The FIFO is off in the applied [`Config`] ([`FifoMode::Off`]); returned by the FIFO methods without
    /// touching the bus.
    FifoOff,
    /// The buffer given to [`Bmp581::fifo_read`] is shorter than one frame of the configured kind (6 bytes
    /// for pressure and temperature, 3 for one quantity); returned without touching the bus.
    BufferTooSmall,
}

/// A register that differs from what the applied configuration put there.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Mismatch {
    /// Register address.
    pub register: u8,
    /// The bits that were compared.
    pub mask: u8,
    /// The value expected under `mask`; bits outside `mask` are 0.
    pub expected: u8,
    /// The value read, whole register (the second of the two reads).
    pub found: u8,
}

/// Counters of what happened while the driver ran. Saturating; nothing clears them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Stats {
    /// Power-on or reset flags (`INT_STATUS.por`, 7.21, p58) seen by [`Bmp581::interrupt_status`]. The
    /// reset that [`Bmp581::init`] causes is not counted: `init` reads and discards that flag.
    pub power_on_resets: u32,
    /// Bus errors the driver returned (each failed transaction, the tolerated missing acknowledge of
    /// the reset command excluded).
    pub bus_errors: u32,
    /// FIFO-full conditions seen: a [`Bmp581::fifo_read`] whose frame count equalled the capacity of the
    /// configured frame kind, and each `INT_STATUS` read ([`Bmp581::interrupt_status`]) that showed the
    /// FIFO-full flag (7.21, p58). One full FIFO seen by both ways is counted by both. What the chip does
    /// when it is full (4.6.1, p23): in streaming mode the oldest frame is deleted and the new one written,
    /// so frames are LOST and the FIFO holds the most recent ones; in stop-on-full mode new frames are NOT
    /// written until there is space again, so the new measurements are lost and the FIFO holds the oldest.
    /// A full FIFO means the loss has begun or is imminent, not how many frames were lost.
    pub fifo_full: u32,
    /// Frames returned by [`Bmp581::fifo_read`]; frames of a read that failed are not counted.
    pub fifo_frames: u32,
}

/// What one [`Bmp581::fifo_read`] did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoRead {
    /// Bytes written to the start of the buffer: `frames` times the frame size.
    pub bytes: usize,
    /// Whole frames read.
    pub frames: u8,
    /// Frames still in the FIFO after this call, as far as the count read at its start knows: the count
    /// minus `frames`. The chip may have written more since.
    pub waiting: u8,
    /// The count equalled (or exceeded) the capacity of the configured frame kind, 16 frames for pressure
    /// and temperature and 32 for one quantity (4.6, p22): the FIFO was full and, as new measurements
    /// arrive, frames are being lost (see [`Stats::fifo_full`] for what the chip does in each mode).
    pub full: bool,
}

/// What a completed [`Bmp581::init`] or [`Bmp581::configure`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Info {
    /// `CHIP_ID` (7.1, p51). 0 after [`Bmp581::configure`], which does not read it.
    pub chip_id: u8,
    /// `REV_ID` (7.2, p51). 0 after [`Bmp581::configure`], which does not read it.
    pub revision: u8,
    /// The pressure oversampling applied: [`Config::pressure_oversampling`]. In normal mode the chip's
    /// validity flag was set, so it is also what the chip uses (7.35, p66); in the other modes the flag is
    /// not looked at, and [`Bmp581::effective_oversampling`] asks the chip.
    pub pressure_oversampling: Oversampling,
    /// The temperature oversampling applied: [`Config::temperature_oversampling`]; see
    /// [`Info::pressure_oversampling`].
    pub temperature_oversampling: Oversampling,
    /// Nominal output period in microseconds ([`OutputDataRate::period_us`]); 0 when the mode is not
    /// [`PowerMode::Normal`].
    pub output_period_us: u32,
    /// Typical time of one measurement in microseconds with the applied oversampling
    /// ([`calc::conversion_time_us`]; temperature only when pressure is off).
    pub conversion_time_us: u32,
}

impl Default for Info {
    fn default() -> Self {
        Self {
            chip_id: 0,
            revision: 0,
            pressure_oversampling: Oversampling::X1,
            temperature_oversampling: Oversampling::X1,
            output_period_us: 0,
            conversion_time_us: 0,
        }
    }
}

/// BMP581 driver over an async `embedded_hal_async::i2c::I2c`.
///
/// The driver is configured once [`init`](Self::init) or [`configure`](Self::configure) has completed
/// and not configured while either runs, was dropped or failed (contract A3). `read`,
/// `interrupt_status` and `trigger_forced` return [`Error::NotConfigured`] while it is not. The raw
/// register methods ([`read_register`](Self::read_register), [`write_register`](Self::write_register),
/// [`read_registers`](Self::read_registers)) are outside that rule and outside the cancel-safety
/// guarantee of the configuring methods: a register written through them is not tracked.
///
/// Every register read is one `write_read` transaction.
pub struct Bmp581<I2C> {
    i2c: I2C,
    address: u8,
    config: Option<Config>,
    revision: Option<u8>,
    stats: Stats,
}

impl<I2C: I2c> Bmp581<I2C> {
    /// Wraps the bus. `address` is [`DEFAULT_ADDRESS`] or [`ALTERNATE_ADDRESS`]. No bus access. The driver
    /// starts not configured.
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            config: None,
            revision: None,
            stats: Stats {
                power_on_resets: 0,
                bus_errors: 0,
                fifo_full: 0,
                fifo_frames: 0,
            },
        }
    }

    /// Returns the bus.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// The configuration applied by the last completed [`init`](Self::init) or
    /// [`configure`](Self::configure); `None` while the driver is not configured.
    pub const fn config(&self) -> Option<&Config> {
        self.config.as_ref()
    }

    /// The counters; see [`Stats`]. Nothing clears them.
    pub const fn stats(&self) -> Stats {
        self.stats
    }

    fn bus_error(&mut self, e: I2C::Error) -> Error<I2C::Error> {
        self.stats.bus_errors = self.stats.bus_errors.saturating_add(1);
        Error::Bus(e)
    }

    /// Soft-resets the chip, checks identity and NVM, and applies `config`.
    ///
    /// The driver is not configured from the start of the call until it returns `Ok`. The configuration is
    /// validated first ([`Error::InvalidConfig`], no bus access). Then: the reset command (a missing
    /// acknowledge is tolerated), a wait, `CHIP_ID` and `REV_ID` in one read, `STATUS` (NVM ready and no
    /// NVM error), the steps of [`Config::plan`] for [`Apply::AfterReset`] (they read `INT_STATUS`, which
    /// clears the reset flag that would otherwise hold INT high once enabled, set oversampling and rate in
    /// standby, then in [`PowerMode::Normal`] the check of the chip's validity flag, the filters, the interrupt,
    /// and the power mode last). With [`PowerMode::Forced`] the chip stays in standby until
    /// [`trigger_forced`](Self::trigger_forced).
    ///
    /// The chip's validity flag (`OSR_EFF.odr_is_valid`, 7.35, p66) is checked in [`PowerMode::Normal`]
    /// only, the one mode in which the datasheet states the limit (4.3.4, p17), and BEFORE the mode write:
    /// the flag is valid in standby once `OSR_CONFIG` and `ODR_CONFIG` are written (measured on the board,
    /// 2026-11-10). A clear flag returns [`Error::ChipRejectsRate`]: the device stays in standby with the
    /// new oversampling and standby rate registers written, the driver is not configured (typed reads
    /// return [`Error::NotConfigured`], raw register access still works), and the application calls
    /// [`configure`](Self::configure) or `init` with a pair the chip accepts. In the other modes the rate
    /// is unused and the flag is not looked at (a configuration that is legal there must not fail), and
    /// `OSR_EFF` is not read. [`Info`] reports the applied oversampling; [`Bmp581::effective_oversampling`]
    /// asks the chip.
    ///
    /// The first sample exists only after the first conversion has finished, which `init` does not wait for
    /// (it returns right after the power-mode write): the application waits for the data-ready edge, or for
    /// [`Info::conversion_time_us`] (from [`calc::conversion_time_us`]), before [`read`](Self::read); a
    /// read made too early returns [`Error::NoSample`] (measured on the board, 2026-11-10).
    ///
    /// Transactions: 1 reset write, 1 read of 2 bytes, 1 read of `STATUS`, the plan (one transaction per
    /// read, write or check step); one delay for the reset wait and one per plan delay.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        self.config = None;
        let plan = config
            .plan(Apply::AfterReset)
            .map_err(Error::InvalidConfig)?;

        match self.write_bytes(reg::CMD, reg::CMD_SOFT_RESET).await {
            Err(e) if matches!(e.kind(), ErrorKind::NoAcknowledge(_)) => {}
            r => r.map_err(|e| self.bus_error(e))?,
        }
        delay.delay_us(RESET_WAIT_US).await;

        let mut identity = [0u8; 2];
        self.read_registers(reg::CHIP_ID, &mut identity).await?;
        let [chip_id, revision] = identity;
        if chip_id != reg::CHIP_ID_VALUE {
            return Err(Error::WrongId { found: chip_id });
        }
        self.revision = Some(revision);
        let status = self.read_register(reg::STATUS).await?;
        if status & reg::STATUS_NVM_RDY == 0 {
            return Err(Error::NvmNotReady);
        }
        if status & reg::STATUS_NVM_ERR != 0 {
            return Err(Error::NvmError);
        }

        self.apply(delay, &plan, config, chip_id, revision).await
    }

    /// Applies `config` to a chip in any state, without a reset.
    ///
    /// The driver is not configured from the start of the call until it returns `Ok`. The configuration is
    /// validated first ([`Error::InvalidConfig`], no bus access). Then the steps of [`Config::plan`] for
    /// [`Apply::InPlace`] (a switch to standby and its wait, every register written whole), with the validity
    /// flag checked before the mode write as in [`init`](Self::init): on [`Error::ChipRejectsRate`] the
    /// device is in standby with the new oversampling and standby rate written and the later steps not
    /// run, and the driver is not configured. Identity and revision are not read: [`Info::chip_id`] and
    /// [`Info::revision`] are 0.
    ///
    /// The first sample of the new configuration exists only after the first conversion has finished,
    /// which `configure` does not wait for: the application waits for the data-ready edge, or for
    /// [`Info::conversion_time_us`] (from [`calc::conversion_time_us`]), before [`read`](Self::read); a
    /// read made too early returns [`Error::NoSample`] (measured on the board, 2026-11-10: such a read
    /// right after `configure` into normal mode returned the reset value of the data registers).
    ///
    /// Transactions: the plan (one per read, write or check step); one delay per plan delay. Measured on the
    /// board, 2026-11-10: 4.1 ms.
    pub async fn configure(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        self.config = None;
        let plan = config.plan(Apply::InPlace).map_err(Error::InvalidConfig)?;
        self.apply(delay, &plan, config, 0, 0).await
    }

    async fn apply(
        &mut self,
        delay: &mut impl DelayNs,
        plan: &Plan,
        config: &Config,
        chip_id: u8,
        revision: u8,
    ) -> Result<Info, Error<I2C::Error>> {
        for step in plan {
            match *step {
                Step::Write { reg, value } => self.write_register(reg, value).await?,
                Step::Read { reg } => {
                    self.read_register(reg).await?;
                }
                Step::DelayUs(us) => delay.delay_us(us).await,
                Step::CheckRateValid => {
                    let osr_eff = self.read_register(reg::OSR_EFF).await?;
                    if osr_eff & reg::OSR_EFF_ODR_IS_VALID == 0 {
                        return Err(Error::ChipRejectsRate { osr_eff });
                    }
                }
            }
        }
        self.config = Some(*config);
        let pressure = config.pressure_oversampling;
        let temperature = config.temperature_oversampling;
        Ok(Info {
            chip_id,
            revision,
            pressure_oversampling: pressure,
            temperature_oversampling: temperature,
            output_period_us: if config.power_mode == PowerMode::Normal {
                config.odr.period_us()
            } else {
                0
            },
            conversion_time_us: if config.pressure_enabled {
                calc::conversion_time_us(pressure, temperature)
            } else {
                calc::temperature_conversion_time_us(temperature)
            },
        })
    }

    /// Reads temperature and pressure in one `write_read` transaction of 6 bytes.
    /// [`Error::NotConfigured`] while the driver is not configured.
    ///
    /// There is no sample until the first conversion has finished: after [`init`](Self::init),
    /// [`configure`](Self::configure) and [`trigger_forced`](Self::trigger_forced) the data registers hold
    /// their reset value (every byte 0x7F, raw 0x7F7F7F, 7.11 to 7.16, p56-57) until then. The application
    /// waits for the data-ready edge (the interrupt pin, or [`interrupt_status`](Self::interrupt_status)),
    /// or for [`Info::conversion_time_us`] (the typical time of one conversion, from
    /// [`calc::conversion_time_us`]), before it reads. A read made too early returns
    /// [`Error::NoSample`]: measured on the board, 2026-11-10, a read right after `configure` into normal
    /// mode returned 130 557.98 Pa with `Ok`, which is this value and not a measurement.
    ///
    /// [`Error::NoSample`] is returned when the raw temperature, or the raw pressure while
    /// [`Config::pressure_enabled`] is set, equals 0x7F7F7F. The check costs no extra bus traffic. With
    /// pressure off the pressure registers may hold that value legitimately and only the temperature is
    /// judged. Why 0x7F7F7F cannot be a measurement: the pressure converts to 130 557.98 Pa, above the
    /// stated measurement range of 30 to 125 kPa (Table 1, p8; key features, p2); the temperature converts
    /// to 127.498 degC, above the stated range of -40 to 85 degC (Table 2, p10; p2) and above the 125 degC
    /// storage limit (Table 6, p13). The datasheet states these as specified ranges, not as a clamp of
    /// the output, so the argument is that the value is outside every range it gives, for both halves; it
    /// does not say that no out-of-range reading can ever exist.
    pub async fn read(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        let Some(config) = self.config else {
            return Err(Error::NotConfigured);
        };
        let mut buf = [0u8; 6];
        self.read_registers(reg::TEMP_DATA_XLSB, &mut buf).await?;
        let sample = RawSample::decode(&buf);
        if sample.temperature_is_reset_value()
            || (config.pressure_enabled && sample.pressure_is_reset_value())
        {
            return Err(Error::NoSample);
        }
        Ok(sample)
    }

    /// Starts one forced-mode measurement with the oversampling and filters of the applied
    /// configuration. The chip returns to standby when done; data-ready then fires.
    ///
    /// The sample exists only after that conversion has finished: the application waits for the data-ready
    /// edge, or for [`Info::conversion_time_us`] (from [`calc::conversion_time_us`]), before
    /// [`read`](Self::read). A read made too early returns [`Error::NoSample`] only when the data
    /// registers still hold the reset value; after an earlier conversion they hold that older sample,
    /// which `read` returns as a valid one.
    /// [`Error::NotConfigured`] while the driver is not configured.
    pub async fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
        let value = self
            .config
            .as_ref()
            .ok_or(Error::NotConfigured)?
            .odr_config(PowerMode::Forced);
        self.write_register(reg::ODR_CONFIG, value).await
    }

    /// Reads `INT_STATUS` (7.21, p58) in one transaction of one byte. The register is clear-on-read: every
    /// flag is returned once, and a flag the application has not read yet is lost to anyone else who reads
    /// the register ([`verify`](Self::verify) and [`recover`](Self::recover) never do). Reports all five
    /// defined bits: data ready, FIFO full, FIFO threshold, out of range and the power-on flag, which is
    /// the only sign of a brown-out or reset while the chip runs; each power-on flag seen is added to
    /// [`Stats::power_on_resets`] (saturating). [`Error::NotConfigured`] while the driver is not
    /// configured, with no bus access.
    pub async fn interrupt_status(&mut self) -> Result<IntStatus, Error<I2C::Error>> {
        if self.config.is_none() {
            return Err(Error::NotConfigured);
        }
        let status = IntStatus::from_raw(self.read_register(reg::INT_STATUS).await?);
        if status.power_on_reset {
            self.stats.power_on_resets = self.stats.power_on_resets.saturating_add(1);
        }
        if status.fifo_full {
            self.stats.fifo_full = self.stats.fifo_full.saturating_add(1);
        }
        Ok(status)
    }

    /// Reads `FIFO_COUNT` (7.8, p55), the number of frames in the FIFO, in one transaction of one byte.
    /// Bits 7:6 are reserved and masked off. [`Error::NotConfigured`] while the driver is not configured and
    /// [`Error::FifoOff`] when the applied configuration has the FIFO off, both with no bus access.
    ///
    /// Do not read it before a flush has finished (4.6.4, p25); [`fifo_flush`](Self::fifo_flush) waits for that.
    pub async fn fifo_count(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.fifo_data_kind()?;
        Ok(self.read_register(reg::FIFO_COUNT).await? & reg::FIFO_COUNT_MASK)
    }

    fn fifo_data_kind(&self) -> Result<FifoData, Error<I2C::Error>> {
        let config = self.config.as_ref().ok_or(Error::NotConfigured)?;
        if config.fifo_enabled() {
            Ok(config.fifo.data)
        } else {
            Err(Error::FifoOff)
        }
    }

    /// Reads the frame count, then every frame that is waiting and fits in `buf`, whole frames only.
    ///
    /// Transactions: 2. First `FIFO_COUNT` (0x17), one byte (7.8, p55). Then one burst from `FIFO_DATA`
    /// (0x29), `frame size * n` bytes, where `n` is the smaller of the count (at most the capacity) and
    /// `buf.len() / frame size`. A burst from `FIFO_DATA` returns consecutive frames, the whole FIFO if
    /// asked in one read; past the last frame it returns the empty frame, 0x7F bytes (4.6.3, p23; 7.23,
    /// p59). Reads are in the granularity of the frame size (4.6.3, p23), so a frame is never split. With
    /// nothing to read, or no room for one frame after the count, there is only the first transaction.
    /// Bytes of `buf` past `bytes` are not touched. Decode the bytes with [`fifo_frames`]
    /// and the configured [`FifoData`] (`config().fifo.data`).
    ///
    /// Reads longer than 15 bytes are NOT one fully queued frame on embassy-rp 0.10 (contract A4e): the
    /// largest fully queued data read is 2 pressure and temperature frames (12 bytes), or 5 frames of one
    /// quantity (15 bytes). A full FIFO is 96 bytes in either kind. The count read is one byte.
    ///
    /// Errors, all before any bus access: [`Error::NotConfigured`], [`Error::FifoOff`],
    /// [`Error::BufferTooSmall`] when `buf` is shorter than one frame.
    ///
    /// In streaming mode the chip needs a host clock of 0.1 MHz or more, else the readout is slower than
    /// the writing and frames are lost (footnote to the streaming mode, 4.6.1, p23).
    ///
    /// Loss: a frame the chip has sent completely is gone from it (4.6.3, p23); one read incompletely stays
    /// and is sent again. If the data read fails or its future is dropped, the frames it had delivered
    /// may or may not have left the chip, and the driver cannot tell: `buf` holds no usable data, the call
    /// returns the error (or nothing), `Stats::fifo_frames` is not advanced, and the next call starts from
    /// the new count. Frames of the failed read may be lost; the error is the report. A read whose
    /// transaction did not start takes no frame. A count that showed a full FIFO is counted in
    /// [`Stats::fifo_full`] once per call, even if the data read then fails.
    pub async fn fifo_read(&mut self, buf: &mut [u8]) -> Result<FifoRead, Error<I2C::Error>> {
        let data = self.fifo_data_kind()?;
        let size = data.frame_bytes();
        if buf.len() < size {
            return Err(Error::BufferTooSmall);
        }
        let capacity = data.capacity_frames();
        let raw = self.read_register(reg::FIFO_COUNT).await? & reg::FIFO_COUNT_MASK;
        let full = raw >= capacity;
        if full {
            self.stats.fifo_full = self.stats.fifo_full.saturating_add(1);
        }
        let count = raw.min(capacity);
        let fit = u8::try_from(buf.len() / size).unwrap_or(u8::MAX);
        let n = count.min(fit);
        let bytes = usize::from(n) * size;
        if let Some(dst) = buf.get_mut(..bytes).filter(|d| !d.is_empty()) {
            self.read_registers(reg::FIFO_DATA, dst).await?;
            self.stats.fifo_frames = self.stats.fifo_frames.saturating_add(u32::from(n));
        }
        Ok(FifoRead {
            bytes,
            frames: n,
            waiting: count - n,
            full,
        })
    }

    /// Empties the FIFO the way the datasheet prescribes (4.6.4, p25): the FIFO is flushed by a change of
    /// `FIFO_SEL.fifo_frame_sel`, and `FIFO_SEL` must only be changed in standby; the flush empties the FIFO,
    /// resets `FIFO_COUNT` and clears the interrupt conditions, and completes within `treconf`, 3 ms, or
    /// `treconf_deep`, 4 ms, in deep standby (Table 4, p11), before which `FIFO_COUNT` must not be read.
    ///
    /// The sequence: the driver is not configured from the start (contract A3); `ODR_CONFIG` to standby and
    /// a 2.5 ms wait; `FIFO_SEL` written 0, which switches the frame selection off and flushes; a 4 ms
    /// wait; then everything [`configure`](Self::configure) does with the applied configuration (which
    /// writes `FIFO_SEL` back, another change and flush, restores the power mode, and reads `INT_STATUS`,
    /// so the interrupt flags pending are cleared, as the flush clears them anyway); a last 4 ms wait.
    /// The device ends in the applied configuration, the driver configured again, and
    /// [`verify`](Self::verify) is Ok. In normal mode 20 transactions and 4 delays (the longest case); the exact trace
    /// is that of `configure` (18 transactions, 1 delay) plus 2 writes and 2 delays at the front and 1 delay at the back. Measurements pause
    /// and restart, so the first new sample takes a conversion time.
    ///
    /// Errors: [`Error::NotConfigured`], [`Error::FifoOff`] with no bus access. After any other error, or if
    /// the future is dropped before the last wait, the driver is not configured and the FIFO state is
    /// unknown: call [`configure`](Self::configure) (with the same [`Config`]), then flush again if needed.
    /// Dropped in the last wait the driver is configured and the device is in the applied configuration, but
    /// the flush may not have finished: wait 4 ms before [`fifo_count`](Self::fifo_count), or flush again.
    /// Frames in the FIFO are discarded, not returned: read them first.
    pub async fn fifo_flush(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.fifo_data_kind()?;
        let Some(config) = self.config else {
            return Err(Error::NotConfigured);
        };
        self.config = None;
        let plan = config.plan(Apply::InPlace).map_err(Error::InvalidConfig)?;
        self.write_register(reg::ODR_CONFIG, config.odr_config(PowerMode::Standby))
            .await?;
        delay.delay_us(plan::STANDBY_WAIT_US).await;
        self.write_register(reg::FIFO_SEL, reg::FIFO_SEL_RESET)
            .await?;
        delay.delay_us(FLUSH_WAIT_US).await;
        self.apply(delay, &plan, &config, 0, 0).await?;
        delay.delay_us(FLUSH_WAIT_US).await;
        Ok(())
    }

    /// Reads `STATUS` (7.22, p58) in one transaction of one byte: core ready, NVM ready, NVM error and
    /// the others. Not clear-on-read. Works whether or not the driver is configured.
    ///
    /// Measured on the board, 2026-11-10: the NVM-ready bit reads 0 whenever the chip is measuring (normal
    /// mode, continuous mode, with or without the FIFO): `STATUS` reads 0x01 there and 0x03 in standby and
    /// in forced mode at rest. The NVM-ready bit is meaningful only in standby; while the chip runs, judge
    /// health by the core-ready bit, which is set in all of those states.
    pub async fn status(&mut self) -> Result<Status, Error<I2C::Error>> {
        Ok(Status::from_raw(self.read_register(reg::STATUS).await?))
    }

    /// Reads `OSR_EFF` (7.35, p66) in one transaction of one byte: the chip's validity flag and the
    /// oversampling it actually uses. Works whether or not the driver is configured. Measured on the
    /// chip (2026-11-10): the flag is valid at once in standby and in normal mode; an invalid pair keeps
    /// the requested rate and silently replaces the oversampling (in standby the bits echo the request).
    pub async fn effective_oversampling(
        &mut self,
    ) -> Result<EffectiveOversampling, Error<I2C::Error>> {
        Ok(EffectiveOversampling::from_raw(
            self.read_register(reg::OSR_EFF).await?,
        ))
    }

    /// Reads back what the applied configuration owns and compares it with what the configuration puts
    /// there. The expected values are the register images the plan writes, not a second table
    /// (`Config::owned_registers`).
    ///
    /// [`Error::NotConfigured`] while the driver is not configured, with no bus access. Otherwise the owned
    /// registers are read in ascending address order, in windows of at most 15 bytes (contract A4e), each
    /// window one `write_read`: for any [`Config`] that is 2 transactions, `INT_CONFIG` to `FIFO_SEL`
    /// (0x14 to 0x18, 5 bytes, `FIFO_COUNT` read and ignored) and `DSP_CONFIG` to `OSR_CONFIG`/`ODR_CONFIG`
    /// (0x30 to 0x37, 8 bytes) or to `OSR_EFF` (0x30 to 0x38, 9 bytes) when the end mode is
    /// [`PowerMode::Normal`]. The masks leave out reserved bits and the forced-mode power-mode field
    /// (`Config::owned_registers`); in normal mode the chip's validity flag and effective oversampling
    /// are compared with the applied ones. Neither window contains `INT_STATUS`, so no interrupt flag is
    /// consumed.
    ///
    /// An entry that differs under its mask is read once more on its own (one more transaction); only if
    /// it differs again does the call return [`Error::ConfigMismatch`] with the register, mask, expected
    /// value and the whole byte found; it returns at once, so a mismatch in the first window costs 2
    /// transactions in all and one in the second window 3. Nothing is written, and the validity state and the stored
    /// [`Config`] are never changed: the application decides what follows (keep reading,
    /// [`configure`](Self::configure), or switch sensors). Cancelling between transactions is safe; call it
    /// again.
    ///
    /// Confirmed on the real chip (measured on the board, 2026-11-10): no false alarm in 12 unlike
    /// configurations, at 4, 1 and 0.125 Hz (the power-mode field stays "normal" between measurements),
    /// with all eight interrupt-pin combinations (the pad-drive bits read back as written) and with
    /// pressure off (the chip echoes the pressure oversampling). 200 back-to-back calls while streaming at
    /// 240 Hz took 513 us each and left the data-ready flag intact.
    pub async fn verify(&mut self) -> Result<(), Error<I2C::Error>> {
        let Some(config) = self.config else {
            return Err(Error::NotConfigured);
        };
        let owned = config.owned_registers();
        let entries = owned.entries();
        let mut i = 0;
        while let Some(first) = entries.get(i) {
            let start = first.reg;
            let mut end = i;
            while entries
                .get(end)
                .is_some_and(|e| e.reg.wrapping_sub(start) < VERIFY_WINDOW)
            {
                end += 1;
            }
            let window = entries.get(i..end).unwrap_or(&[]);
            let len = window
                .last()
                .map_or(1, |e| usize::from(e.reg.wrapping_sub(start)) + 1);
            let mut buf = [0u8; VERIFY_WINDOW as usize];
            if let Some(dst) = buf.get_mut(..len) {
                self.read_registers(start, dst).await?;
            }
            for o in window {
                let at = usize::from(o.reg.wrapping_sub(start));
                let mut found = buf.get(at).copied().unwrap_or(0);
                if (found ^ o.expected) & o.mask != 0 {
                    found = self.read_register(o.reg).await?;
                    if (found ^ o.expected) & o.mask != 0 {
                        return Err(Error::ConfigMismatch(Mismatch {
                            register: o.reg,
                            mask: o.mask,
                            expected: o.expected,
                            found,
                        }));
                    }
                }
            }
            i = end.max(i + 1);
        }
        Ok(())
    }

    /// Brings the bus back in step after a cancelled or failed transfer, then checks the chip (contract
    /// A4g).
    ///
    /// 1. Waits 2 ms through `delay`, with no bus traffic.
    /// 2. One read of `CHIP_ID`, the result discarded (a bus error is still returned).
    /// 3. `CHIP_ID` and `REV_ID` read together in one transaction of 2 bytes. `CHIP_ID` must be 0x50 and
    ///    `REV_ID` must equal what [`init`](Self::init) read (when `init` has not run since the driver was
    ///    built, `REV_ID` is not compared); a wrong value is [`Error::WrongId`] with `found` the byte that
    ///    differs (`CHIP_ID` first).
    /// 4. [`verify`](Self::verify) when configured, [`Error::NotConfigured`] otherwise.
    ///
    /// Transactions: 4 when configured and everything matches (1 + 1 + the 2 of `verify`), 2 when not
    /// configured. It writes nothing to the device and never changes the validity state or the stored
    /// [`Config`]. Cancelling between transactions is safe; call it again.
    ///
    /// Confirmed on the real chip (measured on the board, 2026-11-10): `recover` took 2.8 ms with the 2 ms
    /// wait included, on a chip in normal mode. As in step 3, the revision is compared only after
    /// [`init`](Self::init) has read it: on a driver object that has only been through
    /// [`configure`](Self::configure) the `REV_ID` byte is read but not compared.
    ///
    /// On embassy-rp 0.10 an I2C transfer in flight must not be cancelled: a cancelled read makes later
    /// reads return wrong data with no error until the bus has been quiet for 2 ms and one read has been
    /// discarded, which is what steps 1 and 2 do (measured on the board, 2026-11-10). Put a timeout around
    /// the data-ready wait, not around the bus call.
    ///
    /// What it cannot do: a data line held low by the device needs a bus clear (pulses on SCL while SDA
    /// is low, START and STOP, rebuild the bus) by the firmware's bus owner; a driver that only holds an
    /// `I2c` cannot (contract A4h).
    pub async fn recover(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        delay.delay_us(RECOVER_QUIET_US).await;
        self.read_register(reg::CHIP_ID).await?;
        let mut identity = [0u8; 2];
        self.read_registers(reg::CHIP_ID, &mut identity).await?;
        let [chip_id, revision] = identity;
        if chip_id != reg::CHIP_ID_VALUE {
            return Err(Error::WrongId { found: chip_id });
        }
        if self.revision.is_some_and(|seen| seen != revision) {
            return Err(Error::WrongId { found: revision });
        }
        self.verify().await
    }

    /// Reads one register in one `write_read` transaction. Raw access: outside the validity state.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut b = [0u8; 1];
        self.read_registers(reg, &mut b).await?;
        let [v] = b;
        Ok(v)
    }

    /// Writes one register. Raw access: outside the validity state and the cancel-safety guarantee of
    /// [`init`](Self::init) and [`configure`](Self::configure); the driver does not track the write.
    ///
    /// The chip ignores writes to some configuration registers while it is measuring: measured on the
    /// board, 2026-11-10, a write of 0x08 to the IIR register in normal mode read back 0x00, while a write
    /// to the interrupt-source register in the same mode was accepted. This is why
    /// [`configure`](Self::configure) goes through standby. [`verify`](Self::verify) is unaffected: it
    /// only reads.
    pub async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.write_bytes(reg, value)
            .await
            .map_err(|e| self.bus_error(e))
    }

    async fn write_bytes(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[reg, value]).await
    }

    /// Reads consecutive registers starting at `reg`, in one `write_read` transaction. Raw access:
    /// outside the validity state. More than 15 bytes are not fully queued before the first await on
    /// embassy-rp (contract A4e).
    pub async fn read_registers(
        &mut self,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .await
            .map_err(|e| self.bus_error(e))
    }
}

#[cfg(test)]
mod testbus;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod driver_tests;

#[cfg(test)]
mod settings_tests;

#[cfg(test)]
mod verify_tests;

#[cfg(test)]
mod fifo_tests;

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
