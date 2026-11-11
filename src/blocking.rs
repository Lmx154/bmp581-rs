//! Blocking front-end: the async driver run to completion over adapters that never pend.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorType, I2c, Operation};

use crate::{
    Config, EffectiveOversampling, Error, FifoRead, Info, IntStatus, RawSample, Stats, Status,
};

/// Async-trait view of a blocking bus. Its futures complete on the first poll.
struct BlockingBus<I2C>(I2C);

impl<I2C: ErrorType> ErrorType for BlockingBus<I2C> {
    type Error = I2C::Error;
}

impl<I2C: I2c> embedded_hal_async::i2c::I2c for BlockingBus<I2C> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.0.transaction(address, operations)
    }
}

/// Async-trait view of a blocking delay.
struct BlockingDelay<'a, D>(&'a mut D);

impl<D: DelayNs> embedded_hal_async::delay::DelayNs for BlockingDelay<'_, D> {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.delay_ns(ns);
    }
}

fn block_on<F: Future>(f: F) -> F::Output {
    let mut f = pin!(f);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// BMP581 driver over a blocking `embedded_hal::i2c::I2c`. Same operations as [`crate::Bmp581`].
pub struct Bmp581<I2C> {
    inner: crate::Bmp581<BlockingBus<I2C>>,
}

impl<I2C: I2c> Bmp581<I2C> {
    /// Wraps the bus. `address` is [`crate::DEFAULT_ADDRESS`] or [`crate::ALTERNATE_ADDRESS`].
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self {
            inner: crate::Bmp581::new(BlockingBus(i2c), address),
        }
    }

    /// Returns the bus.
    pub fn release(self) -> I2C {
        self.inner.release().0
    }

    /// Resets, checks the chip and applies `config`. See [`crate::Bmp581::init`].
    pub fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        block_on(self.inner.init(&mut BlockingDelay(delay), config))
    }

    /// Applies `config` without a reset. See [`crate::Bmp581::configure`].
    pub fn configure(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        block_on(self.inner.configure(&mut BlockingDelay(delay), config))
    }

    /// The applied configuration; `None` while not configured. See [`crate::Bmp581::config`].
    pub const fn config(&self) -> Option<&Config> {
        self.inner.config()
    }

    /// Reads temperature and pressure in one transaction. See [`crate::Bmp581::read`]: before the first
    /// conversion has finished it returns [`Error::NoSample`].
    pub fn read(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        block_on(self.inner.read())
    }

    /// Starts one forced-mode measurement.
    pub fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
        block_on(self.inner.trigger_forced())
    }

    /// Reads and clears `INT_STATUS`.
    pub fn interrupt_status(&mut self) -> Result<IntStatus, Error<I2C::Error>> {
        block_on(self.inner.interrupt_status())
    }

    /// Reads `FIFO_COUNT`. See [`crate::Bmp581::fifo_count`].
    pub fn fifo_count(&mut self) -> Result<u8, Error<I2C::Error>> {
        block_on(self.inner.fifo_count())
    }

    /// Reads the count, then the whole frames that fit in `buf`. See [`crate::Bmp581::fifo_read`].
    pub fn fifo_read(&mut self, buf: &mut [u8]) -> Result<FifoRead, Error<I2C::Error>> {
        block_on(self.inner.fifo_read(buf))
    }

    /// Empties the FIFO. See [`crate::Bmp581::fifo_flush`].
    pub fn fifo_flush(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        block_on(self.inner.fifo_flush(&mut BlockingDelay(delay)))
    }

    /// Reads `STATUS`. See [`crate::Bmp581::status`].
    pub fn status(&mut self) -> Result<Status, Error<I2C::Error>> {
        block_on(self.inner.status())
    }

    /// Reads `OSR_EFF`. See [`crate::Bmp581::effective_oversampling`].
    pub fn effective_oversampling(&mut self) -> Result<EffectiveOversampling, Error<I2C::Error>> {
        block_on(self.inner.effective_oversampling())
    }

    /// Compares the registers the applied configuration owns. See [`crate::Bmp581::verify`].
    pub fn verify(&mut self) -> Result<(), Error<I2C::Error>> {
        block_on(self.inner.verify())
    }

    /// Re-syncs the bus and checks the chip. See [`crate::Bmp581::recover`].
    pub fn recover(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        block_on(self.inner.recover(&mut BlockingDelay(delay)))
    }

    /// The counters. See [`crate::Bmp581::stats`].
    pub const fn stats(&self) -> Stats {
        self.inner.stats()
    }

    /// Reads one register.
    pub fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        block_on(self.inner.read_register(reg))
    }

    /// Writes one register.
    pub fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        block_on(self.inner.write_register(reg, value))
    }

    /// Reads consecutive registers starting at `reg`.
    pub fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        block_on(self.inner.read_registers(reg, buf))
    }
}
