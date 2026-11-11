//! Host test harness: a fake BMP581 on an I2C bus (a register file with the device's semantics as the
//! driver uses them), a fake delay, a counter of await points and two executors. One copy per driver
//! repository, on purpose.
//!
//! The chip, the bus and the delay share one state so that a test can inspect the traffic after a future
//! was dropped. Awaits are counted in one sequence across bus and delay; a pend at await `k` returns
//! `Poll::Pending` once, before the operation takes effect, and completes afterwards.
//!
//! Device model (datasheet BST-BMP581-DS004-13): a transaction is a register pointer then data bytes
//! (write) or a read from the pointer, both with auto-increment; only the chip's own address is
//! acknowledged. The soft-reset command (`CMD` 0xB6, 7.36, p67) restores the reset values, sets the
//! power-on flag in `INT_STATUS` (4.3.9, p18) and is not acknowledged, as measured on the real chip.
//! `INT_STATUS` clears on read (7.21, p58). `OSR_EFF` (7.35, p66) is computed when `OSR_CONFIG` or
//! `ODR_CONFIG` is written (V1, measured on the board, 2026-11-10: also in standby and with pressure
//! off): with a valid combination it holds the requested oversampling and the valid flag; with an invalid
//! one the flag is clear and, outside standby, the oversampling is replaced by 2x/2x (codes 1/1) at 160 Hz
//! and above and by 4x/4x (codes 2/2) below (V3, same measurement); in standby it still echoes the
//! request. The validity rule is a function the test chooses; the default is the datasheet table read
//! nominally, as written independently in `settings_tests`.
//!
//! The FIFO (4.6, p22-25): frames a test pushes, `FIFO_COUNT` derived from them, a read of
//! `FIFO_DATA` that stays at the port (no address increment) and pops whole frames in order, leaves an
//! incompletely read frame in place (4.6.3, p23) and pads with the empty frame 0x7F; a push into a full
//! FIFO deletes the oldest frame (stream) or is dropped (stop on full) and sets the FIFO-full flag when
//! its interrupt source is enabled; a change of `FIFO_SEL` (frame or decimation), `OSR_CONFIG` or
//! `ODR_CONFIG` (oversampling, rate, power mode) empties it and clears its flags (4.6.4, p25).

use core::future::Future;
use core::pin::{Pin, pin};
use core::task::{Context, Poll, Waker};
use std::sync::{Arc, Mutex, MutexGuard};
use std::vec::Vec;

use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation};

use crate::reg;

/// The bus error the fake raises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BusFault {
    /// An injected failure of the chosen transaction.
    Injected,
    /// The address was not the chip's, or the chip does not acknowledge (the reset command).
    NoAck,
}

impl embedded_hal::i2c::Error for BusFault {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Injected => ErrorKind::Bus,
            Self::NoAck => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
        }
    }
}

/// What one operation of a transaction put on the bus and got back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Op {
    /// `Operation::Write`: the bytes sent.
    Write(Vec<u8>),
    /// `Operation::Read`: the bytes returned.
    Read(Vec<u8>),
}

/// One logged event, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    /// A transaction that reached the chip (the unacknowledged reset included).
    I2c {
        /// The 7-bit address.
        address: u8,
        /// The operations.
        ops: Vec<Op>,
    },
    /// A delay, in nanoseconds.
    Delay(u64),
}

/// A write of `value` to `reg`, as the driver sends it.
pub(crate) fn w(reg: u8, value: u8) -> Event {
    Event::I2c {
        address: crate::DEFAULT_ADDRESS,
        ops: std::vec![Op::Write(std::vec![reg, value])],
    }
}

/// A read of `values` from `reg` on, as one `write_read`: pointer write, then the data.
pub(crate) fn r(reg: u8, values: &[u8]) -> Event {
    Event::I2c {
        address: crate::DEFAULT_ADDRESS,
        ops: std::vec![Op::Write(std::vec![reg]), Op::Read(values.to_vec())],
    }
}

/// A delay of `us` microseconds.
pub(crate) fn d(us: u64) -> Event {
    Event::Delay(us * 1_000)
}

/// Decides whether the chip finds an output data rate and oversampling valid:
/// `(pressure enabled, osr_p code, osr_t code, odr code)`.
pub(crate) type Rule = fn(bool, u8, u8, u8) -> bool;

/// The datasheet table (Tables 7 and 8, p17) read nominally.
pub(crate) fn nominal_table(pressure: bool, osr_p: u8, osr_t: u8, odr: u8) -> bool {
    crate::settings_tests::table_allows(
        pressure,
        usize::from(osr_p),
        usize::from(osr_t),
        usize::from(odr),
    )
}

/// Size of the fake register file; `CMD` is at 0x7E.
const REGS: usize = 0x80;

/// The six data bytes the fake holds at reset and after: temperature 25.0 degC, pressure 27136 Pa.
pub(crate) const DATA: [u8; 6] = [0x00, 0x00, 0x19, 0x00, 0x80, 0x1A];

pub(crate) struct Chip {
    pub regs: [u8; REGS],
    /// The chip's 7-bit address.
    pub address: u8,
    /// The validity rule behind `OSR_EFF.odr_is_valid`.
    pub rule: Rule,
    /// The value `CHIP_ID` reads.
    pub chip_id: u8,
    /// The value `STATUS` reads after a reset.
    pub status: u8,
    pub log: Vec<Event>,
    /// Transactions started since the last `arm` (failed ones included).
    pub transactions: usize,
    /// Awaits reached since the last `arm`.
    pub awaits: u32,
    /// Fail this transaction (1-based) with `BusFault::Injected`; it has no effect on the chip.
    pub fail_at: Option<usize>,
    /// Return `Pending` once at this await (1-based).
    pub pend_at: Option<u32>,
    pub pended: bool,
    /// Bus rules the driver broke: a read without a preceding register pointer.
    pub violations: usize,
    /// Corrupt the next read of this register once, by XOR with the value (a one-off bad read-back).
    pub glitch: Option<(u8, u8)>,
    /// The FIFO: one entry per frame, oldest first.
    pub fifo: Vec<Vec<u8>>,
    /// Make `FIFO_COUNT` read this instead of the derived count (a value the chip cannot produce).
    pub count_forced: Option<u8>,
}

impl Chip {
    fn new(address: u8) -> Self {
        let mut c = Self {
            regs: [0; REGS],
            address,
            rule: nominal_table,
            chip_id: reg::CHIP_ID_VALUE,
            status: reg::STATUS_RESET,
            log: Vec::new(),
            transactions: 0,
            awaits: 0,
            fail_at: None,
            pend_at: None,
            pended: false,
            violations: 0,
            glitch: None,
            fifo: Vec::new(),
            count_forced: None,
        };
        c.restore_reset_values();
        c
    }

    /// The reset values (section 7, p51-67) of the registers the driver touches; the data bytes are kept.
    fn restore_reset_values(&mut self) {
        self.regs = [0; REGS];
        self.fifo.clear();
        let at = usize::from(reg::TEMP_DATA_XLSB);
        self.regs[at..at + DATA.len()].copy_from_slice(&DATA);
        self.regs[usize::from(reg::CHIP_ID)] = self.chip_id;
        self.regs[usize::from(reg::REV_ID)] = reg::REV_ID_VALUE;
        self.regs[usize::from(reg::INT_CONFIG)] = reg::INT_CONFIG_RESET;
        self.regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_RESET;
        self.regs[usize::from(reg::STATUS)] = self.status;
        self.regs[usize::from(reg::DSP_CONFIG)] = reg::DSP_CONFIG_RESET;
        self.regs[usize::from(reg::OSR_CONFIG)] = reg::OSR_CONFIG_RESET;
        self.regs[usize::from(reg::ODR_CONFIG)] = reg::ODR_CONFIG_RESET;
    }

    fn reg_mut(&mut self, addr: u8) -> &mut u8 {
        &mut self.regs[usize::from(addr) & (REGS - 1)]
    }

    /// Recomputes `OSR_EFF` from `OSR_CONFIG` and `ODR_CONFIG`.
    fn evaluate_osr_eff(&mut self) {
        let osr = self.regs[usize::from(reg::OSR_CONFIG)];
        let odr_config = self.regs[usize::from(reg::ODR_CONFIG)];
        let osr_t = osr & 0b111;
        let osr_p = (osr >> reg::OSR_CONFIG_OSR_P_SHIFT) & 0b111;
        let pressure = osr & reg::OSR_CONFIG_PRESS_EN != 0;
        let odr = (odr_config >> reg::ODR_CONFIG_ODR_SHIFT) & 0x1F;
        let standby = odr_config & 0b11 == 0;
        let requested = (osr_p << 3) | osr_t;
        self.regs[usize::from(reg::OSR_EFF)] = if (self.rule)(pressure, osr_p, osr_t, odr) {
            reg::OSR_EFF_ODR_IS_VALID | requested
        } else if standby {
            requested
        } else if odr <= 4 {
            (1 << 3) | 1
        } else {
            (2 << 3) | 2
        };
    }

    /// Frames the FIFO holds when full, from `FIFO_SEL` (4.6, p22); 0 when it is not enabled.
    fn fifo_capacity(&self) -> usize {
        match self.regs[usize::from(reg::FIFO_SEL)] & reg::FIFO_SEL_FRAME_MASK {
            0 => 0,
            3 => 16,
            _ => 32,
        }
    }

    /// A new frame arrives from the measurement.
    pub fn fifo_push(&mut self, frame: &[u8]) {
        let capacity = self.fifo_capacity();
        if capacity == 0 {
            return;
        }
        if self.fifo.len() >= capacity {
            if self.regs[usize::from(reg::FIFO_CONFIG)] & reg::FIFO_CONFIG_STOP_ON_FULL != 0 {
                return;
            }
            self.fifo.remove(0);
        }
        self.fifo.push(frame.to_vec());
        let full_enabled =
            self.regs[usize::from(reg::INT_SOURCE)] & reg::INT_SOURCE_FIFO_FULL_EN != 0;
        if self.fifo.len() == capacity && full_enabled {
            self.regs[usize::from(reg::INT_STATUS)] |= reg::INT_STATUS_FIFO_FULL;
        }
    }

    fn fifo_flush(&mut self) {
        self.fifo.clear();
        self.regs[usize::from(reg::INT_STATUS)] &=
            !(reg::INT_STATUS_FIFO_FULL | reg::INT_STATUS_FIFO_THS);
    }

    /// A burst from `FIFO_DATA`: whole frames pop, an incomplete one stays, the rest is 0x7F.
    fn fifo_burst(&mut self, buf: &mut [u8]) {
        let mut at = 0;
        while at < buf.len() {
            let Some(frame) = self.fifo.first() else {
                break;
            };
            let take = (buf.len() - at).min(frame.len());
            buf[at..at + take].copy_from_slice(&frame[..take]);
            at += take;
            if take == frame.len() {
                self.fifo.remove(0);
            }
        }
        buf[at..].fill(reg::FIFO_EMPTY_FRAME);
    }

    fn write_reg(&mut self, addr: u8, value: u8) {
        let flushes = match addr {
            reg::FIFO_SEL => 0x1F,
            reg::OSR_CONFIG => 0x3F,
            reg::ODR_CONFIG => 0x7F,
            _ => 0,
        };
        if flushes != 0 && (self.regs[usize::from(addr)] ^ value) & flushes != 0 {
            self.fifo_flush();
        }
        match addr {
            reg::CMD => {
                if value == reg::CMD_SOFT_RESET {
                    self.restore_reset_values();
                    self.regs[usize::from(reg::INT_STATUS)] = reg::INT_STATUS_POR;
                }
            }
            reg::OSR_CONFIG | reg::ODR_CONFIG => {
                *self.reg_mut(addr) = value;
                self.evaluate_osr_eff();
            }
            // Read-only registers ignore writes.
            reg::CHIP_ID
            | reg::REV_ID
            | reg::STATUS
            | reg::INT_STATUS
            | reg::OSR_EFF
            | reg::TEMP_DATA_XLSB..=0x22 => {}
            _ => *self.reg_mut(addr) = value,
        }
    }

    fn read_reg(&mut self, addr: u8) -> u8 {
        if addr == reg::FIFO_COUNT {
            return self
                .count_forced
                .unwrap_or_else(|| u8::try_from(self.fifo.len()).unwrap_or(u8::MAX));
        }
        let v = *self.reg_mut(addr);
        if addr == reg::INT_STATUS {
            *self.reg_mut(addr) = 0;
        }
        v
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), BusFault> {
        self.transactions += 1;
        if self.fail_at == Some(self.transactions) {
            return Err(BusFault::Injected);
        }
        if address != self.address {
            return Err(BusFault::NoAck);
        }
        let mut pointer: Option<u8> = None;
        let mut ops = Vec::new();
        let mut unacknowledged = false;
        for op in operations {
            match op {
                Operation::Write(bytes) => {
                    let mut started = false;
                    for &b in bytes.iter() {
                        if !started {
                            pointer = Some(b);
                            started = true;
                        } else if let Some(p) = pointer {
                            unacknowledged |= p == reg::CMD;
                            self.write_reg(p, b);
                            pointer = Some(p.wrapping_add(1));
                        }
                    }
                    ops.push(Op::Write(bytes.to_vec()));
                }
                Operation::Read(buf) => {
                    if pointer.is_none() {
                        self.violations += 1;
                    }
                    let mut p = pointer.unwrap_or(0);
                    if p == reg::FIFO_DATA {
                        self.fifo_burst(buf);
                        ops.push(Op::Read(buf.to_vec()));
                        continue;
                    }
                    for b in buf.iter_mut() {
                        *b = self.read_reg(p);
                        if let Some((g, x)) = self.glitch
                            && g == p
                        {
                            *b ^= x;
                            self.glitch = None;
                        }
                        p = p.wrapping_add(1);
                    }
                    pointer = Some(p);
                    ops.push(Op::Read(buf.to_vec()));
                }
            }
        }
        self.log.push(Event::I2c { address, ops });
        if unacknowledged {
            Err(BusFault::NoAck)
        } else {
            Ok(())
        }
    }
}

/// The shared state; clone it to keep a handle after the bus has moved into the driver.
#[derive(Clone)]
pub(crate) struct Shared(Arc<Mutex<Chip>>);

impl Shared {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(Chip::new(crate::DEFAULT_ADDRESS))))
    }

    pub fn chip(&self) -> MutexGuard<'_, Chip> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn bus(&self) -> TestBus {
        TestBus(self.clone())
    }

    pub fn delay(&self) -> TestDelay {
        TestDelay(self.clone())
    }

    /// Clears the log and the counters and sets the injections; the register file stays.
    pub fn arm(&self, fail_at: Option<usize>, pend_at: Option<u32>) {
        let mut c = self.chip();
        c.log.clear();
        c.transactions = 0;
        c.awaits = 0;
        c.fail_at = fail_at;
        c.pend_at = pend_at;
        c.pended = false;
        c.violations = 0;
    }

    pub fn log(&self) -> Vec<Event> {
        self.chip().log.clone()
    }

    pub fn awaits(&self) -> u32 {
        self.chip().awaits
    }

    pub fn transactions(&self) -> usize {
        self.chip().transactions
    }

    pub fn violations(&self) -> usize {
        self.chip().violations
    }

    /// A frame arrives in the FIFO.
    pub fn push_frame(&self, frame: &[u8]) {
        self.chip().fifo_push(frame);
    }

    pub fn fifo_len(&self) -> usize {
        self.chip().fifo.len()
    }

    pub fn reg(&self, addr: u8) -> u8 {
        self.chip().regs[usize::from(addr)]
    }
}

/// An await point: counts itself, and pends once if it is the chosen one.
struct Tick {
    shared: Shared,
    counted: bool,
}

impl Future for Tick {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.counted {
            return Poll::Ready(());
        }
        self.counted = true;
        let mut c = self.shared.chip();
        c.awaits += 1;
        if c.pend_at == Some(c.awaits) && !c.pended {
            c.pended = true;
            return Poll::Pending;
        }
        Poll::Ready(())
    }
}

fn tick(shared: &Shared) -> Tick {
    Tick {
        shared: shared.clone(),
        counted: false,
    }
}

/// The fake I2C bus. Both the async and the blocking trait are implemented over one state.
pub(crate) struct TestBus(Shared);

impl ErrorType for TestBus {
    type Error = BusFault;
}

impl embedded_hal_async::i2c::I2c for TestBus {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), BusFault> {
        tick(&self.0).await;
        self.0.chip().transaction(address, operations)
    }
}

impl embedded_hal::i2c::I2c for TestBus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), BusFault> {
        self.0.chip().transaction(address, operations)
    }
}

/// The fake delay; it records and does not wait.
pub(crate) struct TestDelay(Shared);

impl embedded_hal_async::delay::DelayNs for TestDelay {
    async fn delay_ns(&mut self, ns: u32) {
        tick(&self.0).await;
        self.0.chip().log.push(Event::Delay(u64::from(ns)));
    }
}

impl embedded_hal::delay::DelayNs for TestDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.chip().log.push(Event::Delay(u64::from(ns)));
    }
}

/// Runs a future to completion with a no-op waker, polling again after every `Pending`.
pub(crate) fn run<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(v) = future.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// How a future that was polled until its first `Pending` ended.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Polled<T> {
    /// It completed without ever pending: the chosen await is beyond its last.
    Done(T),
    /// It returned `Pending` and was dropped there.
    Cancelled,
}

/// Polls until the first `Pending`, then drops the future.
pub(crate) fn poll_then_drop<F: Future>(future: F) -> Polled<F::Output> {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    match future.as_mut().poll(&mut cx) {
        Poll::Ready(v) => Polled::Done(v),
        Poll::Pending => Polled::Cancelled,
    }
}
