//! The register plan: the ordered steps that configure the device from a [`Config`].
//!
//! The plan is data. It is built without a bus and without allocation; a driver only executes it:
//! [`Step::Write`] is one register write, [`Step::Read`] a register read whose value is dropped,
//! [`Step::DelayUs`] a wait, [`Step::CheckRateValid`] a read of `OSR_EFF` that must show the chip's validity
//! flag set. No step is a read-modify-write: every register the plan writes is written
//! whole, and the fields it does not configure take their reset value (see `image`).
//!
//! # Order
//!
//! The plan starts after the identity and NVM checks of `init` and does not contain the soft reset.
//!
//! 1. Interrupts are cleared and the chip is in standby. After a reset the chip is in deep standby, which
//!    is a standby (4.3, p16; 4.3.2, p16), and `INT_STATUS` is read to clear the reset flag (4.3.9, p18).
//!    In place the chip may be in any mode: `ODR_CONFIG` is written with `pwr_mode` standby and the plan waits
//!    [`STANDBY_WAIT_US`] before it writes anything else (4.3.7, p18).
//! 2. `OSR_CONFIG`, then `ODR_CONFIG` in standby. When the mode is normal, [`Step::CheckRateValid`] follows:
//!    the chip evaluates `OSR_EFF.odr_is_valid` as soon as both registers are written, also in standby
//!    (4.3.4, p17; 7.35, p66; measured on the chip, 2026-11-10: the flag read in standby and the flag read
//!    after entering normal mode never differed, 2048 + 512 combinations). A clear flag stops the executor
//!    here, with the device still in standby and nothing after this step written. In the other modes the
//!    plan has no check: the rate is unused and a configuration legal there must not fail.
//! 3. The filters (`DSP_CONFIG`, `DSP_IIR`) and the FIFO (`FIFO_SEL`, `FIFO_CONFIG`) and the out-of-range
//!    window: these registers take writes in standby only, writes in other modes are lost (4.3.8, p18; 4.6.4,
//!    p25). A FIFO flush follows any change of these settings (4.6.4, p25).
//! 4. The interrupt: the sources, the pin, then `INT_STATUS` is read to clear what the changes raised. In
//!    place, a change between pulsed and latched mode needs the sources off, `INT_STATUS` read, then the
//!    mode set (4.7.3.2, p28): the plan writes `INT_SOURCE` 0 and reads `INT_STATUS` first.
//! 5. `ODR_CONFIG` with the power mode (the only step that leaves standby), unless the mode is [`PowerMode::Forced`]: forced measurements
//!    are triggered one by one and the chip stays in standby (the first measurement starts directly after
//!    the transition, 4.3.7, p18).

use crate::config::{Config, ConfigError, PowerMode};
use crate::reg;

/// Wait after a commanded switch to standby: `tstandby` at most 2.5 ms (Table 4, p11; 4.3.7, p18).
pub const STANDBY_WAIT_US: u32 = 2_500;

/// Most steps a plan can hold. The longest plan, in place and in normal mode, has 19.
pub const MAX_STEPS: usize = 20;

/// One step of a plan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Step {
    /// Write `value` to register `reg`.
    Write {
        /// Register address.
        reg: u8,
        /// The byte to write.
        value: u8,
    },
    /// Read register `reg` and drop the value (`INT_STATUS` is clear-on-read).
    Read {
        /// Register address.
        reg: u8,
    },
    /// Wait this many microseconds.
    DelayUs(u32),
    /// Read `OSR_EFF` and require `odr_is_valid` (7.35, p66). A clear flag ends the run with
    /// [`crate::Error::ChipRejectsRate`] carrying the byte read. Only in the plan of a normal-mode
    /// configuration, between the standby rate write and the rest.
    CheckRateValid,
}

/// Where the plan starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Apply {
    /// Right after a soft reset: the registers are at their reset values (7.36, p67), so the FIFO and
    /// out-of-range registers are written only when their image differs from the reset value. The
    /// registers `init` has always written are always written.
    AfterReset,
    /// On a chip in an unknown state, without a reset: every register is written, after a switch to
    /// standby and its wait.
    InPlace,
}

/// A register plan. Iterate it with [`Plan::steps`] or `for step in &plan`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Plan {
    steps: [Step; MAX_STEPS],
    len: usize,
}

impl Plan {
    /// A plan with no step.
    pub const fn empty() -> Self {
        Self {
            steps: [Step::DelayUs(0); MAX_STEPS],
            len: 0,
        }
    }

    /// The steps, in order.
    pub fn steps(&self) -> &[Step] {
        self.steps.get(..self.len).unwrap_or(&[])
    }

    /// An iterator over the steps.
    pub fn iter(&self) -> core::slice::Iter<'_, Step> {
        self.steps().iter()
    }

    /// Number of steps.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// True when there is no step.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Appends a step; [`ConfigError::PlanOverflow`] when the array is full. A step is never dropped.
    pub fn push(&mut self, step: Step) -> Result<(), ConfigError> {
        let slot = self
            .steps
            .get_mut(self.len)
            .ok_or(ConfigError::PlanOverflow)?;
        *slot = step;
        self.len += 1;
        Ok(())
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), ConfigError> {
        self.push(Step::Write { reg, value })
    }

    fn write_unless_reset(&mut self, apply: Apply, reg: u8, value: u8) -> Result<(), ConfigError> {
        if apply == Apply::InPlace || value != 0 {
            self.write(reg, value)?;
        }
        Ok(())
    }
}

impl Default for Plan {
    fn default() -> Self {
        Self::empty()
    }
}

impl<'a> IntoIterator for &'a Plan {
    type Item = &'a Step;
    type IntoIter = core::slice::Iter<'a, Step>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl Config {
    /// Validates the configuration and builds the register plan for it. A plan that would not fit in
    /// [`MAX_STEPS`] is [`ConfigError::PlanOverflow`], never a plan with a step dropped.
    ///
    /// For `Config::default()` and [`Apply::AfterReset`] the plan is the reads and writes `init` makes
    /// after its identity and NVM checks (`INT_STATUS` read, `OSR_CONFIG`, `ODR_CONFIG` in standby,
    /// `OSR_EFF` check, `DSP_CONFIG`, `DSP_IIR`, `INT_SOURCE`, `INT_CONFIG`, `INT_STATUS` read, `ODR_CONFIG`
    /// in normal mode).
    pub fn plan(&self, apply: Apply) -> Result<Plan, ConfigError> {
        self.validate()?;
        let mut p = Plan::empty();
        let standby = self.odr_config(PowerMode::Standby);

        if apply == Apply::InPlace {
            p.write(reg::ODR_CONFIG, standby)?;
            p.push(Step::DelayUs(STANDBY_WAIT_US))?;
        } else {
            p.push(Step::Read {
                reg: reg::INT_STATUS,
            })?;
        }

        p.write(reg::OSR_CONFIG, self.osr_config())?;
        p.write(reg::ODR_CONFIG, standby)?;
        if self.power_mode == PowerMode::Normal {
            p.push(Step::CheckRateValid)?;
        }
        p.write(reg::DSP_CONFIG, self.dsp_config())?;
        p.write(reg::DSP_IIR, self.dsp_iir())?;
        p.write_unless_reset(apply, reg::FIFO_SEL, self.fifo_sel())?;
        p.write_unless_reset(apply, reg::FIFO_CONFIG, self.fifo_config())?;
        p.write_unless_reset(apply, reg::OOR_THR_P_LSB, self.oor_thr_p_lsb())?;
        p.write_unless_reset(apply, reg::OOR_THR_P_MSB, self.oor_thr_p_msb())?;
        p.write_unless_reset(apply, reg::OOR_RANGE, self.oor_range())?;
        p.write_unless_reset(apply, reg::OOR_CONFIG, self.oor_config())?;

        if apply == Apply::InPlace {
            p.write(reg::INT_SOURCE, reg::INT_SOURCE_RESET)?;
            p.push(Step::Read {
                reg: reg::INT_STATUS,
            })?;
            p.write(reg::INT_CONFIG, self.int_config())?;
            p.write(reg::INT_SOURCE, self.int_source())?;
        } else {
            p.write(reg::INT_SOURCE, self.int_source())?;
            p.write(reg::INT_CONFIG, self.int_config())?;
        }
        p.push(Step::Read {
            reg: reg::INT_STATUS,
        })?;

        if self.power_mode != PowerMode::Forced {
            p.write(reg::ODR_CONFIG, self.odr_config(self.power_mode))?;
        }
        Ok(p)
    }
}
