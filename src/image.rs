//! Register images: the value of each configuration register as a pure function of a [`Config`].
//!
//! An image sets only the bits of fields that [`Config`] owns. Reserved bits, and the fields this crate
//! does not configure (`ODR_CONFIG.deep_dis`, `DSP_CONFIG.comp_pt_en`, `INT_CONFIG.pad_int_drv`), keep
//! their reset value (section 7). [`Config::validate`] decides whether a combination is allowed; the
//! images are total functions that do not check it.

use crate::config::{
    Config, FifoData, FifoMode, IirTap, InterruptDrive, InterruptMode, InterruptPolarity, PowerMode,
};
use crate::reg;

const fn bit(cond: bool, mask: u8) -> u8 {
    if cond { mask } else { 0 }
}

const fn tap(tap: IirTap, filter_on: bool) -> bool {
    match tap {
        IirTap::Auto => filter_on,
        IirTap::Unfiltered => false,
        IirTap::Filtered => true,
    }
}

impl Config {
    /// `OSR_CONFIG` (7.33, p64): `osr_t` bits 2:0, `osr_p` bits 5:3, `press_en` bit 6.
    pub const fn osr_config(&self) -> u8 {
        (reg::OSR_CONFIG_RESET & reg::OSR_CONFIG_RESERVED)
            | bit(self.pressure_enabled, reg::OSR_CONFIG_PRESS_EN)
            | (self.pressure_oversampling.code() << reg::OSR_CONFIG_OSR_P_SHIFT)
            | self.temperature_oversampling.code()
    }

    /// `ODR_CONFIG` for this rate and the given power mode (7.34, p64-65): `pwr_mode` bits 1:0, `odr`
    /// bits 6:2. `deep_dis` keeps its reset value 0.
    pub const fn odr_config(&self, mode: PowerMode) -> u8 {
        (reg::ODR_CONFIG_RESET & reg::ODR_CONFIG_DEEP_DIS)
            | (self.odr.code() << reg::ODR_CONFIG_ODR_SHIFT)
            | mode.code()
    }

    /// `DSP_CONFIG` (7.27, p61): the reset value 0x03 (compensation), the source of each data path
    /// (bits 7:3, see [`IirTap`]) and the forced-mode flush (bit 2).
    pub const fn dsp_config(&self) -> u8 {
        let p = !matches!(self.pressure_iir, crate::IirFilter::Bypass);
        let t = !matches!(self.temperature_iir, crate::IirFilter::Bypass);
        let k = &self.iir_taps;
        reg::DSP_CONFIG_RESET
            | bit(self.iir_flush_forced, reg::DSP_CONFIG_IIR_FLUSH_FORCED_EN)
            | bit(tap(k.data_temperature, t), reg::DSP_CONFIG_SHDW_SEL_IIR_T)
            | bit(tap(k.fifo_temperature, t), reg::DSP_CONFIG_FIFO_SEL_IIR_T)
            | bit(tap(k.data_pressure, p), reg::DSP_CONFIG_SHDW_SEL_IIR_P)
            | bit(tap(k.fifo_pressure, p), reg::DSP_CONFIG_FIFO_SEL_IIR_P)
            | bit(
                tap(k.out_of_range_pressure, p),
                reg::DSP_CONFIG_OOR_SEL_IIR_P,
            )
    }

    /// `DSP_IIR` (7.28, p62): `set_iir_t` bits 2:0, `set_iir_p` bits 5:3.
    pub const fn dsp_iir(&self) -> u8 {
        (reg::DSP_IIR_RESET & reg::DSP_IIR_RESERVED)
            | (self.pressure_iir.code() << reg::DSP_IIR_P_SHIFT)
            | self.temperature_iir.code()
    }

    /// `INT_SOURCE` (7.6, p54): one enable bit per source.
    pub const fn int_source(&self) -> u8 {
        let s = &self.interrupt.sources;
        (reg::INT_SOURCE_RESET & reg::INT_SOURCE_RESERVED)
            | bit(s.data_ready, reg::INT_SOURCE_DRDY_EN)
            | bit(s.fifo_full, reg::INT_SOURCE_FIFO_FULL_EN)
            | bit(s.fifo_threshold, reg::INT_SOURCE_FIFO_THS_EN)
            | bit(s.out_of_range, reg::INT_SOURCE_OOR_P_EN)
    }

    /// `INT_CONFIG` (7.5, p52-53): mode, polarity, drive and pin enable. The pad drive strength
    /// (bits 7:4) keeps its reset value 3.
    pub const fn int_config(&self) -> u8 {
        let p = &self.interrupt;
        (reg::INT_CONFIG_RESET & reg::INT_CONFIG_PAD_DRV_MASK)
            | bit(
                matches!(p.mode, InterruptMode::Latched),
                reg::INT_CONFIG_LATCHED,
            )
            | bit(
                matches!(p.polarity, InterruptPolarity::ActiveHigh),
                reg::INT_CONFIG_ACTIVE_HIGH,
            )
            | bit(
                matches!(p.drive, InterruptDrive::OpenDrain),
                reg::INT_CONFIG_OPEN_DRAIN,
            )
            | bit(p.enabled, reg::INT_CONFIG_EN)
    }

    /// `FIFO_CONFIG` (7.7, p54): threshold bits 4:0 and mode bit 5. The reset value with the FIFO off.
    pub const fn fifo_config(&self) -> u8 {
        let f = &self.fifo;
        if matches!(f.mode, FifoMode::Off) {
            return reg::FIFO_CONFIG_RESET;
        }
        (reg::FIFO_CONFIG_RESET & reg::FIFO_CONFIG_RESERVED)
            | bit(
                matches!(f.mode, FifoMode::StopOnFull),
                reg::FIFO_CONFIG_STOP_ON_FULL,
            )
            | (f.threshold & reg::FIFO_CONFIG_THRESHOLD_MASK)
    }

    /// `FIFO_SEL` (7.9, p55): `fifo_frame_sel` bits 1:0 and `fifo_dec_sel` bits 4:2. The reset value with
    /// the FIFO off.
    pub const fn fifo_sel(&self) -> u8 {
        let f = &self.fifo;
        if matches!(f.mode, FifoMode::Off) {
            return reg::FIFO_SEL_RESET;
        }
        let frame = match f.data {
            FifoData::Temperature => 0b01,
            FifoData::Pressure => 0b10,
            FifoData::PressureAndTemperature => 0b11,
        };
        (reg::FIFO_SEL_RESET & reg::FIFO_SEL_RESERVED)
            | frame
            | (f.decimation.code() << reg::FIFO_SEL_DEC_SHIFT)
    }

    /// `OOR_THR_P_LSB` (7.29, p63): bits 7:0 of the reference pressure; the reset value without a window.
    pub const fn oor_thr_p_lsb(&self) -> u8 {
        match &self.out_of_range {
            Some(w) => (w.reference_pa & 0xFF) as u8,
            None => 0,
        }
    }

    /// `OOR_THR_P_MSB` (7.30, p63): bits 15:8 of the reference pressure; the reset value without a window.
    pub const fn oor_thr_p_msb(&self) -> u8 {
        match &self.out_of_range {
            Some(w) => ((w.reference_pa >> 8) & 0xFF) as u8,
            None => 0,
        }
    }

    /// `OOR_RANGE` (7.31, p63): the half width of the window in pascals; the reset value without a window.
    pub const fn oor_range(&self) -> u8 {
        match &self.out_of_range {
            Some(w) => w.range_pa,
            None => 0,
        }
    }

    /// `OOR_CONFIG` (7.32, p63): bit 16 of the reference in bit 0, `cnt_lim` in bits 7:6; the reset
    /// value without a window.
    pub const fn oor_config(&self) -> u8 {
        match &self.out_of_range {
            Some(w) => {
                bit((w.reference_pa >> 16) & 1 != 0, reg::OOR_CONFIG_THR_P_16)
                    | (w.count.code() << reg::OOR_CONFIG_CNT_LIM_SHIFT)
            }
            None => 0,
        }
    }
}

/// One thing the applied configuration owns: the bits `mask` of register `reg` hold `expected`
/// (already ANDed with `mask`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Owned {
    pub(crate) reg: u8,
    pub(crate) mask: u8,
    pub(crate) expected: u8,
}

/// Most entries [`Config::owned_registers`] lists.
pub(crate) const OWNED_MAX: usize = 13;

/// The owned entries, in ascending register order: the order `verify` reads them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OwnedList {
    entries: [Owned; OWNED_MAX],
    len: usize,
}

impl OwnedList {
    pub(crate) fn entries(&self) -> &[Owned] {
        self.entries.get(..self.len).unwrap_or(&[])
    }
}

impl Config {
    /// What the applied configuration owns, in ascending register order (the order `verify` reads
    /// it). Every register the plans write is listed, whether or not the plan of a given path wrote it
    /// (after a reset the plan skips registers whose image is the reset value, which the chip holds
    /// anyway). The expected values are the images above, the ones the plan writes: there is no second
    /// table. Page numbers are those of BST-BMP581-DS004-13.
    ///
    /// 1. `INT_CONFIG` 0x14, all 8 bits: mode, polarity, drive, enable, pad drive (7.5, p52-53; the
    ///    pad drive field reads back only after 1 us, far less than one bus frame).
    /// 2. `INT_SOURCE` 0x15 under 0x0F; bits 7:4 are reserved, read-only (7.6, p54).
    /// 3. `FIFO_CONFIG` 0x16 under 0x3F; bits 7:6 are reserved, read-only (7.7, p54).
    /// 4. `FIFO_SEL` 0x18 under 0x1F; bits 7:5 are reserved, read-only (7.9, p55). (`FIFO_COUNT` 0x17,
    ///    7.8 p55, changes by itself and is not owned.)
    /// 5. `DSP_CONFIG` 0x30, all 8 bits (7.27, p61).
    /// 6. `DSP_IIR` 0x31 under 0x3F; bits 7:6 are reserved, read-only (7.28, p62).
    /// 7. `OOR_THR_P_LSB` 0x32, `OOR_THR_P_MSB` 0x33, `OOR_RANGE` 0x34, all 8 bits (7.29 to 7.31, p63).
    /// 8. `OOR_CONFIG` 0x35 under 0xC1; bits 5:1 are reserved, read-only (7.32, p63).
    /// 9. `OSR_CONFIG` 0x36 under 0x7F; bit 7 is reserved, read-only (7.33, p64).
    /// 10. `ODR_CONFIG` 0x37: `deep_dis`, `odr` and `pwr_mode` (7.34, p64-65). A read of `pwr_mode`
    ///     "returns the actual mode the device is in", so it is compared (mask 0xFF) for standby,
    ///     normal and continuous mode. In forced mode the chip returns to standby by itself ("cleared
    ///     again upon transition from FORCED to STANDBY", p64), so the field reads forced during a
    ///     measurement and standby after it: the driver cannot predict it, and in forced mode the mask is
    ///     0xFC (the field is not owned).
    /// 11. Only when the end mode is normal: `OSR_EFF` 0x38 under 0xBF (7.35, p66; bit 6 is reserved,
    ///     read-only): the chip's validity flag set and the effective pressure and temperature
    ///     oversampling equal to the applied ones. In the other modes the chip echoes the request or
    ///     the datasheet does not state the limit, so it is not owned (see `init`).
    ///
    /// Not owned because the chip changes them by itself or they are status: `FIFO_COUNT`,
    /// `INT_STATUS` (clear-on-read, 7.21 p58), `STATUS` (7.22 p58), the data and `FIFO_DATA` registers.
    pub(crate) const fn owned_registers(&self) -> OwnedList {
        const fn o(reg: u8, mask: u8, image: u8) -> Owned {
            Owned {
                reg,
                mask,
                expected: image & mask,
            }
        }
        let normal = matches!(self.power_mode, PowerMode::Normal);
        let odr_mask = if matches!(self.power_mode, PowerMode::Forced) {
            !reg::ODR_CONFIG_PWR_MODE_MASK
        } else {
            0xFF
        };
        let eff_mask = reg::OSR_EFF_ODR_IS_VALID | 0x3F;
        let mut list = OwnedList {
            entries: [
                o(reg::INT_CONFIG, 0xFF, self.int_config()),
                o(
                    reg::INT_SOURCE,
                    !reg::INT_SOURCE_RESERVED,
                    self.int_source(),
                ),
                o(
                    reg::FIFO_CONFIG,
                    !reg::FIFO_CONFIG_RESERVED,
                    self.fifo_config(),
                ),
                o(reg::FIFO_SEL, !reg::FIFO_SEL_RESERVED, self.fifo_sel()),
                o(reg::DSP_CONFIG, 0xFF, self.dsp_config()),
                o(reg::DSP_IIR, !reg::DSP_IIR_RESERVED, self.dsp_iir()),
                o(reg::OOR_THR_P_LSB, 0xFF, self.oor_thr_p_lsb()),
                o(reg::OOR_THR_P_MSB, 0xFF, self.oor_thr_p_msb()),
                o(reg::OOR_RANGE, 0xFF, self.oor_range()),
                o(
                    reg::OOR_CONFIG,
                    !reg::OOR_CONFIG_RESERVED,
                    self.oor_config(),
                ),
                o(
                    reg::OSR_CONFIG,
                    !reg::OSR_CONFIG_RESERVED,
                    self.osr_config(),
                ),
                o(reg::ODR_CONFIG, odr_mask, self.odr_config(self.power_mode)),
                o(
                    reg::OSR_EFF,
                    eff_mask,
                    reg::OSR_EFF_ODR_IS_VALID | (self.osr_config() & 0x3F),
                ),
            ],
            len: OWNED_MAX - 1,
        };
        if normal {
            list.len = OWNED_MAX;
        }
        list
    }
}
