//! Register map, names as in the datasheet (BST-BMP581-DS004-13, section 7).

/// Chip identity, reads [`CHIP_ID_VALUE`].
pub const CHIP_ID: u8 = 0x01;
/// ASIC revision, reset value 0x32.
pub const REV_ID: u8 = 0x02;
/// Host interface status.
pub const CHIP_STATUS: u8 = 0x11;
/// Host interface drive and CSB pull-up.
pub const DRIVE_CONFIG: u8 = 0x13;
/// INT pin configuration, reset value 0x35.
pub const INT_CONFIG: u8 = 0x14;
/// INT source selection.
pub const INT_SOURCE: u8 = 0x15;
/// FIFO threshold and mode.
pub const FIFO_CONFIG: u8 = 0x16;
/// FIFO fill level.
pub const FIFO_COUNT: u8 = 0x17;
/// FIFO frame and decimation selection.
pub const FIFO_SEL: u8 = 0x18;
/// Temperature bits 7:0. 0x1D..=0x1F is the 24-bit temperature, little endian.
pub const TEMP_DATA_XLSB: u8 = 0x1D;
/// Temperature bits 15:8.
pub const TEMP_DATA_LSB: u8 = 0x1E;
/// Temperature bits 23:16.
pub const TEMP_DATA_MSB: u8 = 0x1F;
/// Pressure bits 7:0. 0x20..=0x22 is the 24-bit pressure, little endian.
pub const PRESS_DATA_XLSB: u8 = 0x20;
/// Pressure bits 15:8.
pub const PRESS_DATA_LSB: u8 = 0x21;
/// Pressure bits 23:16.
pub const PRESS_DATA_MSB: u8 = 0x22;
/// Interrupt status, cleared by reading.
pub const INT_STATUS: u8 = 0x27;
/// NVM and core status.
pub const STATUS: u8 = 0x28;
/// FIFO output port.
pub const FIFO_DATA: u8 = 0x29;
/// NVM row address.
pub const NVM_ADDR: u8 = 0x2B;
/// NVM data, low byte.
pub const NVM_DATA_LSB: u8 = 0x2C;
/// NVM data, high byte.
pub const NVM_DATA_MSB: u8 = 0x2D;
/// Data path configuration (compensation, IIR source selection).
pub const DSP_CONFIG: u8 = 0x30;
/// IIR filter coefficients.
pub const DSP_IIR: u8 = 0x31;
/// Out-of-range pressure threshold, bits 7:0.
pub const OOR_THR_P_LSB: u8 = 0x32;
/// Out-of-range pressure threshold, bits 15:8.
pub const OOR_THR_P_MSB: u8 = 0x33;
/// Out-of-range window.
pub const OOR_RANGE: u8 = 0x34;
/// Out-of-range threshold bit 16 and count limit.
pub const OOR_CONFIG: u8 = 0x35;
/// Oversampling configuration.
pub const OSR_CONFIG: u8 = 0x36;
/// Output data rate and power mode.
pub const ODR_CONFIG: u8 = 0x37;
/// Effective oversampling and ODR validity.
pub const OSR_EFF: u8 = 0x38;
/// Command register; write [`CMD_SOFT_RESET`] to reset.
pub const CMD: u8 = 0x7E;

/// Value of `CHIP_ID`.
pub const CHIP_ID_VALUE: u8 = 0x50;
/// `CMD` value for a soft reset.
pub const CMD_SOFT_RESET: u8 = 0xB6;

/// `STATUS` bit 1: ready for NVM operations.
pub const STATUS_NVM_RDY: u8 = 1 << 1;
/// `STATUS` bit 2: NVM error.
pub const STATUS_NVM_ERR: u8 = 1 << 2;

/// `INT_STATUS` bit 0: data ready.
pub const INT_STATUS_DRDY: u8 = 1 << 0;
/// `INT_STATUS` bit 1: FIFO full.
pub const INT_STATUS_FIFO_FULL: u8 = 1 << 1;
/// `INT_STATUS` bit 2: FIFO threshold.
pub const INT_STATUS_FIFO_THS: u8 = 1 << 2;
/// `INT_STATUS` bit 3: pressure out of range.
pub const INT_STATUS_OOR_P: u8 = 1 << 3;
/// `INT_STATUS` bit 4: power-on or soft reset complete.
pub const INT_STATUS_POR: u8 = 1 << 4;

/// `INT_SOURCE` bit 0: data-ready enable.
pub const INT_SOURCE_DRDY_EN: u8 = 1 << 0;

/// `INT_CONFIG` bit 0: latched (1) or pulsed (0).
pub const INT_CONFIG_LATCHED: u8 = 1 << 0;
/// `INT_CONFIG` bit 1: active high.
pub const INT_CONFIG_ACTIVE_HIGH: u8 = 1 << 1;
/// `INT_CONFIG` bit 2: open drain (1) or push-pull (0).
pub const INT_CONFIG_OPEN_DRAIN: u8 = 1 << 2;
/// `INT_CONFIG` bit 3: interrupt enable.
pub const INT_CONFIG_EN: u8 = 1 << 3;
/// `INT_CONFIG` bits 7:4 offset: pad drive strength.
pub const INT_CONFIG_PAD_DRV_SHIFT: u8 = 4;

/// `OSR_CONFIG` bit 6: pressure measurement enable.
pub const OSR_CONFIG_PRESS_EN: u8 = 1 << 6;
/// `OSR_CONFIG` bits 5:3 offset: pressure oversampling.
pub const OSR_CONFIG_OSR_P_SHIFT: u8 = 3;

/// `ODR_CONFIG` bits 6:2 offset: output data rate code.
pub const ODR_CONFIG_ODR_SHIFT: u8 = 2;
/// `ODR_CONFIG` bit 7: deep standby disable.
pub const ODR_CONFIG_DEEP_DIS: u8 = 1 << 7;

/// `OSR_EFF` bit 7: the ODR and oversampling combination is valid.
pub const OSR_EFF_ODR_IS_VALID: u8 = 1 << 7;

/// `DSP_CONFIG` reset value: pressure and temperature compensation on.
pub const DSP_CONFIG_RESET: u8 = 0x03;
/// `DSP_CONFIG` bit 3: data registers hold the temperature after the IIR filter.
pub const DSP_CONFIG_SHDW_SEL_IIR_T: u8 = 1 << 3;
/// `DSP_CONFIG` bit 5: data registers hold the pressure after the IIR filter.
pub const DSP_CONFIG_SHDW_SEL_IIR_P: u8 = 1 << 5;

/// `DSP_IIR` bits 5:3 offset: pressure IIR coefficient.
pub const DSP_IIR_P_SHIFT: u8 = 3;
