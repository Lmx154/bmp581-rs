//! Register map, names as in the datasheet (BST-BMP581-DS004-13, section 7).
//!
//! Page numbers are the printed ones ("n | 75" in each page header). In the text extraction of the
//! datasheet a form feed starts each page, so printed page n is the n-th form-feed-separated chunk.
//! Register descriptions are in section 7 (p51 to p67). "Reset" is the reset value of the register
//! (section 7, the page of the register). A reserved field keeps its reset value when written.

/// Chip identity, reads [`CHIP_ID_VALUE`] (7.1, p51).
pub const CHIP_ID: u8 = 0x01;
/// ASIC revision, reset value [`REV_ID_VALUE`] (7.2, p51).
pub const REV_ID: u8 = 0x02;
/// Host interface status, reset 0x00 (7.3, p51): `hif_mode` bits 1:0, `i3c_err_0` bit 2,
/// `i3c_err_3` bit 3, bits 7:4 reserved. Fact only, this crate does not write it.
pub const CHIP_STATUS: u8 = 0x11;
/// Host interface drive and CSB pull-up, reset [`DRIVE_CONFIG_RESET`] (7.4, p52). NVM-backed.
/// Fact only, this crate does not write it.
pub const DRIVE_CONFIG: u8 = 0x13;
/// INT pin configuration, reset value [`INT_CONFIG_RESET`] (7.5, p52-53).
pub const INT_CONFIG: u8 = 0x14;
/// INT source selection, reset 0x00 (7.6, p54). Bits 7:4 reserved.
pub const INT_SOURCE: u8 = 0x15;
/// FIFO threshold and mode, reset 0x00 (7.7, p54). Bits 7:6 reserved.
pub const FIFO_CONFIG: u8 = 0x16;
/// FIFO fill level, reset 0x00 (7.8, p55). `fifo_count` bits 5:0, bits 7:6 reserved.
pub const FIFO_COUNT: u8 = 0x17;
/// FIFO frame and decimation selection, reset 0x00 (7.9, p55). Bits 7:5 reserved.
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
/// Interrupt status, cleared by reading, reset 0x00 (7.21, p58). Bits 7:5 reserved.
pub const INT_STATUS: u8 = 0x27;
/// NVM and core status, reset [`STATUS_RESET`] (7.22, p58).
pub const STATUS: u8 = 0x28;
/// FIFO output port, reset [`FIFO_EMPTY_FRAME`] (7.23, p59; empty frame Table 14, p24).
pub const FIFO_DATA: u8 = 0x29;
/// NVM row address, reset 0x00 (7.24, p60). Fact only.
pub const NVM_ADDR: u8 = 0x2B;
/// NVM data, low byte, reset 0x00 (7.25, p60). Fact only.
pub const NVM_DATA_LSB: u8 = 0x2C;
/// NVM data, high byte, reset 0x00 (7.26, p60). Fact only.
pub const NVM_DATA_MSB: u8 = 0x2D;
/// Data path configuration (compensation, IIR source selection), reset [`DSP_CONFIG_RESET`]
/// (7.27, p61).
pub const DSP_CONFIG: u8 = 0x30;
/// IIR filter coefficients, reset 0x00 (7.28, p62). Bits 7:6 reserved.
pub const DSP_IIR: u8 = 0x31;
/// Out-of-range pressure threshold, bits 7:0, reset 0x00 (7.29, p63).
pub const OOR_THR_P_LSB: u8 = 0x32;
/// Out-of-range pressure threshold, bits 15:8, reset 0x00 (7.30, p63).
pub const OOR_THR_P_MSB: u8 = 0x33;
/// Out-of-range window in pascals, reset 0x00 (7.31, p63).
pub const OOR_RANGE: u8 = 0x34;
/// Out-of-range threshold bit 16 and count limit, reset 0x00 (7.32, p63).
pub const OOR_CONFIG: u8 = 0x35;
/// Oversampling configuration, reset [`OSR_CONFIG_RESET`] (7.33, p64).
pub const OSR_CONFIG: u8 = 0x36;
/// Output data rate and power mode, reset [`ODR_CONFIG_RESET`] (7.34, p64-65).
pub const ODR_CONFIG: u8 = 0x37;
/// Effective oversampling and ODR validity, reset 0x00 (7.35, p66).
pub const OSR_EFF: u8 = 0x38;
/// Command register, reads 0x00 (7.36, p67); write [`CMD_SOFT_RESET`] to reset.
pub const CMD: u8 = 0x7E;

/// Value of `CHIP_ID`: reset 0b0101_0000 (7.1, p51).
pub const CHIP_ID_VALUE: u8 = 0x50;
/// `CMD` value for a soft reset (7.36, p67; 4.3.10, p18). Over I2C the chip does not acknowledge it.
pub const CMD_SOFT_RESET: u8 = 0xB6;

/// `STATUS` bit 1: ready for NVM operations (7.22, p58).
pub const STATUS_NVM_RDY: u8 = 1 << 1;
/// `STATUS` bit 2: NVM error (7.22, p58).
pub const STATUS_NVM_ERR: u8 = 1 << 2;

/// `INT_STATUS` bit 0: data ready (7.21, p58).
pub const INT_STATUS_DRDY: u8 = 1 << 0;
/// `INT_STATUS` bit 1: FIFO full (7.21, p58).
pub const INT_STATUS_FIFO_FULL: u8 = 1 << 1;
/// `INT_STATUS` bit 2: FIFO threshold (7.21, p58).
pub const INT_STATUS_FIFO_THS: u8 = 1 << 2;
/// `INT_STATUS` bit 3: pressure out of range (7.21, p58).
pub const INT_STATUS_OOR_P: u8 = 1 << 3;
/// `INT_STATUS` bit 4: power-on or soft reset complete (7.21, p58).
pub const INT_STATUS_POR: u8 = 1 << 4;

/// `INT_SOURCE` bit 0: data-ready enable (7.6, p54).
pub const INT_SOURCE_DRDY_EN: u8 = 1 << 0;

/// `INT_CONFIG` bit 0: latched (1) or pulsed (0) (7.5, p52-53).
pub const INT_CONFIG_LATCHED: u8 = 1 << 0;
/// `INT_CONFIG` bit 1: active high (7.5, p52-53).
pub const INT_CONFIG_ACTIVE_HIGH: u8 = 1 << 1;
/// `INT_CONFIG` bit 2: open drain (1) or push-pull (0) (7.5, p52-53).
pub const INT_CONFIG_OPEN_DRAIN: u8 = 1 << 2;
/// `INT_CONFIG` bit 3: interrupt enable (7.5, p52-53).
pub const INT_CONFIG_EN: u8 = 1 << 3;
/// `INT_CONFIG` bits 7:4 offset: pad drive strength (7.5, p52-53).
pub const INT_CONFIG_PAD_DRV_SHIFT: u8 = 4;

/// `OSR_CONFIG` bit 6: pressure measurement enable (7.33, p64).
pub const OSR_CONFIG_PRESS_EN: u8 = 1 << 6;
/// `OSR_CONFIG` bits 5:3 offset: pressure oversampling (7.33, p64).
pub const OSR_CONFIG_OSR_P_SHIFT: u8 = 3;

/// `ODR_CONFIG` bits 6:2 offset: output data rate code (7.34, p64-65).
pub const ODR_CONFIG_ODR_SHIFT: u8 = 2;
/// `ODR_CONFIG` bit 7: deep standby disable (7.34, p65).
pub const ODR_CONFIG_DEEP_DIS: u8 = 1 << 7;

/// `OSR_EFF` bit 7: the ODR and oversampling combination is valid (7.35, p66).
pub const OSR_EFF_ODR_IS_VALID: u8 = 1 << 7;

/// `DSP_CONFIG` reset value: pressure and temperature compensation on (7.27, p61).
pub const DSP_CONFIG_RESET: u8 = 0x03;
/// `DSP_CONFIG` bit 3: data registers hold the temperature after the IIR filter (7.27, p61).
pub const DSP_CONFIG_SHDW_SEL_IIR_T: u8 = 1 << 3;
/// `DSP_CONFIG` bit 5: data registers hold the pressure after the IIR filter (7.27, p61).
pub const DSP_CONFIG_SHDW_SEL_IIR_P: u8 = 1 << 5;

/// `DSP_IIR` bits 5:3 offset: pressure IIR coefficient (7.28, p62: the field table gives offset 0
/// for both fields, the Bit row puts `set_iir_p` left of `set_iir_t`, as the board confirms).
pub const DSP_IIR_P_SHIFT: u8 = 3;
/// `REV_ID` reset value 0b0011_0010 (7.2, p51).
pub const REV_ID_VALUE: u8 = 0x32;

/// `CHIP_STATUS` bits 1:0: host interface mode (7.3, p51).
pub const CHIP_STATUS_HIF_MODE_MASK: u8 = 0b11;
/// `CHIP_STATUS` bit 2: I3C SDR parity error (7.3, p51).
pub const CHIP_STATUS_I3C_ERR_0: u8 = 1 << 2;
/// `CHIP_STATUS` bit 3: I3C S0/S1 error, clear on read (7.3, p51).
pub const CHIP_STATUS_I3C_ERR_3: u8 = 1 << 3;

/// `DRIVE_CONFIG` reset value 0b0011_0000 (7.4, p52).
pub const DRIVE_CONFIG_RESET: u8 = 0x30;
/// `DRIVE_CONFIG` bit 0: CSB pull-up, I2C mode only (7.4, p52).
pub const DRIVE_CONFIG_I2C_CSB_PUP_EN: u8 = 1 << 0;
/// `DRIVE_CONFIG` bit 1: SPI 3-wire mode (7.4, p52).
pub const DRIVE_CONFIG_SPI3_EN: u8 = 1 << 1;
/// `DRIVE_CONFIG` bits 7:4 offset: serial pad drive strength (7.4, p52).
pub const DRIVE_CONFIG_PAD_IF_DRV_SHIFT: u8 = 4;

/// `INT_CONFIG` reset value 0b0011_0101: latched, active low, open drain, pin disabled, drive 3
/// (7.5, p52).
pub const INT_CONFIG_RESET: u8 = 0x35;
/// `INT_CONFIG` bits 7:4: pad drive strength for INT (7.5, p53).
pub const INT_CONFIG_PAD_DRV_MASK: u8 = 0xF0;

/// `INT_SOURCE` reset value (7.6, p54).
pub const INT_SOURCE_RESET: u8 = 0x00;
/// `INT_SOURCE` bit 1: FIFO full enable (7.6, p54).
pub const INT_SOURCE_FIFO_FULL_EN: u8 = 1 << 1;
/// `INT_SOURCE` bit 2: FIFO threshold enable (7.6, p54).
pub const INT_SOURCE_FIFO_THS_EN: u8 = 1 << 2;
/// `INT_SOURCE` bit 3: pressure out-of-range enable (7.6, p54).
pub const INT_SOURCE_OOR_P_EN: u8 = 1 << 3;
/// `INT_SOURCE` bits 7:4 are reserved (7.6, p54).
pub const INT_SOURCE_RESERVED: u8 = 0xF0;

/// `FIFO_CONFIG` reset value (7.7, p54).
pub const FIFO_CONFIG_RESET: u8 = 0x00;
/// `FIFO_CONFIG` bits 4:0: threshold in frames, 0 disables it (7.7, p54; 4.6.1, p23).
pub const FIFO_CONFIG_THRESHOLD_MASK: u8 = 0x1F;
/// `FIFO_CONFIG` bit 5: stop-on-full (1) or stream (0) (7.7, p54).
pub const FIFO_CONFIG_STOP_ON_FULL: u8 = 1 << 5;
/// `FIFO_CONFIG` bits 7:6 are reserved (7.7, p54).
pub const FIFO_CONFIG_RESERVED: u8 = 0xC0;

/// `FIFO_COUNT` bits 5:0: frames in the FIFO; bits 7:6 reserved (7.8, p55).
pub const FIFO_COUNT_MASK: u8 = 0x3F;

/// `FIFO_SEL` reset value (7.9, p55).
pub const FIFO_SEL_RESET: u8 = 0x00;
/// `FIFO_SEL` bits 1:0: frame selection. 0 off, 1 temperature, 2 pressure, 3 both (7.9, p55).
pub const FIFO_SEL_FRAME_MASK: u8 = 0b11;
/// `FIFO_SEL` bits 4:2 offset: decimation, one frame in 2^n (7.9, p55; 4.6.1, p23).
pub const FIFO_SEL_DEC_SHIFT: u8 = 2;
/// `FIFO_SEL` bits 7:5 are reserved (7.9, p55).
pub const FIFO_SEL_RESERVED: u8 = 0xE0;

/// Reset value of the data registers `TEMP_DATA_*` and `PRESS_DATA_*` (7.11 to 7.16, p56-57).
pub const DATA_RESET: u8 = 0x7F;
/// The FIFO empty frame, also the reset value of `FIFO_DATA` (Table 14, p24; 7.23, p59).
pub const FIFO_EMPTY_FRAME: u8 = 0x7F;

/// `INT_STATUS` reset value (7.21, p58).
pub const INT_STATUS_RESET: u8 = 0x00;
/// `INT_STATUS` bits 7:5 are reserved (7.21, p58).
pub const INT_STATUS_RESERVED: u8 = 0xE0;

/// `STATUS` reset value: `status_nvm_rdy` set (7.22, p58).
pub const STATUS_RESET: u8 = 0x02;
/// `STATUS` bit 0: the digital core is accessible (7.22, p58).
pub const STATUS_CORE_RDY: u8 = 1 << 0;
/// `STATUS` bit 3: NVM command error (7.22, p58).
pub const STATUS_NVM_CMD_ERR: u8 = 1 << 3;
/// `STATUS` bit 4: an error was corrected by ECC at the last boot-load (7.22, p58).
pub const STATUS_BOOT_ERR_CORRECTED: u8 = 1 << 4;
/// `STATUS` bit 7: crack check passed (7.22, p58).
pub const STATUS_ST_CRACK_PASS: u8 = 1 << 7;

/// `NVM_ADDR` bits 5:0: NVM row address (7.24, p60). Fact only.
pub const NVM_ADDR_ROW_MASK: u8 = 0x3F;
/// `NVM_ADDR` bit 6: NVM programming enable (7.24, p60). Fact only.
pub const NVM_ADDR_PROG_EN: u8 = 1 << 6;
/// First command of the NVM user read and program sequences (7.36, p67; 4.8.1.1, p29). Fact only.
pub const CMD_NVM_FIRST: u8 = 0x5D;
/// Second command of the NVM user read sequence `0x5D, 0xA5` (7.36, p67). Fact only.
pub const CMD_NVM_USR_READ: u8 = 0xA5;
/// Second command of the NVM user program sequence `0x5D, 0xA0` (7.36, p67). Fact only.
pub const CMD_NVM_USR_PROG: u8 = 0xA0;

/// `DSP_CONFIG` bits 1:0: compensation, 3 is pressure and temperature compensated (7.27, p61).
pub const DSP_CONFIG_COMP_PT_EN_MASK: u8 = 0b11;
/// `DSP_CONFIG` bit 2: IIR flush in forced mode (7.27, p61).
pub const DSP_CONFIG_IIR_FLUSH_FORCED_EN: u8 = 1 << 2;
/// `DSP_CONFIG` bit 4: the FIFO holds the temperature after the IIR filter (7.27, p61).
pub const DSP_CONFIG_FIFO_SEL_IIR_T: u8 = 1 << 4;
/// `DSP_CONFIG` bit 6: the FIFO holds the pressure after the IIR filter (7.27, p61).
pub const DSP_CONFIG_FIFO_SEL_IIR_P: u8 = 1 << 6;
/// `DSP_CONFIG` bit 7: the out-of-range check uses the pressure after the IIR filter (7.27, p61).
pub const DSP_CONFIG_OOR_SEL_IIR_P: u8 = 1 << 7;

/// `DSP_IIR` reset value (7.28, p62).
pub const DSP_IIR_RESET: u8 = 0x00;
/// `DSP_IIR` bits 2:0: temperature IIR coefficient (7.28, p62).
pub const DSP_IIR_T_MASK: u8 = 0b111;
/// `DSP_IIR` bits 5:3: pressure IIR coefficient (7.28, p62).
pub const DSP_IIR_P_MASK: u8 = 0b111 << DSP_IIR_P_SHIFT;
/// `DSP_IIR` bits 7:6 are reserved, "write 0x0" (7.28, p62).
pub const DSP_IIR_RESERVED: u8 = 0xC0;

/// `OOR_CONFIG` bit 0: bit 16 of the out-of-range reference pressure (7.32, p63).
pub const OOR_CONFIG_THR_P_16: u8 = 1 << 0;
/// `OOR_CONFIG` bits 5:1 are reserved (7.32, p63).
pub const OOR_CONFIG_RESERVED: u8 = 0x3E;
/// `OOR_CONFIG` bits 7:6 offset: count limit, 1, 3, 7 or 15 samples (7.32, p63).
pub const OOR_CONFIG_CNT_LIM_SHIFT: u8 = 6;

/// `OSR_CONFIG` reset value (7.33, p64).
pub const OSR_CONFIG_RESET: u8 = 0x00;
/// `OSR_CONFIG` bits 2:0: temperature oversampling (7.33, p64).
pub const OSR_CONFIG_OSR_T_MASK: u8 = 0b111;
/// `OSR_CONFIG` bit 7 is reserved (7.33, p64).
pub const OSR_CONFIG_RESERVED: u8 = 0x80;

/// `ODR_CONFIG` reset value 0b0111_0000: 1 Hz, standby, deep standby allowed (7.34, p64).
pub const ODR_CONFIG_RESET: u8 = 0x70;
/// `ODR_CONFIG` bits 1:0: power mode, reads back the actual mode (7.34, p64-65).
pub const ODR_CONFIG_PWR_MODE_MASK: u8 = 0b11;

/// `OSR_EFF` bits 2:0: effective temperature oversampling (7.35, p66).
pub const OSR_EFF_OSR_T_MASK: u8 = 0b111;
/// `OSR_EFF` bits 5:3 offset: effective pressure oversampling (7.35, p66).
pub const OSR_EFF_OSR_P_SHIFT: u8 = 3;
