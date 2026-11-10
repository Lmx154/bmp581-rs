# BMP581 device facts (MARV V2 Rev 1)

Copied from the MARV V2 firmware repository's `docs/devices.md` and `docs/pinout.md`. Page numbers are PDF pages of the datasheet listed at the end. Items marked UNKNOWN or "unverified" have not been checked against the datasheet or on hardware. "Measured" means measured on this board.

## BMP581 (Bosch Sensortec, barometer on I2C1)

Pins: see "Pins and bus" below (7-bit address 0x46).

- CHIP_ID 0x01 = 0x50; REV_ID 0x02 = 0x32 (p51).
- STATUS 0x28: bit1 nvm_rdy (expect 1), bit2 nvm_err (expect 0) (p58, p18).
- INT_STATUS 0x27, clear-on-read: bit4 por, bit0 drdy. 0x10 right after power-up or soft reset (p58).
- OSR_CONFIG 0x36: bit6 press_en, bits 5:3 osr_p, bits 2:0 osr_t (p64).
- ODR_CONFIG 0x37: bit7 deep_dis, bits 6:2 odr (0x0F = 50 Hz, 0x11 = 40 Hz, 0x15 = 20 Hz, 0x17 = 10 Hz,
  0x1C = 1 Hz), bits 1:0 pwr_mode (00 standby, 01 normal). Write configuration in standby first, power mode
  last (p65, p18).
- Data, little-endian 24-bit signed: temperature 0x1D-0x1F, degC = raw/65536; pressure 0x20-0x22,
  Pa = raw/64 (p56-57).
- INT_CONFIG 0x14: bit0 mode (0 pulsed), bit1 polarity (1 active high), bit2 open-drain (0 push-pull), bit3
  enable, bits 7:4 pad drive; reset 0x35. INT_SOURCE 0x15 bit0 = data-ready enable (p52-54).
  Pulse length 90-121 us (p12).
- Soft reset: write 0xB6 to CMD 0x7E; ready after 2 ms; the write may not be acknowledged (p67, p11).
- Measured: after a reset the pending por flag holds INT high and no data-ready edges appear (pin high on
  2000 of 2000 samples) until INT_STATUS has been read once. Read INT_STATUS before enabling the interrupt.
- ODR code 0x00 = 240 Hz. pwr_mode 10 = forced (one measurement, then standby), 11 = continuous.
  OSR_EFF bit7 odr_is_valid reports whether the chip accepts the ODR for the chosen oversampling; it
  updates in standby. Measured: 240 Hz with 128x/128x oversampling is rejected.
- IIR filter: coefficients in DSP_IIR 0x31 (pressure in bits 5:3). The data registers hold unfiltered values
  unless DSP_CONFIG 0x30 bit3 (shdw_sel_iir_t) and bit5 (shdw_sel_iir_p) are set; DSP_CONFIG resets to 0x03.
  Not exercised on the board.
- Measured at 400 kHz I2C: one 6-byte read takes about 242 us. Normal mode at the 240 Hz setting gives
  238.4 Hz; continuous mode with 1x/1x oversampling gives 496 Hz; forced mode with 16x pressure and 2x
  temperature oversampling takes 11.9 ms from trigger to data-ready.

## Pins and bus (from pinout.md)

All GPIO are 3.3 V logic and are not 5 V tolerant.

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 21 | BARO_INT | GPIO input | BMP581 INT |
| 22 | BARO_SDA | I2C1 SDA (F3) | BMP581; 4.7k pull-up R57 |
| 23 | BARO_SCL | I2C1 SCL (F3) | BMP581; 4.7k pull-up R50 |

| Peripheral | GPIOs | Device |
|---|---|---|
| I2C1 | SDA 22, SCL 23 | BMP581 only, address 0x46 |

| Item | Value |
|---|---|
| BMP581 I2C | 4.7k pull-ups R57 (SDA), R50 (SCL) to V3V3_ANA; CSB tied to VDDIO (V3V3_ANA, I2C mode), SDO/ADDR to GND: address 0x46 |

Rails: V3V3_ANA (sensors, their pull-ups, X1, ADC_AVDD) is enabled from V3V3_SYS, so it comes up after V3V3_SYS. V3V3_SYS powers the microSD, its pull-ups, the PSRAM socket and the IO block power pins.

## Datasheet

| Device | URL |
|---|---|
| BMP581 | https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf |
