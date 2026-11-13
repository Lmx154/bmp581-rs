# bmp581

Driver for the Bosch BMP581 barometric pressure sensor over I2C, `no_std`, async (`embedded-hal-async` 1.0) with a blocking front-end (`embedded-hal` 1.0). Those two crates are the only dependencies; `defmt` 1.0 is optional. There is no executor, HAL, pin or timer in the crate. It is written for the MARV V2 Rev 1 flight controller (RP2354B).

Status: version 0.1.0, `publish = false`, and the API may still change. Parts of it have run on the real sensor and parts have not; "Not covered" below says which.

## Works with

The driver works with any HAL and any MCU whose I2C bus and delay implement `embedded-hal-async` 1.0 (async driver) or `embedded-hal` 1.0 (`blocking` module). No embassy crate is needed.

## Quick start

```rust,no_run
# use bmp581::{Bmp581, Config, Error, OutputDataRate, Oversampling, DEFAULT_ADDRESS};
# #[derive(Debug)]
# struct BusError;
# impl embedded_hal::i2c::Error for BusError {
#     fn kind(&self) -> embedded_hal::i2c::ErrorKind { embedded_hal::i2c::ErrorKind::Other }
# }
# struct Bus;
# impl embedded_hal::i2c::ErrorType for Bus { type Error = BusError; }
# impl embedded_hal_async::i2c::I2c for Bus {
#     async fn transaction(&mut self, _address: u8, _ops: &mut [embedded_hal::i2c::Operation<'_>]) -> Result<(), BusError> { Ok(()) }
# }
# struct Delay;
# impl embedded_hal_async::delay::DelayNs for Delay { async fn delay_ns(&mut self, _ns: u32) {} }
# async fn run(i2c: Bus, mut delay: Delay) -> Result<(), Error<BusError>> {
let mut baro = Bmp581::new(i2c, DEFAULT_ADDRESS);
let config = Config::new()
    .with_pressure_oversampling(Oversampling::X16)
    .with_temperature_oversampling(Oversampling::X2)
    .with_odr(OutputDataRate::Hz50_056);
let _info = baro.init(&mut delay, &config).await?;

// Wait for data-ready here. The application owns the interrupt pin, and a timeout
// belongs around this wait, never around a driver call.

match baro.read().await {
    Ok(sample) => {
        let (pascal, deg_c) = (sample.pascals(), sample.celsius());
        let _ = (pascal, deg_c);
    }
    // No conversion has finished yet: the data registers still hold their reset value.
    Err(Error::NoSample) => { /* wait for the next data-ready and read again */ }
    Err(e) => return Err(e),
}
# Ok(())
# }
```

`Config` and the structs inside it are `#[non_exhaustive]`: build them with `Config::new()` and the `with_...` setters (`Config::default()` is `Config::new()`). `Config::new()` is pressure 16x, temperature 2x, 50.056 Hz, no filter, normal mode, data-ready interrupt, FIFO and out-of-range window off.

FIFO path: configure with `Config::with_fifo(FifoConfig::new()...)`, `fifo_read` into a caller buffer, decode with `fifo_frames`:

```rust,no_run
# use bmp581::{Bmp581, Config, Error, FifoConfig, FifoData, FifoFrame, FifoMode, fifo_frames};
# #[derive(Debug)]
# struct BusError;
# impl embedded_hal::i2c::Error for BusError {
#     fn kind(&self) -> embedded_hal::i2c::ErrorKind { embedded_hal::i2c::ErrorKind::Other }
# }
# struct Bus;
# impl embedded_hal::i2c::ErrorType for Bus { type Error = BusError; }
# impl embedded_hal_async::i2c::I2c for Bus {
#     async fn transaction(&mut self, _address: u8, _ops: &mut [embedded_hal::i2c::Operation<'_>]) -> Result<(), BusError> { Ok(()) }
# }
# struct Delay;
# impl embedded_hal_async::delay::DelayNs for Delay { async fn delay_ns(&mut self, _ns: u32) {} }
# async fn run(mut baro: Bmp581<Bus>, mut delay: Delay) -> Result<(), Error<BusError>> {
let fifo = FifoConfig::new().with_mode(FifoMode::Stream).with_data(FifoData::PressureAndTemperature).with_threshold(4);
baro.init(&mut delay, &Config::new().with_fifo(fifo)).await?;
let mut buf = [0u8; 12];
let read = baro.fifo_read(&mut buf).await?;
for frame in fifo_frames(&buf[..read.bytes], FifoData::PressureAndTemperature) {
    if let FifoFrame::PressureTemperature(s) = frame {
        let _ = (s.pascals(), s.celsius());
    }
}
# Ok(())
# }
```

Blocking: `bmp581::blocking::Bmp581::new(i2c, address)` has the same methods without `.await` (`init`, `configure`, `read`, `fifo_read`, `verify`, `recover`, ...) over `embedded_hal::i2c::I2c` and `embedded_hal::delay::DelayNs`. It runs the async code with adapters that complete on the first poll.

## What the application provides

- An I2C bus: `embedded_hal_async::i2c::I2c` (async) or `embedded_hal::i2c::I2c` (`blocking`). The 7-bit address is `DEFAULT_ADDRESS` (0x46, address pin low) or `ALTERNATE_ADDRESS` (0x47, high). The on-board programs run 400 kHz; `i2c_cancel` and `bmp581_cancel` also run 100 kHz and 1 MHz. Address 0x47 has not been run. In streaming FIFO mode the chip needs a host clock of 0.1 MHz or more (datasheet footnote, quoted in the `fifo_read` rustdoc).
- A delay: `DelayNs`, passed to `init`, `configure`, `fifo_flush` and `recover`.
- The interrupt pin. The driver configures the chip's INT output (`InterruptPin`: sources, pulsed or latched, polarity, push-pull or open drain) and owns no pin. The application waits for the edge, and `interrupt_status` reads the flags.

## Rules for correct use

1. Never cancel an I2C transfer in flight: no timeout or `select` around a driver call that is on the bus. Put the timeout around the wait for data-ready. Measured on embassy-rp 0.10.0 (about 13 000 cancelled reads at 100 kHz, 400 kHz and 1 MHz, BMP581 alone on I2C1; CONTRACT.md, "A4 for I2C, measured"): the HAL does nothing when an I2C future is dropped, the controller finishes the frame alone, and the NEXT reads return `Ok` with wrong data and no error (1345 of 3544 checking reads at 400 kHz without recovery). A cancel late in the frame kept following back-to-back reads wrong until the bus had been quiet for 2 ms. No register changed (0 of 325 dumps). The pass/fail program `bmp581_cancel` repeats this with `recover()` after each cancel (reads of read-only registers only): 759 cancels at 400 kHz, 428 at 100 kHz and 366 at 1 MHz, 0 wrong checking reads of 8000, `recover()` never failed, 40 of 40 register dumps equal to the baseline; without recovery it sees the fault (115 of 152 cancelled reads were followed by a wrong read).
2. `recover(delay)` is the way back after any doubt. It waits 2 ms with no bus traffic, makes one discarded read of `CHIP_ID`, reads `CHIP_ID` and `REV_ID` together (`Error::WrongId` if `CHIP_ID` is not 0x50 or `REV_ID` differs from what `init` read; the revision is compared only after `init`), then runs `verify()`. It writes nothing to the device. In the contract's measurement, one discarded one-byte `CHIP_ID` read after a cancel left 0 wrong of 9000 reads; `bmp581_cancel` calls the whole `recover()` and finds 0 wrong reads of 8000.
   `verify()` is the health check and changes nothing: it reads back the registers the applied `Config` owns (2 transactions, a read of 5 bytes and a read of 8 or 9 bytes), compares them under a mask, re-reads a differing one once, and only then returns `Error::ConfigMismatch` with register, mask, expected and found. The application decides what follows (`configure`, keep reading, switch sensors).
   A stuck SDA line cannot be cleared by a driver that only holds the bus object. The firmware's bus owner must pulse SCL while SDA is low, send START/STOP and rebuild the bus; `hil/src/bin/bmp581_cancel.rs` has such a routine (`bus_clear`). That program provoked a stuck SDA line, saw calls fail inside their 20 ms bound, ran the routine, and then read the sensor correctly (200 of 200 reads); the contract reports the same for `i2c_cancel`.
3. Methods that use the configuration return `Error::NotConfigured` until `init` or `configure` has completed, and again after one has failed, been dropped or returned `Error::ChipRejectsRate`. The raw register methods still work.
4. A `read` right after `init`, `configure` or `trigger_forced` returns `Error::NoSample` until the first conversion has finished: wait for data-ready (or `Info::conversion_time_us`). A forced-mode read made too early returns `NoSample` only while the registers still hold the reset value; after an earlier conversion it returns the older sample as valid.
5. On this HAL I2C is interrupt-driven, not DMA-driven. A read of at most 15 bytes is fully queued before the first await; a longer one is not. `fifo_read` into a large buffer can make such a read (the rustdoc gives 12 bytes for pressure and temperature frames and 15 for one quantity as the largest fully queued reads).
6. `interrupt_status` is clear-on-read: a flag read there is gone for anyone else. `verify` and `recover` never read `INT_STATUS`.

## What the driver does

- Settings: `Config` and its setters; `Oversampling`, `OutputDataRate` (the 32 rate codes), `PowerMode` (standby, normal, forced, continuous), pressure off (temperature only), `IirFilter`, `IirTaps`, `iir_flush_forced`, `InterruptPin`/`InterruptSources`, `FifoConfig`, `OutOfRange`. `init` resets and configures; `configure` changes a running device without a reset (the `Info` it returns has `chip_id` and `revision` 0: only `init` reads them); `trigger_forced` starts one forced measurement.
- Validity: `Config::validate` refuses combinations the datasheet forbids (`ConfigError`), including the rate against oversampling limit in normal mode (`calc::max_odr`, `calc::MAX_ODR_NORMAL`). In normal mode `init`/`configure` also read the chip's own flag in standby (`Error::ChipRejectsRate`); `effective_oversampling` reads what the chip uses. `Config::plan` and `plan::Plan` give the register steps behind it, without a bus.
- Typed read: `read` returns `RawSample` (`pascals()`, `celsius()`); `NoSample` as above. `calc::conversion_time_us` gives the typical conversion time.
- FIFO: `fifo_read`, `fifo_count`, `fifo_flush`, and the pure decoder `fifo_frames` yielding `FifoFrame::{PressureTemperature, Temperature, Pressure, Empty, Partial}`. Stream or stop-on-full, threshold and decimation in `FifoConfig`. On the real chip a streaming FIFO stops counting one frame below the datasheet depth (15 pressure and temperature frames, 31 of one quantity) with the FIFO-full flag set, so `FifoRead::full` there means "at that limit" (loss begun or one frame away); `interrupt_status` or the FIFO-full interrupt is the exact indicator (`docs/device.md`). `fifo_flush` discards the frames in the FIFO and re-applies the configuration with waits: in normal mode 20 transactions and 4 delays (the rustdoc's longest case); after an error in it the driver is not configured, so call `configure`. `Error::FifoOff`, `Error::BufferTooSmall`.
- `interrupt_status` (`IntStatus`: data ready, FIFO full, FIFO threshold, out of range, power-on reset).
- Health reads: `status` (`Status`), `effective_oversampling`, `stats()` (`Stats`: power-on resets, bus errors, FIFO-full events, frames read).
- `verify`, `recover`: see rule 2.
- Raw access: `read_register`, `read_registers`, `write_register`, `release`; registers are in `reg`. A register written this way is not tracked by the driver.
- Blocking front-end: `bmp581::blocking::Bmp581`.

## Not covered

Deliberately out of scope (each checked against `src/`): deep-standby control (`reg::ODR_CONFIG_DEEP_DIS` exists as a constant, no `Config` field uses it), the NVM user area (register constants only, no method), SPI and I3C, and the serial interface drive settings (`reg::DRIVE_CONFIG`, constants only).

Exists but has NOT run on the real sensor:

- The FIFO methods have run on the real sensor in `bmp581_fifo` (stream and stop-on-full, flush, threshold and FIFO-full interrupts, all three frame kinds, decimation, overflow). What that program does not cover: FIFO use at 100 kHz and 1 MHz, a threshold above 8, decimation other than 4, and any read or flush that fails or is cancelled mid-transfer.
- No on-board program waits for an out-of-range event. The out-of-range window (reference 100 000 Pa, range 255 Pa, count three) has been applied and read back only.
- `Config` options no `hil` program sets: `iir_taps`, `iir_flush_forced`, `InterruptPin::off`/`with_enabled(false)`, `OutOfRangeCount` One, Seven and Fifteen, `IirFilter` Coeff1, 15, 63 and 127, and `ALTERNATE_ADDRESS`. IIR coefficients 3, 7 and 31 were applied and read back; nothing checks the effect of a filter on the output.

## Measured on the MARV V2 Rev 1 board

RP2354B, embassy-rp 0.10, BMP581 on I2C1. These are observations, not guarantees.

- Uncancelled `write_read`, medians (2026-11-10): 1 byte 115 us and 6 bytes 237 us at 400 kHz; 55 and 106 us at 1 MHz.
- `verify()` while streaming at 240 Hz: 513 us per call (200 back-to-back calls, data-ready flag left intact). `recover()` in normal mode: 2.8 ms with its 2 ms wait. `configure` on a running device: 4.1 ms.
- Before the first conversion the data registers hold 0x7F7F7F, which decodes to 130 557.98 Pa and 127.5 degC, outside the sensor's ranges; this is why `read` returns `NoSample` for it.
- 496 Hz continuous with 1x/1x oversampling; normal mode at the 240 Hz setting gives 238.4 Hz; forced mode with 16x pressure and 2x temperature oversampling takes 11.9 ms from trigger to data-ready; 240 Hz with 128x/128x is rejected by the chip (`docs/device.md`).
- Validity: the chip accepted every rate and oversampling combination the driver's table allows (`bmp581_valid`); the datasheet's printed 100, 50 and 45 Hz limits are read as the codes 100.299, 50.056 and 45.025 Hz, which the chip accepted. In normal mode the validity flag read in standby never differed from the flag after entering normal mode (2048 + 512 combinations).
- `STATUS`: the NVM-ready bit reads 0 while the chip is measuring (0x01 running, 0x03 in standby); judge health by the core-ready bit while running.
- At 4, 1 and 0.125 Hz the power-mode field stays "normal" between measurements; `verify` gave no false alarm in 12 unlike configurations, with all eight interrupt-pin combinations, and with pressure off.

## Tests

Host: `cargo test --locked` (109 unit tests at the time of writing; the two `ignore`d examples in `src/lib.rs` and the examples here, which compile against a dummy bus, are doctests).

On the board, through a debug probe on the SWD header J10 (target `thumbv8m.main-none-eabihf`, `probe-rs` chip RP235x): `tools/run.sh <name>`. The script takes a lock so runs never overlap. The exit status is 0 when every check passed; the log is kept in `hil/logs/`. Wiring: I2C1 SDA on GPIO22, SCL on GPIO23 (4.7k pull-ups R57 and R50), INT on GPIO21, address 0x46.

| Program | What it proves |
|---|---|
| `alive` | tool chain, probe and board |
| `bmp581_smoke` | identity, data-ready rate and missed samples in normal and continuous mode, the driver refusing 240 Hz with 128x/128x (`Error::InvalidConfig`, before anything reaches the chip), plausible values, the blocking front-end |
| `bmp581_ops` | `verify` across 12 configurations, `configure` on a running device, `recover`, `status`, `interrupt_status`, `stats`, `verify` during streaming, three open points (power-mode read-back at low rates, pad-drive and `DSP_CONFIG` read-back, effective oversampling with pressure off) |
| `bmp581_valid` | the chip's validity flag against `Config::validate` for every rate and oversampling pair |
| `bmp581_fifo` | the FIFO on the real chip: streaming at 240 Hz and in continuous mode against the INT pin's edge count, overflow in both modes (the streaming count stops one frame below the datasheet depth, and `fifo_read` reports it full), flush, threshold and FIFO-full interrupts, pressure-only and temperature-only frames, decimation, the three datasheet points |
| `bmp581_cancel` | rule 1 and `recover()`: cancelled reads at 400 kHz, 100 kHz and 1 MHz, each followed by `recover()`, no wrong read afterwards; shows the fault without recovery; a stuck SDA line cleared by the bus-clear routine |
| `i2c_cancel` | the cancel measurement behind rule 1: an experiment that reports findings, not a pass/fail test |

## Features, layout, licence

Feature `defmt` (off by default): `defmt::Format` for the public data and error types.

```text
Cargo.toml       the driver crate (its own workspace root)
src/             the driver; tests are in *_tests.rs, testbus.rs and tests.rs
hil/             on-board programs: its own workspace, the only place that uses embassy-rp
tools/run.sh     builds, flashes and runs one on-board program
docs/device.md   register, timing, pin and datasheet facts
```

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md`.
