# bmp581

Driver for the Bosch BMP581 barometric pressure sensor over I2C, for the MARV V2 Rev 1 flight controller (RP2354B). It is built on the `embedded-hal` 1.0 traits only: no executor, no HAL, no pin, no timer. The crate root is the async driver. The `blocking` module is the same driver for blocking buses. It wraps the bus in an adapter that never pends and runs the same code, so nothing is written twice. All device knowledge (register map, configuration codes, decoding, units) is pure code tested on the host.

## Usage

Async, for example in an embassy task:

```rust
let mut baro = bmp581::Bmp581::new(i2c, bmp581::DEFAULT_ADDRESS);
baro.init(&mut delay, &bmp581::Config::default()).await?;
int_pin.wait_for_rising_edge().await;
let sample = baro.read().await?;
let (pa, deg_c) = (sample.pascals(), sample.celsius());
```

Blocking, on any embedded-hal bus:

```rust
let mut baro = bmp581::blocking::Bmp581::new(i2c, bmp581::DEFAULT_ADDRESS);
baro.init(&mut delay, &bmp581::Config::default())?;
let sample = baro.read()?;
```

Units: `RawSample::pascals` is pascals, `RawSample::celsius` is degrees Celsius.

## What the application provides

- An I2C bus: `embedded_hal_async::i2c::I2c` for the async driver, the blocking `embedded-hal` I2C trait for `blocking`.
- A `DelayNs` for `init`.
- The address: `DEFAULT_ADDRESS` (0x46, address pin low) or `ALTERNATE_ADDRESS` (0x47, address pin high).
- The interrupt pin and the waiting on it. The driver owns no interrupt pin, timer or task. `init` configures the chip's INT output; the application waits on the pin. Every sample signalled by data-ready after `init` returns is valid.

Dependencies: `embedded-hal` 1.0, `embedded-hal-async` 1.0 and, optionally, `defmt`.

## Cargo features

All features are off by default. Without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data and error types |

## Measured on the MARV V2 Rev 1 board

- 496 Hz continuous, with 1x/1x oversampling.
- 242 us per read at 400 kHz I2C (one 6-byte read).
- Normal mode at the 240 Hz setting gives 238.4 Hz.
- Forced mode with 16x pressure and 2x temperature oversampling takes 11.9 ms from trigger to data-ready.
- Measured: 240 Hz with 128x/128x oversampling is rejected by the chip.

## Tests

On the host:

    cargo test

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh alive
    tools/run.sh bmp581_smoke

Used here: Rust 1.99.1 with the `thumbv8m.main-none-eabihf` target, probe-rs 0.31.0, and a debug probe on the SWD header J10.

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `bmp581_smoke` | identity, data-ready rate with missed-sample count, time per read, plausible values at rest, the blocking front-end |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`.

Pins the test uses: I2C1 SDA on GPIO22 and SCL on GPIO23 (4.7k pull-ups R57 and R50), INT on GPIO21. The address is 0x46. The test runs async I2C1 at 400 kHz with data-ready on GPIO21.

## Status

Version 0.1.0, as tested on the Rev 1 board.

- Host: `cargo test` passes. Clippy, rustdoc and rustfmt are clean for the driver crate, and every public item is documented.
- Board: `alive` and `bmp581_smoke` exited 0 when last run.
- On-board test program: `cargo clippy` in `hil/` reports style findings in `bmp581_smoke`. They do not change any result.

Not done for this device:

- FIFO and batch reads.
- The IIR filter is not exercised on the board.

## Layout

```
Cargo.toml       the driver crate (its own workspace root)
src/             the driver
hil/             the on-board test: one package, its own workspace; the only place that uses embassy-rp
  src/lib.rs     reports the verdict to the host
  src/bin/       alive.rs and bmp581_smoke.rs
  build.rs       links memory.x
tools/run.sh     builds, flashes and runs one on-board test
docs/device.md   register, timing and measured facts, pins, datasheet
LICENSE          MIT
```

## License

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md`.
