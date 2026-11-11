//! Pure helpers: tables of the datasheet as data and the numbers derived from them.

use crate::config::{FifoData, OutputDataRate as O, Oversampling};

/// Fastest output data rate in normal mode with pressure and temperature measured, by `osr_p` (row)
/// and `osr_t` (column), both from 1x to 128x. Table 7, p17.
///
/// The datasheet lists "max nominal ODR" values (p17), and each printed value is read as a nominal one:
/// the rate code nearest to it is the fastest allowed. Printed 220, 200, 180, 130, 100, 50 and 45 Hz
/// are the codes 218.537, 199.111, 179.2, 129.855, 100.299, 50.056 and 45.025 Hz (7.34, p65); the other
/// printed values are codes exactly. A code slightly above its printed value (100.299, 50.056 and
/// 45.025 Hz) is therefore allowed. Evidence that the chip accepts every such cell: measured on the
/// board, 2026-11-10 (V2, `hil/src/bin/bmp581_valid.rs`), the chip accepted 100.299, 50.056 and 45.025 Hz
/// where the table prints 100, 50 and 45, and is never stricter than this reading. The chip's own measured
/// table is not used as the limit: it is one chip on one board.
pub const MAX_ODR_NORMAL: [[O; 8]; 8] = [
    // osr_p 1x: 240 240 240 240 200 130 80 40
    [
        O::Hz240,
        O::Hz240,
        O::Hz240,
        O::Hz240,
        O::Hz199_111,
        O::Hz129_855,
        O::Hz80,
        O::Hz40,
    ],
    // osr_p 2x: 240 240 240 220 180 120 70 40
    [
        O::Hz240,
        O::Hz240,
        O::Hz240,
        O::Hz218_537,
        O::Hz179_2,
        O::Hz120,
        O::Hz70,
        O::Hz40,
    ],
    // osr_p 4x: 220 220 200 180 140 100 70 40
    [
        O::Hz218_537,
        O::Hz218_537,
        O::Hz199_111,
        O::Hz179_2,
        O::Hz140,
        O::Hz100_299,
        O::Hz70,
        O::Hz40,
    ],
    // osr_p 8x: 140 140 130 120 100 80 50 35
    [
        O::Hz140,
        O::Hz140,
        O::Hz129_855,
        O::Hz120,
        O::Hz100_299,
        O::Hz80,
        O::Hz50_056,
        O::Hz35,
    ],
    // osr_p 16x: 80 80 80 70 70 50 45 30
    [
        O::Hz80,
        O::Hz80,
        O::Hz80,
        O::Hz70,
        O::Hz70,
        O::Hz50_056,
        O::Hz45_025,
        O::Hz30,
    ],
    // osr_p 32x: 45 45 40 40 40 35 30 20
    [
        O::Hz45_025,
        O::Hz45_025,
        O::Hz40,
        O::Hz40,
        O::Hz40,
        O::Hz35,
        O::Hz30,
        O::Hz20,
    ],
    // osr_p 64x: 20 20 20 20 20 20 15 15
    [
        O::Hz20,
        O::Hz20,
        O::Hz20,
        O::Hz20,
        O::Hz20,
        O::Hz20,
        O::Hz15,
        O::Hz15,
    ],
    // osr_p 128x: 10 10 10 10 10 10 10 5
    [
        O::Hz10,
        O::Hz10,
        O::Hz10,
        O::Hz10,
        O::Hz10,
        O::Hz10,
        O::Hz10,
        O::Hz5,
    ],
];

/// Fastest output data rate in normal mode with temperature only, by `osr_t`. Table 8, p17, read
/// nominally like [`MAX_ODR_NORMAL`]: printed 200 and 130 Hz are the codes 199.111 and 129.855 Hz.
pub const MAX_ODR_NORMAL_TEMPERATURE_ONLY: [O; 8] = [
    O::Hz240,
    O::Hz240,
    O::Hz240,
    O::Hz240,
    O::Hz199_111,
    O::Hz129_855,
    O::Hz80,
    O::Hz40,
];

/// Fastest rate allowed in normal mode (Tables 7 and 8, p17). `pressure` is whether pressure is measured.
pub fn max_odr(pressure: bool, osr_p: Oversampling, osr_t: Oversampling) -> O {
    let t = usize::from(osr_t.code());
    let found = if pressure {
        MAX_ODR_NORMAL
            .get(usize::from(osr_p.code()))
            .and_then(|row| row.get(t))
    } else {
        MAX_ODR_NORMAL_TEMPERATURE_ONLY.get(t)
    };
    // Both codes are 3 bits, so the lookup cannot miss; 240 Hz would be the unchecked value.
    found.copied().unwrap_or(O::Hz240)
}

/// Typical conversion time of pressure in microseconds (`tconv_p`, Table 4, p12; typical, 5 % either way).
pub const fn pressure_conversion_time_us(osr: Oversampling) -> u32 {
    match osr {
        Oversampling::X1 => 1_000,
        Oversampling::X2 => 1_700,
        Oversampling::X4 => 2_900,
        Oversampling::X8 => 5_400,
        Oversampling::X16 => 10_400,
        Oversampling::X32 => 20_400,
        Oversampling::X64 => 40_400,
        Oversampling::X128 => 80_400,
    }
}

/// Typical conversion time of temperature in microseconds (`tconv_t`, Table 4, p12; typical, 5 % either way).
pub const fn temperature_conversion_time_us(osr: Oversampling) -> u32 {
    match osr {
        Oversampling::X1 => 1_000,
        Oversampling::X2 => 1_100,
        Oversampling::X4 => 1_500,
        Oversampling::X8 => 2_100,
        Oversampling::X16 => 3_300,
        Oversampling::X32 => 5_800,
        Oversampling::X64 => 10_800,
        Oversampling::X128 => 20_800,
    }
}

/// Typical time of one measurement of pressure and temperature: the sum of the two conversion times
/// (Table 4, p12; the scope's 11.5 ms for 16x and 2x).
pub const fn conversion_time_us(osr_p: Oversampling, osr_t: Oversampling) -> u32 {
    pressure_conversion_time_us(osr_p) + temperature_conversion_time_us(osr_t)
}

impl O {
    /// Nominal rate in millihertz, as listed in 7.34 (p65).
    pub const fn millihertz(self) -> u32 {
        match self {
            Self::Hz240 => 240_000,
            Self::Hz218_537 => 218_537,
            Self::Hz199_111 => 199_111,
            Self::Hz179_2 => 179_200,
            Self::Hz160 => 160_000,
            Self::Hz149_333 => 149_333,
            Self::Hz140 => 140_000,
            Self::Hz129_855 => 129_855,
            Self::Hz120 => 120_000,
            Self::Hz110_164 => 110_164,
            Self::Hz100_299 => 100_299,
            Self::Hz89_6 => 89_600,
            Self::Hz80 => 80_000,
            Self::Hz70 => 70_000,
            Self::Hz60 => 60_000,
            Self::Hz50_056 => 50_056,
            Self::Hz45_025 => 45_025,
            Self::Hz40 => 40_000,
            Self::Hz35 => 35_000,
            Self::Hz30 => 30_000,
            Self::Hz25_005 => 25_005,
            Self::Hz20 => 20_000,
            Self::Hz15 => 15_000,
            Self::Hz10 => 10_000,
            Self::Hz5 => 5_000,
            Self::Hz4 => 4_000,
            Self::Hz3 => 3_000,
            Self::Hz2 => 2_000,
            Self::Hz1 => 1_000,
            Self::Hz0_5 => 500,
            Self::Hz0_25 => 250,
            Self::Hz0_125 => 125,
        }
    }

    /// Nominal period in microseconds, rounded to the nearest: 1e9 over the rate in millihertz.
    pub const fn period_us(self) -> u32 {
        let mhz = self.millihertz();
        (1_000_000_000 + mhz / 2) / mhz
    }
}

impl FifoData {
    /// Frames the FIFO holds: 16 for pressure and temperature, 32 for one quantity (4.6, p22).
    pub const fn capacity_frames(self) -> u8 {
        match self {
            Self::PressureAndTemperature => 16,
            Self::Temperature | Self::Pressure => 32,
        }
    }

    /// Largest FIFO threshold: 15 for pressure and temperature frames, 31 for the others (4.6.1, p23).
    pub const fn max_threshold(self) -> u8 {
        match self {
            Self::PressureAndTemperature => 15,
            Self::Temperature | Self::Pressure => 31,
        }
    }

    /// Bytes per frame: 6 for pressure and temperature (48 bit), 3 for one quantity (24 bit)
    /// (4.6.3, p23; Tables 11 to 13, p23-24).
    pub const fn frame_bytes(self) -> usize {
        match self {
            Self::PressureAndTemperature => 6,
            Self::Temperature | Self::Pressure => 3,
        }
    }
}
