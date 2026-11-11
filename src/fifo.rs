//! FIFO frames: pure, in-place decoding of a byte buffer read from `FIFO_DATA`.
//!
//! Frame layouts (4.6.3, p23-24; Tables 11 to 14). The FIFO holds one frame per measurement, in one of
//! three kinds chosen by `FIFO_SEL.fifo_frame_sel` (7.9, p55; 4.6.1, p22-23):
//!
//! - pressure and temperature (PT, Table 11, p23-24), 6 bytes: temperature XLSB, LSB, MSB, then pressure
//!   XLSB, LSB, MSB. The temperature comes first, each value is 24 bit little endian, signed two's
//!   complement, like the data registers (4.5, p22; 7.11 to 7.16, p56-57).
//! - temperature only (T, Table 12, p24), 3 bytes: temperature XLSB, LSB, MSB.
//! - pressure only (P, Table 13, p24), 3 bytes: pressure XLSB, LSB, MSB.
//! - the empty frame (Table 14, p24): the byte 0x7F. A read of an empty or disabled FIFO, or of one that
//!   turns empty during the read, returns it (4.6.3, p23); `FIFO_DATA` resets to it (7.23, p59).
//!
//! The datasheet defines the empty frame as one byte, and says reads should be made in the granularity
//! of the frame size (4.6.3, p23). A read of a whole number of frames past the last frame thus returns
//! 0x7F bytes. [`fifo_frames`] therefore calls a whole frame-sized chunk in which EVERY byte is 0x7F a
//! [`FifoFrame::Empty`]: it cannot be a measurement, because 0x7F7F7F lies outside the sensor's ranges
//! (130 557.98 Pa and 127.5 degC; Table 1, p8; Table 2, p10; measured on the board, 2026-11-10). A chunk of
//! which only some bytes are 0x7F is a measurement and is decoded as one (a pressure and temperature
//! frame with only one half at 0x7F7F7F is returned as such; [`RawSample::is_reset_value`] judges it).
//!
//! [`fifo_frames`] never skips a byte and never panics, whatever the input: every input byte belongs to
//! exactly one item, and a tail shorter than one frame comes out once as [`FifoFrame::Partial`].

use crate::config::FifoData;
use crate::reg;
use crate::sample::{RawSample, sign_extend_24};

/// A temperature-only frame (Table 12, p24).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TemperatureFrame {
    /// Signed 24-bit temperature, 1/65536 degC per LSB.
    pub temperature: i32,
}

impl TemperatureFrame {
    /// Temperature in degrees Celsius (the conversion of [`RawSample::celsius`]).
    pub fn celsius(&self) -> f32 {
        RawSample {
            temperature: self.temperature,
            pressure: 0,
        }
        .celsius()
    }
}

/// A pressure-only frame (Table 13, p24).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PressureFrame {
    /// Signed 24-bit pressure, 1/64 Pa per LSB.
    pub pressure: i32,
}

impl PressureFrame {
    /// Pressure in pascals (the conversion of [`RawSample::pascals`]).
    pub fn pascals(&self) -> f32 {
        RawSample {
            temperature: 0,
            pressure: self.pressure,
        }
        .pascals()
    }
}

/// One item of [`fifo_frames`]; the variants of the frame kinds carry the same integers as the data
/// registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum FifoFrame<'a> {
    /// A pressure and temperature frame, 6 bytes (Table 11, p23-24).
    PressureTemperature(RawSample),
    /// A temperature frame, 3 bytes (Table 12, p24).
    Temperature(TemperatureFrame),
    /// A pressure frame, 3 bytes (Table 13, p24).
    Pressure(PressureFrame),
    /// A whole frame-sized chunk of 0x7F bytes: the empty frame (Table 14, p24). Carries the bytes.
    Empty(&'a [u8]),
    /// Trailing bytes that do not make a whole frame, 1 to frame size minus 1 of them, carried as read.
    /// Seen when the buffer was cut by the caller; the chip keeps an incompletely read frame and sends it
    /// again (4.6.3, p23).
    Partial(&'a [u8]),
}

/// The iterator of [`fifo_frames`].
#[derive(Clone, Debug)]
pub struct FifoFrames<'a> {
    rest: &'a [u8],
    data: FifoData,
}

/// Walks `buf` frame by frame, in place, for frames of kind `data`. Pure; no bus access, no allocation.
///
/// Every byte of `buf` is accounted for by exactly one item: whole frames of `data.frame_bytes()` bytes
/// each, then at most one [`FifoFrame::Partial`] with the remaining bytes. No byte is skipped.
pub const fn fifo_frames(buf: &[u8], data: FifoData) -> FifoFrames<'_> {
    FifoFrames { rest: buf, data }
}

impl<'a> Iterator for FifoFrames<'a> {
    type Item = FifoFrame<'a>;

    fn next(&mut self) -> Option<FifoFrame<'a>> {
        if self.rest.is_empty() {
            return None;
        }
        let Some((chunk, tail)) = self.rest.split_at_checked(self.data.frame_bytes()) else {
            let partial = self.rest;
            self.rest = &[];
            return Some(FifoFrame::Partial(partial));
        };
        self.rest = tail;
        if chunk.iter().all(|&b| b == reg::FIFO_EMPTY_FRAME) {
            return Some(FifoFrame::Empty(chunk));
        }
        Some(match *chunk {
            [t0, t1, t2, p0, p1, p2] => FifoFrame::PressureTemperature(RawSample {
                temperature: sign_extend_24(t0, t1, t2),
                pressure: sign_extend_24(p0, p1, p2),
            }),
            [b0, b1, b2] => match self.data {
                FifoData::Pressure => FifoFrame::Pressure(PressureFrame {
                    pressure: sign_extend_24(b0, b1, b2),
                }),
                _ => FifoFrame::Temperature(TemperatureFrame {
                    temperature: sign_extend_24(b0, b1, b2),
                }),
            },
            _ => FifoFrame::Partial(chunk),
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let size = self.data.frame_bytes();
        let n = self.rest.len().div_ceil(size);
        (n, Some(n))
    }
}

impl core::iter::FusedIterator for FifoFrames<'_> {}
