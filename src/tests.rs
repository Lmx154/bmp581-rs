use std::vec::Vec;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};

use crate::*;

#[test]
fn enums_encode_to_datasheet_codes() {
    let osr = [
        Oversampling::X1,
        Oversampling::X2,
        Oversampling::X4,
        Oversampling::X8,
        Oversampling::X16,
        Oversampling::X32,
        Oversampling::X64,
        Oversampling::X128,
    ];
    for (i, o) in osr.iter().enumerate() {
        assert_eq!(o.code(), i as u8);
    }
    let iir = [
        IirFilter::Bypass,
        IirFilter::Coeff1,
        IirFilter::Coeff3,
        IirFilter::Coeff7,
        IirFilter::Coeff15,
        IirFilter::Coeff31,
        IirFilter::Coeff63,
        IirFilter::Coeff127,
    ];
    for (i, f) in iir.iter().enumerate() {
        assert_eq!(f.code(), i as u8);
    }
    assert_eq!(PowerMode::Standby.code(), 0);
    assert_eq!(PowerMode::Normal.code(), 1);
    assert_eq!(PowerMode::Forced.code(), 2);
    assert_eq!(PowerMode::Continuous.code(), 3);
    assert_eq!(OutputDataRate::Hz240.code(), 0x00);
    assert_eq!(OutputDataRate::Hz100_299.code(), 0x0A);
    assert_eq!(OutputDataRate::Hz50_056.code(), 0x0F);
    assert_eq!(OutputDataRate::Hz40.code(), 0x11);
    assert_eq!(OutputDataRate::Hz1.code(), 0x1C);
    assert_eq!(OutputDataRate::Hz0_125.code(), 0x1F);
    assert_eq!(OutputDataRate::Hz50_056.hz(), 50.056);
}

#[test]
fn config_encoders() {
    let c = Config::default();
    assert_eq!(c.osr_config(), 0x40 | (4 << 3) | 1);
    assert_eq!(c.odr_config(PowerMode::Normal), (0x0F << 2) | 1);
    assert_eq!(c.dsp_config(), 0x03);
    assert_eq!(c.dsp_iir(), 0);
    assert_eq!(c.int_source(), 0x01);
    assert_eq!(c.int_config(), 0x3A);
    let c = c
        .with_pressure_iir(IirFilter::Coeff7)
        .with_temperature_iir(IirFilter::Coeff3)
        .with_interrupt(InterruptPin::off());
    assert_eq!(c.dsp_config(), 0x03 | 0x20 | 0x08);
    assert_eq!(c.dsp_iir(), (3 << 3) | 2);
    assert_eq!(c.int_source(), 0);
    assert_eq!(c.int_config(), 0x32);
}

#[test]
fn decode_and_convert() {
    // Temperature 0x19_0000 = 25.0 degC; pressure 0x1A_8000 = 1736704 / 64 = 27136 Pa.
    let s = RawSample::decode(&[0x00, 0x00, 0x19, 0x00, 0x80, 0x1A]);
    assert_eq!(s.temperature, 0x19_0000);
    assert_eq!(s.pressure, 0x1A_8000);
    assert_eq!(s.celsius(), 25.0);
    assert_eq!(s.pascals(), 0x1A_8000 as f32 / 64.0);
    // Negative temperature: -10.5 degC = -688128 = 0xF5_8000.
    let s = RawSample::decode(&[0x00, 0x80, 0xF5, 0x00, 0x00, 0x19]);
    assert_eq!(s.temperature, -688_128);
    assert_eq!(s.celsius(), -10.5);
    assert_eq!(s.pressure, 0x19_0000);
    assert_eq!(s.pascals(), 25_600.0);
}

#[derive(Debug)]
struct Nack;

impl embedded_hal::i2c::Error for Nack {
    fn kind(&self) -> ErrorKind {
        ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
    }
}

/// A register file at one address; the soft-reset write is not acknowledged, as on the real chip.
struct FakeChip {
    address: u8,
    regs: [u8; 256],
    writes: Vec<(u8, u8)>,
}

impl ErrorType for FakeChip {
    type Error = Nack;
}

impl I2c for FakeChip {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Nack> {
        if address != self.address {
            return Err(Nack);
        }
        let mut ptr = 0usize;
        for op in operations {
            match op {
                Operation::Write(bytes) => {
                    ptr = bytes[0] as usize;
                    if bytes.len() == 2 {
                        self.writes.push((bytes[0], bytes[1]));
                        if bytes[0] == reg::CMD {
                            return Err(Nack);
                        }
                        self.regs[ptr] = bytes[1];
                    }
                }
                Operation::Read(buf) => {
                    for b in buf.iter_mut() {
                        *b = self.regs[ptr];
                        ptr += 1;
                    }
                }
            }
        }
        Ok(())
    }
}

struct NoDelay;

impl DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

fn fake() -> FakeChip {
    let mut regs = [0u8; 256];
    regs[reg::CHIP_ID as usize] = 0x50;
    regs[reg::STATUS as usize] = 0x02;
    regs[reg::OSR_EFF as usize] = 0x80 | (4 << 3) | 1;
    FakeChip {
        address: DEFAULT_ADDRESS,
        regs,
        writes: Vec::new(),
    }
}

#[test]
fn blocking_init_then_read() {
    let mut baro = blocking::Bmp581::new(fake(), DEFAULT_ADDRESS);
    baro.init(&mut NoDelay, &Config::default()).unwrap();
    let mut chip = baro.release();
    assert_eq!(
        chip.writes,
        [
            (reg::CMD, 0xB6),
            (reg::OSR_CONFIG, 0x61),
            (reg::ODR_CONFIG, 0x0F << 2),
            (reg::DSP_CONFIG, 0x03),
            (reg::DSP_IIR, 0x00),
            (reg::INT_SOURCE, 0x01),
            (reg::INT_CONFIG, 0x3A),
            (reg::ODR_CONFIG, (0x0F << 2) | 1),
        ]
    );
    chip.regs[0x1D..0x23].copy_from_slice(&[0x00, 0x00, 0x19, 0x00, 0x80, 0x1A]);
    let mut baro = blocking::Bmp581::new(chip, DEFAULT_ADDRESS);
    let mut buf = [0u8; 6];
    baro.read_registers(reg::TEMP_DATA_XLSB, &mut buf).unwrap();
    let s = RawSample::decode(&buf);
    assert_eq!(s.celsius(), 25.0);
    assert_eq!(s.pressure, 0x1A_8000);
}

#[test]
fn init_rejects_wrong_id_and_invalid_config() {
    let mut chip = fake();
    chip.regs[reg::CHIP_ID as usize] = 0x51;
    let mut baro = blocking::Bmp581::new(chip, DEFAULT_ADDRESS);
    assert!(matches!(
        baro.init(&mut NoDelay, &Config::default()),
        Err(Error::WrongId { found: 0x51 })
    ));

    let mut chip = fake();
    chip.regs[reg::OSR_EFF as usize] = 0x00;
    let mut baro = blocking::Bmp581::new(chip, DEFAULT_ADDRESS);
    assert!(matches!(
        baro.init(&mut NoDelay, &Config::default()),
        Err(Error::ChipRejectsRate { osr_eff: 0x00 })
    ));
}
