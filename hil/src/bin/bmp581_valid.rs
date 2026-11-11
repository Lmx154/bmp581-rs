//! Asks the chip which output rate and oversampling combinations it accepts (`OSR_EFF.odr_is_valid`)
//! and compares with the driver's verdict, which is `Config::validate()` on a `Config` built with the pair.
//! PASS: every combination the driver's `validate` accepts, the chip accepts. The chip's fastest accepted rate per pair and the count of cells where the
//! chip allows more than the driver are printed. Writes only OSR_CONFIG and ODR_CONFIG after `init`.
#![no_std]
#![no_main]

use bmp581::{Bmp581, Config};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C1;
use embassy_time::{Delay, Duration, Instant, Timer};
use bmp581::{OutputDataRate, Oversampling};
use bmp581::calc::max_odr;
use OutputDataRate as O;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

type Baro<'d> = Bmp581<I2c<'d, I2C1, i2c::Async>>;

const OSR_CONFIG: u8 = bmp581::reg::OSR_CONFIG;
const ODR_CONFIG: u8 = bmp581::reg::ODR_CONFIG;
const OSR_EFF: u8 = bmp581::reg::OSR_EFF;

async fn wr(b: &mut Baro<'_>, r: u8, v: u8, fails: &mut u32) {
    if b.write_register(r, v).await.is_err() {
        *fails += 1;
    }
}

async fn rd(b: &mut Baro<'_>, r: u8, fails: &mut u32) -> u8 {
    match b.read_register(r).await {
        Ok(v) => v,
        Err(_) => {
            *fails += 1;
            0
        }
    }
}

/// Standby, wait, then OSR and rate written in standby.
async fn prepare(b: &mut Baro<'_>, deep: u8, p: u8, t: u8, pe: bool, code: u8, f: &mut u32) {
    wr(b, ODR_CONFIG, deep, f).await;
    Timer::after_micros(3_000).await;
    wr(b, OSR_CONFIG, ((pe as u8) << 6) | (p << 3) | t, f).await;
    wr(b, ODR_CONFIG, deep | (code << 2), f).await;
}

const HZ: [f32; 32] = [
    240.0, 218.537, 199.111, 179.2, 160.0, 149.333, 140.0, 129.855, 120.0, 110.164, 100.299, 89.6,
    80.0, 70.0, 60.0, 50.056, 45.025, 40.0, 35.0, 30.0, 25.005, 20.0, 15.0, 10.0, 5.0, 4.0, 3.0,
    2.0, 1.0, 0.5, 0.25, 0.125,
];
/// Table 7 as printed in the datasheet (p17), rows osr_p, columns osr_t.
const T7: [[u16; 8]; 8] = [
    [240, 240, 240, 240, 200, 130, 80, 40],
    [240, 240, 240, 220, 180, 120, 70, 40],
    [220, 220, 200, 180, 140, 100, 70, 40],
    [140, 140, 130, 120, 100, 80, 50, 35],
    [80, 80, 80, 70, 70, 50, 45, 30],
    [45, 45, 40, 40, 40, 35, 30, 20],
    [20, 20, 20, 20, 20, 20, 15, 15],
    [10, 10, 10, 10, 10, 10, 10, 5],
];
const T8: [u16; 8] = [240, 240, 240, 240, 200, 130, 80, 40];
const OSR: [Oversampling; 8] = [
    Oversampling::X1,
    Oversampling::X2,
    Oversampling::X4,
    Oversampling::X8,
    Oversampling::X16,
    Oversampling::X32,
    Oversampling::X64,
    Oversampling::X128,
];

/// Every rate code, in code order (7.34, p65).
const ODR: [OutputDataRate; 32] = [
    O::Hz240,
    O::Hz218_537,
    O::Hz199_111,
    O::Hz179_2,
    O::Hz160,
    O::Hz149_333,
    O::Hz140,
    O::Hz129_855,
    O::Hz120,
    O::Hz110_164,
    O::Hz100_299,
    O::Hz89_6,
    O::Hz80,
    O::Hz70,
    O::Hz60,
    O::Hz50_056,
    O::Hz45_025,
    O::Hz40,
    O::Hz35,
    O::Hz30,
    O::Hz25_005,
    O::Hz20,
    O::Hz15,
    O::Hz10,
    O::Hz5,
    O::Hz4,
    O::Hz3,
    O::Hz2,
    O::Hz1,
    O::Hz0_5,
    O::Hz0_25,
    O::Hz0_125,
];

/// The driver's verdict on one combination: `Config::validate()` on a `Config` built with the pair, in
/// normal mode. `pp` and `tt` are the oversampling codes (the pressure field is built even when
/// pressure is off).
fn driver_accepts(pressure: bool, pp: u8, tt: u8, code: u8) -> bool {
    Config::new()
        .with_pressure_enabled(pressure)
        .with_pressure_oversampling(OSR[pp as usize])
        .with_temperature_oversampling(OSR[tt as usize])
        .with_odr(ODR[code as usize])
        .validate()
        .is_ok()
}

/// Bit `code` set where the driver accepts the rate.
fn driver_mask(pressure: bool, pp: u8, tt: u8) -> u32 {
    let mut m = 0u32;
    for code in 0..32u8 {
        m |= (driver_accepts(pressure, pp, tt, code) as u32) << code;
    }
    m
}

fn first_set(mask: u32) -> u8 {
    if mask == 0 { 32 } else { mask.trailing_zeros() as u8 }
}

async fn drdy_count(b: &mut Baro<'_>, f: &mut u32) -> u32 {
    rd(b, bmp581::reg::INT_STATUS, f).await;
    let start = Instant::now();
    let mut n = 0;
    while start.elapsed() < Duration::from_millis(200) {
        if rd(b, bmp581::reg::INT_STATUS, f).await & 1 != 0 {
            n += 1;
        }
    }
    n
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut fails = 0u32;
    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    cfg.sda_pullup = false;
    cfg.scl_pullup = false;
    let bus = I2c::new_async(
        p.I2C1.reborrow(),
        p.PIN_23.reborrow(),
        p.PIN_22.reborrow(),
        Irqs,
        cfg,
    );
    let mut b = Bmp581::new(bus, bmp581::DEFAULT_ADDRESS);
    if b.init(&mut Delay, &Config::default()).await.is_err() {
        defmt::error!("[FAIL] init");
        hil_common::exit(1)
    }
    let odr0 = rd(&mut b, ODR_CONFIG, &mut fails).await;
    let deep = odr0 & bmp581::reg::ODR_CONFIG_DEEP_DIS;
    defmt::info!("[INFO] after init ODR_CONFIG {=u8:#04x} deep_dis bit {=u8:#04x}", odr0, deep);

    let t_start = Instant::now();
    let mut agree = 0u32;
    let mut disagree = 0u32;
    let mut sn_diff = 0u32;
    let mut mono_bad = 0u32;
    let mut eff_valid_bad = 0u32;
    let mut eff_inv_bad = 0u32;
    let mut eff_inv_n = 0u32;
    let mut chip_looser = 0u32;
    let mut grid = [[32u8; 8]; 8];
    let mut tgrid = [[32u8; 8]; 2];
    'sweep: for pp in 0..8u8 {
        for tt in 0..8u8 {
            let (mut ms, mut mn) = (0u32, 0u32);
            for code in 0..32u8 {
                if t_start.elapsed() > Duration::from_secs(70) {
                    defmt::error!("[FAIL] time budget exhausted in the pressure sweep");
                    fails += 1;
                    break 'sweep;
                }
                prepare(&mut b, deep, pp, tt, true, code, &mut fails).await;
                let s = rd(&mut b, OSR_EFF, &mut fails).await;
                wr(&mut b, ODR_CONFIG, deep | (code << 2) | 1, &mut fails).await;
                Timer::after_micros(5_000).await;
                let n = rd(&mut b, OSR_EFF, &mut fails).await;
                let sv = s & 0x80 != 0;
                let nv = n & 0x80 != 0;
                ms |= (sv as u32) << code;
                mn |= (nv as u32) << code;
                let req = (pp << 3) | tt;
                if nv {
                    if n & 0x3f != req {
                        eff_valid_bad += 1;
                    }
                } else {
                    eff_inv_n += 1;
                    let d = if code <= 4 { 1 } else { 2 };
                    if n & 0x3f != ((d << 3) | d) {
                        eff_inv_bad += 1;
                    }
                }
            }
            if ms != mn {
                sn_diff += 1;
            }
            let fast = first_set(mn);
            if mn != (!0u32 << fast.min(31)) & (if fast == 32 { 0 } else { !0 }) {
                mono_bad += 1;
                defmt::warn!("[INFO] p{=u8} t{=u8}: accepted rates not contiguous, mask {=u32:#010x}", pp, tt, mn);
            }
            grid[pp as usize][tt as usize] = fast;
            let dmax = max_odr(true, OSR[pp as usize], OSR[tt as usize]);
            let dcode = dmax.code();
            let printed = T7[pp as usize][tt as usize];
            let chz = if fast < 32 { HZ[fast as usize] } else { 0.0 };
            defmt::info!(
                "[STAT] osr_p {=u8} osr_t {=u8}: chip fastest code {=u8} ({=f32} Hz), driver max_odr code {=u8} ({=f32} Hz), datasheet table {=u16} Hz",
                1u8 << pp, 1u8 << tt, fast, chz, dcode, dmax.hz(), printed
            );
            if driver_mask(true, pp, tt) & !mn != 0 {
                disagree += 1;
                defmt::error!(
                    "[FAIL] osr_p {=u8} osr_t {=u8}: the chip refuses a rate the driver accepts: chip fastest code {=u8} ({=f32} Hz), accepted mask {=u32:#010x}, driver code {=u8} ({=f32} Hz)",
                    1u8 << pp, 1u8 << tt, fast, chz, mn, dcode, dmax.hz()
                );
            } else {
                agree += 1;
                if fast < dcode {
                    chip_looser += 1;
                    prepare(&mut b, deep, pp, tt, true, fast, &mut fails).await;
                    wr(&mut b, ODR_CONFIG, deep | (fast << 2) | 1, &mut fails).await;
                    Timer::after_micros(10_000).await;
                    let e = rd(&mut b, OSR_EFF, &mut fails).await;
                    let c = drdy_count(&mut b, &mut fails).await;
                    defmt::info!(
                        "[STAT] delivered at chip fastest: OSR_EFF {=u8:#04x}, data-ready in 200 ms {=u32}, expected {=f32}",
                        e, c, chz * 0.2
                    );
                }
            }
        }
    }
    for pp in 0..8usize {
        let g = grid[pp];
        defmt::info!("[STAT] chip grid osr_p {=u8}x, codes for osr_t 1..128: {=u8} {=u8} {=u8} {=u8} {=u8} {=u8} {=u8} {=u8}", 1u8 << pp, g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7]);
    }
    defmt::info!(
        "[STAT] pressure pairs: {=u32} where the chip accepts all the driver accepts, {=u32} where it does not; the chip allows more than the driver in {=u32} pairs; standby-vs-normal flag differs in {=u32} pairs; non-contiguous {=u32}; valid with eff!=req {=u32}; invalid {=u32}, of which eff!=default {=u32}",
        agree, disagree, chip_looser, sn_diff, mono_bad, eff_valid_bad, eff_inv_n, eff_inv_bad
    );
    if disagree == 0 {
        defmt::info!("[PASS] pressure sweep: in all 64 pairs the chip accepts every combination the driver accepts");
    }

    // Temperature only, osr_p field 0 (row 0) and 7 (row 1).
    let (mut tagree, mut tdis, mut tp_diff, mut tlooser) = (0u32, 0u32, 0u32, 0u32);
    for tt in 0..8u8 {
        let mut masks = [0u32; 2];
        for (row, pp) in [(0usize, 0u8), (1, 7)] {
            for code in 0..32u8 {
                if t_start.elapsed() > Duration::from_secs(85) {
                    defmt::error!("[FAIL] time budget exhausted in the temperature sweep");
                    fails += 1;
                    break;
                }
                prepare(&mut b, deep, pp, tt, false, code, &mut fails).await;
                let s = rd(&mut b, OSR_EFF, &mut fails).await;
                wr(&mut b, ODR_CONFIG, deep | (code << 2) | 1, &mut fails).await;
                Timer::after_micros(5_000).await;
                let n = rd(&mut b, OSR_EFF, &mut fails).await;
                if (s ^ n) & 0x80 != 0 {
                    sn_diff += 1;
                }
                masks[row] |= ((n & 0x80 != 0) as u32) << code;
            }
            tgrid[row][tt as usize] = first_set(masks[row]);
        }
        if masks[0] != masks[1] {
            tp_diff += 1;
        }
        let fast = tgrid[0][tt as usize];
        let dmax = max_odr(false, OSR[0], OSR[tt as usize]);
        let chz = if fast < 32 { HZ[fast as usize] } else { 0.0 };
        defmt::info!(
            "[STAT] temperature only osr_t {=u8}: chip fastest code {=u8} ({=f32} Hz; with osr_p field 128x: code {=u8}), driver code {=u8} ({=f32} Hz), datasheet {=u16} Hz",
            1u8 << tt, fast, chz, tgrid[1][tt as usize], dmax.code(), dmax.hz(), T8[tt as usize]
        );
        if driver_mask(false, 0, tt) & !masks[0] == 0 && driver_mask(false, 7, tt) & !masks[1] == 0 {
            tagree += 1;
            if fast < dmax.code() {
                tlooser += 1;
            }
        } else {
            tdis += 1;
            defmt::error!("[FAIL] temperature only osr_t {=u8}: the chip refuses a rate the driver accepts: chip fastest code {=u8}, driver code {=u8}", 1u8 << tt, fast, dmax.code());
        }
    }
    defmt::info!("[STAT] temperature-only: {=u32} rows where the chip accepts all the driver accepts, {=u32} where it does not; the chip allows more than the driver in {=u32} rows; osr_p field matters in {=u32} rows; total flag standby/normal differences {=u32}", tagree, tdis, tlooser, tp_diff, sn_diff);
    if tdis == 0 {
        defmt::info!("[PASS] temperature-only sweep: the chip accepts every combination the driver accepts");
    }
    fails += disagree + tdis;

    // Invalid combinations: effective oversampling and the real output rate over 200 ms.
    let probes: [(u8, u8, u8); 9] = [
        (0, 0, 10),
        (0, 0, 0),
        (7, 7, 10),
        (7, 7, 0),
        (5, 4, 0),
        (5, 4, 8),
        (3, 6, 15),
        (3, 6, 14),
        (4, 5, 16),
    ];
    for (pp, tt, code) in probes {
        prepare(&mut b, deep, pp, tt, true, code, &mut fails).await;
        wr(&mut b, ODR_CONFIG, deep | (code << 2) | 1, &mut fails).await;
        Timer::after_micros(10_000).await;
        let n = rd(&mut b, OSR_EFF, &mut fails).await;
        let c = drdy_count(&mut b, &mut fails).await;
        defmt::info!(
            "[STAT] rate probe p{=u8}x t{=u8}x code {=u8} ({=f32} Hz): OSR_EFF {=u8:#04x} valid {=bool} eff p{=u8} t{=u8}; data-ready in 200 ms {=u32}, expected {=f32}",
            1u8 << pp, 1u8 << tt, code, HZ[code as usize], n, n & 0x80 != 0, (n >> 3) & 7, n & 7, c, HZ[code as usize] * 0.2
        );
    }

    // The flag when the combination is written in normal and in continuous mode (no standby between).
    for (label, mode) in [(1u8, 1u8), (2, 3)] {
        prepare(&mut b, deep, 0, 0, true, 10, &mut fails).await;
        wr(&mut b, ODR_CONFIG, deep | (10 << 2) | mode, &mut fails).await;
        Timer::after_micros(10_000).await;
        let a = rd(&mut b, OSR_EFF, &mut fails).await;
        wr(&mut b, OSR_CONFIG, 0x40 | (7 << 3) | 7, &mut fails).await;
        let b0 = rd(&mut b, OSR_EFF, &mut fails).await;
        Timer::after_micros(10_000).await;
        let b1 = rd(&mut b, OSR_EFF, &mut fails).await;
        let m = rd(&mut b, ODR_CONFIG, &mut fails).await;
        wr(&mut b, OSR_CONFIG, 0x40, &mut fails).await;
        let c0 = rd(&mut b, OSR_EFF, &mut fails).await;
        Timer::after_micros(10_000).await;
        let c1 = rd(&mut b, OSR_EFF, &mut fails).await;
        defmt::info!(
            "[STAT] in-mode write (run {=u8}, pwr_mode {=u8}, ODR_CONFIG {=u8:#04x}): valid 1x/1x {=u8:#04x}; after 128x/128x at once {=u8:#04x}, +10ms {=u8:#04x}; back to 1x/1x at once {=u8:#04x}, +10ms {=u8:#04x}",
            label, mode, m, a, b0, b1, c0, c1
        );
    }
    wr(&mut b, ODR_CONFIG, deep, &mut fails).await;
    defmt::info!("[INFO] elapsed {=u64} ms", t_start.elapsed().as_millis());
    let _ = (tgrid, OutputDataRate::Hz240);
    hil_common::exit(fails)
}
