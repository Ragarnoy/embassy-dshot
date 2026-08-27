//! RPM range finder using EDT (Extended DShot Telemetry)
//!
//! Sweeps the full throttle range while reading bidirectional DShot telemetry
//! to discover the minimum and maximum mechanical RPM the motor can sustain.
//!
//! Phases:
//!   1. Arm ESC and enable EDT
//!   2. Sweep throttle 48 → 2000 in small steps, collecting RPM samples
//!   3. Filter outliers per step using IQR (interquartile range) method
//!   4. Ramp down and stop
//!   5. Print summary with min/max RPM, temperature, and voltage
//!
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PIN_15
//!
//! SAFETY: Remove propeller from motor before testing!

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embassy_dshot::rp::{BidirDshotPio, DshotSpeed};
use embassy_dshot::Command;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// Number of motor poles (adjust for your motor)
const MOTOR_POLES: u8 = 14;
/// Maximum throttle value to sweep to
const MAX_THROTTLE: u16 = 2000;
/// Throttle step size during sweep
const THROTTLE_STEP: u16 = 25;
/// Number of telemetry samples to collect at each throttle step
const SAMPLES_PER_STEP: usize = 500;
/// Samples to discard at the start of each step (let motor settle)
const SETTLE_SAMPLES: usize = 200;
/// Minimum valid RPM readings (after filtering) to consider a step stable
const MIN_STABLE_COUNT: usize = 30;
/// Number of repetitions for settings commands
const SETTINGS_REPEAT: u8 = 6;

/// Simple insertion sort for the RPM buffer (no alloc needed)
fn sort(buf: &mut [u32], len: usize) {
    for i in 1..len {
        let key = buf[i];
        let mut j = i;
        while j > 0 && buf[j - 1] > key {
            buf[j] = buf[j - 1];
            j -= 1;
        }
        buf[j] = key;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("=== RPM Range Finder (EDT DShot) ===");
    info!("SAFETY: Ensure propeller is removed!");
    info!("Motor poles: {}, Max throttle: {}", MOTOR_POLES, MAX_THROTTLE);

    let mut dshot = BidirDshotPio::new(p.PIO0, Irqs, p.PIN_14, DshotSpeed::DShot300);

    // =========================================================================
    // Phase 1: Arm and enable EDT
    // =========================================================================
    info!("Arming ESC (2s)...");
    dshot.arm_async(Duration::from_secs(2)).await;
    info!("ESC armed");

    info!("Enabling Extended Telemetry (6x)...");
    dshot
        .send_command_repeated_async(Command::ExtendedTelemetryEnable, SETTINGS_REPEAT)
        .await;
    Timer::after(Duration::from_millis(100)).await;

    // Keep alive after EDT enable
    for _ in 0..200 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    // =========================================================================
    // Phase 2–3: Throttle sweep — record min and max RPM
    // =========================================================================
    info!("Starting throttle sweep (48 -> {})...", MAX_THROTTLE);

    let mut global_min_rpm: u32 = u32::MAX;
    let mut global_max_rpm: u32 = 0;
    let mut min_rpm_throttle: u16 = 0;
    let mut max_rpm_throttle: u16 = 0;
    let mut first_spin_throttle: u16 = 0;
    let mut last_temp: Option<u8> = None;
    let mut last_voltage_mv: Option<u32> = None;
    let mut last_current_ma: Option<u32> = None;
    let mut total_samples: u32 = 0;
    let mut total_ok: u32 = 0;
    let mut total_errors: u32 = 0;

    // Buffer for RPM readings at each step (used for median + outlier filtering)
    let mut rpm_buf = [0u32; SAMPLES_PER_STEP];

    // DShot values 0–47 are special commands, throttle starts at 48
    let mut throttle: u16 = 48;
    while throttle <= MAX_THROTTLE {
        let mut rpm_count: usize = 0;

        // Settle phase: send throttle but discard readings to let motor reach steady-state
        for _ in 0..SETTLE_SAMPLES {
            let _ = dshot.throttle_with_telemetry(throttle).await;
            Timer::after(Duration::from_micros(500)).await;
        }

        // Measurement phase: collect RPM samples
        for _ in 0..SAMPLES_PER_STEP {
            total_samples += 1;

            match dshot.throttle_with_telemetry(throttle).await {
                Ok(telem) => {
                    total_ok += 1;
                    let erpm = telem.erpm;
                    if erpm > 0 {
                        let rpm = erpm / (MOTOR_POLES as u32 / 2);
                        if rpm_count < SAMPLES_PER_STEP {
                            rpm_buf[rpm_count] = rpm;
                            rpm_count += 1;
                        }
                    }
                }
                Err(_) => {
                    total_errors += 1;
                }
            }

            // Also try to capture EDT sensor data periodically
            if total_samples % 50 == 0 {
                if let Ok(edt) = dshot.read_extended_telemetry(throttle).await {
                    match edt {
                        embassy_dshot::ExtendedTelemetry::Temperature(t) => {
                            last_temp = Some(t);
                        }
                        embassy_dshot::ExtendedTelemetry::Voltage(mv) => {
                            last_voltage_mv = Some(mv);
                        }
                        embassy_dshot::ExtendedTelemetry::Current(ma) => {
                            last_current_ma = Some(ma);
                        }
                        _ => {}
                    }
                }
            }

            Timer::after(Duration::from_micros(500)).await;
        }

        // Filter outliers: sort, then use IQR (interquartile range) method
        // Keeps values within [Q1 - 1.5*IQR, Q3 + 1.5*IQR]
        if rpm_count >= MIN_STABLE_COUNT {
            sort(&mut rpm_buf, rpm_count);
            let q1 = rpm_buf[rpm_count / 4];
            let q3 = rpm_buf[3 * rpm_count / 4];
            let iqr = q3.saturating_sub(q1);
            let margin = iqr + iqr / 2; // 1.5 * IQR
            let lower = q1.saturating_sub(margin);
            let upper = q3.saturating_add(margin);

            let mut filtered_sum: u64 = 0;
            let mut filtered_count: u32 = 0;
            let mut filtered_min: u32 = u32::MAX;
            let mut filtered_max: u32 = 0;

            for i in 0..rpm_count {
                let rpm = rpm_buf[i];
                if rpm >= lower && rpm <= upper {
                    filtered_sum += u64::from(rpm);
                    filtered_count += 1;
                    if rpm < filtered_min {
                        filtered_min = rpm;
                    }
                    if rpm > filtered_max {
                        filtered_max = rpm;
                    }
                }
            }

            if filtered_count >= MIN_STABLE_COUNT as u32 {
                let avg_rpm = (filtered_sum / filtered_count as u64) as u32;

                if first_spin_throttle == 0 {
                    first_spin_throttle = throttle;
                    info!(
                        "First stable RPM at throttle {}: avg={}rpm (min={} max={}, {}/{} after filter)",
                        throttle, avg_rpm, filtered_min, filtered_max,
                        filtered_count, rpm_count
                    );
                }

                if filtered_min < global_min_rpm && filtered_min > 0 {
                    global_min_rpm = filtered_min;
                    min_rpm_throttle = throttle;
                }
                if filtered_max > global_max_rpm {
                    global_max_rpm = filtered_max;
                    max_rpm_throttle = throttle;
                }

                info!(
                    "  T={}: avg={}  min={}  max={}  ({}/{})",
                    throttle, avg_rpm, filtered_min, filtered_max,
                    filtered_count, rpm_count
                );
            } else {
                info!(
                    "  T={}: only {}/{} survived filter (Q1={} Q3={})",
                    throttle, filtered_count, rpm_count, q1, q3
                );
            }
        } else {
            info!(
                "  T={}: {} valid readings (unstable)",
                throttle, rpm_count
            );
        }

        throttle = throttle.saturating_add(THROTTLE_STEP);
    }

    // =========================================================================
    // Phase 4: Ramp down and stop
    // =========================================================================
    info!("Ramping down...");
    let mut ramp_down = MAX_THROTTLE;
    while ramp_down > 0 {
        ramp_down = ramp_down.saturating_sub(THROTTLE_STEP);
        for _ in 0..60 {
            let _ = dshot.throttle_with_telemetry(ramp_down).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    info!("Stopping motor...");
    for _ in 0..2000u32 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    // Disable EDT
    dshot
        .send_command_repeated_async(Command::ExtendedTelemetryDisable, SETTINGS_REPEAT)
        .await;
    Timer::after(Duration::from_millis(100)).await;

    // =========================================================================
    // Phase 5: Summary
    // =========================================================================
    info!("============================================");
    info!("=== RPM Range Results ===");
    info!("============================================");
    if global_min_rpm < u32::MAX {
        info!("Min RPM: {} (at throttle {})", global_min_rpm, min_rpm_throttle);
        info!("Max RPM: {} (at throttle {})", global_max_rpm, max_rpm_throttle);
        info!("First stable spin at throttle: {}", first_spin_throttle);
    } else {
        info!("No valid RPM readings obtained!");
    }
    info!(
        "Total samples: {}, OK telemetry: {}, errors: {}",
        total_samples, total_ok, total_errors
    );
    if let Some(temp) = last_temp {
        info!("Last temperature: {}C", temp);
    }
    if let Some(mv) = last_voltage_mv {
        info!("Last voltage: {}.{}V", mv / 1000, (mv % 1000) / 10);
    }
    if let Some(ma) = last_current_ma {
        info!("Last current: {}.{}A", ma / 1000, (ma % 1000) / 100);
    }
    info!("Motor poles: {}", MOTOR_POLES);
    info!("============================================");

    info!("RPM range test complete!");
    loop {
        Timer::after_secs(60).await;
    }
}
