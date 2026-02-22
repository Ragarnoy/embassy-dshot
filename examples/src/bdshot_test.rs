//! Bidirectional DShot telemetry test
//!
//! Tests bidirectional DShot with eRPM telemetry. Change the `DSHOT_SPEED`
//! constant to validate different speeds.
//!
//! Sequence: arm → beep → countdown → ramp up → hold (with telemetry) → ramp down → stop
//!
//! Raw PIO data is logged for the first few errors to help debug GCR decode
//! issues. A PASS/WARN verdict is printed based on telemetry success rate.
//!
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PIN_11
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

use embassy_dshot::dshot_embassy_rp::{BidirDshotPio, DshotSpeed};
use embassy_dshot::{Command, DshotError};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// =============================================================================
// Change this constant to test different DShot speeds:
//   DshotSpeed::DShot150
//   DshotSpeed::DShot300   (default — reliable for bidir)
//   DshotSpeed::DShot600
// =============================================================================
const DSHOT_SPEED: DshotSpeed = DshotSpeed::DShot300;

const MAX_THROTTLE: u16 = 500;
const SAMPLE_COUNT: u32 = 2000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let speed_name = match DSHOT_SPEED {
        DshotSpeed::DShot150 => "DShot150",
        DshotSpeed::DShot300 => "DShot300",
        DshotSpeed::DShot600 => "DShot600",
        DshotSpeed::DShot1200 => "DShot1200",
    };
    info!("=== Bidirectional DShot Telemetry Test: {} ===", speed_name);
    info!("SAFETY: Ensure propeller is removed!");

    let mut dshot = BidirDshotPio::new(
        p.PIO0,
        Irqs,
        p.PIN_11,
        DSHOT_SPEED,
    );

    info!("BidirDshotPio initialized on PIN_11 ({})", speed_name);

    // -------------------------------------------------------------------------
    // Arm ESC with MotorStop (2 seconds)
    // -------------------------------------------------------------------------
    info!("Arming ESC (2s)...");
    dshot.arm_async(Duration::from_secs(2)).await;
    info!("ESC armed");

    // -------------------------------------------------------------------------
    // Beep test — confirms communication works
    // -------------------------------------------------------------------------
    info!("Beep test...");
    for _ in 0..10 {
        dshot.send_command_async(Command::Beep1).await;
        Timer::after(Duration::from_micros(1000)).await;
    }
    Timer::after(Duration::from_millis(320)).await;
    for _ in 0..200 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(1000)).await;
    }
    info!("Did you hear a beep? If yes, {} communication works!", speed_name);

    // -------------------------------------------------------------------------
    // Safety countdown
    // -------------------------------------------------------------------------
    info!("Motor spin in 3 seconds — SECURE MOTOR!");
    dshot.arm_async(Duration::from_secs(3)).await;

    // -------------------------------------------------------------------------
    // Ramp up
    // -------------------------------------------------------------------------
    info!("Ramping up to throttle {}...", MAX_THROTTLE);
    for throttle in (0..=MAX_THROTTLE).step_by(1) {
        if throttle % 100 == 0 {
            info!("  Throttle: {}", throttle);
        }
        for _ in 0..50 {
            let _ = dshot.throttle_with_telemetry(throttle).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // -------------------------------------------------------------------------
    // Hold and read telemetry
    // -------------------------------------------------------------------------
    info!("Holding throttle={}, reading {} telemetry samples...", MAX_THROTTLE, SAMPLE_COUNT);
    info!("Logging raw RX for first 10 GCR errors + 10 CRC errors...");

    let mut success_count = 0u32;
    let mut gcr_error_count = 0u32;
    let mut crc_error_count = 0u32;
    let mut timeout_count = 0u32;

    for i in 0..SAMPLE_COUNT {
        match dshot.throttle_with_telemetry_raw(MAX_THROTTLE).await {
            Ok((raw, Ok(telem))) => {
                success_count += 1;
                if success_count <= 5 || i % 500 == 0 {
                    let rpm_14 = telem.rpm(14);
                    info!("  [{}] OK  raw={:#010x} eRPM={} RPM(14p)={} period={}us",
                        i, raw, telem.erpm, rpm_14,
                        telem.period_us.unwrap_or(0));
                }
            }
            Ok((raw, Err(DshotError::GcrDecodeError))) => {
                gcr_error_count += 1;
                if gcr_error_count <= 10 {
                    info!("  [{}] GCR raw={:#010x}", i, raw);
                }
            }
            Ok((raw, Err(DshotError::InvalidTelemetryCrc))) => {
                crc_error_count += 1;
                if crc_error_count <= 10 {
                    info!("  [{}] CRC raw={:#010x}", i, raw);
                }
            }
            Err(DshotError::TelemetryTimeout) => {
                timeout_count += 1;
            }
            _ => {}
        }
        Timer::after(Duration::from_micros(500)).await;
    }

    let total = success_count + gcr_error_count + crc_error_count + timeout_count;
    let success_pct = if total > 0 { success_count * 100 / total } else { 0 };

    info!("=== {} Results ({} samples) ===", speed_name, total);
    info!("Success: {}/{} ({}%)", success_count, total, success_pct);
    info!("GCR errors: {}", gcr_error_count);
    info!("CRC errors: {}", crc_error_count);
    info!("Timeouts: {}", timeout_count);

    if success_pct >= 90 {
        info!("PASS: {} telemetry success rate >= 90%", speed_name);
    } else {
        info!("WARN: {} telemetry success rate < 90%", speed_name);
    }

    // -------------------------------------------------------------------------
    // Ramp down
    // -------------------------------------------------------------------------
    info!("Ramping down...");
    for throttle in (0..=MAX_THROTTLE).rev().step_by(2) {
        for _ in 0..50 {
            let _ = dshot.throttle_with_telemetry(throttle).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // -------------------------------------------------------------------------
    // Stop
    // -------------------------------------------------------------------------
    info!("Stopping motor...");
    for _ in 0..2000u32 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("Done! {} bidirectional DShot test complete.", speed_name);
    loop {
        Timer::after_secs(60).await;
    }
}
