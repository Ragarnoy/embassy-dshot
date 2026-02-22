//! Twin-engine bidirectional DShot example with RPM comparison
//!
//! Demonstrates using two BidirDshotPio instances (one per PIO block) to
//! drive two ESCs with telemetry, then compares the RPM readings to check
//! that both motors spin at similar speeds.
//!
//! Phases:
//!   1. Arm both ESCs (2s MotorStop)
//!   2. Ramp up to cruise throttle
//!   3. Hold and compare RPMs between engines
//!   4. Differential throttle demo
//!   5. Ramp down and stop
//!
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - Engine 1 (PIO0): GPIO11
//!   - Engine 2 (PIO1): GPIO12
//!
//! SAFETY: Remove propellers before testing!

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::InterruptHandler;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embassy_dshot::dshot_embassy_rp::{BidirDshotPio, DshotSpeed};
use embassy_dshot::Command;

bind_interrupts!(struct Irqs0 {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

bind_interrupts!(struct Irqs1 {
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

const CRUISE_THROTTLE: u16 = 300;
const MOTOR_POLES: u8 = 14;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Twin-engine bidirectional DShot example");
    info!("SAFETY: Ensure propellers are removed!");

    // One BidirDshotPio per PIO block — each drives a single ESC
    let mut engine1 = BidirDshotPio::new(p.PIO0, Irqs0, p.PIN_11, DshotSpeed::DShot300);
    let mut engine2 = BidirDshotPio::new(p.PIO1, Irqs1, p.PIN_12, DshotSpeed::DShot300);

    info!("Engine 1 on PIN_11 (PIO0), Engine 2 on PIN_12 (PIO1)");

    // -------------------------------------------------------------------------
    // Phase 1: Arm both ESCs
    // -------------------------------------------------------------------------
    info!("Arming ESCs (2s)...");
    for _ in 0..2000u32 {
        engine1.send_command_async(Command::MotorStop).await;
        engine2.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_millis(1)).await;
    }
    info!("ESCs armed");

    // -------------------------------------------------------------------------
    // Beep test: different tones to identify each ESC
    // Beep commands need 1ms spacing, >260ms for ESC to sound the tone,
    // then MotorStop keep-alive so the ESC doesn't disarm from signal loss.
    // -------------------------------------------------------------------------
    info!("Beep test: E1=Beep1");
    for _ in 0..10 {
        engine1.send_command_async(Command::Beep1).await;
        engine2.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_millis(1)).await;
    }
    Timer::after(Duration::from_millis(320)).await;
    // Keep both ESCs alive
    for _ in 0..200 {
        engine1.send_command_async(Command::MotorStop).await;
        engine2.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_millis(1)).await;
    }

    info!("Beep test: E2=Beep2");
    for _ in 0..10 {
        engine1.send_command_async(Command::MotorStop).await;
        engine2.send_command_async(Command::Beep2).await;
        Timer::after(Duration::from_millis(1)).await;
    }
    Timer::after(Duration::from_millis(320)).await;
    for _ in 0..200 {
        engine1.send_command_async(Command::MotorStop).await;
        engine2.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_millis(1)).await;
    }

    // -------------------------------------------------------------------------
    // Phase 2: Ramp up
    // -------------------------------------------------------------------------
    info!("Ramping up to throttle {}...", CRUISE_THROTTLE);
    for throttle in (0..=CRUISE_THROTTLE).step_by(5) {
        if throttle % 50 == 0 {
            info!("  Throttle: {}", throttle);
        }
        for _ in 0..40 {
            let _ = engine1.throttle_with_telemetry(throttle).await;
            let _ = engine2.throttle_with_telemetry(throttle).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // -------------------------------------------------------------------------
    // Phase 3: Hold and compare RPMs
    // -------------------------------------------------------------------------
    info!("Holding throttle={}, comparing RPMs...", CRUISE_THROTTLE);

    let mut matched = 0u32;
    let mut mismatched = 0u32;
    let mut e1_only = 0u32;
    let mut e2_only = 0u32;
    let mut both_fail = 0u32;

    // ~5 seconds at 500us interval
    for i in 0..10000u32 {
        let r1 = engine1.throttle_with_telemetry(CRUISE_THROTTLE).await;
        let r2 = engine2.throttle_with_telemetry(CRUISE_THROTTLE).await;
        Timer::after(Duration::from_micros(500)).await;

        match (r1, r2) {
            (Ok(t1), Ok(t2)) => {
                let rpm1 = t1.rpm(MOTOR_POLES);
                let rpm2 = t2.rpm(MOTOR_POLES);

                // Consider RPMs "matched" if within 10% of average
                let avg = (rpm1 + rpm2) / 2;
                let threshold = avg / 10; // 10%
                let diff = if rpm1 > rpm2 { rpm1 - rpm2 } else { rpm2 - rpm1 };

                if diff <= threshold {
                    matched += 1;
                } else {
                    mismatched += 1;
                }

                // Log periodically
                if i % 1000 == 0 || (mismatched > 0 && mismatched <= 5) {
                    let status = if diff <= threshold { "MATCH" } else { "DRIFT" };
                    info!(
                        "  [{}] {} E1={}rpm E2={}rpm diff={}rpm ({}%)",
                        i, status, rpm1, rpm2, diff,
                        if avg > 0 { diff * 100 / avg } else { 0 }
                    );
                }
            }
            (Ok(_), Err(_)) => e1_only += 1,
            (Err(_), Ok(_)) => e2_only += 1,
            (Err(_), Err(_)) => both_fail += 1,
        }
    }

    let total_good = matched + mismatched;
    info!("=== RPM Comparison Results ===");
    info!(
        "Matched (<10%%): {}/{} ({}%)",
        matched,
        total_good,
        if total_good > 0 { matched * 100 / total_good } else { 0 }
    );
    info!("Drifted (>10%%): {}", mismatched);
    info!("E1 only: {}, E2 only: {}, Both fail: {}", e1_only, e2_only, both_fail);

    // -------------------------------------------------------------------------
    // Phase 4: Differential throttle
    // -------------------------------------------------------------------------
    info!("Differential throttle demo (E1 ramps, E2 holds)...");
    for i in 0..50u16 {
        let t1 = 100 + (i * 4);
        let t2 = 100u16;
        for _ in 0..100 {
            let _ = engine1.throttle_with_telemetry(t1).await;
            let _ = engine2.throttle_with_telemetry(t2).await;
            Timer::after(Duration::from_micros(500)).await;
        }
        if i % 10 == 0 {
            info!("  E1={} E2={}", t1, t2);
        }
    }

    // -------------------------------------------------------------------------
    // Phase 5: Ramp down and stop
    // -------------------------------------------------------------------------
    info!("Ramping down...");
    for throttle in (0..=CRUISE_THROTTLE).rev().step_by(5) {
        for _ in 0..40 {
            let _ = engine1.throttle_with_telemetry(throttle).await;
            let _ = engine2.throttle_with_telemetry(throttle).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    info!("Stopping motors...");
    for _ in 0..2000u32 {
        engine1.send_command_async(Command::MotorStop).await;
        engine2.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("Twin-engine test complete!");
    loop {
        Timer::after_secs(60).await;
    }
}
