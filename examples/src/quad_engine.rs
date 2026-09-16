//! Quad-engine bidirectional DShot example
//!
//! Demonstrates four BidirDshotPio instances sharing one PIO block
//! (one loaded program, one SM per ESC) with eRPM telemetry.
//!
//! Sequence: arm → beep → ramp up → hold (telemetry) → ramp down → stop
//!
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - Motor 1 (PIO0, sm0): GPIO11
//!   - Motor 2 (PIO0, sm1): GPIO12
//!   - Motor 3 (PIO0, sm2): GPIO13
//!   - Motor 4 (PIO0, sm3): GPIO14
//!
//! SAFETY: Remove propellers before testing!

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embassy_dshot::rp::{BidirDshotPio, BidirDshotProgram, DshotSpeed};
use embassy_dshot::Command;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const CRUISE_THROTTLE: u16 = 500;
const MOTOR_POLES: u8 = 14;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Quad-engine bidirectional DShot example");
    info!("SAFETY: Ensure propellers are removed!");

    // Four motors share one PIO block: program loaded once, one SM per ESC.
    // NOTE: sm0..sm3 make m1..m4 different types, so no arrays — drive individually.
    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO0, Irqs);
    let prog = BidirDshotProgram::new(&mut common);
    let mut m1 = BidirDshotPio::new(sm0, &mut common, p.PIN_11, &prog, DshotSpeed::DShot300);
    let mut m2 = BidirDshotPio::new(sm1, &mut common, p.PIN_12, &prog, DshotSpeed::DShot300);
    let mut m3 = BidirDshotPio::new(sm2, &mut common, p.PIN_13, &prog, DshotSpeed::DShot300);
    let mut m4 = BidirDshotPio::new(sm3, &mut common, p.PIN_14, &prog, DshotSpeed::DShot300);

    info!("M1..M4 on PIN_11..14 (PIO0/sm0..sm3)");

    // -------------------------------------------------------------------------
    // Arm: send MotorStop for 2 seconds
    // -------------------------------------------------------------------------
    info!("Arming ESCs (2s)...");
    for _ in 0..2000u32 {
        m1.send_command_async(Command::MotorStop).await;
        m2.send_command_async(Command::MotorStop).await;
        m3.send_command_async(Command::MotorStop).await;
        m4.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_millis(1)).await;
    }
    info!("ESCs armed");

    // -------------------------------------------------------------------------
    // Beep test — confirms communication works
    //
    // Only M1 beeps, on purpose: it identifies which physical motor is wired to
    // sm0. The other three are held at MotorStop so they keep their arm state.
    // -------------------------------------------------------------------------
    info!("Beep test (M1 only)...");
    for _ in 0..10 {
        m1.send_command_async(Command::Beep1).await;
        m2.send_command_async(Command::MotorStop).await;
        m3.send_command_async(Command::MotorStop).await;
        m4.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_millis(1)).await;
    }
    Timer::after(Duration::from_millis(320)).await;
    for _ in 0..200 {
        m1.send_command_async(Command::MotorStop).await;
        m2.send_command_async(Command::MotorStop).await;
        m3.send_command_async(Command::MotorStop).await;
        m4.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_millis(1)).await;
    }

    // -------------------------------------------------------------------------
    // Ramp up
    // -------------------------------------------------------------------------
    info!("Ramping up to throttle {}...", CRUISE_THROTTLE);
    for throttle in (0..=CRUISE_THROTTLE).step_by(5) {
        if throttle % 100 == 0 {
            info!("  Throttle: {}", throttle);
        }
        for _ in 0..40 {
            // Concurrent, not sequential: each call can sit on the 500us RX
            // timeout, so awaiting them in turn would cost up to 2ms per
            // iteration. The four state machines are independent, so their
            // telemetry waits overlap.
            let _ = join4(
                m1.throttle_with_telemetry(throttle),
                m2.throttle_with_telemetry(throttle),
                m3.throttle_with_telemetry(throttle),
                m4.throttle_with_telemetry(throttle),
            )
            .await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // -------------------------------------------------------------------------
    // Hold and log telemetry
    // -------------------------------------------------------------------------
    info!("Holding throttle={}, reading telemetry...", CRUISE_THROTTLE);
    for i in 0..2000u32 {
        let (r1, r2, r3, r4) = join4(
            m1.throttle_with_telemetry(CRUISE_THROTTLE),
            m2.throttle_with_telemetry(CRUISE_THROTTLE),
            m3.throttle_with_telemetry(CRUISE_THROTTLE),
            m4.throttle_with_telemetry(CRUISE_THROTTLE),
        )
        .await;
        Timer::after(Duration::from_micros(500)).await;

        if i % 500 == 0 {
            info!(
                "  [{}] M1={}rpm M2={}rpm M3={}rpm M4={}rpm",
                i,
                r1.map(|t| t.rpm(MOTOR_POLES)).unwrap_or(0),
                r2.map(|t| t.rpm(MOTOR_POLES)).unwrap_or(0),
                r3.map(|t| t.rpm(MOTOR_POLES)).unwrap_or(0),
                r4.map(|t| t.rpm(MOTOR_POLES)).unwrap_or(0),
            );
        }
    }

    // -------------------------------------------------------------------------
    // Ramp down
    // -------------------------------------------------------------------------
    info!("Ramping down...");
    for throttle in (0..=CRUISE_THROTTLE).rev().step_by(5) {
        for _ in 0..40 {
            let _ = join4(
                m1.throttle_with_telemetry(throttle),
                m2.throttle_with_telemetry(throttle),
                m3.throttle_with_telemetry(throttle),
                m4.throttle_with_telemetry(throttle),
            )
            .await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // -------------------------------------------------------------------------
    // Stop
    // -------------------------------------------------------------------------
    info!("Stopping motors...");
    for _ in 0..2000u32 {
        m1.send_command_async(Command::MotorStop).await;
        m2.send_command_async(Command::MotorStop).await;
        m3.send_command_async(Command::MotorStop).await;
        m4.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("Quad-engine test complete!");
    loop {
        Timer::after_secs(60).await;
    }
}
