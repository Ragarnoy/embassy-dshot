//! Single ESC control example
//!
//! Demonstrates controlling a single ESC using DShot protocol with
//! arming sequence, beep test, throttle ramp, and graceful shutdown.
//!
//! Sequence: arm → beep → countdown → ramp up → hold → ramp down → stop
//!
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PIN_11
//!
//! SAFETY: Secure motor before running! Remove propeller!

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embassy_dshot::dshot_embassy_rp::{DshotPio, DshotSpeed};
use embassy_dshot::{Command, DshotPioAsync};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const MAX_THROTTLE: u16 = 500;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Starting single ESC DShot example");
    info!("SAFETY: Remove propeller!");

    // Initialize DShot with 1 motor on PIN_11
    let mut dshot = DshotPio::<1, _>::new(
        p.PIO0,
        Irqs,
        p.PIN_11,
        DshotSpeed::DShot600,
    );

    info!("DShot600 initialized on PIN_11");

    // -------------------------------------------------------------------------
    // Arm: send MotorStop for 2 seconds
    // -------------------------------------------------------------------------
    info!("Arming ESC (2s)...");
    dshot.arm_async(Duration::from_secs(2)).await;
    info!("ESC armed — you should have heard startup tones");

    // -------------------------------------------------------------------------
    // Beep test — confirms DShot communication works
    // -------------------------------------------------------------------------
    info!("Sending beep command...");
    for _ in 0..10 {
        dshot.send_command_async(Command::Beep1).await;
        Timer::after(Duration::from_micros(1000)).await;
    }
    // Wait for ESC to process beep
    Timer::after(Duration::from_millis(320)).await;
    // Keep ESC alive with MotorStop
    for _ in 0..200 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(1000)).await;
    }
    info!("Did you hear a beep? If yes, DShot is working!");

    // -------------------------------------------------------------------------
    // Safety countdown before motor spin
    // -------------------------------------------------------------------------
    info!("Motor spin in 3 seconds — SECURE MOTOR!");
    dshot.arm_async(Duration::from_secs(3)).await;

    // -------------------------------------------------------------------------
    // Ramp up throttle
    // -------------------------------------------------------------------------
    info!("Ramping up throttle...");
    for throttle in (0..=MAX_THROTTLE).step_by(1) {
        if throttle % 50 == 0 {
            info!("  Throttle: {}", throttle);
        }
        for _ in 0..50 {
            dshot.throttle_async([throttle]).await.unwrap();
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // -------------------------------------------------------------------------
    // Hold at max throttle
    // -------------------------------------------------------------------------
    info!("Holding at throttle {} for ~4.5s...", MAX_THROTTLE);
    for _ in 0..9000 {
        dshot.throttle_async([MAX_THROTTLE]).await.unwrap();
        Timer::after(Duration::from_micros(500)).await;
    }

    // -------------------------------------------------------------------------
    // Ramp down
    // -------------------------------------------------------------------------
    info!("Ramping down...");
    for throttle in (0..=MAX_THROTTLE).rev().step_by(2) {
        if throttle % 50 == 0 {
            info!("  Throttle: {}", throttle);
        }
        for _ in 0..50 {
            dshot.throttle_async([throttle]).await.unwrap();
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // -------------------------------------------------------------------------
    // Stop
    // -------------------------------------------------------------------------
    info!("Stopping motor...");
    for _ in 0..2000 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("Done! Single ESC example complete.");

    loop {
        Timer::after_secs(60).await;
    }
}
