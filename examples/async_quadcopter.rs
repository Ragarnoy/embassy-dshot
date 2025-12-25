//! Async quadcopter motor control example
//!
//! This example demonstrates using the async API to control 4 motors concurrently
//! with other async tasks like reading sensors or handling commands.
//!
//! Hardware: Raspberry Pi Pico (RP2040)
//! Connections:
//!   - Motor 1: GPIO 13
//!   - Motor 2: GPIO 7
//!   - Motor 3: GPIO 6
//!   - Motor 4: GPIO 12

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use dshot_pio::dshot_embassy_rp::DshotPio;
use dshot_pio::{Command, DshotPioAsync, DshotPioTrait};

// Bind PIO0 interrupt
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Starting async DShot quadcopter example");

    // Initialize DShot with 4 motors
    // System clock: 125 MHz, DShot 600
    // Clock divider = 125_000_000 / (8 * 600 * 1000) = 26.04 ≈ 26
    let mut dshot = DshotPio::<4, _>::new(
        p.PIO0,
        Irqs,
        p.PIN_13, // Motor 1
        p.PIN_7,  // Motor 2
        p.PIN_6,  // Motor 3
        p.PIN_12, // Motor 4
        (26, 0),  // Clock divider for DShot 600
    );

    info!("DShot initialized");

    // Initialize motors: Send motor stop command
    info!("Initializing motors...");
    dshot.send_command(Command::MotorStop);
    Timer::after(Duration::from_millis(10)).await;

    // Arm sequence: Set throttle to zero for 1 second
    info!("Arming motors (throttle zero for 1s)...");
    for _ in 0..100 {
        dshot.throttle_minimum();
        Timer::after(Duration::from_millis(10)).await;
    }

    info!("Motors armed!");

    // Demonstrate async throttle control
    // Gradually increase throttle using async methods
    info!("Ramping up throttle asynchronously...");

    for throttle in (0..=200).step_by(10) {
        // Use async method - this yields to executor if FIFO is full
        dshot
            .throttle_async([throttle, throttle, throttle, throttle])
            .await
            .unwrap();

        // While motors are updating, we could be doing other async work here
        // For example: reading sensors, processing commands, etc.
        Timer::after(Duration::from_millis(100)).await;
    }

    info!("Holding at throttle 200 for 2 seconds...");
    for _ in 0..20 {
        dshot.throttle_async([200, 200, 200, 200]).await.unwrap();
        Timer::after(Duration::from_millis(100)).await;
    }

    // Demonstrate differential throttle (for rotation)
    info!("Demonstrating differential throttle...");
    for i in 0..50 {
        let t = 100 + (i * 2);
        // Motors 1&3 higher, motors 2&4 lower (yaw rotation)
        dshot.throttle_async([t, 100, t, 100]).await.unwrap();
        Timer::after(Duration::from_millis(50)).await;
    }

    // Gradual shutdown
    info!("Shutting down motors...");
    for throttle in (0..=200).rev().step_by(10) {
        dshot
            .throttle_async([throttle, throttle, throttle, throttle])
            .await
            .unwrap();
        Timer::after(Duration::from_millis(50)).await;
    }

    // Final stop
    dshot.throttle_minimum_async().await;
    Timer::after(Duration::from_millis(100)).await;

    // Send motor stop command
    dshot.send_command_async(Command::MotorStop).await;

    info!("Example complete!");

    // Keep running
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
