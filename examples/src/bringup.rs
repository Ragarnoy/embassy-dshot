//! Minimal bringup test - continuous minimum throttle at 2kHz
//!
//! Use to verify:
//! 1. Signal timing with oscilloscope
//! 2. ESC startup tones (signal recognized)
//! 3. ESC stays armed (no timeout)
//!
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PIN_11

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embassy_dshot::rp::{DshotPio, DshotSpeed};
use embassy_dshot::{Command, DshotPioAsync, DshotPioTrait};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Print system clock for verification
    let sys_freq = clk_sys_freq();
    info!("System clock: {} Hz", sys_freq);
    info!("Starting DShot300 bringup test");

    // Initialize DShot150 on PIN_11
    let mut dshot = DshotPio::<1, _>::new(
        p.PIO0,
        Irqs,
        p.PIN_11,
        DshotSpeed::DShot300,
    );

    info!("DShot300 initialized on PIN_11");
    info!("Expected DShot300 timing:");
    info!("  - Bit period: ~3.33us");
    info!("  - Bit 1 HIGH: ~2.5us (75%)");
    info!("  - Bit 0 HIGH: ~1.17us (35%)");

    // Arm ESC with MotorStop (value 0) for 2 seconds
    info!("Sending MotorStop for 2 seconds (arming sequence)...");
    info!("(ESC should produce startup tones)");
    dshot.arm_async(Duration::from_secs(2)).await;

    // Continue with MotorStop to keep ESC armed
    info!("Arming complete. Continuing MotorStop at 1kHz...");
    info!("ESC should stay armed (no motor spin)");

    let mut count: u32 = 0;
    loop {
        dshot.send_command(Command::MotorStop);
        Timer::after(Duration::from_micros(1000)).await;

        count = count.wrapping_add(1);
        if count.is_multiple_of(1000) {
            info!("Running... {} frames sent", count);
        }
    }
}
