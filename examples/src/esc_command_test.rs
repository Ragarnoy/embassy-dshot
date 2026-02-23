//! Comprehensive ESC command validation test
//!
//! Walks through all DShot command categories with defmt logging so the user
//! can verify each one on real hardware.
//!
//! Phases:
//!   1. Beep Test (Commands 1-5)
//!   2. Spin Direction (Commands 7-8, 20-21)
//!   3. 3D Mode (Commands 9-10)
//!   4. LED Control (Commands 22-29)
//!   5. Extended Telemetry (Commands 13-14, 42-47)
//!   6. Audio/Silent Mode (Commands 30-31)
//!   7. Signal Line Telemetry (Commands 32-35)
//!   8. ESC Info (Command 6)
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

use embassy_dshot::rp::{BidirDshotPio, DshotSpeed};
use embassy_dshot::Command;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// Low throttle value for brief motor spin tests
const TEST_THROTTLE: u16 = 200;
/// Number of repetitions for settings commands
const SETTINGS_REPEAT: u8 = 6;

/// Arm ESC by sending MotorStop for the given duration in milliseconds
async fn arm_esc(dshot: &mut BidirDshotPio<'_, impl embassy_rp::pio::Instance>, ms: u32) {
    info!("Arming ESC ({}ms)...", ms);
    dshot.arm_async(Duration::from_millis(ms as u64)).await;
    info!("ESC armed");
}

/// Brief motor spin at low throttle for direction/mode verification
async fn brief_spin(dshot: &mut BidirDshotPio<'_, impl embassy_rp::pio::Instance>, throttle: u16, duration_ms: u32) {
    info!("Spinning motor at throttle {} for {}ms...", throttle, duration_ms);
    // Ramp up
    for t in (0..=throttle).step_by(10) {
        for _ in 0..10 {
            let _ = dshot.throttle_with_telemetry(t).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }
    // Hold
    let hold_frames = duration_ms * 2; // 500us per frame
    for _ in 0..hold_frames {
        let _ = dshot.throttle_with_telemetry(throttle).await;
        Timer::after(Duration::from_micros(500)).await;
    }
    // Ramp down
    for t in (0..=throttle).rev().step_by(10) {
        for _ in 0..10 {
            let _ = dshot.throttle_with_telemetry(t).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }
    // Stop
    for _ in 0..500 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("=== ESC Command Validation Test ===");
    info!("SAFETY: Ensure propeller is removed!");

    let mut dshot = BidirDshotPio::new(
        p.PIO0,
        Irqs,
        p.PIN_11,
        DshotSpeed::DShot600,
    );

    // =========================================================================
    // Initial arm
    // =========================================================================
    arm_esc(&mut dshot, 2000).await;

    // =========================================================================
    // Phase 1: Beep Test (Commands 1-5)
    // =========================================================================
    info!("=== Phase 1: Beep Test (Commands 1-5) ===");
    let beeps = [
        (Command::Beep1, "Beep1"),
        (Command::Beep2, "Beep2"),
        (Command::Beep3, "Beep3"),
        (Command::Beep4, "Beep4"),
        (Command::Beep5, "Beep5"),
    ];
    for (cmd, name) in beeps {
        info!("Sending {}...", name);
        for _ in 0..10 {
            dshot.send_command_async(cmd).await;
            Timer::after(Duration::from_micros(1000)).await;
        }
        // Wait >260ms between beeps for ESC to process
        Timer::after(Duration::from_millis(320)).await;
        // Keep ESC alive with motor stop
        for _ in 0..200 {
            dshot.send_command_async(Command::MotorStop).await;
            Timer::after(Duration::from_micros(1000)).await;
        }
    }
    info!("Phase 1 complete — verify: 5 distinct beep tones heard");

    // =========================================================================
    // Phase 2: Spin Direction (Commands 20-21, 7-8)
    // =========================================================================
    info!("=== Phase 2: Spin Direction ===");
    arm_esc(&mut dshot, 1000).await;

    info!("Setting SpinDirectionNormal (6x)...");
    dshot.send_command_repeated_async(Command::SpinDirectionNormal, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;
    info!("Spinning motor forward...");
    brief_spin(&mut dshot, TEST_THROTTLE, 1000).await;

    arm_esc(&mut dshot, 1000).await;

    info!("Setting SpinDirectonReversed (6x)...");
    dshot.send_command_repeated_async(Command::SpinDirectonReversed, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;
    info!("Spinning motor reversed...");
    brief_spin(&mut dshot, TEST_THROTTLE, 1000).await;

    // Restore normal direction
    arm_esc(&mut dshot, 500).await;
    info!("Restoring SpinDirectionNormal (6x)...");
    dshot.send_command_repeated_async(Command::SpinDirectionNormal, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;

    info!("Phase 2 complete — verify: motor spun both directions");

    // =========================================================================
    // Phase 3: 3D Mode (Commands 9-10)
    // =========================================================================
    info!("=== Phase 3: 3D Mode ===");
    arm_esc(&mut dshot, 1000).await;

    info!("Enabling ThreeDModeOn (6x)...");
    dshot.send_command_repeated_async(Command::ThreeDModeOn, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;
    info!("Brief low throttle test in 3D mode...");
    brief_spin(&mut dshot, 100, 500).await;

    info!("Disabling ThreeDModeOff (6x)...");
    dshot.send_command_repeated_async(Command::ThreeDModeOff, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;

    info!("Phase 3 complete — verify: 3D mode toggled (if ESC supports it)");

    // =========================================================================
    // Phase 4: LED Control (Commands 22-29)
    // =========================================================================
    info!("=== Phase 4: LED Control ===");
    arm_esc(&mut dshot, 1000).await;

    let leds_on = [
        (Command::Led0On, "LED0 On"),
        (Command::Led1On, "LED1 On"),
        (Command::Led2On, "LED2 On"),
        (Command::Led3On, "LED3 On"),
    ];
    let leds_off = [
        (Command::Led0Off, "LED0 Off"),
        (Command::Led1Off, "LED1 Off"),
        (Command::Led2Off, "LED2 Off"),
        (Command::Led3Off, "LED3 Off"),
    ];

    for (cmd, name) in leds_on {
        info!("Sending {} (6x)...", name);
        dshot.send_command_repeated_async(cmd, SETTINGS_REPEAT).await;
        Timer::after(Duration::from_millis(500)).await;
        // Keep alive
        for _ in 0..100 {
            dshot.send_command_async(Command::MotorStop).await;
            Timer::after(Duration::from_micros(1000)).await;
        }
    }
    for (cmd, name) in leds_off {
        info!("Sending {} (6x)...", name);
        dshot.send_command_repeated_async(cmd, SETTINGS_REPEAT).await;
        Timer::after(Duration::from_millis(500)).await;
        for _ in 0..100 {
            dshot.send_command_async(Command::MotorStop).await;
            Timer::after(Duration::from_micros(1000)).await;
        }
    }
    info!("Phase 4 complete — verify: LEDs toggled (if ESC has them)");

    // =========================================================================
    // Phase 5: Extended Telemetry (Commands 13-14, 42-47)
    // =========================================================================
    info!("=== Phase 5: Extended Telemetry ===");

    // --- 5a: Baseline telemetry WITHOUT EDT ---
    info!("--- 5a: Baseline (no EDT) ---");
    arm_esc(&mut dshot, 1000).await;

    // Ramp up
    info!("Ramping to throttle {}...", TEST_THROTTLE);
    for t in (0..=TEST_THROTTLE).step_by(1) {
        for _ in 0..50 {
            let _ = dshot.throttle_with_telemetry(t).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // Baseline: 2000 frames at 500us, no EDT
    info!("Baseline: 2000 samples at 500us interval...");
    let mut base_ok = 0u32;
    let mut base_gcr = 0u32;
    let mut base_crc = 0u32;
    let mut base_timeout = 0u32;
    for i in 0..2000u32 {
        match dshot.throttle_with_telemetry_raw(TEST_THROTTLE).await {
            Ok((_raw, Ok(telem))) => {
                base_ok += 1;
                if base_ok <= 3 {
                    info!("  [{}] OK eRPM={} period={}us", i, telem.erpm, telem.period_us.unwrap_or(0));
                }
            }
            Ok((raw, Err(embassy_dshot::DshotError::GcrDecodeError))) => {
                base_gcr += 1;
                if base_gcr <= 5 {
                    info!("  [{}] GCR raw={:#010x}", i, raw);
                }
            }
            Ok((raw, Err(embassy_dshot::DshotError::InvalidTelemetryCrc))) => {
                base_crc += 1;
                if base_crc <= 5 {
                    info!("  [{}] CRC raw={:#010x}", i, raw);
                }
            }
            Err(_) => { base_timeout += 1; }
            _ => {}
        }
        Timer::after(Duration::from_micros(500)).await;
    }
    let base_total = base_ok + base_gcr + base_crc + base_timeout;
    info!("Baseline: {}/{} OK ({}%), GCR={} CRC={} timeout={}",
        base_ok, base_total,
        if base_total > 0 { base_ok * 100 / base_total } else { 0 },
        base_gcr, base_crc, base_timeout);

    // Stop motor
    for _ in 0..500 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    // --- 5b: Enable EDT and test ---
    info!("--- 5b: With EDT enabled ---");
    arm_esc(&mut dshot, 1000).await;

    info!("Enabling ExtendedTelemetryEnable (6x)...");
    dshot.send_command_repeated_async(Command::ExtendedTelemetryEnable, SETTINGS_REPEAT).await;

    // Check for version response
    info!("Checking for EDT version response...");
    for i in 0..10u32 {
        match dshot.read_extended_telemetry(0).await {
            Ok(telem) => {
                info!("  [{}] {:?}", i, telem);
            }
            Err(e) => {
                if i < 3 {
                    info!("  [{}] error: {:?}", i, e);
                }
            }
        }
        Timer::after(Duration::from_micros(1000)).await;
    }

    // Ramp up (same as baseline)
    info!("Ramping to throttle {}...", TEST_THROTTLE);
    for t in (0..=TEST_THROTTLE).step_by(1) {
        for _ in 0..50 {
            let _ = dshot.throttle_with_telemetry(t).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    // EDT: 2000 frames at 500us using throttle_with_telemetry_raw + EDT decode
    info!("EDT: 2000 samples at 500us interval...");
    let mut edt_ok = 0u32;
    let mut edt_gcr = 0u32;
    let mut edt_crc = 0u32;
    let mut edt_timeout = 0u32;
    let mut edt_erpm = 0u32;
    let mut edt_temp = 0u32;
    let mut edt_volt = 0u32;
    let mut edt_curr = 0u32;
    let mut edt_status = 0u32;
    let mut edt_other = 0u32;

    for i in 0..2000u32 {
        match dshot.throttle_with_telemetry_raw(TEST_THROTTLE).await {
            Ok((raw, _decoded)) => {
                // Try EDT decode on the raw data
                if let Some(raw_16) = embassy_dshot::gcr_decode(raw) {
                    if embassy_dshot::verify_telemetry_crc(raw_16) {
                        edt_ok += 1;
                        let data_12 = raw_16 >> 4;
                        let edt = embassy_dshot::decode_extended_telemetry(data_12);
                        match edt {
                            embassy_dshot::ExtendedTelemetry::Erpm { erpm, period_us } => {
                                edt_erpm += 1;
                                if edt_erpm <= 3 {
                                    info!("  [{}] eRPM={} period={}us", i, erpm, period_us.unwrap_or(0));
                                }
                            }
                            embassy_dshot::ExtendedTelemetry::Temperature(t) => {
                                edt_temp += 1;
                                info!("  [{}] Temperature={}°C", i, t);
                            }
                            embassy_dshot::ExtendedTelemetry::Voltage(mv) => {
                                edt_volt += 1;
                                info!("  [{}] Voltage={}mV ({}.{}V)", i, mv, mv / 1000, (mv % 1000) / 10);
                            }
                            embassy_dshot::ExtendedTelemetry::Current(ma) => {
                                edt_curr += 1;
                                info!("  [{}] Current={}mA ({}A)", i, ma, ma / 1000);
                            }
                            embassy_dshot::ExtendedTelemetry::Status(s) => {
                                edt_status += 1;
                                info!("  [{}] Status={:#04x}", i, s);
                            }
                            other => {
                                edt_other += 1;
                                info!("  [{}] {:?}", i, other);
                            }
                        }
                    } else {
                        edt_crc += 1;
                        if edt_crc <= 5 {
                            info!("  [{}] CRC raw={:#010x} decoded16={:#06x}", i, raw, raw_16);
                        }
                    }
                } else {
                    edt_gcr += 1;
                    if edt_gcr <= 5 {
                        info!("  [{}] GCR raw={:#010x}", i, raw);
                    }
                }
            }
            Err(_) => { edt_timeout += 1; }
        }
        Timer::after(Duration::from_micros(500)).await;
    }
    let edt_total = edt_ok + edt_gcr + edt_crc + edt_timeout;
    info!("EDT: {}/{} OK ({}%), GCR={} CRC={} timeout={}",
        edt_ok, edt_total,
        if edt_total > 0 { edt_ok * 100 / edt_total } else { 0 },
        edt_gcr, edt_crc, edt_timeout);
    info!("  eRPM={} temp={} volt={} curr={} status={} other={}",
        edt_erpm, edt_temp, edt_volt, edt_curr, edt_status, edt_other);

    // Stop motor
    for _ in 0..500 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("Disabling ExtendedTelemetryDisable (6x)...");
    dshot.send_command_repeated_async(Command::ExtendedTelemetryDisable, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;

    info!("Phase 5 complete — verify: telemetry values logged and plausible");

    // =========================================================================
    // Phase 6: Audio/Silent Mode (Commands 30-31)
    // =========================================================================
    info!("=== Phase 6: Audio/Silent Mode ===");
    arm_esc(&mut dshot, 1000).await;

    info!("Sending AudioStreamModeToggle (6x)...");
    dshot.send_command_repeated_async(Command::AudioStreamModeToggle, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(500)).await;
    for _ in 0..200 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(1000)).await;
    }

    info!("Sending SilentModeToggle (6x)...");
    dshot.send_command_repeated_async(Command::SilentModeToggle, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(500)).await;
    for _ in 0..200 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(1000)).await;
    }

    info!("Phase 6 complete — verify: mode changes acknowledged");

    // =========================================================================
    // Phase 7: Signal Line Telemetry (Commands 32-35)
    // =========================================================================
    info!("=== Phase 7: Signal Line Telemetry ===");
    arm_esc(&mut dshot, 1000).await;

    info!("Enabling SignalLineTelemetryEnable (6x)...");
    dshot.send_command_repeated_async(Command::SignalLineTelemetryEnable, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;

    info!("Setting SignalLineContinuousERPMTelemetry (6x)...");
    dshot.send_command_repeated_async(Command::SignalLineContinuousERPMTelemetry, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;

    info!("Spinning motor and reading continuous telemetry...");
    // Ramp up
    for t in (0..=TEST_THROTTLE).step_by(10) {
        for _ in 0..10 {
            let _ = dshot.throttle_with_telemetry(t).await;
            Timer::after(Duration::from_micros(500)).await;
        }
    }

    let mut success = 0u32;
    let mut errors = 0u32;
    for i in 0..100u32 {
        match dshot.throttle_with_telemetry(TEST_THROTTLE).await {
            Ok(telem) => {
                success += 1;
                if i < 5 || i % 20 == 0 {
                    info!("  [{}] eRPM={} period={}us", i, telem.erpm, telem.period_us.unwrap_or(0));
                }
            }
            Err(e) => {
                errors += 1;
                if errors <= 5 {
                    info!("  [{}] error: {:?}", i, e);
                }
            }
        }
        Timer::after(Duration::from_micros(500)).await;
    }
    info!("Signal line telemetry: {}/{} success", success, success + errors);

    // Stop
    for _ in 0..500 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("Disabling SignalLineTelemetryDisable (6x)...");
    dshot.send_command_repeated_async(Command::SignalLineTelemetryDisable, SETTINGS_REPEAT).await;
    Timer::after(Duration::from_millis(100)).await;

    info!("Phase 7 complete — verify: continuous telemetry stream received");

    // =========================================================================
    // Phase 8: ESC Info (Command 6)
    // =========================================================================
    info!("=== Phase 8: ESC Info ===");
    arm_esc(&mut dshot, 1000).await;

    info!("Sending ESCInfo command...");
    dshot.send_command_async(Command::ESCInfo).await;
    // ESC info requires >12ms wait
    Timer::after(Duration::from_millis(15)).await;

    // Read any response
    for i in 0..5u32 {
        match dshot.throttle_with_telemetry_raw(0).await {
            Ok((raw, decoded)) => {
                info!("  [{}] raw={:#010x} decoded={:?}", i, raw, decoded);
            }
            Err(e) => {
                info!("  [{}] error: {:?}", i, e);
            }
        }
        Timer::after(Duration::from_micros(1000)).await;
    }
    info!("Phase 8 complete");

    // =========================================================================
    // Final stop
    // =========================================================================
    info!("=== All phases complete ===");
    for _ in 0..2000 {
        dshot.send_command_async(Command::MotorStop).await;
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("ESC Command Validation Test finished.");
    loop {
        Timer::after_secs(60).await;
    }
}
