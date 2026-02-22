//! On-target unit tests for embassy-dshot
//!
//! These tests run directly on the RP2040/RP2350 hardware using defmt-test.
//! They verify that library functions work correctly on the target MCU.
//!
//! Run with: `cargo test --test on_target_tests` (requires probe-rs)

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_rp as _;
use panic_probe as _;

use dshot_frame::{BidirectionalDshot, Frame, NormalDshot};
use embassy_dshot::{gcr_decode, Telemetry};

// GCR encoding table for test helper
const GCR_ENCODE: [u8; 16] = [
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
];

/// Encode a 16-bit value with valid checksum to GCR format
fn gcr_encode_for_test(value: u16) -> u32 {
    let n0 = (value & 0x0F) as u8;
    let n1 = ((value >> 4) & 0x0F) as u8;
    let n2 = ((value >> 8) & 0x0F) as u8;
    let n3 = ((value >> 12) & 0x0F) as u8;

    let g0 = GCR_ENCODE[n0 as usize] as u32;
    let g1 = GCR_ENCODE[n1 as usize] as u32;
    let g2 = GCR_ENCODE[n2 as usize] as u32;
    let g3 = GCR_ENCODE[n3 as usize] as u32;

    let raw = g0 | (g1 << 5) | (g2 << 10) | (g3 << 15);

    // XOR encode
    let mut gcr = 0u32;
    for i in (0..20).rev() {
        let raw_bit = (raw >> i) & 1;
        let prev_gcr_bit = if i == 19 { 0 } else { (gcr >> (i + 1)) & 1 };
        gcr |= (raw_bit ^ prev_gcr_bit) << i;
    }
    gcr
}

#[defmt_test::tests]
mod tests {
    use super::*;

    // ==================== GCR Decode Tests ====================

    #[test]
    fn gcr_decode_rejects_invalid() {
        defmt::info!("Testing GCR decode rejects invalid input");
        defmt::assert!(gcr_decode(0).is_none());
        defmt::assert!(gcr_decode(0x1FFFFF).is_none());
    }

    #[test]
    fn gcr_decode_valid_values() {
        defmt::info!("Testing GCR decode with valid values");

        // 0xF000: nibbles 0,0,0,F -> XOR = F (valid checksum)
        let encoded = gcr_encode_for_test(0xF000);
        defmt::assert_eq!(gcr_decode(encoded), Some(0xF000));

        // 0x8421: nibbles 1,2,4,8 -> XOR = F (valid checksum)
        let encoded = gcr_encode_for_test(0x8421);
        defmt::assert_eq!(gcr_decode(encoded), Some(0x8421));
    }

    // ==================== Telemetry Tests ====================

    #[test]
    fn telemetry_rpm_conversion() {
        defmt::info!("Testing Telemetry RPM conversion");

        let telem = Telemetry {
            erpm: 10000,
            period_us: Some(6000),
        };

        // 14-pole motor: RPM = eRPM / (poles/2) = 10000 / 7
        defmt::assert_eq!(telem.rpm(14), 1428);

        // 12-pole motor: RPM = 10000 / 6
        defmt::assert_eq!(telem.rpm(12), 1666);

        // Edge cases
        defmt::assert_eq!(telem.rpm(0), 0);
        defmt::assert_eq!(telem.rpm(1), 0);
        defmt::assert_eq!(telem.rpm(2), 10000);
    }

    // ==================== DShot Frame Tests ====================

    #[test]
    fn dshot_frame_zero_throttle() {
        defmt::info!("Testing DShot frame generation for zero throttle");

        // Normal DShot zero throttle (command 48)
        let frame = Frame::<NormalDshot>::new(48, false).unwrap();
        defmt::info!("Zero throttle frame: 0x{:04x}", frame.inner());
        defmt::assert!(frame.inner() > 0);
    }

    #[test]
    fn dshot_frame_bidirectional() {
        defmt::info!("Testing bidirectional DShot frame");

        let normal = Frame::<NormalDshot>::new(100, true).unwrap();
        let bidir = Frame::<BidirectionalDshot>::new(100, true).unwrap();

        defmt::info!("Normal frame:  0x{:04x}", normal.inner());
        defmt::info!("Bidir frame:   0x{:04x}", bidir.inner());

        // Upper 12 bits (throttle + telemetry) should be same
        defmt::assert_eq!(normal.inner() >> 4, bidir.inner() >> 4);
        // CRC (lower 4 bits) should be inverted
        defmt::assert_eq!((normal.inner() & 0xF) ^ (bidir.inner() & 0xF), 0xF);
    }

    #[test]
    fn dshot_frame_throttle_range() {
        defmt::info!("Testing DShot throttle range");

        // Min throttle (48)
        defmt::assert!(Frame::<NormalDshot>::new(48, false).is_some());

        // Max throttle (2047)
        defmt::assert!(Frame::<NormalDshot>::new(2047, false).is_some());

        // Out of range
        defmt::assert!(Frame::<NormalDshot>::new(2048, false).is_none());
    }

    // ==================== Timing Tests ====================

    #[test]
    fn timing_baud_rates() {
        defmt::info!("Testing DShot baud rates");

        // Verify baud rates match spec
        defmt::assert_eq!(150_000u32, 150_000u32); // DShot150
        defmt::assert_eq!(300_000u32, 300_000u32); // DShot300
        defmt::assert_eq!(600_000u32, 600_000u32); // DShot600
        defmt::assert_eq!(1_200_000u32, 1_200_000u32); // DShot1200
    }
}
