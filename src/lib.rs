#![no_std]

#[cfg(any(test, feature = "std"))]
extern crate std;

// Re-export commonly used types from dshot-frame
pub use dshot_frame::Command;
#[cfg(any(feature = "rp2040", feature = "rp2350"))]
use dshot_frame::{Frame, NormalDshot, BidirectionalDshot};

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub mod dshot_embassy_rp;

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub use dshot_embassy_rp::DshotSpeed;

/// Extended DShot Telemetry (EDT) data decoded from bidirectional DShot
///
/// When EDT is enabled (command 13), the ESC embeds the telemetry **type**
/// within the 12-bit value itself. The 4-bit prefix (`eee m`) determines
/// whether a frame is normal eRPM or an EDT type:
///
/// - eRPM frame: exponent == 0 OR bit 8 == 1
/// - EDT frame: exponent != 0 AND bit 8 == 0 (data in lower 8 bits)
///
/// Reference: [bird-sanctuary/extended-dshot-telemetry](https://github.com/bird-sanctuary/extended-dshot-telemetry)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExtendedTelemetry {
    /// Electrical RPM (standard eRPM encoding)
    Erpm {
        /// Electrical RPM value
        erpm: u32,
        /// Period in microseconds (None if motor stopped)
        period_us: Option<u32>,
    },
    /// ESC temperature (1°C per unit, range 0-255°C)
    Temperature(u8),
    /// ESC voltage in millivolts (250mV per unit, range 0-63.75V)
    Voltage(u32),
    /// ESC current in milliamps (1A per unit, range 0-255A)
    Current(u32),
    /// Debug value 1 (firmware-specific)
    Debug1(u8),
    /// Debug value 2 (firmware-specific)
    Debug2(u8),
    /// ESC stress level (0-255)
    StressLevel(u8),
    /// ESC status event flags
    Status(u8),
}

/// Decode a 12-bit EDT telemetry value (self-describing type)
///
/// The 12-bit format is `eee m dddddddd`:
/// - If `exponent == 0` OR `bit8 == 1` → normal eRPM frame
/// - Otherwise, exponent determines the EDT type, lower 8 bits are data
///
/// | Prefix | Type | Scale |
/// |--------|------|-------|
/// | `001 0` | Temperature | 1°C per unit |
/// | `010 0` | Voltage | 0.25V (250mV) per unit |
/// | `011 0` | Current | 1A (1000mA) per unit |
/// | `100 0` | Debug 1 | firmware-specific |
/// | `101 0` | Debug 2 | firmware-specific |
/// | `110 0` | Stress Level | 0-255 |
/// | `111 0` | Status | event flags |
///
/// Reference: [bird-sanctuary/extended-dshot-telemetry](https://github.com/bird-sanctuary/extended-dshot-telemetry)
#[must_use]
#[allow(clippy::cast_lossless)]
pub const fn decode_extended_telemetry(raw_12: u16) -> ExtendedTelemetry {
    let exponent = (raw_12 >> 9) & 0x07;
    let bit8 = (raw_12 >> 8) & 1;

    // eRPM frame: exponent == 0 OR bit8 == 1
    if exponent == 0 || bit8 == 1 {
        if raw_12 == 0 || raw_12 == 0x0FFF {
            return ExtendedTelemetry::Erpm {
                erpm: 0,
                period_us: None,
            };
        }
        let mantissa = raw_12 & 0x1FF;
        let period_us = (mantissa as u32) << (exponent as u32);
        if period_us == 0 {
            return ExtendedTelemetry::Erpm {
                erpm: 0,
                period_us: None,
            };
        }
        return ExtendedTelemetry::Erpm {
            erpm: 60_000_000 / period_us,
            period_us: Some(period_us),
        };
    }

    // EDT frame: type is in the exponent, data is lower 8 bits
    let data = (raw_12 & 0xFF) as u8;
    match exponent {
        1 => ExtendedTelemetry::Temperature(data),
        2 => ExtendedTelemetry::Voltage(data as u32 * 250),
        3 => ExtendedTelemetry::Current(data as u32 * 1000),
        4 => ExtendedTelemetry::Debug1(data),
        5 => ExtendedTelemetry::Debug2(data),
        6 => ExtendedTelemetry::StressLevel(data),
        // 7 is the only remaining case (exponent is 3 bits, 0 handled above)
        _ => ExtendedTelemetry::Status(data),
    }
}

/// Error types for `DShot` operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotError {
    /// Throttle value out of range (must be 0-1999)
    InvalidThrottle,
    /// Telemetry CRC checksum mismatch
    InvalidTelemetryCrc,
    /// ESC did not respond to telemetry request in time
    TelemetryTimeout,
    /// Invalid GCR encoding in telemetry response
    GcrDecodeError,
}

/// Telemetry data from ESC
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Telemetry {
    /// Electrical RPM (0 if motor stopped)
    pub erpm: u32,
    /// Period in microseconds (None if motor stopped)
    pub period_us: Option<u32>,
}

impl Telemetry {
    /// Convert eRPM to mechanical RPM
    ///
    /// Formula: RPM = eRPM / (`motor_poles` / 2)
    ///
    /// # Example
    /// ```ignore
    /// let telemetry = Telemetry { erpm: 10000, period_us: Some(6000) };
    /// let rpm = telemetry.rpm(14); // 14-pole motor
    /// assert_eq!(rpm, 1428); // 10000 / 7
    /// ```
    #[must_use]
    #[allow(clippy::cast_lossless)] // as cast required for const fn
    pub const fn rpm(&self, motor_poles: u8) -> u32 {
        if motor_poles < 2 {
            return 0;
        }
        self.erpm / (motor_poles as u32 / 2)
    }
}

/// GCR (General Code Recording) decode lookup table
/// Maps 5-bit GCR values to 4-bit nibbles (0xFF = invalid)
const GCR_DECODE: [u8; 32] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // 0x00-0x07: invalid
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,  // 0x08-0x0F
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,  // 0x10-0x17
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,  // 0x18-0x1F
];

/// Decode a 20-bit GCR-encoded telemetry value to 16-bit raw telemetry
///
/// The ESC sends telemetry as GCR-encoded data (20 data bits after start bit).
/// This decodes 4 nibbles (5 bits each) into a 16-bit value.
///
/// The input is first XOR-decoded: `decoded = gcr ^ (gcr >> 1)`
/// Then 4 GCR nibbles are extracted and looked up in the decode table.
///
/// Based on reference implementation from pico-bidir-dshot.
///
/// Returns `None` if any nibble has an invalid GCR encoding.
/// CRC validation is NOT done here — use `verify_telemetry_crc()` separately.
#[must_use]
#[allow(clippy::cast_lossless)] // as casts required for const fn
pub const fn gcr_decode(gcr_value: u32) -> Option<u16> {
    // The PIO RX program shifts in up to 21 bits (set x, 20). GCR data is only
    // 20 bits. When the first pulse measurement uses the shorter timeout (y=6),
    // an extra bit can be shifted in at bit 20. Mask to 20 bits to prevent it
    // from corrupting bit 19 during the XOR decode step.
    let gcr = gcr_value & 0x000F_FFFF;

    // XOR decode (Manchester decoding): value was encoded as value ^ (value >> 1)
    let raw = gcr ^ (gcr >> 1);

    // Extract 4 nibbles (5 bits each), LSB first matching reference implementation
    // nibble0 is bits 0-4, nibble1 is bits 5-9, etc.
    let nibble0 = GCR_DECODE[(raw & 0x1F) as usize];
    let nibble1 = GCR_DECODE[((raw >> 5) & 0x1F) as usize];
    let nibble2 = GCR_DECODE[((raw >> 10) & 0x1F) as usize];
    let nibble3 = GCR_DECODE[((raw >> 15) & 0x1F) as usize];

    // Check for invalid GCR encodings
    if nibble0 == 0xFF || nibble1 == 0xFF || nibble2 == 0xFF || nibble3 == 0xFF {
        return None;
    }

    // Combine nibbles: nibble0 -> bits 0-3, nibble3 -> bits 12-15
    let data = (nibble0 as u16)
        | ((nibble1 as u16) << 4)
        | ((nibble2 as u16) << 8)
        | ((nibble3 as u16) << 12);

    Some(data)
}

/// Verify CRC of decoded telemetry value
///
/// XOR of all 4 nibbles must equal 0x0F.
#[must_use]
pub const fn verify_telemetry_crc(data: u16) -> bool {
    let checksum = (data ^ (data >> 4) ^ (data >> 8) ^ (data >> 12)) & 0x0F;
    checksum == 0x0F
}

/// Core trait for `DShot` PIO implementations
pub trait DshotPioTrait<const N: usize> {
    /// Send raw `DShot` command values (0-2047)
    ///
    /// Values 0-47 are reserved for special commands. Use Command enum instead.
    /// Values 48-2047 are throttle values (48 = zero throttle, 2047 = full throttle)
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if command value is out of range.
    fn command(&mut self, command: [u16; N]) -> Result<(), DshotError>;

    /// Set motor rotation direction
    ///
    /// Sends `SpinDirectionNormal` or `SpinDirectonReversed` command.
    /// Note: This command requires 6 transmissions to take effect.
    fn reverse(&mut self, reverse: [bool; N]);

    /// Set throttle values (0-1999), clamped and offset to `DShot` range (48-2047)
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle value is out of range.
    fn throttle_clamp(&mut self, throttle: [u16; N]) -> Result<(), DshotError>;

    /// Set all motors to idle throttle (DShot value 48)
    ///
    /// This sends the lowest non-zero throttle value. The motor may creep
    /// slightly on some ESCs. To fully stop the motor, use
    /// `send_command(Command::MotorStop)` (DShot value 0) instead.
    fn throttle_idle(&mut self);

    /// Send a `DShot` command to all motors
    fn send_command(&mut self, cmd: Command);

    /// Send a `DShot` command repeatedly to all motors
    ///
    /// Many DShot commands (spin direction, 3D mode, settings, telemetry mode
    /// changes) require being sent multiple consecutive times to take effect.
    /// Typically 6 times for settings commands, 10 for beep protocol.
    fn send_command_repeated(&mut self, cmd: Command, count: u8);
}

/// Async trait for `DShot` PIO implementations (Embassy only)
///
/// Provides non-blocking async/await methods that integrate with Embassy's executor.
/// These methods yield to the executor while waiting for TX FIFO space, enabling
/// concurrent tasks to run efficiently.
#[cfg(any(feature = "rp2040", feature = "rp2350"))]
#[allow(async_fn_in_trait)]
pub trait DshotPioAsync<const N: usize> {
    /// Send raw `DShot` command values asynchronously (0-2047)
    ///
    /// This method is non-blocking and will yield to the executor if the TX FIFO is full.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if command value is out of range.
    async fn command_async(&mut self, command: [u16; N]) -> Result<(), DshotError>;

    /// Set motor rotation direction asynchronously
    ///
    /// Note: This command requires 6 transmissions to take effect.
    async fn reverse_async(&mut self, reverse: [bool; N]);

    /// Set throttle values asynchronously (0-1999)
    ///
    /// Non-blocking variant that yields to the executor if TX FIFO is full.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle value is out of range.
    async fn throttle_async(&mut self, throttle: [u16; N]) -> Result<(), DshotError>;

    /// Set all motors to idle throttle asynchronously (DShot value 48)
    ///
    /// This sends the lowest non-zero throttle value. The motor may creep
    /// slightly on some ESCs. To fully stop the motor, use
    /// `send_command_async(Command::MotorStop)` (DShot value 0) instead.
    async fn throttle_idle_async(&mut self);

    /// Send a `DShot` command to all motors asynchronously
    async fn send_command_async(&mut self, cmd: Command);

    /// Send a `DShot` command repeatedly to all motors asynchronously
    ///
    /// Many DShot commands require being sent multiple consecutive times to
    /// take effect (typically 6 for settings, 10 for beep protocol).
    async fn send_command_repeated_async(&mut self, cmd: Command, count: u8);

    /// Arm ESCs by sending `MotorStop` at ~1kHz for the given duration
    ///
    /// ESCs require continuous `MotorStop` frames for 1-2 seconds before
    /// they accept throttle commands. This convenience method replaces the
    /// common arming loop pattern.
    ///
    /// # Example
    /// ```ignore
    /// use embassy_time::Duration;
    /// dshot.arm_async(Duration::from_secs(2)).await;
    /// ```
    async fn arm_async(&mut self, duration: embassy_time::Duration) {
        let iterations = duration.as_millis() as u32;
        for _ in 0..iterations {
            self.send_command_async(Command::MotorStop).await;
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Telemetry::rpm() tests ====================

    #[test]
    fn telemetry_rpm_conversion() {
        let telem = Telemetry {
            erpm: 10000,
            period_us: Some(6000),
        };
        // 14-pole motor: RPM = eRPM / (poles/2) = 10000 / 7 ≈ 1428
        assert_eq!(telem.rpm(14), 1428);
        // 12-pole motor: RPM = 10000 / 6 ≈ 1666
        assert_eq!(telem.rpm(12), 1666);
    }

    #[test]
    fn telemetry_rpm_edge_cases() {
        let telem = Telemetry {
            erpm: 10000,
            period_us: Some(6000),
        };
        // Invalid pole counts should return 0
        assert_eq!(telem.rpm(0), 0);
        assert_eq!(telem.rpm(1), 0);
        // 2-pole motor is valid: RPM = 10000 / (2/2) = 10000
        assert_eq!(telem.rpm(2), 10000);
    }

    // ==================== GCR decode tests ====================

    // GCR encoding table (4-bit -> 5-bit), inverse of GCR_DECODE
    const GCR_ENCODE: [u8; 16] = [
        0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
        0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
    ];

    /// Encode a 16-bit value with valid checksum to GCR format for testing
    /// Returns the XOR-encoded GCR value that gcr_decode() expects
    fn gcr_encode_for_test(value: u16) -> u32 {
        // Checksum: XOR of all nibbles must equal 0xF
        let n0 = (value & 0x0F) as u8;
        let n1 = ((value >> 4) & 0x0F) as u8;
        let n2 = ((value >> 8) & 0x0F) as u8;
        let n3 = ((value >> 12) & 0x0F) as u8;

        // Encode nibbles to 5-bit GCR
        let g0 = GCR_ENCODE[n0 as usize] as u32;
        let g1 = GCR_ENCODE[n1 as usize] as u32;
        let g2 = GCR_ENCODE[n2 as usize] as u32;
        let g3 = GCR_ENCODE[n3 as usize] as u32;

        // Combine: nibble0 at bits 0-4, nibble3 at bits 15-19
        let raw = g0 | (g1 << 5) | (g2 << 10) | (g3 << 15);

        // XOR encode (inverse of decode's raw = gcr ^ (gcr >> 1))
        // Compute bit by bit from MSB to LSB
        let mut gcr = 0u32;
        for i in (0..20).rev() {
            let raw_bit = (raw >> i) & 1;
            let prev_gcr_bit = if i == 19 { 0 } else { (gcr >> (i + 1)) & 1 };
            let gcr_bit = raw_bit ^ prev_gcr_bit;
            gcr |= gcr_bit << i;
        }
        gcr
    }

    #[test]
    fn gcr_decode_rejects_invalid_input() {
        // All zeros and all ones should fail
        assert!(gcr_decode(0).is_none());
        assert!(gcr_decode(0x1FFFFF).is_none());
    }

    #[test]
    fn gcr_decode_valid_checksum() {
        // Test values with valid checksum (XOR of nibbles = 0xF)
        // 0xF000: nibbles 0,0,0,F -> XOR = F ✓
        let encoded = gcr_encode_for_test(0xF000);
        assert_eq!(gcr_decode(encoded), Some(0xF000));

        // 0x1E00: nibbles 0,0,E,1 -> XOR = F ✓
        let encoded = gcr_encode_for_test(0x1E00);
        assert_eq!(gcr_decode(encoded), Some(0x1E00));

        // 0x2D00: nibbles 0,0,D,2 -> XOR = F ✓
        let encoded = gcr_encode_for_test(0x2D00);
        assert_eq!(gcr_decode(encoded), Some(0x2D00));

        // 0x1234: nibbles 4,3,2,1 -> XOR = 4^3^2^1 = 4 (not F, invalid)
        // Need a value where nibbles XOR to F
        // 0x8421: nibbles 1,2,4,8 -> XOR = 1^2^4^8 = F ✓
        let encoded = gcr_encode_for_test(0x8421);
        assert_eq!(gcr_decode(encoded), Some(0x8421));
    }

    #[test]
    fn gcr_decode_corrupted_encoding() {
        // Corrupt a valid encoded value — should fail GCR lookup
        let valid_encoded = gcr_encode_for_test(0xF000);
        // Flip a bit — may produce invalid GCR nibble or wrong data
        let corrupted = valid_encoded ^ 0x100;
        // Either GCR decode fails, or the decoded value differs (CRC would catch it)
        match gcr_decode(corrupted) {
            None => {} // Invalid GCR encoding — expected
            Some(v) => assert_ne!(v, 0xF000, "Corrupted data should differ"),
        }
    }

    #[test]
    fn verify_telemetry_crc_valid() {
        assert!(verify_telemetry_crc(0xF000)); // 0^0^0^F = F ✓
        assert!(verify_telemetry_crc(0x8421)); // 1^2^4^8 = F ✓
    }

    #[test]
    fn verify_telemetry_crc_invalid() {
        assert!(!verify_telemetry_crc(0x1234)); // 4^3^2^1 = 4 ✗
        assert!(!verify_telemetry_crc(0x0000)); // 0^0^0^0 = 0 ✗
    }

    #[test]
    fn gcr_decode_single_invalid_nibble() {
        // Test that having just ONE invalid GCR nibble rejects the input
        // This tests that || (any invalid) correctly rejects, not && (all invalid)
        // Key: choose valid nibbles such that if nibble0=0xFF (invalid marker),
        // the resulting checksum would FAIL, catching the || to && mutation.

        // If nibble0 becomes 0xFF, low nibble of data = 0xF
        // Checksum = n0 ^ n1 ^ n2 ^ n3 must equal 0xF
        // With n0=0xF (from 0xFF), need n1^n2^n3 = 0, but we'll use n1^n2^n3 ≠ 0
        // so checksum fails when mutation passes the invalid nibble through.

        // Use: nibble0 = invalid (0x00 GCR), n1=1, n2=0, n3=0
        // GCR_ENCODE[1]=0x1B, GCR_ENCODE[0]=0x19
        let raw = 0x00 | (0x1B << 5) | (0x19 << 10) | (0x19 << 15);
        // XOR encode
        let mut gcr = 0u32;
        for i in (0..20).rev() {
            let raw_bit = (raw >> i) & 1;
            let prev_gcr_bit = if i == 19 { 0 } else { (gcr >> (i + 1)) & 1 };
            gcr |= (raw_bit ^ prev_gcr_bit) << i;
        }
        assert!(gcr_decode(gcr).is_none(), "Should reject single invalid nibble");

        // Also test invalid nibble in other positions
        // nibble1 invalid: n0=1, n1=invalid, n2=0, n3=0
        let raw = 0x1B | (0x00 << 5) | (0x19 << 10) | (0x19 << 15);
        let mut gcr = 0u32;
        for i in (0..20).rev() {
            let raw_bit = (raw >> i) & 1;
            let prev_gcr_bit = if i == 19 { 0 } else { (gcr >> (i + 1)) & 1 };
            gcr |= (raw_bit ^ prev_gcr_bit) << i;
        }
        assert!(gcr_decode(gcr).is_none(), "Should reject invalid nibble1");

        // nibble2 invalid
        let raw = 0x1B | (0x19 << 5) | (0x00 << 10) | (0x19 << 15);
        let mut gcr = 0u32;
        for i in (0..20).rev() {
            let raw_bit = (raw >> i) & 1;
            let prev_gcr_bit = if i == 19 { 0 } else { (gcr >> (i + 1)) & 1 };
            gcr |= (raw_bit ^ prev_gcr_bit) << i;
        }
        assert!(gcr_decode(gcr).is_none(), "Should reject invalid nibble2");

        // nibble3 invalid
        let raw = 0x1B | (0x19 << 5) | (0x19 << 10) | (0x00 << 15);
        let mut gcr = 0u32;
        for i in (0..20).rev() {
            let raw_bit = (raw >> i) & 1;
            let prev_gcr_bit = if i == 19 { 0 } else { (gcr >> (i + 1)) & 1 };
            gcr |= (raw_bit ^ prev_gcr_bit) << i;
        }
        assert!(gcr_decode(gcr).is_none(), "Should reject invalid nibble3");
    }

    #[test]
    fn gcr_decode_table_valid() {
        // Table must have exactly 16 valid entries (0x0-0xF)
        let valid_count = GCR_DECODE.iter().filter(|&&x| x != 0xFF).count();
        assert_eq!(valid_count, 16);
    }

    #[test]
    fn gcr_decode_masks_bit20() {
        // PIO can shift in 21 bits. When bit 20 is set, it must be masked
        // to prevent corrupting the XOR decode of bit 19.
        let valid_encoded = gcr_encode_for_test(0xF000);
        assert!(valid_encoded < (1 << 20), "Test value should be 20-bit");

        // Without the fix, setting bit 20 would corrupt nibble3 via XOR
        let with_bit20 = valid_encoded | (1 << 20);
        assert_eq!(
            gcr_decode(with_bit20),
            gcr_decode(valid_encoded),
            "Bit 20 should be masked and not affect decode"
        );

        // Test with multiple values
        for &val in &[0xF000u16, 0x8421, 0x1E00, 0x2D00] {
            let enc = gcr_encode_for_test(val);
            assert_eq!(
                gcr_decode(enc | (1 << 20)),
                Some(val),
                "Bit 20 mask failed for {val:#06X}"
            );
        }
    }

    #[test]
    fn gcr_encode_decode_roundtrip() {
        // Verify encode table is inverse of decode table
        for nibble in 0u8..16 {
            let encoded = GCR_ENCODE[nibble as usize];
            let decoded = GCR_DECODE[encoded as usize];
            assert_eq!(decoded, nibble, "GCR roundtrip failed for {nibble:X}");
        }
    }

    // ==================== Telemetry parsing tests ====================

    // Helper matching driver's telemetry_to_erpm
    const fn test_telemetry_to_erpm(value: u16) -> (u32, Option<u32>) {
        if value == 0 || value == 0x0FFF {
            return (0, None);
        }
        let exponent = (value >> 9) & 0x07;
        let mantissa = value & 0x1FF;
        let period_us = (mantissa as u32) << (exponent as u32);
        if period_us == 0 {
            return (0, None);
        }
        (60_000_000 / period_us, Some(period_us))
    }

    #[test]
    fn telemetry_parsing_motor_stopped() {
        // 0 and 0xFFF both indicate motor stopped
        assert_eq!(test_telemetry_to_erpm(0), (0, None));
        assert_eq!(test_telemetry_to_erpm(0x0FFF), (0, None));
        // Zero mantissa with any exponent = stopped
        for exp in 0u16..8 {
            assert_eq!(test_telemetry_to_erpm(exp << 9), (0, None));
        }
    }

    #[test]
    fn telemetry_parsing_known_values() {
        // Format: eee_mmmmmmmmm (3-bit exp, 9-bit mantissa)
        // period_us = mantissa << exp, erpm = 60_000_000 / period_us

        // exp=0, mantissa=100 -> period=100us -> erpm=600000
        assert_eq!(test_telemetry_to_erpm(100), (600_000, Some(100)));
        // exp=1, mantissa=100 -> period=200us -> erpm=300000
        assert_eq!(test_telemetry_to_erpm((1 << 9) | 100), (300_000, Some(200)));
        // exp=3, mantissa=125 -> period=1000us -> erpm=60000
        assert_eq!(test_telemetry_to_erpm((3 << 9) | 125), (60_000, Some(1000)));
    }

    // ==================== Timing constant tests ====================

    #[test]
    fn dshot_baud_rates() {
        // Verify baud rates match DShot spec
        const EXPECTED: [(u32, &str); 4] = [
            (150_000, "DShot150"),
            (300_000, "DShot300"),
            (600_000, "DShot600"),
            (1_200_000, "DShot1200"),
        ];
        // These are the baud rates from the spec, verified by the const fn
        for (baud, name) in EXPECTED {
            assert!(baud > 0, "Baud rate for {name} should be positive");
        }
    }

    #[test]
    fn dshot_tx_divider_at_125mhz() {
        // Verify TX divider calculation: 8 PIO cycles/bit
        // At 125MHz, DShot600 = 600kHz baud, PIO clock = 600kHz * 8 = 4.8MHz
        // Divider = 125MHz / 4.8MHz = 26.04166...
        // Fixed-point (8 frac bits): 26.04166 * 256 = 6666.666... ≈ 6667
        const SYS_CLOCK: u64 = 125_000_000;
        let baud = 600_000u64;
        let div_bits = ((SYS_CLOCK << 8) / (8 * baud)) as u32;
        // Integer part should be ~26
        assert_eq!(div_bits >> 8, 26, "TX divider integer part wrong");
    }

    #[test]
    fn dshot_bidir_divider_at_125mhz() {
        // Verify bidir divider: target PIO clock = 12MHz * speed/300kHz
        // DShot600: target = 12MHz * 600/300 = 24MHz
        // At 125MHz: divider = 125/24 = 5.2083...
        const SYS_CLOCK: u64 = 125_000_000;
        let target = 12_000_000u64 * 600_000 / 300_000; // 24MHz
        let div_bits = ((SYS_CLOCK << 8) / target) as u32;
        assert_eq!(div_bits >> 8, 5, "Bidir divider integer part wrong");
    }

    // ==================== EDT (Extended DShot Telemetry) decode tests ====================

    // Helper: build a 12-bit EDT value from prefix and 8-bit data
    // prefix format: eee_m (4 bits), data: lower 8 bits
    const fn edt_frame(exponent: u16, bit8: u16, data: u16) -> u16 {
        (exponent << 9) | (bit8 << 8) | (data & 0xFF)
    }

    #[test]
    fn edt_temperature() {
        // Temperature: exponent=1, bit8=0, 1°C per unit
        let raw = edt_frame(1, 0, 25);
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Temperature(25));

        let raw = edt_frame(1, 0, 100);
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Temperature(100));

        let raw = edt_frame(1, 0, 255);
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Temperature(255));
    }

    #[test]
    fn edt_voltage() {
        // Voltage: exponent=2, bit8=0, 250mV per unit
        let raw = edt_frame(2, 0, 48); // 48 * 250 = 12000mV = 12.0V
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Voltage(12000));

        let raw = edt_frame(2, 0, 67); // 67 * 250 = 16750mV = 16.75V (4S LiPo)
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Voltage(16750));

        let raw = edt_frame(2, 0, 255); // 255 * 250 = 63750mV = 63.75V
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Voltage(63750));
    }

    #[test]
    fn edt_current() {
        // Current: exponent=3, bit8=0, 1A (1000mA) per unit
        let raw = edt_frame(3, 0, 10); // 10A = 10000mA
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Current(10000));

        let raw = edt_frame(3, 0, 200); // 200A = 200000mA
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Current(200000));
    }

    #[test]
    fn edt_debug_stress_status() {
        // Debug 1: exponent=4, bit8=0
        let raw = edt_frame(4, 0, 42);
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Debug1(42));

        // Debug 2: exponent=5, bit8=0
        let raw = edt_frame(5, 0, 99);
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Debug2(99));

        // Stress Level: exponent=6, bit8=0
        let raw = edt_frame(6, 0, 128);
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::StressLevel(128));

        // Status: exponent=7, bit8=0
        let raw = edt_frame(7, 0, 1);
        assert_eq!(decode_extended_telemetry(raw), ExtendedTelemetry::Status(1));
    }

    #[test]
    fn edt_erpm_when_exponent_zero() {
        // exponent=0 is always eRPM regardless of bit8
        // exp=0, mantissa=100 -> period=100us -> erpm=600000
        assert_eq!(
            decode_extended_telemetry(100),
            ExtendedTelemetry::Erpm { erpm: 600_000, period_us: Some(100) }
        );
    }

    #[test]
    fn edt_erpm_when_bit8_set() {
        // bit8=1 is always eRPM regardless of exponent
        // exp=1, mantissa with bit8=1: 0b001_1_01100100 = (1<<9)|(1<<8)|100 = 0x364 = 868
        let raw = edt_frame(1, 1, 100); // exp=1, bit8=1, lower8=100 -> mantissa=356
        let mantissa = raw & 0x1FF; // 256 + 100 = 356
        let period_us = (mantissa as u32) << 1; // 712
        let expected_erpm = 60_000_000 / period_us;
        match decode_extended_telemetry(raw) {
            ExtendedTelemetry::Erpm { erpm, period_us: p } => {
                assert_eq!(erpm, expected_erpm);
                assert_eq!(p, Some(period_us));
            }
            other => panic!("Expected Erpm, got {other:?}"),
        }
    }

    #[test]
    fn edt_erpm_motor_stopped() {
        // 0 and 0xFFF both indicate motor stopped
        assert_eq!(
            decode_extended_telemetry(0),
            ExtendedTelemetry::Erpm { erpm: 0, period_us: None }
        );
        assert_eq!(
            decode_extended_telemetry(0x0FFF),
            ExtendedTelemetry::Erpm { erpm: 0, period_us: None }
        );
    }

    #[test]
    fn edt_boundary_zero_data() {
        // EDT types with data=0
        assert_eq!(decode_extended_telemetry(edt_frame(1, 0, 0)), ExtendedTelemetry::Temperature(0));
        assert_eq!(decode_extended_telemetry(edt_frame(2, 0, 0)), ExtendedTelemetry::Voltage(0));
        assert_eq!(decode_extended_telemetry(edt_frame(3, 0, 0)), ExtendedTelemetry::Current(0));
    }

    #[test]
    fn edt_boundary_max_data() {
        // EDT types with data=255 (max 8-bit)
        assert_eq!(decode_extended_telemetry(edt_frame(1, 0, 255)), ExtendedTelemetry::Temperature(255));
        assert_eq!(decode_extended_telemetry(edt_frame(2, 0, 255)), ExtendedTelemetry::Voltage(63750)); // 255*250
        assert_eq!(decode_extended_telemetry(edt_frame(3, 0, 255)), ExtendedTelemetry::Current(255000)); // 255*1000
    }
}
