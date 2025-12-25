#![no_std]

#[cfg(any(test, feature = "std"))]
extern crate std;

// Re-export commonly used types from dshot-frame
pub use dshot_frame::{Command, ErpmTelemetry, Frame, NormalDshot, BidirectionalDshot};

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub mod dshot_embassy_rp;

#[cfg(feature = "rp2040-hal")]
pub mod dshot_rp2040_hal;

/// Error types for DShot operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DshotError {
    /// Throttle value out of range (must be 0-1999)
    InvalidThrottle,
    /// Telemetry CRC checksum mismatch
    InvalidTelemetryCrc,
    /// No telemetry data available
    NoTelemetryData,
}

/// Telemetry data from ESC
#[derive(Debug, Clone, Copy)]
pub struct Telemetry {
    /// Electrical RPM (0 if motor stopped)
    pub erpm: u32,
    /// Period in microseconds (None if motor stopped)
    pub period_us: Option<u32>,
}

impl Telemetry {
    /// Convert eRPM to mechanical RPM
    ///
    /// Formula: RPM = eRPM / (motor_poles / 2)
    ///
    /// # Example
    /// ```ignore
    /// let telemetry = Telemetry { erpm: 10000, period_us: Some(6000) };
    /// let rpm = telemetry.rpm(14); // 14-pole motor
    /// assert_eq!(rpm, 1428); // 10000 / 7
    /// ```
    pub fn rpm(&self, motor_poles: u8) -> u32 {
        if motor_poles < 2 {
            return 0;
        }
        self.erpm / (motor_poles as u32 / 2)
    }
}

/// Core trait for DShot PIO implementations
pub trait DshotPioTrait<const N: usize> {
    /// Send raw DShot command values (0-2047)
    ///
    /// Values 0-47 are reserved for special commands. Use Command enum instead.
    /// Values 48-2047 are throttle values (48 = zero throttle, 2047 = full throttle)
    fn command(&mut self, command: [u16; N]) -> Result<(), DshotError>;

    /// Set motor rotation direction
    ///
    /// Sends SpinDirectionNormal or SpinDirectonReversed command.
    /// Note: This command requires 6 transmissions to take effect.
    fn reverse(&mut self, reverse: [bool; N]);

    /// Set throttle values (0-1999), clamped and offset to DShot range (48-2047)
    fn throttle_clamp(&mut self, throttle: [u16; N]) -> Result<(), DshotError>;

    /// Set all motors to minimum throttle (zero)
    fn throttle_minimum(&mut self);

    /// Send a DShot command to all motors
    fn send_command(&mut self, cmd: Command);
}

/// Async trait for DShot PIO implementations (Embassy only)
///
/// Provides non-blocking async/await methods that integrate with Embassy's executor.
/// These methods yield to the executor while waiting for TX FIFO space, enabling
/// concurrent tasks to run efficiently.
#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub trait DshotPioAsync<const N: usize> {
    /// Send raw DShot command values asynchronously (0-2047)
    ///
    /// This method is non-blocking and will yield to the executor if the TX FIFO is full.
    async fn command_async(&mut self, command: [u16; N]) -> Result<(), DshotError>;

    /// Set motor rotation direction asynchronously
    ///
    /// Note: This command requires 6 transmissions to take effect.
    async fn reverse_async(&mut self, reverse: [bool; N]);

    /// Set throttle values asynchronously (0-1999)
    ///
    /// Non-blocking variant that yields to the executor if TX FIFO is full.
    async fn throttle_async(&mut self, throttle: [u16; N]) -> Result<(), DshotError>;

    /// Set all motors to minimum throttle asynchronously
    async fn throttle_minimum_async(&mut self);

    /// Send a DShot command to all motors asynchronously
    async fn send_command_async(&mut self, cmd: Command);
}
#[cfg(test)]
mod tests {
    use super::*;

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
    fn telemetry_rpm_zero_poles() {
        let telem = Telemetry {
            erpm: 10000,
            period_us: Some(6000),
        };
        
        // Invalid pole count should return 0
        assert_eq!(telem.rpm(0), 0);
        assert_eq!(telem.rpm(1), 0);
    }

    #[test]
    fn dshot_error_equality() {
        assert_eq!(DshotError::InvalidThrottle, DshotError::InvalidThrottle);
        assert_ne!(DshotError::InvalidThrottle, DshotError::InvalidTelemetryCrc);
    }

    #[test]
    fn frame_creation_valid() {
        // Test valid throttle values
        assert!(Frame::<NormalDshot>::new(0, false).is_some());
        assert!(Frame::<NormalDshot>::new(999, false).is_some());
        assert!(Frame::<NormalDshot>::new(1999, false).is_some());
    }

    #[test]
    fn frame_creation_invalid() {
        // Out of range should fail
        assert!(Frame::<NormalDshot>::new(2000, false).is_none());
        assert!(Frame::<NormalDshot>::new(3000, false).is_none());
    }

    #[test]
    fn frame_telemetry_flag() {
        let frame_no_telem = Frame::<NormalDshot>::new(1000, false).unwrap();
        let frame_with_telem = Frame::<NormalDshot>::new(1000, true).unwrap();
        
        assert!(!frame_no_telem.telemetry_enabled());
        assert!(frame_with_telem.telemetry_enabled());
    }

    #[test]
    fn frame_speed_roundtrip() {
        for speed in [0, 100, 500, 999, 1500, 1999] {
            let frame = Frame::<NormalDshot>::new(speed, false).unwrap();
            assert_eq!(frame.speed(), speed);
        }
    }

    #[test]
    fn command_enum_values() {
        // Verify some key command values
        assert_eq!(Command::MotorStop as u16, 0);
        assert_eq!(Command::SpinDirectionNormal as u16, 20);
    }

    #[test]
    fn bidir_frame_creation() {
        let frame = Frame::<BidirectionalDshot>::new(1000, true).unwrap();
        assert_eq!(frame.speed(), 1000);
        assert!(frame.telemetry_enabled());
    }
}
