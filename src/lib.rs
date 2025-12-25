#![no_std]

// Re-export commonly used types from dshot-frame
pub use dshot_frame::{Command, ErpmTelemetry, Frame, NormalDshot, BidirectionalDshot};

#[cfg(feature = "embassy-rp")]
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
#[cfg(feature = "embassy-rp")]
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