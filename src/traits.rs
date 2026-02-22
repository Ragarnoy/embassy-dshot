use crate::{DshotError, Command};

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
