use crate::{DshotError, Command};

pub trait DshotPioTrait<const N: usize> {
    /// Send raw `DShot` command values (0-2047).
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if a command value is out of range.
    fn command(&mut self, command: [u16; N]) -> Result<(), DshotError>;

    /// Set motor rotation direction (requires 6 transmissions to take effect).
    fn reverse(&mut self, reverse: [bool; N]);

    /// Set throttle values (0-1999), clamped to `DShot` range (48-2047).
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if a throttle value is invalid.
    fn throttle_clamp(&mut self, throttle: [u16; N]) -> Result<(), DshotError>;

    /// Set all motors to idle throttle (`DShot` value 48).
    ///
    /// This sends the lowest non-zero throttle value. The motor may creep
    /// slightly on some ESCs. To fully stop the motor, use
    /// `send_command(Command::MotorStop)` (`DShot` value 0) instead.
    fn throttle_idle(&mut self);

    /// Send a `DShot` command to all motors
    fn send_command(&mut self, cmd: Command);

    /// Send a `DShot` command repeatedly (6x for settings, 10x for beep).
    fn send_command_repeated(&mut self, cmd: Command, count: u8);
}

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
#[allow(async_fn_in_trait)]
pub trait DshotPioAsync<const N: usize> {
    async fn command_async(&mut self, command: [u16; N]) -> Result<(), DshotError>;
    async fn reverse_async(&mut self, reverse: [bool; N]);
    async fn throttle_async(&mut self, throttle: [u16; N]) -> Result<(), DshotError>;
    async fn throttle_idle_async(&mut self);
    async fn send_command_async(&mut self, cmd: Command);
    async fn send_command_repeated_async(&mut self, cmd: Command, count: u8);

    /// Arm ESC by sending `MotorStop` at ~1kHz for the given duration.
    async fn arm_async(&mut self, duration: embassy_time::Duration) {
        #[allow(clippy::cast_possible_truncation)]
        let iterations = duration.as_millis() as u32;
        for _ in 0..iterations {
            self.send_command_async(Command::MotorStop).await;
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
    }
}
