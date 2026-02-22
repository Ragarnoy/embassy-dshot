mod unidirectional;
mod bidirectional;

pub use unidirectional::DshotPio;
pub use bidirectional::BidirDshotPio;

use crate::{DshotError, Command};
use dshot_frame::{Frame, NormalDshot};
use embassy_rp::clocks::clk_sys_freq;
use fixed::types::extra::U8;
use fixed::FixedU32;

/// Idle throttle value for `Frame::new()` — produces DShot protocol value 48
/// (the lowest non-command throttle value).
///
/// **Note:** This is NOT the same as `Command::MotorStop` (DShot value 0).
/// Some ESCs will creep slightly at this throttle. To fully stop the motor
/// and keep it stopped, use `send_command(Command::MotorStop)` instead.
///
/// `Frame::new()` maps speed 0-1999 to DShot values 48-2047, so idle
/// speed is 0, which corresponds to DShot protocol value 48.
pub const THROTTLE_IDLE: u16 = 0;

/// Convert 12-bit telemetry value to eRPM
/// Format: eeem mmmm mmmm (3-bit exponent, 9-bit mantissa)
/// eRPM = mantissa << exponent
/// Period (us) = 60_000_000 / eRPM (if eRPM > 0)
#[allow(clippy::cast_lossless)] // as casts required for const fn
const fn telemetry_to_erpm(value: u16) -> (u32, Option<u32>) {
    if value == 0 || value == 0x0FFF {
        return (0, None);
    }
    let exponent = (value >> 9) & 0x07;
    let mantissa = value & 0x1FF;
    let period_us = (mantissa as u32) << (exponent as u32);
    if period_us == 0 {
        return (0, None);
    }
    let erpm = 60_000_000 / period_us;
    (erpm, Some(period_us))
}

/// DShot speed variants
#[derive(Clone, Copy, Debug)]
pub enum DshotSpeed {
    DShot150,
    DShot300,
    DShot600,
    DShot1200,
}

impl DshotSpeed {
    /// Get the baud rate in bits per second
    pub const fn baud_rate(self) -> u32 {
        match self {
            Self::DShot150 => 150_000,
            Self::DShot300 => 300_000,
            Self::DShot600 => 600_000,
            Self::DShot1200 => 1_200_000,
        }
    }

    /// TX PIO clock divider (8 PIO cycles per bit)
    fn tx_pio_clock_divider(self) -> FixedU32<U8> {
        let sys_clock = clk_sys_freq() as u64;
        FixedU32::<U8>::from_bits(((sys_clock << 8) / (8 * self.baud_rate() as u64)) as u32)
    }

    /// Bidir PIO clock divider
    /// Target PIO clock = 12MHz * speed/300kHz (e.g. DShot600 = 24MHz)
    /// 32 PIO cycles per bit in the bidir TX phase
    fn bidir_pio_clock_divider(self) -> FixedU32<U8> {
        let sys_clock = clk_sys_freq() as u64;
        let target = 12_000_000u64 * self.baud_rate() as u64 / 300_000;
        FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
    }
}

/// Convert raw command value (0-47) to Command enum safely
pub fn raw_to_command(cmd: u16) -> Option<Command> {
    match cmd {
        0 => Some(Command::MotorStop),
        1 => Some(Command::Beep1),
        2 => Some(Command::Beep2),
        3 => Some(Command::Beep3),
        4 => Some(Command::Beep4),
        5 => Some(Command::Beep5),
        6 => Some(Command::ESCInfo),
        7 => Some(Command::SpinDirection1),
        8 => Some(Command::SpinDirection2),
        9 => Some(Command::ThreeDModeOn),
        10 => Some(Command::ThreeDModeOff),
        11 => Some(Command::SettingsRequest),
        12 => Some(Command::SettingsSave),
        13 => Some(Command::ExtendedTelemetryEnable),
        14 => Some(Command::ExtendedTelemetryDisable),
        20 => Some(Command::SpinDirectionNormal),
        21 => Some(Command::SpinDirectonReversed),
        22 => Some(Command::Led0On),
        23 => Some(Command::Led1On),
        24 => Some(Command::Led2On),
        25 => Some(Command::Led3On),
        26 => Some(Command::Led0Off),
        27 => Some(Command::Led1Off),
        28 => Some(Command::Led2Off),
        29 => Some(Command::Led3Off),
        30 => Some(Command::AudioStreamModeToggle),
        31 => Some(Command::SilentModeToggle),
        32 => Some(Command::SignalLineTelemetryEnable),
        33 => Some(Command::SignalLineTelemetryDisable),
        34 => Some(Command::SignalLineContinuousERPMTelemetry),
        35 => Some(Command::SignalLineContinuousERPMPeriodTelemetry),
        42 => Some(Command::SignalLineTemperatureTelemetry),
        43 => Some(Command::SignalLineVoltageTelemetry),
        44 => Some(Command::SignalLineCurrentTelemetry),
        45 => Some(Command::SignalLineConsumptionTelemetry),
        46 => Some(Command::SignalLineERPMTelemetry),
        47 => Some(Command::SignalLineERPMPeriodTelemetry),
        _ => None, // 15-19, 36-41 are unassigned
    }
}

/// Build a DShot frame from a raw command value (0-2047)
fn make_command_frame(cmd: u16) -> Result<Frame<NormalDshot>, DshotError> {
    if cmd < 48 {
        let command = raw_to_command(cmd).ok_or(DshotError::InvalidThrottle)?;
        Ok(Frame::<NormalDshot>::command(command, false))
    } else {
        Frame::<NormalDshot>::new(cmd.saturating_sub(48), false)
            .ok_or(DshotError::InvalidThrottle)
    }
}
