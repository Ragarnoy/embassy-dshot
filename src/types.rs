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
    /// Convert eRPM to mechanical RPM: `eRPM / (motor_poles / 2)`.
    #[must_use]
    #[allow(clippy::cast_lossless)]
    pub const fn rpm(&self, motor_poles: u8) -> u32 {
        if motor_poles < 2 {
            return 0;
        }
        self.erpm / (motor_poles as u32 / 2)
    }
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
}
