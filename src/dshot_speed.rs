//! `DShot` protocol speeds.
//!
//! Deliberately free of any chip or HAL dependency: the PIO clock dividers that
//! consume this live next to their respective drivers, so that adding a non-RP
//! backend does not require touching this module.

/// `DShot` protocol speed (bit rate).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotSpeed {
    DShot150,
    DShot300,
    DShot600,
    DShot1200,
}

impl DshotSpeed {
    /// Protocol bit rate in bits per second.
    #[must_use]
    pub const fn baud_rate(self) -> u32 {
        match self {
            Self::DShot150 => 150_000,
            Self::DShot300 => 300_000,
            Self::DShot600 => 600_000,
            Self::DShot1200 => 1_200_000,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::DshotSpeed;

    #[test]
    fn baud_rates_match_spec() {
        assert_eq!(DshotSpeed::DShot150.baud_rate(), 150_000);
        assert_eq!(DshotSpeed::DShot300.baud_rate(), 300_000);
        assert_eq!(DshotSpeed::DShot600.baud_rate(), 600_000);
        assert_eq!(DshotSpeed::DShot1200.baud_rate(), 1_200_000);
    }

    #[test]
    fn baud_rate_matches_variant_name() {
        // Each variant is named for its bit rate in kbit/s.
        for (speed, khz) in [
            (DshotSpeed::DShot150, 150),
            (DshotSpeed::DShot300, 300),
            (DshotSpeed::DShot600, 600),
            (DshotSpeed::DShot1200, 1200),
        ] {
            assert_eq!(speed.baud_rate(), khz * 1000);
        }
    }
}
