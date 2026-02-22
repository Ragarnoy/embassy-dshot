#![no_std]

#[cfg(any(test, feature = "std"))]
extern crate std;

mod types;
mod telemetry;
mod traits;

pub use dshot_frame::Command;
pub use types::{DshotError, Telemetry};
pub use telemetry::{ExtendedTelemetry, decode_extended_telemetry, gcr_decode, verify_telemetry_crc};
pub use traits::DshotPioTrait;

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub use traits::DshotPioAsync;

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub mod dshot_embassy_rp;

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub use dshot_embassy_rp::DshotSpeed;
