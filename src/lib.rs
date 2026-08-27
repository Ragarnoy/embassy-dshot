#![no_std]

#[cfg(any(test, feature = "std"))]
extern crate std;

mod telemetry;
mod traits;
mod types;

pub use dshot_frame::Command;
pub use telemetry::{
    decode_extended_telemetry, gcr_decode, verify_telemetry_crc, ExtendedTelemetry,
};
pub use traits::DshotPioTrait;
pub use types::{DshotError, Telemetry};

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub use traits::DshotPioAsync;

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub mod rp;

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub use rp::DshotSpeed;
