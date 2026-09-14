#![no_std]

#[cfg(any(test, feature = "std"))]
extern crate std;

mod dshot_speed;
mod telemetry;
mod traits;
mod types;

pub use dshot_frame::Command;
pub use dshot_speed::DshotSpeed;
pub use telemetry::{
    decode_extended_telemetry, gcr_decode, verify_telemetry_crc, ExtendedTelemetry,
};
pub use traits::DshotPioTrait;
pub use types::{DshotError, Telemetry};

#[cfg(feature = "_rp")]
pub use traits::DshotPioAsync;

#[cfg(feature = "_rp")]
pub mod rp;
