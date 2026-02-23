# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.1] - 2026-02-23

### Changed

- Renamed `dshot_embassy_rp` module to `rp` — imports are now `embassy_dshot::rp::{DshotPio, BidirDshotPio, DshotSpeed}`
- Removed library `Cargo.lock` from version control

## [0.2.0] - 2026-02-22

### Added

- Bidirectional DShot with GCR-encoded eRPM telemetry
- Extended DShot Telemetry (EDT) decoding
- `BidirDshotPio` driver for single-ESC bidirectional communication
- `Telemetry` and `ExtendedTelemetry` types
- `arm_async()` method for ESC arming sequences
- Twin engine example
- `DshotPioTrait` and `DshotPioAsync` traits for generic motor control

### Changed

- Split monolithic source into focused modules (`unidirectional`, `bidirectional`, `telemetry`, `types`, `traits`)
- Consolidated examples: merged `motor_test` into `single_esc`, `speed_test` into `bdshot_test`

## [0.1.0] - 2025-12-26

Initial release as `embassy-dshot`, a fork of [peterkrull/dshot-pio](https://github.com/peterkrull/dshot-pio).

### Added

- Async/await API via `DshotPioAsync` trait
- RP2350 support with `rp2350` feature
- Type-safe frames using `dshot-frame` crate
- Unit tests with `std` feature

### Changed

- Renamed crate from `dshot-pio` to `embassy-dshot`
- Upgraded to `embassy-rp` 0.9.0
- Migrated from `dshot-encoder` to `dshot-frame`
- Features renamed: `embassy-rp` -> `rp2040`, `embassy-rp2350` -> `rp2350`

### Removed

- `rp2040-hal` backend (now embassy-rp only)
