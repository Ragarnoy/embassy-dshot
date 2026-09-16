# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.5.0] - unreleased

### Changed

- **Breaking:** `BidirDshotPio` no longer owns a whole PIO block. The PIO program is
  now loaded once per block via `BidirDshotProgram::new(&mut common)` and shared by up
  to four `BidirDshotPio` drivers, each taking its own state machine and pin. The
  program is 28 instructions against a 32-instruction block, so one copy per motor
  failed at the second motor.

  ```rust,ignore
  // 0.4
  let mut dshot = BidirDshotPio::new(p.PIO0, Irqs, p.PIN_11, DshotSpeed::DShot300);

  // 0.5
  let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
  let prog = BidirDshotProgram::new(&mut common);
  let mut dshot = BidirDshotPio::new(sm0, &mut common, p.PIN_11, &prog, DshotSpeed::DShot300);
  ```

  The type gained a state-machine index parameter, `BidirDshotPio<'a, PIO, const SM: usize>`,
  so drivers for different state machines are different types and cannot share an array.

### Added

- `BidirDshotProgram`, the shared per-PIO-block program handle
- `quad_engine` example — four bidirectional ESCs on one PIO block, with their
  telemetry reads overlapped via `join4` rather than awaited in turn

### Notes

- Multi-ESC bidirectional is built and CI-checked on every supported chip but has not
  been validated against four ESCs on hardware; single-ESC bidirectional remains the
  tested path

## [0.4.0] - 2026-09-14

### Added

- `rp235xa` and `rp235xb` features — RP235xB boards are now supported at all, where
  previously the `rp2350` feature hardcoded `embassy-rp/rp235xa`
- CI: host test powerset, a per-chip check matrix covering every supported target,
  clippy/rustfmt gates, and example builds (`cargo hack`)

### Changed

- `DshotSpeed` moved to its own chip-independent module; still re-exported from `rp`,
  so `embassy_dshot::rp::DshotSpeed` and `embassy_dshot::DshotSpeed` both work
- PIO clock dividers are now free functions taking the system clock as an argument
  instead of methods reading it internally — private API, no downstream impact

### Deprecated

- `rp2350`, now an alias for `rp235xa`; it will be removed in 0.5.0

### Fixed

- Bidirectional PIO TX phase is documented as 40 cycles per bit, not 32 — the program
  has always been 40, matching the clock divider
- Three tests restated constants instead of calling the code they claimed to cover:
  both clock-divider tests recomputed the arithmetic inline, and `dshot_baud_rates`
  never called `DshotSpeed::baud_rate()`. All now exercise the real functions.

## [0.3.0] - 2026-08-27

### Changed

- Upgraded to `embassy-rp` 0.10 and `embassy-time` 0.5.1 — breaking for downstreams still on `embassy-rp` 0.9
- Raised minimum `defmt` to 1.1 and `fixed` to 1.31
- Applied `rustfmt` across the source tree

### Added

- `rpm_range` example: EDT throttle sweep that reports the motor's sustainable RPM range

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
