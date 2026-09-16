# embassy-dshot

DShot ESC protocol driver for embassy-rp (RP2040/RP235xA/RP235xB).

Uses PIO to control up to 4 ESCs per PIO block. Supports both unidirectional and bidirectional DShot with GCR-encoded eRPM telemetry.

## Features

- Async/await with Embassy
- RP2040, RP235xA and RP235xB support
- 1-4 motors per PIO block, unidirectional or bidirectional
- Bidirectional DShot with eRPM telemetry
- Automatic clock divider calculation from system clock
- Type-safe frames via [dshot-frame](https://github.com/sulami/dshot-frame)
- `no_std`

## Usage

```toml
# For RP2040
embassy-dshot = { version = "0.5", features = ["rp2040"] }

# For RP2350 — pick the variant matching your chip package
embassy-dshot = { version = "0.5", features = ["rp235xa"] }
embassy-dshot = { version = "0.5", features = ["rp235xb"] }
```

The `rp2350` feature still works as an alias for `rp235xa`, but is deprecated and
will be removed in 0.5.

## Example

### Single ESC

```rust,ignore
use embassy_dshot::rp::{DshotPio, DshotSpeed};
use embassy_dshot::{DshotPioAsync, Command};

let mut dshot = DshotPio::<1, _>::new(
    p.PIO0, Irqs, p.PIN_0, DshotSpeed::DShot600,
);

dshot.send_command_async(Command::MotorStop).await;
dshot.throttle_async([500]).await.unwrap();
```

### Bidirectional DShot (eRPM telemetry)

The PIO program is loaded once per PIO block with `BidirDshotProgram`, then
shared by up to four `BidirDshotPio` drivers — one per state machine, one ESC
each. The program is 28 instructions and a PIO block holds 32, so loading a
copy per motor would fail at the second one.

```rust,ignore
use embassy_dshot::rp::{BidirDshotPio, BidirDshotProgram, DshotSpeed};

let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
let prog = BidirDshotProgram::new(&mut common);

let mut dshot = BidirDshotPio::new(sm0, &mut common, p.PIN_11, &prog, DshotSpeed::DShot600);

match dshot.throttle_with_telemetry(500).await {
    Ok(telem) => {
        let rpm = telem.rpm(14); // 14-pole motor
    }
    Err(e) => { /* handle error */ }
}
```

### Four motors on one PIO block

```rust,ignore
use embassy_futures::join::join4;

let Pio { mut common, sm0, sm1, sm2, sm3, .. } = Pio::new(p.PIO0, Irqs);
let prog = BidirDshotProgram::new(&mut common);

let mut m1 = BidirDshotPio::new(sm0, &mut common, p.PIN_11, &prog, DshotSpeed::DShot300);
let mut m2 = BidirDshotPio::new(sm1, &mut common, p.PIN_12, &prog, DshotSpeed::DShot300);
let mut m3 = BidirDshotPio::new(sm2, &mut common, p.PIN_13, &prog, DshotSpeed::DShot300);
let mut m4 = BidirDshotPio::new(sm3, &mut common, p.PIN_14, &prog, DshotSpeed::DShot300);

// Await the four together, not in turn: each telemetry read can block for up
// to 500us, so sequential awaits would cost ~2ms per control-loop iteration.
let (t1, t2, t3, t4) = join4(
    m1.throttle_with_telemetry(500),
    m2.throttle_with_telemetry(500),
    m3.throttle_with_telemetry(500),
    m4.throttle_with_telemetry(500),
).await;
```

The state machine index is part of the type (`BidirDshotPio<'_, PIO0, 0>` and
`BidirDshotPio<'_, PIO0, 1>` differ), so the four drivers cannot be held in an
array — drive them individually or with a `join`.

> **Not verified on hardware.** Multi-ESC bidirectional is built and checked in
> CI on every supported chip, but neither the author nor the contributor has a
> four-motor rig with a telemetry-capable ESC to validate it against. Single-ESC
> bidirectional is the tested path. Reports welcome.

## DShot Speeds

Clock dividers are computed automatically from the system clock (`clk_sys_freq()`).

| Speed      | Baud Rate  | Unidirectional | Bidirectional |
|------------|------------|----------------|---------------|
| DShot150   | 150 kbit/s | Yes            | Untested      |
| DShot300   | 300 kbit/s | Yes            | Yes           |
| DShot600   | 600 kbit/s | Yes            | Yes           |
| DShot1200  | 1.2 Mbit/s | Yes            | No            |

**Note:** DShot1200 bidirectional is not supported (panics at construction) — the
PIO RX pulse-width measurement cannot resolve bit periods at 1.2Mbit/s.
DShot150 bidirectional is untested and may not work with all ESCs.

## API

### Unidirectional (`DshotPio`)

Sync (`DshotPioTrait`):
- `throttle_clamp(&mut self, throttle: [u16; N])`
- `send_command(&mut self, cmd: Command)`
- `throttle_idle(&mut self)`

Async (`DshotPioAsync`):
- `throttle_async(&mut self, throttle: [u16; N])`
- `send_command_async(&mut self, cmd: Command)`
- `throttle_idle_async(&mut self)`
- `arm_async(&mut self, duration: Duration)` — send MotorStop at ~1kHz for the given duration

### Bidirectional (`BidirDshotPio`)

Constructed from a state machine plus a `BidirDshotProgram` shared across the
PIO block; one instance drives one ESC.

- `throttle_with_telemetry(&mut self, throttle: u16) -> Result<Telemetry, DshotError>`
- `command_with_telemetry(&mut self, cmd: Command) -> Result<Telemetry, DshotError>`
- `send_command_async(&mut self, cmd: Command)`
- `throttle_idle_async(&mut self)`
- `arm_async(&mut self, duration: Duration)` — send MotorStop at ~1kHz for the given duration
- `read_extended_telemetry(&mut self, throttle: u16) -> Result<ExtendedTelemetry, DshotError>`

### Extended DShot Telemetry (EDT)

When EDT is enabled (`Command::ExtendedTelemetryEnable`, sent 6x), the ESC interleaves
eRPM frames with sensor data. Use `read_extended_telemetry()` on `BidirDshotPio` to
decode the self-describing 12-bit telemetry frames:

- `ExtendedTelemetry::Erpm` — electrical RPM (standard)
- `ExtendedTelemetry::Temperature` — ESC temperature in °C
- `ExtendedTelemetry::Voltage` — supply voltage in millivolts
- `ExtendedTelemetry::Current` — motor current in milliamps
- `ExtendedTelemetry::StressLevel`, `Debug1`, `Debug2`, `Status` — firmware-specific

The standalone `decode_extended_telemetry(raw_12: u16)` function is also available
for custom telemetry pipelines.

## Examples

See the [`examples/`](examples/) directory. Build with:

```sh
# RP235xA (default)
cargo build --manifest-path examples/Cargo.toml --release --bin single_esc

# RP235xB
cargo build --manifest-path examples/Cargo.toml --no-default-features --features rp235xb \
    --target thumbv8m.main-none-eabihf --release --bin single_esc

# RP2040
cargo build --manifest-path examples/Cargo.toml --no-default-features --features rp2040 \
    --target thumbv6m-none-eabi --release --bin single_esc
```

Available examples: `bringup`, `single_esc`, `bdshot_test`, `esc_command_test`, `twin_engine`, `quad_engine`, `rpm_range`

## Credits

Based on [peterkrull/dshot-pio](https://github.com/peterkrull/dshot-pio). Bidirectional DShot based on [pico-bidir-dshot](https://github.com/bird-sanctuary/pico-bidir-dshot).

## License

MIT or Apache-2.0, at your option.
