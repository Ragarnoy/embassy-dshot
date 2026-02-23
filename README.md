# embassy-dshot

DShot ESC protocol driver for embassy-rp (RP2040/RP2350).

Uses PIO to control up to 4 ESCs per PIO block. Supports both unidirectional and bidirectional DShot with GCR-encoded eRPM telemetry.

## Features

- Async/await with Embassy
- RP2040 and RP2350 support
- 1-4 motors per PIO block (unidirectional)
- Bidirectional DShot with eRPM telemetry (single ESC)
- Automatic clock divider calculation from system clock
- Type-safe frames via [dshot-frame](https://github.com/sulami/dshot-frame)
- `no_std`

## Usage

```toml
# For RP2040
embassy-dshot = { version = "0.1", features = ["rp2040"] }

# For RP2350
embassy-dshot = { version = "0.1", features = ["rp2350"] }
```

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

### Twin Engine (bidirectional, 2 motors)

```rust,ignore
use embassy_dshot::rp::{BidirDshotPio, DshotSpeed};

let mut engine1 = BidirDshotPio::new(p.PIO0, Irqs0, p.PIN_11, DshotSpeed::DShot300);
let mut engine2 = BidirDshotPio::new(p.PIO1, Irqs1, p.PIN_12, DshotSpeed::DShot300);

// Read telemetry from both and compare RPMs
let t1 = engine1.throttle_with_telemetry(500).await?;
let t2 = engine2.throttle_with_telemetry(500).await?;
let rpm1 = t1.rpm(14);
let rpm2 = t2.rpm(14);
```

### Bidirectional DShot (eRPM telemetry)

```rust,ignore
use embassy_dshot::rp::{BidirDshotPio, DshotSpeed};

let mut dshot = BidirDshotPio::new(
    p.PIO0, Irqs, p.PIN_11, DshotSpeed::DShot600,
);

match dshot.throttle_with_telemetry(500).await {
    Ok(telem) => {
        let rpm = telem.rpm(14); // 14-pole motor
    }
    Err(e) => { /* handle error */ }
}
```

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
# RP2350 (default)
cargo build --manifest-path examples/Cargo.toml --release --bin single_esc

# RP2040
cargo build --manifest-path examples/Cargo.toml --no-default-features --features rp2040 \
    --target thumbv6m-none-eabi --release --bin single_esc
```

Available examples: `bringup`, `single_esc`, `bdshot_test`, `esc_command_test`, `twin_engine`

## Credits

Based on [peterkrull/dshot-pio](https://github.com/peterkrull/dshot-pio). Bidirectional DShot based on [pico-bidir-dshot](https://github.com/bird-sanctuary/pico-bidir-dshot).

## License

MIT or Apache-2.0, at your option.
