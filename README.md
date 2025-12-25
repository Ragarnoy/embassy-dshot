# embassy-dshot

Async DShot ESC protocol driver for embassy-rp supporting both RP2040 and RP2350.

This crate provides an async-first DShot implementation using the RP-series PIO (Programmable I/O) to control up to 4 ESCs simultaneously per PIO block. Perfect for quadcopters and other multi-rotor applications.

## Features

- ✅ **Async/await** - Non-blocking motor control with Embassy
- ✅ **RP2040 & RP2350** - Support for both chips
- ✅ **Multi-motor** - Control 1-4 motors per PIO block (2 PIO blocks available)
- ✅ **Type-safe** - Uses [dshot-frame](https://github.com/sulami/dshot-frame) for frame encoding
- ✅ **No std** - Embedded-ready with optional std for testing

## Usage

Add to your `Cargo.toml`:

```toml
# For RP2040
embassy-dshot = { version = "0.1", features = ["rp2040"] }

# For RP2350
embassy-dshot = { version = "0.1", features = ["rp2350"] }
```

## Example

```rust
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_dshot::dshot_embassy_rp::DshotPio;
use embassy_dshot::{DshotPioAsync, Command};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create DShot driver for 4 motors
    let mut dshot = DshotPio::<4, _>::new(
        p.PIO0,
        Irqs,
        p.PIN_13,
        p.PIN_7,
        p.PIN_6,
        p.PIN_12,
        (52, 0) // Clock divider for DShot600 @ 125MHz
    );

    // Send commands asynchronously
    dshot.send_command_async(Command::MotorStop).await;
    dshot.throttle_async([1000, 1000, 1000, 1000]).await?;
}
```

## Clock Divider Calculation

For reliable ESC communication, calculate the clock divider:

$$\text{clock divider} = \frac{\text{system clock}}{8 \cdot \text{dshot speed} \cdot 1000}$$

Examples:
- DShot150 @ 125MHz: `(104, 0)`
- DShot300 @ 125MHz: `(52, 0)`
- DShot600 @ 125MHz: `(26, 0)`

Pass as `(integer_part, fractional_part)` where fractional = `(remainder * 256) as u8`.

## API

### Sync API (`DshotPioTrait`)
- `throttle_clamp(&mut self, throttle: [u16; N])`
- `send_command(&mut self, cmd: Command)`
- `throttle_minimum(&mut self)`

### Async API (`DshotPioAsync`)
- `throttle_async(&mut self, throttle: [u16; N])`
- `send_command_async(&mut self, cmd: Command)`
- `throttle_minimum_async(&mut self)`

## Credits

Originally forked from [peterkrull/dshot-pio](https://github.com/peterkrull/dshot-pio). This version has been completely rewritten for embassy-rp 0.9 with async support, RP2350 compatibility, and integration with the dshot-frame crate.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE) or http://opensource.org/licenses/MIT)

at your option.
