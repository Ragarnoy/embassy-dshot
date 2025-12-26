# embassy-dshot

DShot ESC protocol driver for embassy-rp (RP2040/RP2350).

Uses PIO to control up to 4 ESCs per PIO block.

## Features

- Async/await with Embassy
- RP2040 and RP2350 support
- 1-4 motors per PIO block
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

    let mut dshot = DshotPio::<4, _>::new(
        p.PIO0,
        Irqs,
        p.PIN_13,
        p.PIN_7,
        p.PIN_6,
        p.PIN_12,
        (52, 0) // Clock divider for DShot600 @ 125MHz
    );

    dshot.send_command_async(Command::MotorStop).await;
    dshot.throttle_async([1000, 1000, 1000, 1000]).await?;
}
```

## Clock Divider

```
clock_divider = system_clock / (8 * dshot_speed * 1000)
```

| Speed    | @ 125MHz |
|----------|----------|
| DShot150 | (104, 0) |
| DShot300 | (52, 0)  |
| DShot600 | (26, 0)  |

Format: `(integer_part, fractional_part)` where fractional = `(remainder * 256) as u8`.

## API

### Sync (`DshotPioTrait`)
- `throttle_clamp(&mut self, throttle: [u16; N])`
- `send_command(&mut self, cmd: Command)`
- `throttle_minimum(&mut self)`

### Async (`DshotPioAsync`)
- `throttle_async(&mut self, throttle: [u16; N])`
- `send_command_async(&mut self, cmd: Command)`
- `throttle_minimum_async(&mut self)`

## Credits

Based on [peterkrull/dshot-pio](https://github.com/peterkrull/dshot-pio).

## License

MIT or Apache-2.0, at your option.
