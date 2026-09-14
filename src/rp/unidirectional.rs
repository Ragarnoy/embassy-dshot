use super::{make_command_frame, DshotSpeed, THROTTLE_IDLE};
use crate::{Command, DshotError, DshotPioAsync, DshotPioTrait};
use dshot_frame::{Frame, NormalDshot};
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Config, FifoJoin, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
};
use embassy_rp::Peri;
use fixed::types::extra::U8;
use fixed::FixedU32;

/// PIO clock divider for unidirectional `DShot` TX.
///
/// The TX program below spends 8 PIO cycles per `DShot` bit, so the state
/// machine must run at `8 * baud_rate`.
#[allow(clippy::cast_possible_truncation, clippy::cast_lossless)]
const fn tx_pio_clock_divider(speed: DshotSpeed, sys_clock_hz: u32) -> FixedU32<U8> {
    let sys_clock = sys_clock_hz as u64;
    let target = 8 * speed.baud_rate() as u64;
    FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
}

// DShot TX PIO program (8 PIO cycles per bit).
// Frame in lower 16 bits of TX FIFO word, sent MSB-first.
macro_rules! dshot_tx_program {
    () => {
        pio_asm!(
            "set pindirs, 1", // Configure pin as output
            "entry:",
            "    pull block",   // Wait for data
            "    out null, 16", // Discard upper 16 bits
            "    set x, 15",    // 16 bits to send
            "loop:",
            "    set pins, 1",  // Start bit HIGH
            "    out y, 1",     // Get next bit
            "    jmp !y, zero", // Branch if bit is 0
            "    nop [2]",      // bit1: extra HIGH time (3+3=6 cycles HIGH)
            "one:",
            "    set pins, 0",   // End HIGH
            "    jmp x--, loop", // Next bit
            "    jmp reset",
            "zero:",
            "    set pins, 0 [3]", // bit0: end HIGH early + delay (3 cycles HIGH)
            "    jmp x--, loop",   // Next bit
            "    jmp reset",
            "reset:",
            "    nop [31]", // Inter-frame gap
            "    nop [31]",
            "    nop [31]",
            "    jmp entry [31]",
        )
    };
}

/// Unidirectional `DShot` PIO driver (1-4 motors).
pub struct DshotPio<'a, const N: usize, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
}

macro_rules! impl_dshot_pio_new {
    ($n:literal, $($pin_name:ident => $sm:ident),+) => {
        impl<'a, PIO: Instance> DshotPio<'a, $n, PIO> {
            pub fn new(
                pio: Peri<'a, PIO>,
                irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
                $($pin_name: Peri<'a, impl PioPin + 'a>,)+
                speed: DshotSpeed,
            ) -> DshotPio<'a, $n, PIO> {
                let prg = dshot_tx_program!();
                let mut cfg = Config::default();
                let mut pio = Pio::new(pio, irq);

                cfg.clock_divider = tx_pio_clock_divider(speed, clk_sys_freq());
                cfg.shift_out = ShiftConfig {
                    auto_fill: false,
                    direction: ShiftDirection::Left,
                    threshold: 32,
                };
                cfg.fifo_join = FifoJoin::TxOnly;

                let loaded_prg = pio.common.load_program(&prg.program);

                $(
                    let $pin_name = pio.common.make_pio_pin($pin_name);
                    cfg.set_set_pins(&[&$pin_name]);
                    cfg.use_program(&loaded_prg, &[]);
                    pio.$sm.set_config(&cfg);
                    pio.$sm.set_enable(true);
                )+

                DshotPio { pio_instance: pio }
            }
        }
    };
}

impl_dshot_pio_new!(1, pin0 => sm0);
impl_dshot_pio_new!(2, pin0 => sm0, pin1 => sm1);
impl_dshot_pio_new!(3, pin0 => sm0, pin1 => sm1, pin2 => sm2);
impl_dshot_pio_new!(4, pin0 => sm0, pin1 => sm1, pin2 => sm2, pin3 => sm3);

macro_rules! impl_dshot_traits {
    ($n:literal, $($idx:literal => $sm:ident),+) => {
        impl<'d, PIO: Instance> DshotPioTrait<$n> for DshotPio<'d, $n, PIO> {
            fn command(&mut self, command: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = make_command_frame(command[$idx])?;
                    self.pio_instance.$sm.tx().push(u32::from(frame.inner()));
                )+
                Ok(())
            }

            fn reverse(&mut self, reverse: [bool; $n]) {
                $(
                    let cmd = if reverse[$idx] {
                        Command::SpinDirectonReversed
                    } else {
                        Command::SpinDirectionNormal
                    };
                    let frame = Frame::<NormalDshot>::command(cmd, false);
                    self.pio_instance.$sm.tx().push(u32::from(frame.inner()));
                )+
            }

            fn throttle_clamp(&mut self, throttle: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = Frame::<NormalDshot>::new(throttle[$idx].min(1999), false)
                        .ok_or(DshotError::InvalidThrottle)?;
                    self.pio_instance.$sm.tx().push(u32::from(frame.inner()));
                )+
                Ok(())
            }

            fn throttle_idle(&mut self) {
                let frame = Frame::<NormalDshot>::new(THROTTLE_IDLE, false)
                    .expect("Idle throttle should always be valid");
                $(
                    self.pio_instance.$sm.tx().push(u32::from(frame.inner()));
                )+
            }

            fn send_command(&mut self, cmd: Command) {
                let frame = Frame::<NormalDshot>::command(cmd, false);
                $(
                    self.pio_instance.$sm.tx().push(u32::from(frame.inner()));
                )+
            }

            fn send_command_repeated(&mut self, cmd: Command, count: u8) {
                for _ in 0..count {
                    self.send_command(cmd);
                }
            }
        }

        impl<'d, PIO: Instance> DshotPioAsync<$n> for DshotPio<'d, $n, PIO> {
            async fn command_async(&mut self, command: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = make_command_frame(command[$idx])?;
                    self.pio_instance.$sm.tx().wait_push(u32::from(frame.inner())).await;
                )+
                Ok(())
            }

            async fn reverse_async(&mut self, reverse: [bool; $n]) {
                $(
                    let cmd = if reverse[$idx] {
                        Command::SpinDirectonReversed
                    } else {
                        Command::SpinDirectionNormal
                    };
                    let frame = Frame::<NormalDshot>::command(cmd, false);
                    self.pio_instance.$sm.tx().wait_push(u32::from(frame.inner())).await;
                )+
            }

            async fn throttle_async(&mut self, throttle: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = Frame::<NormalDshot>::new(throttle[$idx].min(1999), false)
                        .ok_or(DshotError::InvalidThrottle)?;
                    self.pio_instance.$sm.tx().wait_push(u32::from(frame.inner())).await;
                )+
                Ok(())
            }

            async fn throttle_idle_async(&mut self) {
                let frame = Frame::<NormalDshot>::new(THROTTLE_IDLE, false)
                    .expect("Idle throttle should always be valid");
                $(
                    self.pio_instance.$sm.tx().wait_push(u32::from(frame.inner())).await;
                )+
            }

            async fn send_command_async(&mut self, cmd: Command) {
                let frame = Frame::<NormalDshot>::command(cmd, false);
                $(
                    self.pio_instance.$sm.tx().wait_push(u32::from(frame.inner())).await;
                )+
            }

            async fn send_command_repeated_async(&mut self, cmd: Command, count: u8) {
                for _ in 0..count {
                    self.send_command_async(cmd).await;
                }
            }
        }
    };
}

impl_dshot_traits!(1, 0 => sm0);
impl_dshot_traits!(2, 0 => sm0, 1 => sm1);
impl_dshot_traits!(3, 0 => sm0, 1 => sm1, 2 => sm2);
impl_dshot_traits!(4, 0 => sm0, 1 => sm1, 2 => sm2, 3 => sm3);

#[cfg(test)]
mod tests {
    use super::tx_pio_clock_divider;
    use crate::DshotSpeed;

    /// 8 PIO cycles/bit: DShot600 needs a 4.8MHz PIO clock, so at 125MHz the
    /// divider is 125/4.8 = 26.0417 -> 6666 in 24.8 fixed point.
    #[test]
    fn tx_divider_at_125mhz() {
        let div = tx_pio_clock_divider(DshotSpeed::DShot600, 125_000_000);
        assert_eq!(div.to_bits(), 6666, "raw 24.8 divider wrong");
        assert_eq!(div.to_bits() >> 8, 26, "integer part wrong");
    }

    #[test]
    fn tx_divider_scales_inversely_with_baud() {
        // Halving the bit rate doubles the divider. Uses a clock that divides
        // evenly at both speeds, so fixed-point truncation can't skew the ratio.
        let fast = tx_pio_clock_divider(DshotSpeed::DShot600, 120_000_000).to_bits();
        let slow = tx_pio_clock_divider(DshotSpeed::DShot300, 120_000_000).to_bits();
        assert_eq!(slow, fast * 2);
    }

    #[test]
    fn tx_divider_encodes_eight_cycles_per_bit() {
        // Pick a clock that divides exactly so the check is independent of rounding.
        let div = tx_pio_clock_divider(DshotSpeed::DShot150, 120_000_000);
        let pio_clock = 120_000_000u64 * 256 / u64::from(div.to_bits());
        assert_eq!(pio_clock, 8 * 150_000);
    }
}
