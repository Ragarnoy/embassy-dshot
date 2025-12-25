pub use super::{DshotPioTrait, DshotError, Telemetry, Frame, NormalDshot, Command};

use embassy_rp::Peri;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Config, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
};
use embassy_rp::interrupt::typelevel::Binding;

/// DShot PIO driver for Embassy
pub struct DshotPio<'a, const N : usize, PIO : Instance> {
    pio_instance: Pio<'a, PIO>,
    telemetry_enabled: bool,
}


fn configure_pio_instance<'a,PIO: Instance>  (
    pio: Peri<'a, PIO>,
    irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
    clk_div: (u16, u8),
) -> (Config<'a, PIO>, Pio<'a, PIO>) {
    
    // Define program
    let dshot_pio_program = pio_asm!(
        "set pindirs, 1",
        "entry:"
        "   pull"
        "   out null 16"
        "   set x 15"
        "loop:"
        "   set pins 1"
        "   out y 1"
        "   jmp !y zero"
        "   nop [2]"
        "one:" // 6 and 2
        "   set pins 0"
        "   jmp x-- loop"
        "   jmp reset"
        "zero:" // 3 and 5
        "   set pins 0 [3]"
        "   jmp x-- loop"
        "   jmp reset"
        "reset:" // Blank frame
        "   nop [31]"
        "   nop [31]"
        "   nop [31]"
        "   jmp entry [31]"
    );

    // Configure program
    let mut cfg = Config::default();
    let mut pio = Pio::new(pio,irq);
    cfg.use_program(&pio.common.load_program(&dshot_pio_program.program), &[]);
    cfg.clock_divider = clk_div.0.into();

    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        direction: Default::default(),
        threshold: 32,
    };

    cfg.shift_out = ShiftConfig {
        auto_fill: Default::default(),
        direction: ShiftDirection::Left,
        threshold: Default::default(),
    };

    (cfg,pio)

}

///
/// Defining constructor functions
/// 

impl <'a,PIO: Instance> DshotPio<'a,1,PIO> {
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        clk_div: (u16, u8),
    ) -> DshotPio<'a,1,PIO> {

        let (mut cfg, mut pio) = configure_pio_instance(pio, irq, clk_div);

        // Set pins and enable all state machines
        let pin0 = pio.common.make_pio_pin(pin0);
        cfg.set_set_pins(&[&pin0]);
        pio.sm0.set_config(&cfg);
        pio.sm0.set_enable(true);

        // Return struct of 1 configured DShot state machine
        DshotPio {
            pio_instance: pio,
            telemetry_enabled: false,
        }
    }
}

impl <'a,PIO: Instance> DshotPio<'a,2,PIO> {
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        pin1: Peri<'a, impl PioPin + 'a>,
        clk_div: (u16, u8),
    ) -> DshotPio<'a,2,PIO> {

        let (mut cfg, mut pio) = configure_pio_instance(pio, irq, clk_div);

        // Set pins and enable all state machines
        let pin0 = pio.common.make_pio_pin(pin0);
        cfg.set_set_pins(&[&pin0]);
        pio.sm0.set_config(&cfg);
        pio.sm0.set_enable(true);

        let pin1 = pio.common.make_pio_pin(pin1);
        cfg.set_set_pins(&[&pin1]);
        pio.sm1.set_config(&cfg);
        pio.sm1.set_enable(true);

        // Return struct of 2 configured DShot state machines
        DshotPio {
            pio_instance: pio,
            telemetry_enabled: false,
        }
    }
}

impl <'a,PIO: Instance> DshotPio<'a,3,PIO> {
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        pin1: Peri<'a, impl PioPin + 'a>,
        pin2: Peri<'a, impl PioPin + 'a>,
        clk_div: (u16, u8),
    ) -> DshotPio<'a,3,PIO> {

        let (mut cfg, mut pio) = configure_pio_instance(pio, irq, clk_div);

        // Set pins and enable all state machines
        let pin0 = pio.common.make_pio_pin(pin0);
        cfg.set_set_pins(&[&pin0]);
        pio.sm0.set_config(&cfg);
        pio.sm0.set_enable(true);

        let pin1 = pio.common.make_pio_pin(pin1);
        cfg.set_set_pins(&[&pin1]);
        pio.sm1.set_config(&cfg);
        pio.sm1.set_enable(true);

        let pin2 = pio.common.make_pio_pin(pin2);
        cfg.set_set_pins(&[&pin2]);
        pio.sm2.set_config(&cfg);
        pio.sm2.set_enable(true);
        
        // Return struct of 3 configured DShot state machines
        DshotPio {
            pio_instance: pio,
            telemetry_enabled: false,
        }
    }
}

impl <'a,PIO: Instance> DshotPio<'a,4,PIO> {
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        pin1: Peri<'a, impl PioPin + 'a>,
        pin2: Peri<'a, impl PioPin + 'a>,
        pin3: Peri<'a, impl PioPin + 'a>,
        clk_div: (u16, u8),
    ) -> DshotPio<'a,4,PIO> {

        let (mut cfg, mut pio) = configure_pio_instance(pio, irq, clk_div);

        // Set pins and enable all state machines
        let pin0 = pio.common.make_pio_pin(pin0);
        cfg.set_set_pins(&[&pin0]);
        pio.sm0.set_config(&cfg);
        pio.sm0.set_enable(true);

        let pin1 = pio.common.make_pio_pin(pin1);
        cfg.set_set_pins(&[&pin1]);
        pio.sm1.set_config(&cfg);
        pio.sm1.set_enable(true);

        let pin2 = pio.common.make_pio_pin(pin2);
        cfg.set_set_pins(&[&pin2]);
        pio.sm2.set_config(&cfg);
        pio.sm2.set_enable(true);

        let pin3 = pio.common.make_pio_pin(pin3);
        cfg.set_set_pins(&[&pin3]);
        pio.sm3.set_config(&cfg);
        pio.sm3.set_enable(true);

        // Return struct of 4 configured DShot state machines
        DshotPio {
            pio_instance: pio,
            telemetry_enabled: false,
        }
    }
}

///
/// Implementing DshotPioTrait
/// 

impl<'d, PIO: Instance> super::DshotPioTrait<1> for DshotPio<'d, 1, PIO> {
    /// Send raw DShot command values (0-2047)
    fn command(&mut self, command: [u16; 1]) -> Result<(), DshotError> {
        // For raw commands 0-47, create command frames directly
        if command[0] < 48 {
            let frame = Frame::<NormalDshot>::command(
                unsafe { core::mem::transmute(command[0] as u8) }, // Convert to Command enum
                self.telemetry_enabled
            );
            self.pio_instance.sm0.tx().push(frame.inner() as u32);
            Ok(())
        } else {
            // For values 48-2047, create as throttle (subtract 48)
            let throttle = command[0].saturating_sub(48);
            if throttle >= 2000 {
                return Err(DshotError::InvalidThrottle);
            }
            let frame = Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            self.pio_instance.sm0.tx().push(frame.inner() as u32);
            Ok(())
        }
    }

    /// Set motor rotation direction
    fn reverse(&mut self, reverse: [bool; 1]) {
        let cmd = if reverse[0] {
            Command::SpinDirectonReversed
        } else {
            Command::SpinDirectionNormal
        };
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
    }

    /// Set throttle (0-1999), clamped and converted to DShot range (48-2047)
    fn throttle_clamp(&mut self, throttle: [u16; 1]) -> Result<(), DshotError> {
        let clamped = throttle[0].min(1999);
        let frame = Frame::<NormalDshot>::new(clamped, self.telemetry_enabled)
            .ok_or(DshotError::InvalidThrottle)?;
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
        Ok(())
    }

    /// Set all motors to minimum throttle (zero)
    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
    }

    /// Send a DShot command to all motors
    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
    }
}

impl<'d, PIO: Instance> super::DshotPioTrait<2> for DshotPio<'d, 2, PIO> {
    fn command(&mut self, command: [u16; 2]) -> Result<(), DshotError> {
        for (i, &cmd) in command.iter().enumerate() {
            let frame = if cmd < 48 {
                Frame::<NormalDshot>::command(
                    unsafe { core::mem::transmute(cmd as u8) },
                    self.telemetry_enabled
                )
            } else {
                let throttle = cmd.saturating_sub(48);
                if throttle >= 2000 {
                    return Err(DshotError::InvalidThrottle);
                }
                Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                    .ok_or(DshotError::InvalidThrottle)?
            };
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn reverse(&mut self, reverse: [bool; 2]) {
        let frames: [_; 2] = core::array::from_fn(|i| {
            let cmd = if reverse[i] { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
            Frame::<NormalDshot>::command(cmd, self.telemetry_enabled)
        });
        self.pio_instance.sm0.tx().push(frames[0].inner() as u32);
        self.pio_instance.sm1.tx().push(frames[1].inner() as u32);
    }

    fn throttle_clamp(&mut self, throttle: [u16; 2]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
        self.pio_instance.sm1.tx().push(frame.inner() as u32);
    }

    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
        self.pio_instance.sm1.tx().push(frame.inner() as u32);
    }
}

impl<'d, PIO: Instance> super::DshotPioTrait<3> for DshotPio<'d, 3, PIO> {
    fn command(&mut self, command: [u16; 3]) -> Result<(), DshotError> {
        for (i, &cmd) in command.iter().enumerate() {
            let frame = if cmd < 48 {
                Frame::<NormalDshot>::command(
                    unsafe { core::mem::transmute(cmd as u8) },
                    self.telemetry_enabled
                )
            } else {
                let throttle = cmd.saturating_sub(48);
                if throttle >= 2000 {
                    return Err(DshotError::InvalidThrottle);
                }
                Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                    .ok_or(DshotError::InvalidThrottle)?
            };
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                2 => self.pio_instance.sm2.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn reverse(&mut self, reverse: [bool; 3]) {
        for (i, &rev) in reverse.iter().enumerate() {
            let cmd = if rev { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
            let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                2 => self.pio_instance.sm2.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
    }

    fn throttle_clamp(&mut self, throttle: [u16; 3]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                2 => self.pio_instance.sm2.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
        self.pio_instance.sm1.tx().push(frame.inner() as u32);
        self.pio_instance.sm2.tx().push(frame.inner() as u32);
    }

    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
        self.pio_instance.sm1.tx().push(frame.inner() as u32);
        self.pio_instance.sm2.tx().push(frame.inner() as u32);
    }
}

impl<'d, PIO: Instance> super::DshotPioTrait<4> for DshotPio<'d, 4, PIO> {
    fn command(&mut self, command: [u16; 4]) -> Result<(), DshotError> {
        for (i, &cmd) in command.iter().enumerate() {
            let frame = if cmd < 48 {
                Frame::<NormalDshot>::command(
                    unsafe { core::mem::transmute(cmd as u8) },
                    self.telemetry_enabled
                )
            } else {
                let throttle = cmd.saturating_sub(48);
                if throttle >= 2000 {
                    return Err(DshotError::InvalidThrottle);
                }
                Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                    .ok_or(DshotError::InvalidThrottle)?
            };
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                2 => self.pio_instance.sm2.tx().push(frame.inner() as u32),
                3 => self.pio_instance.sm3.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn reverse(&mut self, reverse: [bool; 4]) {
        for (i, &rev) in reverse.iter().enumerate() {
            let cmd = if rev { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
            let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                2 => self.pio_instance.sm2.tx().push(frame.inner() as u32),
                3 => self.pio_instance.sm3.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
    }

    fn throttle_clamp(&mut self, throttle: [u16; 4]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => self.pio_instance.sm0.tx().push(frame.inner() as u32),
                1 => self.pio_instance.sm1.tx().push(frame.inner() as u32),
                2 => self.pio_instance.sm2.tx().push(frame.inner() as u32),
                3 => self.pio_instance.sm3.tx().push(frame.inner() as u32),
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
        self.pio_instance.sm1.tx().push(frame.inner() as u32);
        self.pio_instance.sm2.tx().push(frame.inner() as u32);
        self.pio_instance.sm3.tx().push(frame.inner() as u32);
    }

    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().push(frame.inner() as u32);
        self.pio_instance.sm1.tx().push(frame.inner() as u32);
        self.pio_instance.sm2.tx().push(frame.inner() as u32);
        self.pio_instance.sm3.tx().push(frame.inner() as u32);
    }
}

///
/// Implementing DshotPioAsync (async/await methods)
///

impl<'d, PIO: Instance> super::DshotPioAsync<1> for DshotPio<'d, 1, PIO> {
    async fn command_async(&mut self, command: [u16; 1]) -> Result<(), DshotError> {
        let frame = if command[0] < 48 {
            Frame::<NormalDshot>::command(
                unsafe { core::mem::transmute(command[0] as u8) },
                self.telemetry_enabled
            )
        } else {
            let throttle = command[0].saturating_sub(48);
            if throttle >= 2000 {
                return Err(DshotError::InvalidThrottle);
            }
            Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?
        };
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        Ok(())
    }

    async fn reverse_async(&mut self, reverse: [bool; 1]) {
        let cmd = if reverse[0] {
            Command::SpinDirectonReversed
        } else {
            Command::SpinDirectionNormal
        };
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
    }

    async fn throttle_async(&mut self, throttle: [u16; 1]) -> Result<(), DshotError> {
        let clamped = throttle[0].min(1999);
        let frame = Frame::<NormalDshot>::new(clamped, self.telemetry_enabled)
            .ok_or(DshotError::InvalidThrottle)?;
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        Ok(())
    }

    async fn throttle_minimum_async(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
    }

    async fn send_command_async(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
    }
}

impl<'d, PIO: Instance> super::DshotPioAsync<2> for DshotPio<'d, 2, PIO> {
    async fn command_async(&mut self, command: [u16; 2]) -> Result<(), DshotError> {
        for (i, &cmd) in command.iter().enumerate() {
            let frame = if cmd < 48 {
                Frame::<NormalDshot>::command(
                    unsafe { core::mem::transmute(cmd as u8) },
                    self.telemetry_enabled
                )
            } else {
                let throttle = cmd.saturating_sub(48);
                if throttle >= 2000 {
                    return Err(DshotError::InvalidThrottle);
                }
                Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                    .ok_or(DshotError::InvalidThrottle)?
            };
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    async fn reverse_async(&mut self, reverse: [bool; 2]) {
        for (i, &rev) in reverse.iter().enumerate() {
            let cmd = if rev { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
            let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
    }

    async fn throttle_async(&mut self, throttle: [u16; 2]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    async fn throttle_minimum_async(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await;
    }

    async fn send_command_async(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await;
    }
}

impl<'d, PIO: Instance> super::DshotPioAsync<3> for DshotPio<'d, 3, PIO> {
    async fn command_async(&mut self, command: [u16; 3]) -> Result<(), DshotError> {
        for (i, &cmd) in command.iter().enumerate() {
            let frame = if cmd < 48 {
                Frame::<NormalDshot>::command(
                    unsafe { core::mem::transmute(cmd as u8) },
                    self.telemetry_enabled
                )
            } else {
                let throttle = cmd.saturating_sub(48);
                if throttle >= 2000 {
                    return Err(DshotError::InvalidThrottle);
                }
                Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                    .ok_or(DshotError::InvalidThrottle)?
            };
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                2 => self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    async fn reverse_async(&mut self, reverse: [bool; 3]) {
        for (i, &rev) in reverse.iter().enumerate() {
            let cmd = if rev { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
            let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                2 => self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
    }

    async fn throttle_async(&mut self, throttle: [u16; 3]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                2 => self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    async fn throttle_minimum_async(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await;
    }

    async fn send_command_async(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await;
    }
}

impl<'d, PIO: Instance> super::DshotPioAsync<4> for DshotPio<'d, 4, PIO> {
    async fn command_async(&mut self, command: [u16; 4]) -> Result<(), DshotError> {
        for (i, &cmd) in command.iter().enumerate() {
            let frame = if cmd < 48 {
                Frame::<NormalDshot>::command(
                    unsafe { core::mem::transmute(cmd as u8) },
                    self.telemetry_enabled
                )
            } else {
                let throttle = cmd.saturating_sub(48);
                if throttle >= 2000 {
                    return Err(DshotError::InvalidThrottle);
                }
                Frame::<NormalDshot>::new(throttle, self.telemetry_enabled)
                    .ok_or(DshotError::InvalidThrottle)?
            };
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                2 => self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await,
                3 => self.pio_instance.sm3.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    async fn reverse_async(&mut self, reverse: [bool; 4]) {
        for (i, &rev) in reverse.iter().enumerate() {
            let cmd = if rev { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
            let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                2 => self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await,
                3 => self.pio_instance.sm3.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
    }

    async fn throttle_async(&mut self, throttle: [u16; 4]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await,
                1 => self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await,
                2 => self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await,
                3 => self.pio_instance.sm3.tx().wait_push(frame.inner() as u32).await,
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    async fn throttle_minimum_async(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm3.tx().wait_push(frame.inner() as u32).await;
    }

    async fn send_command_async(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.pio_instance.sm0.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm1.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm2.tx().wait_push(frame.inner() as u32).await;
        self.pio_instance.sm3.tx().wait_push(frame.inner() as u32).await;
    }
}
