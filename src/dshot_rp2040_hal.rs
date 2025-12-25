pub use super::{DshotPioTrait, DshotError, Telemetry, Frame, NormalDshot, Command};

use rp2040_hal::{
    gpio::{Function, Pin, PinId, PullType, ValidFunction},
    pac::RESETS,
    pio::{
        InstalledProgram, PIOBuilder, PIOExt, PinDir, ShiftDirection, StateMachineIndex, Tx,
        UninitStateMachine, SM0, SM1, SM2, SM3,
    },
};

pub struct DshotPio<const N: usize, P: PIOExt> {
    sm0: Tx<(P, SM0)>,
    sm1: Tx<(P, SM1)>,
    sm2: Tx<(P, SM2)>,
    sm3: Tx<(P, SM3)>,
    telemetry_enabled: bool,
}

fn configure_pio_instance<P: PIOExt>(
    pio_block: P,
    resets: &mut RESETS,
) -> (
    InstalledProgram<P>,
    (
        UninitStateMachine<(P, SM0)>,
        UninitStateMachine<(P, SM1)>,
        UninitStateMachine<(P, SM2)>,
        UninitStateMachine<(P, SM3)>,
    ),
) {
    // Split the PIO block into individual state machines
    let (mut pio, sm0, sm1, sm2, sm3) = pio_block.split(resets);

    // Program that generates DShot signal in PIO state machine
    let dshot_pio_program = pio_proc::pio_asm!(
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

    // Install DShot program into PIO block
    (
        pio.install(&dshot_pio_program.program)
            .expect("Unable to install program into PIO block"),
        (sm0, sm1, sm2, sm3),
    )
}

///
/// Defining constructor functions
///

fn setup_state_machine<P: PIOExt, SM: StateMachineIndex>(
    installed: &InstalledProgram<P>,
    sm: UninitStateMachine<(P, SM)>,
    clk_div: (u16, u8),
    pin: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
) -> Tx<(P, SM)> {

    // Configure pin for use with this PIO block
    let pin = pin.into_function::<P::PinFunction>();

    // SAFETY: We never uninstall the program, so all unsafety considerations are met
    let (mut smx, _, tx) = PIOBuilder::from_installed_program(unsafe { installed.share() })
        .set_pins(pin.id().num, 1)
        .clock_divisor_fixed_point(clk_div.0, clk_div.1)
        .out_shift_direction(ShiftDirection::Left)
        .pull_threshold(32)
        .autopull(true)
        .build(sm);

    smx.set_pindirs([(pin.id().num, PinDir::Output)]);
    smx.start(); // NOTE: This consumes the state machine
    tx
}

fn dummy_state_machine<P: PIOExt, SM: StateMachineIndex>(
    installed: &InstalledProgram<P>,
    sm: UninitStateMachine<(P, SM)>,
) -> Tx<(P, SM)> {
    let (_, _, tx) = PIOBuilder::from_installed_program(unsafe { installed.share() }).build(sm);
    tx
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<1, P> {
    pub fn new<I0>(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        clk_div: (u16, u8),
    ) -> DshotPio<1, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        // Configure the state machine
        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0);

        // Setup dummy program for unused state machines
        let tx1 = dummy_state_machine(&installed, sm.1);
        let tx2 = dummy_state_machine(&installed, sm.2);
        let tx3 = dummy_state_machine(&installed, sm.3);

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
            telemetry_enabled: false,
        }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<2, P> {
    pub fn new(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        pin1: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        clk_div: (u16, u8),
    ) -> DshotPio<2, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        // Configure the state machine
        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0);
        let tx1 = setup_state_machine(&installed, sm.1, clk_div, pin1);

        // Setup dummy program for unused state machines
        let tx2 = dummy_state_machine(&installed, sm.2);
        let tx3 = dummy_state_machine(&installed, sm.3);

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
            telemetry_enabled: false,
        }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<3, P> {
    pub fn new(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        pin1: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        pin2: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        clk_div: (u16, u8),
    ) -> DshotPio<3, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        // Configure the state machine
        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0);
        let tx1 = setup_state_machine(&installed, sm.1, clk_div, pin1);
        let tx2 = setup_state_machine(&installed, sm.2, clk_div, pin2);

        // Setup dummy program for unused state machines
        let tx3 = dummy_state_machine(&installed, sm.3);

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
            telemetry_enabled: false,
        }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<4, P> {
    pub fn new(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        pin1: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        pin2: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        pin3: Pin<impl PinId + ValidFunction<P::PinFunction>, impl Function, impl PullType>,
        clk_div: (u16, u8),
    ) -> DshotPio<4, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        // Configure the state machine
        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0);
        let tx1 = setup_state_machine(&installed, sm.1, clk_div, pin1);
        let tx2 = setup_state_machine(&installed, sm.2, clk_div, pin2);
        let tx3 = setup_state_machine(&installed, sm.3, clk_div, pin3);

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
            telemetry_enabled: false,
        }
    }
}

///
/// Implementing DshotPioTrait
///

impl<P: PIOExt> super::DshotPioTrait<1> for DshotPio<1, P> {
    fn command(&mut self, command: [u16; 1]) -> Result<(), DshotError> {
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
        self.sm0.write(frame.inner() as u32);
        Ok(())
    }

    fn reverse(&mut self, reverse: [bool; 1]) {
        let cmd = if reverse[0] { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.sm0.write(frame.inner() as u32);
    }

    fn throttle_clamp(&mut self, throttle: [u16; 1]) -> Result<(), DshotError> {
        let frame = Frame::<NormalDshot>::new(throttle[0].min(1999), self.telemetry_enabled)
            .ok_or(DshotError::InvalidThrottle)?;
        self.sm0.write(frame.inner() as u32);
        Ok(())
    }

    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.sm0.write(frame.inner() as u32);
    }

    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.sm0.write(frame.inner() as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<2> for DshotPio<2, P> {
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
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn reverse(&mut self, reverse: [bool; 2]) {
        for (i, &rev) in reverse.iter().enumerate() {
            let cmd = if rev { Command::SpinDirectonReversed } else { Command::SpinDirectionNormal };
            let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
            match i {
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                _ => unreachable!(),
            }
        }
    }

    fn throttle_clamp(&mut self, throttle: [u16; 2]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.sm0.write(frame.inner() as u32);
        self.sm1.write(frame.inner() as u32);
    }

    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.sm0.write(frame.inner() as u32);
        self.sm1.write(frame.inner() as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<3> for DshotPio<3, P> {
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
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                2 => { self.sm2.write(frame.inner() as u32); },
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
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                2 => { self.sm2.write(frame.inner() as u32); },
                _ => unreachable!(),
            }
        }
    }

    fn throttle_clamp(&mut self, throttle: [u16; 3]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                2 => { self.sm2.write(frame.inner() as u32); },
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.sm0.write(frame.inner() as u32);
        self.sm1.write(frame.inner() as u32);
        self.sm2.write(frame.inner() as u32);
    }

    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.sm0.write(frame.inner() as u32);
        self.sm1.write(frame.inner() as u32);
        self.sm2.write(frame.inner() as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<4> for DshotPio<4, P> {
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
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                2 => { self.sm2.write(frame.inner() as u32); },
                3 => { self.sm3.write(frame.inner() as u32); },
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
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                2 => { self.sm2.write(frame.inner() as u32); },
                3 => { self.sm3.write(frame.inner() as u32); },
                _ => unreachable!(),
            }
        }
    }

    fn throttle_clamp(&mut self, throttle: [u16; 4]) -> Result<(), DshotError> {
        for (i, &t) in throttle.iter().enumerate() {
            let frame = Frame::<NormalDshot>::new(t.min(1999), self.telemetry_enabled)
                .ok_or(DshotError::InvalidThrottle)?;
            match i {
                0 => { self.sm0.write(frame.inner() as u32); },
                1 => { self.sm1.write(frame.inner() as u32); },
                2 => { self.sm2.write(frame.inner() as u32); },
                3 => { self.sm3.write(frame.inner() as u32); },
                _ => unreachable!(),
            }
        }
        Ok(())
    }

    fn throttle_minimum(&mut self) {
        let frame = Frame::<NormalDshot>::new(0, self.telemetry_enabled)
            .expect("Zero throttle should always be valid");
        self.sm0.write(frame.inner() as u32);
        self.sm1.write(frame.inner() as u32);
        self.sm2.write(frame.inner() as u32);
        self.sm3.write(frame.inner() as u32);
    }

    fn send_command(&mut self, cmd: Command) {
        let frame = Frame::<NormalDshot>::command(cmd, self.telemetry_enabled);
        self.sm0.write(frame.inner() as u32);
        self.sm1.write(frame.inner() as u32);
        self.sm2.write(frame.inner() as u32);
        self.sm3.write(frame.inner() as u32);
    }
}
