use super::{DshotError, Telemetry, Frame, NormalDshot, BidirectionalDshot, Command, gcr_decode, verify_telemetry_crc, ExtendedTelemetry, decode_extended_telemetry};

/// Idle throttle value for `Frame::new()` — produces DShot protocol value 48
/// (the lowest non-command throttle value).
///
/// **Note:** This is NOT the same as `Command::MotorStop` (DShot value 0).
/// Some ESCs will creep slightly at this throttle. To fully stop the motor
/// and keep it stopped, use `send_command(Command::MotorStop)` instead.
///
/// `Frame::new()` maps speed 0-1999 to DShot values 48-2047, so idle
/// speed is 0, which corresponds to DShot protocol value 48.
pub const THROTTLE_IDLE: u16 = 0;

use embassy_rp::Peri;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Config, Direction, FifoJoin, Instance, InterruptHandler, Pio, Pin, PioPin, ShiftConfig,
    ShiftDirection,
};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_time::{with_timeout, Duration, Timer};
use fixed::types::extra::U8;
use fixed::FixedU32;

/// Convert 12-bit telemetry value to eRPM
/// Format: eeem mmmm mmmm (3-bit exponent, 9-bit mantissa)
/// eRPM = mantissa << exponent
/// Period (us) = 60_000_000 / eRPM (if eRPM > 0)
#[allow(clippy::cast_lossless)] // as casts required for const fn
const fn telemetry_to_erpm(value: u16) -> (u32, Option<u32>) {
    if value == 0 || value == 0x0FFF {
        return (0, None);
    }
    let exponent = (value >> 9) & 0x07;
    let mantissa = value & 0x1FF;
    let period_us = (mantissa as u32) << (exponent as u32);
    if period_us == 0 {
        return (0, None);
    }
    let erpm = 60_000_000 / period_us;
    (erpm, Some(period_us))
}

/// DShot speed variants
#[derive(Clone, Copy, Debug)]
pub enum DshotSpeed {
    DShot150,
    DShot300,
    DShot600,
    DShot1200,
}

impl DshotSpeed {
    /// Get the baud rate in bits per second
    pub const fn baud_rate(self) -> u32 {
        match self {
            Self::DShot150 => 150_000,
            Self::DShot300 => 300_000,
            Self::DShot600 => 600_000,
            Self::DShot1200 => 1_200_000,
        }
    }

    /// TX PIO clock divider (8 PIO cycles per bit)
    fn tx_pio_clock_divider(self) -> FixedU32<U8> {
        let sys_clock = clk_sys_freq() as u64;
        FixedU32::<U8>::from_bits(((sys_clock << 8) / (8 * self.baud_rate() as u64)) as u32)
    }

    /// Bidir PIO clock divider
    /// Target PIO clock = 12MHz * speed/300kHz (e.g. DShot600 = 24MHz)
    /// 32 PIO cycles per bit in the bidir TX phase
    fn bidir_pio_clock_divider(self) -> FixedU32<U8> {
        let sys_clock = clk_sys_freq() as u64;
        let target = 12_000_000u64 * self.baud_rate() as u64 / 300_000;
        FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
    }
}

/// Convert raw command value (0-47) to Command enum safely
pub fn raw_to_command(cmd: u16) -> Option<Command> {
    match cmd {
        0 => Some(Command::MotorStop),
        1 => Some(Command::Beep1),
        2 => Some(Command::Beep2),
        3 => Some(Command::Beep3),
        4 => Some(Command::Beep4),
        5 => Some(Command::Beep5),
        6 => Some(Command::ESCInfo),
        7 => Some(Command::SpinDirection1),
        8 => Some(Command::SpinDirection2),
        9 => Some(Command::ThreeDModeOn),
        10 => Some(Command::ThreeDModeOff),
        11 => Some(Command::SettingsRequest),
        12 => Some(Command::SettingsSave),
        13 => Some(Command::ExtendedTelemetryEnable),
        14 => Some(Command::ExtendedTelemetryDisable),
        20 => Some(Command::SpinDirectionNormal),
        21 => Some(Command::SpinDirectonReversed),
        22 => Some(Command::Led0On),
        23 => Some(Command::Led1On),
        24 => Some(Command::Led2On),
        25 => Some(Command::Led3On),
        26 => Some(Command::Led0Off),
        27 => Some(Command::Led1Off),
        28 => Some(Command::Led2Off),
        29 => Some(Command::Led3Off),
        30 => Some(Command::AudioStreamModeToggle),
        31 => Some(Command::SilentModeToggle),
        32 => Some(Command::SignalLineTelemetryEnable),
        33 => Some(Command::SignalLineTelemetryDisable),
        34 => Some(Command::SignalLineContinuousERPMTelemetry),
        35 => Some(Command::SignalLineContinuousERPMPeriodTelemetry),
        42 => Some(Command::SignalLineTemperatureTelemetry),
        43 => Some(Command::SignalLineVoltageTelemetry),
        44 => Some(Command::SignalLineCurrentTelemetry),
        45 => Some(Command::SignalLineConsumptionTelemetry),
        46 => Some(Command::SignalLineERPMTelemetry),
        47 => Some(Command::SignalLineERPMPeriodTelemetry),
        _ => None, // 15-19, 36-41 are unassigned
    }
}

/// Build a DShot frame from a raw command value (0-2047)
fn make_command_frame(cmd: u16) -> Result<Frame<NormalDshot>, DshotError> {
    if cmd < 48 {
        let command = raw_to_command(cmd).ok_or(DshotError::InvalidThrottle)?;
        Ok(Frame::<NormalDshot>::command(command, false))
    } else {
        Frame::<NormalDshot>::new(cmd.saturating_sub(48), false)
            .ok_or(DshotError::InvalidThrottle)
    }
}

/// DShot TX PIO program (8 PIO cycles per bit)
///
/// Frame data must be in the LOWER 16 bits of the 32-bit word pushed to TX FIFO.
/// `out null, 16` discards the upper 16 zero bits, positioning the frame for
/// MSB-first output via subsequent `out` instructions.
macro_rules! dshot_tx_program {
    () => {
        pio_asm!(
            "set pindirs, 1",           // Configure pin as output
            "entry:",
            "    pull block",           // Wait for data
            "    out null, 16",         // Discard upper 16 bits
            "    set x, 15",            // 16 bits to send
            "loop:",
            "    set pins, 1",          // Start bit HIGH
            "    out y, 1",             // Get next bit
            "    jmp !y, zero",         // Branch if bit is 0
            "    nop [2]",              // bit1: extra HIGH time (3+3=6 cycles HIGH)
            "one:",
            "    set pins, 0",          // End HIGH
            "    jmp x--, loop",        // Next bit
            "    jmp reset",
            "zero:",
            "    set pins, 0 [3]",      // bit0: end HIGH early + delay (3 cycles HIGH)
            "    jmp x--, loop",        // Next bit
            "    jmp reset",
            "reset:",
            "    nop [31]",             // Inter-frame gap
            "    nop [31]",
            "    nop [31]",
            "    jmp entry [31]",
        )
    };
}

// ==================== DshotPio (unidirectional TX, 1-4 motors) ====================

/// DShot PIO driver for Embassy RP (1-4 motors)
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

                cfg.clock_divider = speed.tx_pio_clock_divider();
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
        impl<'d, PIO: Instance> super::DshotPioTrait<$n> for DshotPio<'d, $n, PIO> {
            fn command(&mut self, command: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = make_command_frame(command[$idx])?;
                    self.pio_instance.$sm.tx().push(frame.inner() as u32);
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
                    self.pio_instance.$sm.tx().push(frame.inner() as u32);
                )+
            }

            fn throttle_clamp(&mut self, throttle: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = Frame::<NormalDshot>::new(throttle[$idx].min(1999), false)
                        .ok_or(DshotError::InvalidThrottle)?;
                    self.pio_instance.$sm.tx().push(frame.inner() as u32);
                )+
                Ok(())
            }

            fn throttle_idle(&mut self) {
                let frame = Frame::<NormalDshot>::new(THROTTLE_IDLE, false)
                    .expect("Idle throttle should always be valid");
                $(
                    self.pio_instance.$sm.tx().push(frame.inner() as u32);
                )+
            }

            fn send_command(&mut self, cmd: Command) {
                let frame = Frame::<NormalDshot>::command(cmd, false);
                $(
                    self.pio_instance.$sm.tx().push(frame.inner() as u32);
                )+
            }

            fn send_command_repeated(&mut self, cmd: Command, count: u8) {
                for _ in 0..count {
                    self.send_command(cmd);
                }
            }
        }

        impl<'d, PIO: Instance> super::DshotPioAsync<$n> for DshotPio<'d, $n, PIO> {
            async fn command_async(&mut self, command: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = make_command_frame(command[$idx])?;
                    self.pio_instance.$sm.tx().wait_push(frame.inner() as u32).await;
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
                    self.pio_instance.$sm.tx().wait_push(frame.inner() as u32).await;
                )+
            }

            async fn throttle_async(&mut self, throttle: [u16; $n]) -> Result<(), DshotError> {
                $(
                    let frame = Frame::<NormalDshot>::new(throttle[$idx].min(1999), false)
                        .ok_or(DshotError::InvalidThrottle)?;
                    self.pio_instance.$sm.tx().wait_push(frame.inner() as u32).await;
                )+
                Ok(())
            }

            async fn throttle_idle_async(&mut self) {
                let frame = Frame::<NormalDshot>::new(THROTTLE_IDLE, false)
                    .expect("Idle throttle should always be valid");
                $(
                    self.pio_instance.$sm.tx().wait_push(frame.inner() as u32).await;
                )+
            }

            async fn send_command_async(&mut self, cmd: Command) {
                let frame = Frame::<NormalDshot>::command(cmd, false);
                $(
                    self.pio_instance.$sm.tx().wait_push(frame.inner() as u32).await;
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

// ==================== BidirDshotPio (single ESC, bidirectional) ====================

/// Bidirectional DShot PIO driver for single ESC with telemetry support
///
/// Uses a single PIO program with both TX and RX phases, based on the
/// pico-bidir-dshot reference implementation. The ESC responds with
/// GCR-encoded eRPM telemetry on the same signal wire.
///
/// **Supported speeds:** DShot150, DShot300, DShot600.
/// DShot1200 is **not supported** for bidirectional mode — the RX pulse-width
/// measurement loop cannot resolve the shorter bit periods at 1.2Mbit/s.
/// Use `DshotPio` (unidirectional) for DShot1200.
pub struct BidirDshotPio<'a, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
    #[allow(dead_code)] // retained for future set_speed() / release()
    pin: Pin<'a, PIO>,
    #[allow(dead_code)]
    speed: DshotSpeed,
    origin: u8,
}

impl<'a, PIO: Instance> BidirDshotPio<'a, PIO> {
    /// Create a new bidirectional DShot driver for a single ESC
    ///
    /// # Panics
    ///
    /// Panics if `speed` is `DshotSpeed::DShot1200` — bidirectional mode is not
    /// supported at 1.2Mbit/s because the PIO RX pulse-width measurement loop
    /// cannot resolve the shorter bit periods.
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        speed: DshotSpeed,
    ) -> Self {
        assert!(
            !matches!(speed, DshotSpeed::DShot1200),
            "DShot1200 is not supported for bidirectional mode"
        );

        let mut pio = Pio::new(pio, irq);
        let mut pin = pio.common.make_pio_pin(pin0);

        // Pull-up for bidirectional DShot (ESC pulls LOW for data)
        pin.set_pull(Pull::Up);

        // Bidirectional DShot PIO program based on pico-bidir-dshot reference.
        //
        // Program layout (offsets from origin):
        //   origin + 0: push block     (pushes previous RX data)
        //   origin + 1: set pindirs, 1 (pin as output)
        //   origin + 2: pull block     (waits for TX frame — idle position)
        //
        // TX Phase (32 cycles per bit):
        //   14 cycles LOW, 14 cycles data bit (inverted), 11 cycles HIGH, 1 jmp
        //   Pin direction switched inside PIO via set pindirs
        //
        // RX Phase (pulse-width measurement):
        //   Wait for falling edge, measure pulse widths using counting loops.
        //   21 GCR-encoded bits decoded to 16-bit telemetry + CRC.
        //
        //   The wait loop uses a tight 2-cycle poll (matching the reference
        //   pico-bidir-dshot implementation) so the first pulse measurement
        //   starts within 0-2 PIO cycles of the falling edge. A loose wait
        //   loop (e.g. 30 cycles/iter) would consume up to 94% of the first
        //   bit period at DShot600, desynchronizing the entire GCR decode.
        //   No PIO-level timeout is needed: the software `with_timeout` +
        //   `sync_pc()` handles the no-response case.
        let prg = pio_asm!(
            ".wrap_target"
            "push block"                    // Push any pending RX data
            "set pindirs, 1"                // Pin as output
            "pull block"                    // Pull TX frame (inverted)

            // TX Phase: send 16-bit DShot frame
            "out null, 16"                  // Discard upper 16 bits (zeros)
            "tx_bit:"
            "set pins, 0 [13]"              // 14 cycles LOW
            "out pins, 1 [13]"              // 14 cycles: output data bit
            "set pins, 1 [10]"              // 11 cycles HIGH
            "jmp !osre, tx_bit"             // Loop until OSR empty (1 cycle)

            // Prepare for RX
            "set x, 20"                     // 21 bits to receive
            "mov osr, ~null"                // OSR = 0xFFFFFFFF (source of 1s)
            "set pindirs, 0"                // Pin as input

            // Wait for falling edge (tight loop — 2 cycles per check)
            "wait_for_pin:"
            "jmp pin, wait_for_pin [1]"     // Loop while pin HIGH

            // RX Phase: pulse-width measurement
            "new_zero:"
            "set y, 6"                      // 7 iterations (first measurement)
            "jmp meas_zero"

            "another_zero:"
            "set y, 13 [1]"                 // 14 iterations (continuing)

            "meas_zero:"
            "jmp pin, new_one"              // If HIGH, transition to measuring HIGH
            "jmp y--, meas_zero"            // Keep measuring LOW
            "in null, 1"                    // Timeout: long LOW = shift in 0
            "jmp x--, another_zero"         // Next bit
            "jmp done"                      // All bits received

            "new_one:"
            "set y, 6 [1]"                  // 7 iterations
            "jmp meas_one"

            "another_one:"
            "set y, 13 [1]"                 // 14 iterations

            "meas_one:"
            "jmp pin, cont_one"             // Still HIGH, continue measuring
            "jmp new_zero"                  // Went LOW, short HIGH pulse (no shift)
            "cont_one:"
            "jmp y--, meas_one"             // Keep measuring HIGH
            "in osr, 1"                     // Timeout: long HIGH = shift in 1
            "jmp x--, another_one"          // Next bit

            "done:"
            ".wrap"
        );

        let mut cfg = Config::default();
        let loaded = pio.common.load_program(&prg.program);
        let origin = loaded.origin;
        cfg.use_program(&loaded, &[]);

        cfg.clock_divider = speed.bidir_pio_clock_divider();

        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            direction: ShiftDirection::Left,
            threshold: 32,
        };
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            direction: ShiftDirection::Left,
            threshold: 32,
        };

        cfg.fifo_join = FifoJoin::Duplex;

        // All pin config points to the same signal pin
        cfg.set_jmp_pin(&pin);
        cfg.set_set_pins(&[&pin]);
        cfg.set_out_pins(&[&pin]);
        cfg.set_in_pins(&[&pin]);

        pio.sm0.set_config(&cfg);
        pio.sm0.set_pin_dirs(Direction::Out, &[&pin]);
        pio.sm0.restart();
        pio.sm0.set_enable(true);
        // Reference sets clock divider AFTER enabling
        pio.sm0.set_clock_divider(speed.bidir_pio_clock_divider());

        Self {
            pio_instance: pio,
            pin,
            speed,
            origin,
        }
    }

    /// Sync the PIO program counter to ensure we're ready to send.
    ///
    /// If the SM is not at the expected position (pull block at origin+2),
    /// clear the ISR (to prevent stale partial RX data from contaminating
    /// the next frame) and force a jump to set pindirs (origin+1).
    fn sync_pc(&mut self) {
        let expected_pc = self.origin + 2;
        let current_pc = self.pio_instance.sm0.get_addr();

        if current_pc != expected_pc {
            // Clear ISR to discard any partial RX data from an interrupted frame.
            // MOV ISR, NULL = 0b101_00000_110_00_011 = 0xA0C3
            unsafe { self.pio_instance.sm0.exec_instr(0xA0C3) };

            // Construct unconditional JMP instruction: opcode 000, no delay, condition 000
            let jmp_instr = (self.origin + 1) as u16 & 0x1F;
            unsafe { self.pio_instance.sm0.exec_instr(jmp_instr) };
        }
    }

    /// Send a frame, read raw RX value, and decode telemetry
    async fn send_and_receive_raw(&mut self, frame_raw: u16) -> Result<u32, DshotError> {
        // Clear stale RX data
        while self.pio_instance.sm0.rx().try_pull().is_some() {}

        // Sync PC to expected position
        self.sync_pc();

        // Send inverted frame (bidir DShot requirement)
        let tx_data = !frame_raw as u32;

        // TX with timeout
        if with_timeout(
            Duration::from_millis(10),
            self.pio_instance.sm0.tx().wait_push(tx_data),
        )
        .await
        .is_err()
        {
            return Err(DshotError::TelemetryTimeout);
        }

        // RX with timeout (~500us should be plenty for round-trip)
        let rx_data = with_timeout(
            Duration::from_micros(500),
            self.pio_instance.sm0.rx().wait_pull(),
        )
        .await
        .map_err(|_| DshotError::TelemetryTimeout)?;

        if rx_data == 0 {
            return Err(DshotError::TelemetryTimeout);
        }

        Ok(rx_data)
    }

    fn decode_telemetry(rx_data: u32) -> Result<Telemetry, DshotError> {
        // Decode GCR (21 bits -> 16 bits)
        let raw_16 = gcr_decode(rx_data).ok_or(DshotError::GcrDecodeError)?;

        // Verify CRC: XOR all 4 nibbles should equal 0x0F
        if !verify_telemetry_crc(raw_16) {
            return Err(DshotError::InvalidTelemetryCrc);
        }

        // Extract 12-bit eRPM data (remove checksum nibble)
        let erpm_12 = raw_16 >> 4;
        let (erpm, period_us) = telemetry_to_erpm(erpm_12);

        Ok(Telemetry { erpm, period_us })
    }

    /// Send a frame and read telemetry response
    async fn send_and_read_telemetry(&mut self, frame_raw: u16) -> Result<Telemetry, DshotError> {
        let rx_data = self.send_and_receive_raw(frame_raw).await?;
        Self::decode_telemetry(rx_data)
    }

    /// Send throttle and read raw RX data + decoded telemetry
    ///
    /// Returns `(raw_rx_data, Result<Telemetry, DshotError>)`.
    /// Useful for debugging GCR decode issues.
    pub async fn throttle_with_telemetry_raw(&mut self, throttle: u16) -> Result<(u32, Result<Telemetry, DshotError>), DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(throttle, true)
            .ok_or(DshotError::InvalidThrottle)?;
        let rx_data = self.send_and_receive_raw(frame.inner()).await?;
        Ok((rx_data, Self::decode_telemetry(rx_data)))
    }

    /// Send throttle command and read telemetry response
    ///
    /// Returns eRPM telemetry if the ESC responds, or an error if:
    /// - Throttle value is invalid
    /// - ESC doesn't respond (timeout)
    /// - Response has invalid GCR encoding
    /// - Response CRC check fails
    pub async fn throttle_with_telemetry(&mut self, throttle: u16) -> Result<Telemetry, DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(throttle, true)
            .ok_or(DshotError::InvalidThrottle)?;
        self.send_and_read_telemetry(frame.inner()).await
    }

    /// Send a DShot command and read telemetry response
    ///
    /// Use this to send commands like `Command::MotorStop` while reading
    /// back eRPM telemetry from the ESC.
    pub async fn command_with_telemetry(&mut self, cmd: Command) -> Result<Telemetry, DshotError> {
        let frame = Frame::<BidirectionalDshot>::command(cmd, true);
        self.send_and_read_telemetry(frame.inner()).await
    }

    /// Send a DShot command (0-47) in bidirectional mode
    ///
    /// Use `Command::MotorStop` for arming/disarming the ESC.
    pub fn send_command(&mut self, cmd: Command) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::command(cmd, false);
        self.pio_instance.sm0.tx().push(!frame.inner() as u32);
    }

    /// Send a DShot command repeatedly in bidirectional mode
    ///
    /// Many DShot commands require being sent multiple consecutive times to
    /// take effect (typically 6 for settings, 10 for beep protocol).
    ///
    /// A 300us delay between sends ensures the PIO TX+RX cycle completes
    /// before the next frame, preventing `sync_pc()` from interrupting
    /// a frame in progress.
    pub async fn send_command_repeated_async(&mut self, cmd: Command, count: u8) {
        for _ in 0..count {
            self.send_command_async(cmd).await;
            Timer::after(Duration::from_micros(300)).await;
        }
    }

    /// Send a DShot command asynchronously in bidirectional mode
    pub async fn send_command_async(&mut self, cmd: Command) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::command(cmd, false);
        self.pio_instance.sm0.tx().wait_push(!frame.inner() as u32).await;
    }

    /// Send idle throttle (DShot value 48, bidirectional mode)
    ///
    /// The motor may creep slightly. Use `send_command(Command::MotorStop)` to fully stop.
    pub fn throttle_idle(&mut self) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::new(THROTTLE_IDLE, false)
            .expect("Idle throttle should always be valid");
        self.pio_instance.sm0.tx().push(!frame.inner() as u32);
    }

    /// Send idle throttle asynchronously (DShot value 48)
    ///
    /// The motor may creep slightly. Use `send_command_async(Command::MotorStop)` to fully stop.
    pub async fn throttle_idle_async(&mut self) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::new(THROTTLE_IDLE, false)
            .expect("Idle throttle should always be valid");
        self.pio_instance.sm0.tx().wait_push(!frame.inner() as u32).await;
    }

    /// Arm ESC by sending `MotorStop` at ~1kHz for the given duration
    ///
    /// ESCs require continuous `MotorStop` frames for 1-2 seconds before
    /// they accept throttle commands. This convenience method replaces the
    /// common arming loop pattern.
    pub async fn arm_async(&mut self, duration: Duration) {
        let iterations = duration.as_millis() as u32;
        for _ in 0..iterations {
            self.send_command_async(Command::MotorStop).await;
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Send throttle command asynchronously (no telemetry)
    pub async fn throttle_async(&mut self, throttle: u16) -> Result<(), DshotError> {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::new(throttle.min(1999), false)
            .ok_or(DshotError::InvalidThrottle)?;
        self.pio_instance.sm0.tx().wait_push(!frame.inner() as u32).await;
        Ok(())
    }

    /// Send throttle and read Extended DShot Telemetry (EDT) response
    ///
    /// Sends the throttle value with telemetry request bit set, then decodes
    /// the response as self-describing EDT. The telemetry type is determined
    /// from the frame prefix, not from any prior command.
    ///
    /// EDT must be enabled first by sending `Command::ExtendedTelemetryEnable`
    /// at least 6 times (use `send_command_repeated_async`).
    ///
    /// The ESC interleaves normal eRPM frames with EDT frames (temperature,
    /// voltage, current, etc.), so callers should collect multiple samples.
    pub async fn read_extended_telemetry(
        &mut self,
        throttle: u16,
    ) -> Result<ExtendedTelemetry, DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(throttle, true)
            .ok_or(DshotError::InvalidThrottle)?;
        let rx_data = self.send_and_receive_raw(frame.inner()).await?;

        // Decode GCR
        let raw_16 = gcr_decode(rx_data).ok_or(DshotError::GcrDecodeError)?;

        // Verify CRC
        if !verify_telemetry_crc(raw_16) {
            return Err(DshotError::InvalidTelemetryCrc);
        }

        // Extract 12-bit data (remove checksum nibble)
        let data_12 = raw_16 >> 4;
        Ok(decode_extended_telemetry(data_12))
    }
}
