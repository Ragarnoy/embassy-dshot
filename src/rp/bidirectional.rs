use crate::{DshotError, Telemetry, Command, ExtendedTelemetry, gcr_decode, verify_telemetry_crc, decode_extended_telemetry};
use dshot_frame::{Frame, BidirectionalDshot};
use embassy_rp::Peri;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Config, Direction, FifoJoin, Instance, InterruptHandler, Pio, Pin, PioPin, ShiftConfig,
    ShiftDirection,
};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_time::{with_timeout, Duration, Timer};
use super::{DshotSpeed, THROTTLE_IDLE, telemetry_to_erpm};

/// Bidirectional `DShot` PIO driver for single ESC with telemetry.
///
/// Supports `DShot150`, `DShot300`, `DShot600`. `DShot1200` is not supported
/// (panics at construction).
pub struct BidirDshotPio<'a, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
    _pin: Pin<'a, PIO>,
    origin: u8,
}

impl<'a, PIO: Instance> BidirDshotPio<'a, PIO> {
    /// # Panics
    ///
    /// Panics if `speed` is `DshotSpeed::DShot1200`.
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
        //
        // RX Phase (pulse-width measurement):
        //   Wait for falling edge, measure pulse widths using counting loops.
        //   21 GCR-encoded bits decoded to 16-bit telemetry + CRC.
        //   Tight 2-cycle wait loop matches reference implementation.
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

        cfg.set_jmp_pin(&pin);
        cfg.set_set_pins(&[&pin]);
        cfg.set_out_pins(&[&pin]);
        cfg.set_in_pins(&[&pin]);

        pio.sm0.set_config(&cfg);
        pio.sm0.set_pin_dirs(Direction::Out, &[&pin]);
        pio.sm0.restart();
        pio.sm0.set_enable(true);
        pio.sm0.set_clock_divider(speed.bidir_pio_clock_divider());

        Self {
            pio_instance: pio,
            _pin: pin,
            origin,
        }
    }

    /// Reset PIO to the pull-block position if it drifted (e.g. telemetry timeout).
    fn sync_pc(&mut self) {
        let expected_pc = self.origin + 2;
        let current_pc = self.pio_instance.sm0.get_addr();

        if current_pc != expected_pc {
            // Clear ISR to discard any partial RX data from an interrupted frame.
            // MOV ISR, NULL = 0b101_00000_110_00_011 = 0xA0C3
            unsafe { self.pio_instance.sm0.exec_instr(0xA0C3) };

            // Construct unconditional JMP instruction: opcode 000, no delay, condition 000
            let jmp_instr = u16::from(self.origin + 1) & 0x1F;
            unsafe { self.pio_instance.sm0.exec_instr(jmp_instr) };
        }
    }

    /// Send a frame, read raw RX value, and decode telemetry
    async fn send_and_receive_raw(&mut self, frame_raw: u16) -> Result<u32, DshotError> {
        // Clear stale RX data
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();

        let tx_data = u32::from(!frame_raw); // bidir DShot sends inverted

        if with_timeout(
            Duration::from_millis(10),
            self.pio_instance.sm0.tx().wait_push(tx_data),
        )
        .await
        .is_err()
        {
            return Err(DshotError::TelemetryTimeout);
        }

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
        if !verify_telemetry_crc(raw_16) {
            return Err(DshotError::InvalidTelemetryCrc);
        }
        let erpm_12 = raw_16 >> 4;
        let (erpm, period_us) = telemetry_to_erpm(erpm_12);

        Ok(Telemetry { erpm, period_us })
    }

    /// Send a frame and read telemetry response
    async fn send_and_read_telemetry(&mut self, frame_raw: u16) -> Result<Telemetry, DshotError> {
        let rx_data = self.send_and_receive_raw(frame_raw).await?;
        Self::decode_telemetry(rx_data)
    }

    /// Like `throttle_with_telemetry` but also returns the raw RX data for debugging.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle is out of range,
    /// or a telemetry error if the ESC does not respond.
    pub async fn throttle_with_telemetry_raw(&mut self, throttle: u16) -> Result<(u32, Result<Telemetry, DshotError>), DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(throttle, true)
            .ok_or(DshotError::InvalidThrottle)?;
        let rx_data = self.send_and_receive_raw(frame.inner()).await?;
        Ok((rx_data, Self::decode_telemetry(rx_data)))
    }

    /// Send throttle and read eRPM telemetry response.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle is out of range,
    /// or a telemetry error if the ESC does not respond.
    pub async fn throttle_with_telemetry(&mut self, throttle: u16) -> Result<Telemetry, DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(throttle, true)
            .ok_or(DshotError::InvalidThrottle)?;
        self.send_and_read_telemetry(frame.inner()).await
    }

    /// Send a `DShot` command and read telemetry response.
    ///
    /// # Errors
    ///
    /// Returns a telemetry error if the ESC does not respond or CRC fails.
    pub async fn command_with_telemetry(&mut self, cmd: Command) -> Result<Telemetry, DshotError> {
        let frame = Frame::<BidirectionalDshot>::command(cmd, true);
        self.send_and_read_telemetry(frame.inner()).await
    }

    pub fn send_command(&mut self, cmd: Command) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::command(cmd, false);
        self.pio_instance.sm0.tx().push(u32::from(!frame.inner()));
    }

    /// Send a `DShot` command repeatedly (6x for settings, 10x for beep).
    /// 300us delay between sends to let the PIO TX+RX cycle complete.
    pub async fn send_command_repeated_async(&mut self, cmd: Command, count: u8) {
        for _ in 0..count {
            self.send_command_async(cmd).await;
            Timer::after(Duration::from_micros(300)).await;
        }
    }

    pub async fn send_command_async(&mut self, cmd: Command) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::command(cmd, false);
        self.pio_instance.sm0.tx().wait_push(u32::from(!frame.inner())).await;
    }

    /// # Panics
    ///
    /// Panics if the idle throttle frame cannot be constructed (should never happen).
    pub fn throttle_idle(&mut self) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::new(THROTTLE_IDLE, false)
            .expect("Idle throttle should always be valid");
        self.pio_instance.sm0.tx().push(u32::from(!frame.inner()));
    }

    /// # Panics
    ///
    /// Panics if the idle throttle frame cannot be constructed (should never happen).
    pub async fn throttle_idle_async(&mut self) {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::new(THROTTLE_IDLE, false)
            .expect("Idle throttle should always be valid");
        self.pio_instance.sm0.tx().wait_push(u32::from(!frame.inner())).await;
    }

    /// Arm ESC by sending `MotorStop` at ~1kHz for the given duration.
    pub async fn arm_async(&mut self, duration: Duration) {
        #[allow(clippy::cast_possible_truncation)]
        let iterations = duration.as_millis() as u32;
        for _ in 0..iterations {
            self.send_command_async(Command::MotorStop).await;
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle is out of range.
    pub async fn throttle_async(&mut self, throttle: u16) -> Result<(), DshotError> {
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();
        let frame = Frame::<BidirectionalDshot>::new(throttle.min(1999), false)
            .ok_or(DshotError::InvalidThrottle)?;
        self.pio_instance.sm0.tx().wait_push(u32::from(!frame.inner())).await;
        Ok(())
    }

    /// Send throttle and read an EDT response.
    ///
    /// EDT must be enabled first (`Command::ExtendedTelemetryEnable`, 6x).
    /// The ESC interleaves eRPM and EDT frames, so collect multiple samples.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle is out of range,
    /// or a telemetry/GCR/CRC error if the response is invalid.
    pub async fn read_extended_telemetry(
        &mut self,
        throttle: u16,
    ) -> Result<ExtendedTelemetry, DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(throttle, true)
            .ok_or(DshotError::InvalidThrottle)?;
        let rx_data = self.send_and_receive_raw(frame.inner()).await?;
        let raw_16 = gcr_decode(rx_data).ok_or(DshotError::GcrDecodeError)?;
        if !verify_telemetry_crc(raw_16) {
            return Err(DshotError::InvalidTelemetryCrc);
        }
        let data_12 = raw_16 >> 4;
        Ok(decode_extended_telemetry(data_12))
    }
}
