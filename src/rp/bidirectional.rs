use super::{telemetry_to_erpm, DshotSpeed, THROTTLE_IDLE};
use crate::{
    decode_extended_telemetry, gcr_decode, verify_telemetry_crc, Command, DshotError,
    ExtendedTelemetry, Telemetry,
};
use dshot_frame::{BidirectionalDshot, Frame};
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pin, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::Peri;
use embassy_time::{with_timeout, Duration, Timer};
use fixed::types::extra::U8;
use fixed::FixedU32;

/// PIO clock divider for bidirectional `DShot`.
///
/// The TX phase of the PIO program below spends 40 PIO cycles per `DShot` bit,
/// so the state machine must run at `40 * baud_rate`.
#[allow(clippy::cast_possible_truncation, clippy::cast_lossless)]
const fn bidir_pio_clock_divider(speed: DshotSpeed, sys_clock_hz: u32) -> FixedU32<U8> {
    let sys_clock = sys_clock_hz as u64;
    let target = 40 * speed.baud_rate() as u64;
    FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
}

/// Worst-case duration of one bidirectional `DShot` TX+RX cycle, in microseconds.
///
/// A frame is 16 bits at the nominal baud rate; the ESC replies with 21 GCR bits
/// at 5/4 of that rate, after a turnaround gap the spec puts at ~30us. Rounded up.
#[allow(clippy::cast_possible_truncation)]
const fn bidir_cycle_us(speed: DshotSpeed) -> u64 {
    let baud = speed.baud_rate() as u64;
    let tx_us = (16u64 * 1_000_000).div_ceil(baud);
    let rx_us = (21u64 * 4 * 1_000_000).div_ceil(baud * 5);
    tx_us + rx_us + TURNAROUND_US
}

/// Turnaround gap between the end of a frame and the start of the ESC reply.
const TURNAROUND_US: u64 = 30;

/// How many full cycles to wait for TX FIFO space before declaring the state
/// machine wedged.
///
/// Note this is *not* a FIFO-depth argument. The FIFO is 4 deep in
/// [`FifoJoin::Duplex`], but this driver cannot use that depth: every push
/// first calls [`BidirDshotPio::sync_pc`], which resets the state machine
/// unless it is parked at the `pull block` idle instruction — and it is only
/// parked there when the FIFO is empty. Queueing a second frame while the
/// first is still on the wire therefore aborts the first mid-transmission.
/// The driver is strictly one frame at a time, and callers must pace
/// themselves at or below one frame per [`bidir_cycle_us`].
///
/// So in correct use `wait_push` returns immediately and this bound is a pure
/// wedge detector. A few cycles of slack keeps it clear of scheduler jitter
/// while staying ~20x below the 10ms it replaced.
const TX_WEDGE_CYCLES: u64 = 4;

/// Bidirectional `DShot` program loaded into PIO instruction memory.
///
/// Create once per PIO block and share between up to 4 [`BidirDshotPio`]
/// instances. Mirrors `embassy-rp` patterns like `PioUartTxProgram`.
pub struct BidirDshotProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> BidirDshotProgram<'a, PIO> {
    /// Load the Bidirectional `DShot` program into PIO instruction memory,
    /// call this once per PIO block.
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        // Bidirectional DShot PIO program based on pico-bidir-dshot reference.
        //
        // Program layout (offsets from origin):
        //   origin + 0: push block     (pushes previous RX data)
        //   origin + 1: set pindirs, 1 (pin as output)
        //   origin + 2: pull block     (waits for TX frame — idle position)
        //
        // TX Phase (40 cycles per bit):
        //   14 cycles driven LOW, 14 cycles driving the (inverted) data bit,
        //   11 cycles driven HIGH, 1 cycle for the loop jmp.
        //   The middle phase follows the data, so total HIGH time per bit varies.
        //   40 cycles/bit is what bidir_pio_clock_divider encodes.
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
        Self {
            prg: common.load_program(&prg.program),
        }
    }
}

/// Bidirectional `DShot` PIO driver for single ESC with telemetry.
///
/// Supports `DShot150`, `DShot300`, `DShot600`. `DShot1200` is not supported
/// (panics at construction).
pub struct BidirDshotPio<'a, PIO: Instance, const SM: usize> {
    sm: StateMachine<'a, PIO, SM>,
    _pin: Pin<'a, PIO>,
    origin: u8,
    /// Derived from `speed`: how long to wait for TX FIFO space before
    /// reporting the state machine wedged.
    tx_timeout: Duration,
}

impl<'a, PIO: Instance, const SM: usize> BidirDshotPio<'a, PIO, SM> {
    /// # Panics
    ///
    /// Panics if `speed` is `DshotSpeed::DShot1200`.
    pub fn new(
        mut sm: StateMachine<'a, PIO, SM>,
        common: &mut Common<'a, PIO>,
        pin0: Peri<'a, impl PioPin + 'a>,
        program: &BidirDshotProgram<'a, PIO>,
        speed: DshotSpeed,
    ) -> Self {
        assert!(
            !matches!(speed, DshotSpeed::DShot1200),
            "DShot1200 is not supported for bidirectional mode"
        );

        let mut pin = common.make_pio_pin(pin0);

        pin.set_pull(Pull::Up);

        let mut cfg = Config::default();
        let origin = program.prg.origin;
        cfg.use_program(&program.prg, &[]);

        let clock_divider = bidir_pio_clock_divider(speed, clk_sys_freq());
        cfg.clock_divider = clock_divider;

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

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&pin]);
        sm.restart();
        sm.set_enable(true);
        sm.set_clock_divider(clock_divider);

        Self {
            sm,
            _pin: pin,
            origin,
            tx_timeout: Duration::from_micros(bidir_cycle_us(speed) * TX_WEDGE_CYCLES),
        }
    }

    /// Reset PIO to the pull-block position if it drifted (e.g. telemetry timeout).
    fn sync_pc(&mut self) {
        let expected_pc = self.origin + 2;
        let current_pc = self.sm.get_addr();

        if current_pc != expected_pc {
            // Clear ISR to discard any partial RX data from an interrupted frame.
            // MOV ISR, NULL = 0b101_00000_110_00_011 = 0xA0C3
            unsafe { self.sm.exec_instr(0xA0C3) };

            // Construct unconditional JMP instruction: opcode 000, no delay, condition 000
            let jmp_instr = u16::from(self.origin + 1) & 0x1F;
            unsafe { self.sm.exec_instr(jmp_instr) };
        }
    }

    /// Push a frame, bounded by [`Self::tx_timeout`].
    ///
    /// The TX FIFO is 4 deep, so this only ever blocks when the caller is
    /// running ahead of the wire; timing out means the state machine stopped
    /// consuming frames, not that the caller was merely early.
    async fn push_frame_async(&mut self, frame_raw: u16) -> Result<(), DshotError> {
        while self.sm.rx().try_pull().is_some() {}
        self.sync_pc();
        let tx_data = u32::from(!frame_raw); // bidir DShot sends inverted
        with_timeout(self.tx_timeout, self.sm.tx().wait_push(tx_data))
            .await
            .map_err(|_| DshotError::TxBusy)
    }

    /// Push a frame without blocking, reporting a full TX FIFO rather than
    /// letting the hardware discard the write.
    fn push_frame(&mut self, frame_raw: u16) -> Result<(), DshotError> {
        while self.sm.rx().try_pull().is_some() {}
        self.sync_pc();
        let tx_data = u32::from(!frame_raw); // bidir DShot sends inverted
        if self.sm.tx().try_push(tx_data) {
            Ok(())
        } else {
            Err(DshotError::TxBusy)
        }
    }

    /// Send a frame, read raw RX value, and decode telemetry
    async fn send_and_receive_raw(&mut self, frame_raw: u16) -> Result<u32, DshotError> {
        // Clears stale RX, resyncs the PC, and bounds the push.
        self.push_frame_async(frame_raw).await?;

        // Deliberately a flat 500us rather than a multiple of bidir_cycle_us:
        // it must cover TX + turnaround + reply, which is 249us at the slowest
        // supported speed, and this is the one bound validated against real
        // ESCs. Deriving it would tighten DShot600 from 500us to 340us with no
        // hardware to confirm that is safe.
        let rx_data = with_timeout(Duration::from_micros(500), self.sm.rx().wait_pull())
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
    pub async fn throttle_with_telemetry_raw(
        &mut self,
        throttle: u16,
    ) -> Result<(u32, Result<Telemetry, DshotError>), DshotError> {
        let frame =
            Frame::<BidirectionalDshot>::new(throttle, true).ok_or(DshotError::InvalidThrottle)?;
        let rx_data = self.send_and_receive_raw(frame.inner()).await?;
        Ok((rx_data, Self::decode_telemetry(rx_data)))
    }

    /// Send throttle and read eRPM telemetry response.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle is out of range,
    /// or a telemetry error if the ESC does not respond.
    pub async fn throttle_with_telemetry(
        &mut self,
        throttle: u16,
    ) -> Result<Telemetry, DshotError> {
        let frame =
            Frame::<BidirectionalDshot>::new(throttle, true).ok_or(DshotError::InvalidThrottle)?;
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

    /// Queue a `DShot` command without waiting.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::TxBusy` if the TX FIFO is full, meaning the frame
    /// was not queued.
    pub fn send_command(&mut self, cmd: Command) -> Result<(), DshotError> {
        let frame = Frame::<BidirectionalDshot>::command(cmd, false);
        self.push_frame(frame.inner())
    }

    /// Send a `DShot` command repeatedly (6x for settings, 10x for beep).
    /// 300us delay between sends to let the PIO TX+RX cycle complete.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::TxBusy` on the first frame the state machine fails
    /// to accept; earlier frames in the sequence have already been sent.
    pub async fn send_command_repeated_async(
        &mut self,
        cmd: Command,
        count: u8,
    ) -> Result<(), DshotError> {
        for _ in 0..count {
            self.send_command_async(cmd).await?;
            Timer::after(Duration::from_micros(300)).await;
        }
        Ok(())
    }

    /// # Errors
    ///
    /// Returns `DshotError::TxBusy` if the state machine does not accept the
    /// frame within one TX FIFO drain.
    pub async fn send_command_async(&mut self, cmd: Command) -> Result<(), DshotError> {
        let frame = Frame::<BidirectionalDshot>::command(cmd, false);
        self.push_frame_async(frame.inner()).await
    }

    /// # Errors
    ///
    /// Returns `DshotError::TxBusy` if the TX FIFO is full, meaning the frame
    /// was not queued.
    ///
    /// # Panics
    ///
    /// Panics if the idle throttle frame cannot be constructed (should never happen).
    pub fn throttle_idle(&mut self) -> Result<(), DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(THROTTLE_IDLE, false)
            .expect("Idle throttle should always be valid");
        self.push_frame(frame.inner())
    }

    /// # Errors
    ///
    /// Returns `DshotError::TxBusy` if the state machine does not accept the
    /// frame within one TX FIFO drain.
    ///
    /// # Panics
    ///
    /// Panics if the idle throttle frame cannot be constructed (should never happen).
    pub async fn throttle_idle_async(&mut self) -> Result<(), DshotError> {
        let frame = Frame::<BidirectionalDshot>::new(THROTTLE_IDLE, false)
            .expect("Idle throttle should always be valid");
        self.push_frame_async(frame.inner()).await
    }

    /// Arm ESC by sending `MotorStop` at ~1kHz for the given duration.
    ///
    /// # Errors
    ///
    /// Returns `DshotError::TxBusy` if the state machine stops accepting frames
    /// part way through the arming sequence, which leaves the ESC unarmed.
    pub async fn arm_async(&mut self, duration: Duration) -> Result<(), DshotError> {
        #[allow(clippy::cast_possible_truncation)]
        let iterations = duration.as_millis() as u32;
        for _ in 0..iterations {
            self.send_command_async(Command::MotorStop).await?;
            Timer::after(Duration::from_millis(1)).await;
        }
        Ok(())
    }

    /// # Errors
    ///
    /// Returns `DshotError::InvalidThrottle` if throttle is out of range, or
    /// `DshotError::TxBusy` if the state machine does not accept the frame
    /// within one TX FIFO drain.
    pub async fn throttle_async(&mut self, throttle: u16) -> Result<(), DshotError> {
        // Not clamped: a caller that computes 60000 from a bad cast means a
        // fault, not full throttle, and `throttle_with_telemetry` already
        // rejects the same input. Clamping here contradicted the documented
        // `InvalidThrottle` and silently picked the most dangerous value.
        let frame =
            Frame::<BidirectionalDshot>::new(throttle, false).ok_or(DshotError::InvalidThrottle)?;
        self.push_frame_async(frame.inner()).await
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
        let frame =
            Frame::<BidirectionalDshot>::new(throttle, true).ok_or(DshotError::InvalidThrottle)?;
        let rx_data = self.send_and_receive_raw(frame.inner()).await?;
        let raw_16 = gcr_decode(rx_data).ok_or(DshotError::GcrDecodeError)?;
        if !verify_telemetry_crc(raw_16) {
            return Err(DshotError::InvalidTelemetryCrc);
        }
        let data_12 = raw_16 >> 4;
        Ok(decode_extended_telemetry(data_12))
    }
}

#[cfg(test)]
mod tests {
    use super::{bidir_cycle_us, bidir_pio_clock_divider, TX_WEDGE_CYCLES};
    use crate::DshotSpeed;

    /// The cycle bound must cover a real frame: 16 bits out at the nominal
    /// rate, 21 GCR bits back at 5/4 of it, plus the turnaround gap.
    #[test]
    fn cycle_bound_covers_a_full_frame() {
        // DShot300: 53.3us out + 56us back + 30us gap.
        assert_eq!(bidir_cycle_us(DshotSpeed::DShot300), 54 + 56 + 30);
        // DShot600 halves the wire time.
        assert_eq!(bidir_cycle_us(DshotSpeed::DShot600), 27 + 28 + 30);
        // DShot150 doubles it.
        assert_eq!(bidir_cycle_us(DshotSpeed::DShot150), 107 + 112 + 30);
    }

    /// Slower speeds must get longer bounds, or the timeout fires on traffic
    /// that was always going to be slow.
    #[test]
    fn cycle_bound_grows_as_speed_drops() {
        assert!(bidir_cycle_us(DshotSpeed::DShot150) > bidir_cycle_us(DshotSpeed::DShot300));
        assert!(bidir_cycle_us(DshotSpeed::DShot300) > bidir_cycle_us(DshotSpeed::DShot600));
    }

    /// The old hardcoded bound was 10ms. Every supported speed must now come in
    /// far under that, otherwise the fix for #7 changed nothing in practice.
    #[test]
    fn tx_timeout_is_well_under_the_old_10ms() {
        for speed in [
            DshotSpeed::DShot150,
            DshotSpeed::DShot300,
            DshotSpeed::DShot600,
        ] {
            let timeout_us = bidir_cycle_us(speed) * TX_WEDGE_CYCLES;
            assert!(timeout_us < 10_000 / 5, "{speed:?} -> {timeout_us}us");
        }
    }

    /// 40 PIO cycles/bit: DShot600 needs a 24MHz PIO clock, so at 125MHz the
    /// divider is 125/24 = 5.2083 -> 1333 in 24.8 fixed point.
    #[test]
    fn bidir_divider_at_125mhz() {
        let div = bidir_pio_clock_divider(DshotSpeed::DShot600, 125_000_000);
        assert_eq!(div.to_bits(), 1333, "raw 24.8 divider wrong");
        assert_eq!(div.to_bits() >> 8, 5, "integer part wrong");
    }

    #[test]
    fn bidir_divider_encodes_forty_cycles_per_bit() {
        // Pick a clock that divides exactly so the check is independent of rounding.
        let div = bidir_pio_clock_divider(DshotSpeed::DShot300, 120_000_000);
        let pio_clock = 120_000_000u64 * 256 / u64::from(div.to_bits());
        assert_eq!(pio_clock, 40 * 300_000);
    }
}
