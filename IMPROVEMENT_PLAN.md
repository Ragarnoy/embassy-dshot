# DShot-PIO Enhancement Plan

## Executive Summary

This document outlines a comprehensive plan to enhance the `dshot-pio` library by incorporating best practices and features from other leading DShot implementations, with a focus on adding bidirectional support, async/await capabilities, and extended telemetry while maintaining the library's Rust-native, Embassy-first design philosophy.

---

## Feature Comparison Matrix

### Current Implementation: dshot-pio (This Fork)

**Strengths:**
- ✅ Clean Rust API with trait-based design (`DshotPioTrait`)
- ✅ PIO-based hardware timing (precise, no CPU overhead)
- ✅ Support for 1-4 motors per PIO block
- ✅ Dual HAL support: `embassy-rp` and `rp2040-hal`
- ✅ Type-safe compile-time motor count configuration
- ✅ Zero-cost abstractions
- ✅ Const generics for flexibility

**Limitations:**
- ❌ **No bidirectional DShot support** (telemetry bit always `false`)
- ❌ **No async/await in Embassy** (uses blocking `.push()`)
- ❌ **No telemetry reading** (TX-only PIO program)
- ❌ **No DMA support** (CPU must push each packet)
- ❌ **Hardcoded 4 state machines** (wastes resources if using <4 motors)
- ❌ **No error handling** (blind writes to FIFO)
- ❌ **No examples or tests**
- ❌ **No jitter handling** for input signals

**Key Implementation Details:**
- PIO program: 14 instructions (TX only)
- Clock divider formula: `system_clock / (8 * dshot_speed * 1000)`
- Transmission timing: 6+2 cycles (bit 1), 3+5 cycles (bit 0)
- Blank frame: 128 cycles

---

### Reference: pico-dshot-bidir (josephduchesne)

**Strengths:**
- ✅ **Bidirectional DShot** on single wire
- ✅ **Extended telemetry support**
- ✅ PIO-based hardware timing
- ✅ Multiple DShot speeds (300, 600, 1200)
- ✅ Low CPU overhead
- ✅ Compatible with BLHeli32 v32.10+

**Limitations:**
- ❌ C++/Arduino framework (not Rust)
- ❌ No async support
- ❌ Blocking API only

**Key Features to Adopt:**
1. **Bidirectional protocol implementation**
2. **Telemetry packet parsing**
3. **Single-wire TX/RX switching**
4. **Extended DShot Telemetry (EDT) support**

**Implementation Insights:**
- PIO handles pin direction switching (output → input)
- Wait period for ESC response after TX
- Telemetry frame decoding in software

---

### Reference: Pico_Bidir_DShot (Bastian2001)

**Strengths:**
- ✅ **Jitter-resistant edge detection** (hardware-based)
- ✅ **Very efficient PIO programs**:
  - Unidirectional: 4 instructions per 4 ESCs
  - Bidirectional: 28 instructions per ESC
- ✅ **High scalability**: Up to 30-48 ESCs (TX only) or 8-12 ESCs (bidirectional)
- ✅ **Hardware edge detection** via oversampling
- ✅ **Extended telemetry**: eRPM, temperature, voltage, current
- ✅ **Fully asynchronous operation** (PIO-level)
- ✅ **Clock mismatch tolerance**

**Limitations:**
- ❌ C++/Arduino framework (not Rust)
- ❌ GPL 3.0 license (requires careful consideration)

**Key Features to Adopt:**
1. **Hardware-based edge detection** for jitter resistance
2. **Oversampling technique** for reliable RX
3. **Efficient PIO instruction count**
4. **Multiple telemetry types** (basic eRPM + EDT)
5. **eRPM → RPM conversion** formula: `RPM = eRPM / (motor_poles / 2)`

**Implementation Insights:**
- Edge detection in PIO reduces CPU load
- Oversampling handles clock drift between ESC and MCU
- Separate PIO programs for TX-only vs bidirectional modes

---

## Proposed Enhancements

### Priority 1: Critical Features (Breaking Changes)

#### 1.1 Bidirectional DShot Support

**Goal:** Enable telemetry reading from ESCs on the same pin used for commands.

**Technical Approach:**
```rust
pub trait DshotPioTrait<const N: usize> {
    // Existing TX methods
    fn command(&mut self, command: [u16; N]);
    fn throttle_clamp(&mut self, throttle: [u16; N]);

    // NEW: Telemetry methods
    fn read_telemetry(&mut self) -> Result<[Option<Telemetry>; N], TelemetryError>;
    fn enable_telemetry(&mut self, enable: bool);
}

pub struct Telemetry {
    pub erpm: u32,           // Electrical RPM
    pub voltage: Option<u16>, // EDT: millivolts
    pub current: Option<u16>, // EDT: milliamps
    pub temperature: Option<i16>, // EDT: degrees Celsius
}
```

**Implementation Steps:**
1. Design new PIO program with pin direction switching
2. Add RX FIFO handling
3. Implement telemetry packet parser
4. Add CRC validation
5. Handle timing constraints (ESC response window)

**PIO Program Outline:**
```pio
.program dshot_bidir

entry:
    ; TX phase (existing code)
    pull
    out null 16
    set pindirs, 1      ; Set pin to output
    ; ... send 16 bits ...

    ; RX phase (NEW)
    set pindirs, 0      ; Switch pin to input
    wait 0 pin 0        ; Wait for start bit
    ; ... read telemetry bits ...
    push                ; Send to RX FIFO
    jmp entry
```

---

#### 1.2 Async/Await API for Embassy

**Goal:** Provide non-blocking, async methods that integrate with Embassy's executor.

**Current Issue:**
```rust
// Blocking - bad for async tasks
self.pio_instance.sm0.tx().push(value);
```

**Proposed API:**
```rust
impl<'d, PIO: Instance> DshotPio<'d, 1, PIO> {
    // Async method
    pub async fn throttle_async(&mut self, throttle: [u16; 1]) {
        let packet = dshot::throttle_clamp(throttle[0], self.telemetry_enabled);
        self.pio_instance.sm0.tx().write(packet as u32).await;
    }

    // Async telemetry read
    pub async fn read_telemetry_async(&mut self) -> Result<[Telemetry; 1], TelemetryError> {
        let data = self.pio_instance.sm0.rx().read().await;
        Ok([parse_telemetry(data)?])
    }
}
```

**Benefits:**
- Non-blocking operation
- Better integration with Embassy tasks
- Allows concurrent motor control + sensor reading

---

#### 1.3 DMA Support

**Goal:** Offload packet transfers to DMA for zero CPU overhead.

**Implementation:**
```rust
use embassy_rp::dma::{AnyChannel, Channel};

pub struct DshotPioDma<'d, const N: usize, PIO: Instance, DMA: Channel> {
    pio_instance: Pio<'d, PIO>,
    tx_dma: DMA,
}

impl<'d, PIO: Instance, DMA: Channel> DshotPioDma<'d, 1, PIO, DMA> {
    pub async fn throttle_dma(&mut self, throttle: [u16; 1]) {
        let packet = dshot::throttle_clamp(throttle[0], false);
        let buffer = [packet as u32];
        self.pio_instance.sm0.tx()
            .write_dma(&buffer, &mut self.tx_dma)
            .await;
    }
}
```

**Benefits:**
- Zero CPU overhead during transfer
- Frees CPU for other tasks
- Essential for high-frequency updates (>1kHz)

---

### Priority 2: Enhanced Features (Non-Breaking)

#### 2.1 Extended DShot Telemetry (EDT)

**Goal:** Support advanced telemetry beyond basic eRPM.

**Telemetry Types:**
```rust
#[derive(Debug, Clone, Copy)]
pub enum TelemetryType {
    Temperature,    // 0-255°C
    Voltage,        // 0-65.535V (0.01V resolution)
    Current,        // 0-655.35A (0.01A resolution)
    Debug1,         // ESC-specific
    Debug2,
    Debug3,
    StateEvent,     // Status flags
}

pub struct ExtendedTelemetry {
    pub erpm: u32,
    pub data: TelemetryType,
}
```

**API:**
```rust
pub trait DshotPioTrait<const N: usize> {
    fn request_telemetry(&mut self, typ: TelemetryType);
}
```

---

#### 2.2 Jitter-Resistant Edge Detection

**Goal:** Implement oversampling and hardware edge detection for reliable RX.

**Technique:**
- Oversample input pin at 3-4x DShot bit rate
- Use PIO's `wait` instruction with timeout
- Majority voting for bit values
- Compensate for clock drift

**PIO Implementation:**
```pio
.program dshot_rx_jitter_resistant

    ; Oversample at 4x bit rate
    set x, 3
sample_loop:
    in pins, 1
    jmp x-- sample_loop

    ; Majority vote (2+ high = bit is 1)
    push
```

---

#### 2.3 Dynamic State Machine Allocation

**Goal:** Only allocate the state machines actually needed.

**Current Problem:**
```rust
// Always creates 4 SMs, even for 1 motor
impl DshotPio<'a, 1, PIO> {
    pub fn new(...) {
        // Only uses sm0, but PIO instance holds all 4
    }
}
```

**Proposed Solution:**
```rust
// Only store the SMs we use
pub struct DshotPio<'a, const N: usize, PIO: Instance> {
    sm0: Option<StateMachine<'a, PIO, 0>>,
    sm1: Option<StateMachine<'a, PIO, 1>>,
    sm2: Option<StateMachine<'a, PIO, 2>>,
    sm3: Option<StateMachine<'a, PIO, 3>>,
}

impl<'a, PIO: Instance> DshotPio<'a, 1, PIO> {
    pub fn new(...) -> Self {
        Self {
            sm0: Some(sm0),
            sm1: None,
            sm2: None,
            sm3: None,
        }
    }
}
```

---

#### 2.4 Error Handling

**Goal:** Provide meaningful errors instead of panics.

**Error Types:**
```rust
#[derive(Debug)]
pub enum DshotError {
    FifoFull,
    TelemetryTimeout,
    InvalidChecksum,
    InvalidTelemetryData,
}

pub type Result<T> = core::result::Result<T, DshotError>;
```

**API Changes:**
```rust
pub trait DshotPioTrait<const N: usize> {
    fn command(&mut self, command: [u16; N]) -> Result<()>;
    fn read_telemetry(&mut self) -> Result<[Option<Telemetry>; N]>;
}
```

---

### Priority 3: Developer Experience

#### 3.1 Examples

**Proposed Examples:**
1. `examples/basic_single_motor.rs` - Simple throttle control (1 motor)
2. `examples/quadcopter.rs` - 4 motors with coordinated control
3. `examples/bidirectional_telemetry.rs` - Read eRPM from ESC
4. `examples/extended_telemetry.rs` - EDT with temperature/voltage
5. `examples/async_embassy.rs` - Async motor control with other tasks
6. `examples/dma_high_frequency.rs` - DMA transfers for >1kHz update rate

---

#### 3.2 Comprehensive Documentation

**Additions:**
1. Protocol explanation (DShot timing, packet format)
2. Telemetry frame structure
3. EDT command sequence
4. Clock divider calculation examples
5. Hardware setup guide
6. Troubleshooting section

---

#### 3.3 Testing

**Test Coverage:**
1. Unit tests for packet encoding
2. Unit tests for telemetry parsing
3. Integration tests with PIO simulator (if available)
4. Hardware-in-the-loop tests with real ESCs

---

## Implementation Roadmap

### Phase 1: Foundation (Weeks 1-2)
- [ ] Design bidirectional PIO program
- [ ] Implement telemetry packet parser
- [ ] Add basic error types
- [ ] Update trait with telemetry methods

### Phase 2: Core Features (Weeks 3-4)
- [ ] Implement async/await API for Embassy
- [ ] Add RX FIFO handling
- [ ] Implement CRC validation
- [ ] Create basic example

### Phase 3: Advanced Features (Weeks 5-6)
- [ ] Add DMA support
- [ ] Implement EDT
- [ ] Add jitter-resistant edge detection
- [ ] Create advanced examples

### Phase 4: Polish (Week 7)
- [ ] Comprehensive documentation
- [ ] Testing suite
- [ ] Performance benchmarks
- [ ] Update README

---

## Technical Considerations

### 1. PIO Instruction Budget

**Current:** 14 instructions (TX only)
**Target:** <28 instructions (bidirectional, per Bastian2001 reference)

**Constraint:** PIO has 32 instruction slots per program.

### 2. Timing Constraints

**DShot Frame Timing (600kbit/s example):**
- Bit time: 1.67µs
- Bit 1: 1.25µs high, 0.42µs low
- Bit 0: 0.625µs high, 1.04µs low
- Frame duration: ~27µs (16 bits)
- Telemetry response: ~30µs after TX

**Challenge:** Must switch pin direction within ~3µs after TX.

### 3. Embassy Version Compatibility

**Current:** `embassy-rp = "0.3"`
**Consideration:** Async TX/RX methods added in recent Embassy versions.

### 4. License Compatibility

**Current:** MIT (assumed, needs verification)
**Bastian2001 lib:** GPL 3.0

**Action:** Study GPL library for *concepts only*, implement from scratch to maintain MIT license.

---

## Breaking Changes Assessment

### API Breaking Changes
1. `DshotPioTrait` gains new methods → **Not breaking** (default impls possible)
2. New error types → **Breaking** if changing existing methods to return `Result`
3. Telemetry bit flag → **Breaking** if changing dshot-encoder calls

### Migration Path
1. Introduce new `DshotPioTraitV2` alongside existing trait
2. Deprecate old trait in v0.5.0
3. Remove old trait in v1.0.0

---

## Success Metrics

1. **Functionality:**
   - [ ] Successfully read eRPM from real ESC
   - [ ] EDT working with BLHeli32 ESC
   - [ ] Async API works in Embassy executor
   - [ ] DMA transfers functional

2. **Performance:**
   - [ ] <1% CPU usage during DMA transfers
   - [ ] Support >1kHz update rate
   - [ ] Telemetry read latency <50µs

3. **Code Quality:**
   - [ ] 100% documented public API
   - [ ] >80% test coverage
   - [ ] Zero `unsafe` outside of HAL interaction
   - [ ] Clippy clean

4. **Community:**
   - [ ] 3+ working examples
   - [ ] Migration guide published
   - [ ] Issue template created

---

## Questions for Consideration

1. **Should we maintain backward compatibility?**
   - Option A: Keep existing API, add new `bidir` module
   - Option B: Breaking change, bump to v1.0.0

2. **DMA-only or support both?**
   - Option A: Keep blocking `.push()` for simple use cases
   - Option B: DMA-only for consistency

3. **Single trait or separate TX/RX traits?**
   ```rust
   // Option A: Single trait
   trait DshotPioTrait { fn tx(...); fn rx(...); }

   // Option B: Separate traits
   trait DshotTx { fn send(...); }
   trait DshotRx { fn receive(...); }
   ```

4. **How to handle telemetry timeouts?**
   - Option A: Return `Option<Telemetry>` (None on timeout)
   - Option B: Return `Result<Telemetry, Timeout>`

---

## References

1. **DShot Protocol Specification:**
   - https://github.com/betaflight/betaflight/wiki/DShot-ESC-Protocol

2. **BLHeli32 EDT Specification:**
   - https://github.com/bitdump/BLHeli/tree/master/BLHeli_32%20ARM

3. **RP2040 PIO Documentation:**
   - https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf (Chapter 3)

4. **Embassy Async PIO:**
   - https://docs.embassy.dev/embassy-rp/

---

## Conclusion

This enhancement plan transforms `dshot-pio` from a capable unidirectional library into a comprehensive, async-first, bidirectional DShot solution while maintaining its Rust-native philosophy and type safety. The phased approach allows incremental development and testing, minimizing risk while maximizing value delivery.

**Estimated Total Effort:** 7 weeks (part-time) or 3-4 weeks (full-time)

**Risk Level:** Medium (requires careful PIO timing and async integration)

**Value Proposition:**
- First Rust library with full bidirectional DShot + EDT
- True async/await Embassy integration
- Best-in-class performance via DMA
- Production-ready for drone/robotics applications
