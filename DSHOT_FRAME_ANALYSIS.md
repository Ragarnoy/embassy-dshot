# dshot-frame vs dshot-encoder: Compatibility Analysis

## Executive Summary

**Recommendation: YES, migrate to `dshot-frame`** with some adaptations.

The `dshot-frame` crate (v0.2.1, MIT licensed) is significantly more feature-complete than `dshot-encoder` and includes built-in support for bidirectional DShot and telemetry parsing—exactly what we need for our enhancement plan.

---

## Detailed Comparison

### Current: dshot-encoder

**What we currently use:**
```rust
use dshot_encoder as dshot;

// Current API usage (from dshot_embassy_rp.rs)
dshot::THROTTLE_MAX                           // Constant: u16
dshot::reverse(reverse: bool) -> u16          // Returns encoded frame
dshot::throttle_clamp(throttle: u16, telemetry: bool) -> u16
dshot::throttle_minimum(telemetry: bool) -> u16
```

**Characteristics:**
- ✅ Simple function-based API
- ✅ Returns raw u16 values (easy to push to PIO FIFO)
- ✅ Handles CRC calculation
- ✅ Accepts telemetry flag
- ❌ **No telemetry parsing** (RX side)
- ❌ **No bidirectional protocol types**
- ❌ No Command enum (uses raw values)
- ❌ No type safety for protocol variants
- ❌ Limited documentation
- ⚠️ Maintained by same author (peterkrull)

---

### Proposed: dshot-frame

**Available API:**
```rust
use dshot_frame::{Frame, NormalDshot, BidirectionalDshot, ErpmTelemetry, Command};

// TX: Create frame
let frame = Frame::<NormalDshot>::new(throttle: u16, telemetry_request: bool)?;
let frame = Frame::<BidirectionalDshot>::new(throttle: u16, telemetry_request: bool)?;

// Get PWM duty cycles
let duty_cycles = frame.duty_cycles(); // Returns waveform data

// RX: Parse telemetry
let telemetry = ErpmTelemetry::from_frame(data);

// Commands
let cmd = Command::MotorStop; // Typed commands instead of raw values
```

**Characteristics:**
- ✅ **Bidirectional protocol support** (`BidirectionalDshot` type)
- ✅ **Telemetry parsing** (`ErpmTelemetry` struct)
- ✅ **Type-safe protocol variants** (NormalDshot vs BidirectionalDshot)
- ✅ **Command enum** (type-safe commands)
- ✅ **MIT licensed** (compatible with our project)
- ✅ Actively maintained (v0.2.1, updated Oct 2024)
- ✅ Better documentation on docs.rs
- ⚠️ Returns `duty_cycles()` for PWM (needs adaptation for PIO)
- ⚠️ Type parameter `<T>` adds complexity
- ⚠️ Different API requires refactoring

---

## Compatibility Assessment for PIO Usage

### Challenge: duty_cycles() vs Raw Frame Data

**Issue:**
- `dshot-frame` provides `.duty_cycles()` method for PWM timers
- We need the **raw 16-bit frame data** to push to PIO FIFO

**Investigation Needed:**
We need to check if `dshot-frame` exposes:
1. The raw frame bytes/u16 value
2. A method to get the encoded packet without duty cycles
3. Or if we can extract it from the `Frame` struct

**Possible Solutions:**

**Option A: Direct frame access**
```rust
// If Frame exposes the raw data:
let frame = Frame::<BidirectionalDshot>::new(throttle, true)?;
let raw_data: u16 = frame.raw_bits(); // Hypothetical method
self.pio_instance.sm0.tx().push(raw_data as u32);
```

**Option B: Convert duty cycles back to bits**
```rust
// If only duty_cycles() available:
let duty_cycles = frame.duty_cycles();
let raw_data = reconstruct_from_duty_cycles(duty_cycles); // Convert back
```

**Option C: Fork or contribute upstream**
```rust
// Add to dshot-frame crate:
impl<T: DshotProtocol> Frame<T> {
    pub fn raw_frame(&self) -> u16 {
        // Return the 16-bit encoded frame
    }
}
```

**Option D: Implement encoding ourselves, use only RX side**
```rust
// Use dshot-encoder for TX, dshot-frame for RX only
use dshot_encoder as tx_encoder;
use dshot_frame::ErpmTelemetry;

// TX: Keep current approach
let frame = tx_encoder::throttle_clamp(throttle, true);
self.pio_instance.sm0.tx().push(frame as u32);

// RX: Use dshot-frame for parsing
let telemetry_data: u16 = self.pio_instance.sm0.rx().pull();
let telemetry = ErpmTelemetry::from_bits(telemetry_data)?;
```

---

## Migration Impact Analysis

### Code Changes Required

#### 1. Cargo.toml
```diff
[dependencies]
-dshot-encoder = { git = "https://github.com/peterkrull/dshot-encoder" }
+dshot-frame = "0.2.1"
```

#### 2. API Changes in Implementation

**Current (dshot-encoder):**
```rust
fn throttle_clamp(&mut self, throttle: [u16; 1]) {
    self.pio_instance.sm0.tx().push(
        dshot::throttle_clamp(throttle[0], false) as u32
    );
}
```

**Option 1: Full dshot-frame (if raw frame accessible):**
```rust
use dshot_frame::{Frame, NormalDshot};

fn throttle_clamp(&mut self, throttle: [u16; 1]) -> Result<(), FrameError> {
    let frame = Frame::<NormalDshot>::new(throttle[0], false)?;
    self.pio_instance.sm0.tx().push(frame.raw_bits() as u32); // Hypothetical
    Ok(())
}
```

**Option 2: Hybrid approach (dshot-encoder TX, dshot-frame RX):**
```rust
use dshot_encoder as dshot; // Keep for TX
use dshot_frame::ErpmTelemetry; // Add for RX

fn throttle_clamp(&mut self, throttle: [u16; 1]) {
    self.pio_instance.sm0.tx().push(
        dshot::throttle_clamp(throttle[0], false) as u32
    );
}

fn read_telemetry(&mut self) -> Result<Telemetry, TelemetryError> {
    let raw = self.pio_instance.sm0.rx().pull() as u16;
    let erpm = ErpmTelemetry::from_bits(raw)?;
    Ok(Telemetry { erpm: erpm.value() })
}
```

#### 3. Trait Changes

**New telemetry methods:**
```rust
pub trait DshotPioTrait<const N: usize> {
    // Existing
    fn command(&mut self, command: [u16; N]);
    fn reverse(&mut self, reverse: [bool; N]);
    fn throttle_clamp(&mut self, throttle: [u16; N]);
    fn throttle_minimum(&mut self);

    // NEW: Telemetry support
    fn read_telemetry(&mut self) -> Result<[Option<ErpmTelemetry>; N], TelemetryError>;
}
```

---

## Feature Advantages of dshot-frame

### 1. Bidirectional Protocol Types

**Type safety for protocol variants:**
```rust
// Compile-time differentiation
let normal = Frame::<NormalDshot>::new(1000, false);     // Standard DShot
let bidir = Frame::<BidirectionalDshot>::new(1000, true); // Inverted for bidir
```

This prevents mixing protocol types accidentally.

### 2. Command Enum

**Before (dshot-encoder):**
```rust
// Magic numbers - easy to make mistakes
self.command([0]);    // Motor stop? Or something else?
self.command([1]);    // What does this do?
```

**After (dshot-frame):**
```rust
use dshot_frame::Command;

self.command(Command::MotorStop);
self.command(Command::Beep1);
self.command(Command::RotateReverse);
self.command(Command::ThreeDMode);
```

Much clearer and self-documenting!

### 3. ErpmTelemetry Parsing

**Built-in telemetry decoder:**
```rust
// Raw data from PIO RX FIFO
let telemetry_bits: u16 = self.pio_instance.sm0.rx().pull() as u16;

// Parse with dshot-frame
let telemetry = ErpmTelemetry::from_bits(telemetry_bits)?;
let erpm = telemetry.erpm(); // Electrical RPM value

// Convert to mechanical RPM
let rpm = erpm_to_rpm(erpm, motor_poles);
```

No need to implement GCR decoding, CRC validation, etc.—it's all handled.

### 4. Better Error Handling

```rust
// dshot-frame likely returns Results
let frame = Frame::<NormalDshot>::new(throttle, telemetry)?;
// vs
// dshot-encoder might panic on invalid input
```

---

## Risks and Mitigations

### Risk 1: No Direct Raw Frame Access

**Risk:** `dshot-frame` might not expose raw u16 frame data.

**Mitigation:**
1. **Check source code** to verify raw frame access
2. **Open issue** requesting `.raw_bits()` method
3. **Fork and add feature** if needed
4. **Hybrid approach** (encoder for TX, frame for RX)

**Status:** 🔍 **Needs investigation**

### Risk 2: API Breaking Changes

**Risk:** Migrating will break existing API.

**Mitigation:**
1. **Version bump** to v1.0.0 (planned anyway)
2. **Deprecation period** (v0.5.0 with both APIs)
3. **Migration guide** for users

**Status:** ✅ **Acceptable** (planned breaking change)

### Risk 3: Performance Overhead

**Risk:** Type parameters and abstraction may add overhead.

**Mitigation:**
1. **Zero-cost abstractions** in Rust (compile-time)
2. **Benchmark** before and after
3. Frame creation can be **const** evaluated

**Status:** ✅ **Low risk** (Rust optimizes generics)

### Risk 4: Incomplete Documentation

**Risk:** `dshot-frame` might lack examples for our use case.

**Mitigation:**
1. **Read source code** directly
2. **Contact maintainer** (sulami) if needed
3. **Contribute examples** upstream after implementation

**Status:** ⚠️ **Medium risk** (manageable)

---

## Recommendation: Phased Migration

### Phase 1: Investigation (Week 1)
- [ ] Clone `dshot-frame` repository
- [ ] Review source code for raw frame access
- [ ] Test frame encoding matches our current output
- [ ] Verify `ErpmTelemetry` parsing works with test data
- [ ] Check for any hidden features or undocumented APIs

### Phase 2: Hybrid Integration (Week 2)
- [ ] Add `dshot-frame` as dependency alongside `dshot-encoder`
- [ ] Use `dshot-frame::ErpmTelemetry` for RX only
- [ ] Keep `dshot-encoder` for TX temporarily
- [ ] Implement and test bidirectional PIO program
- [ ] Validate telemetry parsing with real ESC

### Phase 3: Full Migration (Week 3)
- [ ] If raw frame access exists: migrate TX to `dshot-frame`
- [ ] If not: contribute upstream or fork with feature
- [ ] Replace all `dshot-encoder` calls
- [ ] Update API to use `Command` enum
- [ ] Add protocol type parameters

### Phase 4: Upstream Contribution (Week 4)
- [ ] If we added features, create PR to `dshot-frame`
- [ ] Share PIO usage example with maintainer
- [ ] Document our integration in README

---

## Immediate Next Steps

### 1. Source Code Investigation

**Task:** Clone and examine `dshot-frame` internals

```bash
git clone https://github.com/sulami/dshot-frame.git
cd dshot-frame
cargo doc --open  # Check full API
rg "impl.*Frame" src/  # Find implementation details
```

**Questions to answer:**
- Does `Frame` store the raw 16-bit value?
- Can we access it directly or add a method?
- How is `duty_cycles()` calculated from the frame?
- Can we reverse-engineer the raw bits from duty cycles?

### 2. Create Test Program

**Task:** Verify encoding compatibility

```rust
// Test both encoders produce same output
use dshot_encoder as old;
use dshot_frame::{Frame, NormalDshot};

fn test_compatibility() {
    let throttle = 1000;
    let telemetry = true;

    let old_frame = old::throttle_clamp(throttle, telemetry);
    let new_frame = Frame::<NormalDshot>::new(throttle, telemetry).unwrap();

    // Compare outputs
    assert_eq!(old_frame, new_frame.raw_bits()); // If method exists
}
```

### 3. Reach Out to Maintainer

**Task:** Contact sulami about PIO use case

```markdown
Subject: Using dshot-frame with RP2040 PIO - raw frame access?

Hi sulami,

I'm working on a Rust library for bidirectional DShot on RP2040 using
PIO state machines. Your dshot-frame crate looks perfect for our needs,
especially the ErpmTelemetry support!

For PIO, I need to push raw 16-bit frame data to hardware FIFOs rather
than PWM duty cycles. Does Frame expose the raw encoded bits, or would
you be open to adding a `Frame::raw_bits()` method?

Use case: https://github.com/Ragarnoy/dshot-pio

Thanks!
```

---

## Decision Matrix

| Criteria | dshot-encoder | dshot-frame | Winner |
|----------|:-------------:|:-----------:|:------:|
| **TX Support** | ✅ Simple | ✅ Type-safe | 🟡 Tie |
| **RX Support** | ❌ None | ✅ ErpmTelemetry | 🏆 dshot-frame |
| **Bidirectional** | ❌ No | ✅ Yes | 🏆 dshot-frame |
| **Type Safety** | ⚠️ Raw u16 | ✅ Typed variants | 🏆 dshot-frame |
| **Command API** | ❌ Magic numbers | ✅ Enum | 🏆 dshot-frame |
| **PIO Integration** | ✅ Direct u16 | ❓ TBD | 🟡 TBD |
| **Documentation** | ⚠️ Minimal | ✅ docs.rs | 🏆 dshot-frame |
| **License** | ❓ Unknown | ✅ MIT | 🏆 dshot-frame |
| **Maintenance** | ⚠️ Same author | ✅ Active | 🟡 Tie |
| **API Stability** | ✅ Stable | ⚠️ Pre-1.0 | 🏆 dshot-encoder |

**Score: dshot-frame wins 7-1 (1 TBD)**

---

## Conclusion

**Adopt `dshot-frame` for bidirectional enhancement**, with the following approach:

1. **Immediate:** Investigate raw frame access in source code
2. **Short-term:** Hybrid approach (encoder TX, frame RX) if needed
3. **Medium-term:** Full migration to `dshot-frame` once validated
4. **Long-term:** Contribute PIO improvements upstream

The `ErpmTelemetry` parsing alone makes `dshot-frame` worth adopting, and the type-safe protocol variants align perfectly with our enhancement goals.

---

## Action Items

- [ ] Clone `dshot-frame` repo and review source
- [ ] Test encoding compatibility
- [ ] Verify `ErpmTelemetry` parsing
- [ ] Contact maintainer about PIO use case
- [ ] Create proof-of-concept integration
- [ ] Update IMPROVEMENT_PLAN.md with findings
- [ ] Make final migration decision

---

**Status:** Analysis complete, awaiting investigation
**Next:** Source code review of dshot-frame internals
**Timeline:** 1-2 days for investigation, decision by end of week

