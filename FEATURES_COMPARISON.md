# Feature Comparison: DShot Libraries

Quick reference comparing features across three DShot implementations.

---

## 🦀 Current: dshot-pio (This Fork - Rust)

### ✅ Current Features
- PIO-based hardware timing (14 instructions)
- Clean trait-based API (`DshotPioTrait<const N>`)
- Support for 1-4 motors per PIO block (up to 8 total on RP2040)
- Dual HAL support: `embassy-rp` v0.3 + `rp2040-hal` v0.11
- Type-safe const generics for motor count
- Configurable DShot speeds (150/300/600/1200)
- Zero-cost abstractions
- `no_std` compatible

### ❌ Missing Features
- **Bidirectional DShot** (telemetry bit always false)
- **Async/await** (Embassy uses blocking `.push()`)
- **Telemetry reading** (PIO is TX-only)
- **DMA support** (all transfers via CPU)
- **Error handling** (uses `.expect()`)
- **Examples** (no demo code)
- **Tests** (no test suite)
- **Jitter resistance** (no edge detection)
- **Dynamic allocation** (always creates 4 SMs)

### 🎯 Best For
- Unidirectional control of 1-4 motors
- Applications not needing telemetry
- Type-safe Rust environments
- Users wanting both HAL options

---

## 🔄 Reference: pico-dshot-bidir (josephduchesne - C++)

### ✅ Features
- **Bidirectional DShot** on single wire
- **Extended telemetry support**
- PIO-based implementation
- Multiple DShot speeds (300/600/1200)
- Low CPU overhead via PIO state machines
- Compatible with BLHeli32 v32.10+
- Arduino/PlatformIO ecosystem
- Working examples included

### ❌ Limitations
- C++/Arduino only (not Rust)
- Blocking API (no async)
- Limited to Arduino framework

### 🎯 Best For
- Arduino users needing bidirectional DShot
- Projects already using PlatformIO
- Quick prototyping with examples

### 📦 Key Learnings for Our Fork
1. **PIO bidirectional protocol** - how to switch TX→RX
2. **Telemetry packet format** - frame structure & parsing
3. **Timing constraints** - ESC response windows
4. **EDT integration** - extended telemetry commands

---

## ⚡ Reference: Pico_Bidir_DShot (Bastian2001 - C++)

### ✅ Features
- **Jitter-resistant edge detection** (hardware oversampling)
- **Highly efficient PIO programs:**
  - TX-only: 4 instructions per 4 ESCs
  - Bidirectional: 28 instructions per ESC
- **Massive scalability:**
  - TX-only: 30-48 ESCs per RP2040
  - Bidirectional: 8-12 ESCs per RP2040
- **Extended DShot Telemetry (EDT):**
  - eRPM (electrical RPM)
  - Temperature (0-255°C)
  - Voltage (0.01V resolution)
  - Current (0.01A resolution)
  - Debug channels
  - State/event flags
- **Hardware edge detection** in PIO
- **Clock mismatch tolerance** via oversampling
- **Fully asynchronous** at PIO level
- **eRPM → RPM conversion** helper
- GPL 3.0 licensed
- Latest release: v1.0.2 (Oct 2024)

### ❌ Limitations
- C++/Arduino only (not Rust)
- GPL 3.0 (copyleft license)
- Arduino framework dependency

### 🎯 Best For
- Arduino users needing maximum ESC count
- Applications requiring robust telemetry
- Noisy electrical environments (jitter resistance)
- Reading comprehensive ESC diagnostics

### 📦 Key Learnings for Our Fork
1. **Hardware edge detection** - oversampling technique
2. **PIO efficiency** - minimal instruction count
3. **Jitter resistance** - clock drift compensation
4. **EDT implementation** - multi-type telemetry
5. **Scalability** - support many ESCs efficiently

---

## 🚀 Proposed: dshot-pio v1.0 (Enhanced)

### 🎯 Target Feature Set

#### Core (Priority 1)
- ✅ Keep all current features
- ✨ **Bidirectional DShot** with telemetry
- ✨ **True async/await** for Embassy
- ✨ **DMA support** for zero-CPU transfers
- ✨ **Error handling** with `Result` types
- ✨ **Basic telemetry** (eRPM reading)

#### Enhanced (Priority 2)
- ✨ **Extended DShot Telemetry (EDT)**
  - Temperature, voltage, current
  - Status flags, debug channels
- ✨ **Jitter-resistant RX** via oversampling
- ✨ **Dynamic SM allocation** (don't waste SMs)
- ✨ **Improved PIO programs** (<28 instructions for bidir)

#### Developer Experience (Priority 3)
- ✨ **Examples:**
  - Basic single motor
  - Quadcopter (4 motors)
  - Bidirectional telemetry
  - Extended telemetry (EDT)
  - Async Embassy integration
  - DMA high-frequency
- ✨ **Comprehensive docs**
- ✨ **Test suite**
- ✨ **Performance benchmarks**

#### API Design
```rust
// Async API
pub trait DshotPioAsync<const N: usize> {
    async fn throttle_async(&mut self, throttle: [u16; N]) -> Result<()>;
    async fn read_telemetry_async(&mut self) -> Result<[Telemetry; N]>;
}

// Telemetry types
pub struct Telemetry {
    pub erpm: u32,
    pub voltage: Option<u16>,    // EDT
    pub current: Option<u16>,    // EDT
    pub temperature: Option<i16>, // EDT
}

// Error handling
pub enum DshotError {
    FifoFull,
    TelemetryTimeout,
    InvalidChecksum,
}
```

---

## 📊 Feature Matrix

| Feature | dshot-pio (current) | josephduchesne | Bastian2001 | dshot-pio v1.0 |
|---------|:-------------------:|:--------------:|:-----------:|:--------------:|
| **Language** | Rust 🦀 | C++ | C++ | Rust 🦀 |
| **Framework** | Embassy/HAL | Arduino | Arduino | Embassy/HAL |
| **License** | MIT (assumed) | MIT | GPL 3.0 | MIT |
| **TX Support** | ✅ | ✅ | ✅ | ✅ |
| **RX Support** | ❌ | ✅ | ✅ | ✅ (planned) |
| **Async/Await** | ❌ | ❌ | ❌ | ✅ (planned) |
| **DMA** | ❌ | ❌ | ❌ | ✅ (planned) |
| **Basic Telemetry** | ❌ | ✅ | ✅ | ✅ (planned) |
| **EDT** | ❌ | ✅ | ✅ | ✅ (planned) |
| **Jitter Resistance** | ❌ | ⚠️ | ✅ | ✅ (planned) |
| **Edge Detection** | ❌ | ⚠️ | ✅ (HW) | ✅ (planned) |
| **Error Handling** | ❌ | ⚠️ | ⚠️ | ✅ (planned) |
| **Examples** | ❌ | ✅ | ⚠️ | ✅ (planned) |
| **Tests** | ❌ | ❌ | ❌ | ✅ (planned) |
| **Max ESCs (TX)** | 8 | ? | 30-48 | 8-16 (planned) |
| **Max ESCs (Bidir)** | - | ? | 8-12 | 4-8 (planned) |
| **PIO Instructions** | 14 | ? | 4-28 | <28 (target) |
| **Type Safety** | ✅✅ | ❌ | ❌ | ✅✅ |
| **Zero-Cost Abstractions** | ✅ | ❌ | ❌ | ✅ |

Legend:
- ✅ = Fully supported
- ⚠️ = Partial/basic support
- ❌ = Not supported
- ? = Unknown/not documented

---

## 🎓 Key Insights from References

### From josephduchesne/pico-dshot-bidir:
1. **Single-wire bidirectional is feasible** on RP2040 PIO
2. **ESC response timing** requires ~30µs window after TX
3. **BLHeli32 compatibility** needs firmware v32.10+
4. **EDT commands** follow specific sequence pattern

### From Bastian2001/pico-bidir-dshot:
1. **Oversampling (3-4x bit rate)** critical for jitter resistance
2. **Hardware edge detection** dramatically improves reliability
3. **Efficient PIO design** enables scaling to 30+ ESCs
4. **Clock drift tolerance** essential for real-world use
5. **Separate PIO programs** for TX-only vs bidirectional optimizes performance

### From Current Implementation:
1. **Trait-based API** provides excellent abstraction
2. **Const generics** enable compile-time optimization
3. **Dual HAL support** valuable for user choice
4. **Type safety** catches errors at compile time

---

## 🛠️ Implementation Strategy

### Phase 1: Foundation (Breaking Changes OK)
Focus on bidirectional support and async API. Accept breaking changes for v1.0.

### Phase 2: Enhanced Features
Add EDT, jitter resistance, and DMA without breaking Phase 1 API.

### Phase 3: Developer Experience
Examples, docs, tests, benchmarks.

### Migration Path
- v0.4.x: Current stable (TX-only)
- v0.5.0: Add new `bidir` module (non-breaking)
- v1.0.0: Full refactor with breaking changes, new async API

---

## 📈 Success Metrics

**Functionality:**
- [ ] Read eRPM from ESC at >100Hz
- [ ] EDT working with BLHeli32
- [ ] Async tasks running concurrently with motor control
- [ ] DMA transfers at >1kHz rate

**Performance:**
- [ ] <1% CPU usage during DMA operation
- [ ] <50µs telemetry read latency
- [ ] Support 4+ bidirectional ESCs per PIO

**Quality:**
- [ ] 100% public API documented
- [ ] 3+ working examples
- [ ] Zero `unsafe` in public API
- [ ] Clippy warnings: 0

---

## 🤔 Open Questions

1. **Backward compatibility:** New module or breaking change?
2. **DMA requirement:** Optional or mandatory for bidir?
3. **Trait design:** Single trait or split TX/RX?
4. **Error strategy:** `Option` vs `Result` for timeouts?
5. **License verification:** Confirm current MIT license?

---

## 📚 References

- **DShot Protocol:** https://github.com/betaflight/betaflight/wiki/DShot-ESC-Protocol
- **BLHeli32 EDT:** https://github.com/bitdump/BLHeli/tree/master/BLHeli_32%20ARM
- **RP2040 PIO:** https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
- **Embassy:** https://docs.embassy.dev/embassy-rp/

---

**Last Updated:** 2025-12-25
**Version:** 1.0
**Status:** Planning Phase
