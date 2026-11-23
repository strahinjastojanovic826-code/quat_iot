# quat_iot

A `#![no_std]` Rust library for IoT and embedded devices that packs 2-bit quaternary values (`00`, `01`, `10`, `11`, corresponding to `0..=3`) into compact byte buffers, saving up to **75% memory** compared to storing one quaternary value per `u8` byte.

## Features

- **`no_std` & Zero Heap Allocation:** Designed with fixed-size const-generic buffers for resource-constrained microcontrollers.
- **75% Memory Savings:** Packs 4 quats (2 bits each) into every single `u8` byte.
- **2-Bit Quaternary Type (`Quat`):** Conversions from `u8` and bit pairs, plus 2-bit bitwise operations (`bit_and`, `bit_or`, `bit_xor`, `bit_not`).
- **Compact Buffer (`QuatBuffer<const MAX_BYTES: usize>`):** Supports `push`, `pop`, `get`, `set`, `push_slice`, `map_in_place`, `as_bytes`, `from_raw_bytes`, and an `ExactSizeIterator` implementation.
- **Ring Buffer (`QuatRingBuffer<const MAX_BYTES: usize>`):** Fixed-capacity FIFO queue with `push`, `push_overwrite`, `pop`, and `peek` for streaming telemetry or sensor samples.
- **Run-Length Encoding (`rle`):** Compresses repeated quaternary sequences into `[quat_u8, count]` pairs via `rle_encode` and `rle_decode`.
- **Framed IoT Packets (`QuatFrame<const MAX_BYTES: usize>`):** Encodes and decodes `[quat_count, packed_payload..., crc8]` frames with `CRC-8` (`0x07`) integrity checking.

## Licensing

This project is dual-licensed under:

1. **GNU Affero General Public License v3.0 (AGPL-3.0)** for open-source use.
2. **Commercial License** for proprietary/commercial applications.

### Commercial License
If you wish to use `quat_iot` in closed-source software, proprietary products, or environments where AGPL-3.0 compliance is not feasible (e.g., embedding DLLs/shared libraries without disclosing host source code), a commercial license is required.

For commercial licensing inquiries, please contact: `strahinjastojanovic826@gmail.com`

# Benchmark Results (`quat_iot`)

Captured in debug mode (`unoptimized + debuginfo`), with all 12 tests passing in `0.11s`[cite: 3].

| Benchmark Name | Iterations | Total Time (ns) | Avg (`ns/op`) | Throughput (`~ops/s`) | Workload Details |
| :--- | ---: | ---: | ---: | ---: | :--- |
| `01_quat_basic_conversion` | `10,000` | `406,700` | `40` | `24,588,148` | 4 conversions/iter[cite: 3] |
| `02_quat_bits_and_errors` | `10,000` | `877,600` | `87` | `11,394,712` | bits + invalid checks[cite: 3] |
| `03_quat_bitwise_logic` | `25,000` | `2,031,700` | `81` | `12,304,966` | AND/OR/XOR/NOT[cite: 3] |
| `04_buffer_push_get_pack` | `25,000` | `4,739,800` | `189` | `5,274,484` | 4 quats in 1 byte[cite: 3] |
| `05_buffer_set_pop_map` | `25,000` | `11,583,100` | `463` | `2,158,316` | mutate + pop + map[cite: 3] |
| `07_ring_fifo_overwrite` | `50,000` | `15,227,000` | `304` | `3,283,640` | push/pop/overwrite[cite: 3] |
| `06_buffer_iter_raw_bytes` | `50,000` | `18,575,800` | `371` | `2,691,674` | iter + raw roundtrip[cite: 3] |
| `10_medium_telemetry_stress` | `320,000` | `22,528,400` | `70` | `14,204,293` | 5k frames x 64 quats[cite: 3] |
| `08_rle_encode_decode` | `50,000` | `46,702,300` | `934` | `1,070,611` | 10-quat RLE roundtrip[cite: 3] |
| `11_high_ring_rle_stream` | `250,000` | `46,385,600` | `185` | `5,389,603` | 250k quats ring + RLE[cite: 3] |
| `09_frame_crc_roundtrip` | `75,000` | `58,100,000` | `774` | `1,290,877` | encode/decode + CRC err[cite: 3] |
| `12_max_stress_iot_pipeline` | `1,000,000` | `112,868,300` | `112` | `8,859,883` | 1M quats (250kB raw -> 62.5kB packed)[cite: 3] |

## Project Structure

```text
quat_iot/
├── Cargo.toml
├── README.md
└── src/
    ├── lib.rs
    ├── error.rs
    ├── quat.rs
    ├── buffer.rs
    ├── ring.rs
    ├── rle.rs
    └── frame.rs
```

## Memory Savings Example

- **1 byte per quat (`[u8; 4]`):** 4 quats = `4 bytes` (32 bits total, 24 bits wasted).
- **Packed quaternary (`QuatBuffer<1>`):** 4 quats = `1 byte` (8 bits total) — **75% memory reduction**.

## Usage Example

```rust
use quat_iot::{rle_decode, rle_encode, Quat, QuatBuffer, QuatFrame, QuatRingBuffer};

fn main() {
    // 1. Pack quats into a compact buffer (4 bytes -> up to 16 quats)
    let mut buf = QuatBuffer::<4>::new();
    buf.push_slice(&[Quat::Zero, Quat::One, Quat::Two, Quat::Three, Quat::Three])
        .unwrap();

    // Access raw packed bytes for UART, SPI, I2C, radio, or flash
    let packed_bytes: &[u8] = buf.as_bytes();
    assert_eq!(packed_bytes.len(), 2);

    // 2. Ring buffer for streaming values
    let mut ring = QuatRingBuffer::<2>::new();
    ring.push(Quat::One).unwrap();
    ring.push(Quat::Two).unwrap();
    assert_eq!(ring.pop().unwrap(), Quat::One);

    // 3. Run-length encoding for repeated states
    let mut rle_out = [0u8; 16];
    let rle_len = rle_encode(&buf, &mut rle_out).unwrap();
    let decoded_rle = rle_decode::<4>(&rle_out[..rle_len]).unwrap();
    assert_eq!(decoded_rle, buf);

    // 4. Frame with quat count header and CRC-8
    let mut wire = [0u8; 32];
    let frame_len = QuatFrame::<4>::encode(&buf, &mut wire).unwrap();
    let received = QuatFrame::<4>::decode(&wire[..frame_len]).unwrap();
    assert_eq!(received, buf);
}
```

## Running Tests & Benchmarks

Run all unit tests and print inline benchmark metrics (`ns/op` and `ops/s`):

```bash
cargo test 
```

```bash
cargo test -- --nocapture
```

For representative timing measurements, run in release mode:

```bash
cargo test --release -- --nocapture
```

## Note on Frame Capacity

`QuatFrame` encodes the number of quats in a single `u8` header byte (`0..=255` quats per frame), so keep framed payloads at or below `255` quats (`<= 63` full packed bytes / `252` quats on a `4`-quat boundary).