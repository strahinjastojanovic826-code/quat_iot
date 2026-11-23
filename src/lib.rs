#![no_std]

//! `quat_iot` is a `no_std` Rust library for packing 2-bit quaternary values
//! (`00`, `01`, `10`, `11`) into compact buffers, ring queues, RLE streams, and framed IoT packets.

pub mod buffer;
pub mod error;
pub mod frame;
pub mod quat;
pub mod ring;
pub mod rle;

pub use buffer::{QuatBuffer, QuatBufferIter};
pub use error::QuatError;
pub use frame::{crc8, QuatFrame};
pub use quat::Quat;
pub use ring::QuatRingBuffer;
pub use rle::{rle_decode, rle_encode, QuatRun};

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::println;
    use std::time::Instant;

    fn report_bench(name: &str, iterations: u128, elapsed_ns: u128, details: &str) {
        let ns_per_op = if iterations > 0 {
            elapsed_ns / iterations
        } else {
            0
        };
        let ops_per_sec = if elapsed_ns > 0 {
            (iterations * 1_000_000_000) / elapsed_ns
        } else {
            0
        };
        println!(
            "[BENCH] {:<34} | iters={:<9} | total={:>8} ns | {:>6} ns/op | ~{:>10} ops/s | {}",
            name, iterations, elapsed_ns, ns_per_op, ops_per_sec, details
        );
    }

    #[test]
    fn test_01_quat_basic_conversion() {
        let start = Instant::now();
        let iters = 10_000u128;

        for _ in 0..iters {
            assert_eq!(Quat::from_u8(0).unwrap(), Quat::Zero);
            assert_eq!(Quat::from_u8(1).unwrap(), Quat::One);
            assert_eq!(Quat::from_u8(2).unwrap(), Quat::Two);
            assert_eq!(Quat::from_u8(3).unwrap(), Quat::Three);
            assert_eq!(Quat::Three.as_u8(), 3);
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("01_quat_basic_conversion", iters, elapsed, "4 conversions/iter");
    }

    #[test]
    fn test_02_quat_bits_and_error_cases() {
        let start = Instant::now();
        let iters = 10_000u128;

        for _ in 0..iters {
            let q = Quat::from_bits(1, 0).unwrap();
            assert_eq!(q, Quat::Two);
            assert_eq!(q.as_bits(), (1, 0));
            assert_eq!(Quat::from_u8(4), Err(QuatError::InvalidValue(4)));
            assert_eq!(Quat::from_bits(1, 2), Err(QuatError::InvalidValue(2)));
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("02_quat_bits_and_errors", iters, elapsed, "bits + invalid checks");
    }

    #[test]
    fn test_03_quat_bitwise_logic() {
        let start = Instant::now();
        let iters = 25_000u128;

        for _ in 0..iters {
            assert_eq!(Quat::Three.bit_and(Quat::One), Quat::One);
            assert_eq!(Quat::One.bit_or(Quat::Two), Quat::Three);
            assert_eq!(Quat::Three.bit_xor(Quat::One), Quat::Two);
            assert_eq!(Quat::One.bit_not(), Quat::Two);
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("03_quat_bitwise_logic", iters, elapsed, "AND/OR/XOR/NOT");
    }

    #[test]
    fn test_04_buffer_push_get_75pct_savings() {
        let start = Instant::now();
        let iters = 25_000u128;

        for _ in 0..iters {
            let mut buf = QuatBuffer::<1>::new(); // 1 byte -> up to 4 quats (75% savings vs 4 bytes)
            buf.push(Quat::Zero).unwrap();
            buf.push(Quat::One).unwrap();
            buf.push(Quat::Two).unwrap();
            buf.push(Quat::Three).unwrap();

            assert_eq!(buf.len(), 4);
            assert_eq!(buf.as_bytes().len(), 1);
            assert_eq!(buf.get(0).unwrap(), Quat::Zero);
            assert_eq!(buf.get(3).unwrap(), Quat::Three);
            assert_eq!(buf.push(Quat::Zero), Err(QuatError::BufferFull));
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("04_buffer_push_get_pack", iters, elapsed, "4 quats in 1 byte");
    }

    #[test]
    fn test_05_buffer_set_pop_and_map() {
        let start = Instant::now();
        let iters = 25_000u128;

        for _ in 0..iters {
            let mut buf = QuatBuffer::<2>::new();
            buf.push_slice(&[Quat::Zero, Quat::One, Quat::Two, Quat::Three])
                .unwrap();
            buf.set(1, Quat::Three).unwrap();
            buf.map_in_place(|q| q.bit_xor(Quat::One)).unwrap();
            let popped = buf.pop().unwrap();

            assert_eq!(popped, Quat::Two);
            assert_eq!(buf.len(), 3);
            assert_eq!(buf.get(1).unwrap(), Quat::Two);
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("05_buffer_set_pop_map", iters, elapsed, "mutate + pop + map");
    }

    #[test]
    fn test_06_buffer_iterator_and_raw_bytes() {
        let start = Instant::now();
        let iters = 50_000u128;

        for _ in 0..iters {
            let mut src = QuatBuffer::<4>::new();
            src.push_slice(&[Quat::Three, Quat::Two, Quat::One, Quat::Zero, Quat::Two])
                .unwrap();

            let count = src.iter().count();
            assert_eq!(count, 5);

            let raw = src.as_bytes();
            let restored = QuatBuffer::<4>::from_raw_bytes(raw, 5).unwrap();
            assert_eq!(restored, src);
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("06_buffer_iter_raw_bytes", iters, elapsed, "iter + raw roundtrip");
    }

    #[test]
    fn test_07_ring_buffer_fifo_and_overwrite() {
        let start = Instant::now();
        let iters = 50_000u128;

        for _ in 0..iters {
            let mut ring = QuatRingBuffer::<1>::new(); // capacity = 4 quats
            ring.push(Quat::Zero).unwrap();
            ring.push(Quat::One).unwrap();
            ring.push(Quat::Two).unwrap();
            ring.push(Quat::Three).unwrap();
            assert!(ring.is_full());

            let evicted = ring.push_overwrite(Quat::Two).unwrap();
            assert_eq!(evicted, Some(Quat::Zero));
            assert_eq!(ring.peek().unwrap(), Quat::One);
            assert_eq!(ring.pop().unwrap(), Quat::One);
            assert_eq!(ring.len(), 3);
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("07_ring_fifo_overwrite", iters, elapsed, "push/pop/overwrite");
    }

    #[test]
    fn test_08_rle_encode_decode_roundtrip() {
        let start = Instant::now();
        let iters = 50_000u128;

        for _ in 0..iters {
            let mut buf = QuatBuffer::<8>::new();
            buf.push_slice(&[
                Quat::One,
                Quat::One,
                Quat::One,
                Quat::One,
                Quat::Three,
                Quat::Three,
                Quat::Zero,
                Quat::Two,
                Quat::Two,
                Quat::Two,
            ])
            .unwrap();

            let mut encoded = [0u8; 16];
            let enc_len = rle_encode(&buf, &mut encoded).unwrap();
            let decoded = rle_decode::<8>(&encoded[..enc_len]).unwrap();
            assert_eq!(decoded, buf);
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("08_rle_encode_decode", iters, elapsed, "10-quat RLE roundtrip");
    }

    #[test]
    fn test_09_frame_encode_decode_and_crc() {
        let start = Instant::now();
        let iters = 75_000u128;

        for _ in 0..iters {
            let mut tx = QuatBuffer::<16>::new();
            tx.push_slice(&[Quat::Three, Quat::One, Quat::Zero, Quat::Two, Quat::Three])
                .unwrap();

            let mut wire = [0u8; 32];
            let len = QuatFrame::<16>::encode(&tx, &mut wire).unwrap();
            assert_eq!(QuatFrame::<16>::peek_quat_count(&wire[..len]).unwrap(), 5);

            let rx = QuatFrame::<16>::decode(&wire[..len]).unwrap();
            assert_eq!(rx, tx);

            let mut corrupted = wire;
            corrupted[1] ^= 0b0000_0011;
            assert!(matches!(
                QuatFrame::<16>::decode(&corrupted[..len]),
                Err(QuatError::CrcMismatch { .. })
            ));
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench("09_frame_crc_roundtrip", iters, elapsed, "encode/decode + CRC err");
    }

    #[test]
    fn test_10_medium_telemetry_pipeline_stress() {
        let start = Instant::now();
        let batches = 5_000u128;
        let quats_per_batch = 64usize; // 16 packed bytes

        for b in 0..batches {
            let mut buf = QuatBuffer::<16>::new();
            for i in 0..quats_per_batch {
                let val = (((b as usize) + i) % 4) as u8;
                buf.push(Quat::from_u8(val).unwrap()).unwrap();
            }

            let mut wire = [0u8; 32];
            let frame_len = QuatFrame::<16>::encode(&buf, &mut wire).unwrap();
            let decoded = QuatFrame::<16>::decode(&wire[..frame_len]).unwrap();
            assert_eq!(decoded.len(), quats_per_batch);
        }

        let total_quats = batches * (quats_per_batch as u128);
        let elapsed = start.elapsed().as_nanos();
        report_bench(
            "10_medium_telemetry_stress",
            total_quats,
            elapsed,
            "5k frames x 64 quats",
        );
    }

    #[test]
    fn test_11_high_ring_and_rle_stream_stress() {
        let start = Instant::now();
        let total_quats = 250_000u128;

        let mut ring = QuatRingBuffer::<32>::new(); // 128 quats capacity
        let mut rle_stage = QuatBuffer::<32>::new();
        let mut rle_wire = [0u8; 256];

        for i in 0..total_quats {
            let q = Quat::from_u8(((i / 8) % 4) as u8).unwrap();
            let _ = ring.push_overwrite(q).unwrap();

            if ring.is_full() {
                rle_stage.clear();
                for _ in 0..64 {
                    rle_stage.push(ring.pop().unwrap()).unwrap();
                }
                let rle_len = rle_encode(&rle_stage, &mut rle_wire).unwrap();
                let restored = rle_decode::<32>(&rle_wire[..rle_len]).unwrap();
                assert_eq!(restored.len(), 64);
            }
        }

        let elapsed = start.elapsed().as_nanos();
        report_bench(
            "11_high_ring_rle_stream",
            total_quats,
            elapsed,
            "250k quats ring + RLE",
        );
    }

    #[test]
    fn test_12_max_stress_full_iot_pipeline() {
        let start = Instant::now();
        let total_quats = 1_000_000u128;
        let chunk_quats = 252usize; // 63 packed bytes (<= 255 quats for u8 header)

        let mut acc = QuatBuffer::<64>::new();
        let mut wire = [0u8; 128];
        let mut processed_quats = 0u128;

        for i in 0..total_quats {
            let q = Quat::from_u8(((i * 3 + (i >> 2)) % 4) as u8).unwrap();
            acc.push(q).unwrap();

            if acc.len() == chunk_quats {
                acc.map_in_place(|x| x.bit_xor(Quat::One)).unwrap();
                let frame_len = QuatFrame::<64>::encode(&acc, &mut wire).unwrap();
                let decoded = QuatFrame::<64>::decode(&wire[..frame_len]).unwrap();
                assert_eq!(decoded.len(), chunk_quats);
                processed_quats += chunk_quats as u128;
                acc.clear();
            }
        }

        if !acc.is_empty() {
            let rem = acc.len();
            acc.map_in_place(|x| x.bit_xor(Quat::One)).unwrap();
            let frame_len = QuatFrame::<64>::encode(&acc, &mut wire).unwrap();
            let decoded = QuatFrame::<64>::decode(&wire[..frame_len]).unwrap();
            assert_eq!(decoded.len(), rem);
            processed_quats += rem as u128;
            acc.clear();
        }

        assert_eq!(processed_quats, total_quats);
        let elapsed = start.elapsed().as_nanos();
        report_bench(
            "12_max_stress_iot_pipeline",
            total_quats,
            elapsed,
            "1M quats (250kB raw -> 62.5kB packed)",
        );
    }
}