use crate::buffer::QuatBuffer;
use crate::error::QuatError;

/// Computes CRC-8 (polynomial `0x07`, init `0x00`) over a byte slice.
pub fn crc8(bytes: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &b in bytes {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Compact IoT wire frame format:
/// `[ quat_count: u8 | packed_payload: [u8; payload_len] | crc8: u8 ]`
///
/// Supported payload size per frame is up to `MAX_BYTES` (`<= 255` bytes, up to `MAX_BYTES * 4` quats).
pub struct QuatFrame<const MAX_BYTES: usize>;

impl<const MAX_BYTES: usize> QuatFrame<MAX_BYTES> {
    /// Inspects the header of a wire frame and returns the declared quat count.
    pub fn peek_quat_count(wire: &[u8]) -> Result<usize, QuatError> {
        if wire.len() < 2 {
            return Err(QuatError::InvalidFrameLength);
        }
        Ok(wire[0] as usize)
    }

    /// Encodes a `QuatBuffer` into `out_wire` and returns the total written byte length.
    pub fn encode<const BUF_BYTES: usize>(
        buffer: &QuatBuffer<BUF_BYTES>,
        out_wire: &mut [u8],
    ) -> Result<usize, QuatError> {
        let payload = buffer.as_bytes();
        if payload.len() > MAX_BYTES || buffer.len() > u8::MAX as usize {
            return Err(QuatError::InvalidFrameLength);
        }

        let total_len = 1 + payload.len() + 1;
        if out_wire.len() < total_len {
            return Err(QuatError::BufferFull);
        }

        out_wire[0] = buffer.len() as u8;
        out_wire[1..1 + payload.len()].copy_from_slice(payload);
        let checksum = crc8(&out_wire[..1 + payload.len()]);
        out_wire[1 + payload.len()] = checksum;

        Ok(total_len)
    }

    /// Encodes a `QuatBuffer` directly into a target `QuatBuffer` of wire bytes
    /// (packing each wire byte as 4 quats is not applicable here; use `encode` for raw wire bytes).
    /// This helper writes the framed wire bytes into a `QuatBuffer<WIRE_BYTES>` only if you pack
    /// raw bytes externally; for direct byte slice output, prefer `encode`.
    pub fn encoded_len<const BUF_BYTES: usize>(buffer: &QuatBuffer<BUF_BYTES>) -> Result<usize, QuatError> {
        let payload_len = buffer.as_bytes().len();
        if payload_len > MAX_BYTES || buffer.len() > u8::MAX as usize {
            return Err(QuatError::InvalidFrameLength);
        }
        Ok(1 + payload_len + 1)
    }

    /// Validates CRC-8 and decodes a wire frame into a `QuatBuffer`.
    pub fn decode(wire: &[u8]) -> Result<QuatBuffer<MAX_BYTES>, QuatError> {
        if wire.len() < 2 {
            return Err(QuatError::InvalidFrameLength);
        }

        let quat_count = wire[0] as usize;
        let expected_payload_len = quat_count.div_ceil(4);
        if expected_payload_len > MAX_BYTES {
            return Err(QuatError::InvalidFrameLength);
        }

        let expected_total_len = 1 + expected_payload_len + 1;
        if wire.len() != expected_total_len {
            return Err(QuatError::InvalidFrameLength);
        }

        let actual_crc = wire[wire.len() - 1];
        let computed_crc = crc8(&wire[..wire.len() - 1]);
        if actual_crc != computed_crc {
            return Err(QuatError::CrcMismatch {
                expected: computed_crc,
                actual: actual_crc,
            });
        }

        let payload = &wire[1..1 + expected_payload_len];
        QuatBuffer::from_raw_bytes(payload, quat_count)
    }
}