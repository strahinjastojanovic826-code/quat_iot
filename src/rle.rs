use crate::buffer::QuatBuffer;
use crate::error::QuatError;
use crate::quat::Quat;

/// Single run-length encoded entry: a quaternary symbol and its repeat count (`1..=255`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuatRun {
    pub quat: Quat,
    pub count: u8,
}

impl QuatRun {
    /// Serializes this run into 2 bytes: `[quat_u8, count]`.
    pub const fn to_bytes(self) -> [u8; 2] {
        [self.quat.as_u8(), self.count]
    }

    /// Parses a run from a 2-byte pair: `[quat_u8, count]`.
    pub const fn from_bytes(pair: [u8; 2]) -> Result<Self, QuatError> {
        if pair[1] == 0 {
            return Err(QuatError::InvalidFrameLength);
        }
        match Quat::from_u8(pair[0]) {
            Ok(quat) => Ok(Self {
                quat,
                count: pair[1],
            }),
            Err(e) => Err(e),
        }
    }
}

/// Encodes a `QuatBuffer` into byte pairs `[quat_u8, count]` in `out_encoded`.
/// Returns the number of bytes written (`runs * 2`).
pub fn rle_encode<const BUF_BYTES: usize>(
    input: &QuatBuffer<BUF_BYTES>,
    out_encoded: &mut [u8],
) -> Result<usize, QuatError> {
    if input.is_empty() {
        return Ok(0);
    }

    let mut written = 0usize;
    let mut current_quat = input.get(0)?;
    let mut current_count: u8 = 1;

    for i in 1..input.len() {
        let q = input.get(i)?;
        if q == current_quat && current_count < u8::MAX {
            current_count += 1;
        } else {
            if written.saturating_add(2) > out_encoded.len() {
                return Err(QuatError::BufferFull);
            }
            let pair = QuatRun {
                quat: current_quat,
                count: current_count,
            }
            .to_bytes();
            out_encoded[written] = pair[0];
            out_encoded[written + 1] = pair[1];
            written += 2;

            current_quat = q;
            current_count = 1;
        }
    }

    if written.saturating_add(2) > out_encoded.len() {
        return Err(QuatError::BufferFull);
    }
    let pair = QuatRun {
        quat: current_quat,
        count: current_count,
    }
    .to_bytes();
    out_encoded[written] = pair[0];
    out_encoded[written + 1] = pair[1];
    written += 2;

    Ok(written)
}

/// Decodes `[quat_u8, count]` byte pairs back into a fixed-capacity `QuatBuffer`.
pub fn rle_decode<const OUT_BYTES: usize>(encoded: &[u8]) -> Result<QuatBuffer<OUT_BYTES>, QuatError> {
    if encoded.len() % 2 != 0 {
        return Err(QuatError::InvalidFrameLength);
    }

    let mut out = QuatBuffer::<OUT_BYTES>::new();
    let mut idx = 0usize;
    while idx < encoded.len() {
        let run = QuatRun::from_bytes([encoded[idx], encoded[idx + 1]])?;
        for _ in 0..run.count {
            out.push(run.quat)?;
        }
        idx += 2;
    }

    Ok(out)
}