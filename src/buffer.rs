use crate::error::QuatError;
use crate::quat::Quat;

/// Fixed-capacity `no_std` buffer that packs 4 quats (8 bits total) into each `u8` byte.
/// `MAX_BYTES` defines the backing byte array size, supporting up to `MAX_BYTES * 4` quats.
///
/// Bit layout within a single byte (MSB to LSB):
/// `[ quat 3 (bits 7..6) | quat 2 (bits 5..4) | quat 1 (bits 3..2) | quat 0 (bits 1..0) ]`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QuatBuffer<const MAX_BYTES: usize> {
    data: [u8; MAX_BYTES],
    len: usize,
}

impl<const MAX_BYTES: usize> Default for QuatBuffer<MAX_BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_BYTES: usize> QuatBuffer<MAX_BYTES> {
    /// Creates an empty quaternary buffer backed by `MAX_BYTES` bytes.
    pub const fn new() -> Self {
        Self {
            data: [0u8; MAX_BYTES],
            len: 0,
        }
    }

    /// Reconstructs a `QuatBuffer` from packed raw bytes and an exact `quat_len`.
    pub fn from_raw_bytes(raw_bytes: &[u8], quat_len: usize) -> Result<Self, QuatError> {
        let max_quats = MAX_BYTES.saturating_mul(4);
        if quat_len > max_quats {
            return Err(QuatError::BufferFull);
        }
        let required_bytes = quat_len.div_ceil(4);
        if raw_bytes.len() < required_bytes || raw_bytes.len() > MAX_BYTES {
            return Err(QuatError::InvalidByteLength {
                expected_max: MAX_BYTES,
                actual: raw_bytes.len(),
            });
        }

        let mut buf = Self::new();
        buf.data[..required_bytes].copy_from_slice(&raw_bytes[..required_bytes]);
        buf.len = quat_len;
        Ok(buf)
    }

    /// Returns the number of quats currently stored.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if no quats are stored.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` if the buffer has reached its maximum quat capacity.
    pub const fn is_full(&self) -> bool {
        self.len >= self.capacity()
    }

    /// Returns the maximum number of quats this buffer can hold.
    pub const fn capacity(&self) -> usize {
        MAX_BYTES.saturating_mul(4)
    }

    /// Returns the number of underlying bytes allocated for the buffer.
    pub const fn byte_capacity(&self) -> usize {
        MAX_BYTES
    }

    /// Clears all stored quats and resets byte storage to zero.
    pub fn clear(&mut self) {
        self.data.fill(0);
        self.len = 0;
    }

    /// Appends a single `Quat` to the end of the buffer.
    pub fn push(&mut self, quat: Quat) -> Result<(), QuatError> {
        if self.len >= self.capacity() {
            return Err(QuatError::BufferFull);
        }
        let idx = self.len;
        self.len += 1;
        self.set(idx, quat)
    }

    /// Removes and returns the last `Quat` in the buffer.
    pub fn pop(&mut self) -> Result<Quat, QuatError> {
        if self.len == 0 {
            return Err(QuatError::BufferEmpty);
        }
        let last_idx = self.len - 1;
        let quat = self.get(last_idx)?;
        self.set(last_idx, Quat::Zero)?;
        self.len = last_idx;
        Ok(quat)
    }

    /// Appends a slice of `Quat` values atomically (rolls back if capacity is exceeded).
    pub fn push_slice(&mut self, quats: &[Quat]) -> Result<(), QuatError> {
        let original_len = self.len;
        for &q in quats {
            if let Err(err) = self.push(q) {
                self.len = original_len;
                return Err(err);
            }
        }
        Ok(())
    }

    /// Reads the `Quat` at `index`.
    pub fn get(&self, index: usize) -> Result<Quat, QuatError> {
        if index >= self.len {
            return Err(QuatError::IndexOutOfBounds {
                index,
                len: self.len,
            });
        }
        let byte_idx = index / 4;
        let shift = (index % 4) * 2;
        let raw = (self.data[byte_idx] >> shift) & 0b11;
        Quat::from_u8(raw)
    }

    /// Overwrites an existing `Quat` at `index`.
    pub fn set(&mut self, index: usize, quat: Quat) -> Result<(), QuatError> {
        if index >= self.len {
            return Err(QuatError::IndexOutOfBounds {
                index,
                len: self.len,
            });
        }
        let byte_idx = index / 4;
        let shift = (index % 4) * 2;
        let mask = !(0b11u8 << shift);
        self.data[byte_idx] = (self.data[byte_idx] & mask) | (quat.as_u8() << shift);
        Ok(())
    }

    /// Transforms every stored `Quat` in place using `f`.
    pub fn map_in_place<F>(&mut self, mut f: F) -> Result<(), QuatError>
    where
        F: FnMut(Quat) -> Quat,
    {
        for i in 0..self.len {
            let current = self.get(i)?;
            self.set(i, f(current))?;
        }
        Ok(())
    }

    /// Exposes the packed raw byte slice currently used by stored quats.
    pub fn as_bytes(&self) -> &[u8] {
        let used_bytes = self.len.div_ceil(4);
        &self.data[..used_bytes]
    }

    /// Returns an iterator over all stored `Quat` values.
    pub fn iter(&self) -> QuatBufferIter<'_, MAX_BYTES> {
        QuatBufferIter {
            buffer: self,
            index: 0,
        }
    }
}

/// Iterator yielding `Quat` elements from a `QuatBuffer`.
pub struct QuatBufferIter<'a, const MAX_BYTES: usize> {
    buffer: &'a QuatBuffer<MAX_BYTES>,
    index: usize,
}

impl<'a, const MAX_BYTES: usize> Iterator for QuatBufferIter<'a, MAX_BYTES> {
    type Item = Quat;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.buffer.len() {
            return None;
        }
        let quat = self.buffer.get(self.index).ok()?;
        self.index += 1;
        Some(quat)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.buffer.len().saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl<'a, const MAX_BYTES: usize> ExactSizeIterator for QuatBufferIter<'a, MAX_BYTES> {
    fn len(&self) -> usize {
        self.buffer.len().saturating_sub(self.index)
    }
}

impl<'a, const MAX_BYTES: usize> IntoIterator for &'a QuatBuffer<MAX_BYTES> {
    type Item = Quat;
    type IntoIter = QuatBufferIter<'a, MAX_BYTES>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}