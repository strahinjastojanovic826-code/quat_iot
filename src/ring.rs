use crate::error::QuatError;
use crate::quat::Quat;

/// Fixed-capacity `no_std` ring (FIFO) buffer storing 4 quats per backing byte.
/// Supports up to `MAX_BYTES * 4` quats without heap allocation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QuatRingBuffer<const MAX_BYTES: usize> {
    data: [u8; MAX_BYTES],
    head: usize,
    tail: usize,
    len: usize,
}

impl<const MAX_BYTES: usize> Default for QuatRingBuffer<MAX_BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_BYTES: usize> QuatRingBuffer<MAX_BYTES> {
    /// Creates an empty quaternary ring buffer backed by `MAX_BYTES` bytes.
    pub const fn new() -> Self {
        Self {
            data: [0u8; MAX_BYTES],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    /// Returns the maximum number of quats that can be held simultaneously.
    pub const fn capacity(&self) -> usize {
        MAX_BYTES.saturating_mul(4)
    }

    /// Returns the number of quats currently queued.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if the ring buffer contains no quats.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` if the ring buffer cannot accept more quats without popping or overwriting.
    pub const fn is_full(&self) -> bool {
        self.len >= self.capacity()
    }

    /// Resets the ring buffer to empty state.
    pub fn clear(&mut self) {
        self.data.fill(0);
        self.head = 0;
        self.tail = 0;
        self.len = 0;
    }

    fn write_slot(&mut self, slot: usize, quat: Quat) -> Result<(), QuatError> {
        let cap = self.capacity();
        if slot >= cap {
            return Err(QuatError::IndexOutOfBounds {
                index: slot,
                len: cap,
            });
        }
        let byte_idx = slot / 4;
        let shift = (slot % 4) * 2;
        let mask = !(0b11u8 << shift);
        self.data[byte_idx] = (self.data[byte_idx] & mask) | (quat.as_u8() << shift);
        Ok(())
    }

    fn read_slot(&self, slot: usize) -> Result<Quat, QuatError> {
        let cap = self.capacity();
        if slot >= cap {
            return Err(QuatError::IndexOutOfBounds {
                index: slot,
                len: cap,
            });
        }
        let byte_idx = slot / 4;
        let shift = (slot % 4) * 2;
        let raw = (self.data[byte_idx] >> shift) & 0b11;
        Quat::from_u8(raw)
    }

    /// Enqueues a single `Quat` at the tail. Returns `Err(QuatError::BufferFull)` if full.
    pub fn push(&mut self, quat: Quat) -> Result<(), QuatError> {
        let cap = self.capacity();
        if cap == 0 || self.len >= cap {
            return Err(QuatError::BufferFull);
        }
        self.write_slot(self.tail, quat)?;
        self.tail = (self.tail + 1) % cap;
        self.len += 1;
        Ok(())
    }

    /// Enqueues a `Quat` at the tail, overwriting the oldest element if full.
    /// Returns the evicted `Quat` if an overwrite occurred.
    pub fn push_overwrite(&mut self, quat: Quat) -> Result<Option<Quat>, QuatError> {
        let cap = self.capacity();
        if cap == 0 {
            return Err(QuatError::BufferFull);
        }
        let evicted = if self.len == cap {
            let old = self.read_slot(self.head)?;
            self.head = (self.head + 1) % cap;
            self.len -= 1;
            Some(old)
        } else {
            None
        };
        self.push(quat)?;
        Ok(evicted)
    }

    /// Dequeues and returns the oldest `Quat` from the head.
    pub fn pop(&mut self) -> Result<Quat, QuatError> {
        let cap = self.capacity();
        if self.len == 0 || cap == 0 {
            return Err(QuatError::BufferEmpty);
        }
        let quat = self.read_slot(self.head)?;
        self.write_slot(self.head, Quat::Zero)?;
        self.head = (self.head + 1) % cap;
        self.len -= 1;
        Ok(quat)
    }

    /// Peeks at the oldest `Quat` without removing it.
    pub fn peek(&self) -> Result<Quat, QuatError> {
        if self.len == 0 {
            return Err(QuatError::BufferEmpty);
        }
        self.read_slot(self.head)
    }
}