/// Errors returned by quaternary packing, buffer, ring, RLE, and frame operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuatError {
    /// Input value is outside the valid quaternary range (0..=3).
    InvalidValue(u8),
    /// Target index is out of bounds for the buffer capacity.
    IndexOutOfBounds { index: usize, len: usize },
    /// Buffer or output slice does not have enough remaining capacity.
    BufferFull,
    /// Buffer or stream is empty and cannot satisfy a pop/read operation.
    BufferEmpty,
    /// Provided raw byte slice length is invalid for the target structure.
    InvalidByteLength { expected_max: usize, actual: usize },
    /// Frame header, payload length, or run-length stream is malformed.
    InvalidFrameLength,
    /// Checksum verification failed on an incoming frame.
    CrcMismatch { expected: u8, actual: u8 },
}