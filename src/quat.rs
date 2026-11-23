use crate::error::QuatError;

/// A single 2-bit quaternary digit representing values 0, 1, 2, or 3
/// (binary `00`, `01`, `10`, `11`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Quat {
    #[default]
    Zero = 0b00,
    One = 0b01,
    Two = 0b10,
    Three = 0b11,
}

impl Quat {
    /// Converts a `u8` (`0..=3`) into a `Quat`.
    pub const fn from_u8(value: u8) -> Result<Self, QuatError> {
        match value {
            0 => Ok(Self::Zero),
            1 => Ok(Self::One),
            2 => Ok(Self::Two),
            3 => Ok(Self::Three),
            _ => Err(QuatError::InvalidValue(value)),
        }
    }

    /// Converts a 2-bit pair (`high_bit`, `low_bit`, each `0` or `1`) into a `Quat`.
    pub const fn from_bits(high_bit: u8, low_bit: u8) -> Result<Self, QuatError> {
        if high_bit > 1 {
            return Err(QuatError::InvalidValue(high_bit));
        }
        if low_bit > 1 {
            return Err(QuatError::InvalidValue(low_bit));
        }
        Self::from_u8((high_bit << 1) | low_bit)
    }

    /// Returns the numeric value (`0..=3`).
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Returns the 2-bit representation as `(high_bit, low_bit)`.
    pub const fn as_bits(self) -> (u8, u8) {
        let v = self as u8;
        ((v >> 1) & 0b01, v & 0b01)
    }

    /// Bitwise AND across the 2-bit quaternary value.
    pub const fn bit_and(self, rhs: Self) -> Self {
        match Self::from_u8(self.as_u8() & rhs.as_u8()) {
            Ok(q) => q,
            Err(_) => Self::Zero,
        }
    }

    /// Bitwise OR across the 2-bit quaternary value.
    pub const fn bit_or(self, rhs: Self) -> Self {
        match Self::from_u8(self.as_u8() | rhs.as_u8()) {
            Ok(q) => q,
            Err(_) => Self::Zero,
        }
    }

    /// Bitwise XOR across the 2-bit quaternary value.
    pub const fn bit_xor(self, rhs: Self) -> Self {
        match Self::from_u8(self.as_u8() ^ rhs.as_u8()) {
            Ok(q) => q,
            Err(_) => Self::Zero,
        }
    }

    /// Bitwise NOT masked to the 2-bit quaternary range (`0b11`).
    pub const fn bit_not(self) -> Self {
        match Self::from_u8((!self.as_u8()) & 0b11) {
            Ok(q) => q,
            Err(_) => Self::Zero,
        }
    }
}

impl TryFrom<u8> for Quat {
    type Error = QuatError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::from_u8(value)
    }
}

impl From<Quat> for u8 {
    fn from(quat: Quat) -> Self {
        quat.as_u8()
    }
}