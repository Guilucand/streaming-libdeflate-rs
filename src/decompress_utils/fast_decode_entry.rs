use std::fmt::Debug;

use crate::{decompress_deflate::FAST_TABLEBITS, decompress_utils::decode_entry::DecodeEntry};

#[derive(Copy, Clone, PartialEq, Eq, Default)]
#[repr(C)] // Avoid reordering
pub struct FastDecodeEntry {
    bits: u8,
    flags: u8,
    len: u16,
    lit: u16,
    off: u16,
}

impl Debug for FastDecodeEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Entry: {} literals len: {} offset: {} consumed_bits: {} offset_bits: {} total_bits: {} {}",
            self.get_literals_count(),
            self.get_len_value(),
            self.get_offset_value(),
            self.get_consumed_bits(),
            self.get_offset_bits(),
            self.get_consumed_bits() +
            self.get_offset_bits(),
            if self.flags != 0 {
                format!("EXCEPTIONAL: {:b}", self.flags)
            } else {
                String::new()
            }
        )

        // f.debug_struct("FastDecodeEntry")
        //     .field("bits", &self.bits)
        //     .field("flags", &self.flags)
        //     .field("len", &self.len)
        //     .field("lit", &self.lit)
        //     .field("off", &self.off)
        //     .finish()
    }
}

impl FastDecodeEntry {
    const CONSUMED_BITS_OFFSET: usize = 4;
    const LENGTH_OFFSET: usize = 4;

    const MAX_LITERALS: u8 = 2;

    const EXCEPTIONAL: u8 = 0b10000000;

    /// All the extra bits are embedded in the entry
    pub const EXC_LEN_FULLSIZE: u8 = Self::EXCEPTIONAL | 0b0000000;

    /// Needs len extra bits
    pub const EXC_LEN_EXTRABIT: u8 = Self::EXCEPTIONAL | 0b0100000;

    /// Needs a subtable to fully decode a litlen
    pub const EXC_LEN_SUBTABLE: u8 = Self::EXCEPTIONAL | 0b1000000;

    /// Is the end of a block
    pub const EXC_END_OF_BLOCK: u8 = Self::EXCEPTIONAL | 0b1100000;

    pub const DEFAULT: FastDecodeEntry = FastDecodeEntry {
        bits: 0,
        flags: 0,
        len: 0,
        lit: 0,
        off: 0,
    };

    const fn create_bits(offset_bits: u8, consumed_bits: u8) -> u8 {
        consumed_bits << Self::CONSUMED_BITS_OFFSET | offset_bits
    }

    const fn create_len(length: u16, lit_count: u16) -> u16 {
        length << Self::LENGTH_OFFSET | lit_count
    }

    const fn create_extra_len(subtable_len: u8) -> u16 {
        subtable_len as u16
    }

    pub fn new_from_len(len: u16) -> Self {
        Self {
            bits: 0,
            flags: 0,
            len: Self::create_len(len, 0),
            lit: 0,
            off: 0,
        }
    }

    pub fn new_from_literal(lit: u8) -> Self {
        Self {
            bits: 0,
            flags: 0,
            len: Self::create_len(0, 1),
            lit: lit as u16,
            off: 0,
        }
    }

    pub fn new_from_litlen_entry(entry: DecodeEntry, codeword: usize) -> Self {
        if entry.is_exceptional() {
            Self {
                bits: Self::create_bits(0, entry.get_maintable_length()),
                flags: if entry.is_subtable_pointer() {
                    Self::EXC_LEN_SUBTABLE
                } else {
                    Self::EXC_END_OF_BLOCK
                },
                len: Self::create_extra_len(entry.get_subtable_length()),
                lit: 0,
                off: if entry.is_subtable_pointer() {
                    entry.get_result() as u16 // Subtable
                } else {
                    0
                },
            }
        } else {
            if entry.is_literal() {
                Self {
                    bits: Self::create_bits(0, entry.get_maintable_length()),
                    flags: 0,
                    len: Self::create_len(0, 1), // 1 literal
                    lit: entry.get_literal() as u16,
                    off: 0,
                }
            } else {
                let extra_len_bits = entry.get_subtable_length() as usize;

                if entry.get_maintable_length() as usize + extra_len_bits <= FAST_TABLEBITS {
                    let remaining_codeword = codeword >> entry.get_maintable_length();
                    let extra_len = (remaining_codeword % (1 << extra_len_bits)) as u16;

                    Self {
                        bits: Self::create_bits(
                            0,
                            entry.get_maintable_length() + extra_len_bits as u8,
                        ),
                        // Once created it is exceptional, the flag is removed if an offset can be decoded from the remaining bytes
                        flags: Self::EXC_LEN_FULLSIZE,
                        len: Self::create_len(entry.get_result() as u16 + extra_len, 0), // len (with extra bits) + 0 literals
                        lit: 0,
                        off: 0,
                    }
                } else {
                    // Only len without included extra bits
                    Self {
                        bits: Self::create_bits(
                            entry.get_subtable_length(),
                            entry.get_maintable_length(),
                        ),
                        flags: Self::EXC_LEN_EXTRABIT,
                        len: Self::create_len(entry.get_result() as u16, 0), // len + 0 literals
                        lit: 0,
                        off: 0,
                    }
                }
            }
        }
    }

    pub fn maybe_add_litlen_entry(&mut self, entry: DecodeEntry, codeword: usize) -> bool {
        debug_assert!(!entry.is_exceptional());

        assert!(self.get_literals_count() <= Self::MAX_LITERALS);

        if entry.is_literal() {
            if self.get_literals_count() >= Self::MAX_LITERALS {
                // No more literals can be added
                return false;
            }

            // Add the literal
            self.lit |= (entry.get_literal() as u16) << (self.get_literals_count() * 8);
            // Increment the literals count
            self.inc_literals_count();

            // Increment the used bits
            self.inc_consumed_bits(entry.get_maintable_length());

            assert!(self.get_literals_count() <= Self::MAX_LITERALS);
            true
        } else {
            // Length match
            let extra_len_bits = entry.get_subtable_length() as usize;

            let remaining_codeword = codeword >> entry.get_maintable_length();
            let extra_len = (remaining_codeword % (1 << extra_len_bits)) as u16;

            let total_len = entry.get_result() as u16 + extra_len;
            self.inc_len_value(total_len);
            self.inc_consumed_bits(entry.get_maintable_length() + entry.get_subtable_length());
            self.flags = Self::EXC_LEN_FULLSIZE;
            assert!(self.get_literals_count() <= Self::MAX_LITERALS);
            true
        }
    }

    pub fn add_offset_entry(&mut self, entry: DecodeEntry) {
        self.inc_consumed_bits(entry.get_maintable_length());
        self.inc_offset_bits(entry.get_subtable_length());
        self.set_offset_value(entry.get_result() as u16);

        assert!(self.get_literals_count() <= Self::MAX_LITERALS);

        self.flags = 0; // Normal entry
    }

    pub const fn get_subtable_index(&self) -> u32 {
        self.off as u32
    }

    pub fn get_subtable_length(&self) -> usize {
        self.len as usize
    }

    pub const fn get_offset_bits(&self) -> u8 {
        self.bits & 0xF
    }

    pub const fn get_exceptional_length_bits(&self) -> u8 {
        self.bits & 0xF
    }

    pub const fn inc_offset_bits(&mut self, amount: u8) {
        self.bits += amount;
    }

    pub const fn get_consumed_bits(&self) -> u8 {
        self.bits >> Self::CONSUMED_BITS_OFFSET
    }

    pub const fn inc_consumed_bits(&mut self, amount: u8) {
        self.bits += amount << Self::CONSUMED_BITS_OFFSET
    }

    pub const fn get_len_value(&self) -> u16 {
        self.len >> Self::LENGTH_OFFSET
    }

    pub const fn inc_len_value(&mut self, amount: u16) {
        self.len += amount << Self::LENGTH_OFFSET;
    }

    pub const fn get_offset_value(&self) -> u16 {
        self.off as u16
    }

    pub const fn set_offset_value(&mut self, amount: u16) {
        self.off = amount;
    }

    pub const fn get_literals(&self) -> u16 {
        self.lit
    }

    pub const fn get_flags(&self) -> u8 {
        self.flags
    }

    pub const fn is_subtable_pointer(&self) -> bool {
        self.flags == Self::EXC_LEN_SUBTABLE
    }

    pub const fn get_literals_count(&self) -> u8 {
        (self.len & 0x0F) as u8
    }

    const fn inc_literals_count(&mut self) {
        self.len += 1;
    }
}
