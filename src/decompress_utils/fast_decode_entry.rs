use std::fmt::Debug;

use crate::decompress_deflate::FAST_TABLEBITS;

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
            self.get_extra_offset_bits(),
            self.get_consumed_bits() +
            self.get_extra_offset_bits(),
            if self.flags != 0 {
                format!("EXCEPTIONAL: {:b}", self.flags)
            } else {
                String::new()
            }
        )
    }
}

impl FastDecodeEntry {
    const CONSUMED_BITS_OFFSET: usize = 4;
    const LENGTH_OFFSET: usize = 4;

    const MAX_LITERALS: u8 = 2;

    /// All the extra bits are embedded in the entry
    pub const EXC_LEN_FULLSIZE: u8 = 0b00000001;

    /// Needs len extra bits
    pub const EXC_LEN_EXTRABIT: u8 = 0b00000010;

    /// Needs a subtable to fully decode a litlen
    pub const EXC_LEN_SUBTABLE: u8 = 0b00000011;

    /// The length could be > 16 but is always <= 32
    pub const EXC_BIGLEN32: u8 = 0b00100000;

    /// The length could be larger than 32
    pub const EXC_BIGLEN: u8 = 0b00010000;

    /// The offset is too small to be copied, defaults to byte by byte copy
    /// This constant should be EXC_BIGLEN32 | EXC_BIGLEN to allow byte by byte copy even if the len is large
    pub const EXC_SMALLOFFSET: u8 = 0b00110000;

    /// Is the end of a block
    pub const EXC_END_OF_BLOCK: u8 = 0b00000111;

    pub const DEFAULT: FastDecodeEntry = FastDecodeEntry {
        bits: 0,
        flags: 0,
        len: 0,
        lit: 0,
        off: 0,
    };

    const fn compute_copy_flags(len: u16, extra_bits: u8) -> u8 {
        let max_extra = (1 << extra_bits) - 1;
        let max_len = len + max_extra;
        if max_len <= 16 {
            0
        } else if max_len <= 32 {
            Self::EXC_BIGLEN32
        } else {
            Self::EXC_BIGLEN
        }
    }

    const fn create_bits(offset_bits: u8, consumed_bits: u8) -> u8 {
        consumed_bits << Self::CONSUMED_BITS_OFFSET | offset_bits
    }

    const fn create_len(length: u16, lit_count: u16) -> u16 {
        length << Self::LENGTH_OFFSET | lit_count
    }

    pub const fn new_from_len(len: u16) -> Self {
        Self {
            bits: 0,
            flags: Self::compute_copy_flags(len, 0),
            len: Self::create_len(len, 0),
            lit: 0,
            off: 0,
        }
    }

    pub const fn new_from_len_with_flags(len: u16, flags: u8) -> Self {
        Self {
            bits: 0,
            flags,
            len: Self::create_len(len, 0),
            lit: 0,
            off: 0,
        }
    }

    pub const fn new_from_len_and_extra_bits(len: u16, extra_bits: u8) -> Self {
        Self {
            bits: Self::create_bits(extra_bits, 0),
            flags: Self::EXC_LEN_EXTRABIT | Self::compute_copy_flags(len, extra_bits),
            len: Self::create_len(len, 0),
            lit: 0,
            off: 0,
        }
    }

    pub const fn new_from_offset_and_extra_bits(offset: u16, extra_bits: u8) -> Self {
        Self {
            bits: Self::create_bits(extra_bits, 0),
            flags: if offset < 8 { Self::EXC_SMALLOFFSET } else { 0 },
            len: 0,
            lit: 0,
            off: offset,
        }
    }

    pub const fn new_from_literal(lit: u8) -> Self {
        Self {
            bits: 0,
            flags: 0,
            len: Self::create_len(0, 1),
            lit: lit as u16,
            off: 0,
        }
    }

    pub const fn new_presym(presym: u8) -> Self {
        Self {
            bits: 0,
            flags: presym,
            len: 0,
            lit: 0,
            off: 0,
        }
    }

    pub const fn new_end_of_block() -> Self {
        Self {
            bits: 0,
            flags: Self::EXC_END_OF_BLOCK,
            len: 0,
            lit: 0,
            off: 0,
        }
    }

    pub fn new_subtable_pointer(subtable_start: u16, subtable_bits: u8) -> Self {
        Self {
            bits: 0,
            flags: Self::EXC_LEN_SUBTABLE,
            len: subtable_bits as u16,
            lit: 0,
            off: subtable_start,
        }
    }

    pub fn new_from_litlen_entry(entry: FastDecodeEntry, codeword: usize) -> Self {
        if entry.flags == FastDecodeEntry::EXC_LEN_EXTRABIT {
            let consumed_bits = entry.get_consumed_bits();
            let extra_len_bits = entry.get_exceptional_length_bits();

            if consumed_bits as usize + extra_len_bits as usize <= FAST_TABLEBITS {
                let remaining_codeword = codeword >> consumed_bits;
                let extra_len = (remaining_codeword % (1 << extra_len_bits)) as u16;

                let total_len = entry.get_len_value() as u16 + extra_len;

                return Self {
                    bits: Self::create_bits(0, consumed_bits + extra_len_bits as u8),
                    // Once created it is exceptional, the flag is removed if an offset can be decoded from the remaining bytes
                    flags: Self::EXC_LEN_FULLSIZE | Self::compute_copy_flags(total_len, 0),
                    len: Self::create_len(total_len, 0), // len (with extra bits) + 0 literals
                    lit: 0,
                    off: 0,
                };
            }
        }

        entry
    }

    pub fn make_decode_table_entry(&self, consumed_bits: u8) -> Self {
        let mut self_ = *self;
        self_.inc_consumed_bits(consumed_bits);
        self_
    }

    pub fn maybe_add_litlen_entry(&mut self, entry: FastDecodeEntry, codeword: usize) -> bool {
        debug_assert!(!entry.is_subtable_pointer());

        if entry.has_literals() {
            if self.get_literals_count() >= Self::MAX_LITERALS {
                // No more literals can be added
                return false;
            }

            // Add the literal
            self.lit |= entry.get_literals() << (self.get_literals_count() * 8);
            // Increment the literals count
            self.inc_literals_count();

            // Increment the used bits
            self.inc_consumed_bits(entry.get_consumed_bits());

            true
        } else {
            // Length match
            let consumed_bits = entry.get_consumed_bits();
            let extra_bits = entry.get_exceptional_length_bits();

            let remaining_codeword = codeword >> consumed_bits;
            let extra_len = (remaining_codeword % (1 << extra_bits)) as u16;

            let total_len = entry.get_len_value() + extra_len;
            self.inc_len_value(total_len);
            self.inc_consumed_bits(extra_bits + consumed_bits);
            self.flags = Self::EXC_LEN_FULLSIZE;
            true
        }
    }

    pub fn add_offset_entry(&mut self, entry: FastDecodeEntry) {
        self.inc_consumed_bits(entry.get_consumed_bits());
        self.inc_offset_bits(entry.get_extra_offset_bits());
        self.set_offset_value(entry.get_offset_value());
        self.flags = entry.flags; // Normal entry
    }

    pub const fn get_subtable_index(&self) -> u32 {
        self.off as u32
    }

    pub fn get_subtable_length(&self) -> usize {
        self.len as usize
    }

    pub const fn get_extra_offset_bits(&self) -> u8 {
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

    pub const fn or_flags(&mut self, flags: u8) {
        self.flags |= flags;
    }

    pub const fn get_state_flags(&self) -> u8 {
        self.flags & 0xF
    }

    pub const fn get_copy_flags(&self) -> u8 {
        self.flags & 0xF0
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

    pub const fn has_literals(&self) -> bool {
        self.get_literals_count() > 0
    }

    pub const fn get_presym(&self) -> u8 {
        self.flags
    }
}
