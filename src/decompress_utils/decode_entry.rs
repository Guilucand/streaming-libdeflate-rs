/*
* A decode table for order TABLEBITS consists of a main table of (1 <<
* TABLEBITS) entries followed by a variable number of subtables.
*
* The decoding algorithm takes the next TABLEBITS bits of compressed data and
* uses them as an index into the decode table.  The resulting entry is either a
* "direct entry", meaning that it contains the value desired, or a "subtable
* pointer", meaning that the entry references a subtable that must be indexed
* using more bits of the compressed data to decode the symbol.
*
* Each decode table (a main table along with its subtables, if any) is
* associated with a Huffman code.  Logically, the result of a decode table
* lookup is a symbol from the alphabet from which the corresponding Huffman
* code was constructed.  A symbol with codeword length n <= TABLEBITS is
* associated with 2**(TABLEBITS - n) direct entries in the table, whereas a
* symbol with codeword length n > TABLEBITS is associated with one or more
* subtable entries.
*
* On top of this basic design, we implement several optimizations:
*
* - We store the length of each codeword directly in each of its decode table
*   entries.  This allows the codeword length to be produced without indexing
*   an additional table.
*
* - When beneficial, we don't store the Huffman symbol itself, but instead data
*   generated from it.  For example, when decoding an offset symbol in DEFLATE,
*   it's more efficient if we can decode the offset base and number of extra
*   offset bits directly rather than decoding the offset symbol and then
*   looking up both of those values in an additional table or tables.
*
* The size of each decode table entry is 32 bits, which provides slightly
* better performance than 16-bit entries on 32 and 64 bit processers, provided
* that the table doesn't get so large that it takes up too much memory and
* starts generating cache misses.  The bits of each decode table entry are
* defined as follows:
* LEGACY:
* - Bits 30 -- 31: flags (see below)
* - Bits 8 -- 29: decode result: a Huffman symbol or related data
* - Bits 0 -- 7: codeword length
* NEW:
 * Here is the format of our litlen decode table entries.  Bits not explicitly
 * described contain zeroes:
 *
 *	Literals:
 *		Bit 31:     1 (HUFFDEC_LITERAL)
 *		Bit 23-16:  literal value
 *		Bit 15:     0 (!HUFFDEC_EXCEPTIONAL)
 *		Bit 14:     0 (!HUFFDEC_SUBTABLE_POINTER)
 *		Bit 13:     0 (!HUFFDEC_END_OF_BLOCK)
 *		Bit 11-8:   remaining codeword length [not used]
 *		Bit 3-0:    remaining codeword length
 *	Lengths:
 *		Bit 31:     0 (!HUFFDEC_LITERAL)
 *		Bit 24-16:  length base value
 *		Bit 15:     0 (!HUFFDEC_EXCEPTIONAL)
 *		Bit 14:     0 (!HUFFDEC_SUBTABLE_POINTER)
 *		Bit 13:     0 (!HUFFDEC_END_OF_BLOCK)
 *		Bit 11-8:   remaining codeword length
 *		Bit 4-0:    remaining codeword length + number of extra bits
 *	End of block:
 *		Bit 31:     0 (!HUFFDEC_LITERAL)
 *		Bit 15:     1 (HUFFDEC_EXCEPTIONAL)
 *		Bit 14:     0 (!HUFFDEC_SUBTABLE_POINTER)
 *		Bit 13:     1 (HUFFDEC_END_OF_BLOCK)
 *		Bit 11-8:   remaining codeword length [not used]
 *		Bit 3-0:    remaining codeword length
 *	Subtable pointer:
 *		Bit 31:     0 (!HUFFDEC_LITERAL)
 *		Bit 30-16:  index of start of subtable
 *		Bit 15:     1 (HUFFDEC_EXCEPTIONAL)
 *		Bit 14:     1 (HUFFDEC_SUBTABLE_POINTER)
 *		Bit 13:     0 (!HUFFDEC_END_OF_BLOCK)
 *		Bit 11-8:   number of subtable bits
 *		Bit 3-0:    number of main table bits
 *
 * This format has several desirable properties:
 *
 *	- The codeword length, length slot base, and number of extra length bits
 *	  are all built in.  This eliminates the need to separately look up this
 *	  information by indexing separate arrays by symbol or length slot.
 *
 *	- The HUFFDEC_* flags enable easily distinguishing between the different
 *	  types of entries.  The HUFFDEC_LITERAL flag enables a fast path for
 *	  literals; the high bit is used for this, as some CPUs can test the
 *	  high bit more easily than other bits.  The HUFFDEC_EXCEPTIONAL flag
 *	  makes it possible to detect the two unlikely cases (subtable pointer
 *	  and end of block) in a single bit flag test.
 *
 *	- The low byte is the number of bits that need to be removed from the
 *	  bitstream; this makes this value easily accessible, and it enables the
 *	  micro-optimization of doing 'bitsleft -= entry' instead of
 *	  'bitsleft -= (u8)entry'.  It also includes the number of extra bits,
 *	  so they don't need to be removed separately.
 *
 *	- The flags in bits 15-13 are arranged to be 0 when the
 *	  "remaining codeword length" in bits 11-8 is needed, making this value
 *	  fairly easily accessible as well via a shift and downcast.
 *
 *	- Similarly, bits 13-12 are 0 when the "subtable bits" in bits 11-8 are
 *	  needed, making it possible to extract this value with '& 0x3F' rather
 *	  than '& 0xF'.  This value is only used as a shift amount, so this can
 *	  save an 'and' instruction as the masking by 0x3F happens implicitly.
 *
 * litlen_decode_results[] contains the static part of the entry for each
 * symbol.  make_decode_table_entry() produces the final entries.
*/

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[repr(transparent)]
pub struct DecodeEntry(pub u32);

impl DecodeEntry {
    /* Indicates a literal entry in the litlen decode table */
    const ENTRY_LITERAL: u32 = 0x80000000;

    /* Indicates that HUFFDEC_SUBTABLE_POINTER or HUFFDEC_END_OF_BLOCK is set */
    const ENTRY_EXCEPTIONAL: u32 = 0x00008000;

    /* Indicates a subtable pointer entry in the litlen or offset decode table */
    const ENTRY_SUBTABLE_POINTER: u32 = 0x00004000;

    /* Indicates an end-of-block entry in the litlen decode table */
    const ENTRY_END_OF_BLOCK: u32 = 0x00002000;

    /* Shift to extract the decode result from a decode table entry.  */
    pub const ENTRY_RESULT_SHIFT: usize = 16;

    pub const ENTRY_SUBTABLE_BITS_SHIFT: usize = 8;

    /* Mask for extracting the codeword length from a decode table entry.  */
    pub const ENTRY_LENGTH_MASK: u32 = 0x3F;

    #[inline(always)]
    pub const fn new_literal(literal: u8) -> Self {
        DecodeEntry(Self::ENTRY_LITERAL | ((literal as u32) << Self::ENTRY_RESULT_SHIFT))
    }

    #[inline(always)]
    pub const fn new_result(result: u32) -> Self {
        DecodeEntry(result << Self::ENTRY_RESULT_SHIFT)
    }

    #[inline(always)]
    pub const fn new_length(length: u32, num_extra_bits: u8) -> Self {
        DecodeEntry(
            (length << Self::ENTRY_RESULT_SHIFT)
                | ((num_extra_bits as u32) << Self::ENTRY_SUBTABLE_BITS_SHIFT),
        )
    }

    #[inline(always)]
    pub const fn new_offset(offset_base: u32, num_extra_bits: u8) -> DecodeEntry {
        DecodeEntry(
            (offset_base << Self::ENTRY_RESULT_SHIFT)
                | ((num_extra_bits as u32) << Self::ENTRY_SUBTABLE_BITS_SHIFT),
        )
    }

    #[inline(always)]
    pub const fn new_subtable_pointer(
        subtable_index: u32,
        subtable_bits: u8,
        main_table_bits: u8,
    ) -> Self {
        DecodeEntry(
            Self::ENTRY_SUBTABLE_POINTER
                | Self::ENTRY_EXCEPTIONAL
                | (subtable_index << Self::ENTRY_RESULT_SHIFT)
                | (subtable_bits as u32) << Self::ENTRY_SUBTABLE_BITS_SHIFT
                | (main_table_bits as u32),
        )
    }

    pub const fn is_literal(self) -> bool {
        (self.0 & Self::ENTRY_LITERAL) != 0
    }

    #[inline(always)]
    pub const fn is_exceptional(self) -> bool {
        (self.0 & Self::ENTRY_EXCEPTIONAL) != 0
    }

    #[inline(always)]
    pub const fn is_subtable_pointer(self) -> bool {
        (self.0 & Self::ENTRY_SUBTABLE_POINTER) != 0
    }

    #[inline(always)]
    pub const fn is_end_of_block(self) -> bool {
        (self.0 & Self::ENTRY_END_OF_BLOCK) != 0
    }

    #[inline(always)]
    pub const fn get_result(self) -> u32 {
        self.0 >> Self::ENTRY_RESULT_SHIFT
    }

    #[inline(always)]
    pub const fn get_literal(self) -> u8 {
        (self.0 >> Self::ENTRY_RESULT_SHIFT) as u8
    }

    #[inline(always)]
    pub const fn get_subtable_length(self) -> u8 {
        ((self.0 >> Self::ENTRY_SUBTABLE_BITS_SHIFT) & Self::ENTRY_LENGTH_MASK) as u8
    }

    #[inline(always)]
    pub const fn get_maintable_length(self) -> u8 {
        self.0 as u8
    }

    /*
     * make_decode_table_entry() creates a decode table entry for the given symbol
     * by combining the static part 'decode_results[sym]' with the dynamic part
     * 'len', which is the remaining codeword length (the codeword length for main
     * table entries, or the codeword length minus TABLEBITS for subtable entries).
     *
     * In all cases, we add 'len' to each of the two low-order bytes to create the
     * appropriately-formatted decode table entry.  See the definitions of the
     * *_decode_results[] arrays below, where the entry format is described.
     */
    pub fn make_decode_table_entry(&self, len: u32) -> DecodeEntry {
        Self(self.0 | len)
    }

    pub const fn end_of_block() -> DecodeEntry {
        Self(Self::ENTRY_EXCEPTIONAL | Self::ENTRY_END_OF_BLOCK)
    }
}
