/*****************************************************************************
 *				Input bitstream                              *
 *****************************************************************************/

/*
 * The state of the "input bitstream" consists of the following variables:
 *
 *	- in_next: pointer to the next unread byte in the input buffer
 *
 *	- in_end: pointer just past the end of the input buffer
 *
 *	- bitbuf: a word-sized variable containing bits that have been read from
 *		  the input buffer.  The buffered bits are right-aligned
 *		  (they're the low-order bits).
 *
 *	- bitsleft: number of bits in 'bitbuf' that are valid.
 *
 * To make it easier for the compiler to optimize the code by keeping variables
 * in registers, these are declared as normal variables and manipulated using
 * macros.
 */

use crate::bitstream::BitStream;
/*
 * The type for the bitbuffer variable ('bitbuf' described above).  For best
 * performance, this should have size equal to a machine word.
 *
 * 64-bit platforms have a significant advantage: they get a bigger bitbuffer
 * which they have to fill less often.
 */
use crate::decompress_deflate::{
    decompress_direct, decompress_with_instr, deflate_decompress_template, LenType, LITLEN_ENOUGH,
    LITLEN_TABLEBITS, OFFSET_ENOUGH, OFFSET_TABLEBITS, PRECODE_ENOUGH, PRECODE_TABLEBITS,
};
use crate::deflate_constants::*;
use crate::unchecked::{UncheckedArray, UncheckedSlice};
use crate::{DeflateInput, DeflateOutput, LibdeflateDecompressor, LibdeflateError};
use nightly_quirks::branch_pred::unlikely;

pub struct DecompressTempData<'a, I: DeflateInput> {
    pub input_bitstream: BitStream<'a, I>,
    pub num_litlen_syms: usize,
    pub num_offset_syms: usize,
    pub block_type: u32,
    pub entry: DecodeEntry,
    pub is_final_block: bool,
}

/*****************************************************************************
 *                              Huffman decoding                             *
 *****************************************************************************/

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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct DecodeEntry(pub u32);

impl DecodeEntry {
    pub const DEFAULT: DecodeEntry = DecodeEntry(0);

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
    const fn new_literal(literal: u8) -> Self {
        DecodeEntry(Self::ENTRY_LITERAL | ((literal as u32) << Self::ENTRY_RESULT_SHIFT))
    }

    #[inline(always)]
    const fn new_result(result: u32) -> Self {
        DecodeEntry(result << Self::ENTRY_RESULT_SHIFT)
    }

    #[inline(always)]
    const fn new_length(length: u32, num_extra_bits: u8) -> Self {
        DecodeEntry(
            (length << Self::ENTRY_RESULT_SHIFT)
                | ((num_extra_bits as u32) << Self::ENTRY_SUBTABLE_BITS_SHIFT),
        )
    }

    #[inline(always)]
    const fn new_offset(offset_base: u32, num_extra_bits: u8) -> DecodeEntry {
        DecodeEntry(
            (offset_base << Self::ENTRY_RESULT_SHIFT)
                | ((num_extra_bits as u32) << Self::ENTRY_SUBTABLE_BITS_SHIFT),
        )
    }

    #[inline(always)]
    const fn new_subtable_pointer(
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
    fn make_decode_table_entry(&self, len: u32) -> DecodeEntry {
        Self(self.0 | len)
    }

    const fn end_of_block() -> DecodeEntry {
        Self(Self::ENTRY_EXCEPTIONAL | Self::ENTRY_END_OF_BLOCK)
    }
}

// /* Shift a decode result into its position in the decode table entry.  */
// #[inline(always)]
// const fn huffdec_result_entry(result: u32) -> u32 {
//     result << HUFFDEC_RESULT_SHIFT
// }

/* The decode result for each precode symbol.  There is no special optimization
 * for the precode; the decode result is simply the symbol value.  */
#[inline(always)]
const fn hr_entry(presym: u32) -> DecodeEntry {
    DecodeEntry::new_result(presym)
}

const PRECODE_DECODE_RESULTS: UncheckedArray<DecodeEntry, DEFLATE_NUM_PRECODE_SYMS> =
    UncheckedArray([
        hr_entry(0),
        hr_entry(1),
        hr_entry(2),
        hr_entry(3),
        hr_entry(4),
        hr_entry(5),
        hr_entry(6),
        hr_entry(7),
        hr_entry(8),
        hr_entry(9),
        hr_entry(10),
        hr_entry(11),
        hr_entry(12),
        hr_entry(13),
        hr_entry(14),
        hr_entry(15),
        hr_entry(16),
        hr_entry(17),
        hr_entry(18),
    ]);

/* The decode result for each litlen symbol.  For literals, this is the literal
 * value itself and the HUFFDEC_LITERAL flag.  For lengths, this is the length
 * base and the number of extra length bits.  */

#[inline(always)]
const fn ld1_entry(literal: u8) -> DecodeEntry {
    DecodeEntry::new_literal(literal)
}

#[inline(always)]
const fn ld2_entry(length_base: u32, num_extra_bits: u8) -> DecodeEntry {
    DecodeEntry::new_length(length_base, num_extra_bits)
}

const LITLEN_DECODE_RESULTS: UncheckedArray<DecodeEntry, DEFLATE_NUM_LITLEN_SYMS> =
    UncheckedArray([
        /* Literals  */
        ld1_entry(0),
        ld1_entry(1),
        ld1_entry(2),
        ld1_entry(3),
        ld1_entry(4),
        ld1_entry(5),
        ld1_entry(6),
        ld1_entry(7),
        ld1_entry(8),
        ld1_entry(9),
        ld1_entry(10),
        ld1_entry(11),
        ld1_entry(12),
        ld1_entry(13),
        ld1_entry(14),
        ld1_entry(15),
        ld1_entry(16),
        ld1_entry(17),
        ld1_entry(18),
        ld1_entry(19),
        ld1_entry(20),
        ld1_entry(21),
        ld1_entry(22),
        ld1_entry(23),
        ld1_entry(24),
        ld1_entry(25),
        ld1_entry(26),
        ld1_entry(27),
        ld1_entry(28),
        ld1_entry(29),
        ld1_entry(30),
        ld1_entry(31),
        ld1_entry(32),
        ld1_entry(33),
        ld1_entry(34),
        ld1_entry(35),
        ld1_entry(36),
        ld1_entry(37),
        ld1_entry(38),
        ld1_entry(39),
        ld1_entry(40),
        ld1_entry(41),
        ld1_entry(42),
        ld1_entry(43),
        ld1_entry(44),
        ld1_entry(45),
        ld1_entry(46),
        ld1_entry(47),
        ld1_entry(48),
        ld1_entry(49),
        ld1_entry(50),
        ld1_entry(51),
        ld1_entry(52),
        ld1_entry(53),
        ld1_entry(54),
        ld1_entry(55),
        ld1_entry(56),
        ld1_entry(57),
        ld1_entry(58),
        ld1_entry(59),
        ld1_entry(60),
        ld1_entry(61),
        ld1_entry(62),
        ld1_entry(63),
        ld1_entry(64),
        ld1_entry(65),
        ld1_entry(66),
        ld1_entry(67),
        ld1_entry(68),
        ld1_entry(69),
        ld1_entry(70),
        ld1_entry(71),
        ld1_entry(72),
        ld1_entry(73),
        ld1_entry(74),
        ld1_entry(75),
        ld1_entry(76),
        ld1_entry(77),
        ld1_entry(78),
        ld1_entry(79),
        ld1_entry(80),
        ld1_entry(81),
        ld1_entry(82),
        ld1_entry(83),
        ld1_entry(84),
        ld1_entry(85),
        ld1_entry(86),
        ld1_entry(87),
        ld1_entry(88),
        ld1_entry(89),
        ld1_entry(90),
        ld1_entry(91),
        ld1_entry(92),
        ld1_entry(93),
        ld1_entry(94),
        ld1_entry(95),
        ld1_entry(96),
        ld1_entry(97),
        ld1_entry(98),
        ld1_entry(99),
        ld1_entry(100),
        ld1_entry(101),
        ld1_entry(102),
        ld1_entry(103),
        ld1_entry(104),
        ld1_entry(105),
        ld1_entry(106),
        ld1_entry(107),
        ld1_entry(108),
        ld1_entry(109),
        ld1_entry(110),
        ld1_entry(111),
        ld1_entry(112),
        ld1_entry(113),
        ld1_entry(114),
        ld1_entry(115),
        ld1_entry(116),
        ld1_entry(117),
        ld1_entry(118),
        ld1_entry(119),
        ld1_entry(120),
        ld1_entry(121),
        ld1_entry(122),
        ld1_entry(123),
        ld1_entry(124),
        ld1_entry(125),
        ld1_entry(126),
        ld1_entry(127),
        ld1_entry(128),
        ld1_entry(129),
        ld1_entry(130),
        ld1_entry(131),
        ld1_entry(132),
        ld1_entry(133),
        ld1_entry(134),
        ld1_entry(135),
        ld1_entry(136),
        ld1_entry(137),
        ld1_entry(138),
        ld1_entry(139),
        ld1_entry(140),
        ld1_entry(141),
        ld1_entry(142),
        ld1_entry(143),
        ld1_entry(144),
        ld1_entry(145),
        ld1_entry(146),
        ld1_entry(147),
        ld1_entry(148),
        ld1_entry(149),
        ld1_entry(150),
        ld1_entry(151),
        ld1_entry(152),
        ld1_entry(153),
        ld1_entry(154),
        ld1_entry(155),
        ld1_entry(156),
        ld1_entry(157),
        ld1_entry(158),
        ld1_entry(159),
        ld1_entry(160),
        ld1_entry(161),
        ld1_entry(162),
        ld1_entry(163),
        ld1_entry(164),
        ld1_entry(165),
        ld1_entry(166),
        ld1_entry(167),
        ld1_entry(168),
        ld1_entry(169),
        ld1_entry(170),
        ld1_entry(171),
        ld1_entry(172),
        ld1_entry(173),
        ld1_entry(174),
        ld1_entry(175),
        ld1_entry(176),
        ld1_entry(177),
        ld1_entry(178),
        ld1_entry(179),
        ld1_entry(180),
        ld1_entry(181),
        ld1_entry(182),
        ld1_entry(183),
        ld1_entry(184),
        ld1_entry(185),
        ld1_entry(186),
        ld1_entry(187),
        ld1_entry(188),
        ld1_entry(189),
        ld1_entry(190),
        ld1_entry(191),
        ld1_entry(192),
        ld1_entry(193),
        ld1_entry(194),
        ld1_entry(195),
        ld1_entry(196),
        ld1_entry(197),
        ld1_entry(198),
        ld1_entry(199),
        ld1_entry(200),
        ld1_entry(201),
        ld1_entry(202),
        ld1_entry(203),
        ld1_entry(204),
        ld1_entry(205),
        ld1_entry(206),
        ld1_entry(207),
        ld1_entry(208),
        ld1_entry(209),
        ld1_entry(210),
        ld1_entry(211),
        ld1_entry(212),
        ld1_entry(213),
        ld1_entry(214),
        ld1_entry(215),
        ld1_entry(216),
        ld1_entry(217),
        ld1_entry(218),
        ld1_entry(219),
        ld1_entry(220),
        ld1_entry(221),
        ld1_entry(222),
        ld1_entry(223),
        ld1_entry(224),
        ld1_entry(225),
        ld1_entry(226),
        ld1_entry(227),
        ld1_entry(228),
        ld1_entry(229),
        ld1_entry(230),
        ld1_entry(231),
        ld1_entry(232),
        ld1_entry(233),
        ld1_entry(234),
        ld1_entry(235),
        ld1_entry(236),
        ld1_entry(237),
        ld1_entry(238),
        ld1_entry(239),
        ld1_entry(240),
        ld1_entry(241),
        ld1_entry(242),
        ld1_entry(243),
        ld1_entry(244),
        ld1_entry(245),
        ld1_entry(246),
        ld1_entry(247),
        ld1_entry(248),
        ld1_entry(249),
        ld1_entry(250),
        ld1_entry(251),
        ld1_entry(252),
        ld1_entry(253),
        ld1_entry(254),
        ld1_entry(255),
        /* End of block  */
        DecodeEntry::end_of_block(),
        /* Lengths  */
        ld2_entry(3, 0),
        ld2_entry(4, 0),
        ld2_entry(5, 0),
        ld2_entry(6, 0),
        ld2_entry(7, 0),
        ld2_entry(8, 0),
        ld2_entry(9, 0),
        ld2_entry(10, 0),
        ld2_entry(11, 1),
        ld2_entry(13, 1),
        ld2_entry(15, 1),
        ld2_entry(17, 1),
        ld2_entry(19, 2),
        ld2_entry(23, 2),
        ld2_entry(27, 2),
        ld2_entry(31, 2),
        ld2_entry(35, 3),
        ld2_entry(43, 3),
        ld2_entry(51, 3),
        ld2_entry(59, 3),
        ld2_entry(67, 4),
        ld2_entry(83, 4),
        ld2_entry(99, 4),
        ld2_entry(115, 4),
        ld2_entry(131, 5),
        ld2_entry(163, 5),
        ld2_entry(195, 5),
        ld2_entry(227, 5),
        ld2_entry(258, 0),
        ld2_entry(258, 0),
        ld2_entry(258, 0),
    ]);

/* The decode result for each offset symbol.  This is the offset base and the
 * number of extra offset bits.  */

#[inline(always)]
const fn odr_entry(offset_base: u32, num_extra_bits: u8) -> DecodeEntry {
    DecodeEntry::new_offset(offset_base, num_extra_bits)
}

const OFFSET_DECODE_RESULTS: UncheckedArray<DecodeEntry, DEFLATE_NUM_OFFSET_SYMS> =
    UncheckedArray([
        odr_entry(1, 0),
        odr_entry(2, 0),
        odr_entry(3, 0),
        odr_entry(4, 0),
        odr_entry(5, 1),
        odr_entry(7, 1),
        odr_entry(9, 2),
        odr_entry(13, 2),
        odr_entry(17, 3),
        odr_entry(25, 3),
        odr_entry(33, 4),
        odr_entry(49, 4),
        odr_entry(65, 5),
        odr_entry(97, 5),
        odr_entry(129, 6),
        odr_entry(193, 6),
        odr_entry(257, 7),
        odr_entry(385, 7),
        odr_entry(513, 8),
        odr_entry(769, 8),
        odr_entry(1025, 9),
        odr_entry(1537, 9),
        odr_entry(2049, 10),
        odr_entry(3073, 10),
        odr_entry(4097, 11),
        odr_entry(6145, 11),
        odr_entry(8193, 12),
        odr_entry(12289, 12),
        odr_entry(16385, 13),
        odr_entry(24577, 13),
        odr_entry(32769, 14),
        odr_entry(49153, 14),
    ]);

#[inline(always)]
const fn bsr32(val: u32) -> u32 {
    (std::mem::size_of::<u32>() * 8) as u32 - 1 - val.leading_zeros()
}

/*
 * Build a table for fast decoding of symbols from a Huffman code.  As input,
 * this function takes the codeword length of each symbol which may be used in
 * the code.  As output, it produces a decode table for the canonical Huffman
 * code described by the codeword lengths.  The decode table is built with the
 * assumption that it will be indexed with "bit-reversed" codewords, where the
 * low-order bit is the first bit of the codeword.  This format is used for all
 * Huffman codes in DEFLATE.
 *
 * @decode_table
 *	The array in which the decode table will be generated.  This array must
 *	have sufficient length; see the definition of the ENOUGH numbers.
 * @lens
 *	An array which provides, for each symbol, the length of the
 *	corresponding codeword in bits, or 0 if the symbol is unused.  This may
 *	alias @decode_table, since nothing is written to @decode_table until all
 *	@lens have been consumed.  All codeword lengths are assumed to be <=
 *	@max_codeword_len but are otherwise considered untrusted.  If they do
 *	not form a valid Huffman code, then the decode table is not built and
 *	%false is returned.
 * @num_syms
 *	The number of symbols in the code, including all unused symbols.
 * @decode_results
 *	An array which provides, for each symbol, the actual value to store into
 *	the decode table.  This value will be directly produced as the result of
 *	decoding that symbol, thereby moving the indirection out of the decode
 *	loop and into the table initialization.
 * @table_bits
 *	The log base-2 of the number of main table entries to use.
 * @max_codeword_len
 *	The maximum allowed codeword length for this Huffman code.
 *	Must be <= DEFLATE_MAX_CODEWORD_LEN.
 * @sorted_syms
 *	A temporary array of length @num_syms.
 *
 * Returns %true if successful; %false if the codeword lengths do not form a
 * valid Huffman code.
 */
pub fn build_decode_table<const DECODE_TABLE_SIZE: usize, const DECODE_RESULTS_SIZE: usize>(
    decode_table: &mut UncheckedArray<DecodeEntry, DECODE_TABLE_SIZE>,
    lens: &UncheckedSlice<LenType>,
    num_syms: usize,
    decode_results: &UncheckedArray<DecodeEntry, DECODE_RESULTS_SIZE>,
    table_bits: usize,
    max_codeword_len: usize,
    mut sorted_syms: *mut u16,
) -> bool {
    let mut len_counts: UncheckedArray<u16, { DEFLATE_MAX_CODEWORD_LEN + 1 }> =
        UncheckedArray([0; DEFLATE_MAX_CODEWORD_LEN + 1]);
    let mut offsets: UncheckedArray<u16, { DEFLATE_MAX_CODEWORD_LEN + 1 }> =
        UncheckedArray([0; DEFLATE_MAX_CODEWORD_LEN + 1]);
    let mut count: u16; /* num codewords remaining with this length */
    let mut codespace_used: u32; /* codespace used out of '2^max_codeword_len' */
    let mut cur_table_end: usize; /* end index of current table */
    let mut subtable_prefix: usize; /* codeword prefix of current subtable */
    let mut subtable_start: usize; /* start index of current subtable */
    let mut subtable_bits: usize; /* log2 of current subtable length */

    /* Count how many codewords have each length, including 0. */
    for sym in 0..num_syms {
        len_counts[lens[sym] as usize] += 1;
    }

    /*
     * Sort the symbols primarily by increasing codeword length and
     * secondarily by increasing symbol value; or equivalently by their
     * codewords in lexicographic order, since a canonical code is assumed.
     *
     * For efficiency, also compute 'codespace_used' in the same pass over
     * 'len_counts[]' used to build 'offsets[]' for sorting.
     */

    /* Ensure that 'codespace_used' cannot overflow. */
    // const_assert!(sizeof(codespace_used) == 4);
    const_assert!(
        u32::MAX / (1u32 << (DEFLATE_MAX_CODEWORD_LEN - 1)) >= DEFLATE_MAX_NUM_SYMS as u32
    );

    // offsets[0] = 0;
    offsets[1] = len_counts[0] as u16;
    codespace_used = 0;
    for len in 1..max_codeword_len {
        offsets[len + 1] = offsets[len] + len_counts[len] as u16;
        codespace_used = (codespace_used << 1) + len_counts[len] as u32;
    }
    codespace_used = (codespace_used << 1) + len_counts[max_codeword_len] as u32;

    for sym in 0..num_syms {
        unsafe {
            *sorted_syms.add(offsets[lens[sym] as usize] as usize) = sym as u16;
        }
        offsets[lens[sym] as usize] += 1;
    }

    sorted_syms = unsafe { sorted_syms.add(offsets[0] as usize) }; /* Skip unused symbols */

    /* lens[] is done being used, so we can write to decode_table[] now. */

    /*
     * Check whether the lengths form a complete code (exactly fills the
     * codespace), an incomplete code (doesn't fill the codespace), or an
     * overfull code (overflows the codespace).  A codeword of length 'n'
     * uses proportion '1/(2^n)' of the codespace.  An overfull code is
     * nonsensical, so is considered invalid.  An incomplete code is
     * considered valid only in two specific cases; see below.
     */

    // if lens.len() == 19 {
    //     let mut checker = CodepointChecker::new(lens.try_into().unwrap());
    //     let first_len = lens[0];
    //     let codespace = checker.roll_lens(first_len, num_syms as u8);

    //     assert_eq!(codespace as u32, codespace_used);
    // }

    // println!("Precode cs: {}", codespace_used);

    /* overfull code? */
    if unlikely(codespace_used > (1u32 << max_codeword_len)) {
        return false;
    }

    /* incomplete code? */
    if unlikely(codespace_used < (1u32 << max_codeword_len)) {
        let sym = if codespace_used == 0 {
            /*
             * An empty code is allowed.  This can happen for the
             * offset code in DEFLATE, since a dynamic Huffman block
             * need not contain any matches.
             */
            0 // (arbitrary)
        } else {
            /*
             * Allow codes with a single used symbol, with codeword
             * length 1.  The DEFLATE RFC is unclear regarding this
             * case.  What zlib's decompressor does is permit this
             * for the litlen and offset codes and assume the
             * codeword is '0' rather than '1'.  We do the same
             * except we allow this for precodes too, since there's
             * no convincing reason to treat the codes differently.
             * We also assign both codewords '0' and '1' to the
             * symbol to avoid having to handle '1' specially.
             */
            if codespace_used != (1u32 << (max_codeword_len - 1)) || len_counts[1] != 1 {
                return false;
            }
            unsafe { *sorted_syms as usize }
        };

        let entry = decode_results[sym].make_decode_table_entry(1);

        /*
         * Note: the decode table still must be fully initialized, in
         * case the stream is malformed and contains bits from the part
         * of the codespace the incomplete code doesn't use.
         */
        for i in 0..(1usize << table_bits) {
            decode_table[i] = entry;
        }
        return true;
    }

    /*
     * The lengths form a complete code.  Now, enumerate the codewords in
     * lexicographic order and fill the decode table entries for each one.
     *
     * First, process all codewords with len <= table_bits.  Each one gets
     * '2^(table_bits-len)' direct entries in the table.
     *
     * Since DEFLATE uses bit-reversed codewords, these entries aren't
     * consecutive but rather are spaced '2^len' entries apart.  This makes
     * filling them naively somewhat awkward and inefficient, since strided
     * stores are less cache-friendly and preclude the use of word or
     * vector-at-a-time stores to fill multiple entries per instruction.
     *
     * To optimize this, we incrementally double the table size.  When
     * processing codewords with length 'len', the table is treated as
     * having only '2^len' entries, so each codeword uses just one entry.
     * Then, each time 'len' is incremented, the table size is doubled and
     * the first half is copied to the second half.  This significantly
     * improves performance over naively doing strided stores.
     *
     * Note that some entries copied for each table doubling may not have
     * been initialized yet, but it doesn't matter since they're guaranteed
     * to be initialized later (because the Huffman code is complete).
     */
    let mut codeword: usize = 0; /* current codeword, bit-reversed */
    let mut len = 1;
    while {
        count = len_counts[len];
        count == 0
    } {
        len += 1;
    }
    cur_table_end = 1 << len;
    while len <= table_bits {
        /* Process all 'count' codewords with length 'len' bits. */
        while count > 0 {
            /* Fill the first entry for the current codeword. */
            decode_table[codeword] = decode_results[unsafe { *sorted_syms as usize }]
                .make_decode_table_entry(len as u32);
            unsafe {
                sorted_syms = sorted_syms.add(1);
            }

            if codeword == cur_table_end - 1 {
                /* Last codeword (all 1's) */
                while len < table_bits {
                    unsafe {
                        std::ptr::copy_nonoverlapping(
                            decode_table.as_ptr(),
                            decode_table.as_mut_ptr().add(cur_table_end),
                            cur_table_end,
                        );
                    }
                    cur_table_end <<= 1;
                    len += 1;
                }
                return true;
            }
            /*
             * To advance to the lexicographically next codeword in
             * the canonical code, the codeword must be incremented,
             * then 0's must be appended to the codeword as needed
             * to match the next codeword's length.
             *
             * Since the codeword is bit-reversed, appending 0's is
             * a no-op.  However, incrementing it is nontrivial.  To
             * do so efficiently, use the 'bsr' instruction to find
             * the last (highest order) 0 bit in the codeword, set
             * it, and clear any later (higher order) 1 bits.  But
             * 'bsr' actually finds the highest order 1 bit, so to
             * use it first flip all bits in the codeword by XOR'ing
             * it with (1U << len) - 1 == cur_table_end - 1.
             */
            let bit = 1 << bsr32((codeword ^ (cur_table_end - 1)) as u32);
            codeword &= bit - 1;
            codeword |= bit;

            count -= 1;
        }

        /* Advance to the next codeword length. */
        loop {
            len += 1;
            if len <= table_bits {
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        decode_table.as_ptr(),
                        decode_table.as_mut_ptr().add(cur_table_end as usize),
                        cur_table_end,
                    );
                }
                cur_table_end <<= 1;
            }

            count = len_counts[len];
            if count != 0 {
                break;
            }
        }
    }

    /* Process codewords with len > table_bits.  These require subtables. */
    cur_table_end = 1 << table_bits;
    subtable_prefix = usize::MAX;
    subtable_start = 0;
    loop {
        /*
         * Start a new subtable if the first 'table_bits' bits of the
         * codeword don't match the prefix of the current subtable.
         */
        if (codeword & ((1 << table_bits) - 1)) != subtable_prefix {
            subtable_prefix = codeword & ((1 << table_bits) - 1);
            subtable_start = cur_table_end;
            /*
             * Calculate the subtable length.  If the codeword has
             * length 'table_bits + n', then the subtable needs
             * '2^n' entries.  But it may need more; if fewer than
             * '2^n' codewords of length 'table_bits + n' remain,
             * then the length will need to be incremented to bring
             * in longer codewords until the subtable can be
             * completely filled.  Note that because the Huffman
             * code is complete, it will always be possible to fill
             * the subtable eventually.
             */
            subtable_bits = len - table_bits;
            codespace_used = count as u32;
            while codespace_used < (1u32 << subtable_bits) {
                subtable_bits += 1;
                codespace_used =
                    (codespace_used << 1) + len_counts[table_bits + subtable_bits] as u32;
            }
            cur_table_end = subtable_start + (1 << subtable_bits);

            /*
             * Create the entry that points from the main table to
             * the subtable.  This entry contains the index of the
             * start of the subtable and the number of bits with
             * which the subtable is indexed (the log base 2 of the
             * number of entries it contains).
             */

            decode_table[subtable_prefix] = DecodeEntry::new_subtable_pointer(
                subtable_start as u32,
                subtable_bits as u8,
                table_bits as u8,
            );

            // decode_table[subtable_prefix] = HUFFDEC_SUBTABLE_POINTER
            //     | huffdec_result_entry(subtable_start as u32)
            //     | subtable_bits as u32;
        }

        /* Fill the subtable entries for the current codeword. */
        let entry = decode_results[unsafe { *sorted_syms as usize }]
            .make_decode_table_entry((len - table_bits) as u32);

        sorted_syms = unsafe { sorted_syms.add(1) };

        let mut i = subtable_start + (codeword >> table_bits);
        let stride = 1 << (len - table_bits);
        loop {
            decode_table[i] = entry;
            i += stride;

            if i >= cur_table_end {
                break;
            }
        }

        /* Advance to the next codeword. */
        if codeword == (1 << len) - 1
        /* last codeword (all 1's)? */
        {
            return true;
        }
        let bit = 1 << bsr32((codeword ^ ((1 << len) - 1)) as u32);
        codeword &= bit - 1;
        codeword |= bit;
        count -= 1;
        while count == 0 {
            len += 1;
            count = len_counts[len];
        }
    }
}

/* Build the decode table for the precode.  */
pub fn build_precode_decode_table(d: &mut LibdeflateDecompressor) -> bool {
    /* When you change TABLEBITS, you must change ENOUGH, and vice versa! */
    const_assert!(PRECODE_TABLEBITS == 7 && PRECODE_ENOUGH == 128);

    return build_decode_table(
        &mut d.l.precode_decode_table,
        d.precode_lens.as_ref(),
        DEFLATE_NUM_PRECODE_SYMS,
        &PRECODE_DECODE_RESULTS,
        PRECODE_TABLEBITS,
        DEFLATE_MAX_PRE_CODEWORD_LEN,
        d.sorted_syms.as_mut_ptr(),
    );
}

/* Build the decode table for the literal/length code.  */
pub fn build_litlen_decode_table(
    d: &mut LibdeflateDecompressor,
    num_litlen_syms: usize,
    _num_offset_syms: usize,
) -> bool {
    /* When you change TABLEBITS, you must change ENOUGH, and vice versa! */
    const_assert!(LITLEN_TABLEBITS == 10 && LITLEN_ENOUGH == 1334);

    return build_decode_table(
        &mut d.litlen_decode_table,
        d.l.lens.as_ref(),
        num_litlen_syms,
        &LITLEN_DECODE_RESULTS,
        LITLEN_TABLEBITS,
        DEFLATE_MAX_LITLEN_CODEWORD_LEN,
        d.sorted_syms.as_mut_ptr(),
    );
}

/* Build the decode table for the offset code.  */
pub fn build_offset_decode_table(
    d: &mut LibdeflateDecompressor,
    num_litlen_syms: usize,
    num_offset_syms: usize,
) -> bool {
    /* When you change TABLEBITS, you must change ENOUGH, and vice versa! */
    const_assert!(OFFSET_TABLEBITS == 8 && OFFSET_ENOUGH == 402);

    return build_decode_table(
        &mut d.offset_decode_table,
        &d.l.lens[num_litlen_syms..d.l.lens.len()],
        num_offset_syms,
        &OFFSET_DECODE_RESULTS,
        OFFSET_TABLEBITS,
        DEFLATE_MAX_OFFSET_CODEWORD_LEN,
        d.sorted_syms.as_mut_ptr(),
    );
}

/*****************************************************************************
 *                         Main decompression routine
 *****************************************************************************/

// typedef enum libdeflate_result (*decompress_func_t)
// 	(struct libdeflate_decompressor * restrict d,
// 	 const void * restrict in, size_t in_nbytes,
// 	 void * restrict out, size_t out_nbytes_avail,
// 	 size_t *actual_in_nbytes_ret, size_t *actual_out_nbytes_ret,
// 	 flush_buffer_func *flush_func,
// 	 void *flush_data);

// #ifdef DISPATCH
// static enum libdeflate_result
// dispatch(struct libdeflate_decompressor * restrict d,
// 	 const void * restrict in, size_t in_nbytes,
// 	 void * restrict out, size_t out_nbytes_avail,
// 	 size_t *actual_in_nbytes_ret, size_t *actual_out_nbytes_ret,
// 	 flush_buffer_func *flush_func,
// 	 void *flush_data);
//
// static volatile decompress_func_t decompress_impl = dispatch;
//
// /* Choose the fastest implementation at runtime */
// static enum libdeflate_result
// dispatch(struct libdeflate_decompressor * restrict d,
// 	 const void * restrict in, size_t in_nbytes,
// 	 void * restrict out, size_t out_nbytes_avail,
// 	 size_t *actual_in_nbytes_ret, size_t *actual_out_nbytes_ret,
// 	 flush_buffer_func *flush_func,
// 	 void *flush_data)
// {
// 	decompress_func_t f = arch_select_decompress_func();
//
// 	if (f == NULL)
// 		f = DEFAULT_IMPL;
//
// 	decompress_impl = f;
// 	return (*f)(d, in, in_nbytes, out, out_nbytes_avail,
// 		    actual_in_nbytes_ret, actual_out_nbytes_ret, flush_func, flush_data);
// }
// #else
// #  define decompress_impl DEFAULT_IMPL /* only one implementation, use it */
// #endif

/*
 * This is the main DEFLATE decompression routine.  See libdeflate.h for the
 * documentation.
 *
 * Note that the real code is in decompress_template.h.  The part here just
 * handles calling the appropriate implementation depending on the CPU features
 * at runtime.
 */
pub fn libdeflate_deflate_decompress<I: DeflateInput, O: DeflateOutput>(
    d: &mut LibdeflateDecompressor,
    in_stream: &mut I,
    out_stream: &mut O,
) -> Result<(), LibdeflateError> {
    decompress_direct(d, in_stream, out_stream)
    // decompress_with_instr(d, in_stream, out_stream)
}

#[cfg(test)]
mod tests {
    use crate::block_finder::CodepointChecker;

    type LenType = u8;
    const DEFLATE_NUM_PRECODE_SYMS: usize = 19;

    #[test]
    fn test_decode_validity_fast() {
        // let start = std::time::Instant::now();
        // let mut value = start.elapsed().as_nanos() as u64;

        // const COUNT: usize = 1000000000;

        // let mut checker1 =
        //     CodepointChecker::new([1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 6, 5, 4, 3, 2, 1, 2, 3, 4]);

        // CodepointChecker::debug_values("se0", checker1.exp_lens[0], 10);
        // CodepointChecker::debug_values("se1", checker1.exp_lens[1], 9);

        // let mut checker2 = checker1.clone();
        // let mut checker3 = checker1.clone();

        // let mut results = 0;
        // for i in 0..COUNT {
        //     results += checker1.roll_lens(((i as u8) + 7) % 8, i as u8 % 16 + 1); // ((i as u8) + 0) % 16);
        //     results += checker2.roll_lens(((i as u8) + 1) % 8, ((i as u8) + 1) % 16);
        //     results += checker3.roll_lens(((i as u8) + 2) % 8, ((i as u8) + 2) % 16);
        // }

        // let elapsed = start.elapsed().as_nanos();
        // let elapsed_per_byte = elapsed as f64 / COUNT as f64 / 3.0 * 8.0;
        // println!(
        //     "Elapsed time: {:.4} nanoseconds per byte: SPEED: {:.2} MB/s value: {}",
        //     elapsed_per_byte,
        //     1_000_000_000.0 / elapsed_per_byte / 1024.0 / 1024.0,
        //     results
        // );
    }
}
