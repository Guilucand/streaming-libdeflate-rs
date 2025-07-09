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

pub mod fast_decode_entry;

use crate::bitstream::BitStream;
/*
 * The type for the bitbuffer variable ('bitbuf' described above).  For best
 * performance, this should have size equal to a machine word.
 *
 * 64-bit platforms have a significant advantage: they get a bigger bitbuffer
 * which they have to fill less often.
 */
use crate::decompress_deflate::{
    LenType, FAST_TABLEBITS, FAST_TABLESIZE, LITLEN_SUBTABLESIZE, LITLEN_TABLEBITS,
    LITLEN_TABLESIZE, OFFSET_SUBTABLESIZE, OFFSET_TABLEBITS, OFFSET_TABLESIZE,
    PRECODE_SUBTABLESIZE, PRECODE_TABLEBITS, PRECODE_TABLESIZE,
};
use crate::decompress_utils::fast_decode_entry::FastDecodeEntry;
use crate::deflate_constants::*;
use crate::unchecked::{UncheckedArray, UncheckedSlice};
use crate::{DeflateInput, LibdeflateDecodeTables};
use nightly_quirks::branch_pred::unlikely;

#[derive(Default, Copy, Clone)]
pub struct DecodeEntryState {
    pub entry: FastDecodeEntry,
    pub offset: usize,
}

pub struct DecompressTempData<'a, I: DeflateInput> {
    pub input_bitstream: BitStream<'a, I>,
    pub block_type: u32,
    pub is_final_block: bool,
    pub fast_entry: FastDecodeEntry,
}

#[inline(always)]
pub unsafe fn copy_word_unaligned(src: *const u64, dst: *mut u64) {
    dst.write_unaligned(src.read_unaligned())
}

#[inline(always)]
pub unsafe fn copy_dword_unaligned(src: *const u64, dst: *mut u64) {
    (dst as *mut u128).write_unaligned((src as *const u128).read_unaligned())
}

/*****************************************************************************
 *                              Huffman decoding                             *
 *****************************************************************************/

// /* Shift a decode result into its position in the decode table entry.  */
// #[inline(always)]
// const fn huffdec_result_entry(result: u32) -> u32 {
//     result << HUFFDEC_RESULT_SHIFT
// }

/* The decode result for each precode symbol.  There is no special optimization
 * for the precode; the decode result is simply the symbol value.  */
#[inline(always)]
const fn hr_entry(presym: u8) -> FastDecodeEntry {
    FastDecodeEntry::new_presym(presym)
}

const PRECODE_DECODE_RESULTS: UncheckedArray<FastDecodeEntry, DEFLATE_NUM_PRECODE_SYMS> =
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
const fn ld1_entry(literal: u8) -> FastDecodeEntry {
    FastDecodeEntry::new_from_literal(literal)
}

#[inline(always)]
const fn ld2_entry(length_base: u16, num_extra_bits: u8) -> FastDecodeEntry {
    FastDecodeEntry::new_from_len_and_extra_bits(length_base, num_extra_bits)
}

const LITLEN_DECODE_RESULTS: UncheckedArray<FastDecodeEntry, DEFLATE_NUM_LITLEN_SYMS> =
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
        FastDecodeEntry::new_end_of_block(),
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
const fn odr_entry(offset_base: u16, num_extra_bits: u8) -> FastDecodeEntry {
    FastDecodeEntry::new_from_offset_and_extra_bits(offset_base, num_extra_bits)
}

const OFFSET_DECODE_RESULTS: UncheckedArray<FastDecodeEntry, DEFLATE_NUM_OFFSET_SYMS> =
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

const fn advance_codeword(codeword: usize, mask: usize) -> usize {
    #[inline(always)]
    const fn bsr32(val: u32) -> u32 {
        ((std::mem::size_of::<u32>() * 8) as u32 - 1).wrapping_sub(val.leading_zeros())
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

    let bit = 1usize.wrapping_shl(bsr32((codeword ^ mask) as u32));
    (codeword & (bit - 1)) | bit
}

#[derive(Clone, Copy)]
pub struct TempFastDecodeBuild {
    codeword: usize,
    new_entry: FastDecodeEntry,
}

/*
 * Builds a fast table decoder, that is able to fully match 12 bits in the format <literals>{0..2}(<len><extra_len><offset>)?
 * it can match up to 2 literals and a small lookback copy, composed of a length (with its extra bits) + the offset bits
 * the next bits possibly encoding the offset extra bits are extracted afterwards if needed, and are not part of the table.
 * If it cannot decode an entry, it defaults to a slow multi-table lookup (without the extra_len embeddings) by borrowing from the litlen table subtables.
 * After that the offset is also decoded using the slow way (possibly with subtables)
*/
pub fn build_fast_decode_table(
    litlen_decode_table: &UncheckedArray<FastDecodeEntry, LITLEN_TABLESIZE>,
    offset_decode_table: &UncheckedArray<FastDecodeEntry, OFFSET_TABLESIZE>,
    fast_table: &mut UncheckedArray<FastDecodeEntry, FAST_TABLESIZE>,
    temp_indices_litlen: &mut Vec<TempFastDecodeBuild>,
) {
    const_assert!(FAST_TABLEBITS == LITLEN_TABLEBITS);

    // Reset the temp indices
    temp_indices_litlen.clear();

    // Init the table with the default litlen values
    for (codeword, (fast_entry, litlen)) in fast_table
        .0
        .iter_mut()
        .zip(litlen_decode_table.0.iter())
        .enumerate()
    {
        *fast_entry = FastDecodeEntry::new_from_litlen_entry(*litlen, codeword);
        let used_bits = fast_entry.get_consumed_bits() as usize;
        let shifted_codeword = codeword >> used_bits;

        if !litlen.is_subtable_pointer() {
            if litlen.has_literals() {
                temp_indices_litlen.push(TempFastDecodeBuild {
                    codeword,
                    new_entry: litlen_decode_table[shifted_codeword],
                });
            } else if fast_entry.get_flags() == FastDecodeEntry::EXC_LEN_FULLSIZE {
                let offset_entry = offset_decode_table[shifted_codeword % OFFSET_TABLESIZE];
                let offset_bits = offset_entry.get_consumed_bits() as usize;
                if used_bits + offset_bits <= FAST_TABLEBITS && !offset_entry.is_subtable_pointer()
                {
                    fast_entry.add_offset_entry(offset_entry);
                }
            }
        }
    }

    // Process litlen extensions
    while temp_indices_litlen.len() > 0 {
        // Filter temp indices
        temp_indices_litlen.retain_mut(|index| {
            let fast_entry = &mut fast_table[index.codeword];

            let litlen_bits = index.new_entry.get_consumed_bits() as usize
                + index.new_entry.get_exceptional_length_bits() as usize;
            let used_bits = fast_entry.get_consumed_bits() as usize;

            if used_bits + litlen_bits <= FAST_TABLEBITS {
                let shifted_codeword = index.codeword >> used_bits;

                if index.new_entry.has_literals() {
                    // Add a new literal entry
                    if !fast_entry.maybe_add_litlen_entry(index.new_entry, shifted_codeword) {
                        // No more entries can be added, exit
                        return false;
                    }
                    // Check if we can add another literal
                    if (fast_entry.get_consumed_bits() as usize) < FAST_TABLEBITS {
                        index.new_entry =
                            litlen_decode_table[shifted_codeword % (1 << OFFSET_TABLEBITS)];
                        return true;
                    }
                } else if index.new_entry.get_flags() == FastDecodeEntry::EXC_LEN_EXTRABIT {
                    // We had a length entry, try to match the offset
                    let shifted_codeword = index.codeword >> (used_bits + litlen_bits);
                    let offset_entry = offset_decode_table[shifted_codeword % OFFSET_TABLESIZE];
                    let offset_bits = offset_entry.get_consumed_bits() as usize;

                    if used_bits + litlen_bits + offset_bits <= FAST_TABLEBITS
                        && !offset_entry.is_subtable_pointer()
                    {
                        // Only if we can match a full length + offset add it to the entry
                        if !fast_entry.maybe_add_litlen_entry(index.new_entry, shifted_codeword) {
                            return false;
                        }
                        fast_entry.add_offset_entry(offset_entry);
                    }
                }
            }
            false
        });
    }
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
 * Returns %true if successful; %false if the codeword lengths do not form a
 * valid Huffman code.
 */
pub fn build_decode_table<
    const DECODE_TABLE_SIZE: usize,
    const DECODE_SUBTABLE_SIZE: usize,
    const DECODE_RESULTS_SIZE: usize,
    const BUILD_SYM_MAP: bool,
>(
    decode_table: &mut UncheckedArray<FastDecodeEntry, DECODE_TABLE_SIZE>,
    decode_subtable: &mut UncheckedArray<FastDecodeEntry, DECODE_SUBTABLE_SIZE>,
    lens: &UncheckedSlice<LenType>,
    num_syms: usize,
    decode_results: &UncheckedArray<FastDecodeEntry, DECODE_RESULTS_SIZE>,
    table_bits: usize,
    max_codeword_len: usize,
    sorted_syms: &mut UncheckedArray<u16, DEFLATE_MAX_NUM_SYMS>,
) -> bool {
    // Count how many codewords have each length, including 0.
    let mut len_counts: UncheckedArray<u16, { DEFLATE_MAX_CODEWORD_LEN + 1 }> =
        UncheckedArray([0; DEFLATE_MAX_CODEWORD_LEN + 1]);

    let mut max_len = 0;
    for sym in 0..num_syms {
        len_counts[lens[sym] as usize] += 1;
        max_len = max_len.max(lens[sym] as usize);
    }

    /*
     * Sort the symbols primarily by increasing codeword length and
     * secondarily by increasing symbol value; or equivalently by their
     * codewords in lexicographic order, since a canonical code is assumed.
     *
     * For efficiency, also compute 'codespace_used' in the same pass over
     * 'len_counts[]' used to build 'offsets[]' for sorting.
     */

    let mut offsets: UncheckedArray<u16, { DEFLATE_MAX_CODEWORD_LEN + 1 }> =
        UncheckedArray([0; DEFLATE_MAX_CODEWORD_LEN + 1]);

    //  codespace used out of '2^max_codeword_len'
    let mut codespace_used: u32 = 0;

    // Second step of counting sort, compute offsets + compute used codepsace
    {
        offsets[1] = len_counts[0] as u16;
        for len in 1..max_codeword_len {
            offsets[len + 1] = offsets[len] + len_counts[len] as u16;
            codespace_used = (codespace_used << 1) + len_counts[len] as u32;
        }
        codespace_used = (codespace_used << 1) + len_counts[max_codeword_len] as u32;
    }

    // Third step of counting sort, move elements in their sorted positions
    for sym in 0..num_syms {
        sorted_syms[offsets[lens[sym] as usize] as usize] = sym as u16;
        offsets[lens[sym] as usize] += 1;
    }

    // Skip unused symbols
    let sorted_syms = &sorted_syms[offsets[0] as usize..num_syms];

    /*
     * Check whether the lengths form a complete code (exactly fills the
     * codespace), an incomplete code (doesn't fill the codespace), or an
     * overfull code (overflows the codespace).  A codeword of length 'n'
     * uses proportion '1/(2^n)' of the codespace.  An overfull code is
     * nonsensical, so is considered invalid.  An incomplete code is
     * considered valid only in two specific cases; see below.
     */

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
            sorted_syms[0] as usize
        };

        let entry = decode_results[sym].make_decode_table_entry(1);

        /*
         * Note: the decode table still must be fully initialized, in
         * case the stream is malformed and contains bits from the part
         * of the codespace the incomplete code doesn't use.
         */
        decode_table.0.fill(entry);
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
    let mut start_len = 1;

    // Skip all the lengths without elements
    while len_counts[start_len] == 0 {
        start_len += 1;
    }

    // end index of current table
    let mut cur_table_end = 1 << (start_len - 1);

    let mut sym_index = 0;

    for len in start_len..(table_bits + 1) {
        // Copy the previous elements into the next table part
        if len != start_len {
            decode_table.0.copy_within(0..cur_table_end, cur_table_end);
        }
        cur_table_end *= 2;

        /* Process all 'len_counts[len]' codewords with length 'len' bits. */
        for _ in 0..len_counts[len] {
            /* Fill the first entry for the current codeword. */
            decode_table[codeword] =
                decode_results[sorted_syms[sym_index] as usize].make_decode_table_entry(len as u8);
            sym_index += 1;

            codeword = advance_codeword(codeword, cur_table_end - 1);
        }
    }

    // No need for extra tables
    if max_len <= table_bits {
        return true;
    }

    /* Process codewords with len > table_bits.  These require subtables. */

    // codeword prefix of current subtable
    let mut subtable_prefix = usize::MAX;
    // start index of current subtable
    let mut subtable_start = 0;
    let mut subtable_end = 0;

    for len in (table_bits + 1)..(max_len + 1) {
        for i in 0..len_counts[len] {
            /*
             * Start a new subtable if the first 'table_bits' bits of the
             * codeword don't match the prefix of the current subtable.
             */

            if (codeword & ((1 << table_bits) - 1)) != subtable_prefix {
                subtable_prefix = codeword & ((1 << table_bits) - 1);
                subtable_start = subtable_end;
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

                // log2 of current subtable length
                let mut subtable_bits = len - table_bits;
                let mut codespace_used = (len_counts[len] - i) as u32;
                while codespace_used < (1u32 << subtable_bits) {
                    subtable_bits += 1;
                    codespace_used =
                        (codespace_used << 1) + len_counts[table_bits + subtable_bits] as u32;
                }

                // Add the current subtable size
                subtable_end += 1 << subtable_bits;

                /*
                 * Create the entry that points from the main table to
                 * the subtable.  This entry contains the index of the
                 * start of the subtable and the number of bits with
                 * which the subtable is indexed (the log base 2 of the
                 * number of entries it contains).
                 */

                decode_table[subtable_prefix] = FastDecodeEntry::new_subtable_pointer(
                    subtable_start as u16,
                    subtable_bits as u8,
                );
            }

            /* Fill the subtable entries for the current codeword. */
            let entry = decode_results[sorted_syms[sym_index] as usize]
                .make_decode_table_entry((len - table_bits) as u8);
            sym_index += 1;

            let mut i = subtable_start + (codeword >> table_bits);
            let stride = 1 << (len - table_bits);
            while i < subtable_end {
                decode_subtable[i] = entry;
                i += stride;
            }

            /* Advance to the next codeword. */
            codeword = advance_codeword(codeword, (1 << len) - 1);
        }
    }

    true
}

/* Build the decode table for the precode.  */
pub fn build_precode_decode_table(tables: &mut LibdeflateDecodeTables) -> bool {
    return build_decode_table::<
        PRECODE_TABLESIZE,
        PRECODE_SUBTABLESIZE,
        DEFLATE_NUM_PRECODE_SYMS,
        false,
    >(
        &mut tables.huffman_decode.precode_decode_table,
        &mut UncheckedArray([]),
        tables.huffman_decode.precode_lens.as_ref(),
        DEFLATE_NUM_PRECODE_SYMS,
        &PRECODE_DECODE_RESULTS,
        PRECODE_TABLEBITS,
        DEFLATE_MAX_PRE_CODEWORD_LEN,
        &mut tables.sorted_syms,
    );
}

/* Build the decode table for the literal/length code.  */
pub fn build_litlen_decode_table(
    tables: &mut LibdeflateDecodeTables,
    num_litlen_syms: usize,
    _num_offset_syms: usize,
) -> bool {
    // TODO: Fix the new impl
    return build_decode_table::<
        LITLEN_TABLESIZE,
        LITLEN_SUBTABLESIZE,
        DEFLATE_NUM_LITLEN_SYMS,
        false,
    >(
        &mut tables.litlen_decode_table,
        &mut tables.litlen_decode_subtable,
        tables.huffman_decode.lens.as_ref(),
        num_litlen_syms,
        &LITLEN_DECODE_RESULTS,
        LITLEN_TABLEBITS,
        DEFLATE_MAX_LITLEN_CODEWORD_LEN,
        &mut tables.sorted_syms,
    );
}

/* Build the decode table for the offset code.  */
pub fn build_offset_decode_table(
    d: &mut LibdeflateDecodeTables,
    num_litlen_syms: usize,
    num_offset_syms: usize,
) -> bool {
    return build_decode_table::<
        OFFSET_TABLESIZE,
        OFFSET_SUBTABLESIZE,
        DEFLATE_NUM_OFFSET_SYMS,
        false,
    >(
        &mut d.offset_decode_table,
        &mut d.offset_decode_subtable,
        &d.huffman_decode.lens[num_litlen_syms..d.huffman_decode.lens.len()],
        num_offset_syms,
        &OFFSET_DECODE_RESULTS,
        OFFSET_TABLEBITS,
        DEFLATE_MAX_OFFSET_CODEWORD_LEN,
        &mut d.sorted_syms,
    );
}

#[cfg(test)]
mod tests {
    // use crate::block_finder::CodepointChecker;

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
