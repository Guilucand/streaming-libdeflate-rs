/*
 * decompress_template.h
 *
 * Copyright 2016 Eric Biggers
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Each TABLEBITS number is the base-2 logarithm of the number of entries in the
 * main portion of the corresponding decode table.  Each number should be large
 * enough to ensure that for typical data, the vast majority of symbols can be
 * decoded by a direct lookup of the next TABLEBITS bits of compressed data.
 * However, this must be balanced against the fact that a larger table requires
 * more memory and requires more time to fill.
 *
 * Note: you cannot change a TABLEBITS number without also changing the
 * corresponding ENOUGH number!
 */

use crate::bitstream::can_ensure;
use crate::bitstream::BitStream;
use crate::bitstream::MAX_ENSURE;
use crate::decode_blocks::decode_huffman_block;
use crate::decompress_utils::decode_entry::DecodeEntry;
use crate::decompress_utils::fast_decode_entry::FastDecodeEntry;
use crate::decompress_utils::*;
use crate::deflate_constants::*;
use crate::streams::deflate_chunked_buffer_input::DeflateChunkedBufferInput;
use crate::unchecked::UncheckedArray;
use crate::{DeflateInput, DeflateOutput, LibdeflateDecodeTables, LibdeflateError};
use nightly_quirks::branch_pred::likely;
use nightly_quirks::branch_pred::unlikely;

pub const PRECODE_TABLEBITS: usize = 7;
pub const LITLEN_TABLEBITS: usize = 10;
// pub const LITLEN_TABLEBITS: usize = 12;
pub const OFFSET_TABLEBITS: usize = 8;
pub const FAST_TABLEBITS: usize = 10;

pub struct OutStreamResult {
    pub written: usize,
    pub crc32: u32,
}

/*
 * Each ENOUGH number is the maximum number of decode table entries that may be
 * required for the corresponding Huffman code, including the main table and all
 * subtables.  Each number depends on three parameters:
 *
 *	(1) the maximum number of symbols in the code (DEFLATE_NUM_*_SYMS)
 *	(2) the number of main table bits (the TABLEBITS numbers defined above)
 *	(3) the maximum allowed codeword length (DEFLATE_MAX_*_CODEWORD_LEN)
 *
 * The ENOUGH numbers were computed using the utility program 'enough' from
 * zlib.  This program enumerates all possible relevant Huffman codes to find
 * the worst-case usage of decode table entries.
 */
const PRECODE_ENOUGH: usize = 128; /* enough 19 7 7	*/
const LITLEN_ENOUGH: usize = 1334; /* enough 288 10 15	*/
// pub const LITLEN_ENOUGH: usize = 4382; /* enough 288 12 15	*/
const OFFSET_ENOUGH: usize = 402; /* enough 32 8 15	*/

pub const PRECODE_TABLESIZE: usize = 1 << PRECODE_TABLEBITS;
pub const LITLEN_TABLESIZE: usize = 1 << LITLEN_TABLEBITS;
pub const OFFSET_TABLESIZE: usize = 1 << OFFSET_TABLEBITS;

pub const PRECODE_SUBTABLESIZE: usize = PRECODE_ENOUGH - PRECODE_TABLESIZE;
pub const LITLEN_SUBTABLESIZE: usize = LITLEN_ENOUGH - LITLEN_TABLESIZE;
pub const OFFSET_SUBTABLESIZE: usize = OFFSET_ENOUGH - OFFSET_TABLESIZE;

pub const FAST_TABLESIZE: usize = 1 << FAST_TABLEBITS; /* enough 32 8 15	*/

/* When you change TABLEBITS, you must change ENOUGH, and vice versa! */
const_assert!(PRECODE_TABLEBITS == 7 && PRECODE_ENOUGH == 128);

/* When you change TABLEBITS, you must change ENOUGH, and vice versa! */
const_assert!(LITLEN_TABLEBITS == 10 && LITLEN_ENOUGH == 1334);
// const_assert!(LITLEN_TABLEBITS == 12 && LITLEN_ENOUGH == 4382);

/* When you change TABLEBITS, you must change ENOUGH, and vice versa! */
const_assert!(OFFSET_TABLEBITS == 8 && OFFSET_ENOUGH == 402);

/*
 * Type for codeword lengths.
 */
pub type LenType = u8;

#[macro_export]
macro_rules! safety_check {
    ($cond:expr) => {
        if !$cond {
            return Err(LibdeflateError::BadData);
        }
    };
}

/*
 * The arrays aren't all needed at the same time.  'precode_lens' and
 * 'precode_decode_table' are unneeded after 'lens' has been filled.
 * Furthermore, 'lens' need not be retained after building the litlen
 * and offset decode tables.  In fact, 'lens' can be in union with
 * 'litlen_decode_table' provided that 'offset_decode_table' is separate
 * and is built first.
 */

#[derive(Clone)]
pub(crate) struct HuffmanDecodeStruct {
    pub(crate) lens: UncheckedArray<
        LenType,
        { DEFLATE_NUM_LITLEN_SYMS + DEFLATE_NUM_OFFSET_SYMS + DEFLATE_MAX_LENS_OVERRUN },
    >,
    pub(crate) precode_lens: UncheckedArray<LenType, DEFLATE_NUM_PRECODE_SYMS>,
    pub(crate) precode_decode_table: UncheckedArray<DecodeEntry, PRECODE_ENOUGH>,
    pub(crate) fast_temp_litlen: Vec<TempFastDecodeBuild>,
}

/*
 * This is the actual DEFLATE decompression routine, lifted out of
 * deflate_decompress.c so that it can be compiled multiple times with different
 * target instruction sets.
 */

// #[inline(never)]
// fn process_instructions<O: DeflateOutput>(
//     instructions: &mut Vec<u16>,
//     output_stream: &mut O,
// ) -> Result<(), LibdeflateError> {
//     // return Ok(());
//     let mut instr_idx = 0;
//     while instr_idx < instructions.len() {
//         let instruction = instructions[instr_idx];

//         if instruction & 0xFF00 == 0xFF00 {
//             // Literal
//             let literal = (instruction & 0x00FF) as u8;
//             safety_check!(output_stream.write(&[literal]));
//         } else {
//             // Match
//             let length = instruction as usize;
//             instr_idx += 1; // Move to the next instruction for offset
//             let offset = instructions[instr_idx] as usize;

//             todo!();
//             safety_check!(output_stream.copy_forward(offset, length));
//         }

//         instr_idx += 1;
//     }

//     instructions.clear();
//     Ok(())
// }

#[inline(always)]
fn process_entry(state: DecodeEntryState, output_stream: &mut impl DeflateOutput) {
    unsafe {
        let mut out_ptr = output_stream.get_output_ptr();
        (out_ptr as *mut u16).write_unaligned(state.entry.get_literals());
        out_ptr = out_ptr.add(state.entry.get_literals_count() as usize);

        let mut src = out_ptr.sub(state.offset) as *const u64;
        let mut dst = out_ptr as *mut u64;

        let word = std::ptr::read_unaligned(src);
        std::ptr::write_unaligned(dst, word);

        let length = state.entry.get_len_value() as usize;

        // Overlapping words
        if unlikely(state.offset > 0 && state.offset < 8) {
            let mut remaining = length as isize;
            let mut src = src as *const u8;
            let mut dst = dst as *mut u8;
            loop {
                for i in 0..8 {
                    *dst.add(i) = *src.add(i);
                }
                remaining -= 8;
                if remaining <= 0 {
                    break;
                }
                src = src.add(8);
                dst = dst.add(8);
            }
        } else if unlikely(length > 8) {
            let mut remaining = length as isize;
            loop {
                src = src.add(1);
                dst = dst.add(1);
                remaining -= 8;

                let word = src.read_unaligned();
                dst.write_unaligned(word);

                if remaining <= 0 {
                    break;
                }
            }
        }

        output_stream.set_output_ptr(out_ptr.add(length));
    }
}

#[inline(always)]
fn decode_block_instruction<I: DeflateInput, O: DeflateOutput, const FIRST_ENTRY: bool>(
    tables: &LibdeflateDecodeTables,
    tmp_data: &mut DecompressTempData<I>,
    output_stream: &mut O,
) -> Result<bool, LibdeflateError> {
    tmp_data.input_bitstream.force_ensure_bits();

    /* Decode a fast symbol.  */
    let mut fast_entry =
        tables.fast_decode_table[tmp_data.input_bitstream.bits(FAST_TABLEBITS) as usize];

    if !FIRST_ENTRY {
        process_entry(tmp_data.last_state, output_stream);
    }

    if likely(fast_entry.get_flags() == 0) {
        let offset_bits = fast_entry.get_offset_bits();
        let consumed_bits = fast_entry.get_consumed_bits();

        tmp_data.last_state = DecodeEntryState {
            entry: fast_entry,
            offset: fast_entry.get_offset_value() as usize
                + tmp_data
                    .input_bitstream
                    .bits_with_offset(consumed_bits, offset_bits) as usize,
        };

        let tot_bits = offset_bits + consumed_bits;
        tmp_data.input_bitstream.remove_bits(tot_bits as usize);

        fast_entry =
            tables.fast_decode_table[tmp_data.input_bitstream.bits(FAST_TABLEBITS) as usize];

        if !FIRST_ENTRY {
            process_entry(tmp_data.last_state, output_stream);
        } else {
            return Ok(true);
        }

        if likely(fast_entry.get_flags() == 0) {
            let offset_bits = fast_entry.get_offset_bits();
            let consumed_bits = fast_entry.get_consumed_bits();

            tmp_data.last_state = DecodeEntryState {
                entry: fast_entry,
                offset: fast_entry.get_offset_value() as usize
                    + tmp_data
                        .input_bitstream
                        .bits_with_offset(consumed_bits, offset_bits)
                        as usize,
            };

            let tot_bits = offset_bits + consumed_bits;
            tmp_data.input_bitstream.remove_bits(tot_bits as usize);
            return Ok(true);
        }
    }

    #[cold]
    #[inline]
    fn cold_path() {}

    match fast_entry.get_flags() {
        FastDecodeEntry::EXC_LEN_FULLSIZE | FastDecodeEntry::EXC_LEN_EXTRABIT => {
            tmp_data
                .input_bitstream
                .remove_bits(fast_entry.get_consumed_bits() as usize);

            let extra_len = if unlikely(fast_entry.get_flags() == FastDecodeEntry::EXC_LEN_EXTRABIT)
            {
                tmp_data
                    .input_bitstream
                    .pop_bits(fast_entry.get_exceptional_length_bits() as usize)
            } else {
                0
            };

            let mut offset_entry = tables.offset_decode_table
                [tmp_data.input_bitstream.bits(OFFSET_TABLEBITS) as usize];

            if unlikely(offset_entry.is_subtable_pointer()) {
                tmp_data.input_bitstream.remove_bits(OFFSET_TABLEBITS);
                offset_entry = tables.offset_decode_subtable[(offset_entry.get_result()
                    + tmp_data
                        .input_bitstream
                        .bits(offset_entry.get_subtable_length() as usize))
                    as usize];
            }

            let offset_consumed = offset_entry.get_maintable_length();
            let offset_extrabit = offset_entry.get_subtable_length();

            fast_entry.inc_len_value(extra_len as u16);
            tmp_data.last_state = DecodeEntryState {
                entry: fast_entry,
                offset: offset_entry.get_result() as usize
                    + tmp_data
                        .input_bitstream
                        .bits_with_offset(offset_consumed, offset_extrabit)
                        as usize,
            };

            tmp_data
                .input_bitstream
                .remove_bits(offset_consumed as usize + offset_extrabit as usize);

            return Ok(true);
        }
        FastDecodeEntry::EXC_END_OF_BLOCK => {
            cold_path();
            tmp_data
                .input_bitstream
                .remove_bits(fast_entry.get_consumed_bits() as usize);
            return Ok(false);
        }
        _ => {
            cold_path();
        }
    }

    /* Litlen subtable required (uncommon case)  */
    tmp_data.input_bitstream.remove_bits(LITLEN_TABLEBITS);

    // Surely a subtable pointer
    let entry = tables.litlen_decode_subtable[(fast_entry.get_subtable_index()
        + tmp_data
            .input_bitstream
            .bits(fast_entry.get_subtable_length() as usize))
        as usize];

    if entry.is_literal() {
        /* Literal  */
        tmp_data
            .input_bitstream
            .remove_bits(entry.get_maintable_length() as usize);

        tmp_data.last_state = DecodeEntryState {
            entry: FastDecodeEntry::new_from_literal(entry.get_literal()),
            offset: 0,
        };

        // literal_cb(context, entry.get_literal())?;
        tmp_data
            .input_bitstream
            .ensure_bits::<false>(DEFLATE_MAX_LITLEN_CODEWORD_LEN);
        return Ok(true);
    }

    /* end-of-block  */
    if unlikely(entry.is_end_of_block()) {
        tmp_data
            .input_bitstream
            .remove_bits(entry.get_maintable_length() as usize);
        return Ok(false);
    }

    tmp_data
        .input_bitstream
        .remove_bits(entry.get_maintable_length() as usize);

    // /* Match  */
    tmp_data.input_bitstream.force_ensure_bits();

    /* Pop the extra length bits and add them to the length base to
     * produce the full length.  */
    let length = entry.get_result()
        + tmp_data
            .input_bitstream
            .pop_bits(entry.get_subtable_length() as usize);

    /* Decode the match offset.  */
    let mut offset_entry =
        tables.offset_decode_table[tmp_data.input_bitstream.bits(OFFSET_TABLEBITS) as usize];
    if unlikely(offset_entry.is_subtable_pointer()) {
        /* Offset subtable required (uncommon case)  */
        tmp_data.input_bitstream.remove_bits(OFFSET_TABLEBITS);
        offset_entry = tables.offset_decode_subtable[(offset_entry.get_result()
            + tmp_data
                .input_bitstream
                .bits(offset_entry.get_subtable_length() as usize))
            as usize];
    }
    tmp_data
        .input_bitstream
        .remove_bits(offset_entry.get_maintable_length() as usize);

    const_assert!(
        can_ensure(DEFLATE_MAX_EXTRA_LENGTH_BITS + DEFLATE_MAX_OFFSET_CODEWORD_LEN)
            && can_ensure(DEFLATE_MAX_EXTRA_OFFSET_BITS)
    );

    /* Pop the extra offset bits and add them to the offset base to
     * produce the full offset.  */
    let offset = offset_entry.get_result()
        + tmp_data
            .input_bitstream
            .pop_bits(offset_entry.get_subtable_length() as usize);

    tmp_data.last_state = DecodeEntryState {
        entry: FastDecodeEntry::new_from_len(length as u16),
        offset: offset as usize,
    };

    Ok(true)
}

// #[inline(never)]
// pub fn decompress_with_instr<I: DeflateInput, O: DeflateOutput>(
//     d: &mut LibdeflateDecodeTables,
//     in_stream: &mut I,
//     out_stream: &mut O,
// ) -> Result<(), LibdeflateError> {
//     const INSTRUCTIONS_BUFFER_SIZE: usize = 8192;
//     let mut instructions = Vec::with_capacity(INSTRUCTIONS_BUFFER_SIZE);

//     deflate_decompress_template(
//         d,
//         in_stream,
//         &mut instructions,
//         // #[inline(always)]
//         // |instructions, input, len| Ok(()),
//         // #[inline(always)]
//         // |instructions, literal| unsafe {
//         //     let len = instructions.len();
//         //     *instructions.get_unchecked_mut(len) = (255 << 8) | (literal as u16);
//         //     instructions.set_len(len + 1);

//         //     Ok(())
//         // },
//         #[inline(always)]
//         |instructions, length, offset| {
//             unsafe {
//                 let len = instructions.len();
//                 *instructions.get_unchecked_mut(len) = length;
//                 *instructions.get_unchecked_mut(len + 1) = offset;
//                 instructions.set_len(len + 2);
//             }
//             Ok(())
//         },
//         |instructions| instructions.len() >= (INSTRUCTIONS_BUFFER_SIZE - 2),
//         |instructions| process_instructions(instructions, out_stream),
//     )
// }

#[inline(never)]
pub fn libdeflate_deflate_decompress<I: DeflateInput, O: DeflateOutput>(
    d: &mut LibdeflateDecodeTables,
    in_stream: &mut I,
    out_stream: &mut O,
) -> Result<(), LibdeflateError> {
    deflate_decompress_template(d, in_stream, out_stream)
}

#[inline(always)]
pub(crate) fn deflate_decompress_template<I: DeflateInput, O: DeflateOutput>(
    tables: &mut LibdeflateDecodeTables,
    in_stream: &mut I,
    out_stream: &mut O,
) -> Result<(), LibdeflateError> {
    let mut tmp_data = DecompressTempData {
        is_final_block: false,
        block_type: 0,
        input_bitstream: BitStream::new(in_stream),
        last_state: DecodeEntryState::default(),
    };

    'decompress_loop: loop {
        if tmp_data.is_final_block {
            break;
        }

        /* Read the next huffman block */
        if decode_huffman_block(tables, &mut tmp_data, out_stream)? {
            // Decoded an uncompressed block
            continue;
        }

        tmp_data
            .input_bitstream
            .input_stream
            .ensure_overread_length();
        // Decode the first instruction without processing it
        if unlikely(!decode_block_instruction::<_, _, true>(
            tables,
            &mut tmp_data,
            out_stream,
        )?) {
            continue;
        }

        'main_loop: loop {
            tmp_data
                .input_bitstream
                .input_stream
                .ensure_overread_length();

            while tmp_data
                .input_bitstream
                .input_stream
                .has_readable_overread()
                && out_stream.has_writable_length(DEFLATE_MAX_MATCH_LEN * 2)
            {
                if !decode_block_instruction::<_, _, false>(tables, &mut tmp_data, out_stream)? {
                    break 'main_loop;
                }
            }

            out_stream.flush_ensure_length(DEFLATE_MAX_MATCH_LEN * 2);

            // Invalid data or eof
            if !tmp_data.input_bitstream.input_stream.has_valid_bytes_slow() {
                break 'decompress_loop;
            }
        }
    }

    /* That was the last block.  */

    /* Discard any readahead bits and check for excessive overread */
    tmp_data.input_bitstream.align_input()?;

    Ok(())
}
