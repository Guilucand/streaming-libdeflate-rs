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

use std::time::Duration;

use crate::bitstream::can_ensure;
use crate::bitstream::BitStream;
use crate::bitstream::MAX_ENSURE;
use crate::decode_blocks::decode_dynamic_huffman_block;
use crate::decode_blocks::decode_huffman_header;
use crate::decode_blocks::decode_uncompressed_block;
use crate::decode_blocks::load_static_huffman_block;
use crate::decompress_utils::*;
use crate::deflate_constants::*;
use crate::unchecked::UncheckedArray;
use crate::{DeflateInput, DeflateOutput, LibdeflateDecompressor, LibdeflateError};
use nightly_quirks::branch_pred::likely;
use nightly_quirks::branch_pred::unlikely;

pub const PRECODE_TABLEBITS: usize = 7;
pub const LITLEN_TABLEBITS: usize = 10;
pub const OFFSET_TABLEBITS: usize = 8;

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
pub const PRECODE_ENOUGH: usize = 128; /* enough 19 7 7	*/
pub const LITLEN_ENOUGH: usize = 1334; /* enough 288 10 15	*/
pub const OFFSET_ENOUGH: usize = 402; /* enough 32 8 15	*/

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

#[derive(Copy, Clone)]
pub(crate) struct _DecStruct {
    pub(crate) lens: UncheckedArray<
        LenType,
        { DEFLATE_NUM_LITLEN_SYMS + DEFLATE_NUM_OFFSET_SYMS + DEFLATE_MAX_LENS_OVERRUN },
    >,
    pub(crate) precode_decode_table: UncheckedArray<DecodeEntry, PRECODE_ENOUGH>,
}

/*
 * This is the actual DEFLATE decompression routine, lifted out of
 * deflate_decompress.c so that it can be compiled multiple times with different
 * target instruction sets.
 */

#[inline(never)]
fn process_instructions<O: DeflateOutput>(
    instructions: &mut Vec<u16>,
    output_stream: &mut O,
) -> Result<(), LibdeflateError> {
    // return Ok(());
    let mut instr_idx = 0;
    while instr_idx < instructions.len() {
        let instruction = instructions[instr_idx];

        if instruction & 0xFF00 == 0xFF00 {
            // Literal
            let literal = (instruction & 0x00FF) as u8;
            safety_check!(output_stream.write(&[literal]));
        } else {
            // Match
            let length = instruction as usize;
            instr_idx += 1; // Move to the next instruction for offset
            let offset = instructions[instr_idx] as usize;

            safety_check!(output_stream.copy_forward(offset, length));
        }

        instr_idx += 1;
    }

    instructions.clear();
    Ok(())
}

#[inline(always)]
fn decode_block_instruction<I: DeflateInput, C>(
    d: &mut LibdeflateDecompressor,
    tmp_data: &mut DecompressTempData<I>,
    context: &mut C,
    literal_cb: fn(&mut C, u8) -> Result<(), LibdeflateError>,
    backref_cb: fn(&mut C, u16, u16) -> Result<(), LibdeflateError>,
) -> Result<bool, LibdeflateError> {
    /* Decode a litlen symbol.  */
    tmp_data
        .input_bitstream
        .ensure_bits(DEFLATE_MAX_LITLEN_CODEWORD_LEN);
    tmp_data.entry =
        d.litlen_decode_table[tmp_data.input_bitstream.bits(LITLEN_TABLEBITS) as usize];

    // Fast literal match
    if likely(tmp_data.entry.is_literal()) {
        /* Literal  */
        tmp_data
            .input_bitstream
            .remove_bits(tmp_data.entry.get_maintable_length() as usize);
        literal_cb(context, tmp_data.entry.get_literal())?;

        return Ok(true);
    }

    if unlikely(tmp_data.entry.is_exceptional()) {
        if unlikely(tmp_data.entry.is_subtable_pointer()) {
            /* Litlen subtable required (uncommon case)  */
            tmp_data.input_bitstream.remove_bits(LITLEN_TABLEBITS);
            tmp_data.entry = d.litlen_decode_table[(tmp_data.entry.get_result()
                + tmp_data
                    .input_bitstream
                    .bits(tmp_data.entry.get_subtable_length() as usize))
                as usize];
        }
        if tmp_data.entry.is_literal() {
            /* Literal  */
            tmp_data
                .input_bitstream
                .remove_bits(tmp_data.entry.get_maintable_length() as usize);
            literal_cb(context, tmp_data.entry.get_literal())?;
            return Ok(true);
        }

        /* end-of-block  */
        if unlikely(tmp_data.entry.is_end_of_block()) {
            tmp_data
                .input_bitstream
                .remove_bits(tmp_data.entry.get_maintable_length() as usize);
            return Ok(false);
        }
    }

    tmp_data
        .input_bitstream
        .remove_bits(tmp_data.entry.get_maintable_length() as usize);

    /* Match  */
    tmp_data.input_bitstream.ensure_bits(MAX_ENSURE);

    /* Pop the extra length bits and add them to the length base to
     * produce the full length.  */
    let length = tmp_data.entry.get_result()
        + tmp_data
            .input_bitstream
            .pop_bits(tmp_data.entry.get_subtable_length() as usize);

    /* Decode the match offset.  */

    tmp_data.entry =
        d.offset_decode_table[tmp_data.input_bitstream.bits(OFFSET_TABLEBITS) as usize];
    if unlikely(tmp_data.entry.is_subtable_pointer()) {
        /* Offset subtable required (uncommon case)  */
        tmp_data.input_bitstream.remove_bits(OFFSET_TABLEBITS);
        tmp_data.entry = d.offset_decode_table[(tmp_data.entry.get_result()
            + tmp_data
                .input_bitstream
                .bits(tmp_data.entry.get_subtable_length() as usize))
            as usize];
    }
    tmp_data
        .input_bitstream
        .remove_bits(tmp_data.entry.get_maintable_length() as usize);

    const_assert!(
        can_ensure(DEFLATE_MAX_EXTRA_LENGTH_BITS + DEFLATE_MAX_OFFSET_CODEWORD_LEN)
            && can_ensure(DEFLATE_MAX_EXTRA_OFFSET_BITS)
    );
    if !can_ensure(
        DEFLATE_MAX_EXTRA_LENGTH_BITS
            + DEFLATE_MAX_OFFSET_CODEWORD_LEN
            + DEFLATE_MAX_EXTRA_OFFSET_BITS,
    ) {
        tmp_data
            .input_bitstream
            .ensure_bits(DEFLATE_MAX_EXTRA_OFFSET_BITS);
    }

    /* Pop the extra offset bits and add them to the offset base to
     * produce the full offset.  */
    let offset = tmp_data.entry.get_result()
        + tmp_data
            .input_bitstream
            .pop_bits(tmp_data.entry.get_subtable_length() as usize);

    /*
     * Copy the match: 'length' bytes at 'out_next - offset' to
     * 'out_next', possibly overlapping.  If the match doesn't end
     * too close to the end of the buffer and offset >= WORDBYTES ||
     * offset == 1, take a fast path which copies a word at a time
     * -- potentially more than the length of the match, but that's
     * fine as long as we check for enough extra space.
     *
     * The remaining cases are not performance-critical so are
     * handled by a simple byte-by-byte copy.
     */

    backref_cb(context, length as u16, offset as u16)?;
    Ok(true)
}

#[inline(never)]
pub fn decompress_with_instr<I: DeflateInput, O: DeflateOutput>(
    d: &mut LibdeflateDecompressor,
    in_stream: &mut I,
    out_stream: &mut O,
) -> Result<(), LibdeflateError> {
    const INSTRUCTIONS_BUFFER_SIZE: usize = 8192;
    let mut instructions = Vec::with_capacity(INSTRUCTIONS_BUFFER_SIZE);

    deflate_decompress_template(
        d,
        in_stream,
        &mut instructions,
        #[inline(always)]
        |instructions, input, len| Ok(()),
        #[inline(always)]
        |instructions, literal| unsafe {
            let len = instructions.len();
            *instructions.get_unchecked_mut(len) = (255 << 8) | (literal as u16);
            instructions.set_len(len + 1);

            Ok(())
        },
        #[inline(always)]
        |instructions, length, offset| {
            unsafe {
                let len = instructions.len();
                *instructions.get_unchecked_mut(len) = length;
                *instructions.get_unchecked_mut(len + 1) = offset;
                instructions.set_len(len + 2);
            }
            Ok(())
        },
        |instructions| instructions.len() >= (INSTRUCTIONS_BUFFER_SIZE - 2),
        |instructions| process_instructions(instructions, out_stream),
    )
}

#[inline(never)]
pub fn decompress_direct<I: DeflateInput, O: DeflateOutput>(
    d: &mut LibdeflateDecompressor,
    in_stream: &mut I,
    out_stream: &mut O,
) -> Result<(), LibdeflateError> {
    deflate_decompress_template(
        d,
        in_stream,
        out_stream,
        |out_stream, input, len| {
            safety_check!(input.read_exact_into(out_stream, len as usize));
            Ok(())
        },
        #[inline(always)]
        |out_stream, literal| {
            if !out_stream.write(&[literal]) {
                return Err(LibdeflateError::InsufficientSpace);
            }
            Ok(())
        },
        #[inline(always)]
        |out_stream, length, offset| {
            safety_check!(out_stream.copy_forward(offset as usize, length as usize));
            Ok(())
        },
        #[inline(always)]
        |_| true,
        #[inline(always)]
        |_| Ok(()),
    )?;

    Ok(())
}

#[inline(always)]
pub(crate) fn deflate_decompress_template<I: DeflateInput, C>(
    d: &mut LibdeflateDecompressor,
    in_stream: &mut I,
    context: &mut C,
    uncompressed_cb: fn(&mut C, &mut BitStream<I>, usize) -> Result<(), LibdeflateError>,
    literal_cb: fn(&mut C, u8) -> Result<(), LibdeflateError>,
    backref_cb: fn(&mut C, u16, u16) -> Result<(), LibdeflateError>,
    call_flush: fn(&C) -> bool,
    mut flush_cb: impl FnMut(&mut C) -> Result<(), LibdeflateError>,
) -> Result<(), LibdeflateError> {
    let mut tmp_data = DecompressTempData {
        is_final_block: false,
        block_type: 0,
        num_litlen_syms: 0,
        num_offset_syms: 0,
        input_bitstream: BitStream::new(in_stream),
        entry: DecodeEntry::DEFAULT,
    };

    'block_done: loop {
        if tmp_data.is_final_block {
            break;
        }

        /* Starting to read the next block.  */
        decode_huffman_header(&mut tmp_data);

        if tmp_data.block_type == DEFLATE_BLOCKTYPE_DYNAMIC_HUFFMAN {
            /* Dynamic Huffman block.  */
            d.static_codes_loaded = false;
            decode_dynamic_huffman_block(d, &mut tmp_data)?;
        } else if tmp_data.block_type == DEFLATE_BLOCKTYPE_UNCOMPRESSED {
            /* Uncompressed block: copy 'len' bytes literally from the input
             * buffer to the output buffer.  */
            decode_uncompressed_block(&mut tmp_data, context, uncompressed_cb)?;
            continue 'block_done;
        } else {
            safety_check!(tmp_data.block_type == DEFLATE_BLOCKTYPE_STATIC_HUFFMAN);

            /*
             * Static Huffman block: build the decode tables for the static
             * codes.  Skip doing so if the tables are already set up from
             * an earlier static block; this speeds up decompression of
             * degenerate input of many empty or very short static blocks.
             *
             * Afterwards, the remainder is the same as decompressing a
             * dynamic Huffman block.
             */

            if !d.static_codes_loaded {
                d.static_codes_loaded = true;
                load_static_huffman_block(d, &mut tmp_data);
            }
        }

        let mut count = 0;

        /* The main DEFLATE decode loop  */
        'main_loop: loop {
            loop {
                count += 1;
                if !decode_block_instruction(d, &mut tmp_data, context, literal_cb, backref_cb)? {
                    break 'main_loop;
                }
                if call_flush(context) {
                    break;
                }
            }
            if tmp_data.input_bitstream.has_overrun() {
                break;
            }
            flush_cb(context)?;
        }
    }

    flush_cb(context)?;
    /* That was the last block.  */

    /* Discard any readahead bits and check for excessive overread */
    tmp_data.input_bitstream.align_input(0)?;

    Ok(())
}
