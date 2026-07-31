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

use crate::bitstream::{BitStream, BitStreamState};
use crate::decode_blocks::{decode_huffman_block, DecodeBlockResult};
use crate::decompress_utils::fast_decode_entry::FastDecodeEntry;
use crate::decompress_utils::*;
use crate::deflate_constants::*;
use crate::unchecked::UncheckedArray;
use crate::{
    DeflateInput, DeflateOutput, LibdeflateDecodeTables, LibdeflateDecompressResult,
    LibdeflateError,
};
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
    pub(crate) precode_decode_table: UncheckedArray<FastDecodeEntry, PRECODE_ENOUGH>,
    pub(crate) fast_temp_litlen: Vec<TempFastDecodeBuild>,
}

#[inline(always)]
fn process_entry<const SINGLE_BYTES: bool>(
    state: DecodeEntryState,
    output_stream: &mut impl DeflateOutput,
) {
    unsafe {
        let mut out_ptr = output_stream.get_output_ptr();
        (out_ptr as *mut u16).write_unaligned(state.entry.get_literals());
        out_ptr = out_ptr.add(state.entry.get_literals_count() as usize);

        let mut src = out_ptr.sub(state.offset) as *const u64;
        let mut dst = out_ptr as *mut u64;

        let length = state.entry.get_len_value() as usize;

        if SINGLE_BYTES {
            if state.offset == 1 {
                /* RLE encoding of previous byte, common if the
                 * data contains many repeated bytes */
                let v = u64::from_ne_bytes([*(src as *const u8); std::mem::size_of::<usize>()]);

                std::ptr::write_unaligned(dst, v);
                dst = dst.add(1);
                std::ptr::write_unaligned(dst, v);
                dst = dst.add(1);

                let mut remaining = length as isize - 16;
                while remaining > 0 {
                    std::ptr::write_unaligned(dst, v);
                    dst = dst.add(1);
                    remaining -= 8;
                }
            }

            let mut remaining = length as isize;
            let mut src = src as *const u8;
            let mut dst = dst as *mut u8;
            loop {
                copy_word_unaligned(src as *const u64, dst as *mut u64);
                src = src.add(state.offset);
                dst = dst.add(state.offset);
                copy_word_unaligned(src as *const u64, dst as *mut u64);
                src = src.add(state.offset);
                dst = dst.add(state.offset);

                remaining -= (state.offset * 2) as isize;
                if remaining <= 0 {
                    break;
                }
            }
        } else {
            let mut remaining = length as isize - 40;
            copy_word_unaligned(src, dst);

            src = src.add(1);
            dst = dst.add(1);

            loop {
                copy_word_unaligned(src, dst);
                src = src.add(1);
                dst = dst.add(1);
                copy_word_unaligned(src, dst);
                src = src.add(1);
                dst = dst.add(1);
                copy_word_unaligned(src, dst);
                src = src.add(1);
                dst = dst.add(1);
                copy_word_unaligned(src, dst);
                src = src.add(1);
                dst = dst.add(1);

                if likely(remaining <= 0) {
                    break;
                }
                remaining -= 32;
            }
        }

        output_stream.set_output_ptr(out_ptr.add(length));
    }
}

#[inline(always)]
fn init_block_instruction<I: DeflateInput>(
    tables: &LibdeflateDecodeTables,
    tmp_data: &mut DecompressTempData<I>,
) {
    tmp_data.input_bitstream.force_ensure_bits();
    tmp_data.fast_entry =
        tables.fast_decode_table[tmp_data.input_bitstream.bits(FAST_TABLEBITS) as usize];
}

#[inline(always)]
fn decode_block_instruction<I: DeflateInput, O: DeflateOutput>(
    tables: &LibdeflateDecodeTables,
    tmp_data: &mut DecompressTempData<I>,
    output_stream: &mut O,
) -> Result<bool, LibdeflateError> {
    if likely(tmp_data.fast_entry.get_flags() == 0) {
        let offset_bits = tmp_data.fast_entry.get_extra_offset_bits();
        let consumed_bits = tmp_data.fast_entry.get_consumed_bits();
        let state = DecodeEntryState {
            entry: tmp_data.fast_entry,
            offset: tmp_data.fast_entry.get_offset_value() as usize
                + tmp_data
                    .input_bitstream
                    .bits_with_offset(consumed_bits, offset_bits) as usize,
        };
        tmp_data
            .input_bitstream
            .remove_bits((offset_bits + consumed_bits) as usize);

        tmp_data.fast_entry =
            tables.fast_decode_table[tmp_data.input_bitstream.bits(FAST_TABLEBITS) as usize];

        process_entry::<false>(state, output_stream);

        if likely(tmp_data.fast_entry.get_flags() == 0) {
            let offset_bits = tmp_data.fast_entry.get_extra_offset_bits();
            let consumed_bits = tmp_data.fast_entry.get_consumed_bits();
            let state = DecodeEntryState {
                entry: tmp_data.fast_entry,
                offset: tmp_data.fast_entry.get_offset_value() as usize
                    + tmp_data
                        .input_bitstream
                        .bits_with_offset(consumed_bits, offset_bits)
                        as usize,
            };
            tmp_data
                .input_bitstream
                .remove_bits((offset_bits + consumed_bits) as usize);

            tmp_data.fast_entry =
                tables.fast_decode_table[tmp_data.input_bitstream.bits(FAST_TABLEBITS) as usize];
            tmp_data.input_bitstream.force_ensure_bits();

            process_entry::<false>(state, output_stream);

            return Ok(true);
        }
        tmp_data.input_bitstream.force_ensure_bits();
    }

    #[cold]
    #[inline]
    fn cold_path() {}

    let state = match tmp_data.fast_entry.get_state_flags() {
        FastDecodeEntry::EXC_LEN_FULLSIZE | FastDecodeEntry::EXC_LEN_EXTRABIT => {
            tmp_data
                .input_bitstream
                .remove_bits(tmp_data.fast_entry.get_consumed_bits() as usize);

            let extra_len = if unlikely(
                tmp_data.fast_entry.get_state_flags() == FastDecodeEntry::EXC_LEN_EXTRABIT,
            ) {
                tmp_data
                    .input_bitstream
                    .pop_bits(tmp_data.fast_entry.get_exceptional_length_bits() as usize)
            } else {
                0
            };

            let mut offset_entry = tables.offset_decode_table
                [tmp_data.input_bitstream.bits(OFFSET_TABLEBITS) as usize];

            if unlikely(offset_entry.is_subtable_pointer()) {
                tmp_data.input_bitstream.remove_bits(OFFSET_TABLEBITS);
                offset_entry = tables.offset_decode_subtable[(offset_entry.get_subtable_index()
                    + tmp_data
                        .input_bitstream
                        .bits(offset_entry.get_subtable_length() as usize))
                    as usize];
            }

            let offset_consumed = offset_entry.get_consumed_bits();
            let offset_extrabit = offset_entry.get_extra_offset_bits();

            tmp_data.fast_entry.inc_len_value(extra_len as u16);
            tmp_data.fast_entry.or_flags(offset_entry.get_copy_flags());
            let state = DecodeEntryState {
                entry: tmp_data.fast_entry,
                offset: offset_entry.get_offset_value() as usize
                    + tmp_data
                        .input_bitstream
                        .bits_with_offset(offset_consumed, offset_extrabit)
                        as usize,
            };

            tmp_data
                .input_bitstream
                .remove_bits(offset_consumed as usize + offset_extrabit as usize);
            state
        }
        FastDecodeEntry::EXC_END_OF_BLOCK => {
            cold_path();
            tmp_data
                .input_bitstream
                .remove_bits(tmp_data.fast_entry.get_consumed_bits() as usize);
            return Ok(false);
        }
        FastDecodeEntry::EXC_LEN_SUBTABLE => {
            cold_path();

            /* Litlen subtable required (uncommon case)  */
            tmp_data.input_bitstream.remove_bits(LITLEN_TABLEBITS);

            // It is a subtable pointer
            tmp_data.fast_entry =
                tables.litlen_decode_subtable[(tmp_data.fast_entry.get_subtable_index()
                    + tmp_data
                        .input_bitstream
                        .bits(tmp_data.fast_entry.get_subtable_length() as usize))
                    as usize];
            tmp_data.input_bitstream.force_ensure_bits();

            return Ok(true);
        }
        _ => {
            let offset_bits = tmp_data.fast_entry.get_extra_offset_bits();
            let consumed_bits = tmp_data.fast_entry.get_consumed_bits();

            let state = DecodeEntryState {
                entry: tmp_data.fast_entry,
                offset: tmp_data.fast_entry.get_offset_value() as usize
                    + tmp_data
                        .input_bitstream
                        .bits_with_offset(consumed_bits, offset_bits)
                        as usize,
            };

            let tot_bits = offset_bits + consumed_bits;
            tmp_data.input_bitstream.remove_bits(tot_bits as usize);
            state
        }
    };

    tmp_data.fast_entry =
        tables.fast_decode_table[tmp_data.input_bitstream.bits(FAST_TABLEBITS) as usize];
    tmp_data.input_bitstream.force_ensure_bits();

    match state.entry.get_copy_flags() {
        FastDecodeEntry::EXC_SMALLOFFSET => {
            process_entry::<true>(state, output_stream);
        }
        _ => {
            process_entry::<false>(state, output_stream);
        }
    }

    Ok(true)
}

pub const MAX_WRITE: usize = (DEFLATE_MAX_MATCH_LEN + (FastDecodeEntry::MAX_LITERALS as usize)) * 2;

const UNCOMPRESSED_COPY_CHUNK_SIZE: usize = 256;

#[derive(Copy, Clone, PartialEq, Eq)]
enum DeflateDecompressStage {
    ReadBlockHeader,
    HuffmanBody,
    UncompressedBody,
    Finish,
    DrainOutput,
    Done,
}

pub struct LibdeflateDeflateDecompressor {
    bitstream: BitStreamState,
    is_final_block: bool,
    block_type: u32,
    fast_entry: FastDecodeEntry,
    stage: DeflateDecompressStage,
    uncompressed_remaining: usize,
}

impl LibdeflateDeflateDecompressor {
    pub fn new() -> Self {
        Self {
            bitstream: BitStreamState::default(),
            is_final_block: false,
            block_type: 0,
            fast_entry: FastDecodeEntry::DEFAULT,
            stage: DeflateDecompressStage::ReadBlockHeader,
            uncompressed_remaining: 0,
        }
    }

    fn save_tmp_data<I: DeflateInput>(&mut self, tmp_data: &DecompressTempData<I>) {
        self.bitstream = tmp_data.input_bitstream.save_state();
        self.is_final_block = tmp_data.is_final_block;
        self.block_type = tmp_data.block_type;
        self.fast_entry = tmp_data.fast_entry;
    }

    #[inline(never)]
    pub fn decompress<I: DeflateInput, O: DeflateOutput>(
        &mut self,
        tables: &mut LibdeflateDecodeTables,
        in_stream: &mut I,
        out_stream: &mut O,
    ) -> Result<LibdeflateDecompressResult, LibdeflateError> {
        if self.stage == DeflateDecompressStage::Done {
            return Ok(LibdeflateDecompressResult::EndOfStream);
        }

        let mut tmp_data = DecompressTempData {
            is_final_block: self.is_final_block,
            block_type: self.block_type,
            input_bitstream: BitStream::from_state(in_stream, self.bitstream),
            fast_entry: self.fast_entry,
        };

        loop {
            match self.stage {
                DeflateDecompressStage::ReadBlockHeader => {
                    safety_check!(tmp_data.input_bitstream.input_stream.has_valid_bytes_slow());

                    match decode_huffman_block(tables, &mut tmp_data, out_stream)? {
                        DecodeBlockResult::Huffman => {
                            self.is_final_block = tmp_data.is_final_block;
                            self.block_type = tmp_data.block_type;
                            tmp_data
                                .input_bitstream
                                .input_stream
                                .ensure_overread_length();
                            init_block_instruction(tables, &mut tmp_data);
                            self.stage = DeflateDecompressStage::HuffmanBody;
                        }
                        DecodeBlockResult::Uncompressed { len } => {
                            self.is_final_block = tmp_data.is_final_block;
                            self.block_type = tmp_data.block_type;
                            self.uncompressed_remaining = len;
                            self.stage = DeflateDecompressStage::UncompressedBody;
                        }
                    }
                }

                DeflateDecompressStage::HuffmanBody => loop {
                    tmp_data
                        .input_bitstream
                        .input_stream
                        .ensure_overread_length();

                    while tmp_data
                        .input_bitstream
                        .input_stream
                        .has_readable_overread()
                    {
                        if !out_stream.has_writable_length(MAX_WRITE) {
                            self.save_tmp_data(&tmp_data);
                            return Ok(LibdeflateDecompressResult::MoreData);
                        }

                        if !decode_block_instruction::<_, _>(tables, &mut tmp_data, out_stream)? {
                            self.stage = if tmp_data.is_final_block {
                                DeflateDecompressStage::Finish
                            } else {
                                DeflateDecompressStage::ReadBlockHeader
                            };
                            break;
                        }
                    }

                    if self.stage != DeflateDecompressStage::HuffmanBody {
                        break;
                    }

                    if !out_stream.has_writable_length(MAX_WRITE) {
                        self.save_tmp_data(&tmp_data);
                        return Ok(LibdeflateDecompressResult::MoreData);
                    }

                    // Invalid data or eof.  Keep the previous behavior and let
                    // final input alignment validate the stream boundary.
                    if !tmp_data.input_bitstream.input_stream.has_valid_bytes_slow() {
                        self.stage = DeflateDecompressStage::Finish;
                        break;
                    }
                },

                DeflateDecompressStage::UncompressedBody => {
                    while self.uncompressed_remaining > 0 {
                        if !out_stream.has_writable_length(UNCOMPRESSED_COPY_CHUNK_SIZE) {
                            self.save_tmp_data(&tmp_data);
                            return Ok(LibdeflateDecompressResult::MoreData);
                        }

                        tmp_data
                            .input_bitstream
                            .input_stream
                            .ensure_overread_length();

                        let copy_len = self
                            .uncompressed_remaining
                            .min(UNCOMPRESSED_COPY_CHUNK_SIZE);
                        let out_ptr = out_stream.get_output_ptr();
                        let out_slice =
                            unsafe { std::slice::from_raw_parts_mut(out_ptr, copy_len) };
                        let copied = tmp_data
                            .input_bitstream
                            .input_stream
                            .read::<true>(out_slice);

                        safety_check!(copied != 0);

                        unsafe {
                            out_stream.set_output_ptr(out_ptr.add(copied));
                        }
                        self.uncompressed_remaining -= copied;
                    }

                    self.stage = if self.is_final_block {
                        DeflateDecompressStage::Finish
                    } else {
                        DeflateDecompressStage::ReadBlockHeader
                    };
                }

                DeflateDecompressStage::Finish => {
                    tmp_data.input_bitstream.align_input()?;
                    self.save_tmp_data(&tmp_data);
                    self.stage = DeflateDecompressStage::DrainOutput;
                }

                DeflateDecompressStage::DrainOutput => {
                    self.save_tmp_data(&tmp_data);
                    if !out_stream.pending_output().is_empty() {
                        return Ok(LibdeflateDecompressResult::MoreData);
                    }
                    self.stage = DeflateDecompressStage::Done;
                    return Ok(LibdeflateDecompressResult::EndOfStream);
                }

                DeflateDecompressStage::Done => {
                    self.save_tmp_data(&tmp_data);
                    return Ok(LibdeflateDecompressResult::EndOfStream);
                }
            }
        }
    }
}

impl Default for LibdeflateDeflateDecompressor {
    fn default() -> Self {
        Self::new()
    }
}
