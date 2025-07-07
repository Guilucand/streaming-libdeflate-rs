// #![cfg_attr(debug_assertions, deny(warnings))]
pub mod bitstream;

pub(crate) mod block_finder;
pub mod decode_blocks;
pub mod decompress_deflate;
pub mod decompress_gzip;
mod decompress_utils;
mod deflate_constants;
mod gzip_constants;
pub mod streams;
pub mod unchecked;

#[macro_use]
extern crate static_assertions;

use crate::decompress_deflate::{
    HuffmanDecodeStruct, OutStreamResult, FAST_TABLESIZE, LITLEN_SUBTABLESIZE, LITLEN_TABLESIZE,
    OFFSET_SUBTABLESIZE, OFFSET_TABLESIZE,
};
use crate::decompress_gzip::libdeflate_gzip_decompress;
use crate::decompress_utils::fast_decode_entry::FastDecodeEntry;
use crate::deflate_constants::DEFLATE_MAX_NUM_SYMS;
use crate::streams::deflate_chunked_buffer_input::DeflateChunkedBufferInput;
use crate::streams::deflate_chunked_buffer_output::DeflateChunkedBufferOutput;
use crate::unchecked::UncheckedArray;
use std::fs::File;
use std::io::Read;
use std::mem::size_of;
use std::path::Path;

/*
 * The main DEFLATE decompressor structure.  Since this implementation only
 * supports full buffer decompression, this structure does not store the entire
 * decompression state, but rather only some arrays that are too large to
 * comfortably allocate on the stack.
 */
pub struct LibdeflateDecodeTables {
    pub(crate) huffman_decode: HuffmanDecodeStruct,
    pub(crate) litlen_decode_table: UncheckedArray<FastDecodeEntry, LITLEN_TABLESIZE>,

    pub(crate) offset_decode_table: UncheckedArray<FastDecodeEntry, OFFSET_TABLESIZE>,

    pub(crate) fast_decode_table: UncheckedArray<FastDecodeEntry, FAST_TABLESIZE>,

    pub(crate) litlen_decode_subtable: UncheckedArray<FastDecodeEntry, LITLEN_SUBTABLESIZE>,
    pub(crate) offset_decode_subtable: UncheckedArray<FastDecodeEntry, OFFSET_SUBTABLESIZE>,

    /* used only during build_decode_table() */
    pub(crate) sorted_syms: UncheckedArray<u16, DEFLATE_MAX_NUM_SYMS>,
    pub(crate) static_codes_loaded: bool,
}

/*
 * Result of a call to libdeflate_deflate_decompress(),
 * libdeflate_zlib_decompress(), or libdeflate_gzip_decompress().
 */
#[derive(Debug)]
pub enum LibdeflateError {
    /* Decompressed failed because the compressed data was invalid, corrupt,
     * or otherwise unsupported.  */
    BadData = 1,

    /* A NULL 'actual_out_nbytes_ret' was provided, but the data would have
     * decompressed to fewer than 'out_nbytes_avail' bytes.  */
    ShortOutput = 2,

    /* The data would have decompressed to more than 'out_nbytes_avail'
     * bytes.  */
    InsufficientSpace = 3,
}

pub trait DeflateInput {
    const MAX_LOOK_BACK: usize = size_of::<usize>() * 2;
    const MAX_OVERREAD: usize = size_of::<usize>() * 2;

    unsafe fn get_le_word_no_advance(&mut self) -> usize;
    fn move_stream_pos<const REFILL: bool>(&mut self, amount: isize);
    fn get_stream_pos_mut(&mut self) -> &mut usize;
    fn tell_stream_pos(&self) -> usize;
    fn read<const REFILL: bool>(&mut self, out_data: &mut [u8]) -> usize;
    // Ensure that the current buffer has at least `Self::MAX_OVERREAD` elements. this function must never fail
    fn ensure_overread_length(&mut self);
    // Check if the stream buffer has at least Self::MAX_OVERREAD bytes remaining with either valid data or eof data
    fn has_readable_overread(&self) -> bool;
    fn has_valid_bytes_slow(&mut self) -> bool;
    fn read_exact_into<O: DeflateOutput>(&mut self, out_stream: &mut O, length: usize) -> bool;

    #[inline(always)]
    fn read_byte<const REFILL: bool>(&mut self) -> u8 {
        let mut byte = [0];
        self.read::<REFILL>(&mut byte);
        byte[0]
    }

    #[inline(always)]
    fn read_le_u16<const REFILL: bool>(&mut self) -> u16 {
        let mut bytes = [0, 0];
        self.read::<REFILL>(&mut bytes);
        u16::from_le_bytes(bytes)
    }

    #[inline(always)]
    fn read_le_u32<const REFILL: bool>(&mut self) -> u32 {
        let mut bytes = [0, 0, 0, 0];
        self.read::<REFILL>(&mut bytes);
        u32::from_le_bytes(bytes)
    }
}

pub trait DeflateOutput {
    const MAX_LOOK_BACK: usize = 32768;
    const OVERWRITE_MAX: usize = 16;

    fn has_writable_length(&mut self, length: usize) -> bool;
    fn flush_ensure_length(&mut self, length: usize) -> bool;

    fn get_output_ptr(&mut self) -> *mut u8;
    unsafe fn set_output_ptr(&mut self, ptr: *mut u8);

    fn final_flush(&mut self) -> Result<OutStreamResult, ()>;
}

pub fn libdeflate_alloc_decode_tables() -> LibdeflateDecodeTables {
    LibdeflateDecodeTables {
        huffman_decode: HuffmanDecodeStruct {
            lens: UncheckedArray::default(),
            precode_lens: UncheckedArray::default(),
            precode_decode_table: UncheckedArray::default(),
            fast_temp_litlen: Vec::with_capacity(FAST_TABLESIZE),
        },
        litlen_decode_table: UncheckedArray::default(),
        offset_decode_table: UncheckedArray::default(),
        fast_decode_table: UncheckedArray::default(),

        litlen_decode_subtable: UncheckedArray::default(),
        offset_decode_subtable: UncheckedArray::default(),

        sorted_syms: UncheckedArray::default(),
        static_codes_loaded: false,
    }
}

pub fn decompress_file_buffered(
    file: impl AsRef<Path>,
    func: impl FnMut(&[u8]) -> Result<(), ()>,
    buf_size: usize,
) -> Result<(), LibdeflateError> {
    let mut read_file = File::open(file).unwrap();
    let mut input_stream =
        DeflateChunkedBufferInput::new(|buf| read_file.read(buf).unwrap_or(0), buf_size);

    let mut output_stream = DeflateChunkedBufferOutput::new(func, buf_size);

    let mut decompressor = libdeflate_alloc_decode_tables();

    while {
        input_stream.ensure_overread_length();
        input_stream.has_valid_bytes_slow()
    } {
        libdeflate_gzip_decompress(&mut decompressor, &mut input_stream, &mut output_stream)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::decompress_file_buffered;
    use rayon::prelude::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Instant;

    #[test]
    fn decompression_speed() {
        let context = Arc::new(AtomicUsize::new(0));

        const PATH: &str = "/home/andrea/genome-assembly/data/salmonella-strains/strains-test";

        let paths = std::fs::read_dir(PATH).unwrap();
        let mut paths_vec = Vec::new();

        for path in paths {
            paths_vec.push(path.unwrap().path());
        }

        paths_vec.sort();
        paths_vec.truncate(10000);
        let start = Instant::now();

        paths_vec.into_par_iter().for_each(|file| {
            let context = context.clone();

            match decompress_file_buffered(
                &file,
                |data| {
                    let mut rem = 0;
                    for d in data {
                        rem += *d as usize;
                    }
                    context.fetch_add(rem, Ordering::Relaxed);
                    Ok(())
                },
                1024 * 512,
            ) {
                Ok(_) => {}
                Err(_error) => {
                    println!("Error: {}", file.display());
                }
            }
        });

        println!("Bench duration: {:.2}", start.elapsed().as_secs_f32());
    }
}
