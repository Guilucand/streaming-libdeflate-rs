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
use crate::decompress_gzip::LibdeflateGzipDecompressor;
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

/* Result of a decompressor resume call. */
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

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum LibdeflateDecompressResult {
    MoreData,
    EndOfStream,
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

    fn get_output_ptr(&mut self) -> *mut u8;
    unsafe fn set_output_ptr(&mut self, ptr: *mut u8);

    fn pending_output(&self) -> &[u8];
    fn consume_output(&mut self);
    fn finish_member(&mut self) -> Result<OutStreamResult, ()>;
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
    mut func: impl FnMut(&[u8]) -> Result<(), ()>,
    buf_size: usize,
) -> Result<(), LibdeflateError> {
    let mut read_file = File::open(file).unwrap();
    let mut input_stream =
        DeflateChunkedBufferInput::new(|buf| read_file.read(buf).unwrap_or(0), buf_size);

    let mut output_stream = DeflateChunkedBufferOutput::new(buf_size);

    let mut decompressor = libdeflate_alloc_decode_tables();

    while {
        input_stream.ensure_overread_length();
        input_stream.has_valid_bytes_slow()
    } {
        let mut gzip_decompressor = LibdeflateGzipDecompressor::new();

        loop {
            match gzip_decompressor.decompress(
                &mut decompressor,
                &mut input_stream,
                &mut output_stream,
            )? {
                LibdeflateDecompressResult::MoreData => {
                    func(output_stream.pending_output())
                        .map_err(|_| LibdeflateError::InsufficientSpace)?;
                    output_stream.consume_output();
                }
                LibdeflateDecompressResult::EndOfStream => break,
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::decompress_deflate::LibdeflateDeflateDecompressor;
    use crate::decompress_gzip::LibdeflateGzipDecompressor;
    use crate::streams::deflate_chunked_buffer_input::DeflateChunkedBufferInput;
    use crate::streams::deflate_chunked_buffer_output::DeflateChunkedBufferOutput;
    use crate::{
        decompress_file_buffered, libdeflate_alloc_decode_tables, DeflateOutput,
        LibdeflateDecompressResult, LibdeflateError,
    };
    use crc32fast::Hasher;
    use rayon::prelude::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Instant;

    fn stored_deflate_blocks(data: &[u8], block_size: usize) -> Vec<u8> {
        let mut deflate = Vec::new();
        let chunks_len = data.chunks(block_size).count().max(1);

        for (index, chunk) in data.chunks(block_size).enumerate() {
            deflate.push(if index + 1 == chunks_len { 1 } else { 0 });
            let len = chunk.len() as u16;
            deflate.extend_from_slice(&len.to_le_bytes());
            deflate.extend_from_slice(&(!len).to_le_bytes());
            deflate.extend_from_slice(chunk);
        }

        if data.is_empty() {
            deflate.push(1);
            deflate.extend_from_slice(&0u16.to_le_bytes());
            deflate.extend_from_slice(&(!0u16).to_le_bytes());
        }

        deflate
    }

    fn gzip_stored(data: &[u8], block_size: usize) -> Vec<u8> {
        let mut gzip = vec![0x1f, 0x8b, 0x08, 0, 0, 0, 0, 0, 0, 0xff];
        gzip.extend_from_slice(&stored_deflate_blocks(data, block_size));

        let mut hasher = Hasher::new();
        hasher.update(data);
        gzip.extend_from_slice(&hasher.finalize().to_le_bytes());
        gzip.extend_from_slice(&(data.len() as u32).to_le_bytes());
        gzip
    }

    fn input_from_bytes<'a>(data: &'a [u8], buf_size: usize) -> DeflateChunkedBufferInput<'a> {
        let mut position = 0;
        DeflateChunkedBufferInput::new(
            move |buf| {
                let count = (data.len() - position).min(buf.len());
                buf[..count].copy_from_slice(&data[position..position + count]);
                position += count;
                count
            },
            buf_size,
        )
    }

    fn consume_output(output: &mut DeflateChunkedBufferOutput<'_>, decoded: &mut Vec<u8>) {
        decoded.extend_from_slice(output.pending_output());
        output.consume_output();
    }

    #[test]
    fn gzip_decompress_returns_more_data_when_output_is_full() {
        let data: Vec<u8> = (0..4096).map(|i| (i % 251) as u8).collect();
        let gzip = gzip_stored(&data, 1024);
        let mut input = input_from_bytes(&gzip, 1024);
        let mut output = DeflateChunkedBufferOutput::new(300);
        let mut tables = libdeflate_alloc_decode_tables();
        let mut decompressor = LibdeflateGzipDecompressor::new();
        let mut decoded = Vec::new();
        let mut more_data_count = 0;

        loop {
            match decompressor
                .decompress(&mut tables, &mut input, &mut output)
                .unwrap()
            {
                LibdeflateDecompressResult::MoreData => {
                    assert!(!output.pending_output().is_empty());
                    more_data_count += 1;
                    consume_output(&mut output, &mut decoded);
                }
                LibdeflateDecompressResult::EndOfStream => break,
            }
        }

        assert!(more_data_count > 1);
        assert_eq!(decoded, data);
    }

    #[test]
    fn deflate_resumes_inside_uncompressed_blocks() {
        let data: Vec<u8> = (0..1500).map(|i| (255 - (i % 251)) as u8).collect();
        let deflate = stored_deflate_blocks(&data, 1500);
        let mut input = input_from_bytes(&deflate, 4096);
        let mut output = DeflateChunkedBufferOutput::new(300);
        let mut tables = libdeflate_alloc_decode_tables();
        let mut decompressor = LibdeflateDeflateDecompressor::new();
        let mut decoded = Vec::new();
        let mut more_data_count = 0;

        loop {
            match decompressor
                .decompress(&mut tables, &mut input, &mut output)
                .unwrap()
            {
                LibdeflateDecompressResult::MoreData => {
                    assert!(!output.pending_output().is_empty());
                    more_data_count += 1;
                    consume_output(&mut output, &mut decoded);
                }
                LibdeflateDecompressResult::EndOfStream => break,
            }
        }

        assert!(more_data_count > 1);
        assert_eq!(decoded, data);
    }

    #[test]
    fn gzip_bad_crc_is_reported_after_pending_output_is_consumed() {
        let data = b"crc validation waits for final output";
        let mut gzip = gzip_stored(data, 1024);
        let crc_start = gzip.len() - 8;
        gzip[crc_start] ^= 0x80;

        let mut input = input_from_bytes(&gzip, 1024);
        let mut output = DeflateChunkedBufferOutput::new(300);
        let mut tables = libdeflate_alloc_decode_tables();
        let mut decompressor = LibdeflateGzipDecompressor::new();
        let mut decoded = Vec::new();
        let mut saw_output = false;

        loop {
            match decompressor.decompress(&mut tables, &mut input, &mut output) {
                Ok(LibdeflateDecompressResult::MoreData) => {
                    saw_output = true;
                    consume_output(&mut output, &mut decoded);
                }
                Ok(LibdeflateDecompressResult::EndOfStream) => {
                    panic!("bad crc unexpectedly reached end of stream");
                }
                Err(LibdeflateError::BadData) => break,
                Err(error) => panic!("unexpected error: {:?}", error),
            }
        }

        assert!(saw_output);
        assert_eq!(decoded, data);
    }

    #[test]
    fn decompress_file_buffered_reads_concatenated_gzip_members() {
        let first = b"first gzip member";
        let second = b"second gzip member";
        let mut gzip = gzip_stored(first, 1024);
        gzip.extend_from_slice(&gzip_stored(second, 1024));

        let path = std::env::temp_dir().join(format!(
            "streaming_libdeflate_concat_{}_{}.gz",
            std::process::id(),
            gzip.len()
        ));
        std::fs::write(&path, &gzip).unwrap();

        let mut decoded = Vec::new();
        let result = decompress_file_buffered(
            &path,
            |data| {
                decoded.extend_from_slice(data);
                Ok(())
            },
            300,
        );
        let _ = std::fs::remove_file(&path);

        result.unwrap();
        assert_eq!(decoded, [first.as_slice(), second.as_slice()].concat());
    }

    #[test]
    fn decompression_speed() {
        let context = Arc::new(AtomicUsize::new(0));

        const PATH: &str = "strains-test";

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
