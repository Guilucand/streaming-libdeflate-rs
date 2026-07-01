/*
 * gzip_decompress.c - decompress with a gzip wrapper
 *
 * Originally public domain; changes after 2016-09-07 are copyrighted.
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

use crate::decompress_deflate::LibdeflateDeflateDecompressor;
use crate::gzip_constants::*;
use crate::{
    safety_check, DeflateInput, DeflateOutput, LibdeflateDecodeTables, LibdeflateDecompressResult,
    LibdeflateError,
};

// struct flush_buffer_data {
// 	flush_buffer_func *user_func;
// 	void *user_data;
// 	int crc;
// };
//
// static int flush_buffer_checksum(void *data, void *buffer, size_t len) {
// 	struct flush_buffer_data *fdata = (struct flush_buffer_data*)data;
// 	fdata->crc = libdeflate_crc32(fdata->crc, buffer, len);
// 	return fdata->user_func(fdata->user_data, buffer, len);
// }

#[derive(Copy, Clone, PartialEq, Eq)]
enum GzipDecompressStage {
    Header,
    Body,
    DrainOutput,
    Trailer,
    Done,
}

pub struct LibdeflateGzipDecompressor {
    stage: GzipDecompressStage,
    deflate: LibdeflateDeflateDecompressor,
}

impl LibdeflateGzipDecompressor {
    pub fn new() -> Self {
        Self {
            stage: GzipDecompressStage::Header,
            deflate: LibdeflateDeflateDecompressor::new(),
        }
    }

    fn read_header<I: DeflateInput>(in_stream: &mut I) -> Result<(), LibdeflateError> {
        /* ID1 */
        if in_stream.read_byte::<true>() != GZIP_ID1 {
            return Err(LibdeflateError::BadData);
        }
        /* ID2 */
        if in_stream.read_byte::<true>() != GZIP_ID2 {
            return Err(LibdeflateError::BadData);
        }
        /* CM */
        if in_stream.read_byte::<true>() != GZIP_CM_DEFLATE {
            return Err(LibdeflateError::BadData);
        }
        let flg = in_stream.read_byte::<true>();

        /* MTIME */
        in_stream.move_stream_pos::<true>(4);
        safety_check!(in_stream.has_valid_bytes_slow());
        /* XFL */
        in_stream.move_stream_pos::<true>(1);
        safety_check!(in_stream.has_valid_bytes_slow());
        /* OS */
        in_stream.move_stream_pos::<true>(1);
        safety_check!(in_stream.has_valid_bytes_slow());

        if (flg & GZIP_FRESERVED) != 0 {
            return Err(LibdeflateError::BadData);
        }

        /* Extra field */
        if (flg & GZIP_FEXTRA) != 0 {
            let xlen = in_stream.read_le_u16::<true>();
            in_stream.move_stream_pos::<true>(xlen as isize);
            safety_check!(in_stream.has_valid_bytes_slow());
        }

        /* Original file name (zero terminated) */
        if (flg & GZIP_FNAME) != 0 {
            while in_stream.read_byte::<true>() != 0 {}
        }

        /* File comment (zero terminated) */
        if (flg & GZIP_FCOMMENT) != 0 {
            while in_stream.read_byte::<true>() != 0 {}
        }

        /* CRC16 for gzip header */
        if (flg & GZIP_FHCRC) != 0 {
            in_stream.move_stream_pos::<true>(2);
            safety_check!(in_stream.has_valid_bytes_slow());
        }

        Ok(())
    }

    pub fn decompress<I: DeflateInput, O: DeflateOutput>(
        &mut self,
        d: &mut LibdeflateDecodeTables,
        in_stream: &mut I,
        out_stream: &mut O,
    ) -> Result<LibdeflateDecompressResult, LibdeflateError> {
        loop {
            match self.stage {
                GzipDecompressStage::Header => {
                    Self::read_header(in_stream)?;
                    self.stage = GzipDecompressStage::Body;
                }

                GzipDecompressStage::Body => {
                    match self.deflate.decompress(d, in_stream, out_stream)? {
                        LibdeflateDecompressResult::MoreData => {
                            return Ok(LibdeflateDecompressResult::MoreData);
                        }
                        LibdeflateDecompressResult::EndOfStream => {
                            self.stage = GzipDecompressStage::DrainOutput;
                        }
                    }
                }

                GzipDecompressStage::DrainOutput => {
                    if !out_stream.pending_output().is_empty() {
                        return Ok(LibdeflateDecompressResult::MoreData);
                    }
                    self.stage = GzipDecompressStage::Trailer;
                }

                GzipDecompressStage::Trailer => {
                    let result = out_stream
                        .finish_member()
                        .map_err(|_| LibdeflateError::InsufficientSpace)?;

                    let gzip_crc = in_stream.read_le_u32::<true>();
                    if result.crc32 != gzip_crc {
                        return Err(LibdeflateError::BadData);
                    }

                    let expected_written = in_stream.read_le_u32::<true>();
                    if result.written as u32 != expected_written {
                        return Err(LibdeflateError::BadData);
                    }

                    self.stage = GzipDecompressStage::Done;
                    return Ok(LibdeflateDecompressResult::EndOfStream);
                }

                GzipDecompressStage::Done => {
                    return Ok(LibdeflateDecompressResult::EndOfStream);
                }
            }
        }
    }
}

impl Default for LibdeflateGzipDecompressor {
    fn default() -> Self {
        Self::new()
    }
}
