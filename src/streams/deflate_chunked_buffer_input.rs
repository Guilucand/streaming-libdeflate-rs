use crate::{decompress_utils::copy_word_unaligned, DeflateInput, DeflateOutput};
use nightly_quirks::utils::NightlyUtils;
use std::cmp::min;

pub struct DeflateChunkedBufferInput<'a> {
    buffer: Box<[u8]>,
    buf_size: usize,
    global_position_offset: usize,
    position: usize,
    end_position: usize,
    overread_position_limit: usize,
    func: Box<dyn FnMut(&mut [u8]) -> usize + 'a>,
}

impl<'a> DeflateChunkedBufferInput<'a> {
    pub fn new<F: FnMut(&mut [u8]) -> usize + 'a>(read_func: F, buf_size: usize) -> Self {
        Self {
            buffer: unsafe {
                NightlyUtils::box_new_uninit_slice_assume_init(buf_size + Self::MAX_OVERREAD)
            },
            buf_size,
            global_position_offset: 0,
            position: 0,
            end_position: 0,
            overread_position_limit: 0,
            func: Box::new(read_func),
        }
    }

    #[cold]
    #[inline(never)]
    fn refill_buffer(&mut self) -> bool {
        let keep_buf_len = min(self.position, Self::MAX_LOOK_BACK);
        let move_offset = self.position - keep_buf_len;
        let move_amount = self.end_position - move_offset;

        self.global_position_offset += move_offset;
        unsafe {
            std::ptr::copy(
                self.buffer.as_ptr().add(move_offset),
                self.buffer.as_mut_ptr(),
                move_amount,
            );
        }
        self.position -= move_offset;
        self.end_position -= move_offset;

        let count = (self.func)(&mut self.buffer[self.end_position..self.buf_size]);

        self.end_position += count;

        // Keep at least MAX_OVERREAD bytes available
        self.overread_position_limit = (self.end_position - self.position).max(Self::MAX_OVERREAD)
            + self.position
            - Self::MAX_OVERREAD;

        self.position < self.end_position
    }
}

impl<'a> DeflateInput for DeflateChunkedBufferInput<'a> {
    #[inline(always)]
    unsafe fn get_le_word_no_advance(&mut self) -> usize {
        usize::from_le_bytes(
            *(self.buffer.as_ptr().add(self.position) as *const [u8; std::mem::size_of::<usize>()]),
        )
        .to_le()
    }

    #[inline(always)]
    fn move_stream_pos<const REFILL: bool>(&mut self, amount: isize) {
        const REFILL: bool = true;
        if REFILL && amount > 0 {
            if self.position + amount as usize > self.end_position {
                self.refill_buffer();
            }
        }

        self.position = self.position.wrapping_add_signed(amount);
    }

    fn tell_stream_pos(&self) -> usize {
        self.global_position_offset + self.position
    }

    #[inline(always)]
    fn read<const REFILL: bool>(&mut self, out_data: &mut [u8]) -> usize {
        const REFILL: bool = true;
        if REFILL && self.end_position < self.position + out_data.len() {
            self.refill_buffer();
        }

        let avail_bytes = if REFILL {
            min(out_data.len(), self.end_position - self.position)
        } else {
            out_data.len()
        };

        unsafe {
            std::ptr::copy_nonoverlapping(
                self.buffer.as_ptr().add(self.position),
                out_data.as_mut_ptr(),
                avail_bytes,
            );
            self.position += avail_bytes;
        }
        avail_bytes
    }

    #[inline(always)]
    fn ensure_overread_length(&mut self) {
        if self.position > self.overread_position_limit {
            self.refill_buffer();
        }
    }

    fn has_readable_overread(&self) -> bool {
        self.position <= self.overread_position_limit
    }

    fn has_valid_bytes_slow(&mut self) -> bool {
        if self.position >= self.end_position {
            self.refill_buffer();
        }
        self.position < self.end_position
    }

    #[inline(always)]
    fn read_exact_into<O: DeflateOutput>(&mut self, out_stream: &mut O, mut length: usize) -> bool {
        const CHUNK_SIZE: usize = 256;
        unsafe {
            while length > 0 {
                out_stream.flush_ensure_length(CHUNK_SIZE);
                self.ensure_overread_length();

                let mut src = self.buffer.as_ptr().add(self.position) as *const u64;
                let mut dst = out_stream.get_output_ptr() as *mut u64;

                let max_copyable = length
                    .min(CHUNK_SIZE)
                    .min(self.end_position - self.position);
                let mut copyable = max_copyable as isize;
                while copyable > 0 {
                    copy_word_unaligned(src, dst);
                    copy_word_unaligned(src.add(1), dst.add(1));
                    src = src.add(2);
                    dst = dst.add(2);
                    copyable -= 16;
                }
                length -= max_copyable - copyable.max(0) as usize;

                let mut src = src as *mut u8;
                let mut dst = dst as *mut u8;
                if copyable < 0 {
                    // Remove extra copied bytes
                    src = src.sub(-copyable as usize);
                    dst = dst.sub(-copyable as usize);
                }

                out_stream.set_output_ptr(dst);

                self.position = src.offset_from(self.buffer.as_ptr()) as usize;
            }
        }

        true
    }
}
