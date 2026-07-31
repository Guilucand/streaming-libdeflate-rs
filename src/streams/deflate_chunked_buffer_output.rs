use crate::{DeflateOutput, OutStreamResult};
use crc32fast::Hasher;
use maligned::{align_first_boxed_default, A64};

pub struct DeflateChunkedBufferOutput<'a> {
    buffer: Box<[u8]>,
    last_usable_ptr: *mut u8,
    current_ptr: *mut u8,
    crc32: Hasher,
    written: usize,
    _marker: std::marker::PhantomData<&'a mut ()>,
}

impl<'a> DeflateChunkedBufferOutput<'a> {
    pub fn new(buf_size: usize) -> Self {
        unsafe {
            let mut buffer = align_first_boxed_default::<_, A64>(
                buf_size + Self::MAX_LOOK_BACK + Self::OVERWRITE_MAX,
            );

            let buffer_start = buffer.as_mut_ptr();

            Self {
                buffer,
                last_usable_ptr: buffer_start.add(buf_size + Self::MAX_LOOK_BACK),
                current_ptr: buffer_start.add(Self::MAX_LOOK_BACK),
                crc32: Hasher::new(),
                written: 0,
                _marker: std::marker::PhantomData,
            }
        }
    }

    fn pending_end_index(&self) -> usize {
        (unsafe { self.current_ptr.offset_from(self.buffer.as_ptr()) }) as usize
    }

    fn reset_output_ptr(&mut self) {
        self.current_ptr = unsafe { self.buffer.as_mut_ptr().add(Self::MAX_LOOK_BACK) };
    }

    fn consume_buffer(&mut self, consumed_offset: usize) {
        let last_index = unsafe { self.current_ptr.offset_from(self.buffer.as_ptr()) } as usize;
        if last_index == Self::MAX_LOOK_BACK {
            return;
        }

        let consumed_index = Self::MAX_LOOK_BACK + consumed_offset;

        self.crc32
            .update(&self.buffer[Self::MAX_LOOK_BACK..consumed_index]);
        self.written += consumed_index - Self::MAX_LOOK_BACK;

        let remaining_bytes = last_index - consumed_index;

        unsafe {
            std::ptr::copy(
                self.buffer
                    .as_ptr()
                    .add(consumed_index - Self::MAX_LOOK_BACK),
                self.buffer.as_mut_ptr(),
                Self::MAX_LOOK_BACK + remaining_bytes,
            );
            self.current_ptr = self
                .buffer
                .as_mut_ptr()
                .add(Self::MAX_LOOK_BACK + remaining_bytes);
        }
    }
}

impl<'a> DeflateOutput for DeflateChunkedBufferOutput<'a> {
    #[inline(always)]
    fn has_writable_length(&mut self, length: usize) -> bool {
        unsafe { self.current_ptr.add(length) <= self.last_usable_ptr }
    }

    #[inline(always)]
    fn get_output_ptr(&mut self) -> *mut u8 {
        self.current_ptr
    }

    #[inline(always)]
    unsafe fn set_output_ptr(&mut self, ptr: *mut u8) {
        self.current_ptr = ptr;
    }

    // #[inline(always)]
    // fn copy_forward(&mut self, prev_offset: usize, length: usize) -> bool {
    //     if self.buffer.len() - self.position <= length {
    //         if !self.flush_buffer(length) {
    //             return false;
    //         }
    //     }

    //     if prev_offset > self.position {
    //         return false;
    //     }

    //     unsafe {
    //         let dest = self.buffer.as_mut_ptr().add(self.position);
    //         copy_rolling(
    //             dest,
    //             dest.add(length),
    //             prev_offset,
    //             self.get_available_buffer().len() >= (length + 3 * size_of::<usize>()),
    //         );
    //     }
    //     self.position += length;

    //     true
    // }

    fn pending_output(&self) -> &[u8] {
        &self.buffer[Self::MAX_LOOK_BACK..self.pending_end_index()]
    }

    fn consume_output(&mut self, consumed_offset: usize) {
        self.consume_buffer(consumed_offset);
    }

    fn increase_buffer_size(&mut self, additional: usize) {
        let padded_additional = additional.next_multiple_of(64);
        let mut new_buffer =
            align_first_boxed_default::<u8, A64>(self.buffer.len() + padded_additional);
        new_buffer[..self.pending_end_index()]
            .copy_from_slice(&self.buffer[..self.pending_end_index()]);
        unsafe {
            self.current_ptr = new_buffer
                .as_mut_ptr()
                .offset(self.current_ptr.offset_from(self.buffer.as_ptr()));
            self.last_usable_ptr = new_buffer
                .as_mut_ptr()
                .add(new_buffer.len() - Self::OVERWRITE_MAX);
        }
        self.buffer = new_buffer;
    }

    #[inline(always)]
    fn finish_member(&mut self) -> Result<OutStreamResult, ()> {
        if !self.pending_output().is_empty() {
            return Err(());
        }

        let result = OutStreamResult {
            written: self.written,
            crc32: self.crc32.clone().finalize(),
        };

        self.crc32 = Hasher::new();
        self.written = 0;
        self.reset_output_ptr();
        Ok(result)
    }
}
