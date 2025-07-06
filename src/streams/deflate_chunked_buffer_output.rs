use crate::{DeflateOutput, OutStreamResult};
use crc32fast::Hasher;
use nightly_quirks::utils::NightlyUtils;

pub struct DeflateChunkedBufferOutput<'a> {
    buffer: Box<[u8]>,
    last_usable_ptr: *mut u8,
    current_ptr: *mut u8,
    crc32: Hasher,
    written: usize,
    func: Box<dyn FnMut(&[u8]) -> Result<(), ()> + 'a>,
}

impl<'a> DeflateChunkedBufferOutput<'a> {
    pub fn new<F: FnMut(&[u8]) -> Result<(), ()> + 'a>(write_func: F, buf_size: usize) -> Self {
        unsafe {
            let mut buffer = NightlyUtils::box_new_uninit_slice_assume_init(
                buf_size + Self::MAX_LOOK_BACK + Self::OVERWRITE_MAX,
            );

            let buffer_start = buffer.as_mut_ptr();

            Self {
                buffer,
                last_usable_ptr: buffer_start.add(buf_size + Self::MAX_LOOK_BACK),
                current_ptr: buffer_start.add(Self::MAX_LOOK_BACK),
                crc32: Hasher::new(),
                written: 0,
                func: Box::new(write_func),
            }
        }
    }

    fn flush_buffer(&mut self) -> bool {
        let last_index = unsafe { self.current_ptr.offset_from(self.buffer.as_ptr()) } as usize;

        self.crc32
            .update(&self.buffer[Self::MAX_LOOK_BACK..last_index]);
        if (self.func)(&self.buffer[Self::MAX_LOOK_BACK..last_index]).is_err() {
            return false;
        }
        self.written += last_index - Self::MAX_LOOK_BACK;

        unsafe {
            std::ptr::copy(
                self.buffer.as_ptr().add(last_index - Self::MAX_LOOK_BACK),
                self.buffer.as_mut_ptr(),
                Self::MAX_LOOK_BACK,
            );
        }
        self.current_ptr = unsafe { self.buffer.as_mut_ptr().add(Self::MAX_LOOK_BACK) };
        true
    }
}

impl<'a> DeflateOutput for DeflateChunkedBufferOutput<'a> {
    #[inline(always)]
    fn has_writable_length(&mut self, length: usize) -> bool {
        unsafe { self.current_ptr.add(length) <= self.last_usable_ptr }
    }

    fn flush_ensure_length(&mut self, length: usize) -> bool {
        if !self.has_writable_length(length) {
            if !self.flush_buffer() {
                return false;
            }
        }
        true
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

    #[inline(always)]
    fn final_flush(&mut self) -> Result<OutStreamResult, ()> {
        self.flush_buffer();
        self.current_ptr = unsafe { self.buffer.as_mut_ptr().add(Self::MAX_LOOK_BACK) };

        let result = OutStreamResult {
            written: self.written,
            crc32: self.crc32.clone().finalize(),
        };

        self.crc32 = Hasher::new();
        self.written = 0;
        Ok(result)
    }
}
