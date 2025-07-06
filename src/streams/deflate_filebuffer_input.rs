// use crate::{DeflateInput, DeflateOutput};
// use filebuffer::FileBuffer;
// use std::cmp::min;
// use std::path::Path;

// pub struct DeflateFileBufferInput {
//     file: FileBuffer,
//     position: usize,
// }

// impl DeflateFileBufferInput {
//     pub fn new(path: impl AsRef<Path>) -> Self {
//         Self {
//             file: FileBuffer::open(path).unwrap(),
//             position: 0,
//         }
//     }
// }

// impl DeflateInput for DeflateFileBufferInput {
//     #[inline(always)]
//     unsafe fn get_le_word_no_advance(&mut self) -> usize {
//         usize::from_le_bytes(
//             *(self.file.as_ptr().add(self.position) as *const [u8; std::mem::size_of::<usize>()]),
//         )
//     }

//     #[inline(always)]
//     fn move_stream_pos<const REFILL: bool>(&mut self, amount: isize) -> bool {
//         self.position = self.position.wrapping_add_signed(amount);
//         self.position < self.file.len()
//     }

//     fn tell_stream_pos(&self) -> usize {
//         self.position
//     }

//     #[inline(always)]
//     fn read<const REFILL: bool>(&mut self, out_data: &mut [u8]) -> usize {
//         let avail_bytes = if REFILL {
//             min(out_data.len(), self.file.len() - self.position)
//         } else {
//             out_data.len()
//         };

//         unsafe {
//             std::ptr::copy_nonoverlapping(
//                 self.file.as_ptr().add(self.position),
//                 out_data.as_mut_ptr(),
//                 avail_bytes,
//             );
//         }
//         self.position += avail_bytes;
//         // avail_bytes
//         unimplemented!("Fix the overread case")
//     }

//     #[inline(always)]
//     fn ensure_overread_length(&mut self, len: usize) -> bool {
//         self.position + len <= self.file.len()
//     }

//     fn has_available_length(&mut self, len: usize) -> bool {
//         self.position + len <= self.file.len()
//     }

//     #[inline(always)]
//     fn read_exact_into<O: DeflateOutput>(&mut self, out_stream: &mut O, mut length: usize) -> bool {
//         while length > 0 {
//             let buffer = out_stream.get_available_buffer();
//             let copyable = min(buffer.len(), length);
//             if self.read::<true>(&mut buffer[0..copyable]) != copyable {
//                 return false;
//             }
//             unsafe {
//                 out_stream.advance_available_buffer_position(copyable);
//             }
//             length -= copyable;
//         }
//         true
//     }
// }
