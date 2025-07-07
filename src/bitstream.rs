use crate::{
    streams::deflate_chunked_buffer_input::DeflateChunkedBufferInput, DeflateInput, DeflateOutput,
    LibdeflateError,
};

type BitBufType = usize;

pub struct BitStream<'a, I: DeflateInput = DeflateChunkedBufferInput<'a>> {
    bitbuf: BitBufType,
    pub bitsleft: usize,
    pub input_stream: &'a mut I,
}

/*
 * Number of bits the bitbuffer variable can hold.
 *
 * This is one less than the obvious value because of the optimized arithmetic
 * in FILL_BITS_WORDWISE() that leaves 'bitsleft' in the range
 * [WORDBITS - 8, WORDBITS - 1] rather than [WORDBITS - 7, WORDBITS].
 */
const BITBUF_MAXBITS: usize = 8 * std::mem::size_of::<BitBufType>() - 1;

/*
 * Evaluates to true if 'n' is a valid argument to ENSURE_BITS(n), or false if
 * 'n' is too large to be passed to ENSURE_BITS(n).  Note: if 'n' is a compile
 * time constant, then this expression will be a compile-type constant.
 * Therefore, CAN_ENSURE() can be used choose between alternative
 * implementations at compile time.
 */
#[inline(always)]
pub const fn can_ensure(n: usize) -> bool {
    n <= MAX_ENSURE
}

/*
 * The maximum number of bits that can be ensured in the bitbuffer variable,
 * i.e. the maximum value of 'n' that can be passed ENSURE_BITS(n).  The decoder
 * only reads whole bytes from memory, so this is the lowest value of 'bitsleft'
 * at which another byte cannot be read without first consuming some bits.
 */
pub const MAX_ENSURE: usize = BITBUF_MAXBITS - 7;

impl<'a, I: DeflateInput> BitStream<'a, I> {
    pub fn new(input_stream: &'a mut I) -> Self {
        Self {
            bitbuf: 0,
            bitsleft: 0,
            input_stream,
        }
    }

    #[inline(always)]
    pub fn bit_position(&self) -> usize {
        self.input_stream.tell_stream_pos() * 8 - (self.bitsleft as usize)
    }

    /*
     * Fill the bitbuffer variable by reading the next word from the input buffer
     * and branchlessly updating 'in_next' and 'bitsleft' based on how many bits
     * were filled.  This can be significantly faster than FILL_BITS_BYTEWISE().
     * However, for this to work correctly, the word must be interpreted in
     * little-endian format.  In addition, the memory access may be unaligned.
     * Therefore, this method is most efficient on little-endian architectures that
     * support fast unaligned access, such as x86 and x86_64.
     *
     * For faster updating of 'bitsleft', we consider the bitbuffer size in bits to
     * be 1 less than the word size and therefore be all 1 bits.  Then the number of
     * bits filled is the value of the 0 bits in position >= 3 when changed to 1.
     * E.g. if words are 64 bits and bitsleft = 16 = b010000 then we refill b101000
     * = 40 bits = 5 bytes.  This uses only 4 operations to update 'in_next' and
     * 'bitsleft': one each of +, ^, >>, and |.  (Not counting operations the
     * compiler optimizes out.)  In contrast, the alternative of:
     *
     *	in_next += (BITBUF_NBITS - bitsleft) >> 3;
     *	bitsleft += (BITBUF_NBITS - bitsleft) & ~7;
     *
     * (where BITBUF_NBITS would be WORDBITS rather than WORDBITS - 1) would on
     * average refill an extra bit, but uses 5 operations: two +, and one each of
     * -, >>, and &.  Also the - and & must be completed before 'bitsleft' can be
     * updated, while the current solution updates 'bitsleft' with no dependencies.
     */
    #[inline(always)]
    unsafe fn fill_bits_wordwise(&mut self) {
        self.bitbuf |= self.input_stream.get_le_word_no_advance() << (self.bitsleft as u8);
        let in_next = self.input_stream.get_stream_pos_mut();
        *in_next += size_of::<BitBufType>() - 1;
        *in_next -= (self.bitsleft >> 3) & 0x7;
        self.bitsleft |= BITBUF_MAXBITS & !7;
    }

    /*
     * Does the bitbuffer variable currently contain at least 'n' bits?
     */
    #[inline(always)]
    fn have_bits(&self, n: usize) -> bool {
        self.bitsleft >= n
    }

    /*
     * Load more bits from the input buffer until the specified number of bits is
     * present in the bitbuffer variable.  'n' cannot be too large; see MAX_ENSURE
     * and CAN_ENSURE().
     */
    #[inline(always)]
    pub fn ensure_bits<const REFILL: bool>(&mut self, n: usize) {
        if REFILL {
            self.input_stream.ensure_overread_length();
        }

        if !self.have_bits(n) {
            unsafe {
                self.fill_bits_wordwise();
            }
        }
    }

    #[inline(always)]
    pub fn force_ensure_bits_refill(&mut self) {
        self.input_stream.ensure_overread_length();
        unsafe {
            self.fill_bits_wordwise();
        }
    }

    #[inline(always)]
    pub fn force_ensure_bits(&mut self) {
        unsafe {
            self.fill_bits_wordwise();
        }
    }

    /*
     * Return the next 'n' bits from the bitbuffer variable without removing them.
     */
    #[inline(always)]
    pub fn bits(&self, n: usize) -> u32 {
        (self.bitbuf as u32) & ((1u32 << (n)) - 1)
    }

    /*
     * Hide the next 'o' bits and return the next 'n' bits from the bitbuffer variable without removing them.
     */
    #[inline(always)]
    pub fn bits_with_offset(&self, o: u8, n: u8) -> u32 {
        (self.bitbuf as u32 >> o) & ((1u32 << (n)) - 1)
    }

    /*
     * Remove the next 'n' bits from the bitbuffer variable.
     */
    #[inline(always)]
    pub fn remove_bits(&mut self, n: usize) {
        self.bitbuf >>= n;
        self.bitsleft -= n
    }

    /*
     * Remove and return the next 'n' bits from the bitbuffer variable.
     */
    #[inline(always)]
    pub fn pop_bits(&mut self, n: usize) -> u32 {
        let tmp = self.bits(n);
        self.remove_bits(n);
        tmp
    }

    /*
     * Verify that the input buffer hasn't been overread, then align the input to
     * the next byte boundary, discarding any remaining bits in the current byte.
     *
     * Note that if the bitbuffer variable currently contains more than 7 bits, then
     * we must rewind 'in_next', effectively putting those bits back.  Only the bits
     * in what would be the "current" byte if we were reading one byte at a time can
     * be actually discarded.
     */
    #[inline(always)]
    pub fn align_input(&mut self) -> Result<(), LibdeflateError> {
        let bitsleft = self.bitsleft as u8;

        self.input_stream
            .move_stream_pos::<true>(-((bitsleft >> 3) as isize));
        self.bitbuf = 0;
        self.bitsleft = 0;

        self.input_stream.ensure_overread_length();

        Ok(())
    }

    /*
     * Read a 16-bit value from the input.  This must have been preceded by a call
     * to ALIGN_INPUT(), and the caller must have already checked for overrun.
     */
    #[inline(always)]
    pub unsafe fn read_u16(&mut self) -> u16 {
        let mut bytes = [0, 0];
        self.input_stream.read::<false>(&mut bytes);
        u16::from_le_bytes(bytes)
    }

    #[inline(always)]
    pub fn read_exact_into<O: DeflateOutput>(&mut self, out_stream: &mut O, length: usize) -> bool {
        self.input_stream.read_exact_into(out_stream, length)
    }

    // #[inline(always)]
    // pub fn has_overrun(&self) -> bool {
    //     self.overrun_count >= (size_of::<usize>() / 8)
    // }
}

#[cfg(test)]
mod tests {
    use crate::{bitstream::BitStream, DeflateInput};

    struct ByteStream {
        data: Vec<u8>,
        position: usize,
    }

    impl DeflateInput for ByteStream {
        unsafe fn get_le_word_no_advance(&mut self) -> usize {
            self.data
                .as_ptr()
                .add(self.position)
                .cast::<usize>()
                .read_unaligned()
        }

        fn move_stream_pos<const REFILL: bool>(&mut self, amount: isize) -> bool {
            self.position = self.position.wrapping_add_signed(amount);
            self.position < self.data.len()
        }

        fn tell_stream_pos(&self) -> usize {
            self.position
        }

        fn read<const REFILL: bool>(&mut self, _out_data: &mut [u8]) -> usize {
            unimplemented!()
        }

        fn has_readable_overread(&self) -> bool {
            true
        }

        fn ensure_overread_length(&mut self) {}

        fn read_exact_into<O: crate::DeflateOutput>(
            &mut self,
            out_stream: &mut O,
            length: usize,
        ) -> bool {
            unimplemented!()
        }
    }

    #[test]
    fn test_bitstream() {
        let mut input = ByteStream {
            data: vec![0x12, 0x34, 0x56, 0x78, 0x01, 0x23, 0x45, 0x67, 0x89],
            position: 0,
        };
        input.data.reserve(8);
        let mut bitstream = BitStream::new(&mut input);
        bitstream.ensure_bits::<true>(16);
        println!("Bit value: {:x}", bitstream.bitbuf);
        println!("Bits left: {}", bitstream.bitsleft);
        println!("16 bits: {:x}", bitstream.bits(16));
        println!(
            "START Bytes position: {}",
            bitstream.input_stream.tell_stream_pos()
        );

        bitstream.remove_bits(7);
        bitstream.ensure_bits::<true>(16);
        bitstream.remove_bits(9);
        bitstream.ensure_bits::<true>(16);

        println!(
            "Bytes position: {}",
            bitstream.input_stream.tell_stream_pos()
        );
        println!("Position: {}", bitstream.bit_position());

        assert_eq!(bitstream.bits(12), 0x856);
        // assert_eq!(bitstream.bits(8), 0x34);
    }
}
