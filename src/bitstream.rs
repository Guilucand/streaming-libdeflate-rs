use nightly_quirks::branch_pred::likely;

use crate::{
    safety_check, streams::deflate_chunked_buffer_input::DeflateChunkedBufferInput, DeflateInput,
    DeflateOutput, LibdeflateError,
};

type BitBufType = usize;

pub struct BitStream<'a, I: DeflateInput = DeflateChunkedBufferInput<'a>> {
    bitbuf: BitBufType,
    bitsleft: usize,
    overrun_count: usize,
    input_stream: &'a mut I,
}

/*
 * Number of bits the bitbuffer variable can hold.
 *
 * This is one less than the obvious value because of the optimized arithmetic
 * in FILL_BITS_WORDWISE() that leaves 'bitsleft' in the range
 * [WORDBITS - 8, WORDBITS - 1] rather than [WORDBITS - 7, WORDBITS].
 */
const BITBUF_NBITS: usize = 8 * std::mem::size_of::<BitBufType>() - 1;

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
pub const MAX_ENSURE: usize = BITBUF_NBITS - 7;

impl<'a, I: DeflateInput> BitStream<'a, I> {
    pub fn new(input_stream: &'a mut I) -> Self {
        Self {
            bitbuf: 0,
            bitsleft: 0,
            overrun_count: 0,
            input_stream,
        }
    }

    #[inline(always)]
    pub fn bit_position(&self) -> usize {
        self.input_stream.tell_stream_pos() * 8 + (self.bitsleft as usize)
    }

    /*
     * Fill the bitbuffer variable, reading one byte at a time.
     *
     * If we would overread the input buffer, we just don't read anything, leaving
     * the bits zeroed but marking them filled.  This simplifies the decompressor
     * because it removes the need to distinguish between real overreads and
     * overreads that occur only because of the decompressor's own lookahead.
     *
     * The disadvantage is that real overreads are not detected immediately.
     * However, this is safe because the decompressor is still guaranteed to make
     * forward progress when presented never-ending 0 bits.  In an existing block
     * output will be getting generated, whereas new blocks can only be uncompressed
     * (since the type code for uncompressed blocks is 0), for which we check for
     * previous overread.  But even if we didn't check, uncompressed blocks would
     * fail to validate because LEN would not equal ~NLEN.  So the decompressor will
     * eventually either detect that the output buffer is full, or detect invalid
     * input, or finish the final block.
     */
    #[inline(always)]
    fn fill_bits_bytewise(&mut self) {
        loop {
            if likely(self.input_stream.ensure_length(1)) {
                let mut byte = [0];
                unsafe {
                    self.input_stream.read_unchecked(&mut byte);
                }
                self.bitbuf |= (byte[0] as BitBufType) << self.bitsleft;
            } else {
                self.overrun_count += 1;
            }
            self.bitsleft += 8;
            if self.bitsleft > BITBUF_NBITS - 8 {
                break;
            }
        }
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
        /* BITBUF_NBITS must be all 1's in binary, see above */
        // const_assert!((BITBUF_NBITS & (BITBUF_NBITS + 1)) == 0);

        self.bitbuf |= self.input_stream.get_le_word_no_advance() << self.bitsleft;
        self.input_stream
            .move_stream_pos(((self.bitsleft ^ BITBUF_NBITS) >> 3) as isize);
        self.bitsleft |= BITBUF_NBITS & !7;
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
    pub fn ensure_bits(&mut self, n: usize) {
        if !self.have_bits(n) {
            if cfg!(target_endian = "little")
                && likely(
                    self.input_stream
                        .ensure_length(std::mem::size_of::<BitBufType>()),
                )
            {
                unsafe {
                    self.fill_bits_wordwise();
                }
            } else {
                self.fill_bits_bytewise();
            }
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
    pub fn align_input(&mut self, ensure_length: usize) -> Result<(), LibdeflateError> {
        safety_check!(self.overrun_count <= (self.bitsleft >> 3));
        self.input_stream
            .move_stream_pos(-(((self.bitsleft >> 3) - self.overrun_count) as isize));
        self.overrun_count = 0;
        self.bitbuf = 0;
        self.bitsleft = 0;

        if ensure_length > 0 && !self.input_stream.ensure_length(ensure_length) {
            return Err(LibdeflateError::InsufficientSpace);
        }

        Ok(())
    }

    /*
     * Read a 16-bit value from the input.  This must have been preceded by a call
     * to ALIGN_INPUT(), and the caller must have already checked for overrun.
     */
    #[inline(always)]
    pub unsafe fn read_u16(&mut self) -> u16 {
        let mut bytes = [0, 0];
        self.input_stream.read_unchecked(&mut bytes);
        u16::from_le_bytes(bytes)
    }

    #[inline(always)]
    pub fn read_exact_into<O: DeflateOutput>(&mut self, out_stream: &mut O, length: usize) -> bool {
        self.input_stream.read_exact_into(out_stream, length)
    }

    #[inline(always)]
    pub fn has_overrun(&self) -> bool {
        self.overrun_count >= (size_of::<usize>() / 8)
    }
}
