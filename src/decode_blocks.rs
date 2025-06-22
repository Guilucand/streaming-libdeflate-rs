use crate::{
    bitstream::{can_ensure, BitStream},
    decompress_deflate::{LenType, PRECODE_TABLEBITS},
    decompress_utils::{
        build_litlen_decode_table, build_offset_decode_table, build_precode_decode_table,
        DecompressTempData, HUFFDEC_LENGTH_MASK, HUFFDEC_RESULT_SHIFT,
    },
    deflate_constants::{
        DEFLATE_MAX_LENS_OVERRUN, DEFLATE_MAX_PRE_CODEWORD_LEN, DEFLATE_NUM_LITLEN_SYMS,
        DEFLATE_NUM_OFFSET_SYMS, DEFLATE_NUM_PRECODE_SYMS,
    },
    safety_check, DeflateInput, DeflateOutput, LibdeflateDecompressor, LibdeflateError,
};

#[inline(always)]
pub fn decode_huffman_header<I: DeflateInput>(tmp_data: &mut DecompressTempData<I>) {
    const_assert!(can_ensure(1 + 2 + 5 + 5 + 4));
    tmp_data.input_bitstream.ensure_bits(1 + 2 + 5 + 5 + 4);

    // let possible_start = tmp_data.input_bitstream.bit_position();

    /* BFINAL: 1 bit  */
    tmp_data.is_final_block = tmp_data.input_bitstream.pop_bits(1) != 0;

    /* BTYPE: 2 bits  */
    tmp_data.block_type = tmp_data.input_bitstream.pop_bits(2);
}

#[inline(always)]
pub fn decode_dynamic_huffman_block<I: DeflateInput>(
    d: &mut LibdeflateDecompressor,
    tmp_data: &mut DecompressTempData<I>,
) -> Result<(), LibdeflateError> {
    /* The order in which precode lengths are stored.  */
    const DEFLATE_PRECODE_LENS_PERMUTATION: [u8; DEFLATE_NUM_PRECODE_SYMS] = [
        16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15,
    ];

    /* Read the codeword length counts.  */

    const_assert!(DEFLATE_NUM_LITLEN_SYMS == ((1 << 5) - 1) + 257);
    tmp_data.num_litlen_syms = (tmp_data.input_bitstream.pop_bits(5) + 257) as usize;
    // println!("num_litlen_syms: {}", tmp_data.num_litlen_syms);

    const_assert!(DEFLATE_NUM_OFFSET_SYMS == ((1 << 5) - 1) + 1);
    tmp_data.num_offset_syms = (tmp_data.input_bitstream.pop_bits(5) + 1) as usize;
    // println!("num_offset_syms: {}", tmp_data.num_offset_syms);

    const_assert!(DEFLATE_NUM_PRECODE_SYMS == ((1 << 4) - 1) + 4);
    let num_explicit_precode_lens = (tmp_data.input_bitstream.pop_bits(4) + 4) as usize;
    // println!("num_explicit_precode_lens: {}", num_explicit_precode_lens);

    /* Read the precode codeword lengths.  */
    const_assert!(DEFLATE_MAX_PRE_CODEWORD_LEN == (1 << 3) - 1);

    for i in 0..num_explicit_precode_lens {
        const_assert!(can_ensure(3));
        tmp_data.input_bitstream.ensure_bits(3);
        d.precode_lens[DEFLATE_PRECODE_LENS_PERMUTATION[i] as usize] =
            tmp_data.input_bitstream.pop_bits(3) as u8;
        // println!(
        //     "Precode len: {}",
        //     d.precode_lens[DEFLATE_PRECODE_LENS_PERMUTATION[i] as usize]
        // );
    }

    // println!("Precode lens: {:?}", d.precode_lens);

    for i in num_explicit_precode_lens..DEFLATE_NUM_PRECODE_SYMS {
        d.precode_lens[DEFLATE_PRECODE_LENS_PERMUTATION[i] as usize] = 0;
    }

    /* Build the decode table for the precode.  */
    safety_check!(build_precode_decode_table(d));

    /* Expand the literal/length and offset codeword lengths.  */
    let mut i = 0;
    while i < tmp_data.num_litlen_syms + tmp_data.num_offset_syms {
        tmp_data
            .input_bitstream
            .ensure_bits(DEFLATE_MAX_PRE_CODEWORD_LEN + 7);

        /* (The code below assumes that the precode decode table
         * does not have any subtables.)  */
        const_assert!(PRECODE_TABLEBITS == DEFLATE_MAX_PRE_CODEWORD_LEN);

        /* Read the next precode symbol.  */
        let entry = d.l.precode_decode_table
            [tmp_data.input_bitstream.bits(DEFLATE_MAX_PRE_CODEWORD_LEN) as usize];
        tmp_data
            .input_bitstream
            .remove_bits((entry & HUFFDEC_LENGTH_MASK) as usize);
        let presym = entry >> HUFFDEC_RESULT_SHIFT;

        if presym < 16 {
            /* Explicit codeword length  */
            d.l.lens[i] = presym as LenType;
            i += 1;
            continue;
        }

        /* Run-length encoded codeword lengths  */

        /* Note: we don't need verify that the repeat count
         * doesn't overflow the number of elements, since we
         * have enough extra spaces to allow for the worst-case
         * overflow (138 zeroes when only 1 length was
         * remaining).
         *
         * In the case of the small repeat counts (presyms 16
         * and 17), it is fastest to always write the maximum
         * number of entries.  That gets rid of branches that
         * would otherwise be required.
         *
         * It is not just because of the numerical order that
         * our checks go in the order 'presym < 16', 'presym ==
         * 16', and 'presym == 17'.  For typical data this is
         * ordered from most frequent to least frequent case.
         */
        const_assert!(DEFLATE_MAX_LENS_OVERRUN == 138 - 1);

        if presym == 16 {
            /* Repeat the previous length 3 - 6 times  */
            safety_check!(i != 0);
            let rep_val = d.l.lens[i - 1];
            const_assert!(3 + ((1 << 2) - 1) == 6);
            let rep_count = (3 + tmp_data.input_bitstream.pop_bits(2)) as usize;
            d.l.lens[i + 0] = rep_val;
            d.l.lens[i + 1] = rep_val;
            d.l.lens[i + 2] = rep_val;
            d.l.lens[i + 3] = rep_val;
            d.l.lens[i + 4] = rep_val;
            d.l.lens[i + 5] = rep_val;
            i += rep_count;
        } else if presym == 17 {
            /* Repeat zero 3 - 10 times  */
            const_assert!(3 + ((1 << 3) - 1) == 10);
            let rep_count = (3 + tmp_data.input_bitstream.pop_bits(3)) as usize;
            d.l.lens[i + 0] = 0;
            d.l.lens[i + 1] = 0;
            d.l.lens[i + 2] = 0;
            d.l.lens[i + 3] = 0;
            d.l.lens[i + 4] = 0;
            d.l.lens[i + 5] = 0;
            d.l.lens[i + 6] = 0;
            d.l.lens[i + 7] = 0;
            d.l.lens[i + 8] = 0;
            d.l.lens[i + 9] = 0;
            i += rep_count;
        } else {
            /* Repeat zero 11 - 138 times  */
            const_assert!(11 + ((1 << 7) - 1) == 138);
            let rep_count = (11 + tmp_data.input_bitstream.pop_bits(7)) as usize;
            d.l.lens[i..(i + rep_count)].inner_slice_mut().fill(0);
            i += rep_count;
        }
    }

    safety_check!(build_offset_decode_table(
        d,
        tmp_data.num_litlen_syms,
        tmp_data.num_offset_syms,
    ));
    safety_check!(build_litlen_decode_table(
        d,
        tmp_data.num_litlen_syms,
        tmp_data.num_offset_syms,
    ));

    Ok(())
}

#[inline(always)]
pub fn decode_uncompressed_block<I: DeflateInput, C>(
    tmp_data: &mut DecompressTempData<I>,
    context: &mut C,
    uncompressed_cb: fn(&mut C, &mut BitStream<I>, usize) -> Result<(), LibdeflateError>,
) -> Result<(), LibdeflateError> {
    tmp_data.input_bitstream.align_input(4)?;

    let len = unsafe { tmp_data.input_bitstream.read_u16() };
    let nlen = unsafe { tmp_data.input_bitstream.read_u16() };

    safety_check!(len == !nlen);

    uncompressed_cb(context, &mut tmp_data.input_bitstream, len as usize)?;

    // todo!();
    Ok(())
}

pub fn load_static_huffman_block<I: DeflateInput>(
    d: &mut LibdeflateDecompressor,
    tmp_data: &mut DecompressTempData<I>,
) {
    const_assert!(DEFLATE_NUM_LITLEN_SYMS == 288);
    const_assert!(DEFLATE_NUM_OFFSET_SYMS == 32);

    for i in 0..144 {
        d.l.lens[i] = 8;
    }
    for i in 144..256 {
        d.l.lens[i] = 9;
    }
    for i in 256..280 {
        d.l.lens[i] = 7;
    }
    for i in 280..288 {
        d.l.lens[i] = 8;
    }

    for i in 288..(288 + 32) {
        d.l.lens[i] = 5;
    }

    tmp_data.num_litlen_syms = DEFLATE_NUM_LITLEN_SYMS;
    tmp_data.num_offset_syms = DEFLATE_NUM_OFFSET_SYMS;

    // Cannot fail
    let res1 = build_offset_decode_table(d, tmp_data.num_litlen_syms, tmp_data.num_offset_syms);
    debug_assert!(res1);
    let res2 = build_litlen_decode_table(d, tmp_data.num_litlen_syms, tmp_data.num_offset_syms);
    debug_assert!(res2);
}
