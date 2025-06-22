use crate::deflate_constants::{DEFLATE_MAX_PRE_CODEWORD_LEN, DEFLATE_NUM_PRECODE_SYMS};

#[derive(Clone)]
pub struct CodepointChecker {
    exp_lens: [u128; 2],
}

const BITS_PER_EXP_LEN: usize = 12;

impl CodepointChecker {
    const EXP_LEN_MASK: u128 = (1u128 << BITS_PER_EXP_LEN) - 1;

    fn value_from_len(len: u8) -> u128 {
        // len => value
        // 0 => 0
        // 1 => 64
        // 2 => 32
        // 3 => 16
        // 4 => 8
        // 5 => 4
        // 6 => 2
        // 7 => 1

        let value = (128u8 >> len) & (-((len != 0) as i8)) as u8;
        value as u128
    }

    pub fn empty() -> Self {
        Self {
            exp_lens: [0u128; 2],
        }
    }

    // pub fn new(lens: [LenType; DEFLATE_NUM_PRECODE_SYMS]) -> Self {
    //     let mut exp_lens = [0u128; 2];

    //     for i in 0..10 {
    //         exp_lens[0] += Self::value_from_len(lens[i]) << (i * BITS_PER_EXP_LEN);
    //     }
    //     for i in 10..DEFLATE_NUM_PRECODE_SYMS {
    //         exp_lens[1] += Self::value_from_len(lens[i]) << ((i - 10) * BITS_PER_EXP_LEN);
    //     }

    //     Self { exp_lens }
    // }

    fn debug_values(name: &str, mut values: u128, count: usize) {
        let mut expanded = vec![];

        for i in 0..count {
            expanded.push(values & Self::EXP_LEN_MASK);
            values >>= BITS_PER_EXP_LEN;
        }

        println!(
            "Values {}: {:?} sum: {}",
            name,
            expanded,
            expanded.iter().sum::<u128>()
        );
    }

    #[inline(always)]
    pub fn roll_lens(&mut self, new: u8, count: u8) -> u16 {
        // Roll the exp_lens by one
        self.exp_lens[0] = (self.exp_lens[0] >> BITS_PER_EXP_LEN)
            | ((self.exp_lens[1] & Self::EXP_LEN_MASK) << (BITS_PER_EXP_LEN * 9));
        self.exp_lens[1] = (self.exp_lens[1] >> BITS_PER_EXP_LEN)
            | (Self::value_from_len(new) << (BITS_PER_EXP_LEN * 8));

        // Self::debug_values("el0au", self.exp_lens[0], 10);
        // Self::debug_values("el1au", self.exp_lens[1], 9);

        let mask0 = u128::MAX >> (128 - count.min(10) as usize * BITS_PER_EXP_LEN);
        let mask1 = (u128::MAX / (1 << BITS_PER_EXP_LEN))
            >> ((128 - BITS_PER_EXP_LEN) - (count.max(10) - 10) as usize * BITS_PER_EXP_LEN);

        // Self::debug_values("el0au_masked", self.exp_lens[0] & mask0, 10);
        // Self::debug_values("el1au_masked", self.exp_lens[1] & mask1, 9);

        // 120 bits (10 values)
        let result = (self.exp_lens[0] & mask0) + (self.exp_lens[1] & mask1);

        // Self::debug_values("result120", result, 10);

        // 60 bits (5 values)
        let result = (result >> BITS_PER_EXP_LEN * 5) as u64 + result as u64;

        // Self::debug_values("result60", result as u128, 5);

        const fn generate_mask(bits: usize) -> u64 {
            (1u64 << bits) - 1
        }

        const MASK_36: u64 = generate_mask(BITS_PER_EXP_LEN * 2);

        // 36 bits (3 values)
        let result = ((result >> BITS_PER_EXP_LEN * 3) & MASK_36) + result;

        // Self::debug_values("result36", result as u128, 3);

        const MASK_24: u64 = generate_mask(BITS_PER_EXP_LEN * 1);

        // 24 bits (2 values)
        let result = ((result >> BITS_PER_EXP_LEN * 2) & MASK_24) as u32 + result as u32;

        // Self::debug_values("result24", result as u128, 2);

        let result = ((result >> BITS_PER_EXP_LEN) + result) as u16 & ((1 << BITS_PER_EXP_LEN) - 1);
        // println!("Final sum: {}", result);
        result
    }
}

pub struct BlockFinder {
    data: u128,
    codepoint_checkers: [CodepointChecker; 3],
}

const LITLEN_OFFSET: usize = 17;
const LAST_LITLEN_OFFSET: usize = LITLEN_OFFSET + (DEFLATE_NUM_PRECODE_SYMS - 1) * 3;
const CODEWORD_MAX_SPACE: u16 = 1 << DEFLATE_MAX_PRE_CODEWORD_LEN as u16;

impl BlockFinder {
    #[inline(always)]
    fn process_position(&mut self, pos: usize) -> Option<usize> {
        let len0 = (self.data >> (0 + pos)) & 0b111; // 3 bits
        let len1 = (self.data >> (1 + pos)) & 0b111; // 3 bits
        let len2 = (self.data >> (2 + pos)) & 0b111; // 3 bits

        let mut result = 0;

        // println!("Precode len0: {}", len0);
        // println!("Precode len1: {}", len1);
        // println!("Precode len2: {}", len2);

        let precode_lengths0 = (self.data >> (pos + 0 - LAST_LITLEN_OFFSET + (3 + 5 + 5))) & 0b1111;
        result |= ((self.codepoint_checkers[0].roll_lens(len0 as u8, 4 + precode_lengths0 as u8)
            == CODEWORD_MAX_SPACE) as u8)
            << 0;
        let precode_lengths1 = (self.data >> (pos + 1 - LAST_LITLEN_OFFSET + (3 + 5 + 5))) & 0b1111;
        result |= ((self.codepoint_checkers[1].roll_lens(len1 as u8, 4 + precode_lengths1 as u8)
            == CODEWORD_MAX_SPACE) as u8)
            << 1;
        let precode_lengths2 = (self.data >> (pos + 2 - LAST_LITLEN_OFFSET + (3 + 5 + 5))) & 0b1111;
        result |= ((self.codepoint_checkers[2].roll_lens(len2 as u8, 4 + precode_lengths2 as u8)
            == CODEWORD_MAX_SPACE) as u8)
            << 2;
        if result != 0 {
            let offset = result.trailing_zeros() as usize;
            let start_position = pos + offset - LAST_LITLEN_OFFSET;
            // println!("Found here raw pos: {}!", start_position);

            // pop 3 bits asserting that  they're 010
            // let shift = 0;
            let block_type_and_final = (self.data >> start_position) & 0b111;

            // println!("Block type and final: {}", block_type_and_final);

            if block_type_and_final != 0b100 {
                // println!(
                //     "Invalid block type and final bits: {}",
                //     block_type_and_final
                // );
                return None;
            }

            // // trash 5 + 5 bits
            // let unused_bits = (init_data >> (3 + shift)) & 0b1111111111;

            // // const_assert!(DEFLATE_NUM_LITLEN_SYMS == ((1 << 5) - 1) + 257);
            // // tmp_data.num_litlen_syms = (pop_bits(&mut tmp_data, 5) + 257) as usize;

            // // const_assert!(DEFLATE_NUM_OFFSET_SYMS == ((1 << 5) - 1) + 1);
            // // tmp_data.num_offset_syms = (pop_bits(&mut tmp_data, 5) + 1) as usize;

            // // get precode_len
            // // const_assert!(DEFLATE_NUM_PRECODE_SYMS == ((1 << 4) - 1) + 4);
            // // let num_explicit_precode_lens = (pop_bits(&mut tmp_data, 4) + 4) as usize;
            // let num_explicit_precode_lens = (init_data >> (13 + shift)) & 0b1111; // 4 bits

            Some(start_position)
        } else {
            None
        }
    }

    pub fn new(init_data: u128) -> Result<usize, Self> {
        let mut self_ = Self {
            codepoint_checkers: [
                CodepointChecker::empty(),
                CodepointChecker::empty(),
                CodepointChecker::empty(),
            ],
            data: init_data,
        };

        let is_finished = init_data & 0x1;
        // println!("Is finished: {}", is_finished);

        let block_type = (init_data >> 1) & 0b11; // 2 bits
                                                  // println!("Block type: {}", block_type);

        for i in 0..(DEFLATE_NUM_PRECODE_SYMS - 1) {
            // println!("Offset: {}", LITLEN_OFFSET + 0 + i * 3);
            let len0 = (init_data >> (LITLEN_OFFSET + 0 + i * 3)) & 0b111; // 3 bits
            let len1 = (init_data >> (LITLEN_OFFSET + 1 + i * 3)) & 0b111; // 3 bits
            let len2 = (init_data >> (LITLEN_OFFSET + 2 + i * 3)) & 0b111; // 3 bits
                                                                           // println!("Precode len0: {}", len0);
            self_.codepoint_checkers[0].roll_lens(len0 as u8, DEFLATE_NUM_PRECODE_SYMS as u8);
            self_.codepoint_checkers[1].roll_lens(len1 as u8, DEFLATE_NUM_PRECODE_SYMS as u8);
            self_.codepoint_checkers[2].roll_lens(len2 as u8, DEFLATE_NUM_PRECODE_SYMS as u8);
        }

        // Process until the bit 122

        for i in (LAST_LITLEN_OFFSET..=122).step_by(3) {
            let result = self_.process_position(i);
            if let Some(position) = result {
                return Ok(position);
            }
        }

        // After processing bit 125 should become bit 71, bit 74 is the first one available to be rewritten
        self_.data >>= 125 - LAST_LITLEN_OFFSET;

        Err(self_)
    }

    pub fn add_data(&mut self, data: [u32; 3]) -> Result<Vec<isize>, ()> {
        // Bit 74 is the first available, copy data to [74..106]
        self.data |= (data[0] as u128) << 74;

        let mut position_results = vec![];

        // Process 11 iterations, first not yet processed is bit 71
        for i in (LAST_LITLEN_OFFSET..=101).step_by(3) {
            let result = self.process_position(i);
            if let Some(position) = result {
                // println!("Found here raw pos: {}!", position);
                position_results.push(position as isize - 74);
            }
        }

        // Now make position 104 be position LAST_LITLEN_OFFSET
        self.data >>= 104 - LAST_LITLEN_OFFSET;

        // Copy the second word to [73..105]
        self.data |= (data[1] as u128) << 73;

        // Process 11 iterations, first not yet processed is always bit 71
        for i in (LAST_LITLEN_OFFSET..=101).step_by(3) {
            let result = self.process_position(i);
            if let Some(position) = result {
                // println!("Found here raw pos2: {}!", position);
                position_results.push(position as isize + 32 - 73);
            }
        }

        // Now make position 104 be position LAST_LITLEN_OFFSET
        self.data >>= 104 - LAST_LITLEN_OFFSET;

        // Copy the third word to [72..104]
        self.data |= (data[2] as u128) << 72;

        // Finally process 10 iterations, first not yet processed is always bit 71
        for i in (LAST_LITLEN_OFFSET..=98).step_by(3) {
            let result = self.process_position(i);
            if let Some(position) = result {
                // println!("Found here raw pos3: {}!", position);
                position_results.push(position as isize + 64 - 72);
            }
        }

        // Now make position 101 be position LAST_LITLEN_OFFSET, with valid data until bit 74 (the cycle can restart)
        self.data >>= 101 - LAST_LITLEN_OFFSET;

        if position_results.is_empty() {
            return Err(());
        } else {
            Ok(position_results)
        }
    }
}

#[cfg(test)]
mod tests {
    use std::io::{BufReader, Read, Seek, SeekFrom};

    #[test]
    fn test_find_block() {
        let mut file =
            BufReader::new(std::fs::File::open("/data/ggcat/data/11SUR1QQSS11.fastq.gz").unwrap());

        let offset = 100000000;

        file.seek(SeekFrom::Start(offset)).unwrap();

        let mut first_data = [0; 16];
        file.read_exact(&mut first_data).unwrap();

        let mut block_finder = match super::BlockFinder::new(u128::from_le_bytes(first_data)) {
            Ok(pos) => {
                panic!("Block found at position: {}", offset * 8 + pos as u64);
            }
            Err(b) => b,
        };

        println!("Started!");

        let mut bytes_count = 16;
        let mut blocks_count = 0;

        let mut data = [0; 12];
        while let Ok(()) = file.read_exact(&mut data) {
            // println!("Checking data: {:?}", data);
            if let Ok(positions) = block_finder.add_data([
                u32::from_le_bytes(data[0..4].try_into().unwrap()),
                u32::from_le_bytes(data[4..8].try_into().unwrap()),
                u32::from_le_bytes(data[8..12].try_into().unwrap()),
            ]) {
                blocks_count += positions.len();
                // for position in positions {
                //     println!(
                //         "Found block at position: {}",
                //         (offset + bytes_count) * 8 + position as u64
                //     );
                // }
                // continue;
                // break;
            }
            bytes_count += 12;
            if bytes_count % (1024 * 1024) == 0 {
                println!(
                    "Processed {} MB bcount: {}",
                    bytes_count / 1024 / 1024,
                    blocks_count
                );
            }
        }
    }
}
