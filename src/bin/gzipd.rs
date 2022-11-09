use streaming_libdeflate_rs::decompress_file_buffered;
use std::fs::File;
use std::io::{Write};
use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt)]
struct GzipParams {
    input: PathBuf,
    #[structopt(short)]
    simulate: bool,
}

fn main() {
    let params: GzipParams = GzipParams::from_args();

    if params.simulate {
        decompress_file_buffered(params.input, move |_| Ok(()), 1024 * 512).unwrap();
    } else {
        let mut write_file = File::create(&params.input.with_extension("")).unwrap();

        decompress_file_buffered(
            params.input,
            move |data| write_file.write_all(data).map_err(|_| ()),
            1024 * 512 * 1024,
        )
        .unwrap();
    };
}
