//! Tier B: the public WSPR decoder reproduces, column for column, what upstream `wsprd`
//! v3.3.0-beta6 prints for the options it takes, on the golden
//! `embedded-poc/assets/golden/wspr/150426_0918.wav`.
//!
//! One test per case of `tests/fixtures/wsprd_beta6/` (regenerate with
//! `scripts/wsprd/gen_fixtures.sh`). Each reads the case's `args` (a `wsprd` command line), turns
//! it into [`WsprExtras`] the way the C parses it (`wsprd.c:853-940`), decodes through
//! `Decoder<Wspr>`, formats through [`mfsk_core::wspr::wsprd_fmt`] and asserts the three text
//! outputs and the saved `hashtable.txt` **byte-equal** to the fixture:
//!
//! - `stdout.txt`: snr, dt, frequency, drift, message;
//! - `ALL_WSPR.TXT`: and sync, pass, blocksize, jitter, decode type, hard errors, Fano cycles and
//!   metric;
//! - `wspr_spots.txt`;
//! - `hashtable.txt` (absent with `-H`, which neither loads nor saves it).
//!
//! Equality of the whole file is also the precision assertion: a decode `wsprd` does not make is a
//! line the fixture lacks, and the test fails on it exactly as on a missing one.
//!
//! The fixtures were made in a directory with no `hashtable.txt`, so the decoder starts with an
//! empty table too, except the `pre*` cases, which start from the table `f` saved
//! (`hashtable_in.txt`). That is the only way OSD (`-o`) ever accepts a codeword: it takes only a
//! call the table already holds, and on this recording `wsprd` finds G8VDQ and W3BI that way.

#![cfg(all(feature = "wspr", any(feature = "fft-rustfft", feature = "fft-extern")))]

#[allow(dead_code)]
mod common;

use std::path::PathBuf;

use mfsk_core::Wspr;
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput, WsprExtras};
use mfsk_core::wspr::{WsprCallsignTable, WsprResult, wsprd_fmt};

const DATE: &str = "150426";
const TIME: &str = "0918";

fn fixture(case: &str, file: &str) -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/wsprd_beta6"
    ))
    .join(case)
    .join(file)
}

fn read(case: &str, file: &str) -> Option<String> {
    std::fs::read_to_string(fixture(case, file)).ok()
}

/// What a `wsprd` command line asks for, as far as the decoder and the printout see it.
struct Cli {
    extras: WsprExtras,
    /// `-f x`, MHz.
    dial_mhz: f64,
    /// `-H`.
    no_table: bool,
}

/// `wsprd.c:853-940`'s `getopt("a:BcC:de:f:HJmo:qstwvz:0")`, for the options the fixtures use.
fn parse(args: &str) -> Cli {
    let mut extras = WsprExtras::wsprd();
    let mut dial_cmdline = 0.0f64;
    let mut dial_error = 0.0f64;
    let mut no_table = false;
    let mut it = args.split_whitespace();
    while let Some(tok) = it.next() {
        let flags = tok
            .strip_prefix('-')
            .unwrap_or_else(|| panic!("bad option {tok}"));
        for (i, c) in flags.char_indices() {
            let rest = &flags[i + 1..];
            // An option that takes a value takes the next word (or the rest of this one).
            let value = |it: &mut std::str::SplitWhitespace<'_>| -> String {
                if rest.is_empty() {
                    it.next().expect("option value").to_string()
                } else {
                    rest.to_string()
                }
            };
            match c {
                'a' => {
                    value(&mut it);
                    break;
                }
                'B' => extras = extras.with_passes(2),
                'C' => {
                    extras = extras.with_max_cycles_per_bit(value(&mut it).parse().unwrap());
                    break;
                }
                'd' => extras = extras.with_more_candidates(true),
                'e' => {
                    dial_error = value(&mut it).parse().unwrap();
                    extras = extras.with_dial_error_hz(dial_error);
                    break;
                }
                'f' => {
                    dial_cmdline = value(&mut it).parse().unwrap();
                    break;
                }
                'H' => no_table = true,
                'o' => {
                    extras = extras.with_osd_depth(value(&mut it).parse().unwrap());
                    break;
                }
                'q' => extras = extras.with_jitter(false),
                's' => extras = extras.with_subtraction(false).with_passes(1),
                'w' => extras = extras.with_freq_span(-150.0, 150.0),
                'z' => {
                    extras = extras.with_fano_bias(value(&mut it).parse().unwrap());
                    break;
                }
                other => panic!("option -{other} is not covered"),
            }
        }
    }
    Cli {
        extras,
        // `wsprd.c:1002`.
        dial_mhz: dial_cmdline - dial_error * 1.0e-6,
        no_table,
    }
}

/// Decode the golden as `wsprd` `args` would and return the four texts.
fn run(case: &str, table_in: Option<&str>) -> (String, String, String, Option<String>) {
    let args = read(case, "args").expect("args");
    let cli = parse(args.trim());
    let Some(path) = common::corpus::golden_path_or_upstream(
        "wspr/150426_0918.wav",
        Some("WSPR/150426_0918.wav"),
    ) else {
        common::skip_or_fail("WSPR golden");
        return Default::default();
    };
    let audio = common::load_wav_f32_opt(&path).expect("wav");
    let mut d = Decoder::<Wspr>::new(DecodeParams::for_band((1400.0, 1620.0)))
        .with_extras(cli.extras.clone());
    // `wsprd.c:1050-1062`: without `-H` the table is loaded from `hashtable.txt` first.
    if let (false, Some(t)) = (cli.no_table, table_in) {
        *d.state_mut().table_mut() = WsprCallsignTable::from_hashtable_txt(t);
    }
    let rows = d.decode(&SlotInput::f32(&audio)).rows;
    let results: Vec<WsprResult> = rows.into_iter().map(|r| r.native).collect();

    let (mut out, mut all, mut spots) = (String::new(), String::new(), String::new());
    for r in wsprd_fmt::in_wsprd_order(&results, cli.dial_mhz) {
        out += &wsprd_fmt::stdout_line(r, TIME, cli.dial_mhz);
        all += &wsprd_fmt::all_wspr_line(r, DATE, TIME, cli.dial_mhz);
        spots += &wsprd_fmt::spots_line(r, DATE, TIME, cli.dial_mhz);
    }
    out += wsprd_fmt::DECODE_FINISHED;
    // `wsprd.c:1636-1645`: saved at the end unless `-H`.
    let table = (!cli.no_table).then(|| d.state().table().to_hashtable_txt());
    (out, all, spots, table)
}

fn check(case: &str) {
    let (out, all, spots, table) = run(case, read(case, "hashtable_in.txt").as_deref());
    if out.is_empty() {
        return; // corpus absent, skipped (or failed) above
    }
    assert_eq!(out, read(case, "stdout.txt").unwrap(), "{case}: stdout");
    assert_eq!(
        all,
        read(case, "ALL_WSPR.TXT").unwrap(),
        "{case}: ALL_WSPR.TXT"
    );
    assert_eq!(
        spots,
        read(case, "wspr_spots.txt").unwrap(),
        "{case}: wspr_spots.txt"
    );
    assert_eq!(table, read(case, "hashtable.txt"), "{case}: hashtable.txt");
}

macro_rules! cases {
    ($($name:ident => $dir:literal),* $(,)?) => {$(
        #[test]
        fn $name() {
            check($dir);
        }
    )*};
}

// `bare` is `wsprd` with no options at all (dial frequency 0). The directory names are
// `wsprd`'s own spellings of the options.
cases! {
    bare => "bare",
    f => "f",
    f_c500_o4 => "f_C500_o4",
    f_c500_o4_d => "f_C500_o4_d",
    f_qb => "f_qB",
    f_s => "f_s",
    f_w => "f_w",
    f_z040 => "f_z040",
    f_c100 => "f_C100",
    f_o1 => "f_o1",
    f_e60 => "f_e60",
    f_h => "f_H",
    f_b => "f_B",
    f_q => "f_q",
    f_e45 => "f_e45",
    // Started from the table `f` saved: OSD now has calls to accept.
    pre => "pre",
    pre_o0 => "pre_o0",
    pre_o1 => "pre_o1",
    pre_o2 => "pre_o2",
    pre_o3 => "pre_o3",
    pre_o4 => "pre_o4",
    pre_o5 => "pre_o5",
    pre_c500_o4_d => "pre_C500_o4_d",
    pre_o4_s => "pre_o4_s",
}

/// `hashtable.txt` read and written as `wsprd` does (`wsprd.c:1053-1060`, `1636-1645`,
/// `wsprd_load_hash_line`): every case's saved table loads and saves back byte-identically.
#[test]
fn hashtable_round_trip() {
    let mut n = 0;
    for entry in std::fs::read_dir(fixture("", "")).unwrap() {
        let case = entry.unwrap().file_name().into_string().unwrap();
        let Some(text) = read(&case, "hashtable.txt") else {
            continue;
        };
        let table = WsprCallsignTable::from_hashtable_txt(&text);
        assert_eq!(table.to_hashtable_txt(), text, "{case}");
        assert_eq!(table.len(), text.lines().count(), "{case}");
        n += 1;
    }
    assert!(n >= 11, "only {n} hashtables found");
}
