//! Tier B: the public WSPR decoder reproduces, column for column, what upstream `wsprd`
//! v3.3.0-beta6 prints for the options it takes, on the golden
//! `embedded-poc/assets/golden/wspr/150426_0918.wav`.
//!
//! One test per case of `tests/fixtures/wsprd_beta6/` (regenerate with
//! `scripts/wsprd/gen_fixtures.sh`). Each reads the case's `args` (a `wsprd` command line), turns
//! it into [`WsprExtras`] the way the C parses it (`wsprd.c:859-918`), decodes through
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
//! `t3*` and `far*` run on synthetic recordings (`common/wspr_t3.rs`, integer noise and the crate's
//! own synthesiser) instead of the golden, so that `wsprd`'s behaviour on a Type 3 whose call is not
//! in the table (it subtracts the codeword of `nhash("...")`, counts its hard errors against that,
//! and so never sees the weak W1AW beside it) and on the 13-byte callsign buffer is vendored
//! without a 2.9 MB WAV. The `Depth` path is tested not to do the same
//! (`depth_path_subtracts_the_decoded_codeword`).
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

#[allow(dead_code)]
#[path = "common/wspr_t3.rs"]
mod wspr_t3;

/// Where a case's audio comes from, and the date and time `wsprd` read from its file name: the
/// golden, or one of the synthetic recordings of `common/wspr_t3.rs` (`t3*`, `far*`), which
/// `gen_fixtures.sh` writes through `examples/gen_wsprd_t3_wav.rs` and this rebuilds in memory.
enum Audio {
    Golden(Vec<f32>),
    Synthetic(Vec<i16>),
}

fn source(case: &str) -> Option<(Audio, &'static str, &'static str)> {
    if case.starts_with("t3") {
        return Some((Audio::Synthetic(wspr_t3::build()), "250101", "0000"));
    }
    if case.starts_with("far") {
        return Some((Audio::Synthetic(wspr_t3::build_far()), "250101", "0002"));
    }
    let Some(path) = common::corpus::golden_path_or_upstream(
        "wspr/150426_0918.wav",
        Some("WSPR/150426_0918.wav"),
    ) else {
        common::skip_or_fail("WSPR golden");
        return None;
    };
    Some((
        Audio::Golden(common::load_wav_f32_opt(&path).expect("wav")),
        "150426",
        "0918",
    ))
}

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

/// `wsprd.c:859-918`'s `getopt("a:BcC:de:f:HJmo:qstwvz:0")`, for the options the fixtures use.
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
        // `wsprd.c:1003`.
        dial_mhz: dial_cmdline - dial_error * 1.0e-6,
        no_table,
    }
}

/// Decode the golden as `wsprd` `args` would and return the four texts.
fn run(case: &str, table_in: Option<&str>) -> (String, String, String, Option<String>) {
    let args = read(case, "args").expect("args");
    let cli = parse(args.trim());
    let Some((audio, date, time)) = source(case) else {
        return Default::default();
    };
    let mut d = Decoder::<Wspr>::new(DecodeParams::for_band((1400.0, 1620.0)))
        .with_extras(cli.extras.clone());
    // `wsprd.c:1051-1060`: without `-H` the table is loaded from `hashtable.txt` first.
    if let (false, Some(t)) = (cli.no_table, table_in) {
        *d.state_mut().table_mut() = WsprCallsignTable::from_hashtable_txt(t);
    }
    let rows = match &audio {
        Audio::Golden(a) => d.decode(&SlotInput::f32(a)),
        Audio::Synthetic(a) => d.decode(&SlotInput::i16(a)),
    }
    .rows;
    let results: Vec<WsprResult> = rows.into_iter().map(|r| r.native).collect();

    let (mut out, mut all, mut spots) = (String::new(), String::new(), String::new());
    for r in wsprd_fmt::in_wsprd_order(&results, cli.dial_mhz) {
        out += &wsprd_fmt::stdout_line(r, time, cli.dial_mhz);
        all += &wsprd_fmt::all_wspr_line(r, date, time, cli.dial_mhz);
        spots += &wsprd_fmt::spots_line(r, date, time, cli.dial_mhz);
    }
    out += wsprd_fmt::DECODE_FINISHED;
    // `wsprd.c:1637-1645`: saved at the end unless `-H`.
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
    // Synthetic: a Type 3 whose call is not in the table (`<...>`) with a -27 dB W1AW beside it;
    // the same with an 11-character call in the Type 3's bucket; a call outside the window that
    // `-e -10` moves into it on the second pass, with and without `-B`.
    t3 => "t3",
    t3_long => "t3_long",
    far_b_e10 => "far_B_e10",
    far_e10 => "far_e10",
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

/// `hashtable.txt` read and written as `wsprd` does (`wsprd.c:1051-1060`, `1636-1645`,
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

/// The `Depth` path subtracts the codeword it decoded, not the one `wsprd` rebuilds from the
/// printed `<...>` text (a deliberate divergence, CLAUDE.md §2.1), so on the `t3` recording it
/// decodes the weak W1AW beside the Type 3 that `wsprd` never hears, and with `wsprd`'s replica
/// switched on it does not.
#[test]
fn depth_path_subtracts_the_decoded_codeword() {
    let samples = wspr_t3::build();
    let texts = |e: WsprExtras| -> Vec<String> {
        let mut d = Decoder::<Wspr>::new(DecodeParams::for_band((1400.0, 1620.0))).with_extras(e);
        d.decode(&SlotInput::i16(&samples))
            .rows
            .into_iter()
            .map(|r| r.decoded.text)
            .collect()
    };
    let wsprd = texts(WsprExtras::wsprd());
    assert_eq!(wsprd, ["<...> FN42AB 37"], "wsprd's replica");
    let depth = texts(WsprExtras::default());
    assert!(
        depth.iter().any(|t| t.starts_with("W1AW FN31")),
        "{depth:?}"
    );
    let right = texts(WsprExtras::wsprd().with_replica_from_text(false));
    assert!(
        right.iter().any(|t| t.starts_with("W1AW FN31")),
        "{right:?}"
    );
}

/// `dial_error_hz` moves a window, and `wsprd` always has one, so without `freq_span_hz` it brings
/// `wsprd`'s +/-110 Hz: the same decodes as the explicit span.
#[test]
fn dial_error_implies_the_span() {
    let Some((Audio::Golden(a), _, _)) = source("f") else {
        return;
    };
    let texts = |e: WsprExtras| -> Vec<String> {
        let mut d = Decoder::<Wspr>::new(DecodeParams::for_band((1400.0, 1620.0))).with_extras(e);
        d.decode(&SlotInput::f32(&a))
            .rows
            .into_iter()
            .map(|r| r.decoded.text)
            .collect()
    };
    let mut implied = WsprExtras::wsprd().with_dial_error_hz(60.0);
    implied.freq_span_hz = None;
    let explicit = WsprExtras::wsprd().with_dial_error_hz(60.0);
    assert_eq!(texts(implied), texts(explicit));
}
