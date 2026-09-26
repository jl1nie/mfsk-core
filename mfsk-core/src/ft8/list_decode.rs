//! WSJT-X's two FT8 list decoders, "a7" and "a8" (#464).
//!
//! Both run in `ft8_decode.f90` after the candidate loop, at the end of a
//! period (`nzhsym == 50`) with AP on, and both decide between a *list* of
//! messages built from call signs already known, instead of decoding the LDPC
//! code: every candidate is a valid codeword by construction.
//!
//! - **a7** (`ft8_a7.f90`, `ft8_decode.f90:250-285`). For each message decoded in
//!   the same sequence one cycle earlier (30 s ago), build the messages the same
//!   pair could send next and pick the one closest to the received LLRs at that
//!   message's frequency and DT. Reported as `iaptype = 7`.
//! - **a8** (`ft8_a8d.f90`, `ft8_decode.f90:287-305`). With MyCall, HisCall and
//!   HisGrid given, correlate the waveforms of the messages of that QSO against
//!   the received signal near the QSO frequency (`nfqso`), and keep the best fit
//!   if it passes three quality tests. Reported as `iaptype = 8`.
//!
//! Where this differs from the Fortran, on purpose:
//! - Messages with a *hashed* call sign in a Type 1 field (`<CALL> CALL -10`) are
//!   not generated: `wsjt77::pack28` does not pack a hash. Messages that need them
//!   (a report or grid with one non-standard call) are skipped; the Type 4 forms
//!   (`<CALL> NONSTD`, with or without RRR / RR73 / 73) are generated.
//! - a7's SNR is [`crate::ft8::llr::compute_snr_db`], this crate's FT8 SNR, not
//!   `ft8_a7d`'s `pbest/xbase` form (the decode, and whether it is accepted, do not
//!   depend on it).
//! - Both run on the request's audio. Upstream runs them on `dd` after the passes'
//!   subtractions.

use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;

use num_complex::{Complex, Complex32};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std makes f32's own methods shadow it
use num_traits::Float;

use crate::engine::dsp::gfsk::{GfskCfg, synth_complex_f32_into};
use crate::engine::pipeline::DecodeResult;
use crate::engine::scalar::Cmplx;
use crate::fec::ldpc::{append_crc14, ldpc_encode};
use crate::ft8::Ft8;
use crate::ft8::decode_block::{SymMask, fill_symbol_spectra};
use crate::msg::wsjt77::{is_standard_callsign, pack77, pack77_type4, unpack77};

/// Pass id of an a7 decode (`iaptype = 7` upstream). The ladder's ids run to 23
/// (OSD `zsave(:,2)` on `llre` is `19 + 4`), so the list decoders start at 30.
pub const PASS_ID_A7: u8 = 30;
/// Pass id of an a8 decode (`iaptype = 8` upstream).
pub const PASS_ID_A8: u8 = 31;

/// `ft8_a7.f90`'s `MAXDEC`: at most this many decodes of one sequence are kept.
const MAXDEC: usize = 200;
/// Messages tried per call-sign pair (`MAXMSG` in `ft8_a7d`, `NMSGS` in `ft8_a8d`).
const NMSGS: usize = 206;

// ────────────────────────────────────────────────────────────────────
// Message lists

/// One party of a list message: the call sign as it is written, and whether the
/// message carries it hashed (`<CALL>`).
#[derive(Clone, Copy)]
struct Party<'s> {
    call: &'s str,
    hashed: bool,
}

/// Pack `c1 c2 tail`, the shapes `ft8_a7d` / `getmsg` build. `None` when this crate
/// cannot pack it (a hashed call sign in a Type 1 field, see the module doc).
fn pack_list(c1: Party<'_>, c2: Party<'_>, tail: &str) -> Option<[u8; 77]> {
    match (c1.hashed, c2.hashed) {
        (false, false) => {
            if let Some(m) = pack77(c1.call, c2.call, tail) {
                return Some(m);
            }
            // `CQ NONSTD`: Type 4 with the CQ flag.
            if c1.call == "CQ" && tail.is_empty() {
                return pack77_type4(c2.call, "", "", true);
            }
            None
        }
        // `<STD> NONSTD [RRR|RR73|73]`: Type 4, hash first (iflip 0).
        (true, false) if is_standard_callsign(c1.call) && !is_standard_callsign(c2.call) => {
            let mut m = pack77_type4(c2.call, c1.call, tail, false)?;
            m[70] = 0;
            Some(m)
        }
        // `NONSTD <STD> [RRR|RR73|73]`: Type 4, hash second (iflip 1).
        (false, true) if !is_standard_callsign(c1.call) && is_standard_callsign(c2.call) => {
            let mut m = pack77_type4(c1.call, c2.call, tail, false)?;
            m[70] = 1;
            Some(m)
        }
        _ => None,
    }
}

/// The report of list message `i` (7..=206): `-50 + (i-7)/2` dB, plain for odd `i`,
/// `R`-prefixed for even, as `write(msg,'(i3.2)')` renders it (`+05`, `-12`).
fn report_of(i: usize) -> (i32, String) {
    let isnr = -50 + (i as i32 - 7) / 2;
    let body = if isnr < 0 {
        alloc::format!("-{:02}", -isnr)
    } else {
        alloc::format!("+{:02}", isnr)
    };
    let text = if i % 2 == 1 {
        body
    } else {
        alloc::format!("R{body}")
    };
    (isnr, text)
}

/// `ft8_a7d`'s message `i` (1..=206) for the pair `call_1 call_2`, `grid4` possibly
/// empty (`ft8_a7.f90:279-312`).
fn a7_message(i: usize, call_1: &str, call_2: &str, grid4: &str) -> Option<[u8; 77]> {
    let std_1 = call_1 == "CQ" || is_standard_callsign(call_1);
    let std_2 = is_standard_callsign(call_2);
    // `QU1RK` stands in for the pair's messages when the pair is a CQ: they are
    // tried so that the second-best distance means something, and a best fit on
    // one of them is rejected below.
    let c1 = if call_1 == "CQ" && i != 5 {
        "QU1RK"
    } else {
        call_1
    };
    let (mut p1, mut p2) = (
        Party {
            call: c1,
            hashed: false,
        },
        Party {
            call: call_2,
            hashed: false,
        },
    );
    if !std_1 {
        if i == 1 || i >= 6 {
            p1.hashed = true;
        }
        if (2..=4).contains(&i) {
            p2.hashed = true;
        }
    } else if !std_2 {
        if i <= 4 || i == 6 {
            p1.hashed = true;
        }
        if i >= 7 {
            p2.hashed = true;
        }
    }
    match i {
        1 => pack_list(p1, p2, ""),
        2 => pack_list(p1, p2, "RRR"),
        3 => pack_list(p1, p2, "RR73"),
        4 => pack_list(p1, p2, "73"),
        5 => {
            let cq = Party {
                call: "CQ",
                hashed: false,
            };
            let c2 = Party {
                call: call_2,
                hashed: false,
            };
            if std_2 && grid4 != "RR73" {
                pack_list(cq, c2, grid4)
            } else {
                pack_list(cq, c2, "")
            }
        }
        6 if std_2 => pack_list(p1, p2, grid4),
        6 => pack_list(p1, p2, ""),
        _ => pack_list(p1, p2, &report_of(i).1),
    }
}

/// `getmsg` (`ft8_a7.f90:380-422`): a8's message `i` for MyCall and HisCall.
/// `None` for a report beyond +-30 dB (`getmsg` returns an empty message there).
fn a8_message(i: usize, mycall: &str, hiscall: &str, hisgrid: &str) -> Option<[u8; 77]> {
    let my_std = is_standard_callsign(mycall);
    let his_std = is_standard_callsign(hiscall);
    let (mut p1, mut p2) = (
        Party {
            call: mycall,
            hashed: false,
        },
        Party {
            call: hiscall,
            hashed: false,
        },
    );
    if !my_std {
        if i == 1 || i >= 6 {
            p1.hashed = true;
        }
        if (2..=4).contains(&i) {
            p2.hashed = true;
        }
    } else if !his_std {
        if i <= 4 || i == 6 {
            p1.hashed = true;
        }
        if i >= 7 {
            p2.hashed = true;
        }
    }
    let grid = hisgrid.get(..4).unwrap_or(hisgrid);
    match i {
        1 => pack_list(p1, p2, ""),
        2 => pack_list(p1, p2, "RRR"),
        3 => pack_list(p1, p2, "RR73"),
        4 => pack_list(p1, p2, "73"),
        5 => {
            let cq = Party {
                call: "CQ",
                hashed: false,
            };
            let his = Party {
                call: hiscall,
                hashed: false,
            };
            pack_list(cq, his, if his_std { grid } else { "" })
        }
        6 if his_std => pack_list(p1, p2, grid),
        6 => pack_list(p1, p2, ""),
        _ => {
            let (isnr, text) = report_of(i);
            if isnr.abs() > 30 {
                return None;
            }
            pack_list(p1, p2, &text)
        }
    }
}

// ────────────────────────────────────────────────────────────────────
// The a7 table (`ft8_a7_save`)

/// One entry of the a7 table: the message's first two words (plus the grid when
/// the last word is one), as `ft8_a7_save` stores it, and where it was decoded.
#[derive(Clone)]
struct A7Entry {
    msg0: String,
    freq_hz: f32,
    dt_sec: f32,
    /// `f0 = -98`: this station was already decoded in the current period near
    /// this frequency, so do not try it.
    done: bool,
}

fn is_grid4(w: &str) -> bool {
    let b = w.as_bytes();
    b.len() == 4
        && (b'A'..=b'R').contains(&b[0])
        && (b'A'..=b'R').contains(&b[1])
        && b[2].is_ascii_digit()
        && b[3].is_ascii_digit()
}

/// `ft8_a7_save`'s view of a decoded message: `None` for the messages it skips
/// (`/`, `<`, `CQ_`), else the stored text.
fn a7_msg0(text: &str) -> Option<String> {
    if text.contains('/') || text.contains('<') {
        return None;
    }
    let w: Vec<&str> = text.split_whitespace().collect();
    if w.is_empty() || w[0].starts_with("CQ_") {
        return None;
    }
    let mut msg0 = if w[0] == "CQ" && w.len() >= 3 && w[1].len() <= 2 {
        alloc::format!("CQ {} {}", w[1], w[2])
    } else {
        alloc::format!("{} {}", w[0], w.get(1).copied().unwrap_or(""))
    };
    if let Some(last) = w.last()
        && is_grid4(last)
    {
        msg0.push(' ');
        msg0.push_str(last);
    }
    Some(msg0.trim_end().into())
}

/// The table for this period: the previous cycle's decodes of this sequence, in
/// their order, capped at [`MAXDEC`].
fn build_table(previous: &[DecodeResult]) -> Vec<A7Entry> {
    let mut table = Vec::new();
    for r in previous {
        if table.len() >= MAXDEC {
            break;
        }
        let Some(text) = unpack77(r.message77()) else {
            continue;
        };
        let Some(msg0) = a7_msg0(&text) else {
            continue;
        };
        table.push(A7Entry {
            msg0,
            freq_hz: r.freq_hz,
            dt_sec: r.dt_sec,
            done: false,
        });
    }
    table
}

/// `ft8_a7_save`'s second half, for a decode of *this* period at `freq_hz`: an
/// entry within 3 Hz whose text contains ` <its second word>` is flagged done.
fn mark_done(table: &mut [A7Entry], text: &str, freq_hz: f32) {
    let Some(msg0) = a7_msg0(text) else {
        return;
    };
    let Some(w2) = msg0.split_whitespace().nth(1) else {
        return;
    };
    let needle = alloc::format!(" {w2}");
    for e in table.iter_mut().filter(|e| !e.done) {
        // Fortran `index(msg0, ' '//w2) >= 3`: found, and not at the very start.
        if (freq_hz - e.freq_hz).abs() <= 3.0 && e.msg0.find(&needle).is_some_and(|p| p >= 2) {
            e.done = true;
        }
    }
}

// ────────────────────────────────────────────────────────────────────
// a7 (`ft8_a7d`)

struct A7Hit {
    message77: [u8; 77],
    freq_hz: f32,
    dt_sec: f32,
    hard_errors: u32,
    snr_db: f32,
}

fn a7_decode(
    audio: &[i16],
    fft_cache: &[Complex<f32>],
    call_1: &str,
    call_2: &str,
    grid4: &str,
    dt_sec: f32,
    freq_hz: f32,
) -> Option<A7Hit> {
    // Peak up in time and frequency (`ft8_a7.f90:139-183`), the same 3-stage
    // search `ft8b.f90` uses, which `fine_refine_3stage` ports.
    let cd0 = crate::engine::dsp::downsample::downsample_cached(
        fft_cache,
        freq_hz,
        &crate::ft8::downsample::FT8_CFG,
    );
    let refine = crate::ft8::refine_fine::fine_refine_3stage(&cd0, dt_sec);
    let freq = freq_hz + refine.delf_hz;
    let dt = refine.dt_sec;

    let mut cs = [[Cmplx::<f32>::default(); 8]; 79];
    fill_symbol_spectra(&mut cs, audio, freq, dt, SymMask::SyncOnly, Some(fft_cache));
    fill_symbol_spectra(&mut cs, audio, freq, dt, SymMask::DataOnly, Some(fft_cache));
    // `ft8_a7d` builds bmeta..bmetd from `abs(cs)` (imetric 1) and scales by 2.83.
    let llr = crate::ft8::llr::compute_llr_metric::<f32>(&cs, false);
    let variants: [&[f32; 174]; 4] = [&llr.llra, &llr.llrb, &llr.llrc, &llr.llrd];

    let mut dmm = [f32::INFINITY; NMSGS];
    let mut dmin = f32::INFINITY;
    let mut best: Option<([u8; 77], u32)> = None;
    for (idx, slot) in dmm.iter_mut().enumerate() {
        let Some(m77) = a7_message(idx + 1, call_1, call_2, grid4) else {
            continue;
        };
        let cw = ldpc_encode(&append_crc14(&m77));
        // Weighted distance of the codeword from each variant's hard decisions.
        let dist = |l: &[f32; 174]| -> f32 {
            cw.iter()
                .zip(l.iter())
                .filter(|&(&b, &v)| (v >= 0.0) != (b == 1))
                .map(|(_, &v)| v.abs())
                .sum()
        };
        let d: [f32; 4] = [
            dist(variants[0]),
            dist(variants[1]),
            dist(variants[2]),
            dist(variants[3]),
        ];
        let dm = d.iter().copied().fold(f32::INFINITY, f32::min);
        *slot = dm;
        if dm < dmin {
            dmin = dm;
            let v = d.iter().position(|&x| x == dm).unwrap_or(0);
            let hard = cw
                .iter()
                .zip(variants[v].iter())
                .filter(|&(&b, &l)| (2.0 * b as f32 - 1.0) * l < 0.0)
                .count() as u32;
            best = Some((m77, hard));
        }
    }
    let (m77, hard_errors) = best?;

    // Second-best distance, and the acceptance tests (`ft8_a7.f90:363-375`).
    let ibest = dmm
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.total_cmp(b.1))
        .map(|(i, _)| i)?;
    dmm[ibest] = f32::INFINITY;
    let dmin2 = dmm.iter().copied().fold(f32::INFINITY, f32::min);
    if dmin > 100.0 || dmin2 / (dmin + 1e-6) < 1.3 {
        return None;
    }
    let text = unpack77(&m77)?;
    if text.starts_with("CQ ") && is_standard_callsign(call_2) && grid4.is_empty() {
        return None;
    }
    if text.starts_with("QU1RK ") {
        return None;
    }

    let tones: [u8; 79] = crate::engine::tx::message_to_tones::<Ft8>(&m77)
        .try_into()
        .ok()?;
    let snr_db = crate::ft8::llr::compute_snr_db(&cs, &tones).max(-25.0);
    Some(A7Hit {
        message77: m77,
        freq_hz: freq,
        dt_sec: dt,
        hard_errors,
        snr_db,
    })
}

// ────────────────────────────────────────────────────────────────────
// a8 (`ft8_a8d`)

/// Complex GFSK waveform of `tones` at 200 Hz, 32 samples a symbol, BT 2, from 0 Hz
/// (`gen_ft8wave(itone,NN,32,2.0,200.0,0.0,cwave,...,1,NWAVE)`).
fn a8_wave(tones: &[u8]) -> Vec<Complex32> {
    const NWAVE: usize = 79 * 32;
    let cfg = GfskCfg {
        sample_rate: 200.0,
        samples_per_symbol: 32,
        bt: 2.0,
        hmod: 1.0,
        ramp_samples: 4, // nint(nsps/8.0)
    };
    let (mut c, mut s) = (vec![0.0f32; NWAVE], vec![0.0f32; NWAVE]);
    synth_complex_f32_into(&mut c, &mut s, tones, 0.0, 1.0, &cfg);
    c.into_iter()
        .zip(s)
        .map(|(re, im)| Complex32::new(re, im))
        .collect()
}

struct A8Hit {
    message77: [u8; 77],
    freq_hz: f32,
    dt_sec: f32,
    hard: u32,
    snr_db: f32,
}

fn a8_decode(
    fft_cache: &[Complex<f32>],
    mycall: &str,
    hiscall: &str,
    hisgrid: &str,
    f1: f32,
) -> Option<A8Hit> {
    const NZZ: usize = 3200;
    const NH: i32 = (NZZ / 2) as i32;
    const NWAVE: usize = 79 * 32;
    const FAC: f32 = 1e-6;
    let df = 200.0 / NZZ as f32;
    let dt = 1.0 / 200.0;

    let cd = crate::engine::dsp::downsample::downsample_cached(
        fft_cache,
        f1,
        &crate::ft8::downsample::FT8_CFG,
    );
    if cd.len() < NZZ {
        return None;
    }
    let mut planner = crate::engine::fft::default_planner();
    let fft = planner.plan_forward(NZZ);
    let fft32 = planner.plan_forward(32);

    // s(-NH..=NH), stored at index j + NH.
    let idx = |j: i32| (j + NH) as usize;
    let mut s = vec![0.0f32; NZZ + 1];
    let mut buf = vec![Complex32::new(0.0, 0.0); NZZ];

    let (mut sbest, mut fbest, mut tbest) = (0.0f32, 0.0f32, 0.0f32);
    let mut msgbest: Option<([u8; 77], Vec<u8>)> = None;
    let mut s1 = vec![0.0f32; NZZ + 1];

    for i in 1..=NMSGS {
        let Some(m77) = a8_message(i, mycall, hiscall, hisgrid) else {
            continue;
        };
        let tones = crate::engine::tx::message_to_tones::<Ft8>(&m77);
        let cwave = a8_wave(&tones);

        let (mut spk, mut fpk, mut tpk, mut lagpk) = (0.0f32, 0.0f32, 0.0f32, 0i32);
        let mut s0 = vec![0.0f32; NZZ + 1];
        let (mut lag1, mut lag2, mut step) = (-200i32, 200i32, 4usize);
        for iter in 0..2 {
            if iter == 1 {
                lag1 = lagpk - 8;
                lag2 = lagpk + 8;
                step = 1;
            }
            for lag in (lag1..=lag2).step_by(step) {
                for (i, b) in buf.iter_mut().enumerate() {
                    *b = if i < NWAVE {
                        let j = i as i32 + lag + 100;
                        if (0..NWAVE as i32).contains(&j) {
                            cd[j as usize] * cwave[i].conj()
                        } else {
                            Complex32::new(0.0, 0.0)
                        }
                    } else {
                        Complex32::new(0.0, 0.0)
                    };
                }
                fft.process(&mut buf);
                for (i, b) in buf.iter().enumerate() {
                    let j = if i as i32 > NH {
                        i as i32 - NZZ as i32
                    } else {
                        i as i32
                    };
                    s[idx(j)] = FAC * b.norm_sqr();
                }
                smo121(&mut s);
                for j in -NH..=NH {
                    if s[idx(j)] > spk {
                        spk = s[idx(j)];
                        fpk = j as f32 * df + f1;
                        lagpk = lag;
                        tpk = lag as f32 * dt;
                        s0.copy_from_slice(&s);
                    }
                }
            }
        }
        if spk > sbest {
            sbest = spk;
            fbest = fpk;
            tbest = tpk;
            msgbest = Some((m77, tones));
            s1.copy_from_slice(&s0);
        }
    }
    let (m77, itone) = msgbest?;

    // Re-centre on the best frequency (`twkfreq1` with a(1) = f1 - fbest).
    let a1 = f1 - fbest;
    if a1.abs() > 5.0 {
        return None;
    }
    let dphi = core::f32::consts::TAU * a1 / 200.0;
    let mut cdr: Vec<Complex32> = cd[..NZZ].to_vec();
    for (k, c) in cdr.iter_mut().enumerate() {
        let ph = dphi * (k + 1) as f32;
        *c *= Complex32::new(ph.cos(), ph.sin());
    }

    // SNR from the best-fit spectrum (`ft8_a8d.f90:137-149`).
    let ave = ((-200..=-100).map(|j| s1[idx(j)]).sum::<f32>()
        + (100..=200).map(|j| s1[idx(j)]).sum::<f32>())
        / 202.0;
    if ave <= 0.0 {
        return None;
    }
    for v in s1.iter_mut() {
        *v = *v / ave - 1.0;
    }
    let s1pk = (-32..=32).map(|j| s1[idx(j)]).fold(f32::MIN, f32::max);
    let (mut sig, mut nsig) = (0.0f32, 0u32);
    for j in -32..=32 {
        if s1[idx(j)] >= 0.5 * s1pk {
            sig += s1[idx(j)];
            nsig += 1;
        }
    }
    let sig = if nsig > 0 { sig / nsig as f32 } else { 0.0 };
    let snr_db = if sig > 0.0 {
        (10.0 * sig.log10() - 35.0).max(-30.0)
    } else {
        -30.0
    };

    // Probability of a correct decode (`ft8_a8d.f90:151-189`).
    let i00 = ((tbest + 0.5) / 0.005).round() as i32;
    let (mut plog, mut nhard, mut nsum) = (0.0f32, 0u32, 0u32);
    let (mut sum_sync, mut sum_sig, mut sum_big) = (0.0f32, 0.0f32, 0.0f32);
    let mut csymb = [Complex32::new(0.0, 0.0); 32];
    for k in 1..=79usize {
        let i0 = 32 * (k as i32 - 1) + i00;
        for (i, c) in csymb.iter_mut().enumerate() {
            let n = i0 + i as i32;
            *c = if (0..NZZ as i32).contains(&n) {
                cdr[n as usize]
            } else {
                Complex32::new(0.0, 0.0)
            };
        }
        fft32.process(&mut csymb);
        let s8: [f32; 8] = core::array::from_fn(|i| csymb[i].norm_sqr());
        let s8sum: f32 = s8.iter().sum();
        let t = itone[k - 1] as usize;
        if s8sum > 0.0 {
            plog += (s8[t] / s8sum).ln();
            nsum += 1;
        }
        // Fortran `maxloc`: the first maximum.
        let ipk = s8
            .iter()
            .enumerate()
            .fold(0usize, |b, (i, &v)| if v > s8[b] { i } else { b });
        if ipk != t {
            nhard += 1;
        }
        if k <= 7 || (37..=43).contains(&k) || k >= 73 {
            sum_sync += s8[t];
        } else {
            sum_sig += s8[t];
        }
        sum_big += s8[ipk];
    }
    if nsum < 79 {
        plog += (79 - nsum) as f32 * 0.125f32.ln();
    }
    let sigobig = if sum_big > 0.0 {
        (sum_sync + sum_sig) / sum_big
    } else {
        0.0
    };
    if nhard > 54 || plog < -159.0 || sigobig < 0.71 {
        return None;
    }
    Some(A8Hit {
        message77: m77,
        freq_hz: fbest,
        dt_sec: tbest,
        hard: nhard,
        snr_db,
    })
}

/// `smo121`: `x(i) = 0.5 x(i) + 0.25 (x(i-1) + x(i+1))` for the interior, in place,
/// each step reading the previous element's value from before it was smoothed.
fn smo121(x: &mut [f32]) {
    if x.len() < 3 {
        return;
    }
    let mut x0 = x[0];
    for i in 1..x.len() - 1 {
        let x1 = x[i];
        x[i] = 0.5 * x[i] + 0.25 * (x0 + x[i + 1]);
        x0 = x1;
    }
}

// ────────────────────────────────────────────────────────────────────
// Driver

/// What the list decoders need from the request.
pub(crate) struct ListDecodeInput<'a> {
    pub audio: &'a [i16],
    pub fft_cache: &'a [Complex<f32>],
    /// This sequence's decodes of one cycle earlier: enables a7 when non-empty.
    pub previous: &'a [DecodeResult],
    /// MyCall, HisCall and HisGrid (from `ap_hint`) and the QSO frequency
    /// (`freq_hint`): enables a8 when all are given.
    pub a8: Option<(&'a str, &'a str, &'a str, f32)>,
    /// Decodes the caller already has (`known`): not reported again.
    pub known: &'a [DecodeResult],
}

fn result_of(
    m77: &[u8; 77],
    freq_hz: f32,
    dt_sec: f32,
    hard: u32,
    snr: f32,
    pass: u8,
) -> DecodeResult {
    DecodeResult {
        info: append_crc14(m77).to_vec().into_boxed_slice(),
        freq_hz,
        dt_sec,
        hard_errors: hard,
        sync_score: 0.0,
        pass,
        sync_cv: 0.0,
        snr_db: snr,
    }
}

/// Run a7, then a8, after the ladder's decodes `results`; returns the new decodes
/// in the order `ft8_decode.f90` reports them. `on_new` sees each one as it is found.
pub(crate) fn run(
    input: &ListDecodeInput<'_>,
    results: &[DecodeResult],
    mut on_new: impl FnMut(&DecodeResult),
) -> Vec<DecodeResult> {
    let mut out: Vec<DecodeResult> = Vec::new();
    let seen = |m: &[u8; 77], out: &[DecodeResult]| {
        results
            .iter()
            .chain(out.iter())
            .chain(input.known.iter())
            .any(|r| r.message77() == m)
    };
    let hiscall = input.a8.map(|(_, h, _, _)| h);
    let mut try_a8 = true;

    if !input.previous.is_empty() {
        let mut table = build_table(input.previous);
        for r in results {
            if let Some(text) = unpack77(r.message77()) {
                mark_done(&mut table, &text, r.freq_hz);
            }
        }
        for k in 0..table.len() {
            if table[k].done {
                continue;
            }
            let e = table[k].clone();
            let mut w = e.msg0.splitn(3, ' ');
            let call_1 = w.next().unwrap_or("");
            let call_2 = w.next().unwrap_or("");
            let mut grid4 = w.next().unwrap_or("").get(..4).unwrap_or("").to_string();
            if grid4 == "RR73" || grid4.contains('+') || grid4.contains('-') {
                grid4.clear();
            }
            let Some(hit) = a7_decode(
                input.audio,
                input.fft_cache,
                call_1,
                call_2,
                &grid4,
                e.dt_sec,
                e.freq_hz,
            ) else {
                continue;
            };
            let text = unpack77(&hit.message77).unwrap_or_default();
            if let Some(h) = hiscall
                && text.contains(h)
            {
                try_a8 = false;
            }
            mark_done(&mut table, &text, hit.freq_hz);
            if seen(&hit.message77, &out) {
                continue;
            }
            let r = result_of(
                &hit.message77,
                hit.freq_hz,
                hit.dt_sec,
                hit.hard_errors,
                hit.snr_db,
                PASS_ID_A7,
            );
            on_new(&r);
            out.push(r);
        }
    }

    if let Some((mycall, hiscall, hisgrid, nfqso)) = input.a8
        && try_a8
        && hiscall.trim().len() >= 3
        && hisgrid.trim().len() >= 4
        && let Some(hit) = a8_decode(input.fft_cache, mycall, hiscall, hisgrid, nfqso)
        && !seen(&hit.message77, &out)
    {
        let r = result_of(
            &hit.message77,
            hit.freq_hz,
            hit.dt_sec,
            hit.hard,
            hit.snr_db,
            PASS_ID_A8,
        );
        on_new(&r);
        out.push(r);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reports_render_as_fortran_i3_2() {
        assert_eq!(report_of(7), (-50, "-50".into()));
        assert_eq!(report_of(8), (-50, "R-50".into()));
        assert_eq!(report_of(7 + 2 * 45), (-5, "-05".into()));
        assert_eq!(report_of(7 + 2 * 55), (5, "+05".into()));
        assert_eq!(report_of(206), (49, "R+49".into()));
    }

    #[test]
    fn a7_table_keeps_the_first_two_words_and_a_grid() {
        assert_eq!(a7_msg0("K1ABC W9XYZ -10").as_deref(), Some("K1ABC W9XYZ"));
        assert_eq!(a7_msg0("CQ W9XYZ EN37").as_deref(), Some("CQ W9XYZ EN37"));
        assert_eq!(
            a7_msg0("CQ DX W9XYZ EN37").as_deref(),
            Some("CQ DX W9XYZ EN37")
        );
        assert_eq!(
            a7_msg0("K1ABC W9XYZ RR73").as_deref(),
            Some("K1ABC W9XYZ RR73")
        );
        assert_eq!(a7_msg0("K1ABC/P W9XYZ -10"), None);
        assert_eq!(a7_msg0("<K1ABC> W9XYZ -10"), None);
    }

    /// Every standard-pair list message packs and unpacks to the text `ft8_a7d`
    /// builds for it.
    #[test]
    fn a7_list_for_a_standard_pair() {
        let text = |i| unpack77(&a7_message(i, "K1ABC", "W9XYZ", "EN37").unwrap()).unwrap();
        assert_eq!(text(1), "K1ABC W9XYZ");
        assert_eq!(text(2), "K1ABC W9XYZ RRR");
        assert_eq!(text(3), "K1ABC W9XYZ RR73");
        assert_eq!(text(4), "K1ABC W9XYZ 73");
        assert_eq!(text(5), "CQ W9XYZ EN37");
        assert_eq!(text(6), "K1ABC W9XYZ EN37");
        assert_eq!(text(7), "K1ABC W9XYZ -50");
        assert_eq!(text(8), "K1ABC W9XYZ R-50");
        assert_eq!(text(206), "K1ABC W9XYZ R+49");
        let cq = |i| unpack77(&a7_message(i, "CQ", "W9XYZ", "EN37").unwrap()).unwrap();
        assert_eq!(cq(5), "CQ W9XYZ EN37");
        assert!(cq(1).starts_with("QU1RK W9XYZ"));
    }

    #[test]
    fn a8_list_stops_at_30_db() {
        assert!(a8_message(7, "K1ABC", "W9XYZ", "EN37").is_none()); // -50
        let (isnr, _) = report_of(7 + 2 * 20);
        assert_eq!(isnr, -30);
        assert!(a8_message(7 + 2 * 20, "K1ABC", "W9XYZ", "EN37").is_some());
    }

    #[test]
    fn smo121_is_the_fortran_one() {
        let mut x = [0.0, 4.0, 0.0, 0.0, 8.0];
        smo121(&mut x);
        assert_eq!(x, [0.0, 2.0, 1.0, 2.0, 8.0]);
    }
}
