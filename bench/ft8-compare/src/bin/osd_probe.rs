// SPDX-License-Identifier: GPL-3.0-only
//! Feed the LLR vectors jt9's ft8b computed (ft8_trace.dat of the JT9_TRACE build) for the
//! candidate that decoded <message> into mfsk-core's own `bp_llr_zsum` -> `osd174_91`, the
//! pair `ft8::decode_block::osd_strategy::try_fallback` uses, and print what comes out.
//! If our OSD decodes the same message from jt9's own input, the difference is upstream of
//! the OSD (the LLRs); if not, it is in the BP zsum or the OSD.
//!
//! usage: osd_probe <ft8_trace.dat> <message>
//!        osd_probe --ours <rtrace> <stage A|C> <f_hz>     (the same table for OUR llr dump of that candidate)
use mfsk_core::fec::ldpc::bp::{BpScratch, bp_llr_zsum_with_scratch};
use mfsk_core::fec::ldpc::osd_npre::{NDEEP2_174_91, osd174_91};
use mfsk_core::fec::ldpc::params::Ldpc174_91Params;
use mfsk_core::msg::wsjt77::unpack77;

fn norm(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ").to_uppercase()
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a[1] == "--ours" {
        let (stage, f0): (&str, f32) = (&a[3], a[4].parse().unwrap());
        let text = std::fs::read_to_string(&a[2]).unwrap();
        let mut blocks: Vec<(String, Vec<Vec<f32>>)> = Vec::new();
        for l in text.lines().filter(|l| l.starts_with("LLR ")) {
            let (head, _, vals) = l.partition_point_helper();
            let t: Vec<&str> = head.split_whitespace().collect();
            if t[1] != stage { continue; }
            let f: f32 = t[3].trim_start_matches("f=").parse().unwrap();
            if (f - f0).abs() > 0.6 { continue; }
            let (k, rest) = vals.split_once(' ').unwrap();
            let v: Vec<f32> = rest.split_whitespace().map(|x| x.parse().unwrap()).collect();
            if k == "1" { blocks.push((format!("{} {} {}", t[1], t[2], t[4]), Vec::new())); }
            blocks.last_mut().unwrap().1.push(v);
        }
        let mut scratch = BpScratch::<Ldpc174_91Params, f32>::new();
        for (name, sets) in blocks {
            println!("our llr dump {name}");
            for (k, s) in sets.iter().enumerate() {
                let llr: [f32; 174] = s.clone().try_into().unwrap();
                for n_iter in [1u32, 2] {
                    let z = bp_llr_zsum_with_scratch::<Ldpc174_91Params>(&mut scratch, &llr, n_iter).to_vec();
                    let z: [f32; 174] = z.try_into().unwrap();
                    let out = match osd174_91(&z, NDEEP2_174_91, None) {
                        Some(o) => format!("{:?} hard_errors={}", unpack77(&o.message77), o.hard_errors),
                        None => "none".to_string(),
                    };
                    println!("  llr{} zsum{n_iter}: {out}", (b'a' + k as u8) as char);
                }
            }
        }
        return;
    }
    let want = norm(&a[2]);
    let text = std::fs::read_to_string(&a[1]).unwrap();
    let mut block: Vec<Vec<f32>> = Vec::new();
    let mut found = None;
    for l in text.lines() {
        let t: Vec<&str> = l.split_whitespace().collect();
        if t.is_empty() { continue; }
        if t[0] == "LLR" {
            if t[5] == "1" { block.clear(); }
            block.push(t[6..6 + 174].iter().map(|x| x.parse().unwrap()).collect());
        } else if t[0] == "DEC" && norm(&t[11..].join(" ")) == want && block.len() == 5 {
            found = Some((t[3].to_string(), t[6].to_string(), block.clone()));
            break;
        }
    }
    let (set, herr, sets) = found.expect("jt9 DEC not found");
    println!("jt9 decoded {want:?} at llr set {set} (OSD), hard errors {herr}");
    let mut scratch = BpScratch::<Ldpc174_91Params, f32>::new();
    for (k, s) in sets.iter().enumerate() {
        let llr: [f32; 174] = s.clone().try_into().unwrap();
        for n_iter in [1u32, 2] {
            let z = bp_llr_zsum_with_scratch::<Ldpc174_91Params>(&mut scratch, &llr, n_iter).to_vec();
            let z: [f32; 174] = z.try_into().unwrap();
            let r = osd174_91(&z, NDEEP2_174_91, None);
            let out = match r {
                Some(o) => format!("{:?} hard_errors={}", unpack77(&o.message77), o.hard_errors),
                None => "none".to_string(),
            };
            println!("  our OSD on jt9's llr{} zsum{n_iter}: {out}", (b'a' + k as u8) as char);
        }
    }
}

trait SplitSet { fn partition_point_helper(&self) -> (&str, &str, &str); }
impl SplitSet for str {
    /// ("LLR C imetric=1 f=.. dt=..", " set=", "<k> <values>")
    fn partition_point_helper(&self) -> (&str, &str, &str) {
        let i = self.find(" set=").unwrap();
        (&self[..i], &self[i..i + 5], &self[i + 5..])
    }
}
