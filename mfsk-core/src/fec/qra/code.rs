//! Generic Q-ary Repeat-Accumulate (QRA) code framework.
//!
//! Ports `lib/qra/qracodes/qracodes.c` from the WSJT-X distribution
//! (the qracodes library by Nico Palermo IV3NWV). One [`QraCode`]
//! instance bundles every constant the encoder + belief-propagation
//! decoder needs for one specific code; concrete codes such as
//! `qra15_65_64_irr_e23` (Q65's FEC) live in their own modules and
//! are exposed as `pub static` [`QraCode`] tables.
//!
//! ## API shape
//!
//! - [`QraCode::encode`] — systematic encoder, K info → N codeword
//!   symbols, all over GF(M).
//! - [`QraCode::mfsk_bessel_metric`] — convert M-FSK squared-amplitude
//!   observations into intrinsic per-symbol probability distributions.
//!   Used by the Q65 receiver before BP.
//! - [`QraCode::extrinsic`] — non-binary belief propagation in the
//!   probability domain, using Walsh-Hadamard transforms for
//!   variable-to-check and check-to-variable updates over GF(M).
//! - [`QraCode::map_decode`] — final maximum-a-posteriori hard
//!   decision: pointwise multiply intrinsic × extrinsic, then argmax
//!   each information symbol's distribution.
//!
//! ## Naming
//!
//! Field names match the C reference (uppercase `K`, `N`, `M`, `NC`,
//! `V`, `C`, `NMSG`, `MAXVDEG`, `MAXCDEG`) so that `grep`-ing across
//! the Rust port and the upstream source stays trivial. The
//! `non_snake_case` lint is silenced for those fields.

use super::{npfwht, pdmath};

/// Code type discriminator, mirroring the C `QRATYPE_*` defines from
/// `lib/qra/q65/qracodes.h`.
///
/// - [`QraCodeType::Normal`] — every codeword symbol carries data.
/// - [`QraCodeType::Crc`] — the last information symbol is a CRC-6
///   over the others; transmitted normally.
/// - [`QraCodeType::CrcPunctured`] — the trailing CRC-6 symbol is
///   not transmitted.
/// - [`QraCodeType::CrcPunctured2`] — the trailing **two** CRC-12
///   symbols are not transmitted (this is the variant used by Q65).
#[repr(i32)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum QraCodeType {
    Normal = 0,
    Crc = 1,
    CrcPunctured = 2,
    CrcPunctured2 = 3,
}

/// Static description of a single QRA code: parameters + the precomputed
/// tables consumed by the encoder and BP decoder.
#[allow(non_snake_case)]
pub struct QraCode {
    /// Number of information symbols.
    pub K: usize,
    /// Codeword length in symbols.
    pub N: usize,
    /// Bits per symbol (M = 2^m).
    pub m: u32,
    /// Symbol alphabet cardinality.
    pub M: usize,
    /// Code grouping factor (1 for Q65).
    pub a: usize,
    /// Number of check symbols (N − K).
    pub NC: usize,
    /// Number of variables in the code graph (= N).
    pub V: usize,
    /// Number of factor nodes (= N + NC + 1).
    pub C: usize,
    /// Number of edges / messages in the bipartite graph.
    pub NMSG: usize,
    /// Maximum variable-node degree in the graph.
    pub MAXVDEG: usize,
    /// Maximum factor-node (check) degree in the graph.
    pub MAXCDEG: usize,
    /// Code type (Normal / CRC / CRC-punctured).
    pub code_type: QraCodeType,
    /// Code rate K / N.
    pub R: f32,
    /// Human-readable code name (e.g. `"qra15_65_64_irr_e23"`).
    pub name: &'static str,

    // ── Encoder tables ──────────────────────────────────────────────
    /// `acc_input_idx[k]` — index into `x[]` of the systematic input
    /// symbol entering the accumulator at check `k`. Length
    /// `NC * a + 1` (the +1 entry is the terminator check used in
    /// debug builds; it is harmless to have it present).
    pub acc_input_idx: &'static [i32],
    /// `acc_input_wlog[k]` — log-α weight (in GF(M)*) applied to that
    /// input before XOR-accumulation.
    pub acc_input_wlog: &'static [i32],
    /// GF(M) discrete log: `gflog[x] = log_α(x)` for `x != 0`.
    pub gflog: &'static [i32],
    /// GF(M) discrete exp: `gfexp[i] = α^i` for `0 ≤ i < M − 1`.
    pub gfexp: &'static [i32],

    // ── Decoder tables ──────────────────────────────────────────────
    /// `msgw[i]` — log-α weight attached to message edge `i`.
    pub msgw: &'static [i32],
    /// `vdeg[v]` — degree of variable node `v`.
    pub vdeg: &'static [i32],
    /// `cdeg[c]` — degree of factor node `c`.
    pub cdeg: &'static [i32],
    /// `v2cmidx[v * MAXVDEG + k]` — message index for the k-th edge
    /// leaving variable `v`. Length `V * MAXVDEG`.
    pub v2cmidx: &'static [i32],
    /// `c2vmidx[c * MAXCDEG + k]` — message index for the k-th edge
    /// leaving check `c`. Length `C * MAXCDEG`.
    pub c2vmidx: &'static [i32],
    /// `gfpmat[w * M + k]` — multiplication-by-`α^w` permutation
    /// table over GF(M). Length `(M − 1) * M`.
    pub gfpmat: &'static [i32],
}

/// Outcome of [`QraCode::extrinsic`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ExtrinsicResult {
    /// Iterations converged: every symbol's extrinsic distribution is
    /// concentrated on a single value (sum of per-symbol max ≈ V).
    Converged { iterations: u32 },
    /// `maxiter` was hit without convergence. The output `pex` still
    /// holds the best-effort distribution and the caller can pass it
    /// to [`QraCode::map_decode`] for a possibly-correct hard decision
    /// (often used as the front-end for an outer CRC check).
    NotConverged,
    /// A factor node had degree 1 — code-table pathology. Should not
    /// happen for the codes shipped in this crate.
    BadCodeTables,
}

/// Reusable scratch buffers for the BP decoder. Allocated once per
/// [`QraCode`] and reused across decode calls — these arrays are
/// sized in `O(NMSG · M)` floats which is non-trivial (~55 kB for
/// Q65), so caller-side reuse matters.
pub struct DecoderScratch {
    v2cmsg: Vec<f32>,
    c2vmsg: Vec<f32>,
    msgout: Vec<f32>,
}

impl DecoderScratch {
    /// Allocate scratch for the given code. Cheap to keep on a worker
    /// or thread-local for the lifetime of a decoding session.
    pub fn for_code(code: &QraCode) -> Self {
        let edges = code.NMSG * code.M;
        Self {
            v2cmsg: vec![0.0; edges],
            c2vmsg: vec![0.0; edges],
            msgout: vec![0.0; code.M],
        }
    }
}

impl QraCode {
    // ────────────────────────────────────────────────────────────────
    // Encoder
    // ────────────────────────────────────────────────────────────────

    /// Systematic encode: copy `x` (K info symbols over GF(M)) into the
    /// first K positions of `y`, then compute `NC` parity symbols by
    /// running the QRA accumulator chain across `acc_input_idx /
    /// acc_input_wlog`.
    ///
    /// `x.len()` must equal `self.K`; `y.len()` must equal `self.N`.
    /// All symbol values must be in `0 .. self.M`.
    pub fn encode(&self, x: &[i32], y: &mut [i32]) {
        assert_eq!(x.len(), self.K, "encode: x.len() != K");
        assert_eq!(y.len(), self.N, "encode: y.len() != N");

        // Systematic prefix.
        y[..self.K].copy_from_slice(x);
        let modulus = (self.M - 1) as i32;

        let mut chk: i32 = 0;
        if self.a == 1 {
            for k in 0..self.NC {
                let idx = self.acc_input_idx[k] as usize;
                let t = x[idx];
                if t != 0 {
                    let lg = (self.gflog[t as usize] + self.acc_input_wlog[k]).rem_euclid(modulus);
                    chk ^= self.gfexp[lg as usize];
                }
                y[self.K + k] = chk;
            }
        } else {
            // Irregular grouping: a > 1, with -1 entries skipped.
            for k in 0..self.NC {
                let kk = self.a * k;
                for j in 0..self.a {
                    let jj = kk + j;
                    let idx = self.acc_input_idx[jj];
                    if idx < 0 {
                        continue;
                    }
                    let t = x[idx as usize];
                    if t != 0 {
                        let lg =
                            (self.gflog[t as usize] + self.acc_input_wlog[jj]).rem_euclid(modulus);
                        chk ^= self.gfexp[lg as usize];
                    }
                }
                y[self.K + k] = chk;
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Front-end: M-FSK Bessel metric
    // ────────────────────────────────────────────────────────────────

    /// Convert per-tone squared amplitudes into per-symbol intrinsic
    /// probability distributions, assuming non-coherent M-FSK
    /// reception in AWGN.
    ///
    /// `rsq.len()` and `pix.len()` must both equal `M * N`. The
    /// symbol-`k` row occupies `pix[M*k .. M*(k+1)]`. Returns the
    /// estimated noise standard deviation `σ̂`, useful as a quality
    /// indicator.
    ///
    /// The metric is calibrated for `Es/No = es_no_metric` (linear,
    /// not dB). Calling code commonly uses a fixed value (~2.8 dB
    /// linearised) chosen to optimise mean WER — Q65 follows the
    /// same convention.
    pub fn mfsk_bessel_metric(&self, pix: &mut [f32], rsq: &[f32], es_no_metric: f32) -> f32 {
        let big_m = self.M;
        let big_n = self.N;
        let nsamples = big_m * big_n;
        assert_eq!(pix.len(), nsamples, "mfsk_bessel_metric: pix length");
        assert_eq!(rsq.len(), nsamples, "mfsk_bessel_metric: rsq length");

        let mut rsum = 0.0_f32;
        for k in 0..nsamples {
            rsum += rsq[k];
            pix[k] = rsq[k].sqrt();
        }
        rsum /= nsamples as f32;

        let sigmaest = (rsum / (1.0 + es_no_metric / big_m as f32) / 2.0).sqrt();
        let cmetric = (2.0 * es_no_metric).sqrt() / sigmaest;

        for k in 0..big_n {
            let row = &mut pix[big_m * k..big_m * (k + 1)];
            ioapprox(row, cmetric);
            pdmath::norm(row);
        }

        sigmaest
    }

    // ────────────────────────────────────────────────────────────────
    // Belief-propagation decoder
    // ────────────────────────────────────────────────────────────────

    /// Iterative non-binary BP: given per-symbol intrinsic probability
    /// distributions `pix` (length `M·V`), iterate until each
    /// variable's outgoing extrinsic distribution is concentrated on
    /// one value or `maxiter` is exhausted.
    ///
    /// On return, `pex` (length `M·V`) holds the extrinsic
    /// distributions — pass them together with `pix` to
    /// [`Self::map_decode`] for the final hard decision.
    ///
    /// `scratch` must come from [`DecoderScratch::for_code(self)`].
    pub fn extrinsic(
        &self,
        pex: &mut [f32],
        pix: &[f32],
        maxiter: u32,
        scratch: &mut DecoderScratch,
    ) -> ExtrinsicResult {
        let big_m = self.M;
        let v = self.V;
        let c = self.C;
        let max_vdeg = self.MAXVDEG;
        let max_cdeg = self.MAXCDEG;

        assert_eq!(pex.len(), big_m * v, "extrinsic: pex length");
        assert_eq!(pix.len(), big_m * v, "extrinsic: pix length");
        assert_eq!(
            scratch.v2cmsg.len(),
            self.NMSG * big_m,
            "extrinsic: scratch belongs to a different code"
        );

        // Initial messages: c->v copy of the intrinsics for the
        // first V check nodes (which are the intrinsic factors), and
        // the v->c outbound copy of the intrinsic for every non-trivial
        // variable degree.
        let init_len = big_m * v;
        scratch.c2vmsg[..init_len].copy_from_slice(pix);

        for nv in 0..v {
            let ndeg = self.vdeg[nv] as usize;
            let msgbase = nv * max_vdeg;
            let intrinsic_row = &pix[big_m * nv..big_m * (nv + 1)];
            for k in 1..ndeg {
                let imsg = self.v2cmidx[msgbase + k] as usize;
                let dst = &mut scratch.v2cmsg[big_m * imsg..big_m * (imsg + 1)];
                dst.copy_from_slice(intrinsic_row);
            }
        }

        let mut rc = ExtrinsicResult::NotConverged;

        for nit in 0..maxiter {
            // ── c->v step ──────────────────────────────────────────
            // Walsh-Hadamard convolution: for parity x1 + x2 + ... = 0
            // over GF(M), Prob(x1) = WHT^{-1}( ∏ WHT(Prob(xi)) ) for
            // i != 1. Output is normalised + permuted by α^{-w}.
            for nc in v..c {
                let ndeg = self.cdeg[nc] as usize;
                if ndeg == 1 {
                    return ExtrinsicResult::BadCodeTables;
                }
                let msgbase = nc * max_cdeg;

                // Forward WHT on every input message in place. After
                // this loop the v2cmsg rows feeding this check are in
                // the WHT "frequency" domain.
                for k in 0..ndeg {
                    let imsg = self.c2vmidx[msgbase + k] as usize;
                    let row = &mut scratch.v2cmsg[big_m * imsg..big_m * (imsg + 1)];
                    npfwht::fwht(row);
                }

                for k in 0..ndeg {
                    scratch.msgout.copy_from_slice(pdmath::uniform(big_m));

                    for kk in 0..ndeg {
                        if kk != k {
                            let imsg = self.c2vmidx[msgbase + kk] as usize;
                            let src = &scratch.v2cmsg[big_m * imsg..big_m * (imsg + 1)];
                            pdmath::imul(&mut scratch.msgout, src);
                        }
                    }

                    // Anti-underflow bias on the DC term, copied
                    // verbatim from the reference: keeps the sum of
                    // post-WHT components strictly positive.
                    scratch.msgout[0] += 1e-7;

                    npfwht::fwht(&mut scratch.msgout);

                    let imsg = self.c2vmidx[msgbase + k] as usize;
                    let wmsg = self.msgw[imsg];
                    let dst = &mut scratch.c2vmsg[big_m * imsg..big_m * (imsg + 1)];

                    if wmsg == 0 {
                        dst.copy_from_slice(&scratch.msgout);
                    } else {
                        let perm = self.gfpmat_row(wmsg as usize);
                        pdmath::bwdperm(dst, &scratch.msgout, perm);
                    }
                }
            }

            // ── v->c step ──────────────────────────────────────────
            // Pointwise product of incoming c->v messages, normalise,
            // then permute by α^{w}.
            for nv in 0..v {
                let ndeg = self.vdeg[nv] as usize;
                let msgbase = nv * max_vdeg;

                for k in 0..ndeg {
                    scratch.msgout.copy_from_slice(pdmath::uniform(big_m));

                    for kk in 0..ndeg {
                        if kk != k {
                            let imsg = self.v2cmidx[msgbase + kk] as usize;
                            let src = &scratch.c2vmsg[big_m * imsg..big_m * (imsg + 1)];
                            pdmath::imul(&mut scratch.msgout, src);
                        }
                    }

                    pdmath::norm(&mut scratch.msgout);

                    let imsg = self.v2cmidx[msgbase + k] as usize;
                    let wmsg = self.msgw[imsg];
                    let dst = &mut scratch.v2cmsg[big_m * imsg..big_m * (imsg + 1)];

                    if wmsg == 0 {
                        dst.copy_from_slice(&scratch.msgout);
                    } else {
                        let perm = self.gfpmat_row(wmsg as usize);
                        pdmath::fwdperm(dst, &scratch.msgout, perm);
                    }
                }
            }

            // ── Convergence check ──────────────────────────────────
            // Each variable's first outgoing message (msg index nv) is
            // the extrinsic distribution towards the rest of the code.
            // If its max approaches 1 for every variable, the EXIT
            // chart has reached (1, 1) and we stop.
            let mut totex = 0.0_f32;
            for nv in 0..v {
                let row = &scratch.v2cmsg[big_m * nv..big_m * (nv + 1)];
                totex += pdmath::max(row);
            }

            if totex > v as f32 - 0.01 {
                rc = ExtrinsicResult::Converged { iterations: nit };
                break;
            }
        }

        // Copy the V outgoing extrinsics into the caller's buffer.
        pex.copy_from_slice(&scratch.v2cmsg[..big_m * v]);
        rc
    }

    /// MAP hard decision on the K information symbols.
    ///
    /// Multiplies extrinsic × intrinsic for each variable, then
    /// argmax. Destroys `pex` in place. `xdec.len()` must equal
    /// `self.K`.
    pub fn map_decode(&self, pex: &mut [f32], pix: &[f32], xdec: &mut [i32]) {
        let big_m = self.M;
        assert_eq!(pex.len(), big_m * self.V, "map_decode: pex length");
        assert_eq!(pix.len(), big_m * self.V, "map_decode: pix length");
        assert_eq!(xdec.len(), self.K, "map_decode: xdec length");

        for k in 0..self.K {
            let row_e = &mut pex[big_m * k..big_m * (k + 1)];
            let row_i = &pix[big_m * k..big_m * (k + 1)];
            pdmath::imul(row_e, row_i);
            // Probability distributions are non-negative; the
            // all-negative argmax case is unreachable here.
            let (_, idx) = pdmath::argmax(row_e).expect("MAP decode: empty distribution");
            xdec[k] = idx as i32;
        }
    }

    /// `gfpmat[w * M .. (w+1) * M]` — the permutation that realises
    /// "multiply distribution by α^w" in the probability domain.
    fn gfpmat_row(&self, logw: usize) -> &[i32] {
        let m = self.M;
        let start = logw * m;
        &self.gfpmat[start..start + m]
    }
}

/// In-place rational approximation of `exp(log I0(C·x))` — the
/// modified Bessel function of the first kind, order 0, applied to
/// each element of `buf`. Matches the polynomial in
/// `qra_ioapprox()` from the C reference.
fn ioapprox(buf: &mut [f32], c: f32) {
    for x in buf.iter_mut() {
        let v = *x * c;
        let vsq = v * v;
        let mut v = vsq * (v + 0.039) / (vsq * 0.9931 + v * 2.6936 + 0.5185);
        if v > 80.0 {
            v = 80.0;
        }
        *x = v.exp();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A trivial GF(2) repetition code (5, 1) for end-to-end testing
    /// of the encoder. Won't exercise BP — that needs a real QRA
    /// table — but it pins the systematic-prefix / accumulator
    /// behaviour against a hand-computable case.
    #[test]
    fn ioapprox_monotonic_increasing() {
        // I0(C·x) is monotonically increasing in x for x >= 0, so
        // the rational approximation must respect that ordering.
        let mut buf = [0.0_f32, 0.5, 1.0, 2.0, 4.0];
        ioapprox(&mut buf, 1.0);
        for w in buf.windows(2) {
            assert!(
                w[0] <= w[1],
                "ioapprox not monotonic: {:?} > {:?}",
                w[0],
                w[1]
            );
        }
    }

    #[test]
    fn ioapprox_clamps_at_80() {
        // Very large C·x must clamp the exponent to 80 to avoid
        // f32 overflow.
        let mut buf = [10.0_f32; 4];
        ioapprox(&mut buf, 100.0);
        let max_allowed = 80.0_f32.exp() * 1.001;
        for &v in &buf {
            assert!(v.is_finite(), "overflowed to {v}");
            assert!(v <= max_allowed, "value {v} exceeds clamp");
        }
    }
}
