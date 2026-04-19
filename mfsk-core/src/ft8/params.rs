//! FT8 protocol parameters (derived from WSJT-X ft8_params.f90)

/// LDPC (174,91): information bits = 77 msg + 14 CRC
pub const KK: usize = 91;
/// Data symbols
pub const ND: usize = 58;
/// Sync symbols (3 Costas 7×7 arrays)
pub const NS: usize = 21;
/// Total channel symbols
pub const NN: usize = NS + ND; // 79
/// Samples per symbol at 12000 S/s
pub const NSPS: usize = 1920;
/// Samples in a full 15-second waveform
pub const NZ: usize = NSPS * NN; // 151,680
/// Samples in the receive buffer (15 s × 12000 S/s)
pub const NMAX: usize = 15 * 12000; // 180,000
/// FFT size for symbol spectra (2 × NSPS)
pub const NFFT1: usize = 2 * NSPS; // 3840
pub const NH1: usize = NFFT1 / 2; // 1920
/// Rough time-sync step size (1/4 symbol)
pub const NSTEP: usize = NSPS / 4; // 480
/// Number of symbol spectra at 1/4-symbol steps
pub const NHSYM: usize = NMAX / NSTEP - 3; // 357

/// Downsample factor: 12000 Hz → 200 Hz
pub const NDOWN: usize = 60;
/// Downsampled sample rate (Hz)
pub const DS_RATE: usize = 12000 / NDOWN; // 200
/// Samples per symbol after downsampling
pub const DS_SPB: usize = NSPS / NDOWN; // 32
/// Downsampled buffer length
pub const NP2: usize = 3200;

/// LDPC codeword length
pub const LDPC_N: usize = 174;
/// LDPC information bits (message + CRC14)
pub const LDPC_K: usize = 91;
/// LDPC parity bits
pub const LDPC_M: usize = LDPC_N - LDPC_K; // 83

/// Message payload bits (before CRC)
pub const MSG_BITS: usize = 77;
/// CRC width
pub const CRC_BITS: usize = 14;

/// Costas array tone pattern
pub const COSTAS: [usize; 7] = [3, 1, 4, 0, 6, 5, 2];

/// Symbol offsets of the three Costas arrays within a frame
/// Positions: 0-6, 36-42, 72-78
pub const COSTAS_POS: [usize; 3] = [0, 36, 72];

/// Number of tones in FT8 (8-FSK)
pub const NTONES: usize = 8;

/// Symbol duration (seconds)
pub const SYMBOL_DT: f32 = NSPS as f32 / 12000.0; // 0.16 s

/// Time step at 1/4-symbol resolution (seconds)
pub const TSTEP: f32 = SYMBOL_DT / 4.0; // 0.04 s

/// Frequency resolution at NFFT1 (Hz/bin)
pub const DF: f32 = 12000.0 / NFFT1 as f32; // 3.125 Hz

/// Gray code map for 8-FSK (tone index → 3-bit Gray code)
pub const GRAYMAP: [usize; 8] = [0, 1, 3, 2, 5, 6, 4, 7];

/// LLR scale factor (empirically tuned in WSJT-X ft8b.f90)
pub const LLR_SCALE: f32 = 2.83;

/// Maximum BP decoder iterations
pub const BP_MAX_ITER: u32 = 30;
