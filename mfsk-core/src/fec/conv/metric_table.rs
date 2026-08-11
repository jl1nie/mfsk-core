// SPDX-License-Identifier: GPL-3.0-or-later
//! wsprd's empirical Fano branch-metric table.
//!
//! Verbatim row 2 of `lib/wsprd/metric_tables.c` — the row
//! `wsprd.c:912-913` selects for WSPR. Entry `i` is the log-likelihood
//! (in bits) of receiving normalised soft symbol `i ∈ [0, 255]` given
//! that a **0** was sent; the bit-1 metric reads the same table
//! backwards (`255 − i`).
//!
//! This is *measured*, not analytic, and it is strongly asymmetric and
//! saturating: `T[0] = +1.0` but `T[255] = −13.25`. That saturation is
//! the whole point. A metric linear in the soft symbol lets a single
//! confident-but-wrong symbol dominate the path metric, and Fano —
//! being a sequential search that commits to a path and only
//! backtracks on a threshold violation — then settles on a wrong
//! codeword rather than failing. WSPR carries no CRC, so nothing
//! downstream catches that. See `docs/notes/SNR_FORMULAS.md` and
//! issue #275 for the measurement that pinned this down.
//!
//! `ConvFano232` (JT9) already carried its own equivalent LUT; WSPR's
//! `ConvFano` was the one still approximating.

// This is a verbatim data table, so an entry that happens to sit near a
// mathematical constant (0.7854 ≈ π/4) is a coincidence, not an
// approximation someone should "fix".
#![allow(clippy::approx_constant)]

/// `metric_tables[2]` from `lib/wsprd/metric_tables.c`.
pub(crate) static WSPRD_METRIC_TABLE: [f32; 256] = [
    0.9999, 0.9998, 0.9998, 0.9998, 0.9998, 0.9998, 0.9997, 0.9997, 0.9997, 0.9997, 0.9997, 0.9996,
    0.9996, 0.9996, 0.9995, 0.9995, 0.9994, 0.9994, 0.9994, 0.9993, 0.9993, 0.9992, 0.9991, 0.9991,
    0.999, 0.9989, 0.9988, 0.9988, 0.9988, 0.9986, 0.9985, 0.9984, 0.9983, 0.9982, 0.998, 0.9979,
    0.9977, 0.9976, 0.9974, 0.9971, 0.9969, 0.9968, 0.9965, 0.9962, 0.996, 0.9957, 0.9953, 0.995,
    0.9947, 0.9941, 0.9937, 0.9933, 0.9928, 0.9922, 0.9917, 0.9911, 0.9904, 0.9897, 0.989, 0.9882,
    0.9874, 0.9863, 0.9855, 0.9843, 0.9832, 0.9819, 0.9806, 0.9792, 0.9777, 0.976, 0.9743, 0.9724,
    0.9704, 0.9683, 0.9659, 0.9634, 0.9609, 0.9581, 0.955, 0.9516, 0.9481, 0.9446, 0.9406, 0.9363,
    0.9317, 0.927, 0.9218, 0.916, 0.9103, 0.9038, 0.8972, 0.8898, 0.8822, 0.8739, 0.8647, 0.8554,
    0.8457, 0.8357, 0.8231, 0.8115, 0.7984, 0.7854, 0.7704, 0.7556, 0.7391, 0.721, 0.7038, 0.684,
    0.6633, 0.6408, 0.6174, 0.5939, 0.5678, 0.541, 0.5137, 0.4836, 0.4524, 0.4193, 0.385, 0.3482,
    0.3132, 0.2733, 0.2315, 0.1891, 0.1435, 0.098, 0.0493, 0.0, -0.051, -0.1052, -0.1593, -0.2177,
    -0.2759, -0.3374, -0.4005, -0.4599, -0.5266, -0.5935, -0.6626, -0.7328, -0.8051, -0.8757,
    -0.9498, -1.0271, -1.1019, -1.1816, -1.2642, -1.3459, -1.4295, -1.5077, -1.5958, -1.6818,
    -1.7647, -1.8548, -1.9387, -2.0295, -2.1152, -2.2154, -2.3011, -2.3904, -2.482, -2.5786,
    -2.673, -2.7652, -2.8616, -2.9546, -3.0526, -3.1445, -3.2445, -3.3416, -3.4357, -3.5325,
    -3.6324, -3.7313, -3.8225, -3.9209, -4.0248, -4.1278, -4.2261, -4.3193, -4.422, -4.5262,
    -4.6214, -4.7242, -4.8234, -4.9245, -5.0298, -5.125, -5.2232, -5.3267, -5.4332, -5.5342,
    -5.6431, -5.727, -5.8401, -5.935, -6.0407, -6.1418, -6.2363, -6.3384, -6.4536, -6.5429,
    -6.6582, -6.7433, -6.8438, -6.9478, -7.0789, -7.1894, -7.2714, -7.3815, -7.481, -7.5575,
    -7.6852, -7.8071, -7.858, -7.9724, -8.1, -8.2207, -8.2867, -8.4017, -8.5287, -8.6347, -8.7082,
    -8.8319, -8.9448, -9.0355, -9.1885, -9.2095, -9.2863, -9.4186, -9.5064, -9.6386, -9.7207,
    -9.8286, -9.9453, -10.0701, -10.1735, -10.3001, -10.2858, -10.5427, -10.5982, -10.7361,
    -10.7042, -10.9212, -11.0097, -11.0469, -11.1155, -11.2812, -11.3472, -11.4988, -11.5327,
    -11.6692, -11.9376, -11.8606, -12.1372, -13.2539,
];
