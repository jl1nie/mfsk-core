// SPDX-License-Identifier: GPL-3.0-only
//
// End-to-end C++ driver for the mfsk-ffi C ABI — encodes a known test
// message for every supported protocol, feeds the synthesised PCM
// back through the matching decoder handle (mfsk_decoder_*), and verifies the decoded
// text round-trips correctly. Doubles as smoke test for the ABI
// (NULL handling, last-error, sample lifetimes) and as proof that each
// mode is actually wired up in the C ABI.
//
// Decode results go into memory this file owns: nothing here frees a
// pointer the library allocated, which is the category the decoder handle
// keeps out of the decode path.
//
// Build: run `./build.sh`.

#include "mfsk.h"

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <algorithm>
#include <fstream>
#include <iterator>

namespace {

// Tally of failed sub-tests — reported at the end so one broken
// protocol doesn't hide the status of the others.
int g_failures = 0;

void fail(const char* proto, const char* detail) {
    std::fprintf(stderr, "  FAIL [%s] %s\n", proto, detail);
    g_failures++;
}

// ── Shared decode helpers ───────────────────────────────────────────

struct Rows {
    MfskDecode items[16];
    size_t len = 0;

    Rows() {
        std::memset(items, 0, sizeof items);
        for (auto& r : items) r.size = sizeof r;
    }
    bool contains(const char* needle) const {
        for (size_t i = 0; i < len; ++i) {
            if (std::strstr(items[i].text, needle) != nullptr) return true;
        }
        return false;
    }
};

void print_rows(const char* tag, const Rows& r) {
    std::printf("  [%s] %zu decode(s):\n", tag, r.len);
    for (size_t i = 0; i < r.len; ++i) {
        const MfskDecode& m = r.items[i];
        std::printf("    freq=%7.2f dt=%+.3f snr=%+.1f err=%u pass=%u text='%s'\n",
                    m.freq_hz, m.dt_sec, m.snr_db, m.hard_errors, m.pass, m.text);
    }
}

using Encoder = MfskStatus (*)(const char*, const char*, const char*, float,
                               float*, size_t, size_t*);

/// The three-stage pipeline: pack → tones → PCM, each into a buffer the
/// caller sized from `mfsk_symbol_count` / `mfsk_synth_output_len`.
/// Nothing is allocated across the boundary and nothing is freed.
std::vector<int16_t> synth_frame(MfskMode mode, const char* a, const char* b,
                                 const char* c, float freq) {
    std::vector<int16_t> out;
    uint8_t msg[77];
    if (mfsk_pack77(a, b, c, msg) != MFSK_STATUS_OK) {
        fail(mfsk_mode_name(mode), "mfsk_pack77 failed");
        return out;
    }
    const size_t n_tones = mfsk_symbol_count(mode);
    if (n_tones == 0) {
        fail(mfsk_mode_name(mode), "no tone stage");
        return out;
    }
    std::vector<uint8_t> tones(n_tones);
    size_t got = 0;
    if (mfsk_message_to_tones(mode, msg, tones.data(), tones.size(), &got) != MFSK_STATUS_OK) {
        fail(mfsk_mode_name(mode), mfsk_last_error());
        return out;
    }
    const size_t n_pcm = mfsk_synth_output_len(mode);
    out.resize(n_pcm);
    size_t wrote = 0;
    if (mfsk_tones_to_i16(mode, tones.data(), tones.size(), freq, 8000,
                          out.data(), out.size(), &wrote) != MFSK_STATUS_OK) {
        fail(mfsk_mode_name(mode), mfsk_last_error());
        out.clear();
    }
    return out;
}

/// A frame placed in a full slot at that mode's TX offset.
std::vector<int16_t> synth_slot(MfskMode mode, const char* a, const char* b,
                                const char* c, float freq) {
    MfskModeInfo info;
    std::memset(&info, 0, sizeof info);
    info.size = sizeof info;
    if (mfsk_mode_info(mode, &info) != MFSK_STATUS_OK) return {};
    const std::vector<int16_t> frame = synth_frame(mode, a, b, c, freq);
    std::vector<int16_t> slot(info.slot_samples_12k, 0);
    const size_t start = static_cast<size_t>(info.tx_start_offset_s * 12000.0f);
    for (size_t i = 0; i < frame.size() && start + i < slot.size(); ++i) {
        slot[start + i] = frame[i];
    }
    return slot;
}

MfskParams params_for(MfskMode mode) {
    MfskParams p;
    std::memset(&p, 0, sizeof p);
    p.size = sizeof p;
    if (mfsk_params_init(mode, &p) != MFSK_STATUS_OK) {
        fail(mfsk_mode_name(mode), "mfsk_params_init failed");
    }
    return p;
}

MfskExtras extras_init() {
    MfskExtras e;
    std::memset(&e, 0, sizeof e);
    e.size = sizeof e;
    if (mfsk_extras_init(&e) != MFSK_STATUS_OK) fail("extras", "mfsk_extras_init failed");
    return e;
}

/// Decode one period on `dec` into `rows`; reports the decoder's own error.
bool decode_i16(MfskDecoder* dec, const std::vector<int16_t>& audio, Rows& rows,
                const char* tag, int64_t period = MFSK_PERIOD_NONE) {
    rows.len = 0;
    if (mfsk_decoder_decode_i16(dec, audio.data(), audio.size(), 12000, period,
                                rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail(tag, mfsk_decoder_last_error(dec));
        return false;
    }
    return true;
}

bool decode_f32(MfskDecoder* dec, const std::vector<float>& audio, Rows& rows,
                const char* tag) {
    rows.len = 0;
    if (mfsk_decoder_decode_f32(dec, audio.data(), audio.size(), 12000,
                                MFSK_PERIOD_NONE, rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail(tag, mfsk_decoder_last_error(dec));
        return false;
    }
    return true;
}

/// Open a decoder with the mode's defaults (or the given blocks).
MfskDecoder* open_dec(const char* tag, MfskMode mode, const MfskParams* p = nullptr,
                      const MfskExtras* e = nullptr) {
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecoder* d = mfsk_decoder_open(mode, p, e, &st);
    if (d == nullptr || st != MFSK_STATUS_OK) {
        fail(tag, mfsk_last_error());
        return nullptr;
    }
    return d;
}

/// Open a decoder, decode one slot, assert the text turns up.
void decoder_roundtrip(const char* tag, MfskMode mode,
                       const std::vector<int16_t>& audio, const char* needle) {
    MfskDecoder* d = open_dec(tag, mode);
    if (d == nullptr) return;
    Rows rows;
    if (decode_i16(d, audio, rows, tag)) {
        print_rows(tag, rows);
        if (!rows.contains(needle)) fail(tag, "expected text missing");
        for (size_t i = 0; i < rows.len; ++i) {
            if (rows.items[i].mode != mode) fail(tag, "row reports the wrong mode");
        }
    }
    mfsk_decoder_close(d);
}

/// A frame of f32 PCM placed `offset_s` into a `slot_s`-second period.
std::vector<float> put_in_slot(const std::vector<float>& frame, float offset_s, int slot_s) {
    std::vector<float> slot(static_cast<size_t>(slot_s) * 12000, 0.0f);
    const size_t at = static_cast<size_t>(offset_s * 12000.0f);
    for (size_t i = 0; i < frame.size() && at + i < slot.size(); ++i) slot[at + i] = frame[i];
    return slot;
}

std::vector<float> encode_f32(Encoder enc, const char* a, const char* b, const char* c,
                              float freq) {
    size_t need = 0;
    enc(a, b, c, freq, nullptr, 0, &need);
    std::vector<float> pcm(need);
    size_t got = 0;
    if (need == 0 || enc(a, b, c, freq, pcm.data(), pcm.size(), &got) != MFSK_STATUS_OK) {
        fail("encode", mfsk_last_error());
        return {};
    }
    pcm.resize(got);
    return pcm;
}

// ── Mode introspection ──────────────────────────────────────────────
//
// Written the way a consumer would: enumerate what the build has, ask each
// mode what it supports, and act on the answer. Anything asserted from a
// list written here would be testing this file, not the library.
void test_mode_introspection() {
    std::printf("\n— Mode introspection\n");

    const uint32_t abi = mfsk_abi_version();
    std::printf("  abi version: %u\n", abi);
    if (abi < 2) {
        fail("introspect", "mfsk_abi_version() predates the introspection surface");
        return;
    }

    const uint32_t n = mfsk_mode_count();
    if (n == 0) {
        fail("introspect", "this build claims to support no modes at all");
        return;
    }
    std::printf("  %u mode(s) in this build\n", n);

    int with_handle = 0, fst4_submodes = 0, snipers = 0;
    uint32_t widest_fft = 0;
    char widest_name[16] = {0};

    for (uint32_t i = 0; i < n; ++i) {
        MfskMode m;
        if (mfsk_mode_at(i, &m) != MFSK_STATUS_OK) {
            fail("introspect", "mfsk_mode_at failed inside 0..count");
            return;
        }

        MfskModeInfo info;
        std::memset(&info, 0, sizeof info);
        info.size = sizeof info;
        if (mfsk_mode_info(m, &info) != MFSK_STATUS_OK) {
            fail("introspect", "mfsk_mode_info failed for an enumerated mode");
            return;
        }

        const char* name = mfsk_mode_name(m);
        if (name == nullptr || std::strcmp(name, info.name) != 0) {
            fail("introspect", "mfsk_mode_name disagrees with MfskModeInfo::name");
            return;
        }
        MfskMode back;
        if (mfsk_mode_from_name(name, &back) != MFSK_STATUS_OK || back != m) {
            fail(name, "did not round-trip through mfsk_mode_from_name");
            return;
        }

        if (info.caps & MFSK_CAP_DECODE_HANDLE) {
            with_handle++;
            // Anything the decoder drives must publish a usable default
            // parameter block, or a caller has nothing to start from.
            MfskParams p;
            std::memset(&p, 0, sizeof p);
            p.size = sizeof p;
            if (mfsk_params_init(m, &p) != MFSK_STATUS_OK) {
                fail(name, "drives the decoder but mfsk_params_init refuses it");
                return;
            }
            if (!(p.band_hi_hz > p.band_lo_hz)) {
                fail(name, "publishes an unusable default band");
                return;
            }
            if (info.decode_fft1_size == 0 &&
                (std::strncmp(name, "FT", 2) == 0 || std::strncmp(name, "FST4", 4) == 0)) {
                fail(name, "drives the decoder but reports no slot transform size");
                return;
            }
            if (info.decode_fft1_size > widest_fft) {
                widest_fft = info.decode_fft1_size;
                std::snprintf(widest_name, sizeof widest_name, "%s", name);
            }
        }

        if (info.caps & MFSK_CAP_SNIPER) {
            snipers++;
            if (m != MFSK_MODE_FT8) {
                fail(name, "advertises a sniper, which is an FT8-only mode");
                return;
            }
        }
        if (std::strncmp(name, "FST4-", 5) == 0) fst4_submodes++;
    }

    // FST4-15, -30, -60A, -120, -300, and -900 and -1800 since #649.
    if (fst4_submodes != 7) {
        fail("introspect", "expected all seven FST4 sub-modes to be addressable");
        return;
    }
    if (with_handle < 7) {
        fail("introspect", "FT8 + FT4 + seven FST4 should all drive the decoder");
        return;
    }
    if (snipers != 1) {
        fail("introspect", "exactly one mode should claim the sniper");
        return;
    }
    std::printf("  %d mode(s) drive the decoder, %d FST4 sub-mode(s) addressable\n",
                with_handle, fst4_submodes);
    std::printf("  largest slot transform: %s at %u points\n", widest_name, widest_fft);

    MfskMode dummy;
    if (mfsk_mode_from_name("FT9", &dummy) != MFSK_STATUS_INVALID_ARG) {
        fail("introspect", "a nonsense mode name should be INVALID_ARG");
    }
    if (mfsk_mode_at(n, &dummy) != MFSK_STATUS_INVALID_ARG) {
        fail("introspect", "one past the end should fail rather than wrap");
    }
    if (mfsk_mode_info(MFSK_MODE_FT8, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("introspect", "a NULL out pointer should be rejected");
    }

    // Size versioning: an older caller declares a smaller struct and must
    // get only its prefix written.
    {
        unsigned char buf[sizeof(MfskModeInfo)];
        std::memset(buf, 0xAA, sizeof buf);
        const uint32_t shortSize = offsetof(MfskModeInfo, ntones);
        std::memcpy(buf, &shortSize, sizeof shortSize);
        if (mfsk_mode_info(MFSK_MODE_FT8, reinterpret_cast<MfskModeInfo*>(buf)) != MFSK_STATUS_OK) {
            fail("introspect", "size-versioned call with an older header failed");
        } else {
            uint32_t written = 0;
            std::memcpy(&written, buf, sizeof written);
            if (written != shortSize) {
                fail("introspect", "size was not rewritten to what was actually written");
            }
            for (size_t i = shortSize; i < sizeof buf; ++i) {
                if (buf[i] != 0xAA) {
                    fail("introspect", "wrote past the caller's declared struct size");
                    break;
                }
            }
        }
    }

    std::printf("  OK\n");
}

// ── The decoder handle ──────────────────────────────────────────────
//
// params → extras → open → rows into caller memory. Nothing here frees a
// pointer the library allocated.
void test_decoder() {
    std::printf("\n— decoder handle: params → open → rows into caller memory\n");

    const std::vector<int16_t> audio =
        synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);

    const MfskParams p = params_for(MFSK_MODE_FT8);
    if (p.size != sizeof p || p.depth != MFSK_DEPTH_DEEP || p.ap_mode != MFSK_AP_OFF) {
        fail("decoder", "FT8 defaults should be Deep with AP off");
    }
    std::printf("  FT8 defaults: band [%.0f, %.0f] depth %u\n", p.band_lo_hz, p.band_hi_hz, p.depth);

    MfskDecoder* s = open_dec("decoder", MFSK_MODE_FT8, &p, nullptr);
    if (s == nullptr) return;

    Rows rows;
    if (!decode_i16(s, audio, rows, "decoder")) {
        mfsk_decoder_close(s);
        return;
    }
    std::printf("  %zu decode(s):\n", rows.len);
    bool found = false;
    for (size_t i = 0; i < rows.len; ++i) {
        const MfskDecode& r = rows.items[i];
        std::printf("    mode=%d freq=%7.2f dt=%+.3f snr=%+.1f cv=%.3f "
                    "info=%u pass=%u text='%s'\n",
                    static_cast<int>(r.mode), r.freq_hz, r.dt_sec, r.snr_db, r.sync_cv,
                    r.info_bits, r.pass, r.text);
        if (std::strstr(r.text, "JA1ABC") != nullptr) found = true;
        if (r.mode != MFSK_MODE_FT8) fail("decoder", "row reports the wrong mode");
        if (r.info_bits != 91) fail("decoder", "FT8 is LDPC(174,91); info_bits should be 91");
    }
    if (!found) fail("decoder", "did not decode the signal it was given");

    // FEC bits come from the decoder, not from a pointer in the row.
    {
        std::vector<uint8_t> bits(128);
        size_t got = 0;
        if (mfsk_decoder_copy_info(s, 0, bits.data(), bits.size(), &got) != MFSK_STATUS_OK ||
            got != 91) {
            fail("decoder", "copy_info failed with a sufficient buffer");
        }
        if (mfsk_decoder_copy_info(s, 99, bits.data(), bits.size(), &got) != MFSK_STATUS_INVALID_ARG) {
            fail("decoder", "copy_info past the last row should be INVALID_ARG");
        }
    }

    // A short buffer reports the count needed rather than truncating.
    size_t needed = 0;
    MfskDecode one;
    std::memset(&one, 0, sizeof one);
    one.size = sizeof one;
    if (mfsk_decoder_decode_i16(s, audio.data(), audio.size(), 12000, MFSK_PERIOD_NONE,
                                &one, 0, &needed) != MFSK_STATUS_INVALID_ARG) {
        fail("decoder", "a zero-capacity buffer should report INVALID_ARG");
    } else if (needed != rows.len) {
        fail("decoder", "*out_len should be the count needed");
    }

    // The f32 entry point takes any level.
    std::vector<float> quiet(audio.size());
    for (size_t i = 0; i < audio.size(); ++i) quiet[i] = audio[i] / 32768.0f * 0.002f;
    Rows qrows;
    if (decode_f32(s, quiet, qrows, "decoder") && !qrows.contains("JA1ABC")) {
        fail("decoder", "a quiet f32 buffer lost the signal");
    }
    mfsk_decoder_close(s);

    // Every mode that claims the handle must open one with its defaults.
    const uint32_t total = mfsk_mode_count();
    int opened = 0;
    for (uint32_t i = 0; i < total; ++i) {
        MfskMode m;
        if (mfsk_mode_at(i, &m) != MFSK_STATUS_OK) continue;
        if ((mfsk_mode_caps(m) & MFSK_CAP_DECODE_HANDLE) == 0) continue;
        MfskDecoder* d = open_dec(mfsk_mode_name(m), m);
        if (d != nullptr) {
            opened++;
            mfsk_decoder_close(d);
        }
    }
    std::printf("  opened a decoder for all %d handle-driving mode(s)\n", opened);
    std::printf("  OK\n");
}

// ── Options the mode lacks fail at open ─────────────────────────────

MfskStatus open_status(MfskMode mode, const MfskParams* p, const MfskExtras* e) {
    MfskStatus st = MFSK_STATUS_OK;
    MfskDecoder* d = mfsk_decoder_open(mode, p, e, &st);
    if (d != nullptr) mfsk_decoder_close(d);
    return st;
}

void test_unsupported_options() {
    std::printf("\n— options: a mode that lacks one refuses at open, with a reason\n");

    MfskExtras e = extras_init();
    e.strategy = MFSK_STRATEGY_SIC_ROUNDS;
    e.sic_rounds = 2;
    if (open_status(MFSK_MODE_FT8, nullptr, &e) != MFSK_STATUS_OK ||
        open_status(MFSK_MODE_FT4, nullptr, &e) != MFSK_STATUS_OK) {
        fail("options", "SIC rounds are FT8's and FT4's");
    }
    if (open_status(MFSK_MODE_FST4S60, nullptr, &e) != MFSK_STATUS_UNSUPPORTED ||
        open_status(MFSK_MODE_WSPR, nullptr, &e) != MFSK_STATUS_UNSUPPORTED) {
        fail("options", "FST4 and WSPR have no subtraction and should refuse");
    } else {
        mfsk_decoder_open(MFSK_MODE_WSPR, nullptr, &e, nullptr);
        std::printf("  refused: %s\n", mfsk_last_error());
    }

    e = extras_init();
    e.a7 = 1;
    if (open_status(MFSK_MODE_FT8, nullptr, &e) != MFSK_STATUS_OK ||
        open_status(MFSK_MODE_FT4, nullptr, &e) != MFSK_STATUS_UNSUPPORTED) {
        fail("options", "a7 is FT8's alone");
    }

    e = extras_init();
    e.sniper_hz = 250.0f;
    if (open_status(MFSK_MODE_FT4, nullptr, &e) != MFSK_STATUS_UNSUPPORTED) {
        fail("options", "FT4 should refuse a narrow-band (sniper) search");
    }

    e = extras_init();
    e.nb_percent = 5;
    if (open_status(MFSK_MODE_FST4S60, nullptr, &e) != MFSK_STATUS_OK ||
        open_status(MFSK_MODE_FT8, nullptr, &e) != MFSK_STATUS_UNSUPPORTED) {
        fail("options", "the noise blanker is FST4's alone");
    }

    // An out-of-range value is the caller's mistake, not a missing option.
    e = extras_init();
    e.nb_percent = 99;
    if (open_status(MFSK_MODE_FST4S60, nullptr, &e) != MFSK_STATUS_INVALID_ARG) {
        fail("options", "nb_percent 99 should be INVALID_ARG");
    }
    MfskParams p = params_for(MFSK_MODE_FT8);
    p.depth = 7;
    if (open_status(MFSK_MODE_FT8, &p, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("options", "depth 7 should be INVALID_ARG, not clamped");
    }
    p = params_for(MFSK_MODE_FT8);
    p.band_hi_hz = p.band_lo_hz;
    if (open_status(MFSK_MODE_FT8, &p, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("options", "an empty band should be INVALID_ARG");
    }

    // set_extras on a live decoder is refused the same way, and changes nothing.
    MfskDecoder* d = open_dec("options", MFSK_MODE_FT4);
    if (d != nullptr) {
        e = extras_init();
        e.a7 = 1;
        if (mfsk_decoder_set_extras(d, &e) != MFSK_STATUS_UNSUPPORTED) {
            fail("options", "set_extras(a7) on FT4 should be UNSUPPORTED");
        }
        mfsk_decoder_close(d);
    }
    std::printf("  OK\n");
}

// ── Per-mode round trips ────────────────────────────────────────────

void test_ft8() {
    std::printf("\n— FT8 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    decoder_roundtrip("FT8", MFSK_MODE_FT8,
                      synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f), "JA1ABC");
}

void test_ft4() {
    std::printf("\n— FT4 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    decoder_roundtrip("FT4", MFSK_MODE_FT4,
                      synth_slot(MFSK_MODE_FT4, "CQ", "JA1ABC", "PM95", 1500.0f), "JA1ABC");
}

void test_fst4() {
    if (std::getenv("RUN_FST4_ROUNDTRIP") == nullptr) {
        std::printf("\n— FST4-60A roundtrip: skipped (set RUN_FST4_ROUNDTRIP=1)\n");
        return;
    }
    // 1000 Hz: FST4's default band is 600-1400 Hz, not FT8's 200-4000.
    std::printf("\n— FST4-60A roundtrip, and all seven sub-modes addressable\n");
    decoder_roundtrip("FST4-60A", MFSK_MODE_FST4S60,
                      synth_slot(MFSK_MODE_FST4S60, "CQ", "JA1ABC", "PM95", 1000.0f), "JA1ABC");

    const MfskMode others[] = {MFSK_MODE_FST4S15,  MFSK_MODE_FST4S30,  MFSK_MODE_FST4S120,
                               MFSK_MODE_FST4S300, MFSK_MODE_FST4S900, MFSK_MODE_FST4S1800};
    for (MfskMode m : others) {
        MfskDecoder* d = open_dec(mfsk_mode_name(m), m);
        if (d != nullptr) mfsk_decoder_close(d);
    }
    std::printf("  all seven FST4 sub-modes open a decoder\n");
}

// ── FST4W: pack, tones, PCM, decode; the hash and the known-call list ──
//
// All through the C calls: nothing here knows FST4W's layout. A WSPR-type
// message packs to 77 bits (`mfsk_fst4w_pack`), and the ordinary tone and
// synthesis stages take it from there.
std::vector<int16_t> fst4w_slot(MfskMode mode, const char* text, float freq_hz) {
    uint8_t msg[77];
    if (mfsk_fst4w_pack(text, msg) != MFSK_STATUS_OK) {
        fail("fst4w", mfsk_last_error());
        return {};
    }
    std::vector<uint8_t> tones(mfsk_symbol_count(mode));
    size_t n = 0;
    if (mfsk_message_to_tones(mode, msg, tones.data(), tones.size(), &n) != MFSK_STATUS_OK) {
        fail("fst4w", mfsk_last_error());
        return {};
    }
    std::vector<int16_t> pcm(mfsk_synth_output_len(mode));
    size_t w = 0;
    if (mfsk_tones_to_i16(mode, tones.data(), tones.size(), freq_hz, 8000, pcm.data(), pcm.size(),
                          &w) != MFSK_STATUS_OK) {
        fail("fst4w", mfsk_last_error());
        return {};
    }
    MfskModeInfo info{};
    info.size = sizeof info;
    mfsk_mode_info(mode, &info);
    std::vector<int16_t> slot(info.slot_samples_12k, 0);
    const size_t at = static_cast<size_t>(info.tx_start_offset_s * 12000.0f);
    for (size_t i = 0; i < w && at + i < slot.size(); ++i) slot[at + i] = pcm[i];
    return slot;
}

void test_fst4w() {
    std::printf("\n— FST4W: pack, transmit, decode, hash22 and the known-call list\n");
    const MfskMode mode = MFSK_MODE_FST4W120;
    MfskDecoder* d = open_dec("FST4W-120", mode);
    if (d == nullptr) return;

    char calls[256] = {0};
    size_t need = 0;
    if (mfsk_decoder_get_wcalls(d, calls, sizeof calls, &need) != MFSK_STATUS_OK || need != 1) {
        fail("fst4w", "a fresh decoder has an empty known-call list");
    }
    if (mfsk_decoder_set_wcalls(d, "JA1XYZ PM95\nVK3NV QF22\n") != MFSK_STATUS_OK) {
        fail("fst4w", mfsk_last_error());
    }
    mfsk_decoder_get_wcalls(d, calls, sizeof calls, &need);
    if (std::strcmp(calls, "JA1XYZ PM95\nVK3NV QF22") != 0) fail("fst4w", "the list reads back");
    mfsk_decoder_set_wcalls(d, "");

    Rows rows;
    if (decode_i16(d, fst4w_slot(mode, "K1ABC FN42 37", 1500.0f), rows, "FST4W-120")) {
        print_rows("FST4W-120", rows);
        if (!rows.contains("K1ABC FN42 37")) fail("fst4w", "the round trip lost the message");
        if (rows.len == 1) {
            const MfskDecode& r = rows.items[0];
            if (r.info_bits != 74 || r.key_bits != 50) fail("fst4w", "74 info bits, a 50-bit key");
            if ((r.flags & MFSK_DECODE_FLAG_HAS_HASH22) != 0) fail("fst4w", "a resolved call has no hash22");
        }
    }
    mfsk_decoder_get_wcalls(d, calls, sizeof calls, &need);
    if (std::strcmp(calls, "K1ABC FN42") != 0) fail("fst4w", "a Keff-66 decode teaches the list");

    Rows hashed;
    MfskDecoder* d2 = open_dec("FST4W-120", mode);
    if (d2 != nullptr) {
        if (decode_i16(d2, fst4w_slot(mode, "<JA1XYZ> PM95AA", 1500.0f), hashed, "FST4W-120 hash")) {
            if (!hashed.contains("<...> PM95AA")) fail("fst4w", "an unresolved call reads <...>");
            if (hashed.len == 1 && (hashed.items[0].flags & MFSK_DECODE_FLAG_HAS_HASH22) == 0) {
                fail("fst4w", "and carries its 22-bit hash");
            }
        }
        mfsk_decoder_close(d2);
    }

    uint8_t msg[77];
    if (mfsk_fst4w_pack("CQ K1ABC FN42", msg) != MFSK_STATUS_DECODE_FAILED) {
        fail("fst4w", "a message FST4W cannot send is refused");
    }
    mfsk_decoder_close(d);

    MfskDecoder* ft8 = open_dec("FT8", MFSK_MODE_FT8);
    if (ft8 != nullptr) {
        if (mfsk_decoder_get_wcalls(ft8, calls, sizeof calls, &need) != MFSK_STATUS_UNSUPPORTED) {
            fail("fst4w", "other modes have no known-call list");
        }
        mfsk_decoder_close(ft8);
    }
}

// ── The modes whose frames have no tone stage here ──────────────────
//
// WSPR, JT9, JT65 and Q65 go through the same handle: the frame is placed
// in the period at the offset the mode's upstream decoder expects and
// decoded as one slot of f32 audio.

void test_wspr() {
    std::printf("\n— WSPR through the decoder handle\n");
    size_t need = 0;
    mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, nullptr, 0, &need);
    std::vector<float> pcm(need);
    size_t got = 0;
    if (mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, pcm.data(), pcm.size(), &got)
            != MFSK_STATUS_OK) {
        fail("WSPR", mfsk_last_error());
        return;
    }
    pcm.resize(got);
    MfskDecoder* d = open_dec("WSPR", MFSK_MODE_WSPR);
    if (d == nullptr) return;
    Rows rows;
    if (decode_f32(d, put_in_slot(pcm, 1.0f, 120), rows, "WSPR")) {
        print_rows("WSPR", rows);
        if (!rows.contains("K1ABC FN42 37")) fail("WSPR", "expected K1ABC FN42 37");
    }
    mfsk_decoder_close(d);
}

void test_jt9_jt65() {
    std::printf("\n— JT9 and JT65: the frame starts the period\n");
    struct Case { const char* tag; MfskMode mode; Encoder enc; float freq; };
    const Case cases[] = {
        {"JT9", MFSK_MODE_JT9, mfsk_encode_jt9, 1350.0f},
        {"JT65", MFSK_MODE_JT65, mfsk_encode_jt65, 1270.0f},
    };
    for (const Case& c : cases) {
        const std::vector<float> frame = encode_f32(c.enc, "CQ", "K1ABC", "FN42", c.freq);
        MfskDecoder* d = open_dec(c.tag, c.mode);
        if (d == nullptr || frame.empty()) { if (d) mfsk_decoder_close(d); continue; }
        Rows rows;
        if (decode_f32(d, put_in_slot(frame, 0.0f, 60), rows, c.tag)) {
            print_rows(c.tag, rows);
            if (!rows.contains("CQ K1ABC FN42")) fail(c.tag, "expected CQ K1ABC FN42");
        }
        mfsk_decoder_close(d);
    }
}

// Q65 by sub-mode name, and the fast-fading metric through MfskExtras.
void test_q65() {
    std::printf("\n— Q65-30A: encode by sub-mode name, plain and fading decode\n");

    size_t need = 0;
    mfsk_encode_q65(MFSK_Q65_SUB_MODE_A30, "CQ", "K1ABC", "FN42", 1000.0f,
                    nullptr, 0, &need);
    if (need == 0) {
        fail("Q65", mfsk_last_error());
        return;
    }
    std::vector<float> pcm(need);
    size_t got = 0;
    if (mfsk_encode_q65(MFSK_Q65_SUB_MODE_A30, "CQ", "K1ABC", "FN42", 1000.0f,
                        pcm.data(), pcm.size(), &got) != MFSK_STATUS_OK) {
        fail("Q65", mfsk_last_error());
        return;
    }
    pcm.resize(got);
    const std::vector<float> slot = put_in_slot(pcm, 0.5f, 30);

    MfskExtras e = extras_init();
    e.t_early_s = 1.0f;
    e.t_late_s = 1.0f;
    MfskDecoder* d = open_dec("Q65", MFSK_MODE_Q65A30, nullptr, &e);
    if (d == nullptr) return;
    Rows rows;
    if (decode_f32(d, slot, rows, "Q65")) {
        print_rows("Q65-30A", rows);
        if (!rows.contains("K1ABC")) fail("Q65", "expected K1ABC");
        for (size_t i = 0; i < rows.len; ++i) {
            if (rows.items[i].mode != MFSK_MODE_Q65A30) {
                fail("Q65", "a Q65-30A row should report MFSK_MODE_Q65A30");
            }
        }
    }

    // The fading metric is two extras fields on the same handle.
    e.fading_b90_ts = 0.1f;
    e.fading_model = MFSK_Q65_FADING_MODEL_GAUSSIAN;
    if (mfsk_decoder_set_extras(d, &e) != MFSK_STATUS_OK) {
        fail("Q65 fading", mfsk_decoder_last_error(d));
    } else {
        Rows fading;
        if (decode_f32(d, slot, fading, "Q65 fading")) {
            print_rows("Q65-30A fading", fading);
            if (!fading.contains("K1ABC")) fail("Q65 fading", "expected K1ABC");
        }
    }
    mfsk_decoder_close(d);
}

// ── Budget ──────────────────────────────────────────────────────────
int g_budget_polls = 0;
extern "C" bool budget_refuse_everything(void*) {
    ++g_budget_polls;
    return false;
}
extern "C" bool budget_allow_everything(void*) {
    ++g_budget_polls;
    return true;
}

std::vector<int16_t> two_stations() {
    std::vector<int16_t> a = synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1200.0f);
    const std::vector<int16_t> b = synth_slot(MFSK_MODE_FT8, "CQ", "VK3NV", "QF22", 1800.0f);
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        a[i] = static_cast<int16_t>(a[i] + b[i]);
    }
    return a;
}

// Early decode (#572): prefixes of one period, checkpoint A's rows first.
void test_prefix() {
    std::printf("\n— decode_prefix: checkpoint A early, the whole period as decode gives it\n");
    const std::vector<int16_t> audio = two_stations();
    MfskDecoder* whole = open_dec("prefix", MFSK_MODE_FT8);
    MfskDecoder* d = open_dec("prefix", MFSK_MODE_FT8);
    if (whole == nullptr || d == nullptr) return;
    Rows want;
    if (mfsk_decoder_decode_i16(whole, audio.data(), audio.size(), 12000, 7, want.items, 16,
                                &want.len) != MFSK_STATUS_OK) {
        fail("prefix", "whole-period decode failed");
    }
    const size_t cuts[3] = {141696, 162432, audio.size()};
    Rows got[3];
    for (int i = 0; i < 3; ++i) {
        if (mfsk_decoder_decode_prefix_i16(d, audio.data(), cuts[i], 12000, 7, got[i].items, 16,
                                           &got[i].len) != MFSK_STATUS_OK) {
            fail("prefix", mfsk_decoder_last_error(d));
        }
    }
    std::printf("  A: %zu row(s), B: %zu, end: %zu (decode: %zu)\n", got[0].len, got[1].len,
                got[2].len, want.len);
    if (got[0].len == 0) fail("prefix", "checkpoint A returned nothing");
    for (size_t i = 0; i < got[0].len; ++i) {
        if (got[0].items[i].stage != MFSK_STAGE_EARLY) fail("prefix", "an A row is not EARLY");
    }
    if (got[1].len != 0) fail("prefix", "checkpoint B returned rows");
    if (got[2].len != want.len) fail("prefix", "the whole period differs from decode");
    for (size_t i = 0; i < got[2].len && i < want.len; ++i) {
        if (std::strcmp(got[2].items[i].text, want.items[i].text) != 0) {
            fail("prefix", "the whole period's rows are not decode's");
        }
    }
    mfsk_decoder_close(d);
    mfsk_decoder_close(whole);
}

void test_budget() {
    std::printf("\n— budget: the caller's predicate cuts the search and the report says so\n");
    const std::vector<int16_t> audio = two_stations();

    MfskDecoder* s = open_dec("budget", MFSK_MODE_FT8);
    if (s == nullptr) return;

    Rows base;
    decode_i16(s, audio, base, "budget");
    const size_t full = base.len;
    std::printf("  unbudgeted: %zu decode(s)\n", full);

    MfskBudgetReport rep;
    std::memset(&rep, 0, sizeof rep);
    rep.size = sizeof rep;
    if (mfsk_decoder_last_budget(s, &rep) != MFSK_STATUS_OK) {
        fail("budget", "last_budget failed");
    } else if (rep.exhausted || rep.candidates_skipped != 0) {
        fail("budget", "no budget was set, so nothing should report as cut");
    } else if (rep.cut_at_sync != -1) {
        fail("budget", "absent cut_at_sync must be -1");
    }

    g_budget_polls = 0;
    if (mfsk_decoder_set_budget(s, budget_refuse_everything, nullptr) != MFSK_STATUS_OK) {
        fail("budget", mfsk_decoder_last_error(s));
    }
    Rows cut;
    decode_i16(s, audio, cut, "budget");
    std::printf("  budgeted to nothing: %zu decode(s), %d poll(s)\n", cut.len, g_budget_polls);
    if (g_budget_polls == 0) fail("budget", "the predicate was never polled");
    if (cut.len >= full) fail("budget", "a refusing budget found as much as no budget");

    std::memset(&rep, 0, sizeof rep);
    rep.size = sizeof rep;
    mfsk_decoder_last_budget(s, &rep);
    std::printf("  report: exhausted=%d skipped=%u ran=%u cut_at_sync=%d\n",
                (int)rep.exhausted, rep.candidates_skipped, rep.stages_run, rep.cut_at_sync);
    if (!rep.exhausted) fail("budget", "work was cut and the report does not say so");

    mfsk_decoder_set_budget(s, budget_allow_everything, nullptr);
    Rows allowed;
    decode_i16(s, audio, allowed, "budget");
    if (allowed.len != full) fail("budget", "a budget that allows everything changed the result");
    mfsk_decoder_set_budget(s, nullptr, nullptr);
    mfsk_decoder_close(s);

    // Every mode with a decoder takes one (#593), and publishes the bit.
    MfskDecoder* w = open_dec("budget", MFSK_MODE_WSPR);
    if (w != nullptr) {
        if ((mfsk_mode_caps(MFSK_MODE_WSPR) & MFSK_CAP_BUDGET) == 0) {
            fail("budget", "WSPR polls the budget but does not publish MFSK_CAP_BUDGET");
        }
        if (mfsk_decoder_set_budget(w, budget_refuse_everything, nullptr) != MFSK_STATUS_OK) {
            fail("budget", mfsk_decoder_last_error(w));
        }
        if (mfsk_decoder_delivery_is_exact(w)) {
            fail("budget", "WSPR's delivery is the parallel contract, not the exact one");
        }
        mfsk_decoder_close(w);
    }
}

// ── Hash resolution across periods ──────────────────────────────────
//
// A decoder's callsign table is its own and outlives the period: a `<...>`
// that period 10 introduced reads as the call in period 11, on the same
// decoder and not on another.

std::vector<int16_t> frame_in_slot(MfskMode mode, const uint8_t* msg77, float freq) {
    std::vector<uint8_t> tones(mfsk_symbol_count(mode));
    size_t n = 0;
    std::vector<int16_t> slot;
    if (mfsk_message_to_tones(mode, msg77, tones.data(), tones.size(), &n) != MFSK_STATUS_OK) {
        fail("hash", mfsk_last_error());
        return slot;
    }
    std::vector<int16_t> pcm(mfsk_synth_output_len(mode));
    size_t w = 0;
    if (mfsk_tones_to_i16(mode, tones.data(), n, freq, 8000, pcm.data(), pcm.size(), &w)
            != MFSK_STATUS_OK) {
        fail("hash", mfsk_last_error());
        return slot;
    }
    slot.assign(180000, 0);
    for (size_t i = 0; i < w; ++i) slot[6000 + i] = pcm[i];
    return slot;
}

void test_hash_resolution() {
    std::printf("\n— hash resolution: <...> resolves in the decoder that heard the call\n");

    uint8_t m4[77];
    if (mfsk_pack77_type4("JA1ABC/QRP", "VK3NV", nullptr, false, m4) != MFSK_STATUS_OK) {
        fail("hash", "pack77_type4");
        return;
    }
    const std::vector<int16_t> heard = synth_slot(MFSK_MODE_FT8, "CQ", "VK3NV", "QF22", 1500.0f);
    const std::vector<int16_t> hashed = frame_in_slot(MFSK_MODE_FT8, m4, 1500.0f);
    if (hashed.empty()) return;

    MfskDecoder* a = open_dec("hash", MFSK_MODE_FT8);
    MfskDecoder* b = open_dec("hash", MFSK_MODE_FT8);
    MfskDecoder* c = open_dec("hash", MFSK_MODE_FT8);
    if (!a || !b || !c) return;

    Rows r0, with, without;
    decode_i16(a, heard, r0, "hash", 10);
    if (!r0.contains("VK3NV")) fail("hash", "period 10 did not decode VK3NV");
    decode_i16(a, hashed, with, "hash", 11);
    decode_i16(b, hashed, without, "hash", 11);
    print_rows("period 11, same decoder", with);
    print_rows("period 11, fresh decoder", without);
    if (!without.contains("<...>")) fail("hash", "a decoder that never heard VK3NV should show <...>");
    if (!with.contains("<VK3NV>")) fail("hash", "the decoder that heard VK3NV should resolve it");
    bool flagged = false;
    for (size_t i = 0; i < with.len; ++i) {
        if (with.items[i].flags & MFSK_DECODE_FLAG_HASH_RESOLVED) flagged = true;
    }
    if (!flagged) fail("hash", "MFSK_DECODE_FLAG_HASH_RESOLVED not set on the resolved row");

    // mfsk_unpack77 leaves it unresolved; the decoder's own table resolves it.
    char text[64];
    size_t len = 0;
    if (mfsk_unpack77(m4, text, sizeof text, &len) != MFSK_STATUS_OK || !std::strstr(text, "<...>")) {
        fail("hash", "mfsk_unpack77 should leave the hash unresolved");
    }
    if (mfsk_decoder_unpack77(a, m4, text, sizeof text, &len) != MFSK_STATUS_OK ||
        !std::strstr(text, "<VK3NV>")) {
        fail("hash", "mfsk_decoder_unpack77 should resolve against the decoder's table");
    }
    if (mfsk_decoder_unpack77(b, m4, text, sizeof text, &len) != MFSK_STATUS_OK ||
        !std::strstr(text, "<...>")) {
        fail("hash", "a decoder that never heard it should not resolve");
    }

    // Teaching from outside works, and clear forgets.
    if (mfsk_decoder_add_callsign(c, "VK3NV") != MFSK_STATUS_OK) {
        fail("hash", "add_callsign");
    }
    Rows taught, forgotten;
    decode_i16(c, hashed, taught, "hash");
    if (!taught.contains("<VK3NV>")) fail("hash", "a taught call should resolve");
    if (mfsk_decoder_clear(c) != MFSK_STATUS_OK) fail("hash", "clear");
    decode_i16(c, hashed, forgotten, "hash");
    if (!forgotten.contains("<...>")) fail("hash", "clear should forget the table");

    // A mode whose messages carry no hashes says so.
    MfskDecoder* w = open_dec("hash", MFSK_MODE_WSPR);
    if (w != nullptr) {
        if (mfsk_decoder_add_callsign(w, "VK3NV") != MFSK_STATUS_UNSUPPORTED) {
            fail("hash", "WSPR add_callsign should be UNSUPPORTED");
        }
        mfsk_decoder_close(w);
    }
    mfsk_decoder_close(a);
    mfsk_decoder_close(b);
    mfsk_decoder_close(c);
}

// ── Streaming rows via callback ─────────────────────────────────────
//
// A real C callback invoked from C++-compiled code through the generated
// header — the one thing the Rust tests cannot exercise, since they call
// the crate's functions directly and never cross a translation-unit
// boundary.

extern "C" void streaming_collect(const MfskDecode* row, void* user_data) {
    auto* out = static_cast<std::vector<std::string>*>(user_data);
    if (row != nullptr) out->emplace_back(row->text);
}

void test_ft8_streaming() {
    std::printf("\n— streaming rows: mfsk_decoder_set_on_decode fires as decodes are found\n");
    std::vector<int16_t> audio = synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1650.0f);

    MfskDecoder* s = open_dec("streaming", MFSK_MODE_FT8);
    if (s == nullptr) return;

    std::vector<std::string> streamed;
    if (mfsk_decoder_set_on_decode(s, streaming_collect, &streamed) != MFSK_STATUS_OK) {
        fail("streaming", "set_on_decode failed");
    }
    Rows rows;
    if (decode_i16(s, audio, rows, "streaming")) {
        print_rows("FT8 streaming", rows);
        std::printf("  streamed via callback: %zu\n", streamed.size());
        if (streamed.empty()) fail("streaming", "the callback never fired");
        if (!rows.contains("JA1ABC")) fail("streaming", "expected JA1ABC");
        if (streamed.size() != rows.len) {
            fail("streaming", "the callback should have seen what the array holds");
        }
    }
    mfsk_decoder_close(s);
}

// ── Stream capture and the UTC slot grid ────────────────────────────
//
// Time enters as a parameter — the library reads no clock — so this is
// usable from a phone that was backgrounded and from a replayed recording
// alike.
void test_stream_capture() {
    std::printf("\n— stream capture: push → set_time → slot ready → fused decode\n");

    const std::vector<int16_t> slot =
        synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskStream* stream = mfsk_stream_open(MFSK_MODE_FT8, 12000, &st);
    if (stream == nullptr || st != MFSK_STATUS_OK) {
        fail("stream", mfsk_last_error());
        return;
    }
    if (mfsk_stream_slot_ready(stream)) fail("stream", "a fresh stream should have no slot ready");

    // A quarter second of lead-in, and the clock says sample 0 was 250 ms
    // before a 15 s boundary: the recording that follows starts on it.
    const int64_t boundary_s = 1700000010;  // a multiple of 15
    const int64_t t0 = boundary_s * 1000000000LL;
    const std::vector<int16_t> lead(3000, 0);
    mfsk_stream_push_i16(stream, lead.data(), lead.size());
    if (mfsk_stream_position(stream) != lead.size()) fail("stream", "position is not the count pushed");
    int32_t change = -1;
    if (mfsk_stream_set_time(stream, t0 - 250000000LL, 0, &change) != MFSK_STATUS_OK ||
        change != MFSK_CLOCK_FIRST) {
        fail("stream", "the first clock reading should report MFSK_CLOCK_FIRST");
    }

    std::vector<int16_t> audio = slot;
    audio.resize(audio.size() + 12000, 0);
    const size_t kChunk = 7777;  // odd, the way a UAC reader delivers
    for (size_t i = 0; i < audio.size(); i += kChunk) {
        const size_t n = std::min(kChunk, audio.size() - i);
        if (mfsk_stream_push_i16(stream, audio.data() + i, n) != MFSK_STATUS_OK) {
            fail("stream", mfsk_last_error());
            mfsk_stream_close(stream);
            return;
        }
    }
    if (!mfsk_stream_slot_ready(stream)) {
        fail("stream", "a full slot was pushed and is not ready");
        mfsk_stream_close(stream);
        return;
    }

    MfskDecoder* d = open_dec("stream", MFSK_MODE_FT8);
    if (d == nullptr) { mfsk_stream_close(stream); return; }
    Rows rows;
    int64_t period = -1, slot_utc_ns = -1;
    if (mfsk_decoder_decode_stream(d, stream, rows.items, 16, &rows.len, &period,
                                   &slot_utc_ns) != MFSK_STATUS_OK) {
        fail("stream", mfsk_decoder_last_error(d));
    } else {
        print_rows("stream", rows);
        std::printf("  slot period %lld, UTC %lld ns\n", (long long)period, (long long)slot_utc_ns);
        if (!rows.contains("JA1ABC")) fail("stream", "expected JA1ABC");
        if (period != boundary_s / 15) fail("stream", "the period should be the boundary the lead-in ends on");
        if (slot_utc_ns != period * 15000000000LL) fail("stream", "slot UTC is not period * 15 s");
        if (mfsk_stream_slot_ready(stream)) fail("stream", "the fused decode should have consumed the slot");
    }

    // Polling before a slot is ready is "not yet", not a failure to guard.
    size_t none = 99;
    if (mfsk_decoder_decode_stream(d, stream, rows.items, 16, &none, nullptr, nullptr)
            != MFSK_STATUS_UNSUPPORTED || none != 0) {
        fail("stream", "an empty stream should report UNSUPPORTED with *out_len = 0");
    }
    mfsk_decoder_close(d);
    mfsk_stream_close(stream);

    // A stream and a decoder of different modes do not mix.
    MfskStream* s4 = mfsk_stream_open(MFSK_MODE_FT4, 12000, nullptr);
    MfskDecoder* d8 = open_dec("stream", MFSK_MODE_FT8);
    if (s4 && d8 &&
        mfsk_decoder_decode_stream(d8, s4, rows.items, 16, &none, nullptr, nullptr)
            != MFSK_STATUS_INVALID_ARG) {
        fail("stream", "a stream and decoder of different modes should be INVALID_ARG");
    }
    mfsk_decoder_close(d8);
    mfsk_stream_close(s4);

    // The ring is sized per mode.
    for (MfskMode m : {MFSK_MODE_FT4, MFSK_MODE_FST4S300}) {
        MfskModeInfo info;
        std::memset(&info, 0, sizeof info);
        info.size = sizeof info;
        mfsk_mode_info(m, &info);
        MfskStream* st2 = mfsk_stream_open(m, 12000, nullptr);
        if (st2 == nullptr) {
            fail(mfsk_mode_name(m), "stream_open failed");
            continue;
        }
        const std::vector<int16_t> quiet(info.slot_samples_12k + 24000, 0);
        mfsk_stream_push_i16(st2, quiet.data(), quiet.size());
        if (!mfsk_stream_slot_ready(st2)) fail(mfsk_mode_name(m), "a full slot should be ready");
        mfsk_stream_close(st2);
    }

    // A mode that is not cut into slots has no stream.
    MfskStatus jst = MFSK_STATUS_OK;
    if (mfsk_stream_open(MFSK_MODE_JTTY, 12000, &jst) != nullptr || jst == MFSK_STATUS_OK) {
        fail("stream", "JTTY has no slots and should refuse a stream");
    }
    mfsk_stream_close(nullptr);
    std::printf("  OK\n");
}

// ── The parameter block and the options reach the decoder ───────────

void test_params() {
    std::printf("\n— params/extras: depth / eq / rx freq / sic / ap reach the decoder\n");
    std::vector<int16_t> audio = synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);

    MfskParams p = params_for(MFSK_MODE_FT8);
    p.rx_freq_hz = 1500.0f;
    p.tol_hz = 20.0f;
    p.ap_mode = MFSK_AP_FULL;
    std::snprintf(p.mycall, sizeof p.mycall, "%s", "K1ABC");
    MfskExtras e = extras_init();
    e.strictness = 2;
    e.eq_mode = 1;
    e.strategy = MFSK_STRATEGY_SIC_ROUNDS;
    e.sic_rounds = 2;
    e.has_ap_hint = 1;
    std::snprintf(e.ap_call1, sizeof e.ap_call1, "%s", "CQ");
    std::snprintf(e.ap_call2, sizeof e.ap_call2, "%s", "JA1ABC");

    MfskDecoder* s = open_dec("params", MFSK_MODE_FT8, &p, &e);
    if (s == nullptr) return;
    Rows rows;
    if (decode_i16(s, audio, rows, "params")) {
        print_rows("params", rows);
        if (!rows.contains("JA1ABC")) fail("params", "every option on lost the signal");
    }

    // The parameter block is rewritten between periods, as the GUI does.
    MfskParams narrow = p;
    narrow.band_lo_hz = 2500.0f;
    narrow.band_hi_hz = 2900.0f;
    narrow.rx_freq_hz = 2700.0f;
    if (mfsk_decoder_set_params(s, &narrow) != MFSK_STATUS_OK) fail("params", mfsk_decoder_last_error(s));
    Rows away;
    decode_i16(s, audio, away, "params");
    if (away.contains("JA1ABC")) fail("params", "a 2500-2900 Hz band still found a 1500 Hz signal");
    if (mfsk_decoder_set_params(s, &p) != MFSK_STATUS_OK) fail("params", mfsk_decoder_last_error(s));
    Rows back;
    decode_i16(s, audio, back, "params");
    if (!back.contains("JA1ABC")) fail("params", "restoring the block did not restore the decode");
    std::printf("  band moved away and back through mfsk_decoder_set_params\n");
    mfsk_decoder_close(s);
}

// ── The narrow-band (sniper) search ─────────────────────────────────

void test_sniper() {
    std::printf("\n— narrow-band search: FT8 only; FT4 gets AP on the wide-band path\n");

    std::vector<int16_t> ft8 = synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);
    MfskParams p = params_for(MFSK_MODE_FT8);
    p.rx_freq_hz = 1500.0f;
    MfskExtras e = extras_init();
    e.sniper_hz = 250.0f;
    MfskDecoder* s = open_dec("sniper", MFSK_MODE_FT8, &p, &e);
    if (s != nullptr) {
        Rows rows;
        if (decode_i16(s, ft8, rows, "sniper")) {
            print_rows("FT8 narrow", rows);
            if (!rows.contains("JA1ABC")) fail("sniper", "aimed at it and missed");
        }
        mfsk_decoder_close(s);
    }

    // FT4 refuses the sniper (test_unsupported_options) but takes the AP
    // hint on its ordinary wide-band decode.
    std::vector<int16_t> ft4 = synth_slot(MFSK_MODE_FT4, "CQ", "JA1ABC", "PM95", 1200.0f);
    MfskExtras w = extras_init();
    w.has_ap_hint = 1;
    std::snprintf(w.ap_call1, sizeof w.ap_call1, "%s", "CQ");
    std::snprintf(w.ap_call2, sizeof w.ap_call2, "%s", "JA1ABC");
    MfskDecoder* fs = open_dec("sniper", MFSK_MODE_FT4, nullptr, &w);
    if (fs != nullptr) {
        Rows rows;
        if (decode_i16(fs, ft4, rows, "sniper")) {
            print_rows("FT4 wide-band + AP", rows);
            if (!rows.contains("JA1ABC")) fail("sniper", "AP did not reach FT4");
        }
        mfsk_decoder_close(fs);
    }
}

// ── Threading ───────────────────────────────────────────────────────
//
// **A decoder is single-threaded**: it owns a callsign hash table it
// mutates on every decode plus what it carries between periods. The
// supported shape is one decoder per thread.

void test_threads_one_decoder_per_thread() {
    std::printf("\n— threads × 1 decoder each: 8 parallel FT8 decodes\n");
    constexpr int kThreads = 8;
    std::atomic<int> ok_count{0};
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&ok_count, t]() {
            std::vector<int16_t> audio =
                synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f + t * 20.0f);
            MfskStatus st = MFSK_STATUS_INTERNAL;
            MfskDecoder* d = mfsk_decoder_open(MFSK_MODE_FT8, nullptr, nullptr, &st);
            if (d == nullptr) return;
            Rows rows;
            const MfskStatus dst = mfsk_decoder_decode_i16(
                d, audio.data(), audio.size(), 12000, MFSK_PERIOD_NONE, rows.items, 16, &rows.len);
            if (dst == MFSK_STATUS_OK && rows.contains("JA1ABC")) ok_count++;
            mfsk_decoder_close(d);
        });
    }
    for (auto& th : ts) th.join();
    std::printf("  → %d/%d OK\n", ok_count.load(), kThreads);
    if (ok_count.load() != kThreads) fail("threads", "one-decoder-per-thread concurrent decode failed");
}

void test_threads_mixed_modes() {
    std::printf("\n— threads × mixed modes (FT8 + FT4 concurrently)\n");
    std::atomic<int> ok_count{0};
    std::vector<std::thread> ts;
    const MfskMode work[] = {MFSK_MODE_FT8, MFSK_MODE_FT4, MFSK_MODE_FT8, MFSK_MODE_FT4};
    constexpr int kJobs = 4;
    for (MfskMode mode : work) {
        ts.emplace_back([&ok_count, mode]() {
            std::vector<int16_t> audio = synth_slot(mode, "CQ", "JA1ABC", "PM95", 1500.0f);
            MfskStatus st = MFSK_STATUS_INTERNAL;
            MfskDecoder* d = mfsk_decoder_open(mode, nullptr, nullptr, &st);
            if (d == nullptr) return;
            Rows rows;
            const MfskStatus dst = mfsk_decoder_decode_i16(
                d, audio.data(), audio.size(), 12000, MFSK_PERIOD_NONE, rows.items, 16, &rows.len);
            if (dst == MFSK_STATUS_OK && rows.contains("JA1ABC")) ok_count++;
            mfsk_decoder_close(d);
        });
    }
    for (auto& th : ts) th.join();
    std::printf("  → %d/%d OK\n", ok_count.load(), kJobs);
    if (ok_count.load() != kJobs) fail("threads", "mixed-mode concurrent decode failed");
}

// ── JTTY: a stateful receiver, fed a recording in chunks ────────────

// Upstream's own sample recording (12 kHz mono 16-bit), the only real
// JTTY audio there is. `build.sh` bakes its path in.
std::vector<int16_t> load_wav_i16(const char* path) {
    std::ifstream f(path, std::ios::binary);
    std::vector<char> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    for (size_t i = 0; i + 8 <= b.size(); ++i) {
        if (std::memcmp(&b[i], "data", 4) == 0) {
            const size_t n = (b.size() - (i + 8)) / 2;
            std::vector<int16_t> pcm(n);
            std::memcpy(pcm.data(), &b[i + 8], n * 2);  // little-endian host
            return pcm;
        }
    }
    return {};
}

void test_jtty() {
    std::printf("\n— JTTY receiver\n");
    const char* expect = "RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!";

    MfskModeInfo info;
    std::memset(&info, 0, sizeof info);
    info.size = sizeof info;
    if (mfsk_mode_info(MFSK_MODE_JTTY, &info) != MFSK_STATUS_OK) {
        fail("jtty", "mfsk_mode_info(JTTY) failed");
        return;
    }
    if (!(info.caps & MFSK_CAP_STREAM_RECEIVER) || (info.caps & MFSK_CAP_DECODE_HANDLE) ||
        !(info.caps & MFSK_CAP_ENCODE)) {
        fail("jtty", "JTTY must be a stream receiver that can encode, with no slot decode handle");
    }
    if (info.slot_samples_12k != 22656 || info.n_symbols != 59) {
        fail("jtty", "JTTY frame geometry is wrong");
    }

    const std::vector<int16_t> pcm = load_wav_i16(MFSK_JTTY_WAV);
    if (pcm.size() < 12000) {
        fail("jtty", "could not read the golden recording (MFSK_JTTY_WAV)");
        return;
    }

    MfskJttyParams params;
    if (mfsk_jtty_params_init(&params) != MFSK_STATUS_OK || params.size != sizeof params) {
        fail("jtty", "params_init");
        return;
    }
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskJttyReceiver* rx = mfsk_jtty_open(12000, &params, &st);
    if (rx == nullptr || st != MFSK_STATUS_OK) {
        fail("jtty", "mfsk_jtty_open");
        return;
    }

    // Fed the way a live audio callback would: 4096 samples at a time,
    // polled after each push. Keep the latest text per message id.
    std::vector<std::pair<uint64_t, std::string>> last;
    bool complete = false;
    for (size_t pos = 0; pos < pcm.size(); pos += 4096) {
        const size_t n = std::min<size_t>(4096, pcm.size() - pos);
        if (mfsk_jtty_push_i16(rx, pcm.data() + pos, n) != MFSK_STATUS_OK) {
            fail("jtty", "push_i16");
            break;
        }
        MfskJttyUpdate u;
        std::memset(&u, 0, sizeof u);
        u.size = sizeof u;
        while (mfsk_jtty_poll(rx, &u) == 1) {
            bool seen = false;
            for (auto& l : last) {
                if (l.first == u.id) { l.second = u.text; seen = true; }
            }
            if (!seen) last.emplace_back(u.id, u.text);
            if (u.complete && std::strcmp(u.text, expect) == 0) complete = true;
        }
    }
    bool found = false;
    for (const auto& l : last) {
        std::printf("  message %llu: %s\n", (unsigned long long)l.first, l.second.c_str());
        if (l.second == expect) found = true;
    }
    if (!found || !complete) fail("jtty", "the recording's message did not come out complete");
    if (mfsk_jtty_pending(rx) != 0) fail("jtty", "queue should be drained");

    // Nothing waiting is 0, not an error; NULL handles are refused.
    MfskJttyUpdate u;
    std::memset(&u, 0, sizeof u);
    if (mfsk_jtty_poll(rx, &u) != 0) fail("jtty", "poll on an empty queue should be 0");
    if (mfsk_jtty_poll(nullptr, &u) >= 0) fail("jtty", "poll(NULL) should be negative");
    if (mfsk_jtty_push_i16(nullptr, pcm.data(), 4) != MFSK_STATUS_NULL_POINTER) {
        fail("jtty", "push(NULL handle) should be NULL_POINTER");
    }
    if (mfsk_jtty_reset(rx) != MFSK_STATUS_OK) fail("jtty", "reset");
    mfsk_jtty_close(rx);
    mfsk_jtty_close(nullptr);  // a no-op, like every free here

    // Transmit: text -> tones -> audio, and back through a fresh receiver.
    {
        const char* text = "CQ K1ABC CQ";
        size_t n_tones = 0;
        if (mfsk_jtty_encode_tones(text, 0, nullptr, 0, &n_tones) != MFSK_STATUS_OK || n_tones != 59) {
            fail("jtty", "encode_tones size query: one frame is 59 tones");
            return;
        }
        std::vector<uint8_t> tones(n_tones);
        if (mfsk_jtty_encode_tones(text, 0, tones.data(), tones.size(), &n_tones) != MFSK_STATUS_OK) {
            fail("jtty", "encode_tones");
            return;
        }
        size_t n_pcm = 0;
        if (mfsk_jtty_tones_to_i16(tones.data(), tones.size(), 1500.0f, 8000.0f, nullptr, 0, &n_pcm) !=
                MFSK_STATUS_OK || n_pcm != mfsk_jtty_synth_len(tones.size())) {
            fail("jtty", "tones_to_i16 size query");
            return;
        }
        std::vector<int16_t> audio(12000, 0);          // a second of lead-in
        std::vector<int16_t> pcm(n_pcm);
        if (mfsk_jtty_tones_to_i16(tones.data(), tones.size(), 1500.0f, 8000.0f, pcm.data(), pcm.size(),
                                   &n_pcm) != MFSK_STATUS_OK) {
            fail("jtty", "tones_to_i16");
            return;
        }
        audio.insert(audio.end(), pcm.begin(), pcm.end());
        audio.resize(audio.size() + 6 * 12000, 0);     // and room to finish

        MfskJttyReceiver* loop = mfsk_jtty_open(12000, nullptr, nullptr);
        if (loop == nullptr) { fail("jtty", "open (NULL params)"); return; }
        bool heard = false;
        for (size_t pos = 0; pos < audio.size(); pos += 4096) {
            const size_t n = std::min<size_t>(4096, audio.size() - pos);
            mfsk_jtty_push_i16(loop, audio.data() + pos, n);
            MfskJttyUpdate u;
            std::memset(&u, 0, sizeof u);
            u.size = sizeof u;
            while (mfsk_jtty_poll(loop, &u) == 1) {
                if (u.complete && std::strcmp(u.text, text) == 0) heard = true;
            }
        }
        mfsk_jtty_close(loop);
        if (!heard) fail("jtty", "the transmitted message did not come back through the receiver");

        // What cannot be sent is refused with a reason.
        const std::string too_long(81, 'A');
        if (mfsk_jtty_encode_tones(too_long.c_str(), 0, nullptr, 0, &n_tones) != MFSK_STATUS_INVALID_ARG) {
            fail("jtty", "an 81-character message should be INVALID_ARG");
        }
    }
}

// ── Wideband IQ receiver (mfsk_iq_*, #534) ──────────────────────────
//
// An FT8 slot made by the library's own synthesiser, placed as
// double-sideband IQ at 48 kS/s (12 kHz audio interpolated by 4, mixed up by
// the dial's offset from the centre) and pushed as bytes in each of the wire
// formats. The receiver must hand back the message, on the channel and mode
// it was added with, at the absolute frequency the dial makes of it.
void test_iq() {
    const uint32_t fs = 48000;
    const double center = 14077000.0;
    const double dial = center + 6000.0;
    const int64_t t0_ns = 1700000100LL * 1000000000LL;

    const std::vector<int16_t> slot = synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);
    if (slot.empty()) { fail("iq", "no FT8 slot to feed"); return; }

    // 12 kHz -> 48 kHz by linear interpolation is enough here: the audio is
    // narrow (<= 3 kHz) and the front end low-passes what the interpolation
    // leaves above it.
    std::vector<float> up(slot.size() * 4);
    for (size_t i = 0; i < slot.size(); ++i) {
        const float a = slot[i] / 32768.0f;
        const float b = (i + 1 < slot.size() ? slot[i + 1] : 0) / 32768.0f;
        for (int k = 0; k < 4; ++k) up[i * 4 + k] = a + (b - a) * (k / 4.0f);
    }
    const double w = 6.283185307179586 * (dial - center) / fs;
    std::vector<float> iq;  // interleaved I,Q
    iq.reserve((up.size() + fs / 2) * 2);
    float peak = 1e-9f;
    for (size_t n = 0; n < up.size(); ++n) {
        const float i = up[n] * static_cast<float>(std::cos(w * n));
        const float q = up[n] * static_cast<float>(std::sin(w * n));
        iq.push_back(i);
        iq.push_back(q);
        peak = std::max(peak, std::max(std::fabs(i), std::fabs(q)));
    }
    for (size_t n = 0; n < fs / 2; ++n) { iq.push_back(0.0f); iq.push_back(0.0f); }
    for (float& v : iq) v = v * 0.7f / peak;

    const uint32_t formats[] = {MFSK_IQ_FORMAT_CF32, MFSK_IQ_FORMAT_CS16, MFSK_IQ_FORMAT_CS8,
                                MFSK_IQ_FORMAT_CU8, MFSK_IQ_FORMAT_CS24};
    for (uint32_t fmt : formats) {
        std::vector<uint8_t> bytes;
        for (float v : iq) {
            switch (fmt) {
            case MFSK_IQ_FORMAT_CF32: {
                uint8_t b[4]; std::memcpy(b, &v, 4);
                bytes.insert(bytes.end(), b, b + 4);
                break;
            }
            case MFSK_IQ_FORMAT_CS16: {
                const int16_t x = static_cast<int16_t>(v * 32768.0f);
                bytes.push_back(x & 0xff); bytes.push_back((x >> 8) & 0xff);
                break;
            }
            case MFSK_IQ_FORMAT_CS8:
                bytes.push_back(static_cast<uint8_t>(static_cast<int8_t>(
                    std::max(-128.0f, std::min(127.0f, std::round(v * 128.0f))))));
                break;
            case MFSK_IQ_FORMAT_CU8:
                bytes.push_back(static_cast<uint8_t>(
                    std::max(0.0f, std::min(255.0f, std::round(v * 128.0f + 128.0f)))));
                break;
            default: {
                const int32_t x = static_cast<int32_t>(std::round(v * 8388608.0f));
                bytes.push_back(x & 0xff); bytes.push_back((x >> 8) & 0xff);
                bytes.push_back((x >> 16) & 0xff);
            }
            }
        }

      if (fmt == MFSK_IQ_FORMAT_CF32) {
        // #601: an FT8 channel decodes early by default. 13 s of IQ (past
        // checkpoint A at 11.8 s, short of the 15 s slot) already gives the
        // row, marked early; the rest of the slot does not queue it again.
        MfskStatus st = MFSK_STATUS_INTERNAL;
        MfskIqReceiver* rx = mfsk_iq_open(fs, center, fmt, 0, &st);
        uint32_t ch = 0;
        if (rx == nullptr || mfsk_iq_add_channel(rx, dial, MFSK_MODE_FT8, nullptr, nullptr, &ch) != MFSK_STATUS_OK) {
            fail("iq", "early: open"); if (rx) mfsk_iq_close(rx); return;
        }
        mfsk_iq_set_time(rx, t0_ns, 0, nullptr);
        const size_t split = static_cast<size_t>(13) * fs * 8;
        mfsk_iq_push(rx, bytes.data(), split);
        int early = 0, later = 0;
        MfskIqDecode d;
        std::memset(&d, 0, sizeof d);
        d.size = sizeof d;
        while (mfsk_iq_poll(rx, &d) == 1) {
            if (std::strstr(d.text, "CQ JA1ABC PM95") == nullptr) continue;
            ++early;
            if (d.stage != MFSK_STAGE_EARLY) fail("iq", "early: the row is not marked MFSK_STAGE_EARLY");
        }
        mfsk_iq_push(rx, bytes.data() + split, bytes.size() - split);
        while (mfsk_iq_poll(rx, &d) == 1) {
            if (std::strstr(d.text, "CQ JA1ABC PM95") != nullptr) ++later;
        }
        if (early != 1 || later != 0) {
            char what[80];
            std::snprintf(what, sizeof what, "early: %d before the slot ended, %d after (want 1, 0)", early, later);
            fail("iq", what);
        }
        mfsk_iq_close(rx);
      }

      for (uint32_t chz : {MFSK_IQ_CHANNELIZER_DIRECT, MFSK_IQ_CHANNELIZER_PFB}) {
        MfskStatus st = MFSK_STATUS_INTERNAL;
        MfskIqReceiver* rx = mfsk_iq_open_with(fs, center, fmt, 0, chz, &st);
        if (rx == nullptr || st != MFSK_STATUS_OK) { fail("iq", "mfsk_iq_open"); return; }
        uint32_t ch = 0xffffffffu;
        if (mfsk_iq_add_channel(rx, dial, MFSK_MODE_FT8, nullptr, nullptr, &ch) != MFSK_STATUS_OK) {
            fail("iq", "add_channel"); mfsk_iq_close(rx); return;
        }
        mfsk_iq_set_time(rx, t0_ns, 0, nullptr);
        // Chunks that split samples.
        for (size_t pos = 0; pos < bytes.size(); pos += 65537) {
            const size_t n = std::min<size_t>(65537, bytes.size() - pos);
            if (mfsk_iq_push(rx, bytes.data() + pos, n) != MFSK_STATUS_OK) {
                fail("iq", "push"); mfsk_iq_close(rx); return;
            }
        }
        bool found = false;
        MfskIqDecode d;
        std::memset(&d, 0, sizeof d);
        d.size = sizeof d;
        while (mfsk_iq_poll(rx, &d) == 1) {
            if (std::strstr(d.text, "CQ JA1ABC PM95") == nullptr) continue;
            found = true;
            if (d.channel != ch || d.mode != MFSK_MODE_FT8) fail("iq", "row names the wrong channel or mode");
            if (std::fabs(d.abs_freq_hz - (dial + d.freq_hz)) > 1e-6) fail("iq", "abs_freq_hz != dial + freq_hz");
            if (std::fabs(d.freq_hz - 1500.0f) > 3.0f) fail("iq", "audio frequency is off");
            if (!d.has_utc || d.slot_start_utc_ns != t0_ns) fail("iq", "slot start UTC is wrong");
        }
        if (!found) {
            char what[80];
            std::snprintf(what, sizeof what, "format %u, channelizer %u: the message did not come out", fmt, chz);
            fail("iq", what);
        }
        if (mfsk_iq_pending(rx) != 0) fail("iq", "queue should be drained");
        if (mfsk_iq_samples_in(rx) != iq.size() / 2) fail("iq", "samples_in is not the count pushed");
        mfsk_iq_close(rx);
      }
    }

    // Refusals are statuses, not crashes.
    MfskStatus st = MFSK_STATUS_OK;
    if (mfsk_iq_open(fs, center, 99, 0, &st) != nullptr || st != MFSK_STATUS_INVALID_ARG) {
        fail("iq", "an unknown format should be INVALID_ARG");
    }
    MfskIqReceiver* rx = mfsk_iq_open(fs, center, MFSK_IQ_FORMAT_CF32, 0, &st);
    if (rx == nullptr) { fail("iq", "open"); return; }
    if (mfsk_iq_add_channel(rx, center - 1000.0, MFSK_MODE_FT8, nullptr, nullptr, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("iq", "a channel on DC should be INVALID_ARG");
    }
    if (mfsk_iq_add_channel(rx, dial, MFSK_MODE_MSK144, nullptr, nullptr, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("iq", "a mode the receiver does not carry should be INVALID_ARG");
    }
    MfskIqDecode d;
    std::memset(&d, 0, sizeof d);
    if (mfsk_iq_poll(nullptr, &d) >= 0) fail("iq", "poll(NULL) should be negative");
    if (mfsk_iq_poll(rx, &d) != 0) fail("iq", "poll on an empty queue should be 0");
    mfsk_iq_close(rx);
    mfsk_iq_close(nullptr);  // a no-op, like every free here
    std::printf("  [iq] all five formats decode through both channelizers, an FT8 row arrives early, refusals are statuses\n");
}

void test_null_handling() {
    std::printf("\n— NULL / invalid-arg handling\n");
    size_t n = 0;

    if (mfsk_params_init(MFSK_MODE_FT8, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "params_init(NULL) should be INVALID_ARG");
    }
    if (mfsk_extras_init(nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "extras_init(NULL) should be INVALID_ARG");
    }
    {
        MfskParams q;
        std::memset(&q, 0, sizeof q);
        q.size = sizeof q;
        if (mfsk_params_init(9999u, &q) != MFSK_STATUS_INVALID_ARG) {
            fail("null", "params_init on a bogus mode should be INVALID_ARG");
        }
        if (mfsk_params_init(MFSK_MODE_MSK144, &q) != MFSK_STATUS_UNKNOWN_PROTOCOL) {
            fail("null", "a mode that is not decoded as a slot has no parameter block");
        }
    }
    if (mfsk_decoder_decode_i16(nullptr, nullptr, 0, 12000, MFSK_PERIOD_NONE,
                                nullptr, 0, &n) >= 0) {
        fail("null", "decode with a NULL decoder should be an error");
    }
    if (mfsk_decoder_copy_info(nullptr, 0, nullptr, 0, &n) >= 0) {
        fail("null", "copy_info with a NULL decoder should be an error");
    }
    if (mfsk_decoder_add_callsign(nullptr, nullptr) >= 0) {
        fail("null", "add_callsign with a NULL decoder should be an error");
    }
    if (mfsk_decoder_last_error(nullptr) != nullptr) {
        fail("null", "last_error(NULL) should be NULL");
    }
    // An out-of-range mode value: a C caller can put any integer in an
    // enum parameter, and the boundary must validate rather than match it
    // as a Rust enum.
    if (mfsk_mode_name(9999u) != nullptr) fail("null", "an unknown mode should have no name");
    if (mfsk_mode_caps(9999u) != 0) fail("null", "an unknown mode should claim no capabilities");
    MfskModeInfo bogus;
    std::memset(&bogus, 0, sizeof bogus);
    bogus.size = sizeof bogus;
    if (mfsk_mode_info(9999u, &bogus) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "mode_info on a bogus mode should be INVALID_ARG");
    }
    MfskStatus bst = MFSK_STATUS_OK;
    if (mfsk_decoder_open(9999u, nullptr, nullptr, &bst) != nullptr ||
        bst != MFSK_STATUS_INVALID_ARG) {
        fail("null", "decoder_open on a bogus mode should be INVALID_ARG");
    }

    if (mfsk_pack77("XXX", "Y2Z", "FN42", nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "pack77 into a NULL buffer should be INVALID_ARG");
    }
    uint8_t m77[77];
    if (mfsk_pack77("XXX", "Y2Z", "FN42", m77) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "an unpackable callsign should fail rather than emit garbage");
    }
    if (mfsk_symbol_count(9999u) != 0 || mfsk_synth_output_len(9999u) != 0) {
        fail("null", "a bogus mode should report no geometry");
    }

    // Freeing null is a no-op, not a crash.
    mfsk_decoder_close(nullptr);
    std::printf("  OK\n");
}

}  // namespace

int main() {
    const uint32_t ver = mfsk_version();
    std::printf("mfsk-ffi version: %u.%u.%u (ABI %u)\n",
                (ver >> 16) & 0xff, (ver >> 8) & 0xff, ver & 0xff,
                mfsk_abi_version());

    test_mode_introspection();
    test_decoder();
    test_unsupported_options();
    test_ft8();
    test_ft8_streaming();
    test_stream_capture();
    test_params();
    test_sniper();
    test_ft4();
    test_fst4();
    test_fst4w();
    test_wspr();
    test_jt9_jt65();
    test_q65();
    test_prefix();
    test_budget();
    test_hash_resolution();
    test_threads_one_decoder_per_thread();
    test_threads_mixed_modes();
    test_jtty();
    test_iq();
    test_null_handling();

    if (g_failures == 0) {
        std::printf("\nALL OK\n");
        return 0;
    }
    std::fprintf(stderr, "\n%d FAILURE(S)\n", g_failures);
    return 1;
}
