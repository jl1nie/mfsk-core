// SPDX-License-Identifier: GPL-3.0-or-later
//
// End-to-end C++ driver for the rs-ft8n FFI — encodes a known test
// message for every supported protocol, feeds the synthesised PCM
// back through the matching decoder handle, and verifies the decoded
// text round-trips correctly. Doubles as smoke test for the ABI
// (NULL handling, last-error, sample lifetimes) and as proof that each
// mode is actually wired up in the C ABI.
//
// Decode results go into memory this file owns: nothing here frees a
// pointer the library allocated, which is the category the v2 surface
// removed from the decode path.
//
// Build: run `./build.sh`.

#include "mfsk.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

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

/// The convenience encoder, for modes with no tone stage. Writes into a
/// buffer this file owns — ask for the size first.
std::vector<int16_t> encode_i16(Encoder enc, const char* a, const char* b,
                                const char* c, float freq) {
    std::vector<int16_t> out;
    size_t need = 0;
    enc(a, b, c, freq, nullptr, 0, &need);
    if (need == 0) return out;
    std::vector<float> pcm(need);
    size_t got = 0;
    if (enc(a, b, c, freq, pcm.data(), pcm.size(), &got) != MFSK_STATUS_OK) return out;
    out.resize(got);
    for (size_t i = 0; i < got; ++i) {
        out[i] = static_cast<int16_t>(pcm[i] * 32767.0f);
    }
    return out;
}

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

MfskDecodeParams defaults_for(MfskMode mode) {
    MfskDecodeParams p;
    std::memset(&p, 0, sizeof p);
    p.size = sizeof p;
    if (mfsk_decode_params_init(mode, &p) != MFSK_STATUS_OK) {
        fail(mfsk_mode_name(mode), "mfsk_decode_params_init failed");
    }
    return p;
}

/// Open a session, decode one slot, assert the text turns up.
void session_roundtrip(const char* tag, MfskMode mode,
                       const std::vector<int16_t>& audio, const char* needle) {
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(mode, nullptr, &st);
    if (s == nullptr || st != MFSK_STATUS_OK) {
        fail(tag, mfsk_last_error());
        return;
    }
    Rows rows;
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000, nullptr,
                                rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail(tag, mfsk_session_last_error(s));
    } else {
        print_rows(tag, rows);
        if (!rows.contains(needle)) fail(tag, "expected text missing");
        for (size_t i = 0; i < rows.len; ++i) {
            if (rows.items[i].mode != mode) fail(tag, "row reports the wrong mode");
        }
    }
    mfsk_session_close(s);
}

// ── Mode introspection (FFI v2 slice 1) ─────────────────────────────
//
// The point of this surface is that a C consumer stops hardcoding a
// capability matrix, so the test has to be written the way a consumer
// would: enumerate what the build has, ask each mode what it supports,
// and act on the answer. Anything asserted from a list written here
// would be testing this file, not the library.
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

    // Walk every mode the way a UI populating a picker would.
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

        // The name must round-trip through the string form, which is
        // what a config file or a CLI flag will carry.
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
            // Anything the decode handle drives must publish a usable
            // default search, or a caller has nothing to start from.
            MfskDecodeDefaults d;
            std::memset(&d, 0, sizeof d);
            d.size = sizeof d;
            if (mfsk_mode_defaults(m, &d) != MFSK_STATUS_OK) {
                fail(name, "drives the decode handle but publishes no defaults");
                return;
            }
            if (!(d.freq_max_hz > d.freq_min_hz) || d.max_cand == 0) {
                fail(name, "publishes an unusable default search");
                return;
            }
            // The trap this field exists for: FT4's threshold is on a
            // different scale from everyone else's.
            if (d.sync_scale == MFSK_SYNC_SCALE_BASELINE_NORMALISED && !(d.sync_min > 1.0f)) {
                fail(name, "baseline-normalised sync_min is at or below the noise floor");
                return;
            }
            if (info.decode_fft1_size == 0) {
                fail(name, "drives the decode handle but reports no slot transform size");
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
        if (std::strncmp(name, "FST4-", 5) == 0) {
            fst4_submodes++;
        }
    }

    // The equivalence this whole redesign is named for: the pre-v2 ABI
    // could address exactly one FST4 sub-mode (`Fst4s60 = 5`), so the
    // other four were unreachable from C for decode and encode alike.
    if (fst4_submodes != 5) {
        fail("introspect", "expected all five FST4 sub-modes to be addressable");
        return;
    }
    if (with_handle < 7) {
        fail("introspect", "FT8 + FT4 + five FST4 should all drive the decode handle");
        return;
    }
    if (snipers != 1) {
        fail("introspect", "exactly one mode should claim the sniper");
        return;
    }
    std::printf("  %d mode(s) drive the decode handle, %d FST4 sub-mode(s) addressable\n",
                with_handle, fst4_submodes);
    std::printf("  largest slot transform: %s at %u points\n", widest_name, widest_fft);

    // A mode this build lacks and a name that is not a mode must be
    // distinguishable — a typo is not the same problem as a missing
    // feature, and today a caller cannot tell.
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

    // Size versioning: an older caller declares a smaller struct and
    // must get only its prefix written. Emulated by declaring a size
    // that stops before the geometry fields.
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

// ── v2 decode session ───────────────────────────────────────────────
//
// Written the way a consumer would: init params from the mode, open a
// session, decode into memory the caller owns. Nothing here frees a
// pointer the library allocated, which is the whole point — that
// category is what makes Kotlin and Swift wrappers leak when an
// exception unwinds past the free.
void test_session_decode() {
    std::printf("\n— v2 decode session: params → open → rows into caller memory\n");

    const std::vector<int16_t> audio =
        synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);

    MfskDecodeParams p;
    std::memset(&p, 0, sizeof p);
    p.size = sizeof p;
    if (mfsk_decode_params_init(MFSK_MODE_FT8, &p) != MFSK_STATUS_OK) {
        fail("session", "mfsk_decode_params_init failed");
        return;
    }
    std::printf("  FT8 defaults: band [%.0f, %.0f] sync_min %.2f max_cand %u\n",
                p.freq_min_hz, p.freq_max_hz, p.sync_min, p.max_cand);

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, &p, &st);
    if (s == nullptr || st != MFSK_STATUS_OK) {
        fail("session", mfsk_last_error());
        return;
    }

    MfskDecode rows[8];
    std::memset(rows, 0, sizeof rows);
    for (auto& r : rows) r.size = sizeof r;
    size_t n = 0;
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000,
                                nullptr, rows, 8, &n) != MFSK_STATUS_OK) {
        fail("session", mfsk_session_last_error(s));
        mfsk_session_close(s);
        return;
    }
    std::printf("  %zu decode(s):\n", n);
    bool found = false;
    for (size_t i = 0; i < n; ++i) {
        std::printf("    mode=%d freq=%7.2f dt=%+.3f snr=%+.1f cv=%.3f "
                    "info=%u pass=%u text='%s'\n",
                    static_cast<int>(rows[i].mode), rows[i].freq_hz, rows[i].dt_sec,
                    rows[i].snr_db, rows[i].sync_cv, rows[i].info_bits,
                    rows[i].pass, rows[i].text);
        if (std::strstr(rows[i].text, "JA1ABC") != nullptr) found = true;
        if (rows[i].mode != MFSK_MODE_FT8) {
            fail("session", "row reports the wrong mode");
        }
        if (rows[i].info_bits != 91) {
            fail("session", "FT8 is LDPC(174,91); info_bits should be 91");
        }
    }
    if (!found) {
        fail("session", "did not decode the signal it was given");
    }

    // FEC bits come from the session, not from a pointer in the row.
    size_t need = 0;
    if (mfsk_session_copy_info(s, 0, nullptr, 0, &need) != MFSK_STATUS_INVALID_ARG ||
        need != 91) {
        fail("session", "copy_info should report the size it needs");
    } else {
        std::vector<uint8_t> bits(need);
        size_t got = 0;
        if (mfsk_session_copy_info(s, 0, bits.data(), bits.size(), &got) != MFSK_STATUS_OK ||
            got != need) {
            fail("session", "copy_info failed with a correctly sized buffer");
        }
    }

    // A short buffer reports the count needed rather than truncating.
    size_t needed = 0;
    MfskDecode one;
    std::memset(&one, 0, sizeof one);
    one.size = sizeof one;
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000,
                                nullptr, &one, 0, &needed) != MFSK_STATUS_INVALID_ARG) {
        fail("session", "a zero-capacity buffer should report INVALID_ARG");
    } else if (needed != n) {
        fail("session", "*out_len should be the count needed");
    }

    mfsk_session_close(s);

    // Asking a mode for something it does not have fails at open, with
    // a message — not silently at decode, which is what the pre-v2
    // options handle did with six of its eleven fields.
    MfskDecodeParams bad;
    std::memset(&bad, 0, sizeof bad);
    bad.size = sizeof bad;
    mfsk_decode_params_init(MFSK_MODE_FT4, &bad);
    bad.sic_early = true;
    MfskStatus badst = MFSK_STATUS_OK;
    if (mfsk_session_open(MFSK_MODE_FT4, &bad, &badst) != nullptr ||
        badst != MFSK_STATUS_UNSUPPORTED) {
        fail("session", "sic_early on FT4 should be refused at open");
    } else {
        std::printf("  refused sic_early on FT4: %s\n", mfsk_last_error());
    }

    // A mode with no decode handle says so, naming the bit to check.
    MfskStatus wst = MFSK_STATUS_OK;
    if (mfsk_session_open(MFSK_MODE_WSPR, nullptr, &wst) != nullptr ||
        wst != MFSK_STATUS_UNSUPPORTED) {
        fail("session", "WSPR has no decode handle and should refuse");
    }

    // Every mode that claims the handle must open one.
    const uint32_t total = mfsk_mode_count();
    int opened = 0;
    for (uint32_t i = 0; i < total; ++i) {
        MfskMode m;
        if (mfsk_mode_at(i, &m) != MFSK_STATUS_OK) continue;
        if ((mfsk_mode_caps(m) & MFSK_CAP_DECODE_HANDLE) == 0) continue;
        MfskStatus ost = MFSK_STATUS_INTERNAL;
        MfskDecodeSession* sess = mfsk_session_open(m, nullptr, &ost);
        if (sess == nullptr || ost != MFSK_STATUS_OK) {
            fail(mfsk_mode_name(m), "claims MFSK_CAP_DECODE_HANDLE but will not open");
        } else {
            opened++;
            mfsk_session_close(sess);
        }
    }
    std::printf("  opened a session for all %d handle-driving mode(s)\n", opened);

    std::printf("  OK\n");
}

// ── Per-mode round trips ────────────────────────────────────────────

void test_ft8() {
    std::printf("\n— FT8 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    session_roundtrip("FT8", MFSK_MODE_FT8,
                      synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f),
                      "JA1ABC");
}

void test_ft4() {
    std::printf("\n— FT4 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    session_roundtrip("FT4", MFSK_MODE_FT4,
                      synth_slot(MFSK_MODE_FT4, "CQ", "JA1ABC", "PM95", 1500.0f),
                      "JA1ABC");
}

void test_fst4() {
    if (std::getenv("RUN_FST4_ROUNDTRIP") == nullptr) {
        std::printf("\n— FST4-60A roundtrip: skipped (set RUN_FST4_ROUNDTRIP=1)\n");
        return;
    }
    std::printf("\n— FST4-60A roundtrip, and all five sub-modes addressable\n");
    session_roundtrip("FST4-60A", MFSK_MODE_FST4S60,
                      synth_slot(MFSK_MODE_FST4S60, "CQ", "JA1ABC", "PM95", 1500.0f),
                      "JA1ABC");

    // The four sub-modes the pre-v2 ABI could not address at all:
    // MfskProtocol had one FST4 entry, so 15/30/120/300 were
    // unreachable from C for decode and encode alike.
    const MfskMode others[] = {MFSK_MODE_FST4S15, MFSK_MODE_FST4S30,
                               MFSK_MODE_FST4S120, MFSK_MODE_FST4S300};
    for (MfskMode m : others) {
        MfskStatus st = MFSK_STATUS_INTERNAL;
        MfskDecodeSession* s = mfsk_session_open(m, nullptr, &st);
        if (s == nullptr) {
            fail(mfsk_mode_name(m), "unreachable — this is the hole v2 closes");
        } else {
            mfsk_session_close(s);
        }
    }
    std::printf("  all five FST4 sub-modes open a session\n");
}

// ── Modes that are not driven by the decode session ─────────────────
//
// Q65 takes a nominal start sample and a time tolerance; WSPR/JT9/JT65
// have no builder, and JT9/JT65 are point decodes at a known carrier
// rather than searches. They keep their own entry points and share the
// row type — MFSK_CAP_DECODE_HANDLE is the bit that says which is which.

void test_wspr() {
    std::printf("\n— WSPR: its own entry point (no decode handle)\n");
    size_t need = 0;
    mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, nullptr, 0, &need);
    std::vector<float> pcm(need);
    size_t got = 0;
    if (mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, pcm.data(), pcm.size(), &got)
            != MFSK_STATUS_OK) {
        fail("WSPR", mfsk_last_error());
        return;
    }
    std::vector<int16_t> audio(got);
    for (size_t i = 0; i < got; ++i) {
        audio[i] = static_cast<int16_t>(pcm[i] * 32767.0f);
    }

    Rows rows;
    if (mfsk_wspr_decode(audio.data(), audio.size(), 12000,
                         rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail("WSPR", mfsk_last_error());
        return;
    }
    print_rows("WSPR", rows);
    if (!rows.contains("K1ABC")) fail("WSPR", "expected K1ABC");
}

void test_jt9() {
    // 1350 Hz on purpose: the pre-v2 ABI hardcoded 1500 with no way to
    // say otherwise, so the frequency being an argument is the thing
    // under test as much as the decode is.
    std::printf("\n— JT9: point decode at a caller-chosen 1350 Hz\n");
    std::vector<int16_t> audio = encode_i16(mfsk_encode_jt9, "CQ", "K1ABC", "FN42", 1350.0f);
    Rows rows;
    if (mfsk_jt9_decode_at(audio.data(), audio.size(), 12000, 1350.0f,
                           rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail("JT9", mfsk_last_error());
        return;
    }
    print_rows("JT9", rows);
    if (!rows.contains("K1ABC")) fail("JT9", "expected K1ABC");
}

void test_jt65() {
    std::printf("\n— JT65: point decode at 1270 Hz\n");
    std::vector<int16_t> audio = encode_i16(mfsk_encode_jt65, "CQ", "K1ABC", "FN42", 1270.0f);
    Rows rows;
    if (mfsk_jt65_decode_at(audio.data(), audio.size(), 12000, 1270.0f,
                            rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail("JT65", mfsk_last_error());
        return;
    }
    print_rows("JT65", rows);
    if (!rows.contains("K1ABC")) fail("JT65", "expected K1ABC");
}

// Q65, and the two enums that reach C only because `cbindgen.toml`
// asks for them. Every `mfsk_q65_*` function takes its sub-mode as
// `uint32_t` — deliberately, so an out-of-range value from a config
// file is a C int rather than an invalid Rust discriminant — which left
// `MfskQ65SubMode` mentioned by no signature and therefore absent from
// the header. A consumer had to write `0` and remember what it meant.
// This test is written the way it should now be possible to write it:
// by name.
void test_q65() {
    std::printf("\n— Q65-30A: encode by name, plain and fading decode\n");

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

    Rows rows;
    if (mfsk_q65_decode(MFSK_Q65_SUB_MODE_A30, pcm.data(), got, 12000, nullptr,
                        rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail("Q65", mfsk_last_error());
        return;
    }
    print_rows("Q65-30A", rows);
    if (!rows.contains("K1ABC")) fail("Q65", "expected K1ABC");
    for (size_t i = 0; i < rows.len; ++i) {
        if (rows.items[i].mode != MFSK_MODE_Q65A30) {
            fail("Q65", "a Q65-30A row should report MFSK_MODE_Q65A30");
        }
    }

    // The fading decoder takes its channel model by name too. 0.1 s is
    // the b90_ts a clean signal tolerates; the point here is the
    // argument, not the sensitivity.
    Rows fading;
    if (mfsk_q65_decode_fading(MFSK_Q65_SUB_MODE_A30, pcm.data(), got, 12000,
                               0.1f, MFSK_Q65_FADING_MODEL_GAUSSIAN, nullptr,
                               fading.items, 16, &fading.len) != MFSK_STATUS_OK) {
        fail("Q65 fading", mfsk_last_error());
        return;
    }
    print_rows("Q65-30A fading", fading);
    if (!fading.contains("K1ABC")) fail("Q65 fading", "expected K1ABC");
}

// The three strategies the capability word used to advertise with no
// way to reach them from C. Written the way a consumer has to write
// them — set the option on the session, then look at what the decode
// did differently.
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

size_t decode_count(MfskDecodeSession* s, const std::vector<int16_t>& audio, Rows& rows) {
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000, nullptr,
                                rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail("strategies", mfsk_session_last_error(s));
        return 0;
    }
    return rows.len;
}

void test_budget_known_cache() {
    std::printf("\n— budget / known / fft cache: the bits are reachable now\n");
    const std::vector<int16_t> audio = two_stations();

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, nullptr, &st);
    if (s == nullptr) { fail("strategies", mfsk_last_error()); return; }

    Rows base;
    const size_t full = decode_count(s, audio, base);
    std::printf("  unbudgeted: %zu decode(s)\n", full);

    MfskBudgetReport rep;
    std::memset(&rep, 0, sizeof rep);
    rep.size = sizeof rep;
    if (mfsk_session_last_budget(s, &rep) != MFSK_STATUS_OK) {
        fail("budget", "last_budget failed");
    } else if (rep.exhausted || rep.candidates_skipped != 0) {
        fail("budget", "no budget was set, so nothing should report as cut");
    } else if (rep.cut_at_sync != -1) {
        fail("budget", "absent cut_at_sync must be -1");
    }

    // A predicate that refuses everything has to cut the search, be
    // polled, and say so afterwards.
    g_budget_polls = 0;
    if (mfsk_session_set_budget(s, budget_refuse_everything, nullptr) != MFSK_STATUS_OK) {
        fail("budget", mfsk_session_last_error(s));
    }
    Rows cut;
    const size_t cutN = decode_count(s, audio, cut);
    std::printf("  budgeted to nothing: %zu decode(s), %d poll(s)\n", cutN, g_budget_polls);
    if (g_budget_polls == 0) fail("budget", "the predicate was never polled");
    if (cutN >= full) fail("budget", "a refusing budget found as much as no budget");

    std::memset(&rep, 0, sizeof rep);
    rep.size = sizeof rep;
    mfsk_session_last_budget(s, &rep);
    std::printf("  report: exhausted=%d skipped=%u ran=%u cut_at_sync=%d\n",
                (int)rep.exhausted, rep.candidates_skipped, rep.stages_run, rep.cut_at_sync);
    if (!rep.exhausted) fail("budget", "work was cut and the report does not say so");

    g_budget_polls = 0;
    mfsk_session_set_budget(s, budget_allow_everything, nullptr);
    Rows allowed;
    if (decode_count(s, audio, allowed) != full) {
        fail("budget", "a budget that allows everything changed the result");
    }
    mfsk_session_set_budget(s, nullptr, nullptr);

    // Known: the second pass over the same slot has nothing new in it.
    if (mfsk_session_keep_known(s, true) != MFSK_STATUS_OK) {
        fail("known", mfsk_session_last_error(s));
    }
    Rows firstPass;
    const size_t firstN = decode_count(s, audio, firstPass);
    if (mfsk_session_known_count(s) != firstN) {
        fail("known", "the session should be carrying the first pass's rows");
    }
    Rows secondPass;
    const size_t secondN = decode_count(s, audio, secondPass);
    std::printf("  known: %zu then %zu\n", firstN, secondN);
    if (secondN != 0) fail("known", "a known signal was reported twice");
    mfsk_session_keep_known(s, false);
    if (mfsk_session_known_count(s) != 0) fail("known", "keep(false) should drop the list");

    // FFT cache: same answer, and a stale one is not reused.
    if (mfsk_session_keep_fft_cache(s, true) != MFSK_STATUS_OK) {
        fail("cache", mfsk_session_last_error(s));
    }
    Rows cached1, cached2;
    decode_count(s, audio, cached1);
    decode_count(s, audio, cached2);
    if (cached1.len != full || cached2.len != full) {
        fail("cache", "reusing the slot transform changed the decode");
    }
    const std::vector<int16_t> other =
        synth_slot(MFSK_MODE_FT8, "CQ", "VK3NV", "QF22", 1800.0f);
    Rows elsewhere;
    decode_count(s, other, elsewhere);
    if (!elsewhere.contains("VK3NV")) {
        fail("cache", "a cache from other audio was reused");
    }
    std::printf("  cache: %zu, %zu, then %zu on different audio\n",
                cached1.len, cached2.len, elsewhere.len);

    mfsk_session_close(s);
}

// ── Streaming delivery ──────────────────────────────────────────────
//
// A real C callback invoked from actual C++-compiled code, through the
// generated header — the one thing `tests/streaming_ffi.rs` cannot
// exercise, since it calls the same crate's functions directly and
// never crosses a compiler/ABI boundary the way a separate translation
// unit does.

extern "C" void streaming_collect(const MfskDecode* row, void* user_data) {
    auto* out = static_cast<std::vector<std::string>*>(user_data);
    if (row != nullptr) out->emplace_back(row->text);
}

void test_ft8_streaming() {
    std::printf("\n— streaming: mfsk_session_set_on_decode fires as decodes are found\n");
    std::vector<int16_t> audio =
        synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1650.0f);

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, nullptr, &st);
    if (s == nullptr) { fail("streaming", mfsk_last_error()); return; }

    std::vector<std::string> streamed;
    if (mfsk_session_set_on_decode(s, streaming_collect, &streamed) != MFSK_STATUS_OK) {
        fail("streaming", "set_on_decode failed");
    }

    Rows rows;
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000, nullptr,
                                rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail("streaming", mfsk_session_last_error(s));
    } else {
        print_rows("FT8 streaming", rows);
        std::printf("  streamed via callback: %zu\n", streamed.size());
        if (streamed.empty()) fail("streaming", "the callback never fired");
        if (!rows.contains("JA1ABC")) fail("streaming", "expected JA1ABC");
        if (streamed.size() != rows.len) {
            fail("streaming", "streamed count should match the array for one clean candidate");
        }
    }
    mfsk_session_close(s);
}

// ── Streaming capture and the slot grid ─────────────────────────────
//
// Generalised from `mfsk-ffi-ft8`'s FT8-only, i16-only front end. The
// ring is sized from the mode, and **time enters as a parameter** —
// the library reads no clock, which is what keeps this usable from a
// phone that was backgrounded and from a replayed recording alike.
void test_stream_capture() {
    std::printf("\n— streaming capture: push → slot ready → fused decode\n");

    const std::vector<int16_t> slot =
        synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskStream* stream = mfsk_stream_open(MFSK_MODE_FT8, 12000, &st);
    if (stream == nullptr || st != MFSK_STATUS_OK) {
        fail("stream", mfsk_last_error());
        return;
    }
    mfsk_stream_set_epoch(stream, 1700000000.0);

    if (mfsk_stream_slot_ready(stream)) {
        fail("stream", "a fresh stream should have no slot ready");
    }
    // Push in chunks, the way a UAC reader delivers.
    const size_t kChunk = 1920;
    for (size_t i = 0; i < slot.size(); i += kChunk) {
        const size_t n = (i + kChunk < slot.size()) ? kChunk : slot.size() - i;
        if (mfsk_stream_push_i16(stream, slot.data() + i, n) != MFSK_STATUS_OK) {
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
    std::printf("  buffered %zu sample(s)\n", mfsk_stream_buffered(stream));

    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, nullptr, &st);
    if (s == nullptr) {
        fail("stream", mfsk_last_error());
        mfsk_stream_close(stream);
        return;
    }
    Rows rows;
    double slot_utc = -1.0;
    const MfskStatus dst = mfsk_session_decode_stream(
        s, stream, nullptr, rows.items, 16, &rows.len, &slot_utc);
    if (dst != MFSK_STATUS_OK) {
        fail("stream", mfsk_session_last_error(s));
    } else {
        print_rows("stream", rows);
        std::printf("  slot started at UTC %.3f\n", slot_utc);
        if (!rows.contains("JA1ABC")) fail("stream", "expected JA1ABC");
        if (slot_utc != 1700000000.0) {
            fail("stream", "the reported slot time should be the epoch the host declared");
        }
        if (mfsk_stream_slot_ready(stream)) {
            fail("stream", "the fused decode should have consumed the slot");
        }
    }

    // Polling before a slot is ready is "not yet", not a failure the
    // caller has to guard against.
    size_t none = 99;
    if (mfsk_session_decode_stream(s, stream, nullptr, rows.items, 16, &none, nullptr)
            != MFSK_STATUS_UNSUPPORTED || none != 0) {
        fail("stream", "an empty stream should report UNSUPPORTED with *out_len = 0");
    }

    mfsk_session_close(s);
    mfsk_stream_close(stream);

    // The ring is sized per mode — an FT8-sized one would be wrong in
    // both directions for FT4 and FST4-300.
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
        const std::vector<int16_t> quiet(info.slot_samples_12k, 0);
        mfsk_stream_push_i16(st2, quiet.data(), quiet.size());
        if (!mfsk_stream_slot_ready(st2)) {
            fail(mfsk_mode_name(m), "a full slot should be ready");
        }
        mfsk_stream_close(st2);
    }

    // A mode with no decode handle has nothing to feed.
    MfskStatus wst = MFSK_STATUS_OK;
    if (mfsk_stream_open(MFSK_MODE_WSPR, 12000, &wst) != nullptr ||
        wst != MFSK_STATUS_UNSUPPORTED) {
        fail("stream", "WSPR has no decode handle and should refuse a stream");
    }
    mfsk_stream_close(nullptr);
    std::printf("  OK\n");
}

// ── Every parameter reaches the decoder ─────────────────────────────
//
// The pre-v2 ABI accepted eleven options and silently dropped six of
// them depending on protocol, and silently *upgraded* a seventh. This
// drives each field and checks the decode survives, then checks that a
// per-call override applies to that call and does not stick.

void test_params() {
    std::printf("\n— params: strictness / eq_mode / freq_hint / sic / ap all reach the decoder\n");
    std::vector<int16_t> audio =
        synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);

    MfskDecodeParams p = defaults_for(MFSK_MODE_FT8);
    p.strictness   = MFSK_STRICTNESS_DEEP;
    p.eq_mode      = MFSK_EQ_MODE_LOCAL;
    p.freq_hint_hz = 1500.0f;
    p.sic_rounds   = 2;
    p.has_ap_hint  = true;
    std::snprintf(p.ap_call1, MFSK_AP_FIELD_LEN, "%s", "JA1ABC");
    std::snprintf(p.ap_call2, MFSK_AP_FIELD_LEN, "%s", "CQ");

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, &p, &st);
    if (s == nullptr) { fail("params", mfsk_last_error()); return; }

    Rows rows;
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000, nullptr,
                                rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
        fail("params", mfsk_session_last_error(s));
    } else {
        print_rows("params", rows);
        if (!rows.contains("JA1ABC")) fail("params", "every option on lost the signal");
    }

    MfskDecodeParams narrow = p;
    narrow.freq_min_hz  = 2500.0f;
    narrow.freq_max_hz  = 2900.0f;
    narrow.freq_hint_hz = 2700.0f;
    Rows away;
    mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000, &narrow,
                            away.items, 16, &away.len);
    Rows back;
    mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000, nullptr,
                            back.items, 16, &back.len);
    if (away.contains("JA1ABC")) {
        fail("params", "a 2500-2900 Hz override still found a 1500 Hz signal");
    }
    if (!back.contains("JA1ABC")) {
        fail("params", "the per-call override leaked into the next call");
    }
    std::printf("  per-call override applied and did not stick\n");
    mfsk_session_close(s);
}

// ── The narrow-band search ──────────────────────────────────────────

void test_sniper() {
    std::printf("\n— narrow-band search: FT8 only; FT4 refuses and gets AP wide-band instead\n");

    std::vector<int16_t> ft8 = synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f);
    MfskDecodeParams p = defaults_for(MFSK_MODE_FT8);
    p.freq_hint_hz = 1500.0f;
    p.search_hz    = 250.0f;
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, &p, &st);
    if (s == nullptr) {
        fail("sniper", mfsk_last_error());
    } else {
        Rows rows;
        if (mfsk_session_decode_i16(s, ft8.data(), ft8.size(), 12000, nullptr,
                                    rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
            fail("sniper", mfsk_session_last_error(s));
        } else {
            print_rows("FT8 narrow", rows);
            if (!rows.contains("JA1ABC")) fail("sniper", "aimed at it and missed");
        }
        mfsk_session_close(s);
    }

    // FT4 refuses: the sniper is the receive half of narrowing an
    // analogue roofing filter, which a contest protocol has no use for.
    MfskDecodeParams q = defaults_for(MFSK_MODE_FT4);
    q.freq_hint_hz = 1200.0f;
    q.search_hz    = 250.0f;
    MfskStatus qst = MFSK_STATUS_OK;
    if (mfsk_session_open(MFSK_MODE_FT4, &q, &qst) != nullptr ||
        qst != MFSK_STATUS_UNSUPPORTED) {
        fail("sniper", "FT4 should refuse a narrow-band search");
    } else {
        std::printf("  FT4 refused: %s\n", mfsk_last_error());
    }

    // And the AP hint the sniper looked like it was *for* reaches FT4
    // anyway, through the ordinary wide-band decode. That coupling was
    // an accident; this is the line that says it is over.
    std::vector<int16_t> ft4 = synth_slot(MFSK_MODE_FT4, "CQ", "JA1ABC", "PM95", 1200.0f);
    MfskDecodeParams w = defaults_for(MFSK_MODE_FT4);
    w.has_ap_hint = true;
    std::snprintf(w.ap_call1, MFSK_AP_FIELD_LEN, "%s", "JA1ABC");
    std::snprintf(w.ap_call2, MFSK_AP_FIELD_LEN, "%s", "CQ");
    MfskDecodeSession* fs = mfsk_session_open(MFSK_MODE_FT4, &w, &st);
    if (fs == nullptr) {
        fail("sniper", mfsk_last_error());
    } else {
        Rows rows;
        if (mfsk_session_decode_i16(fs, ft4.data(), ft4.size(), 12000, nullptr,
                                    rows.items, 16, &rows.len) != MFSK_STATUS_OK) {
            fail("sniper", mfsk_session_last_error(fs));
        } else {
            print_rows("FT4 wide-band + AP", rows);
            if (!rows.contains("JA1ABC")) fail("sniper", "AP did not reach FT4");
        }
        mfsk_session_close(fs);
    }
}

// ── Threading ───────────────────────────────────────────────────────
//
// **A session is single-threaded**, and that is a deliberate change.
// The pre-v2 handle carried one `protocol` field, so sharing it across
// threads was harmless; a session owns a callsign hash table it mutates
// on every decode plus the previous slot's rows, so sharing one would
// be a data race. The supported shape is one session per thread.

void test_threads_one_session_per_thread() {
    std::printf("\n— threads × 1 session each: 8 parallel FT8 decodes\n");
    constexpr int kThreads = 8;
    std::atomic<int> ok_count{0};
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&ok_count, t]() {
            std::vector<int16_t> audio =
                synth_slot(MFSK_MODE_FT8, "CQ", "JA1ABC", "PM95", 1500.0f + t * 20.0f);
            MfskStatus st = MFSK_STATUS_INTERNAL;
            MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, nullptr, &st);
            if (s == nullptr) return;
            Rows rows;
            const MfskStatus dst = mfsk_session_decode_i16(
                s, audio.data(), audio.size(), 12000, nullptr,
                rows.items, 16, &rows.len);
            if (dst == MFSK_STATUS_OK && rows.contains("JA1ABC")) ok_count++;
            mfsk_session_close(s);
        });
    }
    for (auto& th : ts) th.join();
    std::printf("  → %d/%d OK\n", ok_count.load(), kThreads);
    if (ok_count.load() != kThreads) {
        fail("threads", "one-session-per-thread concurrent decode failed");
    }
}

void test_threads_mixed_modes() {
    std::printf("\n— threads × mixed modes (FT8 + FT4 concurrently)\n");
    std::atomic<int> ok_count{0};
    std::vector<std::thread> ts;
    const struct { MfskMode mode; } work[] = {
        {MFSK_MODE_FT8}, {MFSK_MODE_FT4}, {MFSK_MODE_FT8}, {MFSK_MODE_FT4},
    };
    constexpr int kJobs = 4;
    for (const auto& w : work) {
        ts.emplace_back([&ok_count, w]() {
            std::vector<int16_t> audio =
                synth_slot(w.mode, "CQ", "JA1ABC", "PM95", 1500.0f);
            MfskStatus st = MFSK_STATUS_INTERNAL;
            MfskDecodeSession* s = mfsk_session_open(w.mode, nullptr, &st);
            if (s == nullptr) return;
            Rows rows;
            const MfskStatus dst = mfsk_session_decode_i16(
                s, audio.data(), audio.size(), 12000, nullptr,
                rows.items, 16, &rows.len);
            if (dst == MFSK_STATUS_OK && rows.contains("JA1ABC")) ok_count++;
            mfsk_session_close(s);
        });
    }
    for (auto& th : ts) th.join();
    std::printf("  → %d/%d OK\n", ok_count.load(), kJobs);
    if (ok_count.load() != kJobs) {
        fail("threads", "mixed-mode concurrent decode failed");
    }
}

// ── NULL / invalid-arg handling ─────────────────────────────────────

void test_null_handling() {
    std::printf("\n— NULL / invalid-arg handling\n");
    size_t n = 0;

    if (mfsk_decode_params_init(MFSK_MODE_FT8, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "params_init(NULL) should be INVALID_ARG");
    }
    if (mfsk_session_decode_i16(nullptr, nullptr, 0, 12000, nullptr,
                                nullptr, 0, &n) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "decode with a NULL session should be INVALID_ARG");
    }
    if (mfsk_session_copy_info(nullptr, 0, nullptr, 0, &n) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "copy_info with a NULL session should be INVALID_ARG");
    }
    if (mfsk_session_add_callsign(nullptr, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "add_callsign with a NULL session should be INVALID_ARG");
    }
    if (mfsk_session_last_error(nullptr) != nullptr) {
        fail("null", "last_error(NULL) should be NULL");
    }
    if (mfsk_wspr_decode(nullptr, 0, 12000, nullptr, 0, &n) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "wspr_decode(NULL) should be INVALID_ARG");
    }
    // An out-of-range mode value: a C caller can put any integer in an
    // enum parameter, and the boundary must validate rather than match
    // it as a Rust enum. This exact call used to segfault.
    if (mfsk_mode_name(9999u) != nullptr) {
        fail("null", "an unknown mode should have no name");
    }
    if (mfsk_mode_caps(9999u) != 0) {
        fail("null", "an unknown mode should claim no capabilities");
    }
    MfskModeInfo bogus;
    std::memset(&bogus, 0, sizeof bogus);
    bogus.size = sizeof bogus;
    if (mfsk_mode_info(9999u, &bogus) != MFSK_STATUS_INVALID_ARG) {
        fail("null", "mode_info on a bogus mode should be INVALID_ARG");
    }
    MfskStatus bst = MFSK_STATUS_OK;
    if (mfsk_session_open(9999u, nullptr, &bst) != nullptr ||
        bst != MFSK_STATUS_INVALID_ARG) {
        fail("null", "session_open on a bogus mode should be INVALID_ARG");
    }
    if (mfsk_q65_decode(9999u, nullptr, 0, 12000, nullptr, nullptr, 0, &n)
            != MFSK_STATUS_INVALID_ARG) {
        fail("null", "q65_decode with a bogus sub-mode should be INVALID_ARG");
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
    mfsk_session_close(nullptr);
    std::printf("  OK\n");
}

}  // namespace

int main() {
    const uint32_t ver = mfsk_version();
    std::printf("mfsk-ffi version: %u.%u.%u (ABI %u)\n",
                (ver >> 16) & 0xff, (ver >> 8) & 0xff, ver & 0xff,
                mfsk_abi_version());

    test_mode_introspection();
    test_session_decode();
    test_ft8();
    test_ft8_streaming();
    test_stream_capture();
    test_params();
    test_sniper();
    test_ft4();
    test_fst4();
    test_wspr();
    test_jt9();
    test_jt65();
    test_q65();
    test_budget_known_cache();
    test_threads_one_session_per_thread();
    test_threads_mixed_modes();
    test_null_handling();

    if (g_failures == 0) {
        std::printf("\nALL OK\n");
        return 0;
    }
    std::fprintf(stderr, "\n%d FAILURE(S)\n", g_failures);
    return 1;
}
