// SPDX-License-Identifier: GPL-3.0-or-later
//
// End-to-end C++ driver for the rs-ft8n FFI — encodes a known test
// message for every supported protocol, feeds the synthesised PCM
// back through the matching decoder handle, and verifies the decoded
// text round-trips correctly. Doubles as smoke test for the ABI
// (NULL handling, last-error, samples / message-list lifetimes) and
// as proof that each protocol is actually wired up in the C ABI.
//
// Build: run `./build.sh`.

#include "mfsk.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
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

// Helper: does any decoded message text contain `needle` (case-sensitive)?
// `text` is a fixed inline buffer (issue #205), always NUL-terminated —
// no null check needed, unlike the old heap-`CString`-pointer shape.
bool any_contains(const MfskResultList& list, const char* needle) {
    for (size_t i = 0; i < list.len; ++i) {
        const MfskResult& m = list.items[i];
        if (std::strstr(m.text, needle) != nullptr) {
            return true;
        }
    }
    return false;
}

void print_decodes(const char* proto, const MfskResultList& list) {
    std::printf("  [%s] %zu decode(s):\n", proto, list.len);
    for (size_t i = 0; i < list.len; ++i) {
        const MfskResult& m = list.items[i];
        std::printf("    freq=%7.2f dt=%+.3f snr=%+.1f err=%u pass=%u text='%s'\n",
                    m.freq_hz, m.dt_sec, m.snr_db,
                    m.hard_errors, m.pass,
                    m.text);
    }
}

// ── FT8 ──────────────────────────────────────────────────────────────
void test_ft8() {
    std::printf("— FT8 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FT8", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FT8", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("FT8", list);
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FT8", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── FT8 streaming (mfsk_decode_i16_streaming, issue #246 follow-up) ───
//
// A real C callback invoked from actual C++-compiled code, through the
// generated header — the one thing the Rust-side `tests/streaming_ffi.rs`
// suite can't exercise (it calls the same crate's own functions
// directly, never crossing an actual compiler/ABI boundary the way a
// separate C++ translation unit does).
extern "C" void streaming_collect(const MfskResult* result, void* user_data) {
    auto* out = static_cast<std::vector<std::string>*>(user_data);
    out->emplace_back(result->text);
}

void test_ft8_streaming() {
    std::printf("— FT8 streaming: encode 'CQ JA1ABC PM95' at 1650 Hz → mfsk_decode_i16_streaming\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1650.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FT8-streaming", mfsk_last_error());
        return;
    }
    std::vector<int16_t> i16_samples(pcm.len);
    for (size_t i = 0; i < pcm.len; ++i) {
        float s = pcm.samples[i] * 32767.0f;
        if (s > 32767.0f) s = 32767.0f;
        if (s < -32768.0f) s = -32768.0f;
        i16_samples[i] = static_cast<int16_t>(s);
    }

    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    MfskResultList list{};
    std::vector<std::string> streamed;
    const MfskStatus st = mfsk_decode_i16_streaming(
        dec, i16_samples.data(), i16_samples.size(), 12000, nullptr,
        streaming_collect, &streamed, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FT8-streaming", mfsk_last_error() ? mfsk_last_error() : "decode failed");
    } else {
        print_decodes("FT8-streaming (batch list)", list);
        std::printf("  streamed via callback: %zu\n", streamed.size());
        bool streamed_ok = false;
        for (const auto& s : streamed) {
            if (s.find("JA1ABC") != std::string::npos) streamed_ok = true;
        }
        if (!streamed_ok) {
            fail("FT8-streaming", "callback never delivered JA1ABC");
        }
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FT8-streaming", "batch list missing expected callsign/grid");
        }
        if (streamed.size() != list.len) {
            fail("FT8-streaming", "streamed count != batch list count for one clean candidate");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── FT4 ──────────────────────────────────────────────────────────────
void test_ft4() {
    std::printf("— FT4 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft4("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FT4", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT4);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FT4", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("FT4", list);
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FT4", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── WSPR ─────────────────────────────────────────────────────────────
void test_wspr() {
    std::printf("— WSPR roundtrip: encode 'K1ABC FN42 37' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("WSPR", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_WSPR);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("WSPR", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("WSPR", list);
        if (!any_contains(list, "K1ABC") || !any_contains(list, "FN42")) {
            fail("WSPR", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── JT9 ──────────────────────────────────────────────────────────────
void test_jt9() {
    std::printf("— JT9 roundtrip: encode 'CQ K1ABC FN42' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_jt9("CQ", "K1ABC", "FN42", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("JT9", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_JT9);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("JT9", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("JT9", list);
        if (!any_contains(list, "K1ABC") || !any_contains(list, "FN42")) {
            fail("JT9", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── FST4-60A ─────────────────────────────────────────────────────────
// Gated behind the RUN_FST4_ROUNDTRIP environment variable because the
// 60-s slot + outer 786 432-pt FFT makes this multi-second and not
// every developer wants to wait on it every build.
void test_fst4() {
    if (!std::getenv("RUN_FST4_ROUNDTRIP")) {
        std::printf("— FST4-60A roundtrip: skipped (set RUN_FST4_ROUNDTRIP=1)\n");
        return;
    }
    std::printf("— FST4-60A roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_fst4s60("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FST4", mfsk_last_error());
        return;
    }
    // Pad up to a full 60-s slot with 1 s of leading silence so the
    // outer FFT has the window decode_frame expects.
    constexpr size_t kSlot = 60 * 12000;
    std::vector<float> slot(kSlot, 0.0f);
    const size_t offset = 12000;
    const size_t copy_len = std::min(pcm.len, kSlot - offset);
    std::memcpy(slot.data() + offset, pcm.samples, copy_len * sizeof(float));

    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FST4S60);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, slot.data(), slot.size(), 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FST4", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("FST4", list);
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FST4", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── JT65 ─────────────────────────────────────────────────────────────
void test_jt65() {
    std::printf("— JT65 roundtrip: encode 'CQ K1ABC FN42' at 1270 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_jt65("CQ", "K1ABC", "FN42", 1270.0f, &pcm) != MFSK_STATUS_OK) {
        fail("JT65", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_JT65);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("JT65", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("JT65", list);
        if (!any_contains(list, "K1ABC") || !any_contains(list, "FN42")) {
            fail("JT65", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── Multi-thread stress ─────────────────────────────────────────────
//
// The C API documents `MfskDecoder` as "not Sync — one handle per
// thread". These tests back that up with a real multi-threaded
// driver to catch any accidental shared mutable state in the Rust
// backends (thread_local slots, global FFT planners, etc.) that
// would break under concurrent use.
void test_threads_one_handle_per_thread() {
    std::printf("— threads × 1 handle each: 8 parallel FT8 decodes\n");
    constexpr int kThreads = 8;
    std::atomic<int> ok_count{0};
    std::atomic<int> fail_count{0};
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&ok_count, &fail_count, t]() {
            MfskSamples pcm{};
            if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f + t * 20.0f, &pcm) != MFSK_STATUS_OK) {
                fail_count++;
                return;
            }
            MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
            MfskResultList list{};
            const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
            bool ok = (st == MFSK_STATUS_OK) && any_contains(list, "JA1ABC");
            if (ok) ok_count++; else fail_count++;
            mfsk_result_list_free(&list);
            mfsk_decoder_free(dec);
            mfsk_samples_free(&pcm);
        });
    }
    for (auto& th : ts) th.join();
    std::printf("  → %d/%d OK, %d fail\n",
                ok_count.load(), kThreads, fail_count.load());
    if (ok_count.load() != kThreads) {
        fail("threads", "one-handle-per-thread concurrent decode failed");
    }
}

// Even stronger: one handle shared across threads. Spec says "not
// Sync"; this test verifies the *current* implementation in fact
// stays sound under sharing (no internal mutable state on DecoderInner).
// If this ever starts failing, tighten the spec AND fix the cause.
void test_threads_shared_handle() {
    std::printf("— threads × 1 shared handle: 8 parallel FT8 decodes\n");
    constexpr int kThreads = 8;
    std::atomic<int> ok_count{0};
    std::atomic<int> fail_count{0};
    MfskDecoder* shared = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([shared, &ok_count, &fail_count, t]() {
            MfskSamples pcm{};
            if (mfsk_encode_ft8("CQ", "K1ABC", "FN42", 1500.0f + t * 20.0f, &pcm) != MFSK_STATUS_OK) {
                fail_count++;
                return;
            }
            MfskResultList list{};
            const MfskStatus st = mfsk_decode_f32(shared, pcm.samples, pcm.len, 12000, nullptr, &list);
            bool ok = (st == MFSK_STATUS_OK) && any_contains(list, "K1ABC");
            if (ok) ok_count++; else fail_count++;
            mfsk_result_list_free(&list);
            mfsk_samples_free(&pcm);
        });
    }
    for (auto& th : ts) th.join();
    mfsk_decoder_free(shared);
    std::printf("  → %d/%d OK, %d fail\n",
                ok_count.load(), kThreads, fail_count.load());
    if (ok_count.load() != kThreads) {
        fail("threads-shared", "shared-handle concurrent decode failed");
    }
}

// Mixed-protocol threads: ensures per-thread thread_local state
// (like mfsk_last_error) in the Rust side doesn't cross-contaminate.
void test_threads_mixed_protocols() {
    std::printf("— threads × mixed protocols (FT8 + FT4 + WSPR concurrently)\n");
    std::atomic<int> ok_count{0};
    std::atomic<int> fail_count{0};
    auto run_proto = [&](MfskProtocol proto, auto encode_fn, const char* needle) {
        MfskSamples pcm{};
        if (encode_fn(&pcm) != MFSK_STATUS_OK) { fail_count++; return; }
        MfskDecoder* dec = mfsk_decoder_new(proto);
        MfskResultList list{};
        const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
        if (st == MFSK_STATUS_OK && any_contains(list, needle)) ok_count++;
        else fail_count++;
        mfsk_result_list_free(&list);
        mfsk_decoder_free(dec);
        mfsk_samples_free(&pcm);
    };
    std::thread t_ft8([&] {
        run_proto(MFSK_PROTOCOL_FT8,
                  [](MfskSamples* p) { return mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, p); },
                  "JA1ABC");
    });
    std::thread t_ft4([&] {
        run_proto(MFSK_PROTOCOL_FT4,
                  [](MfskSamples* p) { return mfsk_encode_ft4("CQ", "W1AW", "FN31", 1500.0f, p); },
                  "W1AW");
    });
    std::thread t_wspr([&] {
        run_proto(MFSK_PROTOCOL_WSPR,
                  [](MfskSamples* p) { return mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, p); },
                  "K1ABC");
    });
    t_ft8.join();
    t_ft4.join();
    t_wspr.join();
    std::printf("  → %d/3 OK, %d fail\n", ok_count.load(), fail_count.load());
    if (ok_count.load() != 3) {
        fail("threads-mixed", "mixed-protocol concurrent decode failed");
    }
}

// ── Negative paths ──────────────────────────────────────────────────
void test_null_handling() {
    std::printf("— NULL / invalid-arg handling\n");
    // NULL decoder
    MfskResultList list{};
    MfskStatus st = mfsk_decode_f32(nullptr, nullptr, 0, 12000, nullptr, &list);
    if (st != MFSK_STATUS_INVALID_ARG) {
        fail("null", "expected INVALID_ARG for null decoder");
    }
    // Free NULL pointers — must not crash.
    mfsk_decoder_free(nullptr);
    mfsk_result_list_free(nullptr);
    mfsk_samples_free(nullptr);
    // Unknown callsign at encode time → InvalidArg + meaningful error.
    MfskSamples bogus{};
    st = mfsk_encode_ft8("XXX", "Y2Z", "FN42", 1500.0f, &bogus);
    if (st == MFSK_STATUS_OK) {
        fail("null", "expected pack77 failure for bogus callsigns");
        mfsk_samples_free(&bogus);
    }
}

} // namespace

int main() {
    const uint32_t ver = mfsk_version();
    std::printf("mfsk-ffi version: %u.%u.%u\n",
                (ver >> 16) & 0xff,
                (ver >> 8) & 0xff,
                ver & 0xff);

    test_ft8();
    test_ft8_streaming();
    test_ft4();
    test_fst4();
    test_wspr();
    test_jt9();
    test_jt65();
    test_threads_one_handle_per_thread();
    test_threads_shared_handle();
    test_threads_mixed_protocols();
    test_null_handling();

    if (g_failures == 0) {
        std::printf("\nALL OK\n");
        return 0;
    }
    std::fprintf(stderr, "\n%d FAILURE(S)\n", g_failures);
    return 1;
}
