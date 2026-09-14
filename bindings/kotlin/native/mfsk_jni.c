// SPDX-License-Identifier: GPL-3.0-or-later
//
// JNI shim between the Kotlin binding and the mfsk-ffi C ABI.
//
// The shim holds no decoder logic. It converts JNI types to the plain
// handle/pointer pairs `libmfsk.so` expects and nothing else — which is
// the point: everything interesting stays in one place, and a Kotlin
// consumer gets the same behaviour a C consumer gets.
//
// It is written in C rather than Rust-with-`jni` on purpose. The shim
// `#include`s the generated `mfsk.h`, so building it is another
// compiler reading that header as a real translation unit — the check
// that caught a macro-generated function missing from the header and an
// out-of-range enum argument that segfaulted. A Rust shim would link
// against the crate and see none of that.
//
// Build: `bindings/kotlin/build.sh`.

#include <jni.h>
#include <stdlib.h>
#include <string.h>

#include "mfsk.h"

#define CLS "io/github/mfskcore/"

// ── Helpers ─────────────────────────────────────────────────────────

static void throw_ise(JNIEnv* env, const char* msg) {
    jclass c = (*env)->FindClass(env, "java/lang/IllegalStateException");
    if (c != NULL) (*env)->ThrowNew(env, c, msg);
}

/// Report the ABI's own error text, which is more specific than a
/// status code — the session carries its own slot precisely so a
/// coroutine that hopped threads still sees it.
static void throw_from_session(JNIEnv* env, MfskDecodeSession* s, const char* fallback) {
    const char* detail = (s != NULL) ? mfsk_session_last_error(s) : NULL;
    if (detail == NULL) detail = mfsk_last_error();
    throw_ise(env, detail != NULL ? detail : fallback);
}

// ── Introspection ───────────────────────────────────────────────────

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_Mfsk_nativeAbiVersion(JNIEnv* env, jclass cls) {
    (void)env; (void)cls;
    return (jint)mfsk_abi_version();
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_Mfsk_nativeModeCount(JNIEnv* env, jclass cls) {
    (void)env; (void)cls;
    return (jint)mfsk_mode_count();
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_Mfsk_nativeModeAt(JNIEnv* env, jclass cls, jint index) {
    (void)cls;
    MfskMode m;
    if (mfsk_mode_at((uint32_t)index, &m) != MFSK_STATUS_OK) {
        throw_ise(env, "mfsk_mode_at: index out of range");
        return -1;
    }
    return (jint)m;
}

JNIEXPORT jstring JNICALL
Java_io_github_mfskcore_Mfsk_nativeModeName(JNIEnv* env, jclass cls, jint mode) {
    (void)cls;
    const char* name = mfsk_mode_name((uint32_t)mode);
    return (name != NULL) ? (*env)->NewStringUTF(env, name) : NULL;
}

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_Mfsk_nativeModeCaps(JNIEnv* env, jclass cls, jint mode) {
    (void)env; (void)cls;
    return (jlong)mfsk_mode_caps((uint32_t)mode);
}

/// Mode geometry as a flat `int[]`, so the binding needs no per-field
/// JNI field lookups. Order matches `Mfsk.ModeInfo`'s constructor.
JNIEXPORT jintArray JNICALL
Java_io_github_mfskcore_Mfsk_nativeModeInfo(JNIEnv* env, jclass cls, jint mode) {
    (void)cls;
    MfskModeInfo info;
    memset(&info, 0, sizeof info);
    info.size = sizeof info;
    if (mfsk_mode_info((uint32_t)mode, &info) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
        return NULL;
    }
    jint vals[6] = {
        (jint)info.ntones,
        (jint)info.slot_samples_12k,
        (jint)info.fec_k,
        (jint)info.n_symbols,
        (jint)info.decode_fft1_size,
        (jint)(info.tx_start_offset_s * 1000.0f),  /* ms, to stay integral */
    };
    jintArray out = (*env)->NewIntArray(env, 6);
    if (out == NULL) return NULL;
    (*env)->SetIntArrayRegion(env, out, 0, 6, vals);
    return out;
}

// ── Runtime ─────────────────────────────────────────────────────────

/// Worker-thread hooks, which are the reason this exists.
///
/// A rayon worker is a plain pthread; JNI forbids touching a JNIEnv
/// from a thread the VM has not attached. Without these the decode
/// callback could not reach Kotlin at all from a worker thread.
///
/// **`AsDaemon` is load-bearing.** `AttachCurrentThread` makes the
/// thread a *non-daemon* JVM thread, and the JVM will not exit while
/// one is alive. Rayon's pool threads are never joined — they live for
/// the process — so attaching them the ordinary way means the JVM hangs
/// on exit, forever, after everything has otherwise succeeded.
///
/// That is not hypothetical: the first CI run of this binding printed
/// `ALL OK` sixty seconds in and then sat for seventy-two minutes until
/// it was cancelled. `AttachCurrentThreadAsDaemon` is the fix, and
/// `build.sh`'s `timeout` on the JVM stage is what turns a regression
/// here into a failure instead of a stall.
static JavaVM* g_vm = NULL;

/// The VM, captured the moment the library loads.
///
/// `nativeConfigureRuntime` used to be the only thing that set this,
/// which was enough while the hooks were the only users. The decode
/// callback is not: it can fire on a thread of rayon's **global** pool,
/// which nothing attaches, in a process that never called
/// `configureRuntime` at all. Taking the VM here removes that ordering
/// requirement entirely.
JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    (void)reserved;
    g_vm = vm;
    return JNI_VERSION_1_6;
}

static void on_thread_start(uint32_t index, void* user) {
    (void)index; (void)user;
    if (g_vm == NULL) return;
    JNIEnv* env = NULL;
    (*g_vm)->AttachCurrentThreadAsDaemon(g_vm, (void**)&env, NULL);
}

static void on_thread_stop(uint32_t index, void* user) {
    (void)index; (void)user;
    if (g_vm != NULL) (*g_vm)->DetachCurrentThread(g_vm);
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_Mfsk_nativeConfigureRuntime(
        JNIEnv* env, jclass cls, jint threads, jint stackBytes) {
    (void)cls;
    (void)env;  /* the VM comes from JNI_OnLoad */
    MfskRuntimeConfig cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.size = sizeof cfg;
    cfg.num_threads = (uint32_t)(threads > 0 ? threads : 0);
    cfg.thread_stack_bytes = (uint32_t)(stackBytes > 0 ? stackBytes : 0);
    cfg.on_thread_start = on_thread_start;
    cfg.on_thread_stop = on_thread_stop;
    return (jint)mfsk_runtime_configure(&cfg);
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_Mfsk_nativeThreadCount(JNIEnv* env, jclass cls) {
    (void)env; (void)cls;
    return (jint)mfsk_runtime_thread_count();
}

// ── Rows ────────────────────────────────────────────────────────────

/// `MfskDecode`'s constructor signature, in one place: it is the thing
/// that fails at run time rather than compile time when a field is
/// added on the Kotlin side and forgotten here.
static jmethodID row_ctor(JNIEnv* env, jclass rowCls) {
    return (*env)->GetMethodID(env, rowCls, "<init>", "(ILjava/lang/String;FFFFFIIIZ)V");
}

/// One C row as one Kotlin `MfskDecode`. Every field is copied — the
/// row pointer a callback receives is valid only for that call.
static jobject make_row(JNIEnv* env, jclass rowCls, jmethodID ctor, const MfskDecode* r) {
    jstring text = (*env)->NewStringUTF(env, r->text);
    if (text == NULL) return NULL;
    jobject obj = (*env)->NewObject(
        env, rowCls, ctor,
        (jint)r->mode, text,
        (jfloat)r->freq_hz, (jfloat)r->dt_sec, (jfloat)r->snr_db,
        (jfloat)r->sync_score, (jfloat)r->sync_cv,
        (jint)r->hard_errors, (jint)r->info_bits, (jint)r->pass,
        (jboolean)((r->flags & MFSK_DECODE_FLAG_HASH_RESOLVED) != 0));
    (*env)->DeleteLocalRef(env, text);
    return obj;
}

// ── Decode callback ─────────────────────────────────────────────────
//
// `mfsk_session_set_on_decode` delivers rows as they are found, on top
// of the array written at the end of the call. Two JNI hazards, both
// load-bearing:
//
// 1. **The thread may not be attached.** With rayon the callback fires
//    from a worker; only a *private* pool built by `configureRuntime`
//    gets the attach hooks, so a process that never called it has
//    workers the VM has never seen. The callback attaches on demand,
//    `AsDaemon` for the same reason the hooks use it — a non-daemon
//    attach on a thread that is never joined keeps the JVM alive
//    forever.
// 2. **`FindClass` on such a thread looks in the wrong place.** It
//    resolves through the *system* class loader when there is no Java
//    frame on the stack, which cannot see application classes. So the
//    class and both method IDs are looked up on the thread that
//    installs the listener, where a Java frame exists, and kept as
//    global references.

typedef struct {
    jobject listener;   /* global ref to MfskDecodeListener */
    jclass rowCls;      /* global ref to MfskDecode */
    jmethodID onDecode;
    jmethodID rowCtor;
} CallbackCtx;

static void callback_ctx_free(JNIEnv* env, CallbackCtx* ctx) {
    if (ctx == NULL) return;
    if (ctx->listener != NULL) (*env)->DeleteGlobalRef(env, ctx->listener);
    if (ctx->rowCls != NULL) (*env)->DeleteGlobalRef(env, ctx->rowCls);
    free(ctx);
}

static void on_decode(const struct MfskDecode* row, void* user) {
    CallbackCtx* ctx = (CallbackCtx*)user;
    if (ctx == NULL || row == NULL || g_vm == NULL) return;

    JNIEnv* env = NULL;
    const jint attached = (*g_vm)->GetEnv(g_vm, (void**)&env, JNI_VERSION_1_6);
    if (attached == JNI_EDETACHED) {
        if ((*g_vm)->AttachCurrentThreadAsDaemon(g_vm, (void**)&env, NULL) != JNI_OK) return;
    } else if (attached != JNI_OK) {
        return;
    }

    /* The worker has no Java frame, so nothing pops local refs for us. */
    if ((*env)->PushLocalFrame(env, 4) != 0) return;
    jobject obj = make_row(env, ctx->rowCls, ctx->rowCtor, row);
    if (obj != NULL) {
        (*env)->CallVoidMethod(env, ctx->listener, ctx->onDecode, obj);
        /* A throwing listener cannot propagate into a rayon worker, and
           leaving an exception pending makes every later JNI call in
           this callback illegal. Report it and clear it — the decode
           itself is unaffected, and the authoritative rows still come
           back from the call that set this. */
        if ((*env)->ExceptionCheck(env)) {
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
        }
    }
    (*env)->PopLocalFrame(env, NULL);
}

// ── Session ─────────────────────────────────────────────────────────

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskSession_nativeOpen(JNIEnv* env, jclass cls, jint mode) {
    (void)cls;
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open((uint32_t)mode, NULL, &st);
    if (s == NULL) {
        throw_ise(env, mfsk_last_error());
        return 0;
    }
    return (jlong)(intptr_t)s;
}

/// Install (or clear, with a null `listener`) the decode callback.
///
/// Takes the previous context back and frees it, returning the new one,
/// so the Kotlin side owns exactly one `long` per session and cannot
/// leak a global ref by replacing a listener.
JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskSession_nativeSetOnDecode(
        JNIEnv* env, jclass cls, jlong handle, jobject listener, jlong oldCtx) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (s == NULL) { throw_ise(env, "session is closed"); return oldCtx; }

    if (listener == NULL) {
        mfsk_session_set_on_decode(s, NULL, NULL);
        callback_ctx_free(env, (CallbackCtx*)(intptr_t)oldCtx);
        return 0;
    }

    CallbackCtx* ctx = (CallbackCtx*)calloc(1, sizeof *ctx);
    if (ctx == NULL) { throw_ise(env, "out of memory"); return oldCtx; }

    /* The *interface*, not `GetObjectClass(listener)`. A Kotlin `fun
       interface` is satisfied by a lambda, which on a modern compiler
       is an invokedynamic-spun hidden class; taking the method ID from
       the interface sidesteps the question entirely and dispatches
       virtually just the same. */
    jclass listenerCls = (*env)->FindClass(env, CLS "MfskDecodeListener");
    jclass rowCls = (*env)->FindClass(env, CLS "MfskDecode");
    if (listenerCls == NULL || rowCls == NULL) { free(ctx); return oldCtx; }
    ctx->onDecode = (*env)->GetMethodID(
        env, listenerCls, "onDecode", "(L" CLS "MfskDecode;)V");
    ctx->rowCtor = row_ctor(env, rowCls);
    if (ctx->onDecode == NULL || ctx->rowCtor == NULL) { free(ctx); return oldCtx; }
    ctx->listener = (*env)->NewGlobalRef(env, listener);
    ctx->rowCls = (jclass)(*env)->NewGlobalRef(env, rowCls);
    if (ctx->listener == NULL || ctx->rowCls == NULL) {
        callback_ctx_free(env, ctx);
        throw_ise(env, "could not retain the listener");
        return oldCtx;
    }

    const MfskStatus st = mfsk_session_set_on_decode(s, on_decode, ctx);
    if (st != MFSK_STATUS_OK) {
        callback_ctx_free(env, ctx);
        throw_from_session(env, s, "set_on_decode failed");
        return oldCtx;
    }
    /* Only now is the old one unreachable from the decode side. */
    callback_ctx_free(env, (CallbackCtx*)(intptr_t)oldCtx);
    return (jlong)(intptr_t)ctx;
}

/// Close the session and release any listener it carried.
///
/// Order matters: the session holds the raw `CallbackCtx*`, so it has
/// to stop being able to call it before the context is freed. Nothing
/// can be decoding here — `close()` and `decode()` are not safe to
/// call concurrently on one session in any case, which is what
/// "single-threaded by design" means.
JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskSession_nativeClose(
        JNIEnv* env, jclass cls, jlong handle, jlong ctx) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (s != NULL) mfsk_session_set_on_decode(s, NULL, NULL);
    mfsk_session_close(s);
    callback_ctx_free(env, (CallbackCtx*)(intptr_t)ctx);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskSession_nativeAddCallsign(
        JNIEnv* env, jclass cls, jlong handle, jstring call) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    const char* c = (*env)->GetStringUTFChars(env, call, NULL);
    if (c == NULL) return;
    const MfskStatus st = mfsk_session_add_callsign(s, c);
    (*env)->ReleaseStringUTFChars(env, call, c);
    if (st != MFSK_STATUS_OK) throw_from_session(env, s, "add_callsign failed");
}

/// Decode one slot, returning an `Object[]` of `MfskDecode`.
///
/// Rows come back in memory this shim owns for the length of the call —
/// the ABI writes into a caller array, so there is nothing to free and
/// no way to leak one if a Java exception unwinds past this frame.
JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskSession_nativeDecode(
        JNIEnv* env, jclass cls, jlong handle,
        jshortArray samples, jint sampleRate) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (s == NULL) { throw_ise(env, "session is closed"); return NULL; }

    const jsize n = (*env)->GetArrayLength(env, samples);
    jshort* pcm = (*env)->GetShortArrayElements(env, samples, NULL);
    if (pcm == NULL) return NULL;

    enum { kCap = 64 };
    MfskDecode rows[kCap];
    memset(rows, 0, sizeof rows);
    for (int i = 0; i < kCap; ++i) rows[i].size = sizeof rows[i];
    size_t len = 0;
    const MfskStatus st = mfsk_session_decode_i16(
        s, (const int16_t*)pcm, (size_t)n, (uint32_t)sampleRate,
        NULL, rows, kCap, &len);
    (*env)->ReleaseShortArrayElements(env, samples, pcm, JNI_ABORT);

    if (st != MFSK_STATUS_OK) {
        // A short buffer reports the count it needed, so say that
        // rather than "decode failed" — it is a different problem.
        if (len > kCap) {
            throw_ise(env, "more decodes than this shim's row buffer holds");
        } else {
            throw_from_session(env, s, "decode failed");
        }
        return NULL;
    }

    jclass rowCls = (*env)->FindClass(env, CLS "MfskDecode");
    if (rowCls == NULL) return NULL;
    jmethodID ctor = row_ctor(env, rowCls);
    if (ctor == NULL) return NULL;

    jobjectArray out = (*env)->NewObjectArray(env, (jsize)len, rowCls, NULL);
    if (out == NULL) return NULL;
    for (size_t i = 0; i < len; ++i) {
        jobject obj = make_row(env, rowCls, ctor, &rows[i]);
        if (obj == NULL) return NULL;
        (*env)->SetObjectArrayElement(env, out, (jsize)i, obj);
        (*env)->DeleteLocalRef(env, obj);
    }
    return out;
}

// ── Transmit ────────────────────────────────────────────────────────

/// Pack → tones → PCM in one call, returning `short[]`.
///
/// The three stages are separate in C because a caller may want the
/// tone sequence; a Kotlin consumer almost never does, so the binding
/// offers the composition and keeps the stages out of the API surface
/// until someone asks.
JNIEXPORT jshortArray JNICALL
Java_io_github_mfskcore_Mfsk_nativeSynthesize(
        JNIEnv* env, jclass cls, jint mode,
        jstring a, jstring b, jstring c, jfloat freqHz) {
    (void)cls;
    const char* sa = (*env)->GetStringUTFChars(env, a, NULL);
    const char* sb = (*env)->GetStringUTFChars(env, b, NULL);
    const char* sc = (*env)->GetStringUTFChars(env, c, NULL);
    uint8_t msg[77];
    const MfskStatus pst = (sa && sb && sc) ? mfsk_pack77(sa, sb, sc, msg)
                                            : MFSK_STATUS_INVALID_ARG;
    if (sa) (*env)->ReleaseStringUTFChars(env, a, sa);
    if (sb) (*env)->ReleaseStringUTFChars(env, b, sb);
    if (sc) (*env)->ReleaseStringUTFChars(env, c, sc);
    if (pst != MFSK_STATUS_OK) { throw_ise(env, mfsk_last_error()); return NULL; }

    const size_t nTones = mfsk_symbol_count((uint32_t)mode);
    if (nTones == 0) { throw_ise(env, "this mode has no tone stage"); return NULL; }
    uint8_t* tones = (uint8_t*)malloc(nTones);
    if (tones == NULL) { throw_ise(env, "out of memory"); return NULL; }
    size_t got = 0;
    if (mfsk_message_to_tones((uint32_t)mode, msg, tones, nTones, &got) != MFSK_STATUS_OK) {
        free(tones);
        throw_ise(env, mfsk_last_error());
        return NULL;
    }

    const size_t nPcm = mfsk_synth_output_len((uint32_t)mode);
    jshortArray out = (*env)->NewShortArray(env, (jsize)nPcm);
    if (out == NULL) { free(tones); return NULL; }
    jshort* dst = (*env)->GetShortArrayElements(env, out, NULL);
    if (dst == NULL) { free(tones); return NULL; }
    size_t wrote = 0;
    const MfskStatus sst = mfsk_tones_to_i16(
        (uint32_t)mode, tones, got, freqHz, 8000,
        (int16_t*)dst, nPcm, &wrote);
    free(tones);
    (*env)->ReleaseShortArrayElements(env, out, dst, 0);
    if (sst != MFSK_STATUS_OK) { throw_ise(env, mfsk_last_error()); return NULL; }
    return out;
}
