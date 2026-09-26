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
#include <stdio.h>
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
    return (*env)->GetMethodID(env, rowCls, "<init>", "(ILjava/lang/String;FFFFFIIIZZ)V");
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
        (jboolean)((r->flags & MFSK_DECODE_FLAG_HASH_RESOLVED) != 0),
        (jboolean)((r->flags & MFSK_DECODE_FLAG_COPIED_LAST_TX) != 0));
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

// ── Budget predicate ────────────────────────────────────────────────
//
// Same two hazards as the decode callback — the worker may be
// unattached, and its `FindClass` looks in the wrong place — handled
// the same way. One difference worth knowing: this is polled *per
// candidate*, so it is the one upcall in this shim that runs often
// enough for its own cost to matter. A predicate that compares
// `System.nanoTime()` against a captured deadline is what it is for;
// anything heavier belongs on the Kotlin side of a boolean.

typedef struct {
    jobject check;      /* global ref to MfskBudgetCheck */
    jmethodID shouldContinue;
} BudgetCtx;

static void budget_ctx_free(JNIEnv* env, BudgetCtx* ctx) {
    if (ctx == NULL) return;
    if (ctx->check != NULL) (*env)->DeleteGlobalRef(env, ctx->check);
    free(ctx);
}

static bool on_budget(void* user) {
    BudgetCtx* ctx = (BudgetCtx*)user;
    if (ctx == NULL || g_vm == NULL) return true;

    JNIEnv* env = NULL;
    const jint attached = (*g_vm)->GetEnv(g_vm, (void**)&env, JNI_VERSION_1_6);
    if (attached == JNI_EDETACHED) {
        if ((*g_vm)->AttachCurrentThreadAsDaemon(g_vm, (void**)&env, NULL) != JNI_OK) {
            return true;  /* cannot ask — carry on rather than cut blindly */
        }
    } else if (attached != JNI_OK) {
        return true;
    }

    const jboolean go = (*env)->CallBooleanMethod(env, ctx->check, ctx->shouldContinue);
    if ((*env)->ExceptionCheck(env)) {
        /* A throwing predicate is not an answer. Report it, clear it,
           and keep decoding: stopping would turn a bug in the caller's
           clock into silently missing decodes. */
        (*env)->ExceptionDescribe(env);
        (*env)->ExceptionClear(env);
        return true;
    }
    return go == JNI_TRUE;
}

// ── Decode parameters ───────────────────────────────────────────────

/// `MfskDecodeParams` crosses JNI as three flat arrays rather than as an
/// object the shim reads field by field, so there is no per-field
/// `GetFieldID` to get wrong and a field added later is one more slot in
/// each layout below and a line in each of `read_params` / `write_params`.
/// The order is the contract with `MfskDecodeParams.toArrays` in Mfsk.kt.
///
///   floats: freq_min_hz, freq_max_hz, sync_min, freq_hint_hz (NaN unset),
///           search_hz, tx_freq_hz (NaN unset), nb_ftol_hz
///   ints:   max_cand, depth, strictness, eq_mode, sic_rounds, sic_early,
///           has_ap_hint, nb_percent, nb_sweep_step
///   ap:     ap_call1, ap_call2, ap_grid (null entries are empty)
enum { kParamFloats = 7, kParamInts = 9, kParamAp = 3 };

/// Build the C struct from the arrays. False, with an exception pending,
/// on a malformed array — the shim checks lengths because a short array
/// read as a longer layout would be plausible garbage, not an error.
static bool read_params(JNIEnv* env, jfloatArray fa, jintArray ia, jobjectArray ap,
                        MfskDecodeParams* p) {
    if (fa == NULL || ia == NULL || ap == NULL
        || (*env)->GetArrayLength(env, fa) != kParamFloats
        || (*env)->GetArrayLength(env, ia) != kParamInts
        || (*env)->GetArrayLength(env, ap) != kParamAp) {
        throw_ise(env, "decode parameter arrays have the wrong shape");
        return false;
    }
    jfloat f[kParamFloats];
    jint v[kParamInts];
    (*env)->GetFloatArrayRegion(env, fa, 0, kParamFloats, f);
    (*env)->GetIntArrayRegion(env, ia, 0, kParamInts, v);

    memset(p, 0, sizeof *p);
    p->size = sizeof *p;
    p->freq_min_hz = f[0];
    p->freq_max_hz = f[1];
    p->sync_min = f[2];
    p->freq_hint_hz = f[3];
    p->search_hz = f[4];
    p->tx_freq_hz = f[5];
    p->nb_ftol_hz = f[6];
    p->max_cand = (uint32_t)v[0];
    /* Enums go in as the integers the caller wrote; the library checks
       every discriminant before reading it as a Rust enum. */
    p->depth = (MfskDecodeDepth)v[1];
    p->strictness = (MfskStrictness)v[2];
    p->eq_mode = (MfskEqMode)v[3];
    p->sic_rounds = (uint8_t)v[4];
    p->sic_early = v[5] != 0;
    p->has_ap_hint = v[6] != 0;
    p->nb_percent = (uint8_t)v[7];
    p->nb_sweep_step = (uint8_t)v[8];

    char* dst[kParamAp] = { p->ap_call1, p->ap_call2, p->ap_grid };
    for (int i = 0; i < kParamAp; ++i) {
        jstring js = (jstring)(*env)->GetObjectArrayElement(env, ap, i);
        if ((*env)->ExceptionCheck(env)) return false;
        if (js == NULL) continue;
        const char* u = (*env)->GetStringUTFChars(env, js, NULL);
        if (u == NULL) { (*env)->DeleteLocalRef(env, js); return false; }
        /* Truncated to the ABI's inline capacity, NUL kept: a callsign
           is at most 13 characters and the field holds 15. */
        strncpy(dst[i], u, sizeof p->ap_call1 - 1);
        (*env)->ReleaseStringUTFChars(env, js, u);
        (*env)->DeleteLocalRef(env, js);
    }
    return true;
}

/// The reverse, for `nativeParamsInit`: what the library wrote into
/// `p`, back into the caller's arrays. The AP fields come back empty from
/// `mfsk_decode_params_init`, so they are not carried.
static void write_params(JNIEnv* env, const MfskDecodeParams* p,
                         jfloatArray fa, jintArray ia) {
    const jfloat f[kParamFloats] = {
        p->freq_min_hz, p->freq_max_hz, p->sync_min, p->freq_hint_hz,
        p->search_hz, p->tx_freq_hz, p->nb_ftol_hz,
    };
    const jint v[kParamInts] = {
        (jint)p->max_cand, (jint)p->depth, (jint)p->strictness, (jint)p->eq_mode,
        (jint)p->sic_rounds, p->sic_early ? 1 : 0, p->has_ap_hint ? 1 : 0,
        (jint)p->nb_percent, (jint)p->nb_sweep_step,
    };
    (*env)->SetFloatArrayRegion(env, fa, 0, kParamFloats, f);
    (*env)->SetIntArrayRegion(env, ia, 0, kParamInts, v);
}

/// `mfsk_decode_params_init` for `mode`, into the caller's arrays.
JNIEXPORT void JNICALL
Java_io_github_mfskcore_Mfsk_nativeParamsInit(
        JNIEnv* env, jclass cls, jint mode, jfloatArray fa, jintArray ia) {
    (void)cls;
    if (fa == NULL || ia == NULL
        || (*env)->GetArrayLength(env, fa) != kParamFloats
        || (*env)->GetArrayLength(env, ia) != kParamInts) {
        throw_ise(env, "decode parameter arrays have the wrong shape");
        return;
    }
    MfskDecodeParams p;
    memset(&p, 0, sizeof p);
    p.size = sizeof p;
    if (mfsk_decode_params_init((uint32_t)mode, &p) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
        return;
    }
    write_params(env, &p, fa, ia);
}

// ── Session ─────────────────────────────────────────────────────────

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskSession_nativeOpen(
        JNIEnv* env, jclass cls, jint mode,
        jfloatArray fa, jintArray ia, jobjectArray ap) {
    (void)cls;
    MfskStatus st = MFSK_STATUS_INTERNAL;
    /* Null arrays mean "the mode's defaults", which is what NULL params
       means to the library. */
    MfskDecodeParams params;
    const MfskDecodeParams* pp = NULL;
    if (fa != NULL) {
        if (!read_params(env, fa, ia, ap, &params)) return 0;
        pp = &params;
    }
    MfskDecodeSession* s = mfsk_session_open((uint32_t)mode, pp, &st);
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

/// Install (or clear) the budget predicate, taking the previous context
/// back and freeing it — the same ownership shape as the listener.
JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskSession_nativeSetBudget(
        JNIEnv* env, jclass cls, jlong handle, jobject check, jlong oldCtx) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (s == NULL) { throw_ise(env, "session is closed"); return oldCtx; }

    if (check == NULL) {
        mfsk_session_set_budget(s, NULL, NULL);
        budget_ctx_free(env, (BudgetCtx*)(intptr_t)oldCtx);
        return 0;
    }

    BudgetCtx* ctx = (BudgetCtx*)calloc(1, sizeof *ctx);
    if (ctx == NULL) { throw_ise(env, "out of memory"); return oldCtx; }
    jclass checkCls = (*env)->FindClass(env, CLS "MfskBudgetCheck");
    if (checkCls == NULL) { free(ctx); return oldCtx; }
    ctx->shouldContinue = (*env)->GetMethodID(env, checkCls, "shouldContinue", "()Z");
    if (ctx->shouldContinue == NULL) { free(ctx); return oldCtx; }
    ctx->check = (*env)->NewGlobalRef(env, check);
    if (ctx->check == NULL) { free(ctx); throw_ise(env, "could not retain the predicate"); return oldCtx; }

    const MfskStatus st = mfsk_session_set_budget(s, on_budget, ctx);
    if (st != MFSK_STATUS_OK) {
        budget_ctx_free(env, ctx);
        throw_from_session(env, s, "set_budget failed");
        return oldCtx;
    }
    budget_ctx_free(env, (BudgetCtx*)(intptr_t)oldCtx);
    return (jlong)(intptr_t)ctx;
}

/// The last decode's budget report as a flat `int[]`, in the order
/// `MfskBudgetReport`'s Kotlin constructor takes: exhausted (0/1),
/// skipped, ran, cut_at_sync (-1 for absent), cut_at_score in
/// millionths (INT_MIN for absent, since an int array cannot carry a
/// NaN).
JNIEXPORT jintArray JNICALL
Java_io_github_mfskcore_MfskSession_nativeLastBudget(
        JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    MfskBudgetReport rep;
    memset(&rep, 0, sizeof rep);
    rep.size = sizeof rep;
    if (mfsk_session_last_budget(s, &rep) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
        return NULL;
    }
    jint vals[5] = {
        (jint)(rep.exhausted ? 1 : 0),
        (jint)rep.candidates_skipped,
        (jint)rep.stages_run,
        (jint)rep.cut_at_sync,
        rep.cut_at_score == rep.cut_at_score /* not NaN */
            ? (jint)(rep.cut_at_score * 1000000.0f)
            : (jint)0x80000000,
    };
    jintArray out = (*env)->NewIntArray(env, 5);
    if (out == NULL) return NULL;
    (*env)->SetIntArrayRegion(env, out, 0, 5, vals);
    return out;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskSession_nativeKeepKnown(
        JNIEnv* env, jclass cls, jlong handle, jboolean keep) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (mfsk_session_keep_known(s, keep == JNI_TRUE) != MFSK_STATUS_OK) {
        throw_from_session(env, s, "keep_known failed");
    }
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskSession_nativeKnownCount(
        JNIEnv* env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    return (jint)mfsk_session_known_count((const MfskDecodeSession*)(intptr_t)handle);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskSession_nativeKeepFftCache(
        JNIEnv* env, jclass cls, jlong handle, jboolean keep) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (mfsk_session_keep_fft_cache(s, keep == JNI_TRUE) != MFSK_STATUS_OK) {
        throw_from_session(env, s, "keep_fft_cache failed");
    }
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
        JNIEnv* env, jclass cls, jlong handle, jlong ctx, jlong budgetCtx) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (s != NULL) {
        mfsk_session_set_on_decode(s, NULL, NULL);
        mfsk_session_set_budget(s, NULL, NULL);
    }
    mfsk_session_close(s);
    callback_ctx_free(env, (CallbackCtx*)(intptr_t)ctx);
    budget_ctx_free(env, (BudgetCtx*)(intptr_t)budgetCtx);
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
        jshortArray samples, jint sampleRate,
        jfloatArray fa, jintArray ia, jobjectArray ap) {
    (void)cls;
    MfskDecodeSession* s = (MfskDecodeSession*)(intptr_t)handle;
    if (s == NULL) { throw_ise(env, "session is closed"); return NULL; }

    /* A per-call override applies to this call only; null arrays leave
       the session's own parameters in force. */
    MfskDecodeParams params;
    const MfskDecodeParams* pp = NULL;
    if (fa != NULL) {
        if (!read_params(env, fa, ia, ap, &params)) return NULL;
        pp = &params;
    }

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
        pp, rows, kCap, &len);
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

// ── Q65 ─────────────────────────────────────────────────────────────
//
// `mfsk_q65_decode_ex` and the two handles WSJT-X keeps for it. Same
// approach as `MfskDecodeParams`: the struct crosses as flat arrays, in the
// order below, which is the contract with `MfskQ65Params.floatArray` /
// `intArray` / `stringArray` in Mfsk.kt.
//
//   floats: freq_min_hz, freq_max_hz, nominal_start_s, t_early_s, t_late_s,
//           score_threshold, rx_freq_hz (NaN unset), ftol_hz,
//           fading_b90_ts (NaN unset)
//   ints:   max_cand, pileup, eme_delay, max_drift, fading_model, ap_list,
//           has_ap_hint
//   strings: ap_call1, ap_call2, ap_grid, ap_report,
//            list_my_call, list_his_call, list_his_grid (null is empty)
enum { kQ65Floats = 9, kQ65Ints = 7, kQ65Strings = 7 };

static bool read_q65_params(JNIEnv* env, jfloatArray fa, jintArray ia, jobjectArray sa,
                            MfskQ65Params* p) {
    if (fa == NULL || ia == NULL || sa == NULL
        || (*env)->GetArrayLength(env, fa) != kQ65Floats
        || (*env)->GetArrayLength(env, ia) != kQ65Ints
        || (*env)->GetArrayLength(env, sa) != kQ65Strings) {
        throw_ise(env, "Q65 parameter arrays have the wrong shape");
        return false;
    }
    jfloat f[kQ65Floats];
    jint v[kQ65Ints];
    (*env)->GetFloatArrayRegion(env, fa, 0, kQ65Floats, f);
    (*env)->GetIntArrayRegion(env, ia, 0, kQ65Ints, v);

    memset(p, 0, sizeof *p);
    p->size = sizeof *p;
    p->freq_min_hz = f[0];
    p->freq_max_hz = f[1];
    p->nominal_start_s = f[2];
    p->t_early_s = f[3];
    p->t_late_s = f[4];
    p->score_threshold = f[5];
    p->rx_freq_hz = f[6];
    p->ftol_hz = f[7];
    p->fading_b90_ts = f[8];
    p->max_cand = (uint32_t)v[0];
    p->pileup = (uint32_t)v[1];
    p->eme_delay = (uint32_t)v[2];
    p->max_drift = (uint32_t)v[3];
    p->fading_model = (uint32_t)v[4];
    p->ap_list = (uint32_t)v[5];
    p->has_ap_hint = (uint32_t)v[6];

    char* dst[kQ65Strings] = {
        p->ap_call1, p->ap_call2, p->ap_grid, p->ap_report,
        p->list_my_call, p->list_his_call, p->list_his_grid,
    };
    for (int i = 0; i < kQ65Strings; ++i) {
        jstring js = (jstring)(*env)->GetObjectArrayElement(env, sa, i);
        if ((*env)->ExceptionCheck(env)) return false;
        if (js == NULL) continue;
        const char* u = (*env)->GetStringUTFChars(env, js, NULL);
        if (u == NULL) { (*env)->DeleteLocalRef(env, js); return false; }
        strncpy(dst[i], u, sizeof p->ap_call1 - 1);
        (*env)->ReleaseStringUTFChars(env, js, u);
        (*env)->DeleteLocalRef(env, js);
    }
    return true;
}

/// `mfsk_q65_params_init` for `mode`, into the caller's arrays. The strings
/// come back empty, so they are not carried.
JNIEXPORT void JNICALL
Java_io_github_mfskcore_Mfsk_nativeQ65ParamsInit(
        JNIEnv* env, jclass cls, jint mode, jfloatArray fa, jintArray ia) {
    (void)cls;
    if (fa == NULL || ia == NULL
        || (*env)->GetArrayLength(env, fa) != kQ65Floats
        || (*env)->GetArrayLength(env, ia) != kQ65Ints) {
        throw_ise(env, "Q65 parameter arrays have the wrong shape");
        return;
    }
    MfskQ65Params p;
    memset(&p, 0, sizeof p);
    p.size = sizeof p;
    if (mfsk_q65_params_init((uint32_t)mode, &p) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
        return;
    }
    const jfloat f[kQ65Floats] = {
        p.freq_min_hz, p.freq_max_hz, p.nominal_start_s, p.t_early_s, p.t_late_s,
        p.score_threshold, p.rx_freq_hz, p.ftol_hz, p.fading_b90_ts,
    };
    const jint v[kQ65Ints] = {
        (jint)p.max_cand, (jint)p.pileup, (jint)p.eme_delay, (jint)p.max_drift,
        (jint)p.fading_model, (jint)p.ap_list, (jint)p.has_ap_hint,
    };
    (*env)->SetFloatArrayRegion(env, fa, 0, kQ65Floats, f);
    (*env)->SetIntArrayRegion(env, ia, 0, kQ65Ints, v);
}

/// Decode one Q65 slot of 12 kHz-or-other f32 audio; `callers` is 0 or a
/// `MfskQ65Callers` handle. Returns an `Object[]` of `MfskDecode`.
JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_Mfsk_nativeDecodeQ65(
        JNIEnv* env, jclass cls, jint mode, jfloatArray samples, jint sampleRate,
        jfloatArray fa, jintArray ia, jobjectArray sa, jlong callers) {
    (void)cls;
    MfskQ65Params params;
    const MfskQ65Params* pp = NULL;
    if (fa != NULL) {
        if (!read_q65_params(env, fa, ia, sa, &params)) return NULL;
        pp = &params;
    }
    const jsize n = (*env)->GetArrayLength(env, samples);
    jfloat* pcm = (*env)->GetFloatArrayElements(env, samples, NULL);
    if (pcm == NULL) return NULL;

    enum { kCap = 64 };
    MfskDecode rows[kCap];
    memset(rows, 0, sizeof rows);
    for (int i = 0; i < kCap; ++i) rows[i].size = sizeof rows[i];
    size_t len = 0;
    const MfskStatus st = mfsk_q65_decode_ex(
        (uint32_t)mode, (const float*)pcm, (size_t)n, (uint32_t)sampleRate, pp,
        (const MfskQ65Callers*)(intptr_t)callers, NULL, rows, kCap, &len);
    (*env)->ReleaseFloatArrayElements(env, samples, pcm, JNI_ABORT);
    if (st != MFSK_STATUS_OK) {
        if (len > kCap) {
            throw_ise(env, "more decodes than this shim's row buffer holds");
        } else {
            throw_ise(env, mfsk_last_error());
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

/// `mfsk_encode_q65_flagged` for a `MfskMode` Q65 sub-mode, returning f32 PCM
/// at 12 kHz. The C entry point takes `MfskQ65SubMode`, which has its own
/// numbering, so the bridge is here.
JNIEXPORT jfloatArray JNICALL
Java_io_github_mfskcore_Mfsk_nativeSynthesizeQ65(
        JNIEnv* env, jclass cls, jint mode, jstring a, jstring b, jstring c,
        jboolean flagged, jfloat freqHz) {
    (void)cls;
    uint32_t sub;
    switch ((uint32_t)mode) {
        case MFSK_MODE_Q65A15:  sub = MFSK_Q65_SUB_MODE_A15;  break;
        case MFSK_MODE_Q65A30:  sub = MFSK_Q65_SUB_MODE_A30;  break;
        case MFSK_MODE_Q65A60:  sub = MFSK_Q65_SUB_MODE_A60;  break;
        case MFSK_MODE_Q65B60:  sub = MFSK_Q65_SUB_MODE_B60;  break;
        case MFSK_MODE_Q65C60:  sub = MFSK_Q65_SUB_MODE_C60;  break;
        case MFSK_MODE_Q65D60:  sub = MFSK_Q65_SUB_MODE_D60;  break;
        case MFSK_MODE_Q65E60:  sub = MFSK_Q65_SUB_MODE_E60;  break;
        case MFSK_MODE_Q65D120: sub = MFSK_Q65_SUB_MODE_D120; break;
        case MFSK_MODE_Q65E120: sub = MFSK_Q65_SUB_MODE_E120; break;
        case MFSK_MODE_Q65A300: sub = MFSK_Q65_SUB_MODE_A300; break;
        default:
            throw_ise(env, "not a Q65 mode");
            return NULL;
    }
    const char* sa = (*env)->GetStringUTFChars(env, a, NULL);
    const char* sb = (*env)->GetStringUTFChars(env, b, NULL);
    const char* sc = (*env)->GetStringUTFChars(env, c, NULL);
    jfloatArray out = NULL;
    if (sa && sb && sc) {
        size_t need = 0;
        /* A zero-capacity call reports the size it needs. */
        (void)mfsk_encode_q65_flagged(sub, sa, sb, sc, flagged ? 1u : 0u, freqHz, NULL, 0, &need);
        float* buf = (float*)malloc(need * sizeof(float));
        if (buf == NULL) {
            throw_ise(env, "out of memory");
        } else {
            size_t got = 0;
            if (mfsk_encode_q65_flagged(sub, sa, sb, sc, flagged ? 1u : 0u, freqHz,
                                        buf, need, &got) != MFSK_STATUS_OK) {
                throw_ise(env, mfsk_last_error());
            } else {
                out = (*env)->NewFloatArray(env, (jsize)got);
                if (out != NULL) (*env)->SetFloatArrayRegion(env, out, 0, (jsize)got, buf);
            }
            free(buf);
        }
    } else {
        throw_ise(env, "could not read the message strings");
    }
    if (sa) (*env)->ReleaseStringUTFChars(env, a, sa);
    if (sb) (*env)->ReleaseStringUTFChars(env, b, sb);
    if (sc) (*env)->ReleaseStringUTFChars(env, c, sc);
    return out;
}

// ── Q65History ──

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskQ65History_nativeNew(JNIEnv* env, jclass cls) {
    (void)env; (void)cls;
    return (jlong)(intptr_t)mfsk_q65_history_new();
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskQ65History_nativeFree(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    mfsk_q65_history_free((MfskQ65History*)(intptr_t)h);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskQ65History_nativePush(
        JNIEnv* env, jclass cls, jlong h, jfloat freqHz, jstring message) {
    (void)cls;
    const char* m = (*env)->GetStringUTFChars(env, message, NULL);
    if (m == NULL) return;
    const MfskStatus st = mfsk_q65_history_push((MfskQ65History*)(intptr_t)h, freqHz, m);
    (*env)->ReleaseStringUTFChars(env, message, m);
    if (st != MFSK_STATUS_OK) throw_ise(env, mfsk_last_error());
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskQ65History_nativeLen(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return (jint)mfsk_q65_history_len((const MfskQ65History*)(intptr_t)h);
}

/// The DX station, as `String[2]` (call, grid or null), or null when nothing
/// qualifies.
JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskQ65History_nativeLookup(
        JNIEnv* env, jclass cls, jlong h, jfloat rxFreqHz) {
    (void)cls;
    MfskQ65Dx dx;
    memset(&dx, 0, sizeof dx);
    dx.size = sizeof dx;
    const MfskStatus st = mfsk_q65_history_lookup(
        (const MfskQ65History*)(intptr_t)h, rxFreqHz, &dx);
    if (st == MFSK_STATUS_DECODE_FAILED) return NULL;
    if (st != MFSK_STATUS_OK) { throw_ise(env, mfsk_last_error()); return NULL; }
    jclass strCls = (*env)->FindClass(env, "java/lang/String");
    if (strCls == NULL) return NULL;
    jobjectArray out = (*env)->NewObjectArray(env, 2, strCls, NULL);
    if (out == NULL) return NULL;
    jstring call = (*env)->NewStringUTF(env, dx.call);
    if (call == NULL) return NULL;
    (*env)->SetObjectArrayElement(env, out, 0, call);
    (*env)->DeleteLocalRef(env, call);
    if (dx.has_grid) {
        jstring grid = (*env)->NewStringUTF(env, dx.grid);
        if (grid == NULL) return NULL;
        (*env)->SetObjectArrayElement(env, out, 1, grid);
        (*env)->DeleteLocalRef(env, grid);
    }
    return out;
}

// ── Q65Callers ──

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskQ65Callers_nativeNew(JNIEnv* env, jclass cls) {
    (void)env; (void)cls;
    return (jlong)(intptr_t)mfsk_q65_callers_new();
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskQ65Callers_nativeFree(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    mfsk_q65_callers_free((MfskQ65Callers*)(intptr_t)h);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskQ65Callers_nativeRecord(
        JNIEnv* env, jclass cls, jlong h, jfloat freqHz, jstring message, jlong now) {
    (void)cls;
    const char* m = (*env)->GetStringUTFChars(env, message, NULL);
    if (m == NULL) return;
    const MfskStatus st = mfsk_q65_callers_record(
        (MfskQ65Callers*)(intptr_t)h, freqHz, m, (uint64_t)now);
    (*env)->ReleaseStringUTFChars(env, message, m);
    if (st != MFSK_STATUS_OK) throw_ise(env, mfsk_last_error());
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskQ65Callers_nativeExpire(JNIEnv* env, jclass cls, jlong h, jlong now) {
    (void)cls;
    if (mfsk_q65_callers_expire((MfskQ65Callers*)(intptr_t)h, (uint64_t)now) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
    }
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskQ65Callers_nativeRemove(JNIEnv* env, jclass cls, jlong h, jstring call) {
    (void)cls;
    const char* c = (*env)->GetStringUTFChars(env, call, NULL);
    if (c == NULL) return;
    const MfskStatus st = mfsk_q65_callers_remove((MfskQ65Callers*)(intptr_t)h, c);
    (*env)->ReleaseStringUTFChars(env, call, c);
    if (st != MFSK_STATUS_OK) throw_ise(env, mfsk_last_error());
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskQ65Callers_nativeLen(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return (jint)mfsk_q65_callers_len((const MfskQ65Callers*)(intptr_t)h);
}

/// One caller as `String[4]` — call, grid, last-heard (decimal Unix seconds),
/// frequency (decimal Hz) — since JNI has no cheap way to return a mixed
/// record and this is not a hot path. Null past the end.
JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskQ65Callers_nativeGet(
        JNIEnv* env, jclass cls, jlong h, jint index) {
    (void)cls;
    if (index < 0) return NULL;
    MfskQ65Caller c;
    memset(&c, 0, sizeof c);
    c.size = sizeof c;
    if (mfsk_q65_callers_get((const MfskQ65Callers*)(intptr_t)h, (size_t)index, &c)
        != MFSK_STATUS_OK) {
        return NULL;
    }
    char heard[32], freq[32];
    snprintf(heard, sizeof heard, "%llu", (unsigned long long)c.last_heard);
    snprintf(freq, sizeof freq, "%d", (int)c.freq_hz);
    const char* vals[4] = { c.call, c.grid, heard, freq };
    jclass strCls = (*env)->FindClass(env, "java/lang/String");
    if (strCls == NULL) return NULL;
    jobjectArray out = (*env)->NewObjectArray(env, 4, strCls, NULL);
    if (out == NULL) return NULL;
    for (int i = 0; i < 4; ++i) {
        jstring js = (*env)->NewStringUTF(env, vals[i]);
        if (js == NULL) return NULL;
        (*env)->SetObjectArrayElement(env, out, i, js);
        (*env)->DeleteLocalRef(env, js);
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

// ── JTTY receiver ───────────────────────────────────────────────────

static void jtty_fill_params(MfskJttyParams* p, jfloat f0, jfloat ftol, jfloat smin,
                             jfloat nfa, jfloat nfb, jboolean subtract) {
    mfsk_jtty_params_init(p);
    p->f0_hz = f0;
    p->ftol_hz = ftol;
    p->smin_db = smin;
    p->nfa_hz = nfa;
    p->nfb_hz = nfb;
    p->subtract = (subtract == JNI_TRUE) ? 1u : 0u;
}

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskJttyReceiver_nativeOpen(
        JNIEnv* env, jclass cls, jint sampleRate, jfloat f0, jfloat ftol, jfloat smin,
        jfloat nfa, jfloat nfb, jboolean subtract) {
    (void)cls;
    MfskJttyParams p;
    jtty_fill_params(&p, f0, ftol, smin, nfa, nfb, subtract);
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskJttyReceiver* rx = mfsk_jtty_open((uint32_t)sampleRate, &p, &st);
    if (rx == NULL) {
        throw_ise(env, mfsk_last_error());
        return 0;
    }
    return (jlong)(intptr_t)rx;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskJttyReceiver_nativeClose(JNIEnv* env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    mfsk_jtty_close((MfskJttyReceiver*)(intptr_t)handle);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskJttyReceiver_nativeSetParams(
        JNIEnv* env, jclass cls, jlong handle, jfloat f0, jfloat ftol, jfloat smin,
        jfloat nfa, jfloat nfb, jboolean subtract) {
    (void)cls;
    MfskJttyParams p;
    jtty_fill_params(&p, f0, ftol, smin, nfa, nfb, subtract);
    if (mfsk_jtty_set_params((MfskJttyReceiver*)(intptr_t)handle, &p) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
    }
}

/// Everything waiting in the queue as an `MfskJttyUpdate[]`. The rows live
/// in this frame; nothing here is owned by the library.
static jobjectArray jtty_drain(JNIEnv* env, MfskJttyReceiver* rx) {
    jclass cls = (*env)->FindClass(env, CLS "MfskJttyUpdate");
    if (cls == NULL) return NULL;
    jmethodID ctor = (*env)->GetMethodID(env, cls, "<init>", "(JLjava/lang/String;ZFF)V");
    if (ctor == NULL) return NULL;

    const size_t n = mfsk_jtty_pending(rx);
    jobjectArray out = (*env)->NewObjectArray(env, (jsize)n, cls, NULL);
    if (out == NULL) return NULL;
    for (size_t i = 0; i < n; ++i) {
        MfskJttyUpdate u;
        memset(&u, 0, sizeof u);
        u.size = sizeof u;
        if (mfsk_jtty_poll(rx, &u) != 1) break;
        jstring text = (*env)->NewStringUTF(env, u.text);
        if (text == NULL) return NULL;
        jobject obj = (*env)->NewObject(env, cls, ctor, (jlong)u.id, text,
                                        u.complete ? JNI_TRUE : JNI_FALSE,
                                        (jfloat)u.f1_hz, (jfloat)u.start_s);
        (*env)->DeleteLocalRef(env, text);
        if (obj == NULL) return NULL;
        (*env)->SetObjectArrayElement(env, out, (jsize)i, obj);
        (*env)->DeleteLocalRef(env, obj);
    }
    return out;
}

JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskJttyReceiver_nativePush(
        JNIEnv* env, jclass cls, jlong handle, jshortArray samples) {
    (void)cls;
    MfskJttyReceiver* rx = (MfskJttyReceiver*)(intptr_t)handle;
    if (rx == NULL) { throw_ise(env, "receiver is closed"); return NULL; }
    const jsize n = (*env)->GetArrayLength(env, samples);
    jshort* pcm = (*env)->GetShortArrayElements(env, samples, NULL);
    if (pcm == NULL) return NULL;
    const MfskStatus st = mfsk_jtty_push_i16(rx, (const int16_t*)pcm, (size_t)n);
    (*env)->ReleaseShortArrayElements(env, samples, pcm, JNI_ABORT);
    if (st != MFSK_STATUS_OK) { throw_ise(env, mfsk_last_error()); return NULL; }
    return jtty_drain(env, rx);
}

JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskJttyReceiver_nativePoll(JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    MfskJttyReceiver* rx = (MfskJttyReceiver*)(intptr_t)handle;
    if (rx == NULL) { throw_ise(env, "receiver is closed"); return NULL; }
    return jtty_drain(env, rx);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskJttyReceiver_nativeFinish(JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    if (mfsk_jtty_finish((MfskJttyReceiver*)(intptr_t)handle) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
    }
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskJttyReceiver_nativeReset(JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    if (mfsk_jtty_reset((MfskJttyReceiver*)(intptr_t)handle) != MFSK_STATUS_OK) {
        throw_ise(env, mfsk_last_error());
    }
}

// ── JTTY transmit ───────────────────────────────────────────────────

JNIEXPORT jbyteArray JNICALL
Java_io_github_mfskcore_MfskJtty_nativeEncodeTones(
        JNIEnv* env, jclass cls, jstring text, jint profile) {
    (void)cls;
    const char* s = (*env)->GetStringUTFChars(env, text, NULL);
    if (s == NULL) return NULL;
    size_t n = 0;
    MfskStatus st = mfsk_jtty_encode_tones(s, (uint32_t)profile, NULL, 0, &n);
    uint8_t* tones = NULL;
    if (st == MFSK_STATUS_OK && n > 0) {
        tones = (uint8_t*)malloc(n);
        if (tones == NULL) {
            (*env)->ReleaseStringUTFChars(env, text, s);
            throw_ise(env, "out of memory");
            return NULL;
        }
        st = mfsk_jtty_encode_tones(s, (uint32_t)profile, tones, n, &n);
    }
    (*env)->ReleaseStringUTFChars(env, text, s);
    if (st != MFSK_STATUS_OK) {
        free(tones);
        throw_ise(env, mfsk_last_error());
        return NULL;
    }
    jbyteArray out = (*env)->NewByteArray(env, (jsize)n);
    if (out != NULL && n > 0) (*env)->SetByteArrayRegion(env, out, 0, (jsize)n, (const jbyte*)tones);
    free(tones);
    return out;
}

JNIEXPORT jshortArray JNICALL
Java_io_github_mfskcore_MfskJtty_nativeTonesToPcm(
        JNIEnv* env, jclass cls, jbyteArray tones, jfloat freqHz, jfloat amplitude) {
    (void)cls;
    const jsize n = (*env)->GetArrayLength(env, tones);
    jbyte* t = (*env)->GetByteArrayElements(env, tones, NULL);
    if (t == NULL) return NULL;
    size_t need = 0;
    MfskStatus st = mfsk_jtty_tones_to_i16((const uint8_t*)t, (size_t)n, freqHz, amplitude, NULL, 0, &need);
    jshortArray out = NULL;
    if (st == MFSK_STATUS_OK) {
        out = (*env)->NewShortArray(env, (jsize)need);
        jshort* dst = (out != NULL) ? (*env)->GetShortArrayElements(env, out, NULL) : NULL;
        if (dst != NULL) {
            size_t wrote = 0;
            st = mfsk_jtty_tones_to_i16((const uint8_t*)t, (size_t)n, freqHz, amplitude,
                                        (int16_t*)dst, need, &wrote);
            (*env)->ReleaseShortArrayElements(env, out, dst, 0);
        } else {
            (*env)->ReleaseByteArrayElements(env, tones, t, JNI_ABORT);
            return NULL;
        }
    }
    (*env)->ReleaseByteArrayElements(env, tones, t, JNI_ABORT);
    if (st != MFSK_STATUS_OK) { throw_ise(env, mfsk_last_error()); return NULL; }
    return out;
}
