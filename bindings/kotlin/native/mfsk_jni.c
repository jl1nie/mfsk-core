// SPDX-License-Identifier: GPL-3.0-only
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

/// Throw the Kotlin exception for an `MfskStatus`: the typed ones for the
/// statuses a caller can act on (a refused option, a bad value, a mode this
/// build lacks), `MfskException` for the rest. All take `(String, int)`.
static void throw_status(JNIEnv* env, int status, const char* msg) {
    const char* name = CLS "MfskException";
    if (status == MFSK_STATUS_UNSUPPORTED) name = CLS "MfskUnsupportedException";
    else if (status == MFSK_STATUS_INVALID_ARG) name = CLS "MfskInvalidArgException";
    else if (status == MFSK_STATUS_UNKNOWN_PROTOCOL) name = CLS "MfskUnknownModeException";
    jclass c = (*env)->FindClass(env, name);
    if (c == NULL) return;  /* NoClassDefFoundError is pending */
    jmethodID ctor = (*env)->GetMethodID(env, c, "<init>", "(Ljava/lang/String;I)V");
    if (ctor == NULL) return;
    jstring text = (*env)->NewStringUTF(env, msg != NULL ? msg : "mfsk call failed");
    if (text == NULL) return;
    jobject ex = (*env)->NewObject(env, c, ctor, text, (jint)status);
    if (ex != NULL) (*env)->Throw(env, (jthrowable)ex);
}

/// Report the ABI's own error text, which is more specific than a status
/// code. A decoder carries its own slot precisely so a coroutine that hopped
/// threads still sees it; the thread-local one is the fallback (and the only
/// one there is before a handle exists).
static void throw_from_decoder(JNIEnv* env, MfskDecoder* d, int status, const char* fallback) {
    const char* detail = (d != NULL) ? mfsk_decoder_last_error(d) : NULL;
    if (detail == NULL) detail = mfsk_last_error();
    throw_status(env, status, detail != NULL ? detail : fallback);
}

static void throw_last(JNIEnv* env, int status, const char* fallback) {
    const char* detail = mfsk_last_error();
    throw_status(env, status, detail != NULL ? detail : fallback);
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
    return (*env)->GetMethodID(
        env, rowCls, "<init>",
        "(ILjava/lang/String;FFFLjava/lang/Float;Ljava/lang/Float;Ljava/lang/Integer;"
        "IIZZLjava/lang/String;ILjava/lang/Integer;L" CLS "MfskStage;)V");
}

/// `MfskStage?` from `MfskDecode::stage`: null for `MFSK_STAGE_NONE`.
static jobject stage_of(JNIEnv* env, uint8_t stage) {
    const char* name = stage == MFSK_STAGE_EARLY ? "EARLY"
                     : stage == MFSK_STAGE_FINAL ? "FINAL" : NULL;
    if (name == NULL) return NULL;
    jclass c = (*env)->FindClass(env, CLS "MfskStage");
    if (c == NULL) return NULL;
    jfieldID f = (*env)->GetStaticFieldID(env, c, name, "L" CLS "MfskStage;");
    jobject o = f == NULL ? NULL : (*env)->GetStaticObjectField(env, c, f);
    (*env)->DeleteLocalRef(env, c);
    return o;
}

/// A `Float?`: the boxed value when `has`, null otherwise. A number the
/// mode does not report is null on the Kotlin side, as it is `None` in Rust
/// and a clear `MFSK_DECODE_FLAG_HAS_*` bit in C (#594).
static jobject box_float(JNIEnv* env, bool has, float v) {
    if (!has) return NULL;
    jclass c = (*env)->FindClass(env, "java/lang/Float");
    if (c == NULL) return NULL;
    jmethodID m = (*env)->GetStaticMethodID(env, c, "valueOf", "(F)Ljava/lang/Float;");
    jobject o = m == NULL ? NULL : (*env)->CallStaticObjectMethod(env, c, m, (jfloat)v);
    (*env)->DeleteLocalRef(env, c);
    return o;
}

/// An `Int?`, as `box_float`.
static jobject box_int(JNIEnv* env, bool has, int32_t v) {
    if (!has) return NULL;
    jclass c = (*env)->FindClass(env, "java/lang/Integer");
    if (c == NULL) return NULL;
    jmethodID m = (*env)->GetStaticMethodID(env, c, "valueOf", "(I)Ljava/lang/Integer;");
    jobject o = m == NULL ? NULL : (*env)->CallStaticObjectMethod(env, c, m, (jint)v);
    (*env)->DeleteLocalRef(env, c);
    return o;
}

/// The packed message key as lower-case hex, `ceil(key_bits / 8)` bytes:
/// compared by value, unlike a `ByteArray` in a data class.
static jstring key_hex(JNIEnv* env, uint8_t key_bits, const uint8_t* key) {
    char buf[2 * MFSK_DECODE_KEY_LEN + 1];
    static const char hex[] = "0123456789abcdef";
    size_t n = ((size_t)key_bits + 7) / 8;
    if (n > MFSK_DECODE_KEY_LEN) n = MFSK_DECODE_KEY_LEN;
    for (size_t i = 0; i < n; ++i) {
        buf[2 * i] = hex[key[i] >> 4];
        buf[2 * i + 1] = hex[key[i] & 15];
    }
    buf[2 * n] = 0;
    return (*env)->NewStringUTF(env, buf);
}

/// One C row as one Kotlin `MfskDecode`. Every field is copied — the
/// row pointer a callback receives is valid only for that call.
static jobject make_row(JNIEnv* env, jclass rowCls, jmethodID ctor, const MfskDecode* r) {
    jstring text = (*env)->NewStringUTF(env, r->text);
    if (text == NULL) return NULL;
    jstring key = key_hex(env, r->key_bits, r->key);
    if (key == NULL) return NULL;
    jobject sync = box_float(env, (r->flags & MFSK_DECODE_FLAG_HAS_SYNC_SCORE) != 0, r->sync_score);
    jobject cv = box_float(env, (r->flags & MFSK_DECODE_FLAG_HAS_SYNC_CV) != 0, r->sync_cv);
    jobject hard = box_int(env, (r->flags & MFSK_DECODE_FLAG_HAS_HARD_ERRORS) != 0,
                           (int32_t)r->hard_errors);
    jobject delivery = box_int(env, r->delivery >= 0, r->delivery);
    jobject stage = stage_of(env, r->stage);
    if ((*env)->ExceptionCheck(env)) return NULL;
    jobject obj = (*env)->NewObject(
        env, rowCls, ctor,
        (jint)r->mode, text,
        (jfloat)r->freq_hz, (jfloat)r->dt_sec, (jfloat)r->snr_db,
        sync, cv, hard,
        (jint)r->info_bits, (jint)r->pass,
        (jboolean)((r->flags & MFSK_DECODE_FLAG_HASH_RESOLVED) != 0),
        (jboolean)((r->flags & MFSK_DECODE_FLAG_COPIED_LAST_TX) != 0),
        key, (jint)r->key_bits, delivery, stage);
    (*env)->DeleteLocalRef(env, text);
    if (stage) (*env)->DeleteLocalRef(env, stage);
    (*env)->DeleteLocalRef(env, key);
    if (sync) (*env)->DeleteLocalRef(env, sync);
    if (cv) (*env)->DeleteLocalRef(env, cv);
    if (hard) (*env)->DeleteLocalRef(env, hard);
    if (delivery) (*env)->DeleteLocalRef(env, delivery);
    return obj;
}

// ── Decode callback ─────────────────────────────────────────────────
//
// `mfsk_decoder_set_on_decode` delivers rows as they are found, on top
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
    if ((*env)->PushLocalFrame(env, 16) != 0) return;
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

// ── Parameter block and options ─────────────────────────────────────

/// `MfskParams` and `MfskExtras` cross JNI as three flat arrays each rather
/// than as objects the shim reads field by field, so there is no per-field
/// `GetFieldID` to get wrong and a field added later is one more slot in each
/// layout below and a line in each of `read_*` / `write_*`. The order is the
/// contract with `MfskParams.floatArray` / `intArray` / `stringArray` and
/// `MfskExtras.*` in Mfsk.kt, and every array's length is checked here: a
/// short array read as a longer layout would be plausible garbage, not an
/// error.
///
///   params floats:  band_lo_hz, band_hi_hz, rx_freq_hz, tol_hz, tx_freq_hz
///                   (NaN is unset)
///   params ints:    depth, flags, ap_mode, contest, qso_progress
///   params strings: mycall, mygrid, hiscall, hisgrid (null is empty)
///
///   extras floats:  sync_min, sniper_hz, nb_ftol_hz, t_early_s, t_late_s,
///                   score_threshold, fading_b90_ts
///   extras ints:    max_cand, osd, strictness, strategy, sic_rounds, eq_mode,
///                   message_filter, a7, has_ap_hint, nb_percent,
///                   nb_sweep_step, max_cycles_per_bit, chase_trials, pileup,
///                   max_drift, fading_model
///   extras strings: ap_call1, ap_call2, ap_grid, ap_report (null is empty)
enum {
    kPF = 5, kPI = 5, kPS = 4,
    kEF = 7, kEI = 16, kES = 4,
};

static bool shape_ok(JNIEnv* env, jfloatArray fa, jintArray ia, jobjectArray sa,
                     int nf, int ni, int ns, const char* what) {
    if (fa == NULL || ia == NULL || sa == NULL
        || (*env)->GetArrayLength(env, fa) != nf
        || (*env)->GetArrayLength(env, ia) != ni
        || (*env)->GetArrayLength(env, sa) != ns) {
        char msg[96];
        snprintf(msg, sizeof msg, "%s arrays have the wrong shape", what);
        throw_ise(env, msg);
        return false;
    }
    return true;
}

/// Copy the Java strings into the struct's fixed fields. A string that does
/// not fit with its NUL is refused rather than truncated: a callsign cut short
/// is a different callsign.
static bool read_strings(JNIEnv* env, jobjectArray sa, int n, char* const dst[],
                         const size_t cap[], const char* const name[]) {
    for (int i = 0; i < n; ++i) {
        jstring js = (jstring)(*env)->GetObjectArrayElement(env, sa, i);
        if ((*env)->ExceptionCheck(env)) return false;
        if (js == NULL) continue;
        const char* u = (*env)->GetStringUTFChars(env, js, NULL);
        if (u == NULL) { (*env)->DeleteLocalRef(env, js); return false; }
        const size_t len = strlen(u);
        const bool fits = len < cap[i];
        if (fits) memcpy(dst[i], u, len + 1);
        (*env)->ReleaseStringUTFChars(env, js, u);
        (*env)->DeleteLocalRef(env, js);
        if (!fits) {
            char msg[96];
            snprintf(msg, sizeof msg, "%s is longer than %u bytes", name[i], (unsigned)(cap[i] - 1));
            throw_status(env, MFSK_STATUS_INVALID_ARG, msg);
            return false;
        }
    }
    return true;
}

static bool write_strings(JNIEnv* env, jobjectArray sa, int n, const char* const src[]) {
    for (int i = 0; i < n; ++i) {
        jstring js = (*env)->NewStringUTF(env, src[i]);
        if (js == NULL) return false;
        (*env)->SetObjectArrayElement(env, sa, i, js);
        (*env)->DeleteLocalRef(env, js);
    }
    return true;
}

static bool read_params(JNIEnv* env, jfloatArray fa, jintArray ia, jobjectArray sa,
                        MfskParams* p) {
    if (!shape_ok(env, fa, ia, sa, kPF, kPI, kPS, "parameter")) return false;
    jfloat f[kPF];
    jint v[kPI];
    (*env)->GetFloatArrayRegion(env, fa, 0, kPF, f);
    (*env)->GetIntArrayRegion(env, ia, 0, kPI, v);

    memset(p, 0, sizeof *p);
    p->size = sizeof *p;
    p->band_lo_hz = f[0];
    p->band_hi_hz = f[1];
    p->rx_freq_hz = f[2];
    p->tol_hz = f[3];
    p->tx_freq_hz = f[4];
    /* Plain integers: the library checks every value before using it. */
    p->depth = (uint32_t)v[0];
    p->flags = (uint32_t)v[1];
    p->ap_mode = (uint32_t)v[2];
    p->contest = (uint32_t)v[3];
    p->qso_progress = (uint32_t)v[4];

    char* const dst[kPS] = { p->mycall, p->mygrid, p->hiscall, p->hisgrid };
    const size_t cap[kPS] = { sizeof p->mycall, sizeof p->mygrid,
                              sizeof p->hiscall, sizeof p->hisgrid };
    const char* const name[kPS] = { "station call", "station grid", "his call", "his grid" };
    return read_strings(env, sa, kPS, dst, cap, name);
}

static bool write_params(JNIEnv* env, const MfskParams* p,
                         jfloatArray fa, jintArray ia, jobjectArray sa) {
    const jfloat f[kPF] = { p->band_lo_hz, p->band_hi_hz, p->rx_freq_hz, p->tol_hz,
                            p->tx_freq_hz };
    const jint v[kPI] = { (jint)p->depth, (jint)p->flags, (jint)p->ap_mode,
                          (jint)p->contest, (jint)p->qso_progress };
    (*env)->SetFloatArrayRegion(env, fa, 0, kPF, f);
    (*env)->SetIntArrayRegion(env, ia, 0, kPI, v);
    const char* const s[kPS] = { p->mycall, p->mygrid, p->hiscall, p->hisgrid };
    return write_strings(env, sa, kPS, s);
}

static bool read_extras(JNIEnv* env, jfloatArray fa, jintArray ia, jobjectArray sa,
                        MfskExtras* e) {
    if (!shape_ok(env, fa, ia, sa, kEF, kEI, kES, "extras")) return false;
    jfloat f[kEF];
    jint v[kEI];
    (*env)->GetFloatArrayRegion(env, fa, 0, kEF, f);
    (*env)->GetIntArrayRegion(env, ia, 0, kEI, v);

    memset(e, 0, sizeof *e);
    e->size = sizeof *e;
    e->sync_min = f[0];
    e->sniper_hz = f[1];
    e->nb_ftol_hz = f[2];
    e->t_early_s = f[3];
    e->t_late_s = f[4];
    e->score_threshold = f[5];
    e->fading_b90_ts = f[6];
    e->max_cand = (uint32_t)v[0];
    e->osd = v[1];
    e->strictness = v[2];
    e->strategy = (uint32_t)v[3];
    e->sic_rounds = (uint32_t)v[4];
    e->eq_mode = (uint32_t)v[5];
    e->message_filter = (uint32_t)v[6];
    e->a7 = (uint32_t)v[7];
    e->has_ap_hint = (uint32_t)v[8];
    e->nb_percent = (uint32_t)v[9];
    e->nb_sweep_step = (uint32_t)v[10];
    e->max_cycles_per_bit = (uint32_t)v[11];
    e->chase_trials = (uint32_t)v[12];
    e->pileup = (uint32_t)v[13];
    e->max_drift = (uint32_t)v[14];
    e->fading_model = (uint32_t)v[15];

    char* const dst[kES] = { e->ap_call1, e->ap_call2, e->ap_grid, e->ap_report };
    const size_t cap[kES] = { sizeof e->ap_call1, sizeof e->ap_call2,
                              sizeof e->ap_grid, sizeof e->ap_report };
    const char* const name[kES] = { "AP call1", "AP call2", "AP grid", "AP report" };
    return read_strings(env, sa, kES, dst, cap, name);
}

static bool write_extras(JNIEnv* env, const MfskExtras* e,
                         jfloatArray fa, jintArray ia, jobjectArray sa) {
    const jfloat f[kEF] = { e->sync_min, e->sniper_hz, e->nb_ftol_hz, e->t_early_s,
                            e->t_late_s, e->score_threshold, e->fading_b90_ts };
    const jint v[kEI] = {
        (jint)e->max_cand, (jint)e->osd, (jint)e->strictness, (jint)e->strategy,
        (jint)e->sic_rounds, (jint)e->eq_mode, (jint)e->message_filter, (jint)e->a7,
        (jint)e->has_ap_hint, (jint)e->nb_percent, (jint)e->nb_sweep_step,
        (jint)e->max_cycles_per_bit, (jint)e->chase_trials, (jint)e->pileup,
        (jint)e->max_drift, (jint)e->fading_model,
    };
    (*env)->SetFloatArrayRegion(env, fa, 0, kEF, f);
    (*env)->SetIntArrayRegion(env, ia, 0, kEI, v);
    const char* const s[kES] = { e->ap_call1, e->ap_call2, e->ap_grid, e->ap_report };
    return write_strings(env, sa, kES, s);
}

/// `mfsk_params_init` for `mode`, into the caller's arrays.
JNIEXPORT void JNICALL
Java_io_github_mfskcore_Mfsk_nativeParamsInit(
        JNIEnv* env, jclass cls, jint mode, jfloatArray fa, jintArray ia, jobjectArray sa) {
    (void)cls;
    if (!shape_ok(env, fa, ia, sa, kPF, kPI, kPS, "parameter")) return;
    MfskParams p;
    memset(&p, 0, sizeof p);
    p.size = sizeof p;
    const MfskStatus st = mfsk_params_init((uint32_t)mode, &p);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "mfsk_params_init failed"); return; }
    write_params(env, &p, fa, ia, sa);
}

/// `mfsk_extras_init`: every option unset.
JNIEXPORT void JNICALL
Java_io_github_mfskcore_Mfsk_nativeExtrasInit(
        JNIEnv* env, jclass cls, jfloatArray fa, jintArray ia, jobjectArray sa) {
    (void)cls;
    if (!shape_ok(env, fa, ia, sa, kEF, kEI, kES, "extras")) return;
    MfskExtras e;
    memset(&e, 0, sizeof e);
    e.size = sizeof e;
    const MfskStatus st = mfsk_extras_init(&e);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "mfsk_extras_init failed"); return; }
    write_extras(env, &e, fa, ia, sa);
}

// ── Decoder ─────────────────────────────────────────────────────────

/// Both structs optional: null arrays mean NULL, which is "the mode's
/// defaults" to the library. On false an exception is pending.
static bool read_optional(JNIEnv* env,
                          jfloatArray pf, jintArray pi, jobjectArray ps, MfskParams* p,
                          const MfskParams** pp,
                          jfloatArray ef, jintArray ei, jobjectArray es, MfskExtras* e,
                          const MfskExtras** ep) {
    *pp = NULL;
    *ep = NULL;
    if (pf != NULL) {
        if (!read_params(env, pf, pi, ps, p)) return false;
        *pp = p;
    }
    if (ef != NULL) {
        if (!read_extras(env, ef, ei, es, e)) return false;
        *ep = e;
    }
    return true;
}

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeOpen(
        JNIEnv* env, jclass cls, jint mode,
        jfloatArray pf, jintArray pi, jobjectArray ps,
        jfloatArray ef, jintArray ei, jobjectArray es) {
    (void)cls;
    MfskParams params;
    MfskExtras extras;
    const MfskParams* pp;
    const MfskExtras* ep;
    if (!read_optional(env, pf, pi, ps, &params, &pp, ef, ei, es, &extras, &ep)) return 0;
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecoder* d = mfsk_decoder_open((uint32_t)mode, pp, ep, &st);
    if (d == NULL) {
        throw_last(env, st, "mfsk_decoder_open failed");
        return 0;
    }
    return (jlong)(intptr_t)d;
}

/// The decoder's last error, or null.
JNIEXPORT jstring JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeLastError(JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    const char* e = mfsk_decoder_last_error((const MfskDecoder*)(intptr_t)handle);
    return e != NULL ? (*env)->NewStringUTF(env, e) : NULL;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeSetParams(
        JNIEnv* env, jclass cls, jlong handle,
        jfloatArray pf, jintArray pi, jobjectArray ps) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    MfskParams p;
    if (!read_params(env, pf, pi, ps, &p)) return;
    const MfskStatus st = mfsk_decoder_set_params(d, &p);
    if (st != MFSK_STATUS_OK) throw_from_decoder(env, d, st, "set_params failed");
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeSetExtras(
        JNIEnv* env, jclass cls, jlong handle,
        jfloatArray ef, jintArray ei, jobjectArray es) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    MfskExtras e;
    if (!read_extras(env, ef, ei, es, &e)) return;
    const MfskStatus st = mfsk_decoder_set_extras(d, &e);
    if (st != MFSK_STATUS_OK) throw_from_decoder(env, d, st, "set_extras failed");
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeSetQ65Callers(
        JNIEnv* env, jclass cls, jlong handle, jlong callers) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    const MfskStatus st = mfsk_decoder_set_q65_callers(d, (const MfskQ65Callers*)(intptr_t)callers);
    if (st != MFSK_STATUS_OK) throw_from_decoder(env, d, st, "set_q65_callers failed");
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeClear(JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    const MfskStatus st = mfsk_decoder_clear(d);
    if (st != MFSK_STATUS_OK) throw_from_decoder(env, d, st, "clear failed");
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeAddCallsign(
        JNIEnv* env, jclass cls, jlong handle, jstring call) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    const char* c = (*env)->GetStringUTFChars(env, call, NULL);
    if (c == NULL) return;
    const MfskStatus st = mfsk_decoder_add_callsign(d, c);
    (*env)->ReleaseStringUTFChars(env, call, c);
    if (st != MFSK_STATUS_OK) throw_from_decoder(env, d, st, "add_callsign failed");
}

/// Install (or clear, with a null `listener`) the decode callback.
///
/// Takes the previous context back and frees it, returning the new one,
/// so the Kotlin side owns exactly one `long` per decoder and cannot
/// leak a global ref by replacing a listener.
JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeSetOnDecode(
        JNIEnv* env, jclass cls, jlong handle, jobject listener, jlong oldCtx) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    if (d == NULL) { throw_ise(env, "decoder is closed"); return oldCtx; }

    if (listener == NULL) {
        mfsk_decoder_set_on_decode(d, NULL, NULL);
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

    const MfskStatus st = mfsk_decoder_set_on_decode(d, on_decode, ctx);
    if (st != MFSK_STATUS_OK) {
        callback_ctx_free(env, ctx);
        throw_from_decoder(env, d, st, "set_on_decode failed");
        return oldCtx;
    }
    /* Only now is the old one unreachable from the decode side. */
    callback_ctx_free(env, (CallbackCtx*)(intptr_t)oldCtx);
    return (jlong)(intptr_t)ctx;
}

/// Install (or clear) the budget predicate, taking the previous context
/// back and freeing it — the same ownership shape as the listener.
JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeSetBudget(
        JNIEnv* env, jclass cls, jlong handle, jobject check, jlong oldCtx) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    if (d == NULL) { throw_ise(env, "decoder is closed"); return oldCtx; }

    if (check == NULL) {
        mfsk_decoder_set_budget(d, NULL, NULL);
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

    const MfskStatus st = mfsk_decoder_set_budget(d, on_budget, ctx);
    if (st != MFSK_STATUS_OK) {
        budget_ctx_free(env, ctx);
        throw_from_decoder(env, d, st, "set_budget failed");
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
/// `mfsk_decoder_delivery_is_exact`.
JNIEXPORT jboolean JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeDeliveryIsExact(
        JNIEnv* env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    return mfsk_decoder_delivery_is_exact((const MfskDecoder*)(intptr_t)handle) ? JNI_TRUE
                                                                               : JNI_FALSE;
}

JNIEXPORT jintArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativePrefixPoints(
        JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    const MfskDecoder* d = (const MfskDecoder*)(intptr_t)handle;
    size_t pts[16];
    size_t n = 0;
    const MfskStatus st = mfsk_decoder_prefix_points(d, pts, 16, &n);
    if (st != MFSK_STATUS_OK) {
        throw_last(env, st, "prefix_points failed");
        return NULL;
    }
    jint v[16];
    for (size_t i = 0; i < n; ++i) v[i] = (jint)pts[i];
    jintArray out = (*env)->NewIntArray(env, (jsize)n);
    if (out != NULL && n > 0) (*env)->SetIntArrayRegion(env, out, 0, (jsize)n, v);
    return out;
}

JNIEXPORT jintArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeLastBudget(
        JNIEnv* env, jclass cls, jlong handle) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    MfskBudgetReport rep;
    memset(&rep, 0, sizeof rep);
    rep.size = sizeof rep;
    const MfskStatus st = mfsk_decoder_last_budget(d, &rep);
    if (st != MFSK_STATUS_OK) {
        throw_from_decoder(env, d, st, "last_budget failed");
        return NULL;
    }
    jint vals[6] = {
        (jint)(rep.exhausted ? 1 : 0),
        (jint)rep.candidates_skipped,
        (jint)rep.stages_run,
        (jint)rep.cut_at_sync,
        rep.cut_at_score == rep.cut_at_score /* not NaN */
            ? (jint)(rep.cut_at_score * 1000000.0f)
            : (jint)0x80000000,
        (jint)rep.rows_subtracted,
    };
    jintArray out = (*env)->NewIntArray(env, 6);
    if (out == NULL) return NULL;
    (*env)->SetIntArrayRegion(env, out, 0, 6, vals);
    return out;
}

/// Rows the shim can take from one decode. The ABI reports the count it
/// needed on a short buffer, but by then the decode has run, and a decode is
/// not repeatable (it moved the decoder's period-to-period state), so the
/// buffer is generous instead: a slot holds a few dozen rows at the busiest.
enum { kRowCap = 1024 };

static MfskDecode* rows_alloc(JNIEnv* env) {
    MfskDecode* rows = (MfskDecode*)calloc(kRowCap, sizeof *rows);
    if (rows == NULL) { throw_ise(env, "out of memory"); return NULL; }
    for (int i = 0; i < kRowCap; ++i) rows[i].size = sizeof rows[i];
    return rows;
}

/// `len` rows as an `MfskDecode[]`. Null with an exception pending on failure.
static jobjectArray rows_to_java(JNIEnv* env, const MfskDecode* rows, size_t len) {
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

/// Decode one period. `is_f32` picks which of `shorts` / `floats` is read.
static jobjectArray decode_common(JNIEnv* env, jlong handle, jshortArray shorts,
                                  jfloatArray floats, bool is_f32, jint sampleRate,
                                  jlong period, bool prefix) {
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    if (d == NULL) { throw_ise(env, "decoder is closed"); return NULL; }

    MfskDecode* rows = rows_alloc(env);
    if (rows == NULL) return NULL;

    size_t len = 0;
    MfskStatus st;
    if (is_f32) {
        const jsize n = (*env)->GetArrayLength(env, floats);
        jfloat* pcm = (*env)->GetFloatArrayElements(env, floats, NULL);
        if (pcm == NULL) { free(rows); return NULL; }
        st = (prefix ? mfsk_decoder_decode_prefix_f32 : mfsk_decoder_decode_f32)(
            d, (const float*)pcm, (size_t)n, (uint32_t)sampleRate, (int64_t)period, rows,
            kRowCap, &len);
        (*env)->ReleaseFloatArrayElements(env, floats, pcm, JNI_ABORT);
    } else {
        const jsize n = (*env)->GetArrayLength(env, shorts);
        jshort* pcm = (*env)->GetShortArrayElements(env, shorts, NULL);
        if (pcm == NULL) { free(rows); return NULL; }
        st = (prefix ? mfsk_decoder_decode_prefix_i16 : mfsk_decoder_decode_i16)(
            d, (const int16_t*)pcm, (size_t)n, (uint32_t)sampleRate, (int64_t)period, rows,
            kRowCap, &len);
        (*env)->ReleaseShortArrayElements(env, shorts, pcm, JNI_ABORT);
    }

    jobjectArray out = NULL;
    if (st != MFSK_STATUS_OK) {
        if (len > kRowCap) {
            throw_ise(env, "more decodes than this shim's row buffer holds");
        } else {
            throw_from_decoder(env, d, st, "decode failed");
        }
    } else {
        out = rows_to_java(env, rows, len);
    }
    free(rows);
    return out;
}

JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeDecodeI16(
        JNIEnv* env, jclass cls, jlong handle, jshortArray samples, jint sampleRate,
        jlong period) {
    (void)cls;
    return decode_common(env, handle, samples, NULL, false, sampleRate, period, false);
}

JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeDecodePrefixI16(
        JNIEnv* env, jclass cls, jlong handle, jshortArray samples, jint sampleRate,
        jlong period) {
    (void)cls;
    return decode_common(env, handle, samples, NULL, false, sampleRate, period, true);
}

JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeDecodePrefixF32(
        JNIEnv* env, jclass cls, jlong handle, jfloatArray samples, jint sampleRate,
        jlong period) {
    (void)cls;
    return decode_common(env, handle, NULL, samples, true, sampleRate, period, true);
}

JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeDecodeF32(
        JNIEnv* env, jclass cls, jlong handle, jfloatArray samples, jint sampleRate,
        jlong period) {
    (void)cls;
    return decode_common(env, handle, NULL, samples, true, sampleRate, period, false);
}

/// The FEC information bits of the `index`-th row of the last decode.
JNIEXPORT jbyteArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeCopyInfo(
        JNIEnv* env, jclass cls, jlong handle, jint index) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    uint8_t bits[256];
    size_t n = 0;
    const MfskStatus st = mfsk_decoder_copy_info(d, index < 0 ? (size_t)-1 : (size_t)index,
                                                 bits, sizeof bits, &n);
    if (st != MFSK_STATUS_OK) {
        throw_status(env, st, mfsk_last_error() != NULL ? mfsk_last_error() : "copy_info failed");
        return NULL;
    }
    jbyteArray out = (*env)->NewByteArray(env, (jsize)n);
    if (out != NULL && n > 0) (*env)->SetByteArrayRegion(env, out, 0, (jsize)n, (const jbyte*)bits);
    return out;
}

/// Decode the stream's ready slot. Null (no exception) when none is ready;
/// `meta[0]` receives the slot's period and `meta[1]` its UTC start in ns.
JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeDecodeStream(
        JNIEnv* env, jclass cls, jlong handle, jlong stream, jlongArray meta) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    if (d == NULL) { throw_ise(env, "decoder is closed"); return NULL; }
    MfskDecode* rows = rows_alloc(env);
    if (rows == NULL) return NULL;
    size_t len = 0;
    int64_t period = 0, utc = 0;
    const MfskStatus st = mfsk_decoder_decode_stream(
        d, (MfskStream*)(intptr_t)stream, rows, kRowCap, &len, &period, &utc);
    jobjectArray out = NULL;
    if (st == MFSK_STATUS_UNSUPPORTED && len == 0) {
        /* "No whole slot ready yet" is the one way this returns UNSUPPORTED. */
    } else if (st != MFSK_STATUS_OK) {
        if (len > kRowCap) {
            throw_ise(env, "more decodes than this shim's row buffer holds");
        } else {
            throw_from_decoder(env, d, st, "decode_stream failed");
        }
    } else {
        out = rows_to_java(env, rows, len);
        if (out != NULL) {
            const jlong m[2] = { (jlong)period, (jlong)utc };
            (*env)->SetLongArrayRegion(env, meta, 0, 2, m);
        }
    }
    free(rows);
    return out;
}

/// `mfsk_decoder_unpack77`, resolving `<...>` against the decoder's table.
JNIEXPORT jstring JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeUnpack77(
        JNIEnv* env, jclass cls, jlong handle, jbyteArray msg) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    if ((*env)->GetArrayLength(env, msg) != 77) {
        throw_status(env, MFSK_STATUS_INVALID_ARG, "a 77-bit message is 77 bytes");
        return NULL;
    }
    uint8_t bits[77];
    (*env)->GetByteArrayRegion(env, msg, 0, 77, (jbyte*)bits);
    char text[256];
    size_t n = 0;
    const MfskStatus st = mfsk_decoder_unpack77(d, bits, text, sizeof text, &n);
    if (st != MFSK_STATUS_OK) { throw_from_decoder(env, d, st, "unpack77 failed"); return NULL; }
    return (*env)->NewStringUTF(env, text);
}

/// Close the decoder and release the listener and the budget it carried.
///
/// Order matters: the decoder holds the raw `CallbackCtx*` and `BudgetCtx*`,
/// so it has to stop being able to call them before they are freed. Nothing
/// can be decoding here — `close()` and `decode()` are not safe to call
/// concurrently on one decoder in any case, which is what "single-threaded by
/// design" means.
JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskDecoder_nativeClose(
        JNIEnv* env, jclass cls, jlong handle, jlong ctx, jlong budgetCtx) {
    (void)cls;
    MfskDecoder* d = (MfskDecoder*)(intptr_t)handle;
    if (d != NULL) {
        mfsk_decoder_set_on_decode(d, NULL, NULL);
        mfsk_decoder_set_budget(d, NULL, NULL);
    }
    mfsk_decoder_close(d);
    callback_ctx_free(env, (CallbackCtx*)(intptr_t)ctx);
    budget_ctx_free(env, (BudgetCtx*)(intptr_t)budgetCtx);
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

static jbyteArray msg77_to_java(JNIEnv* env, const uint8_t* msg) {
    jbyteArray out = (*env)->NewByteArray(env, 77);
    if (out != NULL) (*env)->SetByteArrayRegion(env, out, 0, 77, (const jbyte*)msg);
    return out;
}

/// `mfsk_pack77`: `call1 call2 report` as 77 bytes, one bit each.
JNIEXPORT jbyteArray JNICALL
Java_io_github_mfskcore_Mfsk_nativePack77(
        JNIEnv* env, jclass cls, jstring a, jstring b, jstring c) {
    (void)cls;
    const char* sa = (*env)->GetStringUTFChars(env, a, NULL);
    const char* sb = (*env)->GetStringUTFChars(env, b, NULL);
    const char* sc = (*env)->GetStringUTFChars(env, c, NULL);
    uint8_t msg[77];
    const MfskStatus st = (sa && sb && sc) ? mfsk_pack77(sa, sb, sc, msg)
                                           : MFSK_STATUS_INVALID_ARG;
    if (sa) (*env)->ReleaseStringUTFChars(env, a, sa);
    if (sb) (*env)->ReleaseStringUTFChars(env, b, sb);
    if (sc) (*env)->ReleaseStringUTFChars(env, c, sc);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "pack77 failed"); return NULL; }
    return msg77_to_java(env, msg);
}

/// `mfsk_pack77_type4`: a non-standard call with a hashed standard one.
JNIEXPORT jbyteArray JNICALL
Java_io_github_mfskcore_Mfsk_nativePack77Type4(
        JNIEnv* env, jclass cls, jstring nonstd, jstring std, jstring report, jboolean isCq) {
    (void)cls;
    const char* sa = (*env)->GetStringUTFChars(env, nonstd, NULL);
    const char* sb = (*env)->GetStringUTFChars(env, std, NULL);
    const char* sc = (report != NULL) ? (*env)->GetStringUTFChars(env, report, NULL) : NULL;
    uint8_t msg[77];
    const MfskStatus st = (sa && sb && (report == NULL || sc))
        ? mfsk_pack77_type4(sa, sb, sc, isCq == JNI_TRUE, msg)
        : MFSK_STATUS_INVALID_ARG;
    if (sa) (*env)->ReleaseStringUTFChars(env, nonstd, sa);
    if (sb) (*env)->ReleaseStringUTFChars(env, std, sb);
    if (sc) (*env)->ReleaseStringUTFChars(env, report, sc);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "pack77_type4 failed"); return NULL; }
    return msg77_to_java(env, msg);
}

/// `mfsk_unpack77` with no decoder, so a `<...>` stays `<...>`.
JNIEXPORT jstring JNICALL
Java_io_github_mfskcore_Mfsk_nativeUnpack77(JNIEnv* env, jclass cls, jbyteArray msg) {
    (void)cls;
    if ((*env)->GetArrayLength(env, msg) != 77) {
        throw_status(env, MFSK_STATUS_INVALID_ARG, "a 77-bit message is 77 bytes");
        return NULL;
    }
    uint8_t bits[77];
    (*env)->GetByteArrayRegion(env, msg, 0, 77, (jbyte*)bits);
    char text[256];
    size_t n = 0;
    const MfskStatus st = mfsk_unpack77(bits, text, sizeof text, &n);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "unpack77 failed"); return NULL; }
    return (*env)->NewStringUTF(env, text);
}

/// Tones → PCM for a packed message, returning `short[]`.
///
/// The stages are separate in C because a caller may want the tone
/// sequence; a Kotlin consumer almost never does, so the binding
/// offers the composition and keeps the stages out of the API surface
/// until someone asks.
JNIEXPORT jshortArray JNICALL
Java_io_github_mfskcore_Mfsk_nativeSynthesize(
        JNIEnv* env, jclass cls, jint mode, jbyteArray message, jfloat freqHz) {
    (void)cls;
    if ((*env)->GetArrayLength(env, message) != 77) {
        throw_status(env, MFSK_STATUS_INVALID_ARG, "a 77-bit message is 77 bytes");
        return NULL;
    }
    uint8_t msg[77];
    (*env)->GetByteArrayRegion(env, message, 0, 77, (jbyte*)msg);

    const size_t nTones = mfsk_symbol_count((uint32_t)mode);
    if (nTones == 0) {
        throw_status(env, MFSK_STATUS_UNSUPPORTED, "this mode has no tone stage");
        return NULL;
    }
    uint8_t* tones = (uint8_t*)malloc(nTones);
    if (tones == NULL) { throw_ise(env, "out of memory"); return NULL; }
    size_t got = 0;
    MfskStatus st = mfsk_message_to_tones((uint32_t)mode, msg, tones, nTones, &got);
    if (st != MFSK_STATUS_OK) {
        free(tones);
        throw_last(env, st, "message_to_tones failed");
        return NULL;
    }

    const size_t nPcm = mfsk_synth_output_len((uint32_t)mode);
    jshortArray out = (*env)->NewShortArray(env, (jsize)nPcm);
    if (out == NULL) { free(tones); return NULL; }
    jshort* dst = (*env)->GetShortArrayElements(env, out, NULL);
    if (dst == NULL) { free(tones); return NULL; }
    size_t wrote = 0;
    st = mfsk_tones_to_i16((uint32_t)mode, tones, got, freqHz, 8000,
                           (int16_t*)dst, nPcm, &wrote);
    free(tones);
    (*env)->ReleaseShortArrayElements(env, out, dst, 0);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "tones_to_i16 failed"); return NULL; }
    return out;
}

/// `mfsk_encode_jt9` / `mfsk_encode_jt65` into a fresh `float[]`.
JNIEXPORT jfloatArray JNICALL
Java_io_github_mfskcore_Mfsk_nativeSynthesizeJt(
        JNIEnv* env, jclass cls, jint mode, jstring a, jstring b, jstring c, jfloat freqHz) {
    (void)cls;
    typedef MfskStatus (*Enc)(const char*, const char*, const char*, float, float*,
                              uintptr_t, uintptr_t*);
    Enc enc = NULL;
    if ((uint32_t)mode == MFSK_MODE_JT9) enc = mfsk_encode_jt9;
    else if ((uint32_t)mode == MFSK_MODE_JT65) enc = mfsk_encode_jt65;
    else {
        throw_status(env, MFSK_STATUS_UNSUPPORTED, "not JT9 or JT65");
        return NULL;
    }
    const char* sa = (*env)->GetStringUTFChars(env, a, NULL);
    const char* sb = (*env)->GetStringUTFChars(env, b, NULL);
    const char* sc = (*env)->GetStringUTFChars(env, c, NULL);
    jfloatArray out = NULL;
    if (sa && sb && sc) {
        size_t need = 0;
        /* A zero-capacity call reports the size it needs. */
        (void)enc(sa, sb, sc, freqHz, NULL, 0, &need);
        float* buf = (float*)malloc(need * sizeof(float));
        if (buf == NULL) {
            throw_ise(env, "out of memory");
        } else {
            size_t got = 0;
            const MfskStatus st = enc(sa, sb, sc, freqHz, buf, need, &got);
            if (st != MFSK_STATUS_OK) {
                throw_last(env, st, "encode failed");
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

/// `mfsk_encode_wspr` into a fresh `float[]`.
JNIEXPORT jfloatArray JNICALL
Java_io_github_mfskcore_Mfsk_nativeSynthesizeWspr(
        JNIEnv* env, jclass cls, jstring call, jstring grid, jint power, jfloat freqHz) {
    (void)cls;
    const char* sc = (*env)->GetStringUTFChars(env, call, NULL);
    const char* sg = (*env)->GetStringUTFChars(env, grid, NULL);
    jfloatArray out = NULL;
    if (sc && sg) {
        size_t need = 0;
        (void)mfsk_encode_wspr(sc, sg, (int32_t)power, freqHz, NULL, 0, &need);
        float* buf = (float*)malloc(need * sizeof(float));
        if (buf == NULL) {
            throw_ise(env, "out of memory");
        } else {
            size_t got = 0;
            const MfskStatus st = mfsk_encode_wspr(sc, sg, (int32_t)power, freqHz, buf, need, &got);
            if (st != MFSK_STATUS_OK) {
                throw_last(env, st, "encode failed");
            } else {
                out = (*env)->NewFloatArray(env, (jsize)got);
                if (out != NULL) (*env)->SetFloatArrayRegion(env, out, 0, (jsize)got, buf);
            }
            free(buf);
        }
    } else {
        throw_ise(env, "could not read the message strings");
    }
    if (sc) (*env)->ReleaseStringUTFChars(env, call, sc);
    if (sg) (*env)->ReleaseStringUTFChars(env, grid, sg);
    return out;
}

// ── Streams ─────────────────────────────────────────────────────────

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskStream_nativeOpen(
        JNIEnv* env, jclass cls, jint mode, jint sampleRate) {
    (void)cls;
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskStream* s = mfsk_stream_open((uint32_t)mode, (uint32_t)sampleRate, &st);
    if (s == NULL) { throw_last(env, st, "mfsk_stream_open failed"); return 0; }
    return (jlong)(intptr_t)s;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskStream_nativeClose(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    mfsk_stream_close((MfskStream*)(intptr_t)h);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskStream_nativePushI16(
        JNIEnv* env, jclass cls, jlong h, jshortArray samples) {
    (void)cls;
    const jsize n = (*env)->GetArrayLength(env, samples);
    jshort* pcm = (*env)->GetShortArrayElements(env, samples, NULL);
    if (pcm == NULL) return;
    const MfskStatus st = mfsk_stream_push_i16((MfskStream*)(intptr_t)h, (const int16_t*)pcm, (size_t)n);
    (*env)->ReleaseShortArrayElements(env, samples, pcm, JNI_ABORT);
    if (st != MFSK_STATUS_OK) throw_last(env, st, "stream push failed");
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskStream_nativePushF32(
        JNIEnv* env, jclass cls, jlong h, jfloatArray samples) {
    (void)cls;
    const jsize n = (*env)->GetArrayLength(env, samples);
    jfloat* pcm = (*env)->GetFloatArrayElements(env, samples, NULL);
    if (pcm == NULL) return;
    const MfskStatus st = mfsk_stream_push_f32((MfskStream*)(intptr_t)h, (const float*)pcm, (size_t)n);
    (*env)->ReleaseFloatArrayElements(env, samples, pcm, JNI_ABORT);
    if (st != MFSK_STATUS_OK) throw_last(env, st, "stream push failed");
}

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskStream_nativePosition(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return (jlong)mfsk_stream_position((const MfskStream*)(intptr_t)h);
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskStream_nativeSetTime(
        JNIEnv* env, jclass cls, jlong h, jlong utcNs, jlong atSample) {
    (void)cls;
    int32_t change = 0;
    const MfskStatus st = mfsk_stream_set_time((MfskStream*)(intptr_t)h, (int64_t)utcNs,
                                               (uint64_t)atSample, &change);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "stream set_time failed"); return 0; }
    return (jint)change;
}

JNIEXPORT jboolean JNICALL
Java_io_github_mfskcore_MfskStream_nativeSlotReady(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return mfsk_stream_slot_ready((const MfskStream*)(intptr_t)h) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_io_github_mfskcore_MfskStream_nativeSlotIsWhole(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return mfsk_stream_slot_is_whole((const MfskStream*)(intptr_t)h) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskStream_nativeSetPrefixPoints(
        JNIEnv* env, jclass cls, jlong h, jintArray points) {
    (void)cls;
    const jsize n = points == NULL ? 0 : (*env)->GetArrayLength(env, points);
    size_t buf[16];
    if (n > 16) {
        throw_status(env, MFSK_STATUS_INVALID_ARG, "more than 16 prefix points");
        return;
    }
    if (n > 0) {
        jint v[16];
        (*env)->GetIntArrayRegion(env, points, 0, n, v);
        for (jsize i = 0; i < n; ++i) {
            if (v[i] < 0) {
                throw_status(env, MFSK_STATUS_INVALID_ARG, "a prefix point is negative");
                return;
            }
            buf[i] = (size_t)v[i];
        }
    }
    const MfskStatus st =
        mfsk_stream_set_prefix_points((MfskStream*)(intptr_t)h, n > 0 ? buf : NULL, (size_t)n);
    if (st != MFSK_STATUS_OK) throw_last(env, st, "set_prefix_points failed");
}

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskStream_nativeDropped(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return (jlong)mfsk_stream_dropped((const MfskStream*)(intptr_t)h);
}

/// The waiting slot's audio, or null when none is ready. `meta[0]` receives
/// its period, `meta[1]` its UTC start in ns (0 with no clock).
JNIEXPORT jshortArray JNICALL
Java_io_github_mfskcore_MfskStream_nativeTakeSlot(
        JNIEnv* env, jclass cls, jlong h, jint cap, jlongArray meta) {
    (void)cls;
    if (cap <= 0) return NULL;
    jshortArray out = (*env)->NewShortArray(env, cap);
    if (out == NULL) return NULL;
    jshort* dst = (*env)->GetShortArrayElements(env, out, NULL);
    if (dst == NULL) return NULL;
    int64_t period = 0, utc = 0;
    const size_t got = mfsk_stream_take_slot_i16((MfskStream*)(intptr_t)h, (int16_t*)dst,
                                                 (size_t)cap, &period, &utc);
    (*env)->ReleaseShortArrayElements(env, out, dst, 0);
    if (got == 0) return NULL;
    const jlong m[2] = { (jlong)period, (jlong)utc };
    (*env)->SetLongArrayRegion(env, meta, 0, 2, m);
    if ((size_t)cap == got) return out;
    jshortArray trimmed = (*env)->NewShortArray(env, (jsize)got);
    if (trimmed == NULL) return NULL;
    jshort* src = (*env)->GetShortArrayElements(env, out, NULL);
    if (src == NULL) return NULL;
    (*env)->SetShortArrayRegion(env, trimmed, 0, (jsize)got, src);
    (*env)->ReleaseShortArrayElements(env, out, src, JNI_ABORT);
    return trimmed;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskStream_nativeClear(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    mfsk_stream_clear((MfskStream*)(intptr_t)h);
}

// ── Wideband IQ ─────────────────────────────────────────────────────

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeOpen(
        JNIEnv* env, jclass cls, jint sampleRate, jdouble centerHz, jint format,
        jboolean iqSwap, jint channelizer) {
    (void)cls;
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskIqReceiver* rx = mfsk_iq_open_with((uint32_t)sampleRate, centerHz, (uint32_t)format,
                                           iqSwap == JNI_TRUE ? 1u : 0u,
                                           (uint32_t)channelizer, &st);
    if (rx == NULL) { throw_last(env, st, "mfsk_iq_open failed"); return 0; }
    return (jlong)(intptr_t)rx;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeClose(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    mfsk_iq_close((MfskIqReceiver*)(intptr_t)h);
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeAddChannel(
        JNIEnv* env, jclass cls, jlong h, jdouble dialHz, jint mode,
        jfloatArray pf, jintArray pi, jobjectArray ps,
        jfloatArray ef, jintArray ei, jobjectArray es) {
    (void)cls;
    MfskParams params;
    MfskExtras extras;
    const MfskParams* pp;
    const MfskExtras* ep;
    if (!read_optional(env, pf, pi, ps, &params, &pp, ef, ei, es, &extras, &ep)) return -1;
    uint32_t channel = 0;
    const MfskStatus st = mfsk_iq_add_channel((MfskIqReceiver*)(intptr_t)h, dialHz,
                                              (uint32_t)mode, pp, ep, &channel);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "add_channel failed"); return -1; }
    return (jint)channel;
}

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeChannelDecoder(
        JNIEnv* env, jclass cls, jlong h, jint channel) {
    (void)env; (void)cls;
    return (jlong)(intptr_t)mfsk_iq_channel_decoder((MfskIqReceiver*)(intptr_t)h, (uint32_t)channel);
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeChannelState(
        JNIEnv* env, jclass cls, jlong h, jint channel) {
    (void)env; (void)cls;
    return (jint)mfsk_iq_channel_state((MfskIqReceiver*)(intptr_t)h, (uint32_t)channel);
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeRemoveChannel(
        JNIEnv* env, jclass cls, jlong h, jint channel) {
    (void)cls;
    const MfskStatus st = mfsk_iq_remove_channel((MfskIqReceiver*)(intptr_t)h, (uint32_t)channel);
    if (st != MFSK_STATUS_OK) throw_last(env, st, "remove_channel failed");
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeSetTime(
        JNIEnv* env, jclass cls, jlong h, jlong utcNs, jlong atSample) {
    (void)cls;
    int32_t change = 0;
    const MfskStatus st = mfsk_iq_set_time((MfskIqReceiver*)(intptr_t)h, (int64_t)utcNs,
                                           (uint64_t)atSample, &change);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "iq set_time failed"); return 0; }
    return (jint)change;
}

/// Returns `{paused, resumed}`.
JNIEXPORT jintArray JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeRetune(
        JNIEnv* env, jclass cls, jlong h, jdouble centerHz) {
    (void)cls;
    uint32_t paused = 0, resumed = 0;
    const MfskStatus st = mfsk_iq_retune((MfskIqReceiver*)(intptr_t)h, centerHz, &paused, &resumed);
    if (st != MFSK_STATUS_OK) { throw_last(env, st, "retune failed"); return NULL; }
    const jint v[2] = { (jint)paused, (jint)resumed };
    jintArray out = (*env)->NewIntArray(env, 2);
    if (out != NULL) (*env)->SetIntArrayRegion(env, out, 0, 2, v);
    return out;
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeSetEarly(
        JNIEnv* env, jclass cls, jlong h, jint channel, jboolean on) {
    (void)cls;
    const MfskStatus st = mfsk_iq_set_early((MfskIqReceiver*)(intptr_t)h, (uint32_t)channel,
                                           on == JNI_TRUE);
    if (st != MFSK_STATUS_OK) throw_last(env, st, "set_early failed");
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeGap(
        JNIEnv* env, jclass cls, jlong h, jlong lost) {
    (void)cls;
    const MfskStatus st = mfsk_iq_gap((MfskIqReceiver*)(intptr_t)h, (uint64_t)lost);
    if (st != MFSK_STATUS_OK) throw_last(env, st, "gap failed");
}

JNIEXPORT void JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativePush(
        JNIEnv* env, jclass cls, jlong h, jbyteArray data, jint length) {
    (void)cls;
    if (length < 0 || length > (*env)->GetArrayLength(env, data)) {
        throw_status(env, MFSK_STATUS_INVALID_ARG, "length is past the end of the array");
        return;
    }
    jbyte* bytes = (*env)->GetByteArrayElements(env, data, NULL);
    if (bytes == NULL) return;
    const MfskStatus st = mfsk_iq_push((MfskIqReceiver*)(intptr_t)h, bytes, (size_t)length);
    (*env)->ReleaseByteArrayElements(env, data, bytes, JNI_ABORT);
    if (st != MFSK_STATUS_OK) throw_last(env, st, "iq push failed");
}

JNIEXPORT jlong JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativeSamplesIn(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return (jlong)mfsk_iq_samples_in((MfskIqReceiver*)(intptr_t)h);
}

JNIEXPORT jint JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativePending(JNIEnv* env, jclass cls, jlong h) {
    (void)env; (void)cls;
    return (jint)mfsk_iq_pending((MfskIqReceiver*)(intptr_t)h);
}

/// Everything waiting, oldest first, as an `MfskIqDecode[]`.
JNIEXPORT jobjectArray JNICALL
Java_io_github_mfskcore_MfskIqReceiver_nativePoll(JNIEnv* env, jclass cls, jlong h) {
    (void)cls;
    MfskIqReceiver* rx = (MfskIqReceiver*)(intptr_t)h;
    jclass rowCls = (*env)->FindClass(env, CLS "MfskIqDecode");
    if (rowCls == NULL) return NULL;
    jmethodID ctor = (*env)->GetMethodID(
        env, rowCls, "<init>",
        "(IILjava/lang/String;DFFFJJZJLjava/lang/Float;Ljava/lang/Float;Ljava/lang/Integer;"
        "IZZLjava/lang/String;ILjava/lang/Integer;L" CLS "MfskStage;)V");
    if (ctor == NULL) return NULL;
    const size_t n = mfsk_iq_pending(rx);
    jobjectArray out = (*env)->NewObjectArray(env, (jsize)n, rowCls, NULL);
    if (out == NULL) return NULL;
    for (size_t i = 0; i < n; ++i) {
        MfskIqDecode r;
        memset(&r, 0, sizeof r);
        r.size = sizeof r;
        if (mfsk_iq_poll(rx, &r) != 1) break;
        jstring text = (*env)->NewStringUTF(env, r.text);
        if (text == NULL) return NULL;
        jstring key = key_hex(env, r.key_bits, r.key);
        if (key == NULL) return NULL;
        jobject sync = box_float(env, (r.flags & MFSK_DECODE_FLAG_HAS_SYNC_SCORE) != 0, r.sync_score);
        jobject cv = box_float(env, (r.flags & MFSK_DECODE_FLAG_HAS_SYNC_CV) != 0, r.sync_cv);
        jobject hard = box_int(env, (r.flags & MFSK_DECODE_FLAG_HAS_HARD_ERRORS) != 0,
                               (int32_t)r.hard_errors);
        jobject delivery = box_int(env, r.delivery >= 0, r.delivery);
        jobject stage = stage_of(env, r.stage);
        if ((*env)->ExceptionCheck(env)) return NULL;
        jobject obj = (*env)->NewObject(
            env, rowCls, ctor, (jint)r.channel, (jint)r.mode, text, (jdouble)r.abs_freq_hz,
            (jfloat)r.freq_hz, (jfloat)r.dt_sec, (jfloat)r.snr_db, (jlong)r.period,
            (jlong)r.slot_start_sample, r.has_utc ? JNI_TRUE : JNI_FALSE,
            (jlong)r.slot_start_utc_ns, sync, cv, hard, (jint)r.pass,
            (jboolean)((r.flags & MFSK_DECODE_FLAG_HASH_RESOLVED) != 0),
            (jboolean)((r.flags & MFSK_DECODE_FLAG_COPIED_LAST_TX) != 0),
            key, (jint)r.key_bits, delivery, stage);
        (*env)->DeleteLocalRef(env, text);
        (*env)->DeleteLocalRef(env, key);
        if (sync) (*env)->DeleteLocalRef(env, sync);
        if (cv) (*env)->DeleteLocalRef(env, cv);
        if (hard) (*env)->DeleteLocalRef(env, hard);
        if (delivery) (*env)->DeleteLocalRef(env, delivery);
        if (stage) (*env)->DeleteLocalRef(env, stage);
        if (obj == NULL) return NULL;
        (*env)->SetObjectArrayElement(env, out, (jsize)i, obj);
        (*env)->DeleteLocalRef(env, obj);
    }
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
    jmethodID ctor = (*env)->GetMethodID(env, cls, "<init>", "(JLjava/lang/String;ZFFFI)V");
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
                                        (jfloat)u.f1_hz, (jfloat)u.start_s, (jfloat)u.snr_db,
                                        (jint)u.kind);
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
