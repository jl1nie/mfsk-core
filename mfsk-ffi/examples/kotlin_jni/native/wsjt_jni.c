// SPDX-License-Identifier: GPL-3.0-or-later
//
// Thin JNI shim that marshals between the Kotlin `Wsjt` class and the
// wsjt-ffi C ABI (libwsjt.so). The shim itself contains no decoder logic
// — it only converts JNI types to the plain-C handle/pointer pairs that
// libwsjt.so expects.
//
// Build (Android NDK):
//   $NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android30-clang \
//     -shared -fPIC -o libwsjt_jni.so wsjt_jni.c -I../../../include \
//     -L<path-to-libwsjt> -lwsjt
// Build (JVM on Linux for testing):
//   gcc -shared -fPIC -o libwsjt_jni.so wsjt_jni.c \
//     -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" \
//     -I../../../include -L../../../../target/release -lwsjt \
//     -Wl,-rpath,$PWD/../../../../target/release

#include <jni.h>
#include <stdlib.h>
#include <string.h>

#include "wsjt.h"

// ── Handle helpers ──────────────────────────────────────────────────────
//
// We stash the native pointer as a jlong on the Kotlin side. JNI
// guarantees jlong is at least 64 bits, so pointer fits fine on all
// supported Android/JVM targets.

static inline WsjtDecoder* handle_from(jlong h) {
    return (WsjtDecoder*)(uintptr_t)h;
}
static inline jlong handle_to(WsjtDecoder* p) {
    return (jlong)(uintptr_t)p;
}

// ── JNI exports ─────────────────────────────────────────────────────────

JNIEXPORT jint JNICALL
Java_io_github_rsft8n_Wsjt_nativeVersion(JNIEnv* env, jclass clazz) {
    (void)env; (void)clazz;
    return (jint)wsjt_version();
}

JNIEXPORT jlong JNICALL
Java_io_github_rsft8n_Wsjt_nativeDecoderNew(JNIEnv* env, jclass clazz, jint protocol) {
    (void)env; (void)clazz;
    WsjtDecoder* dec = wsjt_decoder_new((WsjtProtocol)protocol);
    return handle_to(dec);
}

JNIEXPORT void JNICALL
Java_io_github_rsft8n_Wsjt_nativeDecoderFree(JNIEnv* env, jclass clazz, jlong handle) {
    (void)env; (void)clazz;
    wsjt_decoder_free(handle_from(handle));
}

// Decode from a short[] — returns a String[] where each entry is
// "freq|dt|snr|errors|pass|text" (pipe-separated). Kotlin side parses
// back into data classes. Keeping the marshalling trivial (no custom
// JNI struct per-message) lets the same shim serve plain Java
// consumers unchanged.
JNIEXPORT jobjectArray JNICALL
Java_io_github_rsft8n_Wsjt_nativeDecodeI16(
    JNIEnv* env, jclass clazz,
    jlong handle, jshortArray samples, jint sampleRate) {
    (void)clazz;

    jsize n = (*env)->GetArrayLength(env, samples);
    jshort* ptr = (*env)->GetShortArrayElements(env, samples, NULL);
    if (!ptr) { return NULL; }

    WsjtMessageList list = {0};
    WsjtStatus st = wsjt_decode_i16(
        handle_from(handle),
        (const int16_t*)ptr,
        (size_t)n,
        (uint32_t)sampleRate,
        &list);

    (*env)->ReleaseShortArrayElements(env, samples, ptr, JNI_ABORT);

    if (st != WSJT_STATUS_OK) {
        // Caller gets an empty array on failure; last_error accessible via
        // Java_io_github_rsft8n_Wsjt_nativeLastError.
        wsjt_message_list_free(&list);
        jclass sclass = (*env)->FindClass(env, "java/lang/String");
        return (*env)->NewObjectArray(env, 0, sclass, NULL);
    }

    jclass sclass = (*env)->FindClass(env, "java/lang/String");
    jobjectArray out = (*env)->NewObjectArray(env, (jsize)list.len, sclass, NULL);

    char buf[768];
    for (size_t i = 0; i < list.len; ++i) {
        const WsjtMessage* m = &list.items[i];
        // Use "%g" for float fields; text may contain '|' only in unusual
        // free-text messages — acceptable for an example shim.
        snprintf(buf, sizeof(buf),
                 "%g|%g|%g|%u|%u|%s",
                 (double)m->freq_hz,
                 (double)m->dt_sec,
                 (double)m->snr_db,
                 (unsigned)m->hard_errors,
                 (unsigned)m->pass,
                 m->text ? m->text : "");
        jstring s = (*env)->NewStringUTF(env, buf);
        (*env)->SetObjectArrayElement(env, out, (jsize)i, s);
        (*env)->DeleteLocalRef(env, s);
    }
    wsjt_message_list_free(&list);
    return out;
}

JNIEXPORT jstring JNICALL
Java_io_github_rsft8n_Wsjt_nativeLastError(JNIEnv* env, jclass clazz) {
    (void)clazz;
    const char* msg = wsjt_last_error();
    return (*env)->NewStringUTF(env, msg ? msg : "");
}
