// SPDX-License-Identifier: GPL-3.0-or-later
//
// Kotlin wrapper around the mfsk-ffi C ABI (libmfsk.so) via a thin JNI
// shim (libmfsk_jni.so). Designed for direct drop-in on Android with
// the NDK, and also works on desktop JVM for headless testing.
//
// Usage example (CLI):
//
//     val dec = Mfsk.open(Mfsk.Protocol.FT8)
//     try {
//         val messages = dec.decode(shortArray, sampleRate = 12_000)
//         for (m in messages) println(m)
//     } finally {
//         dec.close()
//     }

package io.github.mfskcore

class Mfsk private constructor(private var handle: Long) : AutoCloseable {

    // Mirrors `MfskProtocol` in mfsk.h. Q65's dedicated sub-mode /
    // AP-hint / fading / AP-list function family isn't reachable
    // through this generic-handle shim — see mfsk.h's `mfsk_q65_*`
    // group if a Kotlin/JNI consumer needs those.
    enum class Protocol(val id: Int) {
        FT8(0), FT4(1), WSPR(2), JT9(3), JT65(4), FST4S60(5), Q65A30(6)
    }

    data class Message(
        val freqHz: Float,
        val dtSec: Float,
        val snrDb: Float,
        val hardErrors: Int,
        val pass: Int,
        val text: String,
    )

    /** Decode a slot of 16-bit PCM audio at the given sample rate. */
    fun decode(samples: ShortArray, sampleRate: Int): List<Message> {
        check(handle != 0L) { "Mfsk handle is closed" }
        val raw = nativeDecodeI16(handle, samples, sampleRate) ?: emptyArray()
        return raw.map(::parseMessage)
    }

    /** Release the native decoder handle. Idempotent. */
    override fun close() {
        if (handle != 0L) {
            nativeDecoderFree(handle)
            handle = 0L
        }
    }

    companion object {
        init { System.loadLibrary("mfsk_jni") }

        fun open(protocol: Protocol): Mfsk {
            val h = nativeDecoderNew(protocol.id)
            require(h != 0L) { "mfsk_decoder_new failed: ${nativeLastError()}" }
            return Mfsk(h)
        }

        /** Library version as `(major shl 16) or (minor shl 8) or patch`. */
        val version: Int get() = nativeVersion()

        /** Most recent error recorded on this thread by libmfsk. */
        fun lastError(): String = nativeLastError()

        // --- JNI methods implemented in mfsk_jni.c -------------------------
        @JvmStatic private external fun nativeVersion(): Int
        @JvmStatic private external fun nativeDecoderNew(protocol: Int): Long
        @JvmStatic private external fun nativeDecoderFree(handle: Long)
        @JvmStatic private external fun nativeDecodeI16(
            handle: Long,
            samples: ShortArray,
            sampleRate: Int,
        ): Array<String>?
        @JvmStatic private external fun nativeLastError(): String

        private fun parseMessage(raw: String): Message {
            // Format from mfsk_jni.c: freq|dt|snr|errors|pass|text
            // split(limit = 6) preserves pipes inside `text` (unlikely but
            // possible in free-text messages).
            val parts = raw.split("|", limit = 6)
            return Message(
                freqHz = parts[0].toFloat(),
                dtSec = parts[1].toFloat(),
                snrDb = parts[2].toFloat(),
                hardErrors = parts[3].toInt(),
                pass = parts[4].toInt(),
                text = parts.getOrElse(5) { "" },
            )
        }
    }
}
