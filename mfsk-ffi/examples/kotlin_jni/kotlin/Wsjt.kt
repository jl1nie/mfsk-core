// SPDX-License-Identifier: GPL-3.0-or-later
//
// Kotlin wrapper around the rs-ft8n C ABI (libwsjt.so) via a thin JNI
// shim (libwsjt_jni.so). Designed for direct drop-in on Android with
// the NDK, and also works on desktop JVM for headless testing.
//
// Usage example (CLI):
//
//     val dec = Wsjt.open(Wsjt.Protocol.FT8)
//     try {
//         val messages = dec.decode(shortArray, sampleRate = 12_000)
//         for (m in messages) println(m)
//     } finally {
//         dec.close()
//     }

package io.github.rsft8n

class Wsjt private constructor(private var handle: Long) : AutoCloseable {

    enum class Protocol(val id: Int) { FT8(0), FT4(1) }

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
        check(handle != 0L) { "Wsjt handle is closed" }
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
        init { System.loadLibrary("wsjt_jni") }

        fun open(protocol: Protocol): Wsjt {
            val h = nativeDecoderNew(protocol.id)
            require(h != 0L) { "wsjt_decoder_new failed: ${nativeLastError()}" }
            return Wsjt(h)
        }

        /** Library version as `(major shl 16) or (minor shl 8) or patch`. */
        val version: Int get() = nativeVersion()

        /** Most recent error recorded on this thread by libwsjt. */
        fun lastError(): String = nativeLastError()

        // --- JNI methods implemented in wsjt_jni.c ------------------------
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
            // Format from wsjt_jni.c: freq|dt|snr|errors|pass|text
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
