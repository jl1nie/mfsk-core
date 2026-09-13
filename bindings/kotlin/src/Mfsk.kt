// SPDX-License-Identifier: GPL-3.0-or-later
//
// Kotlin binding for mfsk-core's C ABI.
//
// Thin on purpose: handle lifecycle, one decode call, one synthesis
// call, and the capability queries. Everything that decides anything
// stays in `mfsk-ffi`, so a Kotlin consumer gets exactly what a C
// consumer gets — including the refusals.
//
// Works on a desktop JVM and on Android with the NDK; CI builds the
// shim and runs the test below on Linux every PR.

package io.github.mfskcore

/// One decoded transmission.
///
/// A value type, not a handle: the ABI writes rows into memory the
/// caller owns, so there is nothing here to close and nothing that can
/// outlive a session.
data class MfskDecode(
    /// The concrete sub-mode — FST4-120 rather than "FST4".
    val mode: Int,
    val text: String,
    val freqHz: Float,
    val dtSec: Float,
    val snrDb: Float,
    val syncScore: Float,
    /// Coefficient of variation of the per-block sync powers: near 0 on
    /// a stable channel, elevated under QSB or fading.
    val syncCv: Float,
    val hardErrors: Int,
    /// FEC information bits — 91 (CRC-14) or 101 (CRC-24).
    val infoBits: Int,
    /// Which decode pass produced this row. **Protocol-private**: the
    /// numbers mean different things per mode, and are diagnostics, not
    /// something to branch on.
    val pass: Int,
    /// The text needed the callsign hash table to resolve a `<...>`
    /// reference.
    val hashResolved: Boolean,
) {
    val modeName: String get() = Mfsk.modeName(mode)
}

/// Geometry a host needs to size a buffer or place a transmission.
data class MfskModeInfo(
    val ntones: Int,
    val slotSamples12k: Int,
    val fecK: Int,
    val nSymbols: Int,
    /// Points in the forward FFT the decoder takes over the whole slot.
    ///
    /// **Read this before budgeting.** FT4 takes 92 160 and FST4-300
    /// takes 4 194 304 — a factor of 45 no other field hints at, and the
    /// reason one memory budget for every mode is wrong on a phone.
    val decodeFft1Size: Int,
    /// Milliseconds from the slot boundary to the first frame symbol.
    val txStartOffsetMs: Int,
)

object Mfsk {
    init {
        System.loadLibrary("mfsk_jni")
    }

    // Capability bits, mirroring `MFSK_CAP_*` in mfsk.h.
    const val CAP_DECODE_HANDLE = 1L shl 0
    const val CAP_SNIPER = 1L shl 1
    const val CAP_AP_NARROW = 1L shl 2
    const val CAP_AP_WIDEBAND = 1L shl 3
    const val CAP_SIC_ROUNDS = 1L shl 4
    const val CAP_SIC_EARLY = 1L shl 5
    const val CAP_OSD = 1L shl 6
    const val CAP_EQ_MODE = 1L shl 7
    const val CAP_STRICTNESS = 1L shl 8
    const val CAP_BUDGET = 1L shl 9
    const val CAP_KNOWN_FILTER = 1L shl 10
    const val CAP_KNOWN_SUBTRACT = 1L shl 11
    const val CAP_FFT_CACHE = 1L shl 12
    const val CAP_ON_RESULT = 1L shl 13
    const val CAP_ENCODE = 1L shl 14

    /// The boundary's own revision, separate from the crate version.
    val abiVersion: Int get() = nativeAbiVersion()

    /// Modes **this build** supports, which is not every `MfskMode`
    /// value — protocols are feature-gated. Enumerate rather than
    /// hardcode; that is the whole reason the introspection exists.
    fun modes(): List<Int> = (0 until nativeModeCount()).map { nativeModeAt(it) }

    fun modeName(mode: Int): String = nativeModeName(mode) ?: "mode:$mode"

    fun caps(mode: Int): Long = nativeModeCaps(mode)

    fun supports(mode: Int, cap: Long): Boolean = (caps(mode) and cap) != 0L

    fun modeInfo(mode: Int): MfskModeInfo {
        val v = nativeModeInfo(mode)
        return MfskModeInfo(v[0], v[1], v[2], v[3], v[4], v[5])
    }

    /// Take the decode off rayon's global pool.
    ///
    /// **Call this once, before the first decode, on Android.** The
    /// global pool is `num_cpus` threads with 2 MiB stacks, never
    /// joined, and — the part that matters here — not attached to ART,
    /// so nothing running on one can touch a JNIEnv. This installs a
    /// private pool whose workers the shim attaches and detaches.
    ///
    /// Returns false if a pool was already configured; it can only be
    /// built once, because rayon cannot rebuild one its threads may be
    /// parked in.
    ///
    /// The shim attaches workers with `AttachCurrentThreadAsDaemon`,
    /// not `AttachCurrentThread`. The difference is whether the JVM can
    /// ever exit: rayon's threads are never joined, and a non-daemon
    /// attached thread keeps the VM alive forever. It matters on a
    /// desktop JVM immediately and on Android whenever a process is
    /// expected to end.
    fun configureRuntime(threads: Int = 0, stackBytes: Int = 0): Boolean =
        nativeConfigureRuntime(threads, stackBytes) == 0

    /// Worker threads decoding will use. 1 means serial.
    val threadCount: Int get() = nativeThreadCount()

    /// Pack `call1 call2 report` and synthesise a frame at 12 kHz.
    ///
    /// Throws if the message does not fit the format or the mode has no
    /// tone stage — `mfsk_symbol_count` returning 0 is how the ABI says
    /// which modes those are.
    fun synthesize(mode: Int, call1: String, call2: String, report: String, freqHz: Float)
        : ShortArray = nativeSynthesize(mode, call1, call2, report, freqHz)

    @JvmStatic private external fun nativeAbiVersion(): Int
    @JvmStatic private external fun nativeModeCount(): Int
    @JvmStatic private external fun nativeModeAt(index: Int): Int
    @JvmStatic private external fun nativeModeName(mode: Int): String?
    @JvmStatic private external fun nativeModeCaps(mode: Int): Long
    @JvmStatic private external fun nativeModeInfo(mode: Int): IntArray
    @JvmStatic private external fun nativeConfigureRuntime(threads: Int, stackBytes: Int): Int
    @JvmStatic private external fun nativeThreadCount(): Int
    @JvmStatic private external fun nativeSynthesize(
        mode: Int, call1: String, call2: String, report: String, freqHz: Float,
    ): ShortArray
}

/// A decode session.
///
/// **Single-threaded**, deliberately: it owns a callsign hash table it
/// mutates on every decode, so one per thread. Concurrent decodes on
/// separate sessions are supported.
///
/// The hash table is why this is a session rather than a function. A
/// `<...>` reference cannot be expanded without having seen the
/// callsign, and a table that does not outlive the slot that populated
/// it is worth nothing.
class MfskSession private constructor(private var handle: Long) : AutoCloseable {

    companion object {
        /// Open a session for `mode`, which must claim
        /// [Mfsk.CAP_DECODE_HANDLE]. Q65 takes a nominal start sample
        /// and a tolerance and WSPR/JT9/JT65 have no builder, so those
        /// are refused here rather than decoding something differently
        /// shaped.
        @JvmStatic
        fun open(mode: Int): MfskSession = MfskSession(nativeOpen(mode))

        @JvmStatic private external fun nativeOpen(mode: Int): Long
        @JvmStatic private external fun nativeClose(handle: Long)
        @JvmStatic private external fun nativeAddCallsign(handle: Long, call: String)
        @JvmStatic private external fun nativeDecode(
            handle: Long, samples: ShortArray, sampleRate: Int,
        ): Array<MfskDecode>
    }

    /// Decode one slot of 16-bit PCM.
    fun decode(samples: ShortArray, sampleRate: Int = 12_000): List<MfskDecode> {
        check(handle != 0L) { "session is closed" }
        return nativeDecode(handle, samples, sampleRate).toList()
    }

    /// Teach the session a callsign, so a later slot's `<...>`
    /// reference to it resolves. Decoded messages populate the table
    /// automatically; this is for calls known from a band map or a log.
    fun addCallsign(call: String) {
        check(handle != 0L) { "session is closed" }
        nativeAddCallsign(handle, call)
    }

    override fun close() {
        if (handle != 0L) {
            nativeClose(handle)
            handle = 0L
        }
    }
}
