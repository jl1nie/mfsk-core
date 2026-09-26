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

/// Rows delivered as they are found, for a host that wants to show
/// them before the call returns.
///
/// A `fun interface`, so a lambda is enough:
/// `session.onDecode { row -> ... }`.
fun interface MfskDecodeListener {
    fun onDecode(row: MfskDecode)
}

/// Polled during a decode to ask whether to keep going. Returning false
/// stops the search and returns what has been found.
///
/// **The library reads no clock**, so the deadline is yours:
/// ```kotlin
/// val deadline = System.nanoTime() + 200_000_000
/// session.setBudget { System.nanoTime() < deadline }
/// ```
/// This is polled per candidate and crosses JNI each time, so keep it
/// to a comparison — anything heavier belongs behind a boolean the JVM
/// side has already computed.
fun interface MfskBudgetCheck {
    fun shouldContinue(): Boolean
}

/// What a budgeted decode left undone. All-zero when no budget was set
/// or it was never reached.
data class MfskBudgetReport(
    /// The predicate refused at least once: rows are missing that an
    /// unbudgeted call would have found.
    val exhausted: Boolean,
    /// Units of work declined — a candidate on the single-pass engines,
    /// a whole SIC round on `sicRounds`.
    val candidatesSkipped: Int,
    /// Units of work actually run, counted the same way.
    val stagesRun: Int,
    /// Costas sync quality of the best skipped candidate, or null.
    /// **FT8 only** — it is the key FT8's scheduler orders by, so it
    /// says whether the cut took noise or a station.
    val cutAtSync: Int?,
    /// Sync score of the best skipped candidate on that protocol's own
    /// scale, or null when there was none.
    val cutAtScore: Float?,
)

/// An a-priori hypothesis, as the message's own fields in order.
///
/// `call1` is the message's **first** callsign field — `"CQ"` for a CQ, not
/// the transmitting station — and locks message bits 0-28, `call2` the
/// second and locks 29-57, `grid` locks 58-73. A hint locks bits rather
/// than steering a search, so the wrong order removes decodes instead of
/// costing a fraction of a dB. Each field is truncated to the ABI's 15
/// characters.
data class MfskApHint(
    val call1: String = "",
    val call2: String = "",
    val grid: String = "",
)

/// WSJT-X's **NB** setting for FST4: an impulse-noise blanker run before
/// the slot transform, for the ignition and power-line clicks whose energy
/// the transform spreads across the whole band. Needs
/// [Mfsk.CAP_NOISE_BLANKER].
sealed interface MfskNoiseBlanker {
    /// Blank the loudest [percent] of samples, `0..=25` (the GUI's range;
    /// more is refused rather than clamped). 0 blanks nothing.
    data class Percent(val percent: Int) : MfskNoiseBlanker

    /// Decode once per blanking level `0, step, 2*step, .. 20` percent.
    /// [step] is 5, 2 or 1 (the GUI offers 5 and 2). Every level above 0
    /// searches only within [toleranceHz] of [MfskDecodeParams.freqHintHz],
    /// so **without a hint only the 0 % pass runs**. Costs up to 21 decodes.
    data class Sweep(val step: Int, val toleranceHz: Float) : MfskNoiseBlanker
}

/// Everything a decode can be asked to do — `MfskDecodeParams` in the C
/// ABI.
///
/// **Start from [Mfsk.defaultParams] and `copy` what you want to change.**
/// There is deliberately no constructor default: zeroing the fields is not
/// equivalent to a mode's defaults — a zero [maxCand] or a zero band
/// decodes nothing — which is why the ABI has an init call at all.
///
/// A parameter the mode does not have is an error at
/// [MfskSession.open], not a field silently dropped at decode time; the
/// `Mfsk.CAP_*` bits say which mode has what.
data class MfskDecodeParams(
    /// Search band edges, Hz.
    val freqMinHz: Float,
    val freqMaxHz: Float,
    /// Sync threshold. **Not comparable across modes**: FT4's is measured
    /// on a different scale from FT8's and FST4's.
    val syncMin: Float,
    val maxCand: Int,
    /// [DEPTH_MODE_DEFAULT], [DEPTH_BP_ALL] or [DEPTH_BP_ALL_OSD].
    val depth: Int,
    /// [STRICTNESS_STRICT], [STRICTNESS_NORMAL] or [STRICTNESS_DEEP].
    val strictness: Int,
    /// [EQ_OFF] or [EQ_LOCAL]. A property of the *input audio* — it
    /// flattens a passband an analogue filter has tilted — not of the
    /// search.
    val eqMode: Int,
    /// Prioritise candidates near this frequency, or null. It is also the
    /// QSO frequency for the a-priori passes: an AP hint that locks both
    /// callsigns is tried only within 50 Hz of it (or of [txFreqHz] on
    /// FT8), and not at all when it is unset.
    val freqHintHz: Float?,
    /// Successive-interference-cancellation rounds, 0 for none. Needs
    /// [Mfsk.CAP_SIC_ROUNDS].
    val sicRounds: Int,
    /// Checkpoint-emulation early decode. Needs [Mfsk.CAP_SIC_EARLY].
    val sicEarly: Boolean,
    /// A-priori hint, or null. Needs [Mfsk.CAP_AP_WIDEBAND].
    val apHint: MfskApHint?,
    /// Half-width of a narrow-band search, Hz; 0 for the mode's default.
    /// Needs [Mfsk.CAP_SNIPER], and [freqHintHz] as the carrier to aim at.
    val searchHz: Float,
    /// The operator's transmit frequency (WSJT-X's `nftx`), or null. FT8
    /// also tries the AP hypothesis that locks both callsigns within 50 Hz
    /// of it. Needs [Mfsk.CAP_TX_FREQ] and the wide-band search.
    val txFreqHz: Float?,
    /// Impulse-noise blanker, or null for none. Needs
    /// [Mfsk.CAP_NOISE_BLANKER].
    val noiseBlanker: MfskNoiseBlanker?,
) {
    companion object {
        const val DEPTH_MODE_DEFAULT = 0
        /// Full LLR-variant staircase + BP, no OSD fallback.
        const val DEPTH_BP_ALL = 1
        /// Above + OSD fallback (host-only).
        const val DEPTH_BP_ALL_OSD = 2

        const val STRICTNESS_STRICT = 0
        const val STRICTNESS_NORMAL = 1
        /// Deliberately exceeds WSJT-X's own FT8 ceiling; exploratory.
        const val STRICTNESS_DEEP = 2

        const val EQ_OFF = 0
        const val EQ_LOCAL = 1

        // Slot counts of the three arrays that cross JNI — the layout is
        // documented at `read_params` in mfsk_jni.c and must move with it.
        internal const val FLOATS = 7
        internal const val INTS = 9

        internal fun fromArrays(f: FloatArray, v: IntArray): MfskDecodeParams =
            MfskDecodeParams(
                freqMinHz = f[0],
                freqMaxHz = f[1],
                syncMin = f[2],
                maxCand = v[0],
                depth = v[1],
                strictness = v[2],
                eqMode = v[3],
                // NaN is the ABI's spelling of "unset": 0 Hz is a frequency.
                freqHintHz = f[3].takeUnless { it.isNaN() },
                sicRounds = v[4],
                sicEarly = v[5] != 0,
                // `mfsk_decode_params_init` never writes an AP hint.
                apHint = null,
                searchHz = f[4],
                txFreqHz = f[5].takeUnless { it.isNaN() },
                noiseBlanker = when {
                    v[8] != 0 -> MfskNoiseBlanker.Sweep(v[8], f[6])
                    v[7] != 0 -> MfskNoiseBlanker.Percent(v[7])
                    else -> null
                },
            )
    }

    internal fun floatArray(): FloatArray {
        val sweep = noiseBlanker as? MfskNoiseBlanker.Sweep
        return floatArrayOf(
            freqMinHz, freqMaxHz, syncMin, freqHintHz ?: Float.NaN,
            searchHz, txFreqHz ?: Float.NaN, sweep?.toleranceHz ?: 0f,
        )
    }

    internal fun intArray(): IntArray {
        val pct = (noiseBlanker as? MfskNoiseBlanker.Percent)?.percent ?: 0
        val step = (noiseBlanker as? MfskNoiseBlanker.Sweep)?.step ?: 0
        return intArrayOf(
            maxCand, depth, strictness, eqMode, sicRounds,
            if (sicEarly) 1 else 0, if (apHint != null) 1 else 0, pct, step,
        )
    }

    internal fun apArray(): Array<String?> =
        arrayOf(apHint?.call1, apHint?.call2, apHint?.grid)
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
    /// Received by a stateful receiver handle rather than a slot decode
    /// (JTTY): see [MfskJttyReceiver].
    const val CAP_STREAM_RECEIVER = 1L shl 15
    /// WSJT-X's impulse-noise blanker reaches the decoder: every FST4
    /// sub-mode, no other. See [MfskDecodeParams.noiseBlanker].
    const val CAP_NOISE_BLANKER = 1L shl 16
    /// The transmit frequency steers the a-priori search: FT8 only. See
    /// [MfskDecodeParams.txFreqHz].
    const val CAP_TX_FREQ = 1L shl 17

    /// The boundary's own revision, separate from the crate version.
    val abiVersion: Int get() = nativeAbiVersion()

    /// Modes **this build** supports, which is not every `MfskMode`
    /// value — protocols are feature-gated. Enumerate rather than
    /// hardcode; that is the whole reason the introspection exists.
    fun modes(): List<Int> = (0 until nativeModeCount()).map { nativeModeAt(it) }

    fun modeName(mode: Int): String = nativeModeName(mode) ?: "mode:$mode"

    fun caps(mode: Int): Long = nativeModeCaps(mode)

    fun supports(mode: Int, cap: Long): Boolean = (caps(mode) and cap) != 0L

    /// `mode`'s published decode defaults — the starting point every
    /// [MfskDecodeParams] should come from, then `copy` what differs:
    ///
    /// ```kotlin
    /// val p = Mfsk.defaultParams(ft8).copy(freqHintHz = 1500f, txFreqHz = 1500f)
    /// MfskSession.open(ft8, p)
    /// ```
    ///
    /// Throws if the mode is not in this build.
    fun defaultParams(mode: Int): MfskDecodeParams {
        val f = FloatArray(MfskDecodeParams.FLOATS)
        val v = IntArray(MfskDecodeParams.INTS)
        nativeParamsInit(mode, f, v)
        return MfskDecodeParams.fromArrays(f, v)
    }

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
    @JvmStatic private external fun nativeParamsInit(mode: Int, floats: FloatArray, ints: IntArray)
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
        ///
        /// [params] are the session's search parameters, from
        /// [Mfsk.defaultParams]; null is the mode's defaults. **A parameter
        /// the mode does not support fails here**, with a message naming
        /// the mode, rather than being dropped at decode time.
        @JvmStatic
        fun open(mode: Int, params: MfskDecodeParams? = null): MfskSession =
            MfskSession(
                nativeOpen(
                    mode, params?.floatArray(), params?.intArray(), params?.apArray(),
                ),
            )

        @JvmStatic private external fun nativeOpen(
            mode: Int, floats: FloatArray?, ints: IntArray?, ap: Array<String?>?,
        ): Long
        @JvmStatic private external fun nativeClose(
            handle: Long, callbackCtx: Long, budgetCtx: Long,
        )
        @JvmStatic private external fun nativeSetBudget(
            handle: Long, check: MfskBudgetCheck?, oldCtx: Long,
        ): Long
        @JvmStatic private external fun nativeLastBudget(handle: Long): IntArray
        @JvmStatic private external fun nativeKeepKnown(handle: Long, keep: Boolean)
        @JvmStatic private external fun nativeKnownCount(handle: Long): Int
        @JvmStatic private external fun nativeKeepFftCache(handle: Long, keep: Boolean)
        @JvmStatic private external fun nativeAddCallsign(handle: Long, call: String)
        @JvmStatic private external fun nativeSetOnDecode(
            handle: Long, listener: MfskDecodeListener?, oldCtx: Long,
        ): Long
        @JvmStatic private external fun nativeDecode(
            handle: Long, samples: ShortArray, sampleRate: Int,
            floats: FloatArray?, ints: IntArray?, ap: Array<String?>?,
        ): Array<MfskDecode>
    }

    /// Decode one slot of 16-bit PCM.
    ///
    /// [params] override the session's for **this call only** and do not
    /// stick — a decode with a narrower band, say, does not narrow the next
    /// one. Null uses what the session was opened with.
    fun decode(
        samples: ShortArray,
        sampleRate: Int = 12_000,
        params: MfskDecodeParams? = null,
    ): List<MfskDecode> {
        check(handle != 0L) { "session is closed" }
        return nativeDecode(
            handle, samples, sampleRate,
            params?.floatArray(), params?.intArray(), params?.apArray(),
        ).toList()
    }

    /// Deliver decodes to `listener` as they are found, **in addition
    /// to** the list [decode] returns. Pass null to stop.
    ///
    /// The returned list stays authoritative: this exists for a UI that
    /// wants rows during a long slot — FST4-300's is five minutes —
    /// rather than as a second way to read the result.
    ///
    /// **Threading.** With rayon (the `desktop` feature) the listener
    /// is called from worker threads, possibly several at once, in
    /// completion order. Without it (`mobile`) there is one thread and
    /// candidate order. So the listener must be safe to call
    /// concurrently, and an Android one that touches the UI has to post
    /// to the main looper rather than assume it is on it.
    ///
    /// Unlike a callback on a plain pthread, this one does not require
    /// [Mfsk.configureRuntime] first: the shim attaches the worker
    /// thread itself if the VM has never seen it, as a daemon so the
    /// JVM can still exit.
    ///
    /// An exception thrown by the listener cannot propagate into a
    /// rayon worker: it is printed and swallowed, and the decode
    /// continues.
    fun onDecode(listener: MfskDecodeListener?) {
        check(handle != 0L) { "session is closed" }
        callbackCtx = nativeSetOnDecode(handle, listener, callbackCtx)
    }

    /// Poll `check` during every subsequent decode; returning false
    /// stops the search and returns what was found. Pass null to remove
    /// the budget.
    ///
    /// **The triage sweep is a floor**: it is never gated, so a budget
    /// shorter than it returns nothing *and still spends that time*.
    /// Measured at ~13 ms of a ~28 ms FT8 decode. `maxCand` is the knob
    /// that moves the floor.
    ///
    /// Throws if the mode does not publish [Mfsk.CAP_BUDGET].
    fun setBudget(check: MfskBudgetCheck?) {
        check(handle != 0L) { "session is closed" }
        budgetCtx = nativeSetBudget(handle, check, budgetCtx)
    }

    /// What the budget cut short on the **last** decode.
    val lastBudget: MfskBudgetReport
        get() {
            check(handle != 0L) { "session is closed" }
            val v = nativeLastBudget(handle)
            return MfskBudgetReport(
                exhausted = v[0] != 0,
                candidatesSkipped = v[1],
                stagesRun = v[2],
                cutAtSync = if (v[3] >= 0) v[3] else null,
                // Int.MIN_VALUE is the "absent" spelling: an int array
                // cannot carry the NaN the C struct uses.
                cutAtScore = if (v[4] != Int.MIN_VALUE) v[4] / 1_000_000.0f else null,
            )
        }

    /// Carry each decode's results into the next as **known** signals:
    /// skipped rather than re-reported, and subtracted from the audio
    /// where the mode publishes [Mfsk.CAP_KNOWN_SUBTRACT].
    ///
    /// `false` both stops carrying and drops what is held, which is how
    /// a new slot starts.
    fun keepKnown(keep: Boolean) {
        check(handle != 0L) { "session is closed" }
        nativeKeepKnown(handle, keep)
    }

    /// How many known signals the session carries into the next decode.
    val knownCount: Int
        get() = if (handle != 0L) nativeKnownCount(handle) else 0

    /// Keep the slot FFT and reuse it for the next decode **of the same
    /// audio**. Reuse is checked rather than trusted: the cache is
    /// stored with a fingerprint of the audio it came from, so a decode
    /// of anything else transforms afresh instead of returning a
    /// confident wrong answer.
    fun keepFftCache(keep: Boolean) {
        check(handle != 0L) { "session is closed" }
        nativeKeepFftCache(handle, keep)
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
            // The native side clears the callback before closing and
            // then frees the context: the session holds a raw pointer
            // to it, so the order is not decorative.
            nativeClose(handle, callbackCtx, budgetCtx)
            callbackCtx = 0L
            budgetCtx = 0L
            handle = 0L
        }
    }

    /// Opaque pointer to the shim's per-session callback state — the
    /// listener's global ref and the cached IDs. Owned here so there is
    /// exactly one, and freed by [close].
    private var callbackCtx: Long = 0L

    /// Same, for the budget predicate.
    private var budgetCtx: Long = 0L
}

/// One JTTY message as far as it is known.
///
/// A message is reported each time it grows and once more when it
/// completes; [id] is stable for its life, so a UI replaces its row by
/// [id]. Updates are coalesced per message between polls.
data class MfskJttyUpdate(
    val id: Long,
    /// The text so far. Frames that were never heard show as ` ... `;
    /// TEXT5 spaces as `~`, as upstream shows them.
    val text: String,
    /// The end-of-message frame has arrived.
    val complete: Boolean,
    /// Frequency of the latest frame, Hz.
    val freqHz: Float,
    /// Start of the first frame, seconds from the first sample pushed
    /// since the receiver was opened or reset.
    val startSeconds: Float,
)

/// What a JTTY receiver looks for. The defaults are `rjtty`'s.
data class MfskJttyParams(
    /// The operator's receive frequency, Hz (channel 0's centre).
    val f0Hz: Float = 1500f,
    /// Half-width of channel 0, Hz.
    val ftolHz: Float = 50f,
    /// Sync-gate S/N floor on channel 0, dB.
    val sminDb: Float = 4.6f,
    /// The band channels 1 and 2 watch for stations off [f0Hz], Hz.
    val nfaHz: Float = 200f,
    val nfbHz: Float = 2800f,
    /// Take each decoded frame off the signal and search again; off is a
    /// single-signal receiver that loses a weak station under a strong one.
    val subtract: Boolean = true,
)

/// A JTTY receiver: WSJT-X 3.2's non-slotted keyboard mode. Audio goes
/// in as it arrives, in chunks of any size; messages come out as they
/// are assembled.
///
/// **Not a slot decode.** JTTY frames start whenever the sender likes
/// and a message is several of them, so the receiver keeps state — the
/// search window, the messages under assembly, the audio a re-sweep of
/// earlier windows still needs — and reports *updates*.
///
/// **Single-threaded**: one thread at a time, like [MfskSession].
/// [push] decodes every window the audio completes before it returns —
/// a few tens of milliseconds per 0.47 s of audio — so call it off the
/// UI thread.
///
/// ```kotlin
/// MfskJttyReceiver.open(48_000).use { rx ->
///     for (chunk in audioChunks) for (u in rx.push(chunk)) show(u.id, u.text)
/// }
/// ```
class MfskJttyReceiver private constructor(private var handle: Long) : AutoCloseable {

    companion object {
        /// Open a receiver taking 16-bit mono PCM at `sampleRate`
        /// (anything but 12 000 Hz is resampled, linearly). Throws if
        /// the parameters are invalid or the build lacks JTTY.
        @JvmStatic
        fun open(sampleRate: Int = 12_000, params: MfskJttyParams = MfskJttyParams()) =
            MfskJttyReceiver(
                nativeOpen(
                    sampleRate, params.f0Hz, params.ftolHz, params.sminDb,
                    params.nfaHz, params.nfbHz, params.subtract,
                ),
            )

        @JvmStatic private external fun nativeOpen(
            sampleRate: Int, f0Hz: Float, ftolHz: Float, sminDb: Float,
            nfaHz: Float, nfbHz: Float, subtract: Boolean,
        ): Long
        @JvmStatic private external fun nativeClose(handle: Long)
        @JvmStatic private external fun nativeSetParams(
            handle: Long, f0Hz: Float, ftolHz: Float, sminDb: Float,
            nfaHz: Float, nfbHz: Float, subtract: Boolean,
        )
        @JvmStatic private external fun nativePush(
            handle: Long, samples: ShortArray,
        ): Array<MfskJttyUpdate>
        @JvmStatic private external fun nativePoll(handle: Long): Array<MfskJttyUpdate>
        @JvmStatic private external fun nativeFinish(handle: Long)
        @JvmStatic private external fun nativeReset(handle: Long)
    }

    /// Feed audio and return the message updates it produced (already
    /// drained from the receiver's queue, oldest first, one per message).
    fun push(samples: ShortArray): List<MfskJttyUpdate> {
        check(handle != 0L) { "receiver is closed" }
        return nativePush(handle, samples).toList()
    }

    /// Updates waiting that a previous call has not returned. [push]
    /// already drains after itself, so this is for a caller that wants
    /// to poll on its own schedule; it is empty otherwise.
    fun poll(): List<MfskJttyUpdate> {
        check(handle != 0L) { "receiver is closed" }
        return nativePoll(handle).toList()
    }

    /// The audio has ended: report every message still waiting for a
    /// continuation one last time, as incomplete. A live receiver never
    /// needs this.
    fun finish(): List<MfskJttyUpdate> {
        check(handle != 0L) { "receiver is closed" }
        nativeFinish(handle)
        return nativePoll(handle).toList()
    }

    /// Change the settings; they apply from the next window.
    fun setParams(params: MfskJttyParams) {
        check(handle != 0L) { "receiver is closed" }
        nativeSetParams(
            handle, params.f0Hz, params.ftolHz, params.sminDb,
            params.nfaHz, params.nfbHz, params.subtract,
        )
    }

    /// Forget everything and start again at sample 0.
    fun reset() {
        check(handle != 0L) { "receiver is closed" }
        nativeReset(handle)
    }

    override fun close() {
        if (handle != 0L) {
            nativeClose(handle)
            handle = 0L
        }
    }
}

/// JTTY transmit: the text packer and the synthesiser.
///
/// Upstream's `pack_jtty` picks the fewest frames for the text (a callsign, a grid,
/// a report or a control phrase is one frame; anything else is five characters a
/// frame) and `genjtty` turns them into tones. The F-key templates and N1MM tags
/// WSJT-X puts around it are not part of this library.
object MfskJtty {
    /// The exchange profile: only [RTTY_ROUNDUP] changes the packing (it adds
    /// serial-number and state candidates).
    const val PROFILE_UNKNOWN = 0
    const val PROFILE_FIELD_DAY = 1
    const val PROFILE_RTTY_ROUNDUP = 2

    /// Channel tones (0..3), 59 per frame. Empty for an empty message. Throws if
    /// the text cannot be sent: over 80 characters, over 16 frames, or an RTTY
    /// serial that does not fit.
    fun tones(text: String, profile: Int = PROFILE_UNKNOWN): ByteArray {
        Mfsk.modes() // loads the native library
        return nativeEncodeTones(text, profile)
    }

    /// 16-bit PCM at 12 kHz for `tones`, `freqHz` the frequency of tone 0, `amplitude`
    /// the peak in counts.
    fun synthesize(tones: ByteArray, freqHz: Float = 1500f, amplitude: Float = 8000f): ShortArray {
        Mfsk.modes()
        return nativeTonesToPcm(tones, freqHz, amplitude)
    }

    /// Text straight to audio: [tones] then [synthesize].
    fun encode(
        text: String, profile: Int = PROFILE_UNKNOWN, freqHz: Float = 1500f, amplitude: Float = 8000f,
    ): ShortArray {
        val t = tones(text, profile)
        return if (t.isEmpty()) ShortArray(0) else synthesize(t, freqHz, amplitude)
    }

    @JvmStatic private external fun nativeEncodeTones(text: String, profile: Int): ByteArray
    @JvmStatic private external fun nativeTonesToPcm(
        tones: ByteArray, freqHz: Float, amplitude: Float,
    ): ShortArray
}
