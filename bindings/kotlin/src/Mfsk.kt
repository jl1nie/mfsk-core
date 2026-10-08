// SPDX-License-Identifier: GPL-3.0-only
//
// Kotlin binding for mfsk-core's C ABI.
//
// Thin on purpose: handle lifecycle, one decode call, one synthesis
// call, and the capability queries. Everything that decides anything
// stays in `mfsk-ffi`, so a Kotlin consumer gets exactly what a C
// consumer gets — including the refusals.
//
// One decoder handle serves every slot mode (FT8, FT4, FST4, WSPR, JT9,
// JT65, Q65): [MfskDecoder], opened with an [MfskParams] (WSJT-X's
// per-period parameter block) and an [MfskExtras] (the library's options).
// JTTY has no slot and keeps its own receiver, [MfskJttyReceiver].
//
// Works on a desktop JVM and on Android with the NDK; CI builds the
// shim and runs the test below on Linux every PR.

package io.github.mfskcore

/// A call the library refused or could not do. The message is the ABI's own
/// text for the failure (the decoder's per-handle error slot when there is a
/// handle), and [status] the `MfskStatus` it came with.
open class MfskException(message: String, val status: Int) : RuntimeException(message)

/// `MFSK_STATUS_UNSUPPORTED`: the mode exists in this build but does not have
/// what was asked for (an option it lacks, a budget it does not poll, hashed
/// calls it does not carry). The message names the mode and the option. The
/// `Mfsk.CAP_*` bits say in advance which mode has what.
class MfskUnsupportedException(message: String, status: Int) : MfskException(message, status)

/// `MFSK_STATUS_INVALID_ARG`: a value out of range, a buffer too short, a
/// parameter block that is not usable. Refused, never clamped.
class MfskInvalidArgException(message: String, status: Int) : MfskException(message, status)

/// `MFSK_STATUS_UNKNOWN_PROTOCOL`: the mode is not in this build.
class MfskUnknownModeException(message: String, status: Int) : MfskException(message, status)

/// One decoded transmission.
///
/// A value type, not a handle: the ABI writes rows into memory the
/// caller owns, so there is nothing here to close and nothing that can
/// outlive a decoder.
data class MfskDecode(
    /// The concrete sub-mode — FST4-120 rather than "FST4".
    val mode: Int,
    val text: String,
    val freqHz: Float,
    val dtSec: Float,
    val snrDb: Float,
    /// Sync score on the scale of the mode's own search, so **not comparable
    /// between modes**. Null where the mode reports none: WSPR, JT9, JT65,
    /// Q65, and FT8's a7 / a8 list decodes, which run no sync search.
    val syncScore: Float?,
    /// Coefficient of variation of the per-block sync powers: near 0 on
    /// a stable channel, elevated under QSB or fading. Null wherever
    /// [syncScore] is.
    val syncCv: Float?,
    /// Hard-decision errors the FEC corrected; 0 is a clean decode. Null for
    /// WSPR, JT9, JT65 and Q65, whose decoders report no such count.
    val hardErrors: Int?,
    /// Length of the information block [MfskDecoder.copyInfo] returns —
    /// 91 for FT8 and FT4, 101 for FST4, 50 for WSPR, 72 for JT9 and JT65,
    /// 77 for Q65.
    val infoBits: Int,
    /// Which decode pass produced this row. **Protocol-private**: the
    /// numbers mean different things per mode, and are diagnostics, not
    /// something to branch on.
    val pass: Int,
    /// The text needed the callsign hash table to resolve a `<...>`
    /// reference.
    val hashResolved: Boolean,
    /// The sender set WSJT-X 3.2's **Q65 Pileup** "copied last Tx" flag, the
    /// spare 78th payload bit; WSJT-X marks such a decode with `#`. Q65 only.
    val copiedLastTx: Boolean = false,
    /// The message's identity key: its [keyBits] bits (77 for FT8, FT4, FST4
    /// and Q65; 72 for JT9 and JT65; 50 for WSPR), packed most significant bit
    /// first, as lower-case hex. The same message in two decoders has one key
    /// even when its text differs (a `<...>` resolved in one only). One message
    /// at two frequencies has one key too, so add [freqHz] to tell signals apart.
    val key: String = "",
    val keyBits: Int = 0,
    /// Which delivery of the period this row is, or came from: a row handed to
    /// [MfskDecoder.onDecode] carries its position (0, 1, 2...), a returned row
    /// the position of the delivery it was, so the two pair exactly. Null for a
    /// returned row the listener never saw, and with no listener.
    val delivery: Int? = null,
    /// When a [MfskDecoder.decodePrefix] sequence found the row; null from a
    /// plain [MfskDecoder.decode].
    val stage: MfskStage? = null,
) {
    val modeName: String get() = Mfsk.modeName(mode)
}

/// When in a period a [MfskDecoder.decodePrefix] sequence found a row
/// (`MFSK_STAGE_*`, #572).
enum class MfskStage {
    /// Before the period ended — FT8's checkpoint A, ~11.8 s in — in time to
    /// answer the station in the next period.
    EARLY,
    /// By the call whose audio was the whole period.
    FINAL,
}

/// Rows delivered as they are found, for a host that wants to show
/// them before the call returns.
///
/// A `fun interface`, so a lambda is enough:
/// `decoder.onDecode { row -> ... }`.
fun interface MfskDecodeListener {
    fun onDecode(row: MfskDecode)
}

/// Polled during a decode to ask whether to keep going. Returning false
/// stops the search and returns what has been found.
///
/// **The library reads no clock**, so the deadline is yours:
/// ```kotlin
/// val deadline = System.nanoTime() + 200_000_000
/// decoder.setBudget { System.nanoTime() < deadline }
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
    /// Rows subtracted before a later search saw them: FT8
    /// [MfskStrategy.SicEarly]'s checkpoint-B and -C loops. Fewer than the
    /// rows returned, with [exhausted] set, means the cut came while
    /// cleaning up rather than while searching. 0 everywhere else.
    val rowsSubtracted: Int = 0,
)

// ── The parameter block ─────────────────────────────────────────────

/// `ndepth & 7`.
enum class MfskDepth(internal val code: Int) {
    FAST(1), NORMAL(2),
    /// The GUI's default.
    DEEP(3);

    internal companion object {
        // 0 is the ABI's spelling of "the default", which is Deep.
        fun of(code: Int): MfskDepth = entries.firstOrNull { it.code == code } ?: DEEP
    }
}

/// Which a-priori hypotheses the QSO context may add (`lft8apon`, `lapcqonly`).
enum class MfskApMode(internal val code: Int) {
    OFF(0),
    /// Only the CQ hypothesis.
    CQ_ONLY(1),
    /// Every hypothesis the QSO context allows.
    FULL(2);

    internal companion object {
        fun of(code: Int): MfskApMode = entries.first { it.code == code }
    }
}

/// `ncontest`.
enum class MfskContest(internal val code: Int) {
    NONE(0),
    /// NA VHF, WW Digi, ARRL Digi, Q65 pileup: a 4-character grid exchange.
    GRID_EXCHANGE(1),
    EU_VHF(2), FIELD_DAY(3), RTTY_ROUNDUP(4),
    /// FT8 DXpedition, Fox.
    FOX(6),
    /// FT8 DXpedition, Hound.
    HOUND(7);

    internal companion object {
        fun of(code: Int): MfskContest = entries.first { it.code == code }
    }
}

/// `nQSOProgress`: where the operator's own QSO stands, which decides the AP
/// hypotheses tried.
enum class MfskQsoProgress(internal val code: Int) {
    CALLING(0), REPLYING(1), REPORT(2), ROGER_REPORT(3), ROGERS(4), SIGNOFF(5);

    internal companion object {
        fun of(code: Int): MfskQsoProgress = entries.first { it.code == code }
    }
}

/// `mycall` and `mygrid`.
data class MfskStation(val call: String = "", val grid: String = "")

/// `hiscall`, `hisgrid` and `nQSOProgress`.
data class MfskQso(
    val hisCall: String = "",
    val hisGrid: String = "",
    val progress: MfskQsoProgress = MfskQsoProgress.CALLING,
)

/// The per-period parameter block — WSJT-X's `params` common block
/// (`lib/jt9com.f90`): what the GUI fills before each period and the decoder
/// reads. A mode reads what its upstream decoder reads and ignores the rest,
/// as `jt9` does; [depth] decides every search setting the way `ndepth` does.
///
/// **Start from [Mfsk.defaultParams] and `copy` what you want to change.**
/// The constructor's defaults are the neutral ones, not each mode's: FT4's
/// AP starts on and FT8's off, and the band is the mode's own.
/// The text fields are limited to the ABI's inline capacity (15 characters
/// for a call, 7 for a grid); longer is refused, not truncated.
data class MfskParams(
    /// Audio band searched, Hz (`nfa`..`nfb`).
    val band: ClosedFloatingPointRange<Float>,
    /// The Rx frequency, Hz (`nfqso`), or null.
    val rxFreqHz: Float? = null,
    /// Tolerance around the Rx frequency, Hz (`ntol`), or null.
    val tolHz: Float? = null,
    /// The Tx frequency, Hz (`nftx`), or null. Steers FT8's AP search.
    val txFreqHz: Float? = null,
    val depth: MfskDepth = MfskDepth.DEEP,
    /// Average over periods (`ndepth & 16`; JT65 and Q65). Needs a `period`
    /// on each decode, consecutive ones.
    val averaging: Boolean = false,
    /// JT65 deep search (`ndepth & 32`).
    val deepSearch: Boolean = false,
    /// EME delay (`emedelay`; Q65: the late edge reaches +5.5 s).
    val emeDelay: Boolean = false,
    val station: MfskStation = MfskStation(),
    val qso: MfskQso = MfskQso(),
    val ap: MfskApMode = MfskApMode.OFF,
    val contest: MfskContest = MfskContest.NONE,
) {
    internal companion object {
        // Slot counts of the three arrays that cross JNI — the layout is
        // documented at `read_params` in mfsk_jni.c and must move with it.
        const val FLOATS = 5
        const val INTS = 5
        const val STRINGS = 4

        fun fromArrays(f: FloatArray, v: IntArray, s: Array<String?>): MfskParams =
            MfskParams(
                band = f[0]..f[1],
                // NaN is the ABI's spelling of "unset": 0 Hz is a frequency.
                rxFreqHz = f[2].takeUnless { it.isNaN() },
                tolHz = f[3].takeUnless { it.isNaN() },
                txFreqHz = f[4].takeUnless { it.isNaN() },
                depth = MfskDepth.of(v[0]),
                averaging = v[1] and 1 != 0,
                deepSearch = v[1] and 2 != 0,
                emeDelay = v[1] and 4 != 0,
                ap = MfskApMode.of(v[2]),
                contest = MfskContest.of(v[3]),
                station = MfskStation(s[0] ?: "", s[1] ?: ""),
                qso = MfskQso(s[2] ?: "", s[3] ?: "", MfskQsoProgress.of(v[4])),
            )
    }

    internal fun floatArray() = floatArrayOf(
        band.start, band.endInclusive,
        rxFreqHz ?: Float.NaN, tolHz ?: Float.NaN, txFreqHz ?: Float.NaN,
    )

    internal fun intArray() = intArrayOf(
        depth.code,
        (if (averaging) 1 else 0) or (if (deepSearch) 2 else 0) or (if (emeDelay) 4 else 0),
        ap.code, contest.code, qso.progress.code,
    )

    internal fun stringArray(): Array<String?> =
        arrayOf(station.call, station.grid, qso.hisCall, qso.hisGrid)
}

// ── The library's options ───────────────────────────────────────────

/// An a-priori hypothesis, as the message's own fields in order, beside the
/// QSO-context AP (it wins when both are given).
///
/// [call1] is the message's **first** callsign field — `"CQ"` for a CQ, not
/// the transmitting station — and locks message bits 0-28, [call2] the
/// second and locks 29-57, [grid] locks 58-73, [report] is `"RRR"`, `"RR73"`,
/// `"73"` or a report. A hint locks bits rather than steering a search, so the
/// wrong order removes decodes instead of costing a fraction of a dB. Each
/// field is limited to the ABI's 15 characters.
data class MfskApHint(
    val call1: String = "",
    val call2: String = "",
    val grid: String = "",
    val report: String = "",
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
    /// searches only within [toleranceHz] of [MfskParams.rxFreqHz], so
    /// **without an Rx frequency only the 0 % pass runs**. Costs up to 21
    /// decodes.
    data class Sweep(val step: Int, val toleranceHz: Float) : MfskNoiseBlanker
}

/// How the search is run, in place of the depth's own.
sealed interface MfskStrategy {
    /// The depth's.
    data object Default : MfskStrategy

    /// One pass, no subtraction. Since 0.12.0 FT8 and FT4 subtract by default,
    /// as WSJT-X does.
    data object SinglePass : MfskStrategy

    /// [rounds] rounds of flat successive-interference cancellation. FT8, FT4.
    data class SicRounds(val rounds: Int) : MfskStrategy

    /// FT8's checkpointed passes (`sic_early`). FT8 only.
    data object SicEarly : MfskStrategy
}

/// `0` off, `1` local per-signal equalisation. A property of the *input audio*
/// — it flattens a passband an analogue filter has tilted — not of the search.
enum class MfskEqMode(internal val code: Int) { OFF(0), LOCAL(1) }

/// The accept/reject profile.
enum class MfskStrictness(internal val code: Int) {
    STRICT(0), NORMAL(1),
    /// Deliberately exceeds WSJT-X's own FT8 ceiling; exploratory.
    DEEP(2);

    internal companion object {
        fun of(code: Int): MfskStrictness? = entries.firstOrNull { it.code == code }
    }
}

/// Which messages are kept.
enum class MfskMessageFilter(internal val code: Int) {
    /// The protocol's own message filter.
    DEFAULT(0),
    /// The codec's verdict alone.
    CODEC(1),
}

/// The Q65 fast-fading metric: [b90Ts] is spread bandwidth times symbol period
/// (typical 0.05 near-AWGN, 1.0 moderate, 5+ severe).
data class MfskQ65Fading(val b90Ts: Float, val model: Int = GAUSSIAN) {
    companion object {
        /// Libration-limited EME, WSJT-X's default.
        const val GAUSSIAN = 0
        const val LORENTZIAN = 1
    }
}

/// The library's options beyond the parameter block — `MfskExtras` in the C
/// ABI. Every field's default is "unset", which is the depth's own value, so
/// `MfskExtras()` is what `mfsk_extras_init` writes.
///
/// **An option the mode does not have is refused**, with
/// [MfskUnsupportedException] naming it, when the extras are applied
/// ([MfskDecoder.open], [MfskDecoder.setExtras]) — never dropped at decode
/// time. [MfskDecoder.setExtras] replaces the whole block, so what you leave
/// unset goes back to the depth's value.
data class MfskExtras(
    /// Sync threshold over the depth's. **Not comparable across modes.**
    val syncMin: Float? = null,
    /// Candidate budget over the depth's.
    val maxCand: Int? = null,
    /// OSD over the depth's.
    val osd: Boolean? = null,
    val strictness: MfskStrictness? = null,
    val strategy: MfskStrategy = MfskStrategy.Default,
    val eqMode: MfskEqMode = MfskEqMode.OFF,
    val messageFilter: MfskMessageFilter = MfskMessageFilter.DEFAULT,
    /// FT8's a7 list decoder (`ft8_a7.f90`), fed by the decoder's own decodes
    /// two periods back. Needs a `period` on each decode.
    val a7: Boolean = false,
    /// Half-width of FT8's roofing-filter ("sniper") search around the Rx
    /// frequency, Hz; null is the wide-band search. It matches an *analogue*
    /// roofing filter the operator has narrowed. Needs [Mfsk.CAP_SNIPER].
    val sniperHz: Float? = null,
    val apHint: MfskApHint? = null,
    /// FST4 only.
    val noiseBlanker: MfskNoiseBlanker? = null,
    /// WSPR, JT9, JT65, Q65: how far before the nominal start a frame may
    /// begin, seconds (default the mode's own).
    val tEarlyS: Float? = null,
    /// As [tEarlyS], after the nominal start.
    val tLateS: Float? = null,
    /// As [tEarlyS]: coarse-sync acceptance, 0..1.
    val scoreThreshold: Float? = null,
    /// WSPR: Fano cycles per bit (`wsprd -C`).
    val maxCyclesPerBit: Int? = null,
    /// JT65: Chase trials (`nvec`).
    val chaseTrials: Int? = null,
    /// **Q65 Pileup**: a reply carrying "copied last Tx" matches an AP hint
    /// that names both callsigns. Needs [apHint].
    val pileup: Boolean = false,
    /// Q65 **Max Drift**, spectrum bins `0..=50`; 0 is off. Costs `2*bins+1`
    /// times the plain search.
    val maxDrift: Int = 0,
    /// Q65 fast-fading metric, or null for the plain one.
    val fading: MfskQ65Fading? = null,
) {
    internal companion object {
        // Slot counts of the arrays that cross JNI — the layout is documented
        // at `read_extras` in mfsk_jni.c and must move with it.
        const val FLOATS = 7
        const val INTS = 16
        const val STRINGS = 4

        fun fromArrays(f: FloatArray, v: IntArray, s: Array<String?>): MfskExtras {
            fun nz(x: Float) = x.takeUnless { it.isNaN() }
            return MfskExtras(
                syncMin = nz(f[0]),
                maxCand = v[0].takeIf { it > 0 },
                osd = when (v[1]) { 0 -> false; 1 -> true; else -> null },
                strictness = MfskStrictness.of(v[2]),
                strategy = when (v[3]) {
                    1 -> MfskStrategy.SinglePass
                    2 -> MfskStrategy.SicRounds(v[4])
                    3 -> MfskStrategy.SicEarly
                    else -> MfskStrategy.Default
                },
                eqMode = if (v[5] != 0) MfskEqMode.LOCAL else MfskEqMode.OFF,
                messageFilter = if (v[6] != 0) MfskMessageFilter.CODEC else MfskMessageFilter.DEFAULT,
                a7 = v[7] != 0,
                sniperHz = f[1].takeIf { it > 0f },
                apHint = if (v[8] != 0) {
                    MfskApHint(s[0] ?: "", s[1] ?: "", s[2] ?: "", s[3] ?: "")
                } else null,
                noiseBlanker = when {
                    v[10] != 0 -> MfskNoiseBlanker.Sweep(v[10], f[2])
                    v[9] != 0 -> MfskNoiseBlanker.Percent(v[9])
                    else -> null
                },
                tEarlyS = nz(f[3]), tLateS = nz(f[4]), scoreThreshold = nz(f[5]),
                maxCyclesPerBit = v[11].takeIf { it > 0 },
                chaseTrials = v[12].takeIf { it > 0 },
                pileup = v[13] != 0,
                maxDrift = v[14],
                fading = nz(f[6])?.let { MfskQ65Fading(it, v[15]) },
            )
        }
    }

    internal fun floatArray() = floatArrayOf(
        syncMin ?: Float.NaN, sniperHz ?: 0f,
        (noiseBlanker as? MfskNoiseBlanker.Sweep)?.toleranceHz ?: 0f,
        tEarlyS ?: Float.NaN, tLateS ?: Float.NaN, scoreThreshold ?: Float.NaN,
        fading?.b90Ts ?: Float.NaN,
    )

    internal fun intArray() = intArrayOf(
        maxCand ?: 0,
        when (osd) { null -> -1; false -> 0; true -> 1 },
        strictness?.code ?: -1,
        when (strategy) {
            MfskStrategy.Default -> 0
            MfskStrategy.SinglePass -> 1
            is MfskStrategy.SicRounds -> 2
            MfskStrategy.SicEarly -> 3
        },
        (strategy as? MfskStrategy.SicRounds)?.rounds ?: 0,
        eqMode.code, messageFilter.code, if (a7) 1 else 0,
        if (apHint != null) 1 else 0,
        (noiseBlanker as? MfskNoiseBlanker.Percent)?.percent ?: 0,
        (noiseBlanker as? MfskNoiseBlanker.Sweep)?.step ?: 0,
        maxCyclesPerBit ?: 0, chaseTrials ?: 0,
        if (pileup) 1 else 0, maxDrift,
        fading?.model ?: MfskQ65Fading.GAUSSIAN,
    )

    internal fun stringArray(): Array<String?> =
        arrayOf(apHint?.call1, apHint?.call2, apHint?.grid, apHint?.report)
}

/// The DX station [MfskQ65History.lookup] found — WSJT-X's `dxcall` and `dxgrid`.
data class MfskQ65Dx(val call: String, val grid: String?)

/// One station [MfskQ65Callers] remembers.
data class MfskQ65Caller(
    /// Up to six characters.
    val call: String,
    /// The four-character grid it sent.
    val grid: String,
    /// When it was last heard, Unix seconds, as passed to
    /// [MfskQ65Callers.record].
    val lastHeard: Long,
    /// Its audio frequency then, Hz.
    val freqHz: Int,
)

/// The 100 most recent Q65 decodes and their frequencies — WSJT-X's `q65_hist`.
/// It is how a "Decode Again" with no DX call entered finds the DX station, so
/// the full-AP list can be built without the operator typing the call.
///
/// A [MfskDecoder] does not keep one, so the history is the application's: feed
/// it each decode with [record], ask with [lookup]. **Not thread-safe**; close it when
/// done.
class MfskQ65History : AutoCloseable {
    private var handle: Long = run { Mfsk.modes(); nativeNew() }

    companion object {
        @JvmStatic private external fun nativeNew(): Long
        @JvmStatic private external fun nativeFree(handle: Long)
        @JvmStatic private external fun nativePush(handle: Long, freqHz: Float, message: String)
        @JvmStatic private external fun nativeLen(handle: Long): Int
        @JvmStatic private external fun nativeLookup(handle: Long, rxFreqHz: Float): Array<String?>?
    }

    /// Remember one decode at [freqHz] (tone 0); the 100 most recent are kept.
    fun push(freqHz: Float, message: String) {
        check(handle != 0L) { "history is closed" }
        nativePush(handle, freqHz, message)
    }

    /// Remember every row of a decode, as `q65_decode.f90` calls `q65_hist`
    /// after each one.
    fun record(rows: List<MfskDecode>) {
        for (r in rows) push(r.freqHz, r.text)
    }

    val size: Int get() = if (handle != 0L) nativeLen(handle) else 0

    /// The DX station from the most recent decode within 10 Hz of [rxFreqHz]
    /// whose first word is 3 to 12 characters — so a `CQ ...` decode is passed
    /// over for an older one — or null when nothing qualifies.
    fun lookup(rxFreqHz: Float): MfskQ65Dx? {
        check(handle != 0L) { "history is closed" }
        val v = nativeLookup(handle, rxFreqHz) ?: return null
        return MfskQ65Dx(v[0]!!, v[1])
    }

    override fun close() {
        if (handle != 0L) {
            nativeFree(handle)
            handle = 0L
        }
    }
}

/// The contest caller list — WSJT-X's `q65_hist2`: up to 50 stations that called
/// with a grid, from which the contest full-AP list is built
/// ([MfskDecoder.setQ65Callers] with [MfskContest.GRID_EXCHANGE]). Times are
/// yours (Unix seconds), since the library reads no clock. **Not thread-safe**; close it when done.
class MfskQ65Callers : AutoCloseable {
    private var handle: Long = run { Mfsk.modes(); nativeNew() }

    /// The native handle, for [MfskDecoder.setQ65Callers].
    internal val raw: Long get() {
        check(handle != 0L) { "caller list is closed" }
        return handle
    }

    companion object {
        @JvmStatic private external fun nativeNew(): Long
        @JvmStatic private external fun nativeFree(handle: Long)
        @JvmStatic private external fun nativeRecord(
            handle: Long, freqHz: Float, message: String, now: Long,
        )
        @JvmStatic private external fun nativeExpire(handle: Long, now: Long)
        @JvmStatic private external fun nativeRemove(handle: Long, call: String)
        @JvmStatic private external fun nativeLen(handle: Long): Int
        @JvmStatic private external fun nativeGet(handle: Long, index: Int): Array<String>?
    }

    /// Remember a decode at [freqHz] heard at [now]: a compound call is ignored,
    /// ` R ` is taken out, the second word is the caller and the next four
    /// characters its grid. A known caller is refreshed; a new one is added only
    /// if it sent a grid, the oldest making room once 50 are held.
    fun record(freqHz: Float, message: String, now: Long) {
        nativeRecord(raw, freqHz, message, now)
    }

    /// Drop callers not heard for more than 24 hours. Call before each decode.
    fun expire(now: Long) = nativeExpire(raw, now)

    /// Forget one caller (worked, say).
    fun remove(call: String) = nativeRemove(raw, call)

    val size: Int get() = if (handle != 0L) nativeLen(handle) else 0

    /// The stations, oldest first.
    val callers: List<MfskQ65Caller>
        get() = (0 until size).mapNotNull { i ->
            nativeGet(raw, i)?.let {
                MfskQ65Caller(it[0], it[1], it[2].toLong(), it[3].toInt())
            }
        }

    override fun close() {
        if (handle != 0L) {
            nativeFree(handle)
            handle = 0L
        }
    }
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
    /// Kept for ABI parity. Every slot mode is driven through [MfskDecoder]
    /// now, whether or not it sets this bit.
    const val CAP_DECODE_HANDLE = 1L shl 0
    const val CAP_SNIPER = 1L shl 1
    const val CAP_AP_NARROW = 1L shl 2
    const val CAP_AP_WIDEBAND = 1L shl 3
    const val CAP_SIC_ROUNDS = 1L shl 4
    const val CAP_SIC_EARLY = 1L shl 5
    const val CAP_OSD = 1L shl 6
    const val CAP_EQ_MODE = 1L shl 7
    const val CAP_STRICTNESS = 1L shl 8
    /// [MfskDecoder.setBudget] is honoured: every mode with a decoder.
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
    /// sub-mode, no other. See [MfskExtras.noiseBlanker].
    const val CAP_NOISE_BLANKER = 1L shl 16
    /// The transmit frequency steers the a-priori search: FT8 only. See
    /// [MfskParams.txFreqHz].
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

    /// `mode`'s published defaults — the starting point every [MfskParams]
    /// should come from, then `copy` what differs:
    ///
    /// ```kotlin
    /// val p = Mfsk.defaultParams(ft8).copy(rxFreqHz = 1500f, txFreqHz = 1500f)
    /// MfskDecoder.open(ft8, p).use { ... }
    /// ```
    ///
    /// Throws [MfskException] if the mode has no slot decoder in this build
    /// (MSK144, JTTY) or is not a mode at all.
    fun defaultParams(mode: Int): MfskParams {
        val f = FloatArray(MfskParams.FLOATS)
        val v = IntArray(MfskParams.INTS)
        val s = arrayOfNulls<String>(MfskParams.STRINGS)
        nativeParamsInit(mode, f, v, s)
        return MfskParams.fromArrays(f, v, s)
    }

    /// What `mfsk_extras_init` writes: every option unset. It equals
    /// `MfskExtras()`; the library is asked so that a change to the ABI's
    /// sentinels shows up as a test failure rather than a silent difference.
    internal fun libraryDefaultExtras(): MfskExtras {
        val f = FloatArray(MfskExtras.FLOATS)
        val v = IntArray(MfskExtras.INTS)
        val s = arrayOfNulls<String>(MfskExtras.STRINGS)
        nativeExtrasInit(f, v, s)
        return MfskExtras.fromArrays(f, v, s)
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

    // ── Transmit ────────────────────────────────────────────────────

    /// Pack `call1 call2 report` into the 77-bit message (one bit per byte).
    /// Throws [MfskException] if it does not fit the format.
    fun pack77(call1: String, call2: String, report: String): ByteArray {
        modes() // loads the native library
        return nativePack77(call1, call2, report)
    }

    /// Pack a **type 4** message: a non-standard call, with the standard one
    /// carried as a hash, so a receiver that has not heard the standard call
    /// reads `<...>`. [report] may be null; [isCq] makes it a CQ.
    fun pack77Type4(nonstdCall: String, stdCall: String, report: String?, isCq: Boolean): ByteArray {
        modes()
        return nativePack77Type4(nonstdCall, stdCall, report, isCq)
    }

    /// Unpack a 77-bit message to text. A `<...>` hash stays `<...>`; ask a
    /// [MfskDecoder.unpack77] that has heard the call to resolve it.
    fun unpack77(message77: ByteArray): String {
        modes()
        return nativeUnpack77(message77)
    }

    /// Pack `call1 call2 report` and synthesise a frame at 12 kHz.
    ///
    /// Throws if the message does not fit the format or the mode has no
    /// tone stage — `mfsk_symbol_count` returning 0 is how the ABI says
    /// which modes those are.
    fun synthesize(mode: Int, call1: String, call2: String, report: String, freqHz: Float)
        : ShortArray = synthesize(mode, pack77(call1, call2, report), freqHz)

    /// Synthesise a packed 77-bit message at 12 kHz. Modes with a tone stage:
    /// FT8, FT4, FST4.
    fun synthesize(mode: Int, message77: ByteArray, freqHz: Float): ShortArray {
        modes()
        return nativeSynthesize(mode, message77, freqHz)
    }

    /// Pack `call1 call2 grid-or-report` and synthesise a Q65 frame at 12 kHz.
    /// [copiedLastTx] sets Pileup's "copied last Tx" flag, the spare 78th bit.
    fun synthesizeQ65(
        mode: Int, call1: String, call2: String, gridOrReport: String, freqHz: Float,
        copiedLastTx: Boolean = false,
    ): FloatArray = nativeSynthesizeQ65(mode, call1, call2, gridOrReport, copiedLastTx, freqHz)

    /// A standard JT9 or JT65 message (`call1 call2 grid-or-report`) as f32
    /// PCM at 12 kHz.
    fun synthesizeJt(mode: Int, call1: String, call2: String, gridOrReport: String, freqHz: Float)
        : FloatArray = nativeSynthesizeJt(mode, call1, call2, gridOrReport, freqHz)

    /// A Type-1 WSPR message (`call grid power_dbm`) as f32 PCM at 12 kHz.
    fun synthesizeWspr(call: String, grid: String, powerDbm: Int, freqHz: Float): FloatArray {
        modes()
        return nativeSynthesizeWspr(call, grid, powerDbm, freqHz)
    }

    @JvmStatic private external fun nativeAbiVersion(): Int
    @JvmStatic private external fun nativeModeCount(): Int
    @JvmStatic private external fun nativeModeAt(index: Int): Int
    @JvmStatic private external fun nativeModeName(mode: Int): String?
    @JvmStatic private external fun nativeModeCaps(mode: Int): Long
    @JvmStatic private external fun nativeParamsInit(
        mode: Int, floats: FloatArray, ints: IntArray, strings: Array<String?>,
    )
    @JvmStatic private external fun nativeExtrasInit(
        floats: FloatArray, ints: IntArray, strings: Array<String?>,
    )
    @JvmStatic private external fun nativeModeInfo(mode: Int): IntArray
    @JvmStatic private external fun nativeConfigureRuntime(threads: Int, stackBytes: Int): Int
    @JvmStatic private external fun nativeThreadCount(): Int
    @JvmStatic private external fun nativePack77(a: String, b: String, c: String): ByteArray
    @JvmStatic private external fun nativePack77Type4(
        nonstd: String, std: String, report: String?, isCq: Boolean,
    ): ByteArray
    @JvmStatic private external fun nativeUnpack77(message77: ByteArray): String
    @JvmStatic private external fun nativeSynthesize(
        mode: Int, message77: ByteArray, freqHz: Float,
    ): ShortArray
    @JvmStatic private external fun nativeSynthesizeQ65(
        mode: Int, a: String, b: String, c: String, flagged: Boolean, freqHz: Float,
    ): FloatArray
    @JvmStatic private external fun nativeSynthesizeJt(
        mode: Int, a: String, b: String, c: String, freqHz: Float,
    ): FloatArray
    @JvmStatic private external fun nativeSynthesizeWspr(
        call: String, grid: String, powerDbm: Int, freqHz: Float,
    ): FloatArray
}

// ── The decoder ─────────────────────────────────────────────────────

/// How a [MfskStream] clock reading was taken.
enum class MfskClockChange {
    /// The first reading: the clock is anchored.
    FIRST,
    /// The clock moved towards the reading by at most its slew limit (400 ppm);
    /// open slots are unaffected.
    SLEWED,
    /// The reading was more than a second away: the clock re-anchored and the
    /// slot that straddled the jump is dropped.
    STEPPED;

    internal companion object {
        fun of(code: Int) = entries[code.coerceIn(0, entries.size - 1)]
    }
}

/// What [MfskDecoder.decodeStream] decoded: the stream's ready slot, with its
/// index on the UTC grid.
data class MfskSlotDecode(
    /// The slot's index on the mode's UTC grid (UTC `period * T` with a clock
    /// set, counted from the first sample without one). Also what the decoder
    /// was given as the period.
    val period: Long,
    /// UTC of the slot start, ns since the Unix epoch, or null when no clock
    /// was set on the stream.
    val slotStartUtcNs: Long?,
    val rows: List<MfskDecode>,
)

/// A decoder: one persistent decoder of one mode, driven once per period like
/// WSJT-X's own (`jt9 -s`). Serves FT8, FT4, every FST4 sub-mode, WSPR, JT9,
/// JT65 and every Q65 sub-mode through the same calls.
///
/// It owns what upstream keeps across periods and nothing else: the callsign
/// hash table (never shared with another decoder, as upstream's is not), FT8's
/// a7 list, Q65's and JT65's averages, WSPR's call table. A `<...>` reference
/// cannot be expanded without having seen the callsign, and a table that does
/// not outlive the slot that populated it is worth nothing — which is why this
/// is a handle and not a function.
///
/// **Single-threaded**, deliberately: one decode at a time on a decoder, one
/// decoder per thread. Concurrent decodes on separate decoders are supported.
///
/// ```kotlin
/// val p = Mfsk.defaultParams(ft8).copy(rxFreqHz = 1500f)
/// MfskDecoder.open(ft8, p, MfskExtras(a7 = true)).use { dec ->
///     for ((period, audio) in periods) for (row in dec.decode(audio, period)) show(row)
/// }
/// ```
class MfskDecoder private constructor(
    private var handle: Long,
    /// The `MfskMode` this decodes.
    val mode: Int,
    private val owned: Boolean,
) : AutoCloseable {

    companion object {
        /// `period` for a decode whose slot index is not known; the decoder
        /// then leaves its period-to-period state (a7, averaging) alone.
        const val PERIOD_NONE = Long.MIN_VALUE

        /// Open a decoder for `mode`; [params] and [extras] null are the mode's
        /// defaults (`Mfsk.defaultParams(mode)`, nothing set).
        ///
        /// **An option the mode does not support fails here** with
        /// [MfskUnsupportedException] naming the mode and the option, rather
        /// than being dropped at decode time. A value out of range is an
        /// [MfskInvalidArgException]; a mode this build lacks, an
        /// [MfskUnknownModeException].
        @JvmStatic
        fun open(mode: Int, params: MfskParams? = null, extras: MfskExtras? = null): MfskDecoder =
            MfskDecoder(
                nativeOpen(
                    mode,
                    params?.floatArray(), params?.intArray(), params?.stringArray(),
                    extras?.floatArray(), extras?.intArray(), extras?.stringArray(),
                ),
                mode, owned = true,
            )

        /// A decoder the IQ receiver owns, for configuring only.
        internal fun borrowed(handle: Long, mode: Int) = MfskDecoder(handle, mode, owned = false)

        @JvmStatic private external fun nativeOpen(
            mode: Int,
            pf: FloatArray?, pi: IntArray?, ps: Array<String?>?,
            ef: FloatArray?, ei: IntArray?, es: Array<String?>?,
        ): Long
        @JvmStatic private external fun nativeClose(handle: Long, callbackCtx: Long, budgetCtx: Long)
        @JvmStatic private external fun nativeLastError(handle: Long): String?
        @JvmStatic private external fun nativeSetParams(
            handle: Long, pf: FloatArray, pi: IntArray, ps: Array<String?>,
        )
        @JvmStatic private external fun nativeSetExtras(
            handle: Long, ef: FloatArray, ei: IntArray, es: Array<String?>,
        )
        @JvmStatic private external fun nativeSetQ65Callers(handle: Long, callers: Long)
        @JvmStatic private external fun nativeClear(handle: Long)
        @JvmStatic private external fun nativeAddCallsign(handle: Long, call: String)
        @JvmStatic private external fun nativeSetOnDecode(
            handle: Long, listener: MfskDecodeListener?, oldCtx: Long,
        ): Long
        @JvmStatic private external fun nativeSetBudget(
            handle: Long, check: MfskBudgetCheck?, oldCtx: Long,
        ): Long
        @JvmStatic private external fun nativeLastBudget(handle: Long): IntArray
        @JvmStatic private external fun nativeDeliveryIsExact(handle: Long): Boolean
        @JvmStatic private external fun nativePrefixPoints(handle: Long): IntArray
        @JvmStatic private external fun nativeDecodePrefixI16(
            handle: Long, samples: ShortArray, sampleRate: Int, period: Long,
        ): Array<MfskDecode>
        @JvmStatic private external fun nativeDecodePrefixF32(
            handle: Long, samples: FloatArray, sampleRate: Int, period: Long,
        ): Array<MfskDecode>
        @JvmStatic private external fun nativeDecodeI16(
            handle: Long, samples: ShortArray, sampleRate: Int, period: Long,
        ): Array<MfskDecode>
        @JvmStatic private external fun nativeDecodeF32(
            handle: Long, samples: FloatArray, sampleRate: Int, period: Long,
        ): Array<MfskDecode>
        @JvmStatic private external fun nativeCopyInfo(handle: Long, index: Int): ByteArray
        @JvmStatic private external fun nativeUnpack77(handle: Long, message77: ByteArray): String
        @JvmStatic private external fun nativeDecodeStream(
            handle: Long, stream: Long, meta: LongArray,
        ): Array<MfskDecode>?
    }

    private fun live(): Long {
        check(handle != 0L) { "decoder is closed" }
        return handle
    }

    private fun owner(what: String): Long {
        check(owned) { "$what: this decoder belongs to an IQ receiver, which decodes with it" }
        return live()
    }

    /// The last error recorded **on this decoder**, or null. Unlike the
    /// library's global one it is not thread-local, so a coroutine that hopped
    /// threads still reads it. The exceptions this class throws carry it.
    val lastError: String? get() = nativeLastError(live())

    /// Change the parameter block between periods, as the GUI rewrites it
    /// before each one. What the decoder carries between periods is kept.
    fun setParams(params: MfskParams) {
        nativeSetParams(live(), params.floatArray(), params.intArray(), params.stringArray())
    }

    /// Replace the library's options. What the block leaves unset goes back to
    /// the depth's value; an option the mode lacks throws
    /// [MfskUnsupportedException] and nothing changes. What the decoder carries
    /// between periods is kept.
    fun setExtras(extras: MfskExtras) {
        nativeSetExtras(live(), extras.floatArray(), extras.intArray(), extras.stringArray())
    }

    /// Q65 only: the contest callers heard (`q65_hist2`), so that with
    /// [MfskContest.GRID_EXCHANGE] they join the full-AP list. The list is
    /// **copied** at this call: record more callers, then set it again. Null
    /// removes it. Survives [setExtras].
    fun setQ65Callers(callers: MfskQ65Callers?) {
        nativeSetQ65Callers(live(), callers?.raw ?: 0L)
    }

    /// Forget everything the decoder carries between periods (WSJT-X's "Clear
    /// Avg" and `ndepth & 128`): the hash table, a7, averages.
    fun clear() = nativeClear(live())

    /// Teach the decoder a callsign, so a later period's `<...>` reference to
    /// it resolves. Decoded messages populate the table by themselves; this is
    /// for calls known from outside (a band map, a log). Throws
    /// [MfskUnsupportedException] for a mode whose messages carry no hashed
    /// calls (WSPR, say).
    fun addCallsign(call: String) = nativeAddCallsign(live(), call)

    /// Decode one period of 16-bit PCM.
    ///
    /// [period] is the period's index on the UTC grid (`utc_seconds / T`), or
    /// null when it is not known; a decoder keeps what upstream keeps across
    /// periods, and the parts that need consecutive periods (a7, averaging)
    /// use them only when the index is given. [sampleRate] other than 12 000
    /// is resampled.
    ///
    /// [onRow], if given, sees each row as it is found for this call only;
    /// see [onDecode] for the threading rules. The returned list stays
    /// authoritative.
    fun decode(
        samples: ShortArray,
        period: Long? = null,
        sampleRate: Int = 12_000,
        onRow: MfskDecodeListener? = null,
    ): List<MfskDecode> = withRowListener(onRow) {
        nativeDecodeI16(owner("decode"), samples, sampleRate, period ?: PERIOD_NONE).toList()
    }

    /// Decode one period of 32-bit float PCM, **at any level**: the modes
    /// whose engines work in float (WSPR, JT9, JT65, Q65) never see 16 bits,
    /// and FT8, FT4 and FST4, which take 16-bit audio as WSJT-X does, get it
    /// scaled to a fixed level, so a caller never picks one.
    fun decode(
        samples: FloatArray,
        period: Long? = null,
        sampleRate: Int = 12_000,
        onRow: MfskDecodeListener? = null,
    ): List<MfskDecode> = withRowListener(onRow) {
        nativeDecodeF32(owner("decode"), samples, sampleRate, period ?: PERIOD_NONE).toList()
    }

    /// Decode the period so far, keeping what this period has already found
    /// (#572). Call it as audio arrives with **every sample of the period
    /// received up to now** and the period's index; the decoder infers the
    /// stage from the length. FT8 returns checkpoint A's rows at 141 696
    /// samples (~11.8 s, [MfskStage.EARLY]), nothing at 162 432, and the
    /// period's complete set at 180 000 — the rows [decode] gives for the same
    /// audio. Other calls, and every call of a mode with no early decode before
    /// the whole period, return nothing. [onDecode] sees each row once across
    /// the period. A null period makes it a plain [decode].
    fun decodePrefix(
        samples: ShortArray,
        period: Long?,
        sampleRate: Int = 12_000,
        onRow: MfskDecodeListener? = null,
    ): List<MfskDecode> = withRowListener(onRow) {
        nativeDecodePrefixI16(owner("decodePrefix"), samples, sampleRate, period ?: PERIOD_NONE).toList()
    }

    /// [decodePrefix] for float PCM at any level; the first prefix of a period
    /// sets the gain for the rest of it.
    fun decodePrefix(
        samples: FloatArray,
        period: Long?,
        sampleRate: Int = 12_000,
        onRow: MfskDecodeListener? = null,
    ): List<MfskDecode> = withRowListener(onRow) {
        nativeDecodePrefixF32(owner("decodePrefix"), samples, sampleRate, period ?: PERIOD_NONE).toList()
    }

    private inline fun <T> withRowListener(l: MfskDecodeListener?, body: () -> T): T {
        if (l == null) return body()
        val previous = listener
        onDecode(l)
        try {
            return body()
        } finally {
            onDecode(previous)
        }
    }

    /// The FEC information bits ([MfskDecode.infoBits] of them, one per byte)
    /// of the `index`-th row of the **last** decode. They are not a row field
    /// because only a caller doing subtraction or persistence wants them.
    /// Throws [MfskInvalidArgException] past the last row.
    fun copyInfo(index: Int): ByteArray = nativeCopyInfo(owner("copyInfo"), index)

    /// The prefix lengths, in 12 kHz samples, at which [decodePrefix] does
    /// work before the whole period under the current settings:
    /// `[141696, 162432]` for FT8 at Normal or Deep depth, empty otherwise.
    /// Hand them to [MfskStream.setPrefixPoints]; ask again after changing
    /// the params or extras.
    val prefixPoints: IntArray get() = nativePrefixPoints(live())

    /// Decode the stream's ready slot, with the slot's own index as the
    /// period, or return null when no slot is ready yet — so this can be
    /// polled in place of [MfskStream.slotReady]. The stream has to be for the
    /// same mode. On a stream with [MfskStream.setPrefixPoints] a prefix is
    /// decoded as [decodePrefix] would: checkpoint A's rows come back at
    /// ~11.8 s with [MfskStage.EARLY].
    fun decodeStream(stream: MfskStream): MfskSlotDecode? {
        val meta = LongArray(2)
        val rows = nativeDecodeStream(owner("decodeStream"), stream.raw, meta) ?: return null
        return MfskSlotDecode(
            period = meta[0],
            slotStartUtcNs = meta[1].takeIf { it != 0L },
            rows = rows.toList(),
        )
    }

    /// As [Mfsk.unpack77], resolving `<...>` calls against this decoder's own
    /// callsign table.
    fun unpack77(message77: ByteArray): String = nativeUnpack77(live(), message77)

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
        // Allowed on a decoder an IQ receiver lends too: its rows, early ones
        // included, reach the listener while the receiver's `push` runs.
        callbackCtx = nativeSetOnDecode(live(), listener, callbackCtx)
        this.listener = listener
    }

    private var listener: MfskDecodeListener? = null

    /// Poll `check` during every subsequent decode; returning false
    /// stops the search and returns what was found. Pass null to remove
    /// the budget.
    ///
    /// **The triage sweep is a floor**: it is never gated, so a budget
    /// shorter than it returns nothing *and still spends that time*.
    /// Measured at ~13 ms of a ~28 ms FT8 decode. `maxCand` is the knob
    /// that moves the floor.
    ///
    /// Every mode with a decoder takes one ([Mfsk.CAP_BUDGET]); WSPR, JT9,
    /// JT65 and Q65 report only [MfskBudgetReport.exhausted].
    fun setBudget(check: MfskBudgetCheck?) {
        budgetCtx = nativeSetBudget(live(), check, budgetCtx)
    }

    /// What the budget cut short on the **last** decode.
    val lastBudget: MfskBudgetReport
        get() {
            val v = nativeLastBudget(live())
            return MfskBudgetReport(
                exhausted = v[0] != 0,
                candidatesSkipped = v[1],
                stagesRun = v[2],
                cutAtSync = if (v[3] >= 0) v[3] else null,
                // Int.MIN_VALUE is the "absent" spelling: an int array
                // cannot carry the NaN the C struct uses.
                cutAtScore = if (v[4] != Int.MIN_VALUE) v[4] / 1_000_000.0f else null,
                rowsSubtracted = v[5],
            )
        }

    /// Whether a decode with the current mode, depth and extras delivers
    /// exactly the rows it returns, once each and in order, to
    /// [onDecode] (`STREAMING.md` §3a). False is completion order with a
    /// transient duplicate possible (§3b): FT8's single pass and sniper, FT4
    /// at [MfskDepth.FAST], FST4, WSPR. Pair by [MfskDecode.delivery] either
    /// way. Ask again after changing the parameters or the extras.
    val deliveryIsExact: Boolean
        get() = nativeDeliveryIsExact(live())

    override fun close() {
        // A decoder an IQ receiver lends is the receiver's to close: closing
        // it here does nothing, and its listener and budget stay installed
        // until the channel is removed (`releaseLent`).
        if (handle != 0L && owned) {
            // The native side clears the callback and the budget before
            // closing and then frees their contexts: the decoder holds raw
            // pointers to them, so the order is not decorative.
            nativeClose(handle, callbackCtx, budgetCtx)
            callbackCtx = 0L
            budgetCtx = 0L
            handle = 0L
        }
    }

    /// For [MfskIqReceiver], before it frees the C decoder this lends: take
    /// the listener and the budget off it and free their contexts, then
    /// refuse further use.
    internal fun releaseLent() {
        if (handle == 0L || owned) return
        if (callbackCtx != 0L) nativeSetOnDecode(handle, null, callbackCtx)
        if (budgetCtx != 0L) nativeSetBudget(handle, null, budgetCtx)
        listener = null
        callbackCtx = 0L
        budgetCtx = 0L
        handle = 0L
    }

    /// Opaque pointer to the shim's per-decoder callback state — the
    /// listener's global ref and the cached IDs. Owned here so there is
    /// exactly one, and freed by [close].
    private var callbackCtx: Long = 0L

    /// Same, for the budget predicate.
    private var budgetCtx: Long = 0L
}

// ── Streams ─────────────────────────────────────────────────────────

/// A capture stream for one slot mode: audio goes in as it arrives, in chunks
/// of any size, and whole slots come out cut on the mode's UTC grid.
///
/// With no clock set ([setTime]) the grid free-runs from the first sample,
/// right for replaying a recording; with one, a slot starts on its own
/// boundary and a drifting clock moves the boundary by milliseconds, losing no
/// slot. At most one completed slot waits; a newer one replaces it
/// ([dropped] counts those).
///
/// Decode the ready slot with [MfskDecoder.decodeStream], or take its audio
/// with [takeSlot]. Single-threaded; close it when done.
class MfskStream private constructor(private var handle: Long, val mode: Int) : AutoCloseable {

    companion object {
        /// Open a stream for `mode`, accepting 16-bit audio at `sampleRate`
        /// (anything but 12 000 Hz is resampled, linearly). Throws
        /// [MfskUnsupportedException] for a mode that is not cut into slots.
        @JvmStatic
        fun open(mode: Int, sampleRate: Int = 12_000): MfskStream {
            Mfsk.modes() // loads the native library
            return MfskStream(nativeOpen(mode, sampleRate), mode)
        }

        @JvmStatic private external fun nativeOpen(mode: Int, sampleRate: Int): Long
        @JvmStatic private external fun nativeClose(handle: Long)
        @JvmStatic private external fun nativePushI16(handle: Long, samples: ShortArray)
        @JvmStatic private external fun nativePushF32(handle: Long, samples: FloatArray)
        @JvmStatic private external fun nativePosition(handle: Long): Long
        @JvmStatic private external fun nativeSetTime(handle: Long, utcNs: Long, atSample: Long): Int
        @JvmStatic private external fun nativeSlotReady(handle: Long): Boolean
        @JvmStatic private external fun nativeSlotIsWhole(handle: Long): Boolean
        @JvmStatic private external fun nativeSetPrefixPoints(handle: Long, points: IntArray)
        @JvmStatic private external fun nativeDropped(handle: Long): Long
        @JvmStatic private external fun nativeTakeSlot(handle: Long, cap: Int, meta: LongArray): ShortArray?
        @JvmStatic private external fun nativeClear(handle: Long)
    }

    internal val raw: Long
        get() {
            check(handle != 0L) { "stream is closed" }
            return handle
        }

    fun push(samples: ShortArray) = nativePushI16(raw, samples)

    /// Float PCM, nominally -1.0..1.0.
    fun push(samples: FloatArray) = nativePushF32(raw, samples)

    /// 12 kHz samples taken in so far: the stream's clock, to pass as
    /// `atSample` to [setTime].
    val position: Long get() = nativePosition(raw)

    /// The stream's sample [atSample] (12 kHz, as [position] counts) was at UTC
    /// [utcNs] (ns since the Unix epoch). Call it as often as you have a
    /// reading: the stream follows the readings at up to 400 ppm, so noisy
    /// readings and a drifting clock move slot boundaries by milliseconds and
    /// lose nothing.
    fun setTime(utcNs: Long, atSample: Long): MfskClockChange =
        MfskClockChange.of(nativeSetTime(raw, utcNs, atSample))

    /// Whether a slot is waiting: a completed one, or with [setPrefixPoints]
    /// the slot so far.
    val slotReady: Boolean get() = nativeSlotReady(raw)

    /// Whether the waiting slot is whole rather than a prefix; false when none
    /// is waiting.
    val slotIsWhole: Boolean get() = nativeSlotIsWhole(raw)

    /// Early decode, off by default: from the next slot, the stream also makes
    /// the slot so far ready at each of these 12 kHz sample counts, then the
    /// whole slot. Pass [MfskDecoder.prefixPoints] and decode every slot with
    /// [MfskDecoder.decodeStream]. An empty array turns it off. Opt-in because
    /// a [takeSlot] caller would otherwise get short slots.
    fun setPrefixPoints(points: IntArray) = nativeSetPrefixPoints(raw, points)

    /// Completed slots a newer one replaced before they were taken.
    val dropped: Long get() = nativeDropped(raw)

    /// Take the waiting slot's audio (12 kHz) with its period and UTC start, or
    /// null if none is ready. Prefer [MfskDecoder.decodeStream], which does not
    /// copy it out and back in.
    fun takeSlot(): Triple<ShortArray, Long, Long?>? {
        val meta = LongArray(2)
        val audio = nativeTakeSlot(raw, Mfsk.modeInfo(mode).slotSamples12k, meta) ?: return null
        return Triple(audio, meta[0], meta[1].takeIf { it != 0L })
    }

    /// Drop the waiting slot and the one being cut, keeping the clock.
    fun clear() = nativeClear(raw)

    override fun close() {
        if (handle != 0L) {
            nativeClose(handle)
            handle = 0L
        }
    }
}

// ── Wideband IQ ─────────────────────────────────────────────────────

/// `mfsk_iq_open`'s sample format.
enum class MfskIqFormat(internal val code: Int) {
    /// `f32` I, `f32` Q, little-endian.
    CF32(0),
    /// `i16` I, `i16` Q, full scale 32768.
    CS16(1),
    /// `i8` I, `i8` Q, full scale 128 (HackRF).
    CS8(2),
    /// `u8` I, `u8` Q, 128 = zero (RTL-SDR).
    CU8(3),
    /// 24-bit signed I, Q.
    CS24(4),
}

enum class MfskIqChannelizer(internal val code: Int) {
    /// One filter chain per channel from the input rate. The default, and the
    /// cheapest for up to about four channels.
    DIRECT(0),
    /// A polyphase filter bank shared by every channel: a fixed cost of about
    /// three direct channels, then about a quarter of one per channel.
    PFB(1),
}

/// One decode out of [MfskIqReceiver.poll]: the row of a channel of a wideband
/// IQ stream, with the absolute RF frequency and where its slot started.
class MfskIqDecode internal constructor(
    /// The handle [MfskIqReceiver.addChannel] returned.
    val channel: Int,
    /// The channel's concrete mode.
    val mode: Int,
    val text: String,
    /// RF frequency of tone 0, Hz: the channel's dial plus [freqHz].
    val absFreqHz: Double,
    /// Audio frequency of tone 0 within the channel, Hz.
    val freqHz: Float,
    val dtSec: Float,
    val snrDb: Float,
    /// The slot's index on the mode's UTC grid (counted from sample 0 without
    /// a clock).
    val period: Long,
    /// Index (of the IQ stream, complex samples) the slot started at.
    val slotStartSample: Long,
    hasUtc: Boolean,
    utcNs: Long,
    /// As [MfskDecode.syncScore]: null where the mode reports none.
    val syncScore: Float?,
    /// As [MfskDecode.syncCv].
    val syncCv: Float?,
    /// As [MfskDecode.hardErrors].
    val hardErrors: Int?,
    /// As [MfskDecode.pass]. Protocol-private.
    val pass: Int,
    /// As [MfskDecode.hashResolved].
    val hashResolved: Boolean,
    /// As [MfskDecode.copiedLastTx].
    val copiedLastTx: Boolean,
    /// As [MfskDecode.key]: compare rows by this and [freqHz], not by text.
    val key: String,
    val keyBits: Int,
    /// As [MfskDecode.delivery], for a listener set on the channel's decoder.
    val delivery: Int?,
    /// [MfskStage.EARLY] for a row found before the slot was whole (FT8's
    /// checkpoint A, ~11.8 s; see [MfskIqReceiver.setEarly]),
    /// [MfskStage.FINAL] for one the whole slot found, null from a plain
    /// decode (early decode off).
    val stage: MfskStage? = null,
) {
    /// UTC of the slot start, ns since the Unix epoch, or null on a
    /// free-running grid.
    val slotStartUtcNs: Long? = if (hasUtc) utcNs else null

    override fun toString() =
        "MfskIqDecode(ch=$channel ${Mfsk.modeName(mode)} ${"%.1f".format(absFreqHz)} Hz '$text')"
}

/// A wideband IQ receiver: one IQ stream, any number of channels, each with
/// its own [MfskDecoder] — the SDR skimmer's shape.
///
/// Push IQ bytes; every slot the push completes is decoded before it returns,
/// and what was found waits for [poll]. **Single-threaded**; call [push] off
/// the UI thread. Close it when done.
class MfskIqReceiver private constructor(private var handle: Long) : AutoCloseable {

    companion object {
        /// Open a receiver for an IQ stream of [sampleRate] complex samples per
        /// second (12 000 or more), [centerHz] the RF frequency of DC, [iqSwap]
        /// when I and Q are exchanged (sound-card IQ often is).
        @JvmStatic
        fun open(
            sampleRate: Int, centerHz: Double,
            format: MfskIqFormat = MfskIqFormat.CS16,
            iqSwap: Boolean = false,
            channelizer: MfskIqChannelizer = MfskIqChannelizer.DIRECT,
        ): MfskIqReceiver {
            Mfsk.modes()
            return MfskIqReceiver(
                nativeOpen(sampleRate, centerHz, format.code, iqSwap, channelizer.code),
            )
        }

        @JvmStatic private external fun nativeOpen(
            sampleRate: Int, centerHz: Double, format: Int, iqSwap: Boolean, channelizer: Int,
        ): Long
        @JvmStatic private external fun nativeClose(handle: Long)
        @JvmStatic private external fun nativeAddChannel(
            handle: Long, dialHz: Double, mode: Int,
            pf: FloatArray?, pi: IntArray?, ps: Array<String?>?,
            ef: FloatArray?, ei: IntArray?, es: Array<String?>?,
        ): Int
        @JvmStatic private external fun nativeChannelDecoder(handle: Long, channel: Int): Long
        @JvmStatic private external fun nativeChannelState(handle: Long, channel: Int): Int
        @JvmStatic private external fun nativeRemoveChannel(handle: Long, channel: Int)
        @JvmStatic private external fun nativeSetTime(handle: Long, utcNs: Long, atSample: Long): Int
        @JvmStatic private external fun nativeRetune(handle: Long, centerHz: Double): IntArray
        @JvmStatic private external fun nativeGap(handle: Long, lost: Long)
        @JvmStatic private external fun nativeSetEarly(handle: Long, channel: Int, on: Boolean)
        @JvmStatic private external fun nativePush(handle: Long, data: ByteArray, length: Int)
        @JvmStatic private external fun nativeSamplesIn(handle: Long): Long
        @JvmStatic private external fun nativePending(handle: Long): Int
        @JvmStatic private external fun nativePoll(handle: Long): Array<MfskIqDecode>
    }

    private fun live(): Long {
        check(handle != 0L) { "receiver is closed" }
        return handle
    }

    /// Add a channel whose dial (audio 0 Hz) is [dialHz], carrying [mode], decoded
    /// with its own decoder opened from [params] and [extras] (null: the mode's
    /// defaults). Returns the channel handle rows carry. Throws
    /// [MfskInvalidArgException] if the channel cannot be placed (DC inside its
    /// 0-6 kHz audio window, or the window outside the IQ band) or the mode is
    /// not one the receiver carries, [MfskUnsupportedException] for an option
    /// the mode lacks.
    fun addChannel(
        dialHz: Double, mode: Int, params: MfskParams? = null, extras: MfskExtras? = null,
    ): Int {
        val ch = nativeAddChannel(
            live(), dialHz, mode,
            params?.floatArray(), params?.intArray(), params?.stringArray(),
            extras?.floatArray(), extras?.intArray(), extras?.stringArray(),
        )
        modes[ch] = mode
        return ch
    }

    /// The channel's decoder, for the calls that configure one: [MfskDecoder.setParams],
    /// [MfskDecoder.setExtras], [MfskDecoder.addCallsign], [MfskDecoder.unpack77],
    /// [MfskDecoder.setQ65Callers], [MfskDecoder.clear], and
    /// [MfskDecoder.onDecode] / [MfskDecoder.setBudget]: the listener sees the
    /// channel's rows as [push] finds them, early ones included, and the
    /// budget bounds each decode [push] runs. **Borrowed**: closing it does
    /// nothing, it dies with the channel (or the receiver), and it refuses to
    /// decode — the receiver does. The same object each time for a channel.
    /// Null if there is no such channel.
    fun channelDecoder(channel: Int): MfskDecoder? {
        lent[channel]?.let { return it }
        val h = nativeChannelDecoder(live(), channel)
        if (h == 0L) return null
        return MfskDecoder.borrowed(h, channelMode(channel)).also { lent[channel] = it }
    }

    /// The decoders [channelDecoder] handed out, one per channel, so a
    /// listener or budget set on one is released before the C decoder goes.
    private val lent = HashMap<Int, MfskDecoder>()

    private val modes = HashMap<Int, Int>()
    private fun channelMode(channel: Int) = modes[channel] ?: -1

    /// Whether a channel is being received: true, or false when a retune left
    /// its audio window outside the band (it keeps its dial and decoder and
    /// resumes when a later retune brings it back). Null if there is no such
    /// channel.
    fun isActive(channel: Int): Boolean? = when (nativeChannelState(live(), channel)) {
        0 -> true
        1 -> false
        else -> null
    }

    fun removeChannel(channel: Int) {
        val h = live()
        // The listener's context is freed only once the C side cannot call it.
        lent.remove(channel)?.releaseLent()
        nativeRemoveChannel(h, channel)
        modes.remove(channel)
    }

    /// The stream's complex sample [atSample] (as [samplesIn] counts) was at
    /// UTC [utcNs]. Call it as often as you have a reading; see
    /// [MfskStream.setTime].
    fun setTime(utcNs: Long, atSample: Long): MfskClockChange =
        MfskClockChange.of(nativeSetTime(live(), utcNs, atSample))

    /// The tuner moved to [centerHz]: every channel that still fits is
    /// re-placed, one that no longer fits is paused, and the open slots are
    /// dropped. Returns (paused, resumed) channel counts.
    fun retune(centerHz: Double): Pair<Int, Int> {
        val v = nativeRetune(live(), centerHz)
        return v[0] to v[1]
    }

    /// [lost] samples never arrived: the clock advances past them and the open
    /// slots are dropped.
    fun gap(lost: Long) = nativeGap(live(), lost)

    /// Decode a channel early, or not. On (the default): an FT8 channel at
    /// Normal or Deep depth also decodes the slot so far at ~11.8 s, so those
    /// rows reach the channel decoder's listener and [poll] before the slot is
    /// whole, with [MfskStage.EARLY]; the whole slot adds the rest without
    /// repeating them. Other modes and depths decode the whole slot either way.
    fun setEarly(channel: Int, on: Boolean) = nativeSetEarly(live(), channel, on)

    /// Push IQ in the format the receiver was opened with, little-endian, I
    /// then Q; a sample split across calls is carried over. Decodes every slot
    /// this completes, and every early checkpoint it reaches, before returning.
    fun push(data: ByteArray, length: Int = data.size) = nativePush(live(), data, length)

    /// Complex samples consumed so far, gaps included: the stream's clock.
    val samplesIn: Long get() = nativeSamplesIn(live())

    /// Decodes waiting for [poll].
    val pending: Int get() = nativePending(live())

    /// Take every waiting decode, oldest first.
    fun poll(): List<MfskIqDecode> = nativePoll(live()).toList()

    override fun close() {
        if (handle != 0L) {
            for (d in lent.values) d.releaseLent()
            lent.clear()
            nativeClose(handle)
            handle = 0L
        }
    }
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
/// **Single-threaded**: one thread at a time, like [MfskDecoder].
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