// SPDX-License-Identifier: GPL-3.0-or-later
//
// JVM test for the Kotlin binding. Run by `bindings/kotlin/build.sh`,
// which CI runs on every source change — about 70 seconds, against the
// ~5 minute test job that gates the run anyway, so it costs no wall
// clock.
//
// What this is for: the binding is thin, so the risk is not decoder
// behaviour — that is tested in Rust and in the C++ driver. The risk is
// **marshalling**: a JNI signature that does not match the Kotlin
// declaration fails at run time with `UnsatisfiedLinkError`, not at
// compile time, and a field read in the wrong order produces plausible
// garbage. So this round-trips real audio and checks the values.

package io.github.mfskcore

private var failures = 0

private fun check(what: String, ok: Boolean) {
    if (ok) {
        println("  ok   $what")
    } else {
        System.err.println("  FAIL $what")
        failures++
    }
}

private fun <T> checkEq(what: String, got: T, want: T) {
    check("$what (got $got, want $want)", got == want)
}

fun main() {
    println("mfsk Kotlin binding — ABI ${Mfsk.abiVersion}")
    check("ABI is v2 or newer", Mfsk.abiVersion >= 2)

    // ── Introspection ───────────────────────────────────────────────
    val modes = Mfsk.modes()
    println("  ${modes.size} mode(s): ${modes.joinToString { Mfsk.modeName(it) }}")
    check("the build reports some modes", modes.isNotEmpty())

    val ft8 = modes.firstOrNull { Mfsk.modeName(it) == "FT8" }
    check("FT8 is addressable", ft8 != null)
    if (ft8 == null) { System.exit(1); return }

    val fst4 = modes.filter { Mfsk.modeName(it).startsWith("FST4-") }
    checkEq("all five FST4 sub-modes are addressable", fst4.size, 5)

    // The capability bits must cross intact. Pinning known answers
    // rather than "some bit is set" — the latter passes on garbage.
    check("FT8 drives the decode handle", Mfsk.supports(ft8, Mfsk.CAP_DECODE_HANDLE))
    check("FT8 has the sniper", Mfsk.supports(ft8, Mfsk.CAP_SNIPER))
    val ft4 = modes.first { Mfsk.modeName(it) == "FT4" }
    check("FT4 has no sniper", !Mfsk.supports(ft4, Mfsk.CAP_SNIPER))
    check("FT4 does wide-band AP", Mfsk.supports(ft4, Mfsk.CAP_AP_WIDEBAND))
    val wspr = modes.firstOrNull { Mfsk.modeName(it) == "WSPR" }
    if (wspr != null) {
        check("WSPR does not drive the decode handle",
              !Mfsk.supports(wspr, Mfsk.CAP_DECODE_HANDLE))
    }

    // Geometry, checked against known values so a field read in the
    // wrong order cannot pass.
    val info = Mfsk.modeInfo(ft8)
    checkEq("FT8 tones", info.ntones, 8)
    checkEq("FT8 slot samples", info.slotSamples12k, 180_000)
    checkEq("FT8 FEC k", info.fecK, 91)
    checkEq("FT8 symbols", info.nSymbols, 79)
    checkEq("FT8 slot FFT", info.decodeFft1Size, 192_000)
    checkEq("FT8 TX offset (ms)", info.txStartOffsetMs, 500)

    val f300 = fst4.first { Mfsk.modeName(it) == "FST4-300" }
    val i300 = Mfsk.modeInfo(f300)
    checkEq("FST4-300 slot FFT", i300.decodeFft1Size, 4_194_304)
    check("the slot-FFT spread is visible from Kotlin",
          i300.decodeFft1Size / info.decodeFft1Size >= 20)

    // ── Runtime ─────────────────────────────────────────────────────
    // On Android this is what lets a worker thread reach a JNIEnv at
    // all. Here it just has to take effect.
    val configured = Mfsk.configureRuntime(threads = 2, stackBytes = 512 * 1024)
    if (configured) {
        checkEq("the configured pool is the one in use", Mfsk.threadCount, 2)
    } else {
        println("  --   runtime already configured or unavailable (no rayon)")
    }

    // ── Round trip ──────────────────────────────────────────────────
    val frame = Mfsk.synthesize(ft8, "CQ", "JA1ABC", "PM95", 1500.0f)
    check("synthesis produced a frame", frame.isNotEmpty())
    val slot = ShortArray(info.slotSamples12k)
    val start = info.txStartOffsetMs * 12
    for (i in frame.indices) {
        if (start + i < slot.size) slot[start + i] = frame[i]
    }

    MfskSession.open(ft8).use { s ->
        val rows = s.decode(slot)
        println("  ${rows.size} decode(s)")
        for (r in rows) {
            println("    ${r.modeName} ${r.freqHz} Hz dt=${r.dtSec} snr=${r.snrDb} " +
                    "cv=${r.syncCv} info=${r.infoBits} '${r.text}'")
        }
        check("the round trip decoded", rows.any { it.text.contains("JA1ABC") })
        // Marshalling: every field has to be the one it says it is.
        val r = rows.firstOrNull { it.text.contains("JA1ABC") }
        if (r != null) {
            checkEq("row reports FT8", r.mode, ft8)
            checkEq("row reports FT8's info width", r.infoBits, 91)
            check("row frequency is near 1500 Hz", Math.abs(r.freqHz - 1500.0f) < 10.0f)
            check("sync score is populated", r.syncScore > 0.0f)
            check("nothing needed the hash table here", !r.hashResolved)
        }
    }

    // ── Callback delivery ───────────────────────────────────────────
    //
    // The listener can be called from a rayon worker, so the sink is
    // synchronized. What is checked is that the rows arriving early are
    // the rows the call returns — the array stays authoritative.
    MfskSession.open(ft8).use { s ->
        val streamed = java.util.Collections.synchronizedList(mutableListOf<MfskDecode>())
        s.onDecode { row -> streamed.add(row) }

        val rows = s.decode(slot)
        checkEq("the listener saw every row", streamed.size, rows.size)
        check("the listener's rows are the returned rows",
              streamed.map { it.text }.toSet() == rows.map { it.text }.toSet())
        check("a streamed row carries its fields",
              streamed.all { it.mode == ft8 && Math.abs(it.freqHz - 1500.0f) < 10.0f })

        val afterFirst = streamed.size
        s.onDecode(null)
        val second = s.decode(slot)
        checkEq("null stops delivery", streamed.size, afterFirst)
        check("and does not stop decoding", second.isNotEmpty())

        // A listener that throws cannot propagate into a worker thread,
        // so the shim reports and clears it. The stack trace below is
        // expected output, not a failure.
        println("  --   the next stack trace is deliberate (throwing listener)")
        s.onDecode { throw RuntimeException("listener blew up") }
        val third = s.decode(slot)
        check("a throwing listener does not break the decode", third.isNotEmpty())
        s.onDecode(null)
    }

    // ── Budget, known, FFT cache ────────────────────────────────────
    //
    // Three capability bits that used to be advertised with nothing to
    // call. Two stations in one slot, so a budget has something to cut
    // and a known list something to remove.
    val second = Mfsk.synthesize(ft8, "CQ", "VK3NV", "QF22", 1800.0f)
    val busy = slot.copyOf()
    for (i in second.indices) {
        val at = start + i
        if (at < busy.size) {
            busy[at] = (busy[at] + second[i]).coerceIn(-32768, 32767).toShort()
        }
    }

    MfskSession.open(ft8).use { s ->
        val full = s.decode(busy)
        check("the busy slot decodes both stations", full.size >= 2)

        val empty = s.lastBudget
        check("no budget means an empty report",
              !empty.exhausted && empty.candidatesSkipped == 0 &&
              empty.cutAtSync == null && empty.cutAtScore == null)

        var polls = 0
        s.setBudget { polls++; false }
        val cut = s.decode(busy)
        check("the predicate was polled", polls > 0)
        check("a refusing budget finds less (got ${cut.size} of ${full.size})",
              cut.size < full.size)
        val report = s.lastBudget
        println("  budget report: exhausted=${report.exhausted} " +
                "skipped=${report.candidatesSkipped} ran=${report.stagesRun} " +
                "cutAtSync=${report.cutAtSync} cutAtScore=${report.cutAtScore}")
        check("the report says work was cut", report.exhausted)
        check("and counts what it skipped", report.candidatesSkipped > 0)
        check("FT8 reports where the cut fell", report.cutAtSync != null)

        s.setBudget(null)
        checkEq("removing the budget restores the search", s.decode(busy).size, full.size)

        // Known: the second pass over the same slot has nothing new.
        s.keepKnown(true)
        checkEq("nothing is known yet", s.knownCount, 0)
        val firstPass = s.decode(busy)
        checkEq("the first pass becomes known", s.knownCount, firstPass.size)
        checkEq("a known signal is not reported twice", s.decode(busy).size, 0)
        s.keepKnown(false)
        checkEq("keepKnown(false) drops the list", s.knownCount, 0)

        // FFT cache: same answer twice, and a stale one is not reused.
        s.keepFftCache(true)
        checkEq("the cached pass decodes the same", s.decode(busy).size, full.size)
        checkEq("and so does the one that reuses it", s.decode(busy).size, full.size)
        val other = ShortArray(info.slotSamples12k)
        for (i in second.indices) if (start + i < other.size) other[start + i] = second[i]
        check("a cache from other audio is not reused",
              s.decode(other).any { it.text.contains("VK3NV") })
    }

    // ── Refusals reach Kotlin as exceptions ─────────────────────────
    if (wspr != null) {
        var threw = false
        try {
            MfskSession.open(wspr).close()
        } catch (e: IllegalStateException) {
            threw = true
            println("  WSPR refused: ${e.message}")
            check("the refusal names the capability to check",
                  e.message?.contains("MFSK_CAP_DECODE_HANDLE") == true)
        }
        check("a mode without the decode handle is refused", threw)
    }

    var threwPack = false
    try {
        Mfsk.synthesize(ft8, "XXX", "Y2Z", "FN42", 1500.0f)
    } catch (e: IllegalStateException) {
        threwPack = true
    }
    check("an unpackable callsign throws rather than emitting garbage", threwPack)

    // A closed session must not be usable.
    val closed = MfskSession.open(ft8)
    closed.close()
    closed.close()  // idempotent
    var threwClosed = false
    try {
        closed.decode(slot)
    } catch (e: IllegalStateException) {
        threwClosed = true
    }
    check("a closed session refuses to decode", threwClosed)

    // ── Decode parameters ───────────────────────────────────────────
    //
    // The parameters cross JNI as three flat arrays, so the risk is a slot
    // read as the wrong field: plausible garbage, not a crash. Each field
    // therefore has to be *seen* — by a refusal that names it, or by a
    // decode that only comes out right if it arrived.
    val d = Mfsk.defaultParams(ft8)
    checkEq("FT8's default band starts at", d.freqMinHz, 100.0f)
    checkEq("FT8's default band ends at", d.freqMaxHz, 3000.0f)
    check("a default candidate budget", d.maxCand > 0)
    checkEq("default depth is the full ladder", d.depth, MfskDecodeParams.DEPTH_BP_ALL_OSD)
    checkEq("default strictness", d.strictness, MfskDecodeParams.STRICTNESS_NORMAL)
    checkEq("equalisation is off", d.eqMode, MfskDecodeParams.EQ_OFF)
    check("no frequency hint (NaN in C)", d.freqHintHz == null)
    check("no transmit frequency (NaN in C)", d.txFreqHz == null)
    check("no AP hint", d.apHint == null)
    check("no noise blanker", d.noiseBlanker == null)
    checkEq("no SIC", d.sicRounds, 0)

    // The arrays are the wire format: what goes out has to come back.
    for (nb in listOf(null, MfskNoiseBlanker.Percent(7), MfskNoiseBlanker.Sweep(2, 33.0f))) {
        val p = d.copy(
            freqMinHz = 210.0f, freqMaxHz = 2900.0f, syncMin = 1.7f, maxCand = 41,
            depth = MfskDecodeParams.DEPTH_BP_ALL, strictness = MfskDecodeParams.STRICTNESS_DEEP,
            eqMode = MfskDecodeParams.EQ_LOCAL, freqHintHz = 1234.5f, sicRounds = 3,
            sicEarly = true, searchHz = 60.0f, txFreqHz = 1500.25f, noiseBlanker = nb,
        )
        checkEq("params survive the arrays ($nb)",
                MfskDecodeParams.fromArrays(p.floatArray(), p.intArray()), p)
    }

    fun refused(what: String, needle: String, mode: Int, p: MfskDecodeParams) {
        var msg: String? = null
        try {
            MfskSession.open(mode, p).close()
        } catch (e: IllegalStateException) {
            msg = e.message
        }
        check("$what is refused, naming $needle (got: $msg)", msg?.contains(needle) == true)
    }
    val ft4d = Mfsk.defaultParams(ft4)
    val f15 = fst4.first { Mfsk.modeName(it) == "FST4-15" }
    val f15d = Mfsk.defaultParams(f15)

    // Each int slot, by the refusal that names it.
    refused("an out-of-range depth", "depth", ft8, d.copy(depth = 9))
    refused("an out-of-range strictness", "strictness", ft8, d.copy(strictness = 9))
    refused("an out-of-range eq mode", "eq_mode", ft8, d.copy(eqMode = 9))
    refused("a zero candidate budget", "max_cand", ft8, d.copy(maxCand = 0))
    refused("SIC on FST4", "successive-interference", f15, f15d.copy(sicRounds = 2))
    refused("early decode on FT4", "checkpoint", ft4, ft4d.copy(sicEarly = true))
    refused("a noise blanker level past the GUI's range", "nb_percent",
            f15, f15d.copy(noiseBlanker = MfskNoiseBlanker.Percent(26)))
    refused("a blanker sweep step of 3", "nb_sweep_step",
            f15, f15d.copy(noiseBlanker = MfskNoiseBlanker.Sweep(3, 20.0f)))
    // Each float slot.
    refused("an inverted band", "search band", ft8, d.copy(freqMinHz = 2000.0f, freqMaxHz = 1000.0f))
    refused("a blanker sweep with no window", "nb_ftol_hz",
            f15, f15d.copy(noiseBlanker = MfskNoiseBlanker.Sweep(5, 0.0f)))
    refused("a narrow search on FT4", "MFSK_CAP_SNIPER", ft4, ft4d.copy(searchHz = 250.0f))
    refused("a narrow search with no carrier", "freq_hint_hz", ft8, d.copy(searchHz = 250.0f))
    refused("a transmit frequency on FT4", "MFSK_CAP_TX_FREQ", ft4, ft4d.copy(txFreqHz = 1500.0f))
    refused("a blanker on FT8", "MFSK_CAP_NOISE_BLANKER",
            ft8, d.copy(noiseBlanker = MfskNoiseBlanker.Percent(2)))

    check("only FT8 has a transmit frequency",
          Mfsk.supports(ft8, Mfsk.CAP_TX_FREQ) && !Mfsk.supports(ft4, Mfsk.CAP_TX_FREQ))
    check("only FST4 has the noise blanker",
          fst4.all { Mfsk.supports(it, Mfsk.CAP_NOISE_BLANKER) } &&
              !Mfsk.supports(ft8, Mfsk.CAP_NOISE_BLANKER))

    // What each accepted shape opens.
    for (m in fst4) {
        val base = Mfsk.defaultParams(m)
        for (nb in listOf(
            MfskNoiseBlanker.Percent(0), MfskNoiseBlanker.Percent(25),
            MfskNoiseBlanker.Sweep(1, 20.0f), MfskNoiseBlanker.Sweep(5, 20.0f),
        )) {
            MfskSession.open(m, base.copy(noiseBlanker = nb)).close()
        }
    }
    MfskSession.open(ft8, d.copy(searchHz = 250.0f, freqHintHz = 1500.0f)).close()
    MfskSession.open(ft8, d.copy(txFreqHz = 1500.0f)).close()

    // The band reaches the decoder, and a per-call override does not stick.
    MfskSession.open(ft8, d).use { s ->
        val off = d.copy(freqMinHz = 2500.0f, freqMaxHz = 2900.0f)
        check("a band that excludes the signal finds nothing",
              s.decode(slot, params = off).none { it.text.contains("JA1ABC") })
        check("and the override did not stick",
              s.decode(slot).any { it.text.contains("JA1ABC") })
    }

    // The AP hint, the QSO frequency and the transmit frequency reach the
    // decoder: an AP hypothesis that locks both callsigns is tried only
    // within 50 Hz of the QSO frequency or the transmit frequency. Here the
    // QSO frequency is 300 Hz off the signal, so the station is left to the
    // ordinary ladder unless the transmit frequency brings it back in range.
    // Kotlin's own noise, so the counts are this test's, not the Rust ones.
    run {
        val f0 = 1500.0f
        val sigFrame = Mfsk.synthesize(ft8, "K1JT", "HA0DU", "-12", f0)
        val sigma = Math.sqrt(
            (8000.0 * 0.1) * (8000.0 * 0.1) / 2.0 / (Math.pow(10.0, -22.0 / 10.0) * 2500.0 / 6000.0),
        )
        val ap = d.copy(apHint = MfskApHint("K1JT", "HA0DU"), freqHintHz = f0 + 300.0f)
        val withTx = ap.copy(txFreqHz = f0)
        var without = 0
        var with = 0
        var lost = 0
        var unexpected = 0
        for (seed in 0 until 12) {
            val rng = java.util.Random(seed.toLong())
            val audio = ShortArray(info.slotSamples12k)
            for (i in audio.indices) {
                val sig = if (i - start in sigFrame.indices) sigFrame[i - start] * 0.1 else 0.0
                audio[i] = Math.round(sig + sigma * rng.nextGaussian())
                    .coerceIn(-32768L, 32767L).toShort()
            }
            MfskSession.open(ft8, ap).use { a ->
                MfskSession.open(ft8, withTx).use { b ->
                    val ra = a.decode(audio)
                    val rb = b.decode(audio)
                    unexpected += (ra + rb).count { it.text != "K1JT HA0DU -12" }
                    without += ra.size
                    with += rb.size
                    if (ra.isNotEmpty() && rb.isEmpty()) lost++
                }
            }
        }
        println("  AP + QSO frequency 300 Hz off: $without of 12 without txFreqHz, $with with")
        checkEq("nothing but the injected message decodes", unexpected, 0)
        checkEq("txFreqHz never loses a station the hint alone found", lost, 0)
        check("txFreqHz recovers stations the out-of-range hint misses", with >= without + 3)
    }

    // ── Q65: the WSJT-X 3.2 settings, and the two lists it keeps ────
    //
    // Q65 has no decode handle, so this is `Mfsk.decodeQ65`. Each setting has
    // to change what is decoded: a field that is accepted and ignored looks
    // identical to one that works.
    val q30 = modes.firstOrNull { Mfsk.modeName(it) == "Q65-30A" }
    check("Q65-30A is addressable", q30 != null)
    if (q30 != null) {
        val qd = Mfsk.q65DefaultParams(q30)
        checkEq("Q65's default band starts at", qd.freqMinHz, 200.0f)
        checkEq("Q65's default window is WSJT-X's ±1 s (early)", qd.tEarlyS, 1.0f)
        checkEq("and late", qd.tLateS, 1.0f)
        checkEq("Q65-30A's frame starts 0.5 s in", qd.nominalStartS, 0.5f)
        checkEq("F Tol defaults to 10 Hz", qd.ftolHz, 10.0f)
        check("no Rx frequency, fading, list or hint by default",
              qd.rxFreqHz == null && qd.fading == null && qd.list == null && qd.apHint == null)
        check("Q65 defaults for a non-Q65 mode are refused", try {
            Mfsk.q65DefaultParams(ft8); false
        } catch (e: IllegalStateException) { true })

        for (fading in listOf(null, MfskQ65Fading(1.5f, MfskQ65Fading.LORENTZIAN))) {
            val p = qd.copy(
                freqMinHz = 900.0f, freqMaxHz = 2100.0f, nominalStartS = 0.75f, tEarlyS = 2.0f,
                tLateS = 3.0f, scoreThreshold = 0.2f, maxCand = 5, pileup = true, emeDelay = true,
                maxDrift = 20, rxFreqHz = 1500.5f, ftolHz = 25.0f, fading = fading,
            )
            checkEq("Q65 params survive the arrays ($fading)",
                    MfskQ65Params.fromArrays(p.floatArray(), p.intArray()), p)
        }

        val slotLen = Mfsk.modeInfo(q30).slotSamples12k
        fun slotOf(sig: FloatArray, startS: Float): FloatArray {
            val a = FloatArray(slotLen)
            val at = Math.round(startS * 12_000f)
            for (i in sig.indices) if (at + i < a.size) a[at + i] = sig[i]
            return a
        }
        val want = "K1ABC JA1ABC -15"
        val sig = Mfsk.synthesizeQ65(q30, "K1ABC", "JA1ABC", "-15", 1500.0f)
        val flaggedSig = Mfsk.synthesizeQ65(q30, "K1ABC", "JA1ABC", "-15", 1500.0f, copiedLastTx = true)
        check("the flag changes the transmission", !sig.contentEquals(flaggedSig))

        fun refusedQ65(what: String, needle: String, p: MfskQ65Params, callers: MfskQ65Callers? = null) {
            var msg: String? = null
            try {
                Mfsk.decodeQ65(q30, FloatArray(slotLen), p, callers)
            } catch (e: IllegalStateException) {
                msg = e.message
            }
            check("$what is refused, naming $needle (got: $msg)", msg?.contains(needle) == true)
        }
        val my = MfskQ65List.Standard("K1ABC", "JA1ABC")
        refusedQ65("Pileup with no hint", "AP hint", qd.copy(pileup = true))
        refusedQ65("an Rx frequency with no list", "ap_list", qd.copy(rxFreqHz = 1500.0f))
        refusedQ65("a list with fading", "mutually exclusive",
                   qd.copy(list = my, fading = MfskQ65Fading(1.0f)))
        refusedQ65("drift with fading", "fast-fading",
                   qd.copy(maxDrift = 10, fading = MfskQ65Fading(1.0f)))
        refusedQ65("drift past 50", "max_drift", qd.copy(maxDrift = 51))
        refusedQ65("the contest list with no callers", "MfskQ65Callers",
                   qd.copy(list = MfskQ65List.Contest("K1ABC")))
        refusedQ65("an unknown fading model", "fading_model",
                   qd.copy(fading = MfskQ65Fading(1.0f, model = 7)))

        // dt is measured from the nominal start; the flag reaches the row.
        for ((startS, wantDt) in listOf(0.5f to 0.0f, 0.9f to 0.4f, 0.2f to -0.3f)) {
            val rows = Mfsk.decodeQ65(q30, slotOf(sig, startS), qd)
            checkEq("the frame at $startS s decodes", rows.map { it.text }, listOf(want))
            check("dt is $wantDt (got ${rows[0].dtSec})", Math.abs(rows[0].dtSec - wantDt) < 0.05f)
            checkEq("the row carries its mode", rows[0].mode, q30)
            check("an unflagged frame is not flagged", !rows[0].copiedLastTx)
        }
        val flaggedRows = Mfsk.decodeQ65(q30, slotOf(flaggedSig, 0.5f), qd)
        checkEq("the flagged frame decodes", flaggedRows.map { it.text }, listOf(want))
        check("and says it copied the last Tx", flaggedRows[0].copiedLastTx)

        // EME delay: a frame 3 s late is beyond ±1 s.
        val late = slotOf(sig, 3.5f)
        check("a frame 3 s late is not in the default window",
              Mfsk.decodeQ65(q30, late, qd).isEmpty())
        val eme = Mfsk.decodeQ65(q30, late, qd.copy(emeDelay = true))
        checkEq("the EME delay reaches it", eme.map { it.text }, listOf(want))
        check("at dt +3 s (got ${eme.firstOrNull()?.dtSec})",
              eme.isNotEmpty() && Math.abs(eme[0].dtSec - 3.0f) < 0.05f)

        // Pileup: a MyCall + DxCall hint cannot match a flagged reply without it.
        val hint = MfskQ65ApHint(call1 = "K1ABC", call2 = "JA1ABC")
        val pile = Mfsk.decodeQ65(q30, slotOf(flaggedSig, 0.5f), qd.copy(pileup = true, apHint = hint))
        checkEq("Pileup decodes a flagged reply under a MyCall + DxCall hint",
                pile.map { it.text }, listOf(want))
        check("and flags it", pile.isNotEmpty() && pile[0].copiedLastTx)
        checkEq("an unflagged reply decodes under Pileup too",
                Mfsk.decodeQ65(q30, slotOf(sig, 0.5f), qd.copy(pileup = true, apHint = hint))
                    .map { it.text }, listOf(want))

        // q3: a window that holds nothing else, so only the list decode at the
        // Rx frequency can find it.
        val q3 = qd.copy(
            freqMinHz = 3900.0f, freqMaxHz = 3950.0f,
            list = MfskQ65List.Standard("K1ABC", "JA1ABC", "PM95"),
        )
        check("the scan window holds nothing",
              Mfsk.decodeQ65(q30, slotOf(sig, 0.5f), q3).isEmpty())
        checkEq("q3 finds the list message at the Rx frequency",
                Mfsk.decodeQ65(q30, slotOf(sig, 0.5f), q3.copy(rxFreqHz = 1500.0f)).map { it.text },
                listOf(want))
        check("and not when the Rx frequency is 100 Hz off",
              Mfsk.decodeQ65(q30, slotOf(sig, 0.5f), q3.copy(rxFreqHz = 1600.0f)).isEmpty())

        // The contest list: K1ABC W9XYZ RR73 is on it only because W9XYZ called.
        MfskQ65Callers().use { callers ->
            val rr73 = slotOf(Mfsk.synthesizeQ65(q30, "K1ABC", "W9XYZ", "RR73", 1500.0f), 0.5f)
            val contest = qd.copy(
                freqMinHz = 3900.0f, freqMaxHz = 3950.0f, rxFreqHz = 1500.0f,
                list = MfskQ65List.Contest("K1ABC"),
            )
            check("nobody listed: nothing to find",
                  Mfsk.decodeQ65(q30, rr73, contest, callers).isEmpty())
            callers.record(1500.0f, "K1ABC W9XYZ EN37", now = 1_000L)
            checkEq("a remembered caller decodes",
                    Mfsk.decodeQ65(q30, rr73, contest, callers).map { it.text },
                    listOf("K1ABC W9XYZ RR73"))
        }
    }

    MfskQ65Callers().use { c ->
        c.record(1500.0f, "K1ABC W9XYZ EN37", 100L)
        c.record(1510.0f, "K1ABC JA1ABC R PM95", 100L)
        c.record(1520.0f, "K1ABC VK3ABC -15", 100L)        // no grid: not added
        c.record(1530.0f, "K1ABC W9XYZ/R EN37", 100L)      // compound: ignored
        checkEq("the caller list holds two", c.size, 2)
        checkEq("first caller", c.callers[0], MfskQ65Caller("W9XYZ", "EN37", 100L, 1500))
        checkEq("second caller", c.callers[1], MfskQ65Caller("JA1ABC", "PM95", 100L, 1510))
        c.record(1600.0f, "K1ABC W9XYZ RR73", 500L)
        checkEq("a known caller is refreshed", c.callers[0].lastHeard, 500L)
        c.expire(100L + 24 * 3600 + 1)
        checkEq("24 hours on only the refreshed one is left", c.size, 1)
        c.remove("W9XYZ")
        checkEq("and it can be forgotten", c.size, 0)
    }
    val closedCallers = MfskQ65Callers()
    closedCallers.close()
    closedCallers.close()  // idempotent
    check("a closed caller list refuses use", try {
        closedCallers.record(1500.0f, "K1ABC W9XYZ EN37", 0L); false
    } catch (e: IllegalStateException) { true })

    MfskQ65History().use { h ->
        check("an empty history finds nothing", h.lookup(1500.0f) == null)
        h.push(1500.0f, "K1ABC JA1ABC PM95")
        checkEq("the DX station within 10 Hz", h.lookup(1508.0f), MfskQ65Dx("JA1ABC", "PM95"))
        check("and not outside it", h.lookup(1520.0f) == null)
        h.push(1500.0f, "CQ VK3ABC QF22")
        checkEq("a later CQ is passed over", h.lookup(1500.0f), MfskQ65Dx("JA1ABC", "PM95"))
        h.push(1500.0f, "K1ABC W9XYZ -15")
        checkEq("a message with no grid names the call alone",
                h.lookup(1500.0f), MfskQ65Dx("W9XYZ", null))
        h.record(listOf(MfskDecode(
            mode = 0, text = "K1ABC DL1XYZ JO62", freqHz = 700.0f, dtSec = 0f, snrDb = 0f,
            syncScore = 0f, syncCv = 0f, hardErrors = 0, infoBits = 0, pass = 0,
            hashResolved = false,
        )))
        checkEq("rows go in as pushes", h.lookup(700.0f), MfskQ65Dx("DL1XYZ", "JO62"))
        checkEq("the history holds what was pushed", h.size, 4)
        for (i in 0 until 120) h.push(2000.0f + i, "K1ABC JA1ABC PM95")
        checkEq("only the 100 most recent are kept", h.size, 100)
    }

    // ── JTTY: a stateful receiver, fed a recording in chunks ────────
    val jtty = modes.firstOrNull { Mfsk.modeName(it) == "JTTY" }
    check("JTTY is addressable", jtty != null)
    if (jtty != null) {
        check("JTTY is a stream receiver", Mfsk.supports(jtty, Mfsk.CAP_STREAM_RECEIVER))
        check("JTTY has no slot decode handle", !Mfsk.supports(jtty, Mfsk.CAP_DECODE_HANDLE))
        checkEq("JTTY frame is 22656 samples", Mfsk.modeInfo(jtty).slotSamples12k, 22_656)

        val wavPath = System.getProperty("mfsk.jtty.wav")
        check("the golden JTTY recording path is set (-Dmfsk.jtty.wav)", wavPath != null)
        if (wavPath != null) {
            val bytes = java.io.File(wavPath).readBytes()
            var at = -1
            for (i in 0 until bytes.size - 8) {
                if (bytes[i] == 'd'.code.toByte() && bytes[i + 1] == 'a'.code.toByte() &&
                    bytes[i + 2] == 't'.code.toByte() && bytes[i + 3] == 'a'.code.toByte()) {
                    at = i + 8
                    break
                }
            }
            check("the recording has a data chunk", at > 0)
            val n = (bytes.size - at) / 2
            val pcm = ShortArray(n) { i ->
                val lo = bytes[at + 2 * i].toInt() and 0xff
                val hi = bytes[at + 2 * i + 1].toInt()
                ((hi shl 8) or lo).toShort()
            }
            val expect = "RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!"
            val latest = LinkedHashMap<Long, MfskJttyUpdate>()
            MfskJttyReceiver.open().use { rx ->
                var pos = 0
                while (pos < pcm.size) {
                    val end = minOf(pos + 4096, pcm.size)
                    for (u in rx.push(pcm.copyOfRange(pos, end))) latest[u.id] = u
                    pos = end
                }
                check("push drains after itself", rx.poll().isEmpty())
                check("nothing is left open once the message completes",
                      rx.finish().none { it.text.startsWith("RAN ALL NIGHT") })
            }
            for (u in latest.values) println("  message ${u.id}: ${u.text}")
            val msg = latest.values.firstOrNull { it.text == expect }
            check("the recording's message comes out", msg != null)
            check("and it is complete", msg?.complete == true)
            check("at about 1507 Hz", msg != null && Math.abs(msg.freqHz - 1507f) < 3f)

            // Another rate goes through the resampler.
            val up = ShortArray(pcm.size * 2)
            for (i in 0 until pcm.size - 1) {
                up[2 * i] = pcm[i]
                up[2 * i + 1] = ((pcm[i] + pcm[i + 1]) / 2).toShort()
            }
            val seen = ArrayList<MfskJttyUpdate>()
            MfskJttyReceiver.open(24_000).use { rx ->
                var pos = 0
                while (pos < up.size) {
                    val end = minOf(pos + 8192, up.size)
                    seen.addAll(rx.push(up.copyOfRange(pos, end)))
                    pos = end
                }
            }
            check("24 kHz audio decodes too", seen.any { it.text == expect })
        }

        var threwParams = false
        try {
            MfskJttyReceiver.open(12_000, MfskJttyParams(nfaHz = 2000f, nfbHz = 1000f)).close()
        } catch (e: IllegalStateException) {
            threwParams = true
        }
        check("an empty band is refused", threwParams)

        // Transmit: text -> tones -> audio, and back through a receiver.
        val txTones = MfskJtty.tones("CQ K1ABC CQ")
        checkEq("one frame is 59 tones", txTones.size, 59)
        val txPcm = ShortArray(12_000) + MfskJtty.synthesize(txTones) + ShortArray(6 * 12_000)
        val heard = ArrayList<MfskJttyUpdate>()
        MfskJttyReceiver.open().use { rx ->
            var pos = 0
            while (pos < txPcm.size) {
                val end = minOf(pos + 4096, txPcm.size)
                heard.addAll(rx.push(txPcm.copyOfRange(pos, end)))
                pos = end
            }
        }
        check("the transmitted message comes back through the receiver",
              heard.any { it.text == "CQ K1ABC CQ" && it.complete })
        check("encode is tones then synthesize",
              MfskJtty.encode("CQ K1ABC CQ").size == txPcm.size - 12_000 - 6 * 12_000)
        check("an empty message has nothing to send", MfskJtty.tones("   ").isEmpty())
        var threwLong = false
        try {
            MfskJtty.tones("A".repeat(81))
        } catch (e: IllegalStateException) {
            threwLong = true
        }
        check("a message over 80 characters is refused", threwLong)

        val closedRx = MfskJttyReceiver.open()
        closedRx.close()
        closedRx.close()  // idempotent
        var threwRx = false
        try {
            closedRx.push(ShortArray(16))
        } catch (e: IllegalStateException) {
            threwRx = true
        }
        check("a closed receiver refuses audio", threwRx)
    }

    if (failures == 0) {
        println("\nALL OK")
    } else {
        System.err.println("\n$failures FAILURE(S)")
        System.exit(1)
    }
}
