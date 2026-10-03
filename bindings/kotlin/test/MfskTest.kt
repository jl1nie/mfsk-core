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

/// Runs [body], expecting it to throw a [T] whose message contains [needle].
private inline fun <reified T : Throwable> refused(what: String, needle: String, body: () -> Unit) {
    var msg: String? = null
    var wrongType: Throwable? = null
    try {
        body()
    } catch (e: Throwable) {
        if (e is T) msg = e.message else wrongType = e
    }
    check(
        "$what throws ${T::class.simpleName}, naming '$needle' (got: ${wrongType ?: msg})",
        wrongType == null && msg?.contains(needle) == true,
    )
}

private fun List<MfskDecode>.texts() = map { it.text }

fun main() {
    println("mfsk Kotlin binding — ABI ${Mfsk.abiVersion}")
    check("ABI is v3 or newer", Mfsk.abiVersion >= 3)

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
    val wspr = modes.first { Mfsk.modeName(it) == "WSPR" }
    check("only FT8 has a transmit frequency",
          Mfsk.supports(ft8, Mfsk.CAP_TX_FREQ) && !Mfsk.supports(ft4, Mfsk.CAP_TX_FREQ))
    check("only FST4 has the noise blanker",
          fst4.all { Mfsk.supports(it, Mfsk.CAP_NOISE_BLANKER) } &&
              !Mfsk.supports(ft8, Mfsk.CAP_NOISE_BLANKER))

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

    MfskDecoder.open(ft8).use { dec ->
        checkEq("the decoder knows its mode", dec.mode, ft8)
        val rows = dec.decode(slot)
        println("  ${rows.size} decode(s)")
        for (r in rows) {
            println("    ${r.modeName} ${r.freqHz} Hz dt=${r.dtSec} snr=${r.snrDb} " +
                    "cv=${r.syncCv} info=${r.infoBits} '${r.text}'")
        }
        check("the round trip decoded", rows.any { it.text.contains("CQ JA1ABC PM95") })
        // Marshalling: every field has to be the one it says it is.
        val r = rows.firstOrNull { it.text.contains("JA1ABC") }
        if (r != null) {
            checkEq("row reports FT8", r.mode, ft8)
            checkEq("row reports FT8's info width", r.infoBits, 91)
            check("row frequency is near 1500 Hz", Math.abs(r.freqHz - 1500.0f) < 10.0f)
            check("sync score is populated", r.syncScore > 0.0f)
            check("nothing needed the hash table here", !r.hashResolved)
            val bits = dec.copyInfo(rows.indexOf(r))
            checkEq("the FEC information bits come back", bits.size, 91)
            check("and are bits", bits.all { it == 0.toByte() || it == 1.toByte() })
        }
        refused<MfskInvalidArgException>("a row past the last", "index") { dec.copyInfo(99) }
        check("no decoder error was recorded for it", dec.lastError == null)
        // A recording at another rate is resampled; float audio is taken at any level.
        val quiet = FloatArray(slot.size) { slot[it] / 32_768.0f * 0.002f }
        check("quiet float audio decodes", dec.decode(quiet).any { it.text.contains("JA1ABC") })
        val doubled = ShortArray(slot.size * 2) { slot[it / 2] }
        check("24 kHz audio decodes", dec.decode(doubled, sampleRate = 24_000).any { it.text.contains("JA1ABC") })
    }

    // ── Callback delivery ───────────────────────────────────────────
    //
    // The listener can be called from a rayon worker, so the sink is
    // synchronized. What is checked is that the rows arriving early are
    // the rows the call returns — the array stays authoritative.
    MfskDecoder.open(ft8).use { dec ->
        val streamed = java.util.Collections.synchronizedList(mutableListOf<MfskDecode>())
        dec.onDecode { row -> streamed.add(row) }

        val rows = dec.decode(slot)
        checkEq("the listener saw every row", streamed.size, rows.size)
        check("the listener's rows are the returned rows",
              streamed.map { it.text }.toSet() == rows.map { it.text }.toSet())
        check("a streamed row carries its fields",
              streamed.all { it.mode == ft8 && Math.abs(it.freqHz - 1500.0f) < 10.0f })

        val afterFirst = streamed.size
        dec.onDecode(null)
        val second = dec.decode(slot)
        checkEq("null stops delivery", streamed.size, afterFirst)
        check("and does not stop decoding", second.isNotEmpty())

        // A per-call listener is for that call only, and gives the standing
        // one back afterwards.
        val perCall = java.util.Collections.synchronizedList(mutableListOf<MfskDecode>())
        dec.onDecode { row -> streamed.add(row) }
        val before = streamed.size
        val third = dec.decode(slot, onRow = { perCall.add(it) })
        checkEq("the per-call listener saw every row", perCall.size, third.size)
        checkEq("and the standing one saw none of them", streamed.size, before)
        dec.decode(slot)
        check("the standing listener is back", streamed.size > before)
        dec.onDecode(null)

        // A listener that throws cannot propagate into a worker thread,
        // so the shim reports and clears it. The stack trace below is
        // expected output, not a failure.
        println("  --   the next stack trace is deliberate (throwing listener)")
        dec.onDecode { throw RuntimeException("listener blew up") }
        val fourth = dec.decode(slot)
        check("a throwing listener does not break the decode", fourth.isNotEmpty())
        dec.onDecode(null)
    }

    // ── Budget ──────────────────────────────────────────────────────
    //
    // Two stations in one slot, so a budget has something to cut.
    val second = Mfsk.synthesize(ft8, "CQ", "VK3NV", "QF22", 1800.0f)
    val busy = slot.copyOf()
    for (i in second.indices) {
        val at = start + i
        if (at < busy.size) {
            busy[at] = (busy[at] + second[i]).coerceIn(-32768, 32767).toShort()
        }
    }

    // Pinned to the single pass: its scheduler polls per candidate, which is
    // what candidatesSkipped and cutAtSync describe. FT8's default since
    // 0.12.0 (sic_early) polls per stage.
    MfskDecoder.open(ft8, null, MfskExtras(strategy = MfskStrategy.SinglePass)).use { dec ->
        val full = dec.decode(busy)
        check("the busy slot decodes both stations", full.size >= 2)

        val empty = dec.lastBudget
        check("no budget means an empty report",
              !empty.exhausted && empty.candidatesSkipped == 0 &&
              empty.cutAtSync == null && empty.cutAtScore == null)

        var polls = 0
        dec.setBudget { polls++; false }
        val cut = dec.decode(busy)
        check("the predicate was polled", polls > 0)
        check("a refusing budget finds less (got ${cut.size} of ${full.size})",
              cut.size < full.size)
        val report = dec.lastBudget
        println("  budget report: exhausted=${report.exhausted} " +
                "skipped=${report.candidatesSkipped} ran=${report.stagesRun} " +
                "cutAtSync=${report.cutAtSync} cutAtScore=${report.cutAtScore}")
        check("the report says work was cut", report.exhausted)
        check("and counts what it skipped", report.candidatesSkipped > 0)
        check("FT8 reports where the cut fell", report.cutAtSync != null)

        dec.setBudget(null)
        checkEq("removing the budget restores the search", dec.decode(busy).size, full.size)
    }
    MfskDecoder.open(wspr).use { w ->
        refused<MfskUnsupportedException>("a budget on WSPR", "MFSK_CAP_BUDGET") { w.setBudget { true } }
        w.setBudget(null) // removing one is always fine
    }

    // ── Typed refusals ──────────────────────────────────────────────
    //
    // An option the mode lacks is MfskUnsupportedException naming it, at open
    // (or setExtras) and never at decode time; a value out of range is an
    // MfskInvalidArgException; a mode this build lacks is not either.
    val f60 = fst4.first { Mfsk.modeName(it) == "FST4-60A" }
    val f15 = fst4.first { Mfsk.modeName(it) == "FST4-15" }
    val jt9 = modes.first { Mfsk.modeName(it) == "JT9" }
    fun opens(mode: Int, e: MfskExtras): Boolean = try {
        MfskDecoder.open(mode, null, e).close(); true
    } catch (e: MfskUnsupportedException) { false }

    // FST4 has no subtraction (fst4_decode.f90), FT4 no a7, WSPR no AP.
    val sic = MfskExtras(strategy = MfskStrategy.SicRounds(2))
    check("SIC rounds open on FT8 and FT4", opens(ft8, sic) && opens(ft4, sic))
    check("and are refused on FST4 and WSPR", !opens(f60, sic) && !opens(wspr, sic))
    refused<MfskUnsupportedException>("SIC on FST4", "subtraction") { MfskDecoder.open(f60, null, sic) }
    val a7 = MfskExtras(a7 = true)
    check("a7 opens on FT8 and not FT4", opens(ft8, a7) && !opens(ft4, a7))
    val hint = MfskExtras(apHint = MfskApHint("CQ"))
    check("an AP hint opens on FT8 and not WSPR or JT9",
          opens(ft8, hint) && !opens(wspr, hint) && !opens(jt9, hint))
    refused<MfskUnsupportedException>("an AP hint on WSPR", "WSPR") { MfskDecoder.open(wspr, null, hint) }
    val nb = MfskExtras(noiseBlanker = MfskNoiseBlanker.Percent(5))
    check("the noise blanker opens on FST4 and not FT8", opens(f60, nb) && !opens(ft8, nb))
    refused<MfskUnsupportedException>("a blanker on FT8", "noise blanker") { MfskDecoder.open(ft8, null, nb) }
    check("early decode opens on FT8 and not FT4",
          opens(ft8, MfskExtras(strategy = MfskStrategy.SicEarly)) &&
              !opens(ft4, MfskExtras(strategy = MfskStrategy.SicEarly)))
    check("a narrow search opens on FT8 and not FT4",
          opens(ft8, MfskExtras(sniperHz = 250f)) && !opens(ft4, MfskExtras(sniperHz = 250f)))
    check("a Q65 option is refused on FT8", !opens(ft8, MfskExtras(maxDrift = 5)))
    // The same refusal on a decoder that is already open leaves it alone.
    MfskDecoder.open(ft8).use { d ->
        refused<MfskUnsupportedException>("setExtras with a blanker", "noise blanker") { d.setExtras(nb) }
        check("the decoder's own error slot has it too", d.lastError?.contains("noise blanker") == true)
        d.setExtras(MfskExtras(osd = true, strictness = MfskStrictness.NORMAL))
        check("and a good block applies afterwards", d.decode(slot).isNotEmpty())
    }
    // Out of range is the caller's mistake, not a missing option.
    refused<MfskInvalidArgException>("a blanker level past the GUI's range", "nb_percent") {
        MfskDecoder.open(f60, null, MfskExtras(noiseBlanker = MfskNoiseBlanker.Percent(99)))
    }
    refused<MfskInvalidArgException>("a blanker sweep step of 3", "nb_sweep_step") {
        MfskDecoder.open(f60, null, MfskExtras(noiseBlanker = MfskNoiseBlanker.Sweep(3, 20f)))
    }
    refused<MfskInvalidArgException>("a blanker sweep with no window", "nb_ftol_hz") {
        MfskDecoder.open(f60, null, MfskExtras(noiseBlanker = MfskNoiseBlanker.Sweep(5, 0f)))
    }
    refused<MfskInvalidArgException>("drift past 50 bins", "max_drift") {
        MfskDecoder.open(modes.first { Mfsk.modeName(it) == "Q65-30A" }, null, MfskExtras(maxDrift = 51))
    }
    refused<MfskUnsupportedException>("Pileup with no AP hint", "Pileup") {
        MfskDecoder.open(modes.first { Mfsk.modeName(it) == "Q65-30A" }, null, MfskExtras(pileup = true))
    }
    // What each accepted shape opens.
    for (m in fst4) {
        for (n in listOf(
            MfskNoiseBlanker.Percent(0), MfskNoiseBlanker.Percent(25),
            MfskNoiseBlanker.Sweep(1, 20f), MfskNoiseBlanker.Sweep(5, 20f),
        )) {
            MfskDecoder.open(m, null, MfskExtras(noiseBlanker = n)).close()
        }
    }

    // A mode that is not decoded as a slot has no parameter block.
    val msk = modes.firstOrNull { Mfsk.modeName(it) == "MSK144" }
    if (msk != null) {
        refused<MfskUnknownModeException>("a parameter block for MSK144", "decoder") { Mfsk.defaultParams(msk) }
        refused<MfskUnknownModeException>("a decoder for MSK144", "decoder") { MfskDecoder.open(msk) }
    }
    refused<MfskInvalidArgException>("a mode number that is not a mode", "mode") { Mfsk.defaultParams(9_999) }

    refused<MfskException>("an unpackable callsign", "") { Mfsk.synthesize(ft8, "XXX", "Y2Z", "FN42", 1500.0f) }
    refused<MfskUnsupportedException>("synthesis for a mode with no tone stage", "tone stage") {
        Mfsk.synthesize(wspr, "CQ", "JA1ABC", "PM95", 1500.0f)
    }

    // A closed decoder must not be usable.
    val closed = MfskDecoder.open(ft8)
    closed.close()
    closed.close()  // idempotent
    refused<IllegalStateException>("a closed decoder", "closed") { closed.decode(slot) }

    // ── Parameters ──────────────────────────────────────────────────
    //
    // The parameters cross JNI as flat arrays, so the risk is a slot read as
    // the wrong field: plausible garbage, not a crash. Each field therefore
    // has to be *seen* — by a refusal that names it, by a decode that only
    // comes out right if it arrived, or by coming back from the library.
    val d = Mfsk.defaultParams(ft8)
    checkEq("FT8's default band", d.band, 200.0f..4000.0f)
    checkEq("default depth is the GUI's", d.depth, MfskDepth.DEEP)
    checkEq("FT8's AP starts off", d.ap, MfskApMode.OFF)
    checkEq("FT4's AP starts on", Mfsk.defaultParams(ft4).ap, MfskApMode.FULL)
    checkEq("FST4-60's band", Mfsk.defaultParams(f60).band, 600.0f..1400.0f)
    check("no Rx, tolerance or Tx frequency (NaN in C)",
          d.rxFreqHz == null && d.tolHz == null && d.txFreqHz == null)
    check("no averaging, deep search or EME delay", !d.averaging && !d.deepSearch && !d.emeDelay)
    checkEq("no station", d.station, MfskStation())
    checkEq("no QSO", d.qso, MfskQso())
    checkEq("no contest", d.contest, MfskContest.NONE)
    checkEq("the library's unset extras are MfskExtras()", Mfsk.libraryDefaultExtras(), MfskExtras())

    // The arrays are the wire format: what goes out has to come back.
    for (depth in MfskDepth.entries) for (contest in MfskContest.entries) {
        val p = d.copy(
            band = 210.0f..2900.0f, rxFreqHz = 1234.5f, tolHz = 17.25f, txFreqHz = 1500.25f,
            depth = depth, averaging = true, deepSearch = true, emeDelay = true,
            station = MfskStation("K1ABC", "FN42"),
            qso = MfskQso("JA1ABC", "PM95", MfskQsoProgress.ROGERS),
            ap = MfskApMode.CQ_ONLY, contest = contest,
        )
        checkEq("params survive the arrays ($depth, $contest)",
                MfskParams.fromArrays(p.floatArray(), p.intArray(), p.stringArray()), p)
    }
    for (e in listOf(
        MfskExtras(),
        MfskExtras(
            syncMin = 1.7f, maxCand = 41, osd = false, strictness = MfskStrictness.DEEP,
            strategy = MfskStrategy.SicRounds(3), eqMode = MfskEqMode.LOCAL,
            messageFilter = MfskMessageFilter.CODEC, a7 = true, sniperHz = 60f,
            apHint = MfskApHint("K1JT", "HA0DU", "KP20", "-12"),
            noiseBlanker = MfskNoiseBlanker.Sweep(2, 33f),
            tEarlyS = 0.5f, tLateS = 2.5f, scoreThreshold = 0.2f, maxCyclesPerBit = 9_000,
            chaseTrials = 40, pileup = true, maxDrift = 20,
            fading = MfskQ65Fading(1.5f, MfskQ65Fading.LORENTZIAN),
        ),
        MfskExtras(osd = true, strategy = MfskStrategy.SinglePass, noiseBlanker = MfskNoiseBlanker.Percent(7)),
        MfskExtras(strategy = MfskStrategy.SicEarly),
    )) {
        checkEq("extras survive the arrays", MfskExtras.fromArrays(e.floatArray(), e.intArray(), e.stringArray()), e)
    }

    // Bad blocks are refused, not clamped.
    refused<MfskInvalidArgException>("an inverted band", "band") {
        MfskDecoder.open(ft8, d.copy(band = 2000.0f..1000.0f))
    }
    refused<MfskInvalidArgException>("a call too long for its field", "station call") {
        MfskDecoder.open(ft8, d.copy(station = MfskStation("K1ABCDEFGHIJKLMNOP")))
    }
    refused<MfskInvalidArgException>("an AP field too long for its slot", "AP call1") {
        MfskDecoder.open(ft8, null, MfskExtras(apHint = MfskApHint("A".repeat(16))))
    }

    // The band reaches the decoder, and setParams puts it back.
    MfskDecoder.open(ft8, d).use { dec ->
        val off = d.copy(band = 2500.0f..2900.0f)
        dec.setParams(off)
        check("a band that excludes the signal finds nothing",
              dec.decode(slot).none { it.text.contains("JA1ABC") })
        dec.setParams(d)
        check("and setParams brings it back", dec.decode(slot).any { it.text.contains("JA1ABC") })
        refused<MfskInvalidArgException>("setParams with an inverted band", "band") {
            dec.setParams(d.copy(band = 3000.0f..100.0f))
        }
        check("and the refused block did not stick", dec.decode(slot).any { it.text.contains("JA1ABC") })
    }

    // The AP hint, the Rx frequency and the transmit frequency reach the
    // decoder: an AP hypothesis that locks both callsigns is tried only
    // within 50 Hz of the Rx frequency or the transmit frequency. Here the
    // Rx frequency is 300 Hz off the signal, so the station is left to the
    // ordinary ladder unless the transmit frequency brings it back in range.
    // Kotlin's own noise, so the counts are this test's, not the Rust ones.
    run {
        val f0 = 1500.0f
        val sigFrame = Mfsk.synthesize(ft8, "K1JT", "HA0DU", "-12", f0)
        val sigma = Math.sqrt(
            (8000.0 * 0.1) * (8000.0 * 0.1) / 2.0 / (Math.pow(10.0, -22.0 / 10.0) * 2500.0 / 6000.0),
        )
        val ap = d.copy(ap = MfskApMode.FULL, rxFreqHz = f0 + 300.0f, tolHz = 20f)
        val withTx = ap.copy(txFreqHz = f0)
        val apHint = MfskExtras(apHint = MfskApHint("K1JT", "HA0DU"))
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
            MfskDecoder.open(ft8, ap, apHint).use { a ->
                MfskDecoder.open(ft8, withTx, apHint).use { b ->
                    val ra = a.decode(audio)
                    val rb = b.decode(audio)
                    unexpected += (ra + rb).count { it.text != "K1JT HA0DU -12" }
                    without += ra.size
                    with += rb.size
                    if (ra.isNotEmpty() && rb.isEmpty()) lost++
                }
            }
        }
        println("  AP + Rx frequency 300 Hz off: $without of 12 without txFreqHz, $with with")
        checkEq("nothing but the injected message decodes", unexpected, 0)
        checkEq("txFreqHz never loses a station the hint alone found", lost, 0)
        check("txFreqHz recovers stations the out-of-range hint misses", with >= without + 3)
    }

    // ── What a decoder keeps between periods ────────────────────────
    //
    // The callsign table is the decoder's own and outlives the period: a
    // `<...>` that period 10 introduced reads as the call in period 11 — on the
    // same decoder, and not on another.
    run {
        fun slotOf(msg77: ByteArray): ShortArray {
            val pcm = Mfsk.synthesize(ft8, msg77, 1500.0f)
            val s = ShortArray(info.slotSamples12k)
            for (i in pcm.indices) if (6_000 + i < s.size) s[6_000 + i] = pcm[i]
            return s
        }
        val heard = slotOf(Mfsk.pack77("CQ", "VK3NV", "QF22"))
        // A type 4 message: a non-standard call, with VK3NV only a hash.
        val type4 = Mfsk.pack77Type4("JA1ABC/QRP", "VK3NV", null, false)
        val hashed = slotOf(type4)
        check("without a decoder the hash stays a hash", Mfsk.unpack77(type4).contains("<...>"))

        MfskDecoder.open(ft8).use { a ->
            MfskDecoder.open(ft8).use { b ->
                check("period 10 names VK3NV", a.decode(heard, period = 10).any { it.text.contains("VK3NV") })
                val with = a.decode(hashed, period = 11)
                val without = b.decode(hashed, period = 11)
                // `a` learned VK3NV in period 10; `b` never heard it.
                check("the decoder that heard it resolves the hash (${with.texts()})",
                      with.any { it.text.contains("<VK3NV>") })
                check("and flags the row", with.any { it.hashResolved })
                check("another decoder does not (${without.texts()})",
                      without.any { it.text.contains("<...>") })
                check("unpack77 resolves against the decoder's table", a.unpack77(type4).contains("<VK3NV>"))
                check("and not against another's", b.unpack77(type4).contains("<...>"))
            }
        }
        // Teaching it from outside works too, and clearing forgets.
        MfskDecoder.open(ft8).use { c ->
            c.addCallsign("VK3NV")
            check("a taught call resolves", c.decode(hashed).any { it.text.contains("<VK3NV>") })
            c.clear()
            check("clear forgets it", c.decode(hashed).any { it.text.contains("<...>") })
        }
        // A mode whose messages carry no hashes says so.
        MfskDecoder.open(wspr).use { w ->
            refused<MfskUnsupportedException>("a callsign for WSPR", "hashed") { w.addCallsign("VK3NV") }
        }
    }

    // ── The other modes decode through the same handle ──────────────
    fun placed(frame: FloatArray, offsetS: Float, slotS: Int): FloatArray {
        val a = FloatArray(slotS * 12_000)
        val at = Math.round(offsetS * 12_000f)
        for (i in frame.indices) if (at + i < a.size) a[at + i] = frame[i]
        return a
    }
    MfskDecoder.open(wspr).use { dec ->
        val rows = dec.decode(placed(Mfsk.synthesizeWspr("K1ABC", "FN42", 37, 1500.0f), 1.0f, 120))
        check("WSPR decodes (${rows.texts()})", rows.any { it.text.contains("K1ABC FN42 37") })
        checkEq("and the row says WSPR", rows.firstOrNull()?.mode, wspr)
    }
    for (name in listOf("JT9", "JT65")) {
        val m = modes.firstOrNull { Mfsk.modeName(it) == name } ?: continue
        MfskDecoder.open(m).use { dec ->
            val rows = dec.decode(placed(Mfsk.synthesizeJt(m, "CQ", "K1ABC", "FN42", 1500.0f), 0.0f, 60))
            check("$name decodes (${rows.texts()})", rows.any { it.text.contains("CQ K1ABC FN42") })
        }
    }

    // ── Q65: the WSJT-X 3.2 settings, and the two lists it keeps ────
    //
    // Each setting has to change what is decoded: a field that is accepted and
    // ignored looks identical to one that works.
    val q30 = modes.firstOrNull { Mfsk.modeName(it) == "Q65-30A" }
    check("Q65-30A is addressable", q30 != null)
    if (q30 != null) {
        val qd = Mfsk.defaultParams(q30)
        val slotLen = Mfsk.modeInfo(q30).slotSamples12k
        fun slotOf(sig: FloatArray, startS: Float) = placed(sig, startS, slotLen / 12_000)
        val want = "K1ABC JA1ABC -15"
        val sig = Mfsk.synthesizeQ65(q30, "K1ABC", "JA1ABC", "-15", 1500.0f)
        val flaggedSig = Mfsk.synthesizeQ65(q30, "K1ABC", "JA1ABC", "-15", 1500.0f, copiedLastTx = true)
        check("the flag changes the transmission", !sig.contentEquals(flaggedSig))
        val window = MfskExtras(tEarlyS = 1.0f, tLateS = 1.0f)

        // dt is measured from the nominal start; the flag reaches the row.
        MfskDecoder.open(q30, qd, window).use { dec ->
            for ((startS, wantDt) in listOf(0.5f to 0.0f, 0.9f to 0.4f, 0.2f to -0.3f)) {
                val rows = dec.decode(slotOf(sig, startS))
                checkEq("the frame at $startS s decodes", rows.texts(), listOf(want))
                check("dt is $wantDt (got ${rows.firstOrNull()?.dtSec})",
                      rows.isNotEmpty() && Math.abs(rows[0].dtSec - wantDt) < 0.05f)
                checkEq("the row carries its mode", rows.firstOrNull()?.mode, q30)
                check("an unflagged frame is not flagged", rows.none { it.copiedLastTx })
            }
            val flagged = dec.decode(slotOf(flaggedSig, 0.5f))
            checkEq("the flagged frame decodes", flagged.texts(), listOf(want))
            check("and says it copied the last Tx", flagged.isNotEmpty() && flagged[0].copiedLastTx)

            // A frame 3 s late is beyond ±1 s.
            val late = slotOf(sig, 3.5f)
            check("a frame 3 s late is not in the window", dec.decode(late).isEmpty())
            dec.setParams(qd.copy(emeDelay = true))
            val eme = dec.decode(late)
            checkEq("the EME delay reaches it", eme.texts(), listOf(want))
            check("at dt +3 s (got ${eme.firstOrNull()?.dtSec})",
                  eme.isNotEmpty() && Math.abs(eme[0].dtSec - 3.0f) < 0.05f)
        }

        // Pileup: a MyCall + DxCall hint cannot match a flagged reply without it.
        val pileup = window.copy(
            apHint = MfskApHint(call1 = "K1ABC", call2 = "JA1ABC"), pileup = true,
        )
        MfskDecoder.open(q30, qd, pileup).use { dec ->
            val pile = dec.decode(slotOf(flaggedSig, 0.5f))
            checkEq("Pileup decodes a flagged reply under a MyCall + DxCall hint", pile.texts(), listOf(want))
            check("and flags it", pile.isNotEmpty() && pile[0].copiedLastTx)
            checkEq("an unflagged reply decodes under Pileup too",
                    dec.decode(slotOf(sig, 0.5f)).texts(), listOf(want))
        }

        // q3: a window that holds nothing else, so only the list decode at the
        // Rx frequency can find it.
        val q3 = qd.copy(
            band = 3900.0f..3950.0f, ap = MfskApMode.FULL,
            station = MfskStation("K1ABC"), qso = MfskQso("JA1ABC", "PM95"),
        )
        MfskDecoder.open(q30, q3, window).use { dec ->
            check("the scan window holds nothing", dec.decode(slotOf(sig, 0.5f)).isEmpty())
            dec.setParams(q3.copy(rxFreqHz = 1500.0f))
            checkEq("q3 finds the list message at the Rx frequency",
                    dec.decode(slotOf(sig, 0.5f)).texts(), listOf(want))
            dec.setParams(q3.copy(rxFreqHz = 1600.0f))
            check("and not when the Rx frequency is 100 Hz off", dec.decode(slotOf(sig, 0.5f)).isEmpty())
        }

        // The contest list: K1ABC W9XYZ RR73 is on it only because W9XYZ called.
        MfskQ65Callers().use { callers ->
            val rr73 = slotOf(Mfsk.synthesizeQ65(q30, "K1ABC", "W9XYZ", "RR73", 1500.0f), 0.5f)
            val contest = qd.copy(
                band = 3900.0f..3950.0f, ap = MfskApMode.FULL, rxFreqHz = 1500.0f,
                station = MfskStation("K1ABC"), contest = MfskContest.GRID_EXCHANGE,
            )
            MfskDecoder.open(q30, contest, window).use { dec ->
                dec.setQ65Callers(callers)
                check("nobody listed: nothing to find", dec.decode(rr73).isEmpty())
                callers.record(1500.0f, "K1ABC W9XYZ EN37", now = 1_000L)
                dec.setQ65Callers(callers) // copied at the call: set it again
                checkEq("a remembered caller decodes", dec.decode(rr73).texts(), listOf("K1ABC W9XYZ RR73"))
                dec.setExtras(window)
                checkEq("and survives setExtras", dec.decode(rr73).texts(), listOf("K1ABC W9XYZ RR73"))
                dec.setQ65Callers(null)
                check("removing the list removes the decode", dec.decode(rr73).isEmpty())
            }
            MfskDecoder.open(ft8).use { f ->
                refused<MfskUnsupportedException>("callers on a decoder that is not Q65", "Q65") {
                    f.setQ65Callers(callers)
                }
            }
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

    // ── Streams and the clock ───────────────────────────────────────
    //
    // A stream cuts slots on the UTC grid; the slot comes with its index, and a
    // clock reading is followed, not jumped to.
    MfskStream.open(ft8).use { s ->
        MfskDecoder.open(ft8).use { dec ->
            check("nothing is ready on a new stream", !s.slotReady && dec.decodeStream(s) == null)
            // A 15 s boundary plus a quarter of a second of lead-in, and the clock says so.
            val t0 = 1_700_000_010L * 1_000_000_000L
            s.push(ShortArray(3_000))
            checkEq("the first reading anchors the clock",
                    s.setTime(t0 - 250_000_000L, 0L), MfskClockChange.FIRST)
            checkEq("the stream counts its samples", s.position, 3_000L)
            // The recording from the next boundary on, in odd chunks.
            val audio = slot + ShortArray(12_000)
            var pos = 0
            while (pos < audio.size) {
                val end = minOf(pos + 7_777, audio.size)
                s.push(audio.copyOfRange(pos, end))
                pos = end
            }
            check("a whole slot is ready", s.slotReady)
            val got = dec.decodeStream(s)
            check("the stream's slot decodes (${got?.rows?.texts()})",
                  got != null && got.rows.any { it.text.contains("CQ JA1ABC PM95") })
            if (got != null) {
                checkEq("the period is the boundary the lead-in ends on", got.period, 1_700_000_010L / 15)
                checkEq("and its UTC start follows", got.slotStartUtcNs, got.period * 15_000_000_000L)
            }
            check("nothing more is ready", !s.slotReady && dec.decodeStream(s) == null)
            checkEq("and no slot was dropped", s.dropped, 0L)

            // Taking the audio instead gives the same slot.
            s.push(slot + ShortArray(12_000))
            val taken = s.takeSlot()
            check("takeSlot hands the slot over", taken != null && taken.first.size == slot.size)
            s.clear()
        }
        // A stream and a decoder of different modes do not mix.
        MfskDecoder.open(ft4).use { other ->
            refused<MfskInvalidArgException>("a stream of another mode", "different modes") {
                other.decodeStream(s)
            }
        }
    }
    refused<MfskException>("a stream for a mode that is not cut into slots", "mode") {
        MfskStream.open(modes.first { Mfsk.modeName(it) == "JTTY" })
    }

    // ── Wideband IQ ─────────────────────────────────────────────────
    //
    // One FT8 signal (CQ JA1ABC PM95 at audio 1500 Hz, 0.5 s into the slot)
    // in white noise, as double-sideband IQ at 48 kS/s.
    run {
        val fs = 48_000
        val center = 14_077_000.0
        val dial = center + 6_000.0
        val t0 = 1_700_000_100L * 1_000_000_000L
        val rng = java.util.Random(7)
        val n = 15 * fs + fs / 2
        val re = FloatArray(n)
        val im = FloatArray(n)
        val w = 2.0 * Math.PI * (dial - center) / fs
        for (i in 0 until n) {
            // 12 kHz audio up to 48 kHz by linear interpolation; the channel's
            // filter removes the images.
            val x = i / 4.0
            val k = x.toInt()
            val a = if (k < slot.size) slot[k].toDouble() else 0.0
            val b = if (k + 1 < slot.size) slot[k + 1].toDouble() else 0.0
            val v = (a + (b - a) * (x - k)) + 600.0 * rng.nextGaussian()
            re[i] = (v * Math.cos(w * i)).toFloat()
            im[i] = (v * Math.sin(w * i)).toFloat()
        }
        val peak = (re + im).maxOf { Math.abs(it) }
        val cf32 = java.nio.ByteBuffer.allocate(n * 8).order(java.nio.ByteOrder.LITTLE_ENDIAN)
        val cs16 = java.nio.ByteBuffer.allocate(n * 4).order(java.nio.ByteOrder.LITTLE_ENDIAN)
        for (i in 0 until n) {
            cf32.putFloat(re[i] * 0.7f / peak).putFloat(im[i] * 0.7f / peak)
            cs16.putShort((re[i] * 0.7f / peak * 32_768f).toInt().toShort())
                .putShort((im[i] * 0.7f / peak * 32_768f).toInt().toShort())
        }

        for ((format, bytes) in listOf(MfskIqFormat.CF32 to cf32.array(), MfskIqFormat.CS16 to cs16.array())) {
            MfskIqReceiver.open(fs, center, format).use { rx ->
                val ch = rx.addChannel(dial, ft8, Mfsk.defaultParams(ft8))
                check("the channel is active", rx.isActive(ch) == true)
                check("an unknown channel is null", rx.isActive(ch + 7) == null)
                checkEq("the first clock reading", rx.setTime(t0, 0L), MfskClockChange.FIRST)
                // Odd chunk sizes, so samples split across calls.
                var pos = 0
                while (pos < bytes.size) {
                    val end = minOf(pos + 65_537, bytes.size)
                    rx.push(bytes.copyOfRange(pos, end))
                    pos = end
                }
                checkEq("the receiver counted every sample ($format)", rx.samplesIn, n.toLong())
                checkEq("decodes wait to be polled", rx.pending > 0, true)
                val rows = rx.poll()
                checkEq("and polling drains them", rx.pending, 0)
                val hit = rows.firstOrNull { it.text == "CQ JA1ABC PM95" }
                check("$format: the injected message comes out (${rows.joinToString()})", hit != null)
                if (hit != null) {
                    checkEq("the row carries its channel", hit.channel, ch)
                    checkEq("and its mode", hit.mode, ft8)
                    check("at about 1500 Hz audio", Math.abs(hit.freqHz - 1500f) < 2f)
                    check("and the dial plus that as RF", Math.abs(hit.absFreqHz - (dial + hit.freqHz)) < 1e-3)
                    checkEq("on the anchored grid", hit.slotStartUtcNs, t0)
                    checkEq("from sample 0", hit.slotStartSample, 0L)
                }
                // The channel's decoder is borrowed for configuration.
                val cd = rx.channelDecoder(ch)!!
                cd.setParams(Mfsk.defaultParams(ft8).copy(rxFreqHz = 1500f))
                cd.addCallsign("VK3NV")
                refused<IllegalStateException>("decoding with a borrowed decoder", "IQ receiver") { cd.decode(slot) }
                cd.close()
                check("closing the borrowed decoder does nothing to the channel", rx.isActive(ch) == true)
                check("a missing channel has no decoder", rx.channelDecoder(ch + 7) == null)
                val (paused, resumed) = rx.retune(center + 1_000_000.0)
                checkEq("a far retune pauses the channel", paused, 1)
                checkEq("and resumes none", resumed, 0)
                check("which reads as paused", rx.isActive(ch) == false)
                rx.removeChannel(ch)
                refused<MfskInvalidArgException>("removing it twice", "channel") { rx.removeChannel(ch) }
            }
        }
        MfskIqReceiver.open(fs, center, MfskIqFormat.CF32).use { rx ->
            refused<MfskInvalidArgException>("a channel with DC inside its window", "") {
                rx.addChannel(center, ft8)
            }
            refused<MfskUnsupportedException>("a channel option the mode lacks", "noise blanker") {
                rx.addChannel(dial, ft8, null, MfskExtras(noiseBlanker = MfskNoiseBlanker.Percent(5)))
            }
        }
        refused<MfskInvalidArgException>("an IQ rate below 12 kHz", "sample_rate") {
            MfskIqReceiver.open(8_000, 0.0)
        }
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
