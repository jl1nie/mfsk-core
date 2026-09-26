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
