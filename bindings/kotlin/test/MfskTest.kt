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

    if (failures == 0) {
        println("\nALL OK")
    } else {
        System.err.println("\n$failures FAILURE(S)")
        System.exit(1)
    }
}
