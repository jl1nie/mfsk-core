// SPDX-License-Identifier: GPL-3.0-or-later
//
// The decode session — `mfsk_session_*`. "Session" rather than
// "decoder" because of what it owns: a callsign hash table and the
// previous slot's rows, both of which only mean anything across more
// than one call.

import CMfsk

/// A decoder for one mode, carrying the state that spans slots.
///
/// Modes with ``Capabilities/decodeHandle`` decode through this. The
/// others have their own entry points — ``WSPR``, ``JT9``, ``JT65`` —
/// because they are shaped differently, not because they are lesser.
public final class DecodeSession {
    let handle: OpaquePointer
    public let mode: Mode

    /// Open a session, validated against `params`.
    ///
    /// **A parameter the mode does not support is an error here**, not a
    /// field silently dropped at decode time.
    public init(mode: Mode, params: DecodeParams? = nil) throws {
        var status = MFSK_STATUS_OK
        var opened: OpaquePointer?
        if let params {
            opened = params.withC { mfsk_session_open(mode.rawValue, $0, &status) }
        } else {
            opened = mfsk_session_open(mode.rawValue, nil, &status)
        }
        guard let opened else {
            throw MfskError(status: status, detail: globalLastError())
        }
        self.handle = opened
        self.mode = mode
    }

    deinit {
        // Clear the callback before closing: the C side holds a raw
        // pointer to `callbackBox`, and this object's own deinit is the
        // last moment at which that pointer is still valid.
        if callbackBox != nil { mfsk_session_set_on_decode(handle, nil, nil) }
        if budgetBox != nil { mfsk_session_set_budget(handle, nil, nil) }
        mfsk_session_close(handle)
    }

    /// Deliver decodes through `handler` as they are found, **in
    /// addition to** the array `decode(_:)` returns at the end of the
    /// call. Pass nil to stop.
    ///
    /// The returned array stays authoritative: a UI that wants to show
    /// rows during a 300 s FST4 slot uses this, and anything that wants
    /// the definitive set uses the return value.
    ///
    /// **Threading.** On a `desktop` build (rayon) the handler fires
    /// from worker threads, possibly several concurrently, in
    /// completion order. On a `mobile` build there is one thread and
    /// candidate order — a *stronger* contract, and the honest answer
    /// to what dropping rayon costs. Write the handler to be safe under
    /// the weaker one: this binding does not serialise it for you,
    /// because a lock here would be invisible overhead for the single
    /// threaded build and the wrong lock for most callers on the other.
    ///
    /// The handler is retained until it is replaced or the session is
    /// released, and it is cleared before the handle is closed — so it
    /// cannot fire into a deallocated closure.
    public func onDecode(_ handler: ((Decode) -> Void)?) throws {
        guard let handler else {
            try check(mfsk_session_set_on_decode(handle, nil, nil), detail: failureDetail())
            callbackBox = nil
            return
        }
        let box = CallbackBox(handler)
        // Unretained on purpose: the box is owned by `callbackBox`
        // below for exactly as long as the C side can call it, so
        // passing a retained pointer would leak it on every replacement.
        let context = Unmanaged.passUnretained(box).toOpaque()
        let status = mfsk_session_set_on_decode(handle, decodeTrampoline, context)
        guard status == MFSK_STATUS_OK else {
            throw MfskError(status: status, detail: failureDetail())
        }
        callbackBox = box
    }

    /// Keeps the handler alive while the C side holds a pointer to it.
    private var callbackBox: CallbackBox?

    /// Same, for the budget predicate.
    fileprivate var budgetBox: BudgetBox?

    /// The last error recorded **on this handle**.
    ///
    /// Prefer this over the thread-local global whenever a session is in
    /// hand: the global reads nil after an `async` caller hops threads
    /// between the failing call and the question.
    ///
    /// Not every failure lands here. `mfsk_session_copy_info` takes the
    /// handle as `const*`, so it has nowhere to write one and records
    /// only the thread-local — which is why every throw from this type
    /// falls back to that, rather than dropping the reason on the floor.
    public var lastError: String? {
        guard let p = mfsk_session_last_error(handle) else { return nil }
        return String(cString: p)
    }

    /// Whichever of the two error slots the failing call actually wrote.
    func failureDetail() -> String? { lastError ?? globalLastError() }

    /// Decode one slot of 16-bit PCM.
    ///
    /// `params` overrides the session's own for this call only, and is
    /// validated the same way.
    public func decode(
        _ samples: [Int16],
        sampleRate: UInt32 = 12_000,
        params: DecodeParams? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                withParams(params) { p in
                    mfsk_session_decode_i16(handle, audio.baseAddress, UInt(audio.count),
                                            sampleRate, p, out, capacity, found)
                }
            }
        }
    }

    /// Decode one slot of 32-bit float PCM, nominally `-1.0...1.0`.
    ///
    /// Not a lossier wrapper: the float path resamples in float and
    /// converts once at the end, where the decoders take i16 anyway.
    public func decode(
        _ samples: [Float],
        sampleRate: UInt32 = 12_000,
        params: DecodeParams? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                withParams(params) { p in
                    mfsk_session_decode_f32(handle, audio.baseAddress, UInt(audio.count),
                                            sampleRate, p, out, capacity, found)
                }
            }
        }
    }

    /// Decode the stream's buffered slot in place, or nil if no slot is
    /// ready yet.
    ///
    /// Fused on purpose: FST4-300's slot is 3 600 000 samples, and
    /// taking it out only to hand it back moves 7 MB for nothing.
    public func decode(
        _ stream: CaptureStream,
        params: DecodeParams? = nil
    ) throws -> (decodes: [Decode], slotStartUTC: Double)? {
        var slotStart: Double = 0
        do {
            let rows = try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                withParams(params) { p in
                    mfsk_session_decode_stream(handle, stream.handle, p, out, capacity, found, &slotStart)
                }
            }
            return (rows, slotStart)
        } catch let error as MfskError where error.code == .unsupported {
            // The documented "no slot ready yet" answer, so a caller can
            // poll this instead of asking `slotReady` first.
            return nil
        }
    }

    /// Teach the session a callsign, so a later slot's `<...>` reference
    /// to it resolves.
    ///
    /// Decoded messages populate the table by themselves; this is for
    /// callsigns known from outside — a band map, an operator's log.
    public func addCallsign(_ call: String) throws {
        try check(mfsk_session_add_callsign(handle, call), detail: failureDetail())
    }

    /// The FEC information bits of the `index`-th row of the last
    /// decode. `Decode.informationBitCount` says how many there are.
    ///
    /// Deliberately not a row field: 91 or 101 bytes that only a caller
    /// doing subtraction or persistence wants.
    public func informationBits(at index: Int) throws -> [UInt8] {
        // 101 (CRC-24) is the widest the ABI documents; ask for 128 so a
        // wider block would come back as a short-buffer error rather
        // than a silently truncated answer.
        var bits = [UInt8](repeating: 0, count: 128)
        var written: UInt = 0
        let status = bits.withUnsafeMutableBufferPointer { buffer in
            mfsk_session_copy_info(handle, UInt(index), buffer.baseAddress, UInt(buffer.count), &written)
        }
        try check(status, detail: failureDetail())
        return Array(bits.prefix(Int(written)))
    }

    private func withParams<R>(
        _ params: DecodeParams?,
        _ body: (UnsafePointer<MfskDecodeParams>?) -> R
    ) -> R {
        guard let params else { return body(nil) }
        return params.withC { body($0) }
    }
}


/// The Swift closure a `MfskDecodeCallback` reaches, boxed so it has a
/// stable address to hand across the boundary.
final class CallbackBox {
    let handler: (Decode) -> Void
    init(_ handler: @escaping (Decode) -> Void) { self.handler = handler }
}

/// The C function pointer itself. A top-level function, not a closure:
/// only a capture-free closure converts to `@convention(c)`, and the
/// state travels through `user_data` instead.
private let decodeTrampoline: MfskDecodeCallback = { row, context in
    guard let row, let context else { return }
    // The row pointer is valid only for the duration of this call, so
    // `Decode.init` copying every field — including the text out of the
    // fixed-size C array — is what makes the value safe to keep.
    Unmanaged<CallbackBox>.fromOpaque(context).takeUnretainedValue().handler(Decode(row.pointee))
}

/// A budget predicate, boxed for the same reason the decode callback is.
final class BudgetBox {
    let check: () -> Bool
    init(_ check: @escaping () -> Bool) { self.check = check }
}

private let budgetTrampoline: MfskBudgetCheck = { context in
    guard let context else { return true }
    return Unmanaged<BudgetBox>.fromOpaque(context).takeUnretainedValue().check()
}

/// What a budgeted decode left undone.
///
/// All-zero — `exhausted == false` — when no budget was set or it was
/// never reached, so it can be read unconditionally.
public struct BudgetReport: Sendable, Equatable {
    /// The predicate refused at least once: rows are missing that an
    /// unbudgeted call would have found.
    public let exhausted: Bool
    /// Units of work declined: a candidate on the single-pass engines
    /// and every sniper, a whole SIC round on `sicRounds`.
    public let candidatesSkipped: UInt32
    /// Units of work actually run, counted the same way.
    public let stagesRun: UInt32
    /// Costas sync quality of the best skipped candidate — **FT8
    /// only**, because that is the number FT8's scheduler orders by, so
    /// it says whether the cut took noise or a station. nil on FT4 and
    /// FST4, which rank by score.
    public let cutAtSync: UInt32?
    /// Sync score of the best skipped candidate, on that protocol's own
    /// scale. nil when there was no such candidate.
    public let cutAtScore: Float?

    init(_ raw: MfskBudgetReport) {
        self.exhausted = raw.exhausted
        self.candidatesSkipped = raw.candidates_skipped
        self.stagesRun = raw.stages_run
        self.cutAtSync = raw.cut_at_sync >= 0 ? UInt32(raw.cut_at_sync) : nil
        self.cutAtScore = raw.cut_at_score.isNaN ? nil : raw.cut_at_score
    }
}

extension DecodeSession {
    /// Poll `check` during every subsequent decode; returning `false`
    /// stops the search and returns what was found. Pass nil to remove
    /// the budget.
    ///
    /// **The library reads no clock** — `Instant::now` does not exist on
    /// every target it builds for — so the deadline is yours:
    /// ```swift
    /// let deadline = DispatchTime.now().uptimeNanoseconds + 200_000_000
    /// try session.setBudget { DispatchTime.now().uptimeNanoseconds < deadline }
    /// ```
    ///
    /// **The triage sweep is a floor**: it is never gated, so a budget
    /// shorter than it returns nothing *and still spends that time*.
    /// `maxCandidates` is the knob that moves the floor.
    ///
    /// The predicate is polled from rayon workers on a `desktop` build,
    /// so it must be safe to call concurrently — a captured deadline
    /// compared against a clock is, which is the shape this is for.
    ///
    /// Throws ``MfskError/Code/unsupported`` for a mode without
    /// ``Capabilities/budget``.
    public func setBudget(_ check: (() -> Bool)?) throws {
        guard let check else {
            try self.check(mfsk_session_set_budget(handle, nil, nil))
            budgetBox = nil
            return
        }
        let box = BudgetBox(check)
        let status = mfsk_session_set_budget(handle, budgetTrampoline,
                                             Unmanaged.passUnretained(box).toOpaque())
        guard status == MFSK_STATUS_OK else {
            throw MfskError(status: status, detail: failureDetail())
        }
        budgetBox = box
    }

    /// What the budget cut short on the **last** decode.
    public var lastBudget: BudgetReport {
        var raw = MfskBudgetReport()
        raw.size = UInt32(MemoryLayout<MfskBudgetReport>.size)
        _ = mfsk_session_last_budget(handle, &raw)
        return BudgetReport(raw)
    }

    /// Carry each decode's results into the next as **known** signals:
    /// skipped rather than re-reported, and subtracted from the audio
    /// where the mode publishes ``Capabilities/knownSubtract``, so what
    /// they were masking can surface.
    ///
    /// `false` both stops carrying and drops what is held, which is how
    /// a new slot starts. Throws ``MfskError/Code/unsupported`` for a
    /// mode without ``Capabilities/knownFilter``.
    public func keepKnown(_ keep: Bool) throws {
        try check(mfsk_session_keep_known(handle, keep), detail: failureDetail())
    }

    /// How many known signals the session is carrying into the next
    /// decode.
    public var knownCount: Int { Int(mfsk_session_known_count(handle)) }

    /// Keep the slot FFT and reuse it for the next decode **of the same
    /// audio** — FST4-300's transform is 4 194 304 points, so a second
    /// pass over one slot is where this pays.
    ///
    /// Reuse is checked rather than trusted: the cache is stored with a
    /// fingerprint of the audio it came from, and a decode of anything
    /// else transforms afresh instead of returning a confident wrong
    /// answer.
    public func keepFFTCache(_ keep: Bool) throws {
        try check(mfsk_session_keep_fft_cache(handle, keep), detail: failureDetail())
    }

    private func check(_ status: MfskStatus, detail: @autoclosure () -> String? = nil) throws {
        guard status != MFSK_STATUS_OK else { return }
        throw MfskError(status: status, detail: detail() ?? failureDetail())
    }
}
