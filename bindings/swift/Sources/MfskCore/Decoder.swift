// SPDX-License-Identifier: GPL-3.0-only
//
// The decoder — `mfsk_decoder_*`. One handle for every slot mode: FT8, FT4,
// the five FST4 periods, WSPR, JT9, JT65 and the ten Q65 sub-modes. What it
// owns is what only means something across more than one call: the callsign
// hash table, a7's list, the averages — WSJT-X's state between periods.

import CMfsk

/// A decoder for one mode, carrying the state that spans periods.
///
/// **One per thread.** The class is deliberately not `Sendable`: a decode
/// mutates it, and the C handle is not synchronised. Open one per worker, or
/// serialise access yourself. (The ABI keeps its last-error string per handle
/// for exactly the async-hop reason; see ``lastError``.)
///
/// Modes with no slot (JTTY, see ``JttyReceiver``) and MSK144 have no
/// decoder: opening one throws ``MfskError/Code/unknownProtocol``.
public final class Decoder {
    let handle: OpaquePointer
    public let mode: Mode
    /// False for the decoder an ``IQReceiver`` channel lends out: the
    /// receiver owns it and closes it.
    private let ownsHandle: Bool
    /// Keeps the receiver alive for as long as a borrowed decoder is.
    private let owner: AnyObject?

    /// `period` for a decode whose slot index is not known; the decoder then
    /// leaves its period-to-period state (a7, averaging) alone. This is
    /// `MFSK_PERIOD_NONE` (`INT64_MIN`), written out because Clang does not
    /// import a macro that names another; `ABIContractTests` pins the value
    /// against the C side's behaviour rather than the macro.
    static let periodNone = Int64.min

    /// Open a decoder.
    ///
    /// - Parameters:
    ///   - params: nil for the mode's defaults.
    ///   - extras: nil for the depth's own values.
    ///
    /// **An option the mode does not support is an error here**, not a field
    /// silently dropped at decode time: ``MfskError/Code/unsupported`` with
    /// the option named in `detail`.
    public init(mode: Mode, params: DecodeParams? = nil, extras: Extras? = nil) throws {
        var status = MFSK_STATUS_OK
        let opened: OpaquePointer? = try withOptionalParams(params) { p in
            try withOptionalExtras(extras) { e in
                mfsk_decoder_open(mode.rawValue, p, e, &status)
            }
        }
        guard let opened else {
            throw MfskError(status: status, detail: globalLastError())
        }
        self.handle = opened
        self.mode = mode
        self.ownsHandle = true
        self.owner = nil
    }

    /// A decoder the C side owns, lent by `owner` (an ``IQReceiver``).
    init(borrowing handle: OpaquePointer, mode: Mode, owner: AnyObject) {
        self.handle = handle
        self.mode = mode
        self.ownsHandle = false
        self.owner = owner
    }

    deinit {
        // Clear the callbacks before closing (or, for a borrowed decoder,
        // before letting go): the C side holds raw pointers to the boxes
        // below, and this object's own deinit is the last moment at which
        // they are still valid.
        if callbackBox != nil { mfsk_decoder_set_on_decode(handle, nil, nil) }
        if budgetBox != nil { mfsk_decoder_set_budget(handle, nil, nil) }
        if ownsHandle { mfsk_decoder_close(handle) }
    }

    // MARK: Configuration

    /// Change the parameter block between periods, as the GUI rewrites it
    /// before each one. What the decoder carries between periods is kept.
    public func setParams(_ params: DecodeParams) throws {
        let status = try params.withC { mfsk_decoder_set_params(handle, $0) }
        try check(status, detail: failureDetail())
    }

    /// Replace the library's options. What the block leaves unset goes back
    /// to the depth's value; an option the mode lacks is
    /// ``MfskError/Code/unsupported`` and nothing changes.
    public func setExtras(_ extras: Extras) throws {
        let status = try extras.withC { mfsk_decoder_set_extras(handle, $0) }
        try check(status, detail: failureDetail())
    }

    /// Q65 only: the contest callers heard (`q65_hist2`), so that with
    /// ``DecodeParams/Contest/gridExchange`` they join the full-AP list. The
    /// list is **copied** at this call; call again after the list changes.
    /// nil removes it. Survives ``setExtras(_:)``.
    public func setQ65Callers(_ callers: Q65Callers?) throws {
        try check(mfsk_decoder_set_q65_callers(handle, callers?.handle), detail: failureDetail())
    }

    /// Forget everything the decoder carries between periods (WSJT-X's
    /// "Clear Avg" and `ndepth & 128`): the hash table, a7, the averages.
    public func clear() throws {
        try check(mfsk_decoder_clear(handle), detail: failureDetail())
    }

    /// Teach the decoder a callsign, so a later period's `<...>` reference to
    /// it resolves.
    ///
    /// Decoded messages populate the table by themselves; this is for
    /// callsigns known from outside — a band map, an operator's log. Throws
    /// ``MfskError/Code/unsupported`` for a mode whose messages carry no
    /// hashed calls (WSPR, JT9, JT65).
    public func addCallsign(_ call: String) throws {
        try check(mfsk_decoder_add_callsign(handle, call), detail: failureDetail())
    }

    // MARK: Per-row delivery

    /// Deliver decodes through `handler` as they are found, **in addition
    /// to** the array `decode` returns at the end of the call. Pass nil to
    /// stop.
    ///
    /// The returned array stays authoritative: a UI that wants to show rows
    /// during a 300 s FST4 slot uses this, and anything that wants the
    /// definitive set uses the return value.
    ///
    /// **Threading.** On a `desktop` build (rayon) the handler fires from
    /// worker threads, possibly several concurrently, in completion order.
    /// On a `mobile` build there is one thread and candidate order — a
    /// stronger contract. Write the handler to be safe under the weaker one:
    /// this binding does not serialise it for you.
    ///
    /// The handler is retained until it is replaced or the decoder is
    /// released, and it is cleared before the handle is closed, so it cannot
    /// fire into a deallocated closure.
    public func onDecode(_ handler: ((Decode) -> Void)?) throws {
        guard let handler else {
            try install(nil)
            return
        }
        try install(CallbackBox(handler))
    }

    /// Detach the Swift closures from the C side without closing anything.
    /// For ``IQReceiver``, which frees the C decoder under a still-live
    /// wrapper when its channel is removed.
    func releaseCallbacks() {
        if callbackBox != nil { mfsk_decoder_set_on_decode(handle, nil, nil) }
        if budgetBox != nil { mfsk_decoder_set_budget(handle, nil, nil) }
        callbackBox = nil
        budgetBox = nil
    }

    private func install(_ box: CallbackBox?) throws {
        let status: MfskStatus
        if let box {
            // Unretained on purpose: the box is owned by `callbackBox`
            // for exactly as long as the C side can call it, so passing a
            // retained pointer would leak it on every replacement.
            status = mfsk_decoder_set_on_decode(handle, decodeTrampoline,
                                                Unmanaged.passUnretained(box).toOpaque())
        } else {
            status = mfsk_decoder_set_on_decode(handle, nil, nil)
        }
        try check(status, detail: failureDetail())
        callbackBox = box
    }

    /// Keeps the handler alive while the C side holds a pointer to it.
    private var callbackBox: CallbackBox?

    /// Same, for the budget predicate.
    fileprivate var budgetBox: BudgetBox?

    /// Run `body` with `handler` standing in for the persistent one, then put
    /// the persistent one back.
    private func withHandler<R>(_ handler: ((Decode) -> Void)?, _ body: () throws -> R) throws -> R {
        guard let handler else { return try body() }
        let saved = callbackBox
        try install(CallbackBox(handler))
        defer { try? install(saved) }
        return try body()
    }

    // MARK: Errors

    /// The last error recorded **on this handle**.
    ///
    /// Prefer this over the thread-local global whenever a decoder is in
    /// hand: the global reads nil after an `async` caller hops threads
    /// between the failing call and the question.
    ///
    /// Not every failure lands here. `mfsk_decoder_copy_info` takes the
    /// handle as `const*`, so it has nowhere to write one and records only
    /// the thread-local — which is why every throw from this type falls back
    /// to that, rather than dropping the reason on the floor.
    public var lastError: String? {
        guard let p = mfsk_decoder_last_error(handle) else { return nil }
        return String(cString: p)
    }

    /// Whichever of the two error slots the failing call actually wrote.
    func failureDetail() -> String? { lastError ?? globalLastError() }

    // MARK: Decoding

    /// Decode one period of 16-bit PCM.
    ///
    /// - Parameters:
    ///   - sampleRate: anything but 12 000 is resampled.
    ///   - period: the period's index on the UTC grid (`UTC seconds / T`),
    ///     or nil when it is not known. A decoder given nil leaves its
    ///     period-to-period state (a7, averaging) alone.
    ///   - handler: called per row as it is found, for this call only (see
    ///     ``onDecode(_:)`` for the threading contract).
    public func decode(
        _ samples: [Int16],
        sampleRate: UInt32 = 12_000,
        period: Int64? = nil,
        handler: ((Decode) -> Void)? = nil
    ) throws -> [Decode] {
        // An empty Swift array may hand over a null base address, which the
        // ABI reads as a null pointer; nothing to decode either way.
        guard !samples.isEmpty else { return [] }
        return try withHandler(handler) {
            try samples.withUnsafeBufferPointer { audio in
                try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                    mfsk_decoder_decode_i16(handle, audio.baseAddress, UInt(audio.count),
                                            sampleRate, period ?? Decoder.periodNone,
                                            out, capacity, found)
                }
            }
        }
    }

    /// Decode one period of 32-bit float PCM, **any level**.
    ///
    /// At 12 kHz the float goes to the decoder as it is: the modes whose
    /// engines work in `float` (WSPR, JT9, JT65, Q65) never see 16 bits, and
    /// FT8, FT4 and FST4, which take 16-bit audio as WSJT-X does, get it
    /// scaled to a fixed level, so a caller never picks one. At another rate
    /// it is resampled and peak-normalised first.
    public func decode(
        _ samples: [Float],
        sampleRate: UInt32 = 12_000,
        period: Int64? = nil,
        handler: ((Decode) -> Void)? = nil
    ) throws -> [Decode] {
        guard !samples.isEmpty else { return [] }
        return try withHandler(handler) {
            try samples.withUnsafeBufferPointer { audio in
                try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                    mfsk_decoder_decode_f32(handle, audio.baseAddress, UInt(audio.count),
                                            sampleRate, period ?? Decoder.periodNone,
                                            out, capacity, found)
                }
            }
        }
    }

    /// Decode the period so far, keeping what this period has already found
    /// (#572). Call it as audio arrives with **every sample of the period
    /// received up to now** and the period's index; the decoder infers the
    /// stage from the length. FT8 returns checkpoint A's rows at 141 696
    /// samples (~11.8 s, ``Decode/Stage/early``), nothing at 162 432, and the
    /// period's complete set at 180 000 — the rows ``decode(_:sampleRate:period:handler:)``
    /// gives for the same audio. Other calls, and every call of a mode with no
    /// early decode before the whole period, return nothing. ``onDecode(_:)``
    /// sees each row once across the period. A nil period makes it a plain
    /// decode.
    public func decodePrefix(
        _ samples: [Int16],
        sampleRate: UInt32 = 12_000,
        period: Int64?,
        handler: ((Decode) -> Void)? = nil
    ) throws -> [Decode] {
        guard !samples.isEmpty else { return [] }
        return try withHandler(handler) {
            try samples.withUnsafeBufferPointer { audio in
                try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                    mfsk_decoder_decode_prefix_i16(handle, audio.baseAddress, UInt(audio.count),
                                                   sampleRate, period ?? Decoder.periodNone,
                                                   out, capacity, found)
                }
            }
        }
    }

    /// ``decodePrefix(_:sampleRate:period:handler:)`` for float PCM at any
    /// level; the first prefix of a period sets the gain for the rest of it.
    public func decodePrefix(
        _ samples: [Float],
        sampleRate: UInt32 = 12_000,
        period: Int64?,
        handler: ((Decode) -> Void)? = nil
    ) throws -> [Decode] {
        guard !samples.isEmpty else { return [] }
        return try withHandler(handler) {
            try samples.withUnsafeBufferPointer { audio in
                try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                    mfsk_decoder_decode_prefix_f32(handle, audio.baseAddress, UInt(audio.count),
                                                   sampleRate, period ?? Decoder.periodNone,
                                                   out, capacity, found)
                }
            }
        }
    }

    /// What the stream decode found, and which slot it
    /// was.
    public struct SlotDecode: Sendable, Equatable {
        public let decodes: [Decode]
        /// The slot's index on the mode's UTC grid — what the decoder was
        /// given as the period.
        public let period: Int64
        /// UTC of the slot's first sample, nanoseconds since the Unix epoch,
        /// or nil when the stream has no clock (a free-running grid).
        public let slotStartUTCNanoseconds: Int64?

        /// ``slotStartUTCNanoseconds`` in seconds.
        public var slotStartUTC: Double? {
            slotStartUTCNanoseconds.map { Double($0) / 1e9 }
        }
    }

    /// Decode the stream's ready slot in place, or nil if no slot is ready
    /// yet — so a caller can poll this instead of asking
    /// ``CaptureStream/isSlotReady`` first.
    ///
    /// Fused on purpose: FST4-300's slot is 3 600 000 samples, and taking it
    /// out only to hand it back moves 7 MB for nothing. The slot's own index
    /// is the period. The stream must have been opened for this decoder's
    /// mode; otherwise ``MfskError/Code/invalidArgument``.
    ///
    /// On a stream with ``CaptureStream/setPrefixPoints(_:)`` a prefix is
    /// decoded as ``decodePrefix(_:sampleRate:period:handler:)`` would:
    /// checkpoint A's rows come back at ~11.8 s with ``Decode/Stage/early``.
    public func decode(
        _ stream: CaptureStream,
        handler: ((Decode) -> Void)? = nil
    ) throws -> SlotDecode? {
        var period: Int64 = 0
        var utc: Int64 = 0
        do {
            let rows = try withHandler(handler) {
                try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                    mfsk_decoder_decode_stream(handle, stream.handle, out, capacity, found,
                                               &period, &utc)
                }
            }
            return SlotDecode(decodes: rows, period: period,
                              slotStartUTCNanoseconds: utc == 0 ? nil : utc)
        } catch let error as MfskError where error.code == .unsupported {
            // The documented "no slot ready yet" answer.
            return nil
        }
    }

    /// The FEC information bits of the `index`-th row of the last decode.
    /// ``Decode/informationBitCount`` says how many there are.
    ///
    /// Deliberately not a row field: 91 or 101 bytes that only a caller doing
    /// subtraction or persistence wants.
    public func informationBits(at index: Int) throws -> [UInt8] {
        // 101 (CRC-24) is the widest the ABI documents; ask for 128 so a
        // wider block would come back as a short-buffer error rather than a
        // silently truncated answer.
        var bits = [UInt8](repeating: 0, count: 128)
        var written: UInt = 0
        let status = bits.withUnsafeMutableBufferPointer { buffer in
            mfsk_decoder_copy_info(handle, UInt(index), buffer.baseAddress, UInt(buffer.count), &written)
        }
        try check(status, detail: failureDetail())
        return Array(bits.prefix(Int(written)))
    }
}

/// The Swift closure a `MfskDecodeCallback` reaches, boxed so it has a
/// stable address to hand across the boundary.
final class CallbackBox {
    let handler: (Decode) -> Void
    init(_ handler: @escaping (Decode) -> Void) { self.handler = handler }
}

/// The C function pointer itself. A top-level function, not a closure: only
/// a capture-free closure converts to `@convention(c)`, and the state travels
/// through `user_data` instead.
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
/// All-zero — `exhausted == false` — when no budget was set or it was never
/// reached, so it can be read unconditionally.
public struct BudgetReport: Sendable, Equatable {
    /// The predicate refused at least once: rows are missing that an
    /// unbudgeted call would have found.
    public let exhausted: Bool
    /// Units of work declined: a candidate on the single-pass engines and
    /// every sniper, a whole SIC round on a rounds strategy.
    public let candidatesSkipped: UInt32
    /// Units of work actually run, counted the same way.
    public let stagesRun: UInt32
    /// Costas sync quality of the best skipped candidate — **FT8 only**,
    /// because that is the number FT8's scheduler orders by, so it says
    /// whether the cut took noise or a station. nil on FT4 and FST4, which
    /// rank by score.
    public let cutAtSync: UInt32?
    /// Sync score of the best skipped candidate, on that protocol's own
    /// scale. nil when there was no such candidate.
    public let cutAtScore: Float?
    /// Rows subtracted before a later search saw them: FT8
    /// ``Extras/Strategy/sicEarly``'s checkpoint-B and -C loops. Fewer than
    /// the rows returned, with ``exhausted`` set, means the cut came while
    /// cleaning up rather than while searching. 0 everywhere else.
    public let rowsSubtracted: UInt32

    init(_ raw: MfskBudgetReport) {
        self.exhausted = raw.exhausted
        self.candidatesSkipped = raw.candidates_skipped
        self.stagesRun = raw.stages_run
        self.cutAtSync = raw.cut_at_sync >= 0 ? UInt32(raw.cut_at_sync) : nil
        self.cutAtScore = raw.cut_at_score.isNaN ? nil : raw.cut_at_score
        self.rowsSubtracted = raw.rows_subtracted
    }
}

extension Decoder {
    /// Poll `check` during every subsequent decode; returning `false` stops
    /// the search and returns what was found. Pass nil to remove the budget.
    ///
    /// **The library reads no clock** — `Instant::now` does not exist on
    /// every target it builds for — so the deadline is yours:
    /// ```swift
    /// let deadline = DispatchTime.now().uptimeNanoseconds + 200_000_000
    /// try decoder.setBudget { DispatchTime.now().uptimeNanoseconds < deadline }
    /// ```
    ///
    /// **The triage sweep is a floor**: it is never gated, so a budget
    /// shorter than it returns nothing *and still spends that time*.
    /// ``Extras/maxCandidates`` is the knob that moves the floor.
    ///
    /// The predicate is polled from rayon workers on a `desktop` build, so it
    /// must be safe to call concurrently — a captured deadline compared
    /// against a clock is, which is the shape this is for.
    ///
    /// Every mode with a decoder takes one (``Capabilities/budget``); WSPR,
    /// JT9, JT65 and Q65 report only ``BudgetReport/exhausted``.
    public func setBudget(_ check: (() -> Bool)?) throws {
        guard let check else {
            try self.check(mfsk_decoder_set_budget(handle, nil, nil))
            budgetBox = nil
            return
        }
        let box = BudgetBox(check)
        let status = mfsk_decoder_set_budget(handle, budgetTrampoline,
                                             Unmanaged.passUnretained(box).toOpaque())
        guard status == MFSK_STATUS_OK else {
            throw MfskError(status: status, detail: failureDetail())
        }
        budgetBox = box
    }

    /// Whether a decode with the current mode, depth and extras hands
    /// ``onDecode(_:)`` exactly the rows it returns, once each and in order
    /// (`STREAMING.md` §3a). false is completion order with a transient
    /// duplicate possible (§3b): FT8's single pass and sniper, FT4 at
    /// ``DecodeParams/Depth/fast``, FST4, WSPR. Pair by ``Decode/delivery``
    /// either way. Ask again after changing the parameters or the extras.
    public var deliveryIsExact: Bool {
        mfsk_decoder_delivery_is_exact(handle)
    }

    /// The prefix lengths, in 12 kHz samples, at which
    /// ``decodePrefix(_:sampleRate:period:handler:)`` does work before the
    /// whole period under the current settings: `[141_696, 162_432]` for FT8
    /// at normal or deep depth, empty otherwise. Hand them to
    /// ``CaptureStream/setPrefixPoints(_:)``; ask again after changing the
    /// parameters or the extras.
    public var prefixPoints: [Int] {
        var buffer = [UInt](repeating: 0, count: 16)
        var count: UInt = 0
        let status = buffer.withUnsafeMutableBufferPointer {
            mfsk_decoder_prefix_points(handle, $0.baseAddress, UInt($0.count), &count)
        }
        guard status == MFSK_STATUS_OK else { return [] }
        return buffer.prefix(Int(count)).map { Int($0) }
    }

    /// What the budget cut short on the **last** decode.
    public var lastBudget: BudgetReport {
        var raw = MfskBudgetReport()
        raw.size = UInt32(MemoryLayout<MfskBudgetReport>.size)
        _ = mfsk_decoder_last_budget(handle, &raw)
        return BudgetReport(raw)
    }

    private func check(_ status: MfskStatus, detail: @autoclosure () -> String? = nil) throws {
        guard status != MFSK_STATUS_OK else { return }
        throw MfskError(status: status, detail: detail() ?? failureDetail())
    }
}

/// `params.withC`, or `body(nil)` where the ABI reads NULL as "the mode's
/// defaults".
func withOptionalParams<R>(
    _ params: DecodeParams?,
    _ body: (UnsafePointer<MfskParams>?) throws -> R
) throws -> R {
    guard let params else { return try body(nil) }
    return try params.withC { try body($0) }
}

/// As ``withOptionalParams(_:_:)``, for the extras block.
func withOptionalExtras<R>(
    _ extras: Extras?,
    _ body: (UnsafePointer<MfskExtras>?) throws -> R
) throws -> R {
    guard let extras else { return try body(nil) }
    return try extras.withC { try body($0) }
}
