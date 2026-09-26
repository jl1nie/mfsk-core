// SPDX-License-Identifier: GPL-3.0-or-later
//
// Q65 with WSJT-X 3.2's settings — `mfsk_q65_decode_ex` — and the two lists
// WSJT-X keeps for it.
//
// The four strategy functions in `Q65.swift` pick a strategy by name and scan a
// fixed, deliberately wide window. Pileup, Max Drift, the EME delay and the q3
// list decode are combinations of them, so there is one call and a
// size-versioned parameter struct instead: ``Q65/Params``.

import CMfsk

extension Q65 {
    /// The fast-fading metric.
    public struct Fading: Sendable, Equatable {
        /// Spread bandwidth × symbol period. Typical values: 0.05
        /// near-AWGN, 1.0 moderate, 5.0+ severe.
        public var b90Ts: Float
        public var model: Q65FadingModel

        public init(b90Ts: Float, model: Q65FadingModel = .gaussian) {
            self.b90Ts = b90Ts
            self.model = model
        }
    }

    /// Full-AP list decoding. With ``Params/rxFrequencyHz`` it is WSJT-X's
    /// **q3** decode, run first at the Rx frequency; without it, template
    /// matching at every coarse candidate.
    public enum List: Sendable, Equatable {
        /// The standard QSO list for `myCall` and the DX station.
        case standard(myCall: String, hisCall: String, hisGrid: String? = nil)
        /// The **contest list** (`q65_set_list2`): every caller in the
        /// ``Q65Callers`` passed to ``Q65/decode(_:mode:params:callers:sampleRate:hashTable:)``,
        /// and the DX station too if `hisCall` is given.
        case contest(myCall: String, hisCall: String? = nil, hisGrid: String? = nil)
    }

    /// Everything a Q65 decode can be asked to do — `MfskQ65Params` in the C
    /// ABI.
    ///
    /// **Start from ``init(mode:)`` and change what you want.** Zeroing the
    /// fields is not the mode's defaults: a zero `maxCandidates` or an empty
    /// band decodes nothing. A combination the engine would quietly not honour
    /// — a list with fading, an Rx frequency with no list, Pileup with no AP
    /// hint, Max Drift with fading — is refused by the decode with the reason,
    /// not dropped.
    public struct Params: Sendable, Equatable {
        public var frequencyRangeHz: ClosedRange<Float>
        /// Where `dt = 0` is in the buffer, seconds — the mode's
        /// `tx_start_offset_s` by default, right for a buffer that begins at
        /// the slot boundary. It also places the period Max Drift normalises
        /// over and the q3 decode's slot start, so it has to be true and not
        /// merely convenient.
        public var nominalStartSeconds: Float
        /// How far before / after ``nominalStartSeconds`` a frame may start.
        /// Default ±1 s, WSJT-X's own window.
        public var earlyToleranceSeconds: Float
        public var lateToleranceSeconds: Float
        /// Coarse-sync acceptance, a fraction of sync plus noise (0..1).
        public var syncThreshold: Float
        public var maxCandidates: UInt32
        /// **Q65 Pileup**: an AP hint naming both callsigns and nothing after
        /// them leaves the spare 78th bit free, so a reply carrying the
        /// "copied last Tx" flag still matches. Needs ``apHint``.
        public var pileup: Bool
        /// **EME delay** ("Decode at 52 s"): the late edge reaches +5.5 s
        /// (+4.0 s on Q65-15) instead of ``lateToleranceSeconds``.
        public var emeDelay: Bool
        /// **Max Drift**, spectrum bins `0...50`; 0 is off. Costs `2*bins+1`
        /// times the plain search, so narrow ``frequencyRangeHz`` to match.
        public var maxDrift: UInt32
        /// The Rx frequency the q3 decode looks around, or nil. Needs ``list``.
        public var rxFrequencyHz: Float?
        /// WSJT-X's F Tol around ``rxFrequencyHz``.
        public var ftolHz: Float
        public var fading: Fading?
        public var list: List?
        public var apHint: APHint?

        /// `mode`'s defaults: the library's own (±1 s window, threshold 0.1, 8
        /// candidates) and the mode's nominal start. Throws if `mode` is not a
        /// Q65 mode in this build.
        public init(mode: Mode) throws {
            var raw = MfskQ65Params()
            raw.size = UInt32(MemoryLayout<MfskQ65Params>.size)
            try check(mfsk_q65_params_init(mode.rawValue, &raw))
            self.init(raw)
        }

        init(_ raw: MfskQ65Params) {
            self.frequencyRangeHz = raw.freq_min_hz...max(raw.freq_min_hz, raw.freq_max_hz)
            self.nominalStartSeconds = raw.nominal_start_s
            self.earlyToleranceSeconds = raw.t_early_s
            self.lateToleranceSeconds = raw.t_late_s
            self.syncThreshold = raw.score_threshold
            self.maxCandidates = raw.max_cand
            self.pileup = raw.pileup != 0
            self.emeDelay = raw.eme_delay != 0
            self.maxDrift = raw.max_drift
            // NaN is the ABI's spelling of "unset": 0 Hz is a frequency.
            self.rxFrequencyHz = raw.rx_freq_hz.isNaN ? nil : raw.rx_freq_hz
            self.ftolHz = raw.ftol_hz
            self.fading = raw.fading_b90_ts.isNaN
                ? nil
                : Fading(b90Ts: raw.fading_b90_ts,
                         model: Q65FadingModel(rawValue: raw.fading_model) ?? .gaussian)
            // `mfsk_q65_params_init` never writes a list or a hint.
            self.list = nil
            self.apHint = nil
        }

        /// Marshal into the C struct for the duration of `body`.
        func withC<R>(_ body: (UnsafePointer<MfskQ65Params>) throws -> R) rethrows -> R {
            var raw = MfskQ65Params()
            raw.size = UInt32(MemoryLayout<MfskQ65Params>.size)
            raw.freq_min_hz = frequencyRangeHz.lowerBound
            raw.freq_max_hz = frequencyRangeHz.upperBound
            raw.nominal_start_s = nominalStartSeconds
            raw.t_early_s = earlyToleranceSeconds
            raw.t_late_s = lateToleranceSeconds
            raw.score_threshold = syncThreshold
            raw.max_cand = maxCandidates
            raw.pileup = pileup ? 1 : 0
            raw.eme_delay = emeDelay ? 1 : 0
            raw.max_drift = maxDrift
            raw.rx_freq_hz = rxFrequencyHz ?? Float.nan
            raw.ftol_hz = ftolHz
            raw.fading_b90_ts = fading?.b90Ts ?? Float.nan
            raw.fading_model = (fading?.model ?? .gaussian).rawValue
            if let hint = apHint {
                raw.has_ap_hint = 1
                setCArray(&raw.ap_call1, to: hint.call1 ?? "")
                setCArray(&raw.ap_call2, to: hint.call2 ?? "")
                setCArray(&raw.ap_grid, to: hint.grid ?? "")
                setCArray(&raw.ap_report, to: hint.report ?? "")
            }
            switch list {
            case .none:
                break
            case .standard(let my, let his, let grid):
                raw.ap_list = 1
                setCArray(&raw.list_my_call, to: my)
                setCArray(&raw.list_his_call, to: his)
                setCArray(&raw.list_his_grid, to: grid ?? "")
            case .contest(let my, let his, let grid):
                raw.ap_list = 2
                setCArray(&raw.list_my_call, to: my)
                setCArray(&raw.list_his_call, to: his ?? "")
                setCArray(&raw.list_his_grid, to: grid ?? "")
            }
            return try withUnsafePointer(to: &raw) { try body($0) }
        }
    }

    /// Decode one Q65 slot with every setting WSJT-X 3.2 offers — Pileup, Max
    /// Drift, the EME delay, the q3 list decode, the contest list — in one
    /// call.
    ///
    /// Takes a ``Mode`` (a Q65 one), not a ``Q65SubMode``, and reports
    /// ``Decode/dtSeconds`` from ``Params/nominalStartSeconds`` as WSJT-X's DT
    /// column does — the four strategy functions report `start_sample /
    /// 12000`. Throws, with the reason, if the settings cannot be honoured
    /// together.
    ///
    /// - Parameters:
    ///   - params: nil for ``Params/init(mode:)``.
    ///   - callers: read only by ``List/contest(myCall:hisCall:hisGrid:)``.
    public static func decode(
        _ samples: [Float],
        mode: Mode,
        params: Params? = nil,
        callers: Q65Callers? = nil,
        sampleRate: UInt32 = 12_000,
        hashTable: CallsignHashTable? = nil
    ) throws -> [Decode] {
        let params = try params ?? Params(mode: mode)
        return try params.withC { p in
            try samples.withUnsafeBufferPointer { audio in
                try collectRows { out, capacity, found in
                    mfsk_q65_decode_ex(mode.rawValue, audio.baseAddress, UInt(audio.count),
                                       sampleRate, p, callers?.handle, hashTable?.handle,
                                       out, capacity, found)
                }
            }
        }
    }

    /// ``encode(subMode:call1:call2:gridOrReport:frequencyHz:)`` with Pileup's
    /// "copied last Tx" flag: `copiedLastTx` sets the spare 78th payload bit,
    /// which a Pileup receiver reports as ``Decode/copiedLastTx``.
    public static func encode(
        subMode: Q65SubMode,
        call1: String,
        call2: String,
        gridOrReport: String,
        copiedLastTx: Bool,
        frequencyHz: Float
    ) throws -> [Float] {
        try encodeAudio { out, capacity, written in
            mfsk_encode_q65_flagged(subMode.rawValue, call1, call2, gridOrReport,
                                    copiedLastTx ? 1 : 0, frequencyHz, out, capacity, written)
        }
    }
}

/// The 100 most recent Q65 decodes and their frequencies — WSJT-X's `q65_hist`.
/// It is how a "Decode Again" with no DX call entered finds the DX station, so
/// the full-AP list can be built without the operator typing the call.
///
/// The decoder is stateless, so the history is the application's: feed it each
/// decode with ``record(_:)``, ask with ``lookup(rxFrequencyHz:)``. **Not
/// thread-safe.**
public final class Q65History {
    let handle: OpaquePointer

    public init() throws {
        guard let opened = mfsk_q65_history_new() else {
            throw MfskError(status: MFSK_STATUS_INTERNAL, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_q65_history_free(handle) }

    /// The DX station ``lookup(rxFrequencyHz:)`` found — WSJT-X's `dxcall` and
    /// `dxgrid`.
    public struct DX: Sendable, Equatable {
        public var call: String
        public var grid: String?
    }

    /// Remember one decode at `frequencyHz` (tone 0); the 100 most recent are
    /// kept.
    public func push(frequencyHz: Float, message: String) throws {
        try check(mfsk_q65_history_push(handle, frequencyHz, message))
    }

    /// Remember every row of a decode, as `q65_decode.f90` calls `q65_hist`
    /// after each one.
    public func record(_ rows: [Decode]) throws {
        for row in rows { try push(frequencyHz: row.frequencyHz, message: row.text) }
    }

    public var count: Int { Int(mfsk_q65_history_len(handle)) }

    /// The DX station from the most recent decode within 10 Hz of
    /// `rxFrequencyHz` whose first word is 3 to 12 characters — so a `CQ ...`
    /// decode is passed over for an older one — or nil when nothing qualifies.
    public func lookup(rxFrequencyHz: Float) throws -> DX? {
        var raw = MfskQ65Dx()
        raw.size = UInt32(MemoryLayout<MfskQ65Dx>.size)
        let status = mfsk_q65_history_lookup(handle, rxFrequencyHz, &raw)
        if status == MFSK_STATUS_DECODE_FAILED { return nil }
        try check(status)
        return DX(call: stringFromCArray(raw.call),
                  grid: raw.has_grid != 0 ? stringFromCArray(raw.grid) : nil)
    }
}

/// The contest caller list — WSJT-X's `q65_hist2`: up to 50 stations that
/// called with a grid, from which the contest full-AP list is built
/// (``Q65/List/contest(myCall:hisCall:hisGrid:)``). Times are yours (Unix
/// seconds), since the library reads no clock. **Not thread-safe.**
public final class Q65Callers {
    let handle: OpaquePointer

    public init() throws {
        guard let opened = mfsk_q65_callers_new() else {
            throw MfskError(status: MFSK_STATUS_INTERNAL, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_q65_callers_free(handle) }

    /// One remembered station.
    public struct Caller: Sendable, Equatable {
        /// Up to six characters.
        public var call: String
        /// The four-character grid it sent.
        public var grid: String
        /// When it was last heard, as passed to ``record(frequencyHz:message:now:)``.
        public var lastHeard: UInt64
        /// Its audio frequency then, Hz.
        public var frequencyHz: Int32
    }

    /// Remember a decode at `frequencyHz` heard at `now`: a compound call is
    /// ignored, ` R ` is taken out, the second word is the caller and the next
    /// four characters its grid. A known caller is refreshed; a new one is
    /// added only if it sent a grid, the oldest making room once 50 are held.
    public func record(frequencyHz: Float, message: String, now: UInt64) throws {
        try check(mfsk_q65_callers_record(handle, frequencyHz, message, now))
    }

    /// Drop callers not heard for more than 24 hours. Call before each decode.
    public func expire(now: UInt64) throws {
        try check(mfsk_q65_callers_expire(handle, now))
    }

    /// Forget one caller (worked, say).
    public func remove(_ call: String) throws {
        try check(mfsk_q65_callers_remove(handle, call))
    }

    public var count: Int { Int(mfsk_q65_callers_len(handle)) }

    /// The stations, oldest first.
    public var callers: [Caller] {
        (0..<count).compactMap { index in
            var raw = MfskQ65Caller()
            raw.size = UInt32(MemoryLayout<MfskQ65Caller>.size)
            guard mfsk_q65_callers_get(handle, UInt(index), &raw) == MFSK_STATUS_OK else {
                return nil
            }
            return Caller(call: stringFromCArray(raw.call), grid: stringFromCArray(raw.grid),
                          lastHeard: raw.last_heard, frequencyHz: raw.freq_hz)
        }
    }
}
