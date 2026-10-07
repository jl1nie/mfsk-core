// SPDX-License-Identifier: GPL-3.0-only
//
// The decode row and the search parameters, both mirroring
// size-versioned C structs. Rows go into memory this binding owns:
// nothing here frees a pointer the library allocated, which is the
// category the v2 C surface removed from the decode path — and the one
// that makes a wrapper leak when an error unwinds between the call and
// the free.

import CMfsk

/// One decoded transmission.
public struct Decode: Sendable, Equatable {
    /// The **concrete sub-mode**, not the family: all five FST4 periods
    /// stay distinct here because the addressing model does.
    public let mode: Mode
    public let text: String
    public let frequencyHz: Float
    /// Time offset from the slot's `dt = 0` reference, seconds.
    public let dtSeconds: Float
    /// Estimated SNR in a 2500 Hz reference bandwidth, dB.
    public let snrDB: Float
    /// Sync score on the scale of the mode's own search, so **not
    /// comparable between modes**. nil where the mode reports none: WSPR,
    /// JT9, JT65, Q65, and FT8's a7 / a8 list decodes, which run no sync
    /// search (`MFSK_DECODE_FLAG_HAS_SYNC_SCORE` clear).
    public let syncScore: Float?
    /// Coefficient of variation of the per-block sync powers — near 0
    /// on a stable channel, elevated under QSB. The only fading
    /// indicator the row carries. nil wherever ``syncScore`` is.
    public let syncCV: Float?
    /// Hard-decision errors the FEC had to correct; 0 is a clean decode.
    /// nil for WSPR, JT9, JT65 and Q65, whose decoders report no count.
    public let hardErrors: UInt32?
    /// Width of the information block ``Decoder/informationBits(at:)``
    /// returns — 91 for FT8 and FT4, 101 for FST4, 50 for WSPR, 72 for JT9
    /// and JT65, 77 for Q65.
    public let informationBitCount: UInt16
    /// Which decode pass produced this row. **Protocol-private**: the
    /// numbers mean different things per mode, and are for diagnostics,
    /// not for logic.
    public let pass: UInt8
    /// The text needed the callsign hash table to resolve a `<...>`
    /// reference — `MFSK_DECODE_FLAG_HASH_RESOLVED`, which is a named
    /// constant in the header only because the Kotlin shim found it
    /// missing.
    public let usedHashTable: Bool
    /// The sender set WSJT-X 3.2's **Q65 Pileup** "copied last Tx" flag, the
    /// spare 78th payload bit — `MFSK_DECODE_FLAG_COPIED_LAST_TX`. WSJT-X
    /// marks such a decode with `#`. Q65 rows only; false elsewhere.
    public let copiedLastTx: Bool
    /// The message's identity key: ``keyBits`` bits (77 for FT8, FT4, FST4
    /// and Q65; 72 for JT9 and JT65; 50 for WSPR), packed most significant
    /// bit first. The same message in two decoders has one key even when its
    /// text differs; one message at two frequencies has one key too, so add
    /// ``frequencyHz`` to tell signals apart.
    public let key: [UInt8]
    public let keyBits: UInt8
    /// Which delivery of the period this row is, or came from: a row handed
    /// to ``Decoder/onDecode(_:)`` carries its position (0, 1, 2...), a
    /// returned row the position of the delivery it was, so the two pair
    /// exactly. nil for a returned row the handler never saw, and with no
    /// handler.
    public let delivery: UInt32?

    init(_ raw: MfskDecode) {
        self.mode = Mode(rawValue: UInt32(raw.mode.rawValue)) ?? .ft8
        self.text = stringFromCArray(raw.text)
        self.frequencyHz = raw.freq_hz
        self.dtSeconds = raw.dt_sec
        self.snrDB = raw.snr_db
        let detail = RowDetail(flags: raw.flags, syncScore: raw.sync_score, syncCV: raw.sync_cv,
                               hardErrors: raw.hard_errors, keyBits: raw.key_bits, key: raw.key,
                               delivery: raw.delivery)
        self.syncScore = detail.syncScore
        self.syncCV = detail.syncCV
        self.hardErrors = detail.hardErrors
        self.informationBitCount = raw.info_bits
        self.pass = raw.pass
        self.usedHashTable = detail.usedHashTable
        self.copiedLastTx = detail.copiedLastTx
        self.key = detail.key
        self.keyBits = raw.key_bits
        self.delivery = detail.delivery
    }
}

/// The detail fields `MfskDecode` and `MfskIqDecode` share, unpacked once:
/// a number whose `MFSK_DECODE_FLAG_HAS_*` bit is clear is nil, as it is
/// `None` in Rust (#594).
struct RowDetail {
    let syncScore: Float?
    let syncCV: Float?
    let hardErrors: UInt32?
    let usedHashTable: Bool
    let copiedLastTx: Bool
    let key: [UInt8]
    let delivery: UInt32?

    init<K>(flags: UInt8, syncScore: Float, syncCV: Float, hardErrors: UInt32,
            keyBits: UInt8, key: K, delivery: Int32) {
        func has(_ bit: Int32) -> Bool { flags & UInt8(bit) != 0 }
        self.syncScore = has(MFSK_DECODE_FLAG_HAS_SYNC_SCORE) ? syncScore : nil
        self.syncCV = has(MFSK_DECODE_FLAG_HAS_SYNC_CV) ? syncCV : nil
        self.hardErrors = has(MFSK_DECODE_FLAG_HAS_HARD_ERRORS) ? hardErrors : nil
        self.usedHashTable = has(MFSK_DECODE_FLAG_HASH_RESOLVED)
        self.copiedLastTx = has(MFSK_DECODE_FLAG_COPIED_LAST_TX)
        let bytes = (Int(keyBits) + 7) / 8
        self.key = withUnsafeBytes(of: key) { Array($0.prefix(bytes)) }
        self.delivery = delivery >= 0 ? UInt32(delivery) : nil
    }
}

/// The per-period parameter block, after WSJT-X's `params` common block
/// (`lib/jt9com.f90`) — what the GUI fills before each period and the
/// decoder reads. `MfskParams` in the C ABI.
///
/// Start from a mode's own defaults with ``init(mode:)`` and change what
/// you want: zeroing the fields by hand is *not* equivalent (a zero band
/// decodes nothing, and 0 Hz is a frequency, not "unset" — which is why
/// ``rxFrequencyHz``, ``toleranceHz`` and ``txFrequencyHz`` are `Optional`
/// here and NaN there).
///
/// A mode reads what its upstream decoder reads and ignores the rest, as
/// `jt9` does; ``depth`` decides every search setting the way `ndepth`
/// does. The library's own options — the ones WSJT-X has no field for —
/// are in ``Extras``.
public struct DecodeParams: Sendable, Equatable {
    /// `ndepth & 7`. A value the ABI does not know is refused, never
    /// clamped; the C default (0) is the GUI's, ``deep``.
    public enum Depth: UInt32, Sendable {
        case fast = 1
        case normal = 2
        case deep = 3
    }

    /// A-priori decoding from the QSO context. FT8 and JT65 default to
    /// ``off``, as the GUI's "Enable AP" boxes do.
    public enum AP: UInt32, Sendable {
        case off = 0
        /// `lapcqonly`: hypotheses for a CQ only.
        case cqOnly = 1
        /// Every hypothesis the QSO context allows.
        case full = 2
    }

    /// `ncontest`. The numbers are WSJT-X's; 5 is not a contest here.
    public enum Contest: UInt32, Sendable {
        case none = 0
        case gridExchange = 1
        case euVHF = 2
        case fieldDay = 3
        case rttyRoundUp = 4
        case fox = 6
        case hound = 7
    }

    /// `nQSOProgress`.
    public enum QSOProgress: UInt32, Sendable {
        case calling = 0
        case replying = 1
        case report = 2
        case rogerReport = 3
        case rogers = 4
        case signoff = 5
    }

    /// The operator: `mycall` and `mygrid`. An empty string is "not set".
    /// The ABI keeps 15 characters of call and 7 of grid.
    public struct Station: Sendable, Equatable {
        public var call: String
        public var grid: String

        public init(call: String = "", grid: String = "") {
            self.call = call
            self.grid = grid
        }
    }

    /// The QSO in progress: `hiscall`, `hisgrid` and `nQSOProgress`.
    public struct QSO: Sendable, Equatable {
        public var hisCall: String
        public var hisGrid: String
        public var progress: QSOProgress

        public init(hisCall: String = "", hisGrid: String = "", progress: QSOProgress = .calling) {
            self.hisCall = hisCall
            self.hisGrid = hisGrid
            self.progress = progress
        }
    }

    /// The audio band searched, Hz (`nfa`...`nfb`).
    public var bandHz: ClosedRange<Float>
    /// The Rx frequency (`nfqso`), or nil.
    public var rxFrequencyHz: Float?
    /// Tolerance around the Rx frequency (`ntol`), or nil.
    public var toleranceHz: Float?
    /// The Tx frequency (`nftx`), or nil. FT8 tries an a-priori hypothesis
    /// that locks both callsigns within 50 Hz of it. A mode without
    /// ``Capabilities/transmitFrequency`` ignores it.
    public var txFrequencyHz: Float?
    public var depth: Depth
    /// Average successive periods (`ndepth & 16`): JT65 and Q65.
    public var averaging: Bool
    /// Deep search (`ndepth & 32`): JT65.
    public var deepSearch: Bool
    /// EME delay (`emedelay`): Q65's late edge reaches +5.5 s (+4.0 s on
    /// Q65-15).
    public var emeDelay: Bool
    public var station: Station
    public var qso: QSO
    public var ap: AP
    public var contest: Contest

    /// This mode's own defaults — `mfsk_params_init`, which is the only
    /// right starting point. Throws ``MfskError/Code/unknownProtocol`` for
    /// a mode with no slot decoder in this build (MSK144, JTTY).
    public init(mode: Mode) throws {
        var raw = MfskParams()
        raw.size = UInt32(MemoryLayout<MfskParams>.size)
        try check(mfsk_params_init(mode.rawValue, &raw))
        self.init(raw)
    }

    init(_ raw: MfskParams) {
        self.bandHz = raw.band_lo_hz...max(raw.band_lo_hz, raw.band_hi_hz)
        // NaN is the ABI's spelling of "unset": 0 Hz is a frequency.
        self.rxFrequencyHz = raw.rx_freq_hz.isNaN ? nil : raw.rx_freq_hz
        self.toleranceHz = raw.tol_hz.isNaN ? nil : raw.tol_hz
        self.txFrequencyHz = raw.tx_freq_hz.isNaN ? nil : raw.tx_freq_hz
        // 0 is documented as "the default, Deep".
        self.depth = Depth(rawValue: raw.depth) ?? .deep
        self.averaging = raw.flags & UInt32(MFSK_PARAM_AVERAGING) != 0
        self.deepSearch = raw.flags & UInt32(MFSK_PARAM_DEEP_SEARCH) != 0
        self.emeDelay = raw.flags & UInt32(MFSK_PARAM_EME_DELAY) != 0
        self.station = Station(call: stringFromCArray(raw.mycall), grid: stringFromCArray(raw.mygrid))
        self.qso = QSO(hisCall: stringFromCArray(raw.hiscall),
                       hisGrid: stringFromCArray(raw.hisgrid),
                       progress: QSOProgress(rawValue: raw.qso_progress) ?? .calling)
        self.ap = AP(rawValue: raw.ap_mode) ?? .off
        self.contest = Contest(rawValue: raw.contest) ?? .none
    }

    /// Marshal into the C struct for the duration of `body`. One copy, not
    /// a sequence of fallible setter calls. Throws
    /// ``MfskError/Code/invalidArgument`` for a callsign or grid longer than
    /// the ABI's inline field, rather than truncating it.
    func withC<R>(_ body: (UnsafePointer<MfskParams>) throws -> R) throws -> R {
        var raw = MfskParams()
        raw.size = UInt32(MemoryLayout<MfskParams>.size)
        raw.depth = depth.rawValue
        var flags: UInt32 = 0
        if averaging { flags |= UInt32(MFSK_PARAM_AVERAGING) }
        if deepSearch { flags |= UInt32(MFSK_PARAM_DEEP_SEARCH) }
        if emeDelay { flags |= UInt32(MFSK_PARAM_EME_DELAY) }
        raw.flags = flags
        raw.ap_mode = ap.rawValue
        raw.contest = contest.rawValue
        raw.qso_progress = qso.progress.rawValue
        raw.band_lo_hz = bandHz.lowerBound
        raw.band_hi_hz = bandHz.upperBound
        raw.rx_freq_hz = rxFrequencyHz ?? Float.nan
        raw.tol_hz = toleranceHz ?? Float.nan
        raw.tx_freq_hz = txFrequencyHz ?? Float.nan
        try setCArrayChecked(&raw.mycall, to: station.call, field: "station.call")
        try setCArrayChecked(&raw.mygrid, to: station.grid, field: "station.grid")
        try setCArrayChecked(&raw.hiscall, to: qso.hisCall, field: "qso.hisCall")
        try setCArrayChecked(&raw.hisgrid, to: qso.hisGrid, field: "qso.hisGrid")
        return try withUnsafePointer(to: &raw) { try body($0) }
    }
}

/// The library's options beyond the parameter block, per mode —
/// `MfskExtras` in the C ABI.
///
/// Every field starts **unset**, which is not the same as zero in C
/// (0 `strictness` is Strict, 0 `osd` is off), so the optionals here are
/// the point: `nil` leaves the depth's own value alone. An option the mode
/// does not have is refused when the decoder is opened or the block
/// replaced, with ``MfskError/Code/unsupported`` and a detail naming it —
/// never dropped. A value out of range is ``MfskError/Code/invalidArgument``.
///
/// ``Decoder/setExtras(_:)`` replaces the whole block, so what you leave
/// unset goes back to the depth's value.
public struct Extras: Sendable, Equatable {
    /// Accept/reject profile.
    public enum Strictness: Int32, Sendable {
        case strict = 0
        /// WSJT-X's own ceiling for FT8; independently-tuned values for
        /// FT4/FST4.
        case normal = 1
        /// Deliberately exceeds WSJT-X's own FT8 ceiling; exploratory.
        case deep = 2
    }

    /// How the period's candidates are decoded, in place of the depth's
    /// own.
    public enum Strategy: Sendable, Equatable {
        case modeDefault
        /// One pass, no subtraction. FT8 and FT4 subtract by default, as
        /// WSJT-X does.
        case singlePass
        /// This many rounds of subtraction. FT8 and FT4 only.
        case sicRounds(UInt32)
        /// The checkpointed passes. FT8 only.
        case sicEarly
    }

    /// Equalisation. A property of the **input audio** — it flattens a
    /// passband an analogue filter has tilted — not of the search, which
    /// is why it costs decodes on flat synthetic audio.
    public enum Equalisation: UInt32, Sendable {
        case off = 0
        /// Per-signal equalisation using local Costas pilot tones.
        case local = 1
    }

    /// WSJT-X's **NB** setting for FST4 — an impulse-noise blanker run
    /// before the slot transform.
    public enum NoiseBlanker: Sendable, Equatable {
        /// Blank the loudest `n` percent of samples, `0...25`; more is
        /// refused rather than clamped.
        case percent(UInt8)
        /// Decode once per blanking level `0, step, 2*step, .. 20` percent.
        /// `step` is 5, 2 or 1. Every level above 0 searches only within
        /// `toleranceHz` of ``DecodeParams/rxFrequencyHz``.
        case sweep(step: UInt8, toleranceHz: Float)
    }

    /// An a-priori hypothesis, as the message's own fields in order:
    /// `call1` is the first callsign field — `"CQ"` for a CQ, not the
    /// transmitting station — and locks message bits 0-28, `call2` the
    /// second and locks 29-57, `grid` and `report` both lock 58-73, so
    /// they are alternatives (`report` is `"RRR"`, `"RR73"`, `"73"` or a
    /// signal report). A hint locks bits rather than steering a search, so
    /// **the wrong order removes decodes** rather than costing a little
    /// sensitivity. It sits beside the QSO-context AP of
    /// ``DecodeParams/ap`` and wins when given.
    ///
    /// Each field keeps 15 characters.
    public struct APHint: Sendable, Equatable {
        public var call1: String
        public var call2: String
        public var grid: String
        public var report: String

        public init(call1: String = "", call2: String = "", grid: String = "", report: String = "") {
            self.call1 = call1
            self.call2 = call2
            self.grid = grid
            self.report = report
        }
    }

    /// Q65's fast-fading metric.
    public struct Fading: Sendable, Equatable {
        /// Spread bandwidth times symbol period. Typical values: 0.05
        /// near-AWGN, 1.0 moderate, 5.0+ severe.
        public var b90Ts: Float
        public var model: Q65FadingModel

        public init(b90Ts: Float, model: Q65FadingModel = .gaussian) {
            self.b90Ts = b90Ts
            self.model = model
        }
    }

    /// Sync threshold over the depth's. **Not comparable across modes**:
    /// FT4 and FST4 normalise the spectrum so noise sits at ~1.0, FT8 uses
    /// an absolute Costas score, and the rest a fraction in 0...1.
    public var syncMin: Float?
    /// Candidate budget over the depth's.
    public var maxCandidates: UInt32?
    /// OSD over the depth's: nil the depth's, false off, true on.
    public var osd: Bool?
    public var strictness: Strictness?
    public var strategy: Strategy = .modeDefault
    public var equalisation: Equalisation = .off
    /// Take the codec's verdict alone instead of the protocol's own
    /// message filter.
    public var codecMessageFilter: Bool = false
    /// FT8's a7 list decoder (`ft8_a7.f90`), fed by the decoder's own
    /// decodes two periods back. Needs a period index on every decode.
    public var a7: Bool = false
    /// Half-width of FT8's roofing-filter search around the Rx frequency,
    /// Hz; nil is the wide-band search. This is what the transceiver's
    /// analogue roofing filter makes useful.
    public var sniperHalfWidthHz: Float?
    public var apHint: APHint?
    /// FST4's noise blanker.
    public var noiseBlanker: NoiseBlanker?
    /// WSPR, JT9, JT65 and Q65: how far before the nominal start a frame
    /// may begin, seconds.
    public var earlyToleranceSeconds: Float?
    /// As ``earlyToleranceSeconds``, after the nominal start.
    public var lateToleranceSeconds: Float?
    /// As ``earlyToleranceSeconds``: coarse-sync acceptance, 0...1.
    public var scoreThreshold: Float?
    /// WSPR: Fano cycles per bit (`wsprd -C`).
    public var maxCyclesPerBit: UInt32?
    /// JT65: Chase trials (`nvec`).
    public var chaseTrials: UInt32?
    /// **Q65 Pileup**: a reply carrying "copied last Tx" still matches an
    /// AP hint naming both callsigns. Needs ``apHint``.
    public var pileup: Bool = false
    /// **Q65 Max Drift** in spectrum bins, `0...50`; 0 is off. Costs
    /// `2*bins+1` times the plain search.
    public var maxDrift: UInt32 = 0
    public var fading: Fading?

    /// Everything unset: the depth's own values.
    public init() {}

    func withC<R>(_ body: (UnsafePointer<MfskExtras>) throws -> R) throws -> R {
        var raw = MfskExtras()
        raw.size = UInt32(MemoryLayout<MfskExtras>.size)
        // "Unset" is not zero (see the type's note), so start from the
        // library's own idea of it and override what was given.
        try check(mfsk_extras_init(&raw))
        if let syncMin { raw.sync_min = syncMin }
        if let maxCandidates { raw.max_cand = maxCandidates }
        if let osd { raw.osd = osd ? 1 : 0 }
        if let strictness { raw.strictness = strictness.rawValue }
        switch strategy {
        case .modeDefault:
            break
        case .singlePass:
            raw.strategy = UInt32(MFSK_STRATEGY_SINGLE_PASS)
        case .sicRounds(let rounds):
            raw.strategy = UInt32(MFSK_STRATEGY_SIC_ROUNDS)
            raw.sic_rounds = rounds
        case .sicEarly:
            raw.strategy = UInt32(MFSK_STRATEGY_SIC_EARLY)
        }
        raw.eq_mode = equalisation.rawValue
        raw.message_filter = codecMessageFilter ? 1 : 0
        raw.a7 = a7 ? 1 : 0
        if let sniperHalfWidthHz { raw.sniper_hz = sniperHalfWidthHz }
        if let hint = apHint {
            raw.has_ap_hint = 1
            try setCArrayChecked(&raw.ap_call1, to: hint.call1, field: "apHint.call1")
            try setCArrayChecked(&raw.ap_call2, to: hint.call2, field: "apHint.call2")
            try setCArrayChecked(&raw.ap_grid, to: hint.grid, field: "apHint.grid")
            try setCArrayChecked(&raw.ap_report, to: hint.report, field: "apHint.report")
        }
        switch noiseBlanker {
        case .none:
            break
        case .percent(let n):
            raw.nb_percent = UInt32(n)
        case .sweep(let step, let toleranceHz):
            raw.nb_sweep_step = UInt32(step)
            raw.nb_ftol_hz = toleranceHz
        }
        if let earlyToleranceSeconds { raw.t_early_s = earlyToleranceSeconds }
        if let lateToleranceSeconds { raw.t_late_s = lateToleranceSeconds }
        if let scoreThreshold { raw.score_threshold = scoreThreshold }
        if let maxCyclesPerBit { raw.max_cycles_per_bit = maxCyclesPerBit }
        if let chaseTrials { raw.chase_trials = chaseTrials }
        raw.pileup = pileup ? 1 : 0
        raw.max_drift = maxDrift
        if let fading {
            raw.fading_b90_ts = fading.b90Ts
            raw.fading_model = fading.model.rawValue
        }
        return try withUnsafePointer(to: &raw) { try body($0) }
    }
}

/// Run a call that fills caller-owned rows, growing the buffer if the
/// library says it was short.
///
/// The ABI's contract is that `*out_len` always receives the number of
/// decodes found, so a short buffer comes back as `INVALID_ARG` with
/// the required count rather than a truncated answer you cannot detect.
/// One retry is enough: the second buffer is sized from that count. The
/// default capacity is generous (256 rows is about 30 KB) precisely
/// because the retry **decodes the period again**, with the callback
/// firing a second time; a period with more rows than that is not a thing
/// a receiver meets.
func collectRows(
    capacity: Int = 256,
    errorDetail: () -> String? = { globalLastError() },
    _ call: (UnsafeMutablePointer<MfskDecode>, UInt, UnsafeMutablePointer<UInt>) -> MfskStatus
) throws -> [Decode] {
    var template = MfskDecode()
    template.size = UInt32(MemoryLayout<MfskDecode>.size)
    var rows = [MfskDecode](repeating: template, count: max(capacity, 1))
    var found: UInt = 0
    let status = rows.withUnsafeMutableBufferPointer { buffer in
        call(buffer.baseAddress!, UInt(buffer.count), &found)
    }
    if status != MFSK_STATUS_OK {
        if status == MFSK_STATUS_INVALID_ARG, Int(found) > rows.count {
            return try collectRows(capacity: Int(found), errorDetail: errorDetail, call)
        }
        throw MfskError(status: status, detail: errorDetail())
    }
    return rows.prefix(Int(found)).map(Decode.init)
}
