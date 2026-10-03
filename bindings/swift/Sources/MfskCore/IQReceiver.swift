// SPDX-License-Identifier: GPL-3.0-or-later
//
// The wideband IQ receiver — `mfsk_iq_*`. Bytes of IQ in, decodes out of
// `poll`, one ``Decoder`` per channel: a channel is a dial frequency and a
// mode, and the receiver cuts its slots on the mode's UTC grid and decodes
// each one with the channel's own decoder before `push` returns.

import CMfsk

/// The wire format of the IQ bytes ``IQReceiver/push(_:)`` takes. Always
/// little-endian, I then Q.
public enum IQFormat: UInt32, Sendable {
    /// 32-bit float, 8 bytes per complex sample.
    case cf32 = 0
    /// Signed 16-bit.
    case cs16 = 1
    /// Signed 8-bit.
    case cs8 = 2
    /// Unsigned 8-bit, centred on 128 (RTL-SDR).
    case cu8 = 3
    /// Signed 24-bit.
    case cs24 = 4
}

/// How the receiver brings each channel down to audio.
public enum IQChannelizer: UInt32, Sendable {
    /// One filter chain per channel: cheapest for a few.
    case direct = 0
    /// A polyphase filter bank shared by every channel: a fixed cost of
    /// about three direct channels, then about a quarter of one per
    /// channel. Not available under 40 kS/s.
    case pfb = 1
}

/// Whether a channel is being received.
public enum IQChannelState: Int32, Sendable {
    case active = 0
    /// Its audio window no longer fits the IQ band after a retune. It keeps
    /// its dial and its decoder, and resumes when a later retune brings it
    /// back inside the band.
    case paused = 1
}

/// One decode out of ``IQReceiver/poll()``: a row of a channel, with the
/// absolute RF frequency and where its slot started.
public struct IQDecode: Sendable, Equatable {
    /// The handle ``IQReceiver/addChannel(dialHz:mode:params:extras:)``
    /// returned.
    public let channel: UInt32
    public let mode: Mode
    /// RF frequency of tone 0, Hz: the channel's dial plus ``frequencyHz``.
    public let absoluteFrequencyHz: Double
    /// The slot's index on the mode's UTC grid (UTC `period * T` with a
    /// clock set, counted from sample 0 without one).
    public let period: Int64
    /// Index (of the IQ stream, in complex samples) the slot started at.
    public let slotStartSample: UInt64
    /// UTC of the slot start, nanoseconds since the Unix epoch, or nil on a
    /// free-running grid.
    public let slotStartUTCNanoseconds: Int64?
    /// Audio frequency of tone 0 within the channel, Hz.
    public let frequencyHz: Float
    public let dtSeconds: Float
    public let snrDB: Float
    public let text: String

    init(_ raw: MfskIqDecode) {
        self.channel = raw.channel
        self.mode = Mode(rawValue: UInt32(raw.mode.rawValue)) ?? .ft8
        self.absoluteFrequencyHz = raw.abs_freq_hz
        self.period = raw.period
        self.slotStartSample = raw.slot_start_sample
        self.slotStartUTCNanoseconds = raw.has_utc != 0 ? raw.slot_start_utc_ns : nil
        self.frequencyHz = raw.freq_hz
        self.dtSeconds = raw.dt_sec
        self.snrDB = raw.snr_db
        self.text = stringFromCArray(raw.text)
    }
}

/// A receiver for an IQ stream. Not thread-safe: push and poll from one
/// thread, or serialise.
public final class IQReceiver {
    let handle: OpaquePointer

    /// Decoders handed out by ``decoder(forChannel:)``, weakly: a strong
    /// reference here would be a cycle with the decoder's hold on this
    /// receiver.
    private var lent: [UInt32: WeakDecoder] = [:]

    private struct WeakDecoder {
        weak var decoder: Decoder?
    }

    /// Open a receiver for `sampleRate` complex samples per second (any
    /// integer of 12 000 or more whose ratio to 12 kHz is a small
    /// fraction), `centerHz` the RF frequency of DC. `iqSwap` is for I and Q
    /// exchanged (sound-card IQ often is).
    public init(
        sampleRate: UInt32,
        centerHz: Double,
        format: IQFormat = .cf32,
        iqSwap: Bool = false,
        channelizer: IQChannelizer = .direct
    ) throws {
        var status = MFSK_STATUS_OK
        guard let opened = mfsk_iq_open_with(sampleRate, centerHz, format.rawValue,
                                             iqSwap ? 1 : 0, channelizer.rawValue, &status) else {
            throw MfskError(status: status, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_iq_close(handle) }

    // MARK: Channels

    /// Add a channel whose dial (audio 0 Hz) is `dialHz`, carrying `mode`,
    /// decoded with its own decoder opened from `params` and `extras` (nil
    /// for the mode's defaults). Returns the handle rows carry.
    ///
    /// Throws ``MfskError/Code/invalidArgument`` when the channel cannot be
    /// placed (DC inside its 0-6 kHz audio window, or the window outside the
    /// IQ band) or the mode is not one the receiver carries, and
    /// ``MfskError/Code/unsupported`` for an option the mode lacks.
    @discardableResult
    public func addChannel(
        dialHz: Double,
        mode: Mode,
        params: DecodeParams? = nil,
        extras: Extras? = nil
    ) throws -> UInt32 {
        var channel: UInt32 = 0
        let status: MfskStatus = try withOptionalParams(params) { p in
            try withOptionalExtras(extras) { e in
                mfsk_iq_add_channel(handle, dialHz, mode.rawValue, p, e, &channel)
            }
        }
        try check(status)
        modes[channel] = mode
        return channel
    }

    private var modes: [UInt32: Mode] = [:]

    /// The channel's decoder, for the calls that configure one
    /// (``Decoder/setParams(_:)``, ``Decoder/setExtras(_:)``,
    /// ``Decoder/addCallsign(_:)``, ``Decoder/clear()``,
    /// ``Decoder/setQ65Callers(_:)``). **Borrowed**: the receiver owns the
    /// C decoder and decodes with it, so do not call its `decode`, and do
    /// not use it after ``removeChannel(_:)``. Nil if there is no such
    /// channel.
    public func decoder(forChannel channel: UInt32) -> Decoder? {
        if let existing = lent[channel]?.decoder { return existing }
        guard let mode = modes[channel],
              let raw = mfsk_iq_channel_decoder(handle, channel) else { return nil }
        let decoder = Decoder(borrowing: raw, mode: mode, owner: self)
        lent[channel] = WeakDecoder(decoder: decoder)
        return decoder
    }

    /// Whether the channel is being received, or nil if there is no such
    /// channel.
    public func state(ofChannel channel: UInt32) -> IQChannelState? {
        IQChannelState(rawValue: mfsk_iq_channel_state(handle, channel))
    }

    /// Remove a channel. Throws ``MfskError/Code/invalidArgument`` if there
    /// is no such channel.
    public func removeChannel(_ channel: UInt32) throws {
        // The C decoder is about to be freed: let go of the closures the
        // C side holds first, so a wrapper still alive cannot be torn down
        // against freed memory.
        lent[channel]?.decoder?.releaseCallbacks()
        try check(mfsk_iq_remove_channel(handle, channel))
        lent[channel] = nil
        modes[channel] = nil
    }

    // MARK: Time

    /// The stream's complex sample `atSample` (as ``samplesIn`` counts) was
    /// at UTC `utcNanoseconds`. nil for `atSample` is ``samplesIn``.
    ///
    /// Call it as often as you have a reading: the receiver follows readings
    /// at up to 400 ppm, so a drifting crystal or host clock moves slot
    /// boundaries by milliseconds and loses no slot; only a jump of more
    /// than a second drops the slots that straddle it. Without any reading
    /// the grid free-runs from sample 0, right for replaying a recording.
    @discardableResult
    public func setTime(utcNanoseconds: Int64, atSample: UInt64? = nil) throws -> ClockChange {
        var change: Int32 = 0
        try check(mfsk_iq_set_time(handle, utcNanoseconds, atSample ?? samplesIn, &change))
        return ClockChange(rawValue: change) ?? .stepped
    }

    /// The tuner moved to `centerHz`: every channel that still fits is
    /// re-placed against it, one that no longer fits is paused, and the open
    /// slots are dropped; the sample clock continues. Returns how many
    /// channels changed state; ask ``state(ofChannel:)`` which.
    @discardableResult
    public func retune(centerHz: Double) throws -> (paused: Int, resumed: Int) {
        var paused: UInt32 = 0
        var resumed: UInt32 = 0
        try check(mfsk_iq_retune(handle, centerHz, &paused, &resumed))
        return (Int(paused), Int(resumed))
    }

    /// `lost` samples never arrived: the clock advances past them and the
    /// open slots are dropped.
    public func gap(lostSamples lost: UInt64) throws {
        try check(mfsk_iq_gap(handle, lost))
    }

    // MARK: Samples in, decodes out

    /// Push IQ bytes in the format the receiver was opened with,
    /// little-endian, I then Q; a sample split across calls is carried over.
    /// Every slot this completes is decoded before the call returns; what it
    /// found waits for ``poll()``. A call can therefore take as long as a
    /// decode.
    public func push(_ bytes: [UInt8]) throws {
        guard !bytes.isEmpty else { return }
        try bytes.withUnsafeBytes { raw in
            try check(mfsk_iq_push(handle, raw.baseAddress, UInt(raw.count)))
        }
    }

    /// Complex samples consumed so far, gaps included: the stream's clock.
    public var samplesIn: UInt64 { mfsk_iq_samples_in(handle) }

    /// How many decodes wait for ``poll()``.
    public var pending: Int { Int(mfsk_iq_pending(handle)) }

    /// The oldest waiting decode, or nil if none is waiting. Call until nil
    /// after every push.
    public func poll() throws -> IQDecode? {
        var raw = MfskIqDecode()
        raw.size = UInt32(MemoryLayout<MfskIqDecode>.size)
        let r = mfsk_iq_poll(handle, &raw)
        if r < 0 {
            throw MfskError(status: MfskStatus(rawValue: r), detail: globalLastError())
        }
        return r == 1 ? IQDecode(raw) : nil
    }

    /// Everything waiting, oldest first.
    public func drain() throws -> [IQDecode] {
        var rows: [IQDecode] = []
        while let row = try poll() { rows.append(row) }
        return rows
    }
}
