// SPDX-License-Identifier: GPL-3.0-or-later
//
// JTTY — WSJT-X 3.2's non-slotted keyboard mode — `mfsk_jtty_*`.
//
// Not a slot decode: JTTY frames start whenever the sender likes and a
// message is several of them, so the receiver keeps state (the search
// window, the messages under assembly, the audio a re-sweep of earlier
// windows still needs) and reports *updates*. That is why this is a
// class of its own rather than a ``DecodeSession`` mode: the mode's
// capabilities say ``Capabilities/streamReceiver``, not
// ``Capabilities/decodeHandle``.

import CMfsk

/// What a ``JttyReceiver`` looks for. The defaults are `rjtty`'s.
public struct JttyParams: Sendable, Equatable {
    /// The operator's receive frequency, Hz (channel 0's centre).
    public var frequencyHz: Float = 1500
    /// Half-width of channel 0, Hz.
    public var toleranceHz: Float = 50
    /// Sync-gate S/N floor on channel 0, dB.
    public var minimumSyncDB: Float = 4.6
    /// The band channels 1 and 2 watch for stations off ``frequencyHz``, Hz.
    public var bandHz: ClosedRange<Float> = 200...2800
    /// Take each decoded frame off the signal and search again. Off is a
    /// single-signal receiver that loses a weak station under a strong one.
    public var subtracts: Bool = true

    public init() {}

    func raw() -> MfskJttyParams {
        var p = MfskJttyParams()
        mfsk_jtty_params_init(&p)
        p.f0_hz = frequencyHz
        p.ftol_hz = toleranceHz
        p.smin_db = minimumSyncDB
        p.nfa_hz = bandHz.lowerBound
        p.nfb_hz = bandHz.upperBound
        p.subtract = subtracts ? 1 : 0
        return p
    }
}

/// One message as far as it is known.
///
/// A message is reported each time it grows and once more when it
/// completes; ``id`` is stable for its life, so a UI replaces its row by
/// ``id``. Updates are coalesced per message between polls.
public struct JttyUpdate: Sendable, Equatable {
    public let id: UInt64
    /// The text so far. Frames never heard show as ` ... `; TEXT5 spaces
    /// as `~`, as upstream shows them.
    public let text: String
    /// The end-of-message frame has arrived.
    public let isComplete: Bool
    /// Frequency of the latest frame, Hz.
    public let frequencyHz: Float
    /// Start of the first frame, seconds from the first sample pushed
    /// since the receiver was created or reset.
    public let startSeconds: Float

    init(_ raw: MfskJttyUpdate) {
        self.id = raw.id
        self.text = stringFromCArray(raw.text)
        self.isComplete = raw.complete != 0
        self.frequencyHz = raw.f1_hz
        self.startSeconds = raw.start_s
    }
}

/// A JTTY receiver: audio in as it arrives, in chunks of any size;
/// message updates out.
///
/// **One thread at a time.** ``push(_:)-(Int16)`` decodes every window
/// the audio completes before it returns — a few tens of milliseconds
/// per 0.47 s of audio — so call it off the main actor.
public final class JttyReceiver {
    let handle: OpaquePointer

    /// Open a receiver taking mono PCM at `sampleRate` (anything but
    /// 12 000 Hz is resampled, linearly).
    public init(sampleRate: UInt32 = 12_000, params: JttyParams = JttyParams()) throws {
        var status = MFSK_STATUS_OK
        var raw = params.raw()
        guard let opened = mfsk_jtty_open(sampleRate, &raw, &status) else {
            throw MfskError(status: status, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_jtty_close(handle) }

    /// Feed 16-bit PCM and return the updates it produced (drained from
    /// the receiver's queue, oldest first, one per message).
    @discardableResult
    public func push(_ samples: [Int16]) throws -> [JttyUpdate] {
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_jtty_push_i16(handle, audio.baseAddress, UInt(audio.count)))
        }
        return try poll()
    }

    /// Feed 32-bit float PCM, nominally `-1.0...1.0`.
    @discardableResult
    public func push(_ samples: [Float]) throws -> [JttyUpdate] {
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_jtty_push_f32(handle, audio.baseAddress, UInt(audio.count)))
        }
        return try poll()
    }

    /// Everything waiting that a previous call has not returned. The
    /// `push` calls already drain after themselves, so this is for a
    /// caller that wants its own schedule; it is empty otherwise.
    public func poll() throws -> [JttyUpdate] {
        var updates: [JttyUpdate] = []
        while true {
            var raw = MfskJttyUpdate()
            raw.size = UInt32(MemoryLayout<MfskJttyUpdate>.size)
            let r = mfsk_jtty_poll(handle, &raw)
            if r == 1 {
                updates.append(JttyUpdate(raw))
            } else if r == 0 {
                return updates
            } else {
                throw MfskError(status: MfskStatus(rawValue: r), detail: globalLastError())
            }
        }
    }

    /// The audio has ended: report every message still waiting for a
    /// continuation one last time, as incomplete. A live receiver never
    /// needs this.
    public func finish() throws -> [JttyUpdate] {
        try check(mfsk_jtty_finish(handle))
        return try poll()
    }

    /// Change the settings; they apply from the next window.
    public func setParams(_ params: JttyParams) throws {
        var raw = params.raw()
        try check(mfsk_jtty_set_params(handle, &raw))
    }

    /// Forget everything and start again at sample 0.
    public func reset() throws {
        try check(mfsk_jtty_reset(handle))
    }
}
