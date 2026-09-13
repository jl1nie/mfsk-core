// SPDX-License-Identifier: GPL-3.0-or-later
//
// Errors. Every fallible C entry point returns `MfskStatus` and records
// a human-readable reason the caller fetches separately — a small
// stable numeric set plus a string that carries the detail
// (`mfsk.h`, `MfskStatus`). Both halves are kept: the code is what a
// `catch` switches on, the detail is what a log line wants.

import CMfsk

/// A failed `mfsk_*` call: the ABI's status code plus the reason string
/// that call recorded.
public struct MfskError: Error, CustomStringConvertible, Sendable {
    /// The ABI's status discriminants. `MFSK_STATUS_OK` is deliberately
    /// absent — success is not an error, and making it unrepresentable
    /// here means a caught `MfskError` never has to be re-checked for it.
    public enum Code: Int32, Sendable {
        /// A required pointer argument was null.
        case nullPointer = -1
        /// A non-null argument was invalid — malformed UTF-8, an
        /// out-of-range enum, an audio buffer shorter than the slot, a
        /// caller buffer too small. `detail` says which.
        case invalidArgument = -2
        /// The mode is not in this build (feature-gated out), or is not
        /// a mode at all.
        case unknownProtocol = -3
        /// Ran cleanly but produced nothing usable — e.g. a callsign
        /// that will not pack into the protocol's payload.
        case decodeFailed = -4
        /// An invariant of the Rust implementation was violated. Always
        /// a bug upstream; please report it.
        case internalError = -5
        /// The mode is here but does not offer what was asked for.
        /// Distinct from ``unknownProtocol``, which means it is absent.
        case unsupported = -6
        /// A status this binding predates. The ABI promises existing
        /// discriminants never change, so a caller that switches on the
        /// ones it knows falls through to here rather than trapping.
        case unrecognised = -128
    }

    /// The status the call returned.
    public let code: Code
    /// What the library recorded about this specific failure.
    public let detail: String

    public var description: String { "\(code) (\(code.rawValue)): \(detail)" }

    init(status: MfskStatus, detail: String?) {
        self.code = Code(rawValue: status.rawValue) ?? .unrecognised
        self.detail = detail ?? "(no detail recorded)"
    }
}

/// The thread-local last-error string, or nil if this thread has none.
///
/// Thread-local is the trap: a Swift `async` caller that hops executors
/// between the failing call and this one reads nil. Prefer
/// ``DecodeSession/lastError`` wherever a session is in hand — that one
/// lives on the handle, which is why `mfsk_session_last_error` exists.
@inline(__always)
func globalLastError() -> String? {
    guard let p = mfsk_last_error() else { return nil }
    return String(cString: p)
}

/// Throw unless the call succeeded, attaching whatever reason it recorded.
@inline(__always)
func check(_ status: MfskStatus, detail: @autoclosure () -> String? = globalLastError()) throws {
    guard status != MFSK_STATUS_OK else { return }
    throw MfskError(status: status, detail: detail())
}

/// Read a fixed-size C `char` array (imported as a tuple) up to its NUL.
///
/// Tuple-typed, not pointer-typed, because that is how the Clang
/// importer spells `char name[16]` — and reading it through a rebound
/// pointer would depend on the tuple's storage being contiguous, which
/// `withUnsafeBytes` guarantees and a `withMemoryRebound` on the tuple
/// element does not.
func stringFromCArray<T>(_ array: T) -> String {
    withUnsafeBytes(of: array) { raw in
        String(decoding: raw.prefix(while: { $0 != 0 }), as: UTF8.self)
    }
}

/// Write `value` into a fixed-size C `char` array, NUL-terminated and
/// truncated to fit. Returns false if it had to truncate.
@discardableResult
func setCArray<T>(_ destination: inout T, to value: String) -> Bool {
    withUnsafeMutableBytes(of: &destination) { raw in
        for i in raw.indices { raw[i] = 0 }
        let bytes = Array(value.utf8)
        let room = raw.count - 1  // keep the NUL
        let n = min(bytes.count, room)
        for i in 0..<n { raw[i] = bytes[i] }
        return n == bytes.count
    }
}
