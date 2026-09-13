// SPDX-License-Identifier: GPL-3.0-or-later
//
// Process-wide settings: the thread pool, and the two version numbers.

import CMfsk

/// Library-wide configuration and identity.
public enum Runtime {
    /// Configure the thread pool every subsequent decode runs on.
    ///
    /// **Call once, before the first decode.** The pool is built on the
    /// first call and kept for the life of the process; a second call
    /// throws ``MfskError/Code/unsupported`` rather than silently
    /// ignoring you, because rayon cannot rebuild a pool threads may be
    /// parked in. A build without the `parallel` feature throws the same
    /// error — there is one thread there and nothing to configure, which
    /// is a *stronger* contract rather than a missing one.
    ///
    /// - Parameters:
    ///   - threadCount: 0 for the default (`num_cpus`); **1 forces
    ///     serial decoding**.
    ///   - stackBytesPerThread: 0 for rayon's 2 MiB default — i.e.
    ///     `num_cpus × 2 MiB` of address space reserved before the first
    ///     sample is decoded, which is worth lowering on iOS.
    ///
    /// The ABI's two thread hooks are deliberately not wrapped: they
    /// exist so an Android JNI consumer can `AttachCurrentThread` on
    /// each worker, and there is no Apple-platform equivalent to call.
    public static func configure(threadCount: UInt32 = 0, stackBytesPerThread: UInt32 = 0) throws {
        var config = MfskRuntimeConfig()
        config.size = UInt32(MemoryLayout<MfskRuntimeConfig>.size)
        config.num_threads = threadCount
        config.thread_stack_bytes = stackBytesPerThread
        try withUnsafePointer(to: &config) { try check(mfsk_runtime_configure($0)) }
    }

    /// How many worker threads decoding will use. 1 means serial.
    public static var threadCount: UInt32 { mfsk_runtime_thread_count() }

    /// The ABI revision. This moves only when the C surface changes
    /// shape, so it is the one to check before deciding a header and a
    /// library agree — ``libraryVersion`` moves for release reasons that
    /// have nothing to do with the boundary.
    public static var abiVersion: UInt32 { mfsk_abi_version() }

    /// The library's release number.
    public static var libraryVersion: Version {
        let packed = mfsk_version()
        return Version(major: Int((packed >> 16) & 0xFF),
                       minor: Int((packed >> 8) & 0xFF),
                       patch: Int(packed & 0xFF))
    }

    public struct Version: Sendable, Equatable, Comparable, CustomStringConvertible {
        public let major: Int
        public let minor: Int
        public let patch: Int

        public var description: String { "\(major).\(minor).\(patch)" }

        public static func < (lhs: Version, rhs: Version) -> Bool {
            (lhs.major, lhs.minor, lhs.patch) < (rhs.major, rhs.minor, rhs.patch)
        }
    }
}
