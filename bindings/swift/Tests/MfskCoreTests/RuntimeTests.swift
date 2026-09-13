// SPDX-License-Identifier: GPL-3.0-or-later

import XCTest
@testable import MfskCore

final class RuntimeTests: XCTestCase {
    func testVersionsAreReported() {
        XCTAssertGreaterThan(Runtime.libraryVersion, Runtime.Version(major: 0, minor: 0, patch: 0))
        XCTAssertGreaterThanOrEqual(Runtime.abiVersion, 2)
    }

    func testThreadCountIsAtLeastOne() {
        // 1 means serial — either a build without `parallel`, or a pool
        // configured that way. Both are legal; 0 never is.
        XCTAssertGreaterThanOrEqual(Runtime.threadCount, 1)
    }

    func testConfiguringTwiceIsRefusedRatherThanIgnored() throws {
        // Process-wide and one-shot by design: the pool is built on the
        // first call and kept for the life of the process, so this test
        // asserts the *contract* without caring which call was first —
        // another test having decoded already makes the first attempt
        // the refused one, and that is still the contract.
        let first = Result { try Runtime.configure(threadCount: Runtime.threadCount) }
        let second = Result { try Runtime.configure(threadCount: Runtime.threadCount) }
        if case .success = first, case .success = second {
            XCTFail("two configure calls both succeeded; the pool cannot be rebuilt")
        }
        if case .failure(let error) = second {
            XCTAssertEqual((error as? MfskError)?.code, .unsupported)
        }
    }
}
