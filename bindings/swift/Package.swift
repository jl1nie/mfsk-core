// swift-tools-version: 5.9
//
// SwiftPM wrapper around mfsk-ffi's C ABI. The library it links,
// `libmfsk`, is built by cargo out of `mfsk-ffi/` — `scripts/test.sh`
// does both halves and passes the `-L` this package deliberately does
// not hardcode: an `unsafeFlags` here would bar the package from being
// used as a dependency at all, which is the one thing a binding must
// not do.
import PackageDescription

let package = Package(
    name: "MfskCore",
    products: [
        .library(name: "MfskCore", targets: ["MfskCore"])
    ],
    targets: [
        .systemLibrary(name: "CMfsk", path: "Sources/CMfsk"),
        .target(name: "MfskCore", dependencies: ["CMfsk"]),
        // CMfsk is a test dependency too, so `ABIContractTests` can
        // check this binding's own constants against the header's
        // rather than restating them and hoping.
        .testTarget(name: "MfskCoreTests", dependencies: ["MfskCore", "CMfsk"]),
    ]
)
