// swift-tools-version: 6.2
import PackageDescription

// WhisperKit and FluidAudio are not marked Sendable yet: Swift 5 mode.
let swift5: [SwiftSetting] = [.swiftLanguageMode(.v5)]

let package = Package(
    name: "Scribe",
    platforms: [.macOS(.v15)],
    products: [
        .executable(name: "Scribe", targets: ["Scribe"]),
        .executable(name: "scribe-bench", targets: ["scribe-bench"]),
    ],
    dependencies: [
        .package(url: "https://github.com/argmaxinc/argmax-oss-swift.git", from: "1.1.1"),
        .package(url: "https://github.com/FluidInference/FluidAudio.git", from: "0.18.2", traits: ["Diarizer"]),
    ],
    targets: [
        .target(name: "ScribeKit", dependencies: [
            .product(name: "WhisperKit", package: "argmax-oss-swift"),
            .product(name: "FluidAudio", package: "FluidAudio"),
        ], swiftSettings: swift5),
        .executableTarget(name: "Scribe", dependencies: ["ScribeKit"], swiftSettings: swift5),
        .executableTarget(name: "scribe-bench", dependencies: ["ScribeKit"], swiftSettings: swift5),
    ]
)
