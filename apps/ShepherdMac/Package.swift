// swift-tools-version: 5.10
import PackageDescription

let package = Package(
    name: "Shepherd",
    platforms: [.macOS(.v14)],
    targets: [
        .executableTarget(
            name: "Shepherd",
            path: "Sources/Shepherd",
            swiftSettings: [.unsafeFlags(["-O", "-wmo"])]
        )
    ]
)
