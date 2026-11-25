import Foundation

/// Starts the bundled `shepherd` routing server (unless one is already answering on the port)
/// and stops it when the app quits.
@MainActor
final class EngineProcess {
    static let shared = EngineProcess()

    let port: Int
    private var process: Process?

    var baseURL: URL { URL(string: "http://127.0.0.1:\(port)")! }

    private init() {
        port = Int(ProcessInfo.processInfo.environment["SHEPHERD_PORT"] ?? "") ?? 5057
    }

    /// The server binary ships inside the app bundle (overridable for development).
    private var binary: URL? {
        if let p = ProcessInfo.processInfo.environment["SHEPHERD_BIN"] { return URL(fileURLWithPath: p) }
        return Bundle.main.url(forResource: "shepherd", withExtension: nil)
    }

    /// Graph files live in ~/Library/Application Support/Shepherd so they can be refreshed
    /// (e.g. by the nightly OSM refresh script) without rebuilding the app.
    static var dataDir: URL {
        if let p = ProcessInfo.processInfo.environment["SHEPHERD_DATA"] { return URL(fileURLWithPath: p) }
        return FileManager.default.homeDirectoryForCurrentUser
            .appendingPathComponent("Library/Application Support/Shepherd")
    }

    private func data(_ name: String) -> URL? {
        let u = Self.dataDir.appendingPathComponent(name)
        return FileManager.default.fileExists(atPath: u.path) ? u : nil
    }

    private var logURL: URL {
        let dir = FileManager.default.homeDirectoryForCurrentUser.appendingPathComponent("Library/Logs/Shepherd")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir.appendingPathComponent("engine.log")
    }

    func startIfNeeded() async throws {
        if await isUp() { return }
        let graphs = ["il.graph", "il-foot.graph"].compactMap(data)
        guard let binary, !graphs.isEmpty else {
            throw APIError(code: "NoEngine", message: "Routing data not found in \(Self.dataDir.path).")
        }
        var args = ["serve"] + graphs.map(\.path) + ["--bind", "127.0.0.1:\(port)"]
        if let area = data("judea-samaria.geojson") {
            args += ["--avoid-area", "judea-samaria=\(area.path)"]
        }
        let p = Process()
        p.executableURL = binary
        p.arguments = args
        var env = ProcessInfo.processInfo.environment
        if let key = Keychain.get("here-api-key"), !key.isEmpty {
            env["HERE_API_KEY"] = key
        } else {
            env.removeValue(forKey: "HERE_API_KEY")
        }
        p.environment = env
        FileManager.default.createFile(atPath: logURL.path, contents: nil)
        let log = try FileHandle(forWritingTo: logURL)
        p.standardOutput = log
        p.standardError = log
        try p.run()
        process = p
        // Mapping the graphs is instant; customizing takes about a second per profile.
        for _ in 0..<600 {
            if await isUp() { return }
            if !p.isRunning { break }
            try await Task.sleep(for: .milliseconds(100))
        }
        throw APIError(code: "NoEngine", message: "The routing engine did not start. See ~/Library/Logs/Shepherd/engine.log")
    }

    func isUp() async -> Bool {
        var req = URLRequest(url: baseURL.appendingPathComponent("health"))
        req.timeoutInterval = 0.5
        guard let (_, resp) = try? await URLSession.shared.data(for: req) else { return false }
        return (resp as? HTTPURLResponse)?.statusCode == 200
    }

    /// Restarts the engine we started (e.g. after the traffic API key changed).
    func restart() async throws {
        stop()
        for _ in 0..<50 where await isUp() {
            try await Task.sleep(for: .milliseconds(100))
        }
        try await startIfNeeded()
    }

    func stop() {
        guard let p = process, p.isRunning else { return }
        p.terminate() // SIGTERM: the server shuts down gracefully.
        p.waitUntilExit()
        process = nil
    }
}
