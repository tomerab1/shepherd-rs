import CoreLocation
import Foundation
import MapKit

/// The routing engine running on the phone (the Rust library, via its C interface).
/// Handles are thread-safe; calls block, so run them off the main thread.
final class LocalEngine: @unchecked Sendable {
    private let handle: OpaquePointer
    let profile: String

    init(graph: URL, profile: String, avoidArea: (name: String, geojson: URL)?) throws {
        var err: UnsafeMutablePointer<CChar>?
        let h = graph.path.withCString { g in
            (avoidArea?.name ?? "").withCString { n in
                (avoidArea?.geojson.path ?? "").withCString { a in
                    shepherd_open(g, n, a, &err)
                }
            }
        }
        guard let h else {
            let msg = err.map { String(cString: $0) } ?? "unknown error"
            if let err { shepherd_string_free(err) }
            throw APIError(code: "EngineLoad", message: "Couldn't load \(graph.lastPathComponent): \(msg)")
        }
        handle = h
        self.profile = profile
    }

    deinit { shepherd_close(handle) }

    /// Takes ownership of a JSON string from the library and decodes it.
    private func decode<T: Decodable>(_ p: UnsafeMutablePointer<CChar>?) throws -> T {
        guard let p else { throw APIError(code: "InternalError", message: "engine returned nothing") }
        defer { shepherd_string_free(p) }
        let data = Data(bytesNoCopy: p, count: strlen(p), deallocator: .none)
        if let e = try? JSONDecoder().decode(APIError.self, from: data), e.code != "Ok" { throw e }
        return try JSONDecoder().decode(T.self, from: data)
    }

    func route(_ points: [CLLocationCoordinate2D], exclude: String?) throws -> RouteResponse {
        var lonlat: [Double] = []
        lonlat.reserveCapacity(points.count * 2)
        for p in points { lonlat += [p.longitude, p.latitude] }
        let out = lonlat.withUnsafeBufferPointer { buf in
            (exclude ?? "").withCString { ex in shepherd_route(handle, buf.baseAddress, points.count, ex) }
        }
        return try decode(out)
    }

    /// "estimate", "timed-only" or "off".
    func setSignalMode(_ mode: String) {
        _ = shepherd_set_signal_mode(handle, mode == "timed-only" ? 1 : mode == "off" ? 2 : 0)
    }

    private struct Lights: Decodable { let lights: [[Double]] }

    func lights(in r: MKCoordinateRegion) throws -> [CLLocationCoordinate2D] {
        let (s, n) = (r.center.latitude - r.span.latitudeDelta / 2, r.center.latitude + r.span.latitudeDelta / 2)
        let (w, e) = (r.center.longitude - r.span.longitudeDelta / 2, r.center.longitude + r.span.longitudeDelta / 2)
        let l: Lights = try decode(shepherd_lights(handle, w, s, e, n))
        return l.lights.map { CLLocationCoordinate2D(latitude: $0[1], longitude: $0[0]) }
    }
}

/// Both profiles, loaded from the app bundle.
struct Engines: Sendable {
    let drive: LocalEngine
    let walk: LocalEngine

    static func load() throws -> Engines {
        let b = Bundle.main
        guard let car = b.url(forResource: "il", withExtension: "graph"),
              let foot = b.url(forResource: "il-foot", withExtension: "graph")
        else { throw APIError(code: "NoData", message: "Routing data is missing from the app.") }
        let area = b.url(forResource: "judea-samaria", withExtension: "geojson").map { (name: "judea-samaria", geojson: $0) }
        // Build both metrics in parallel.
        var d: Result<LocalEngine, Error>?
        var w: Result<LocalEngine, Error>?
        DispatchQueue.concurrentPerform(iterations: 2) { i in
            if i == 0 {
                d = Result { try LocalEngine(graph: car, profile: "driving", avoidArea: area) }
            } else {
                w = Result { try LocalEngine(graph: foot, profile: "foot", avoidArea: area) }
            }
        }
        return Engines(drive: try d!.get(), walk: try w!.get())
    }

    func engine(for mode: TravelMode) -> LocalEngine { mode == .drive ? drive : walk }
}
