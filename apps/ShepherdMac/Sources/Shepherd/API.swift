import CoreLocation
import Foundation

// MARK: - Response models (OSRM-compatible subset returned by the shepherd server)

struct RouteResponse: Decodable {
    let code: String
    let routes: [Route]
    let waypoints: [Waypoint]
    let metric_version: UInt64
}

struct Route: Decodable {
    let distance: Double
    let duration: Double
    let geometry: String?
    let legs: [Leg]
    let traffic_lights: [[Double]]?

    var lights: [CLLocationCoordinate2D] {
        (traffic_lights ?? []).map { CLLocationCoordinate2D(latitude: $0[1], longitude: $0[0]) }
    }
}

struct Leg: Decodable {
    let distance: Double
    let duration: Double
    let summary: String
    let steps: [Step]
}

struct Step: Decodable, Identifiable {
    let id = UUID()
    let distance: Double
    let duration: Double
    let name: String
    let maneuver: Maneuver

    private enum CodingKeys: String, CodingKey { case distance, duration, name, maneuver }
}

struct Maneuver: Decodable {
    let location: [Double]
    let type: String
    let modifier: String?
    let exit: Int?
    let instruction: String

    var coordinate: CLLocationCoordinate2D {
        CLLocationCoordinate2D(latitude: location[1], longitude: location[0])
    }
}

struct Waypoint: Decodable {
    let location: [Double]
    let name: String
    let distance: Double
}

struct APIError: Decodable, Error, LocalizedError {
    let code: String
    let message: String
    var errorDescription: String? { message }
}

struct Health: Decodable {
    struct Graph: Decodable {
        let source: String
        let num_segments: Int
        let num_ebg_nodes: Int
        let num_cch_edges: Int
        let bbox: [Double]
    }
    let status: String
    let graph: Graph
    let metric_version: UInt64
    let customize_ms: Double
}

struct UpdateResult: Decodable {
    struct Active: Decodable {
        let closed: Int
        let speed: Int
        let factor: Int
    }
    let metric_version: UInt64
    let customize_ms: Double
    let segments_affected: Int
    let active: Active
}

// MARK: - Client

struct RoutingClient {
    var base: URL

    private func get<T: Decodable>(_ path: String, query: [URLQueryItem] = []) async throws -> T {
        var comps = URLComponents(url: base.appendingPathComponent(path), resolvingAgainstBaseURL: false)!
        if !query.isEmpty { comps.queryItems = query }
        let (data, response) = try await URLSession.shared.data(from: comps.url!)
        return try decode(data, response)
    }

    private func send<T: Decodable>(_ method: String, _ path: String, body: [String: Any]? = nil) async throws -> T {
        var req = URLRequest(url: URL(string: path, relativeTo: base)!)
        req.httpMethod = method
        if let body {
            req.setValue("application/json", forHTTPHeaderField: "content-type")
            req.httpBody = try JSONSerialization.data(withJSONObject: body)
        }
        let (data, response) = try await URLSession.shared.data(for: req)
        return try decode(data, response)
    }

    private func decode<T: Decodable>(_ data: Data, _ response: URLResponse) throws -> T {
        if let http = response as? HTTPURLResponse, http.statusCode >= 400 {
            throw (try? JSONDecoder().decode(APIError.self, from: data))
                ?? APIError(code: "HTTP\(http.statusCode)", message: "Server error \(http.statusCode)")
        }
        return try JSONDecoder().decode(T.self, from: data)
    }

    func health() async throws -> Health { try await get("health") }

    func route(_ points: [CLLocationCoordinate2D], profile: String, exclude: String?) async throws -> RouteResponse {
        let coords = points.map { String(format: "%.6f,%.6f", $0.longitude, $0.latitude) }.joined(separator: ";")
        var q: [URLQueryItem] = [
            .init(name: "steps", value: "true"),
            .init(name: "overview", value: "full"),
            .init(name: "geometries", value: "polyline6"),
        ]
        if let exclude { q.append(.init(name: "exclude", value: exclude)) }
        return try await get("route/v1/\(profile)/\(coords)", query: q)
    }

    func closeRoads(near c: CLLocationCoordinate2D, radius: Double, profile: String) async throws -> UpdateResult {
        try await send("POST", "admin/updates?profile=\(profile)", body: [
            "closures": [["near": [c.longitude, c.latitude], "radius_m": radius]]
        ])
    }

    func slowDown(near c: CLLocationCoordinate2D, radius: Double, factor: Double, profile: String) async throws -> UpdateResult {
        try await send("POST", "admin/updates?profile=\(profile)", body: [
            "traffic": [["near": [c.longitude, c.latitude], "radius_m": radius, "factor": factor]]
        ])
    }

    func clearUpdates(profile: String) async throws -> UpdateResult {
        try await send("DELETE", "admin/updates?profile=\(profile)")
    }
}

// MARK: - Polyline decoding

enum Polyline {
    static func decode(_ s: String, precision: Double = 1e6) -> [CLLocationCoordinate2D] {
        var coords: [CLLocationCoordinate2D] = []
        coords.reserveCapacity(s.utf8.count / 4)
        var lat = 0, lon = 0
        var it = s.utf8.makeIterator()
        func next() -> Int? {
            var result = 0, shift = 0
            while let b = it.next() {
                let v = Int(b) - 63
                result |= (v & 0x1f) << shift
                shift += 5
                if v < 0x20 { return (result & 1) != 0 ? ~(result >> 1) : (result >> 1) }
            }
            return nil
        }
        while let dlat = next(), let dlon = next() {
            lat += dlat
            lon += dlon
            coords.append(CLLocationCoordinate2D(latitude: Double(lat) / precision, longitude: Double(lon) / precision))
        }
        return coords
    }
}
