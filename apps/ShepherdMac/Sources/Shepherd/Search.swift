import MapKit
import Security
import SwiftUI

/// Address / place search with autocomplete, backed by Apple MapKit (no API key needed).
@MainActor
@Observable
final class PlaceSearch: NSObject, MKLocalSearchCompleterDelegate {
    var suggestions: [MKLocalSearchCompletion] = []
    private let completer = MKLocalSearchCompleter()

    /// Searches are biased to (not limited to) Israel.
    static let region = MKCoordinateRegion(
        center: CLLocationCoordinate2D(latitude: 31.5, longitude: 34.9),
        span: MKCoordinateSpan(latitudeDelta: 4.5, longitudeDelta: 2.5))

    override init() {
        super.init()
        completer.delegate = self
        completer.resultTypes = [.address, .pointOfInterest]
        completer.region = Self.region
    }

    func update(_ text: String) {
        let q = text.trimmingCharacters(in: .whitespaces)
        if q.count < 2 || AppModel.parse(q) != nil {
            completer.cancel()
            suggestions = []
        } else {
            completer.queryFragment = q
        }
    }

    func clear() {
        completer.cancel()
        suggestions = []
    }

    nonisolated func completerDidUpdateResults(_ c: MKLocalSearchCompleter) {
        let results = Array(c.results.prefix(6))
        Task { @MainActor in self.suggestions = results }
    }

    nonisolated func completer(_ c: MKLocalSearchCompleter, didFailWithError error: Error) {
        Task { @MainActor in self.suggestions = [] }
    }

    /// Resolves a picked suggestion to a coordinate and a display name.
    static func resolve(_ s: MKLocalSearchCompletion) async throws -> (CLLocationCoordinate2D, String) {
        let resp = try await MKLocalSearch(request: MKLocalSearch.Request(completion: s)).start()
        guard let item = resp.mapItems.first else {
            throw APIError(code: "NotFound", message: "Couldn't find \"\(s.title)\".")
        }
        return (item.placemark.coordinate, display(s.title, s.subtitle))
    }

    /// Free-text search (when the user presses Return without picking a suggestion).
    static func search(_ text: String) async throws -> (CLLocationCoordinate2D, String) {
        let req = MKLocalSearch.Request()
        req.naturalLanguageQuery = text
        req.region = region
        let resp = try await MKLocalSearch(request: req).start()
        guard let item = resp.mapItems.first else {
            throw APIError(code: "NotFound", message: "No place found for \"\(text)\".")
        }
        return (item.placemark.coordinate, item.name ?? text)
    }

    static func display(_ title: String, _ subtitle: String) -> String {
        subtitle.isEmpty ? title : "\(title), \(subtitle)"
    }
}

/// Minimal Keychain storage for the HERE API key.
enum Keychain {
    private static let service = "local.shepherd-rs.Shepherd"

    static func get(_ account: String) -> String? {
        let q: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: service,
            kSecAttrAccount as String: account,
            kSecReturnData as String: true,
            kSecMatchLimit as String: kSecMatchLimitOne,
        ]
        var out: AnyObject?
        guard SecItemCopyMatching(q as CFDictionary, &out) == errSecSuccess, let d = out as? Data else { return nil }
        return String(data: d, encoding: .utf8)
    }

    static func set(_ account: String, _ value: String?) {
        let base: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: service,
            kSecAttrAccount as String: account,
        ]
        SecItemDelete(base as CFDictionary)
        guard let value, !value.isEmpty else { return }
        var add = base
        add[kSecValueData as String] = Data(value.utf8)
        SecItemAdd(add as CFDictionary, nil)
    }
}
