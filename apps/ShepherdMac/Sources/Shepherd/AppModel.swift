import MapKit
import SwiftUI

enum ClickMode: String, CaseIterable, Identifiable {
    case start = "Start"
    case end = "Destination"
    case close = "Close road"
    case slow = "Traffic jam"
    var id: String { rawValue }
    var symbol: String {
        switch self {
        case .start: "circle.circle.fill"
        case .end: "mappin.circle.fill"
        case .close: "xmark.octagon.fill"
        case .slow: "car.2.fill"
        }
    }
}

enum TravelMode: String, CaseIterable, Identifiable {
    case drive = "Drive"
    case walk = "Walk"
    var id: String { rawValue }
    var profile: String { self == .drive ? "driving" : "foot" }
    var symbol: String { self == .drive ? "car.fill" : "figure.walk" }
    /// Speed used by the simulator (m/s).
    var simSpeed: Double { self == .drive ? 13.9 : 1.4 }
    /// How far from the route counts as "off route" (m).
    var offRouteM: Double { self == .drive ? 60 : 35 }
}

struct Incident: Identifiable {
    let id = UUID()
    let coordinate: CLLocationCoordinate2D
    let closed: Bool
}

enum EngineStatus {
    case starting
    case ready(Health)
    case failed(String)
}

@MainActor
@Observable
final class AppModel {
    var status: EngineStatus = .starting
    var start: CLLocationCoordinate2D? { didSet { startText = Self.format(start) } }
    var end: CLLocationCoordinate2D? { didSet { endText = Self.format(end) } }
    var startText = ""
    var endText = ""
    var clickMode: ClickMode = .start
    var mode: TravelMode = .drive
    var avoidJudeaSamaria = false
    var route: Route?
    var routeCoords: [CLLocationCoordinate2D] = []
    var routedMetricVersion: UInt64 = 0
    var computeMs: Double = 0
    var isRouting = false
    var errorMessage: String?
    var incidents: [Incident] = []
    var lastUpdateMs: Double?
    var selectedStep: Step.ID?
    var camera: MapCameraPosition = .region(
        MKCoordinateRegion(
            center: CLLocationCoordinate2D(latitude: 31.9, longitude: 34.95),
            span: MKCoordinateSpan(latitudeDelta: 1.2, longitudeDelta: 1.2)))

    // Location & navigation
    let location = LocationTracker()
    var waitingForLocation = false
    var navigating = false
    var simulate = false
    var nav = NavState()
    var track: RouteTrack?
    var fix: CLLocationCoordinate2D?
    private let simulator = RouteSimulator()
    private var rerouting = false

    private var client: RoutingClient { RoutingClient(base: EngineProcess.shared.baseURL) }

    var steps: [Step] { route?.legs.flatMap(\.steps) ?? [] }
    var exclude: String? { avoidJudeaSamaria ? "judea-samaria" : nil }

    init() {
        location.onFix = { [weak self] loc in self?.handleFix(loc) }
    }

    // MARK: Engine

    func boot() async {
        status = .starting
        do {
            try await EngineProcess.shared.startIfNeeded()
            let h = try await client.health()
            status = .ready(h)
            if start == nil && end == nil {
                // A useful first route: Tel Aviv (Azrieli) to Jerusalem (Jaffa Gate).
                start = CLLocationCoordinate2D(latitude: 32.0744, longitude: 34.7918)
                end = CLLocationCoordinate2D(latitude: 31.7767, longitude: 35.2276)
                await computeRoute()
            }
        } catch {
            status = .failed(error.localizedDescription)
        }
    }

    // MARK: Interaction

    func mapClicked(_ c: CLLocationCoordinate2D) async {
        switch clickMode {
        case .start:
            start = c
            clickMode = .end
            if end != nil { await computeRoute() }
        case .end:
            end = c
            await computeRoute()
        case .close:
            await addIncident(at: c, closed: true)
        case .slow:
            await addIncident(at: c, closed: false)
        }
    }

    func commitText() async {
        if !startText.isEmpty && Self.parse(startText) == nil {
            errorMessage = "Start must be \"lat, lon\" (e.g. 32.0744, 34.7918)"
            return
        }
        if !endText.isEmpty && Self.parse(endText) == nil {
            errorMessage = "Destination must be \"lat, lon\" (e.g. 31.7767, 35.2276)"
            return
        }
        if let s = Self.parse(startText) { start = s }
        if let e = Self.parse(endText) { end = e }
        await computeRoute()
    }

    func swap() async {
        (start, end) = (end, start)
        await computeRoute()
    }

    func clear() {
        stopNavigation()
        start = nil
        end = nil
        route = nil
        routeCoords = []
        track = nil
        errorMessage = nil
        clickMode = .start
    }

    func computeRoute(fit: Bool = true) async {
        guard let s = start, let e = end else { return }
        isRouting = true
        defer { isRouting = false }
        let t0 = Date()
        do {
            let r = try await client.route([s, e], profile: mode.profile, exclude: exclude)
            computeMs = Date().timeIntervalSince(t0) * 1000
            route = r.routes.first
            routedMetricVersion = r.metric_version
            routeCoords = route?.geometry.map { Polyline.decode($0) } ?? []
            if let route {
                track = RouteTrack(coords: routeCoords, steps: steps, duration: route.duration)
            }
            nav.index = 0
            errorMessage = nil
            if fit && !navigating { fitRoute() }
        } catch {
            route = nil
            routeCoords = []
            track = nil
            errorMessage = error.localizedDescription
        }
    }

    func addIncident(at c: CLLocationCoordinate2D, closed: Bool) async {
        do {
            let r = closed
                ? try await client.closeRoads(near: c, radius: 40, profile: mode.profile)
                : try await client.slowDown(near: c, radius: 60, factor: 6, profile: mode.profile)
            lastUpdateMs = r.customize_ms
            if r.segments_affected == 0 {
                errorMessage = "No road within reach of that point."
                return
            }
            incidents.append(Incident(coordinate: c, closed: closed))
            await computeRoute(fit: false)
        } catch {
            errorMessage = error.localizedDescription
        }
    }

    func clearIncidents() async {
        do {
            for p in ["driving", "foot"] { _ = try? await client.clearUpdates(profile: p) }
            incidents.removeAll()
            lastUpdateMs = nil
            await computeRoute(fit: false)
        }
    }

    func focus(_ step: Step) {
        selectedStep = step.id
        withAnimation {
            camera = .camera(MapCamera(centerCoordinate: step.maneuver.coordinate, distance: 1200))
        }
    }

    func fitRoute() {
        guard !routeCoords.isEmpty else { return }
        var rect = MKMapRect.null
        for c in routeCoords {
            let p = MKMapPoint(c)
            rect = rect.union(MKMapRect(x: p.x, y: p.y, width: 0, height: 0))
        }
        let pad = max(rect.width, rect.height) * 0.15 + 1500
        withAnimation { camera = .rect(rect.insetBy(dx: -pad, dy: -pad)) }
    }

    // MARK: Location

    /// Sets the start to the device's current position (asks for permission the first time).
    func useMyLocation() {
        if let last = location.last, Date().timeIntervalSince(last.timestamp) < 60 {
            setStartFromLocation(last.coordinate)
            return
        }
        waitingForLocation = true
        location.start()
    }

    private func setStartFromLocation(_ c: CLLocationCoordinate2D) {
        waitingForLocation = false
        start = c
        fix = c
        clickMode = .end
        if end != nil { Task { await computeRoute() } } else {
            withAnimation { camera = .camera(MapCamera(centerCoordinate: c, distance: 3000)) }
        }
    }

    // MARK: Navigation

    func startNavigation() {
        guard track != nil else { return }
        navigating = true
        nav = NavState()
        selectedStep = nil
        if simulate, let track {
            simulator.start(track: track, speed: mode.simSpeed) { [weak self] loc in self?.handleFix(loc) }
        } else {
            location.start()
            if let last = location.last { handleFix(last) }
        }
    }

    func stopNavigation() {
        navigating = false
        simulator.stop()
        if !waitingForLocation { location.stop() }
    }

    func handleFix(_ loc: CLLocation) {
        if waitingForLocation {
            setStartFromLocation(loc.coordinate)
            if !navigating { location.stop() }
            return
        }
        fix = loc.coordinate
        guard navigating, let track else { return }

        // Measured pace: GPS speed when valid, otherwise distance / time between fixes.
        var measured: Double?
        if loc.speed >= 0 {
            measured = loc.speed
        } else if let prev = nav.lastFix {
            let dt = loc.timestamp.timeIntervalSince(prev.timestamp)
            if dt >= 1 { measured = loc.distance(from: prev) / dt }
        }
        if let v = measured, loc.horizontalAccuracy < 80 {
            nav.speed = nav.speed.map { $0 * 0.7 + v * 0.3 } ?? v
        }
        nav.lastFix = loc

        let p = track.project(loc.coordinate, hint: nav.index)
        nav.index = p.index
        nav.along = p.along
        nav.offRoute = p.off
        nav.remaining = max(0, track.total - p.along)

        // Time left: walking uses your measured pace; driving uses the traffic-aware route
        // estimate for the remaining part (instant car speed is too noisy).
        let routeEstimate = track.total > 0 ? track.duration * nav.remaining / track.total : 0
        if mode == .walk, let v = nav.speed, v > 0.3 {
            nav.remainingTime = nav.remaining / v
        } else {
            nav.remainingTime = routeEstimate
        }
        nav.eta = Date().addingTimeInterval(nav.remainingTime)

        if let i = track.stepAt.firstIndex(where: { $0 > p.along + 3 }) {
            nav.nextStep = i
            nav.toNext = track.stepAt[i] - p.along
        } else {
            nav.nextStep = max(0, track.stepAt.count - 1)
            nav.toNext = nav.remaining
        }

        if nav.remaining < 20 {
            nav.arrived = true
            simulator.stop()
            if !simulate { location.stop() }
        }

        withAnimation(.easeInOut(duration: 0.8)) {
            camera = .camera(MapCamera(
                centerCoordinate: loc.coordinate,
                distance: mode == .walk ? 700 : 2500,
                heading: loc.course >= 0 ? loc.course : 0))
        }

        // Off route for a few fixes in a row: re-route from here.
        if p.off > mode.offRouteM && loc.horizontalAccuracy < 80 {
            nav.offRouteFixes += 1
        } else {
            nav.offRouteFixes = 0
        }
        if nav.offRouteFixes >= 3 && !rerouting && !nav.arrived {
            rerouting = true
            nav.offRouteFixes = 0
            start = loc.coordinate
            Task {
                await computeRoute(fit: false)
                rerouting = false
            }
        }
    }

    // MARK: Formatting

    static func format(_ c: CLLocationCoordinate2D?) -> String {
        guard let c else { return "" }
        return String(format: "%.5f, %.5f", c.latitude, c.longitude)
    }

    static func parse(_ s: String) -> CLLocationCoordinate2D? {
        let parts = s.split(whereSeparator: { $0 == "," || $0 == " " }).compactMap { Double($0) }
        guard parts.count == 2, (-90...90).contains(parts[0]), (-180...180).contains(parts[1]) else { return nil }
        return CLLocationCoordinate2D(latitude: parts[0], longitude: parts[1])
    }

    static func distance(_ m: Double) -> String {
        m < 1000 ? "\(Int((m / 10).rounded() * 10)) m" : String(format: "%.1f km", m / 1000)
    }

    static func duration(_ s: Double) -> String {
        let mins = Int((s / 60).rounded())
        if mins < 1 { return "<1 min" }
        return mins < 60 ? "\(mins) min" : "\(mins / 60) h \(String(format: "%02d", mins % 60)) min"
    }

    static func speed(_ mps: Double?) -> String {
        guard let v = mps else { return "–" }
        return String(format: "%.1f km/h", v * 3.6)
    }
}
