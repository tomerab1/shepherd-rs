import MapKit
import SwiftUI
import UIKit

enum TravelMode: String, CaseIterable, Identifiable {
    case drive = "Drive"
    case walk = "Walk"
    var id: String { rawValue }
    var symbol: String { self == .drive ? "car.fill" : "figure.walk" }
    /// Speed used by the simulator (m/s).
    var simSpeed: Double { self == .drive ? 13.9 : 1.4 }
    /// How far from the route counts as "off route" (m).
    var offRouteM: Double { self == .drive ? 50 : 30 }
}

enum EngineStatus {
    case loading
    case ready
    case failed(String)
}

@MainActor
@Observable
final class PhoneModel {
    var status: EngineStatus = .loading
    private var engines: Engines?

    var start: CLLocationCoordinate2D?
    var end: CLLocationCoordinate2D?
    var startText = ""
    var endText = ""
    private var startResolvedText = ""
    private var endResolvedText = ""
    /// The start follows the device ("My location") until the user picks another one.
    var startIsMyLocation = true
    let startSearch = PlaceSearch()
    let endSearch = PlaceSearch()

    var mode: TravelMode = .walk {
        didSet { if mode != oldValue { Task { await computeRoute() } } }
    }
    var avoidJudeaSamaria = false {
        didSet { if avoidJudeaSamaria != oldValue { Task { await computeRoute(fit: false) } } }
    }
    var signalMode = "estimate"
    var showAllLights = true
    var viewLights: [CLLocationCoordinate2D] = []
    private var lightsTask: Task<Void, Never>?

    var route: Route?
    var routeCoords: [CLLocationCoordinate2D] = []
    var computeMs: Double = 0
    var isRouting = false
    var errorMessage: String?
    var camera: MapCameraPosition = .userLocation(
        fallback: .region(MKCoordinateRegion(
            center: CLLocationCoordinate2D(latitude: 32.07, longitude: 34.79),
            span: MKCoordinateSpan(latitudeDelta: 0.08, longitudeDelta: 0.08))))

    // Location & navigation
    let location = LocationTracker()
    let voice = VoiceGuide()
    var navigating = false
    var simulate = false
    var simMultiplier: Double = 5
    var nav = NavState()
    var track: RouteTrack?
    var fix: CLLocationCoordinate2D?
    /// While navigating, the camera follows you unless you pan away.
    var following = true
    private let simulator = RouteSimulator()
    private var rerouting = false

    var steps: [Step] { route?.legs.flatMap(\.steps) ?? [] }
    var exclude: String? { avoidJudeaSamaria ? "judea-samaria" : nil }

    init() {
        location.onFix = { [weak self] loc in self?.handleFix(loc) }
        startText = "My location"
        startResolvedText = startText
    }

    // MARK: Engine

    func boot() async {
        location.start()
        let t0 = Date()
        do {
            engines = try await Task.detached(priority: .userInitiated) { try Engines.load() }.value
            status = .ready
            print("engines ready in \(Int(Date().timeIntervalSince(t0) * 1000)) ms")
            if end != nil { await computeRoute() }
        } catch {
            status = .failed(error.localizedDescription)
        }
    }

    private var engine: LocalEngine? { engines?.engine(for: mode) }

    // MARK: Places

    func setStart(_ c: CLLocationCoordinate2D?, label: String?) {
        start = c
        startIsMyLocation = false
        startText = label ?? Self.format(c)
        startResolvedText = startText
        startSearch.clear()
    }

    func useMyLocationAsStart() {
        startIsMyLocation = true
        startText = "My location"
        startResolvedText = startText
        startSearch.clear()
        location.start()
        Task { await computeRoute() }
    }

    func setEnd(_ c: CLLocationCoordinate2D?, label: String?) {
        end = c
        endText = label ?? Self.format(c)
        endResolvedText = endText
        endSearch.clear()
    }

    func pick(_ s: MKLocalSearchCompletion, forStart: Bool) async {
        do {
            let (c, name) = try await PlaceSearch.resolve(s)
            if forStart { setStart(c, label: name) } else { setEnd(c, label: name) }
            await computeRoute()
        } catch {
            errorMessage = error.localizedDescription
        }
    }

    private func resolveField(_ text: String, current: CLLocationCoordinate2D?, resolved: String) async throws -> (CLLocationCoordinate2D, String)? {
        let t = text.trimmingCharacters(in: .whitespaces)
        if t.isEmpty { return nil }
        if let c = Self.parse(t) { return (c, Self.format(c)) }
        if t == resolved, let current { return (current, t) }
        return try await PlaceSearch.search(t)
    }

    func commitText(start isStart: Bool) async {
        do {
            if isStart {
                if startText.trimmingCharacters(in: .whitespaces).isEmpty || startText == "My location" {
                    useMyLocationAsStart()
                    return
                }
                if let (c, name) = try await resolveField(startText, current: start, resolved: startResolvedText) {
                    setStart(c, label: name)
                }
            } else if let (c, name) = try await resolveField(endText, current: end, resolved: endResolvedText) {
                setEnd(c, label: name)
            }
            await computeRoute()
        } catch {
            errorMessage = error.localizedDescription
        }
    }

    /// Long-press on the map: route there.
    func mapPressed(_ c: CLLocationCoordinate2D) async {
        guard !navigating else { return }
        setEnd(c, label: nil)
        await computeRoute()
    }

    func swap() async {
        guard let e = end else { return }
        let (et) = endText
        if startIsMyLocation {
            setEnd(fix, label: fix == nil ? "" : "My location")
        } else {
            setEnd(start, label: startText)
        }
        setStart(e, label: et)
        await computeRoute()
    }

    func clear() {
        stopNavigation()
        setEnd(nil, label: "")
        route = nil
        routeCoords = []
        track = nil
        errorMessage = nil
    }

    // MARK: Routing

    /// Where the route starts: the typed place, or the device's position.
    private var origin: CLLocationCoordinate2D? { startIsMyLocation ? (fix ?? location.last?.coordinate) : start }

    func computeRoute(fit: Bool = true) async {
        guard let engine, let e = end else { return }
        guard let s = origin else {
            errorMessage = location.error ?? "Waiting for your location… (or type a start)"
            return
        }
        isRouting = true
        defer { isRouting = false }
        let t0 = Date()
        let exclude = exclude
        do {
            let r = try await Task.detached(priority: .userInitiated) { try engine.route([s, e], exclude: exclude) }.value
            computeMs = Date().timeIntervalSince(t0) * 1000
            route = r.routes.first
            routeCoords = route?.geometry.map { Polyline.decode($0) } ?? []
            if let route { track = RouteTrack(coords: routeCoords, steps: steps, duration: route.duration) }
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

    func applySignalMode() async {
        guard let engines else { return }
        let m = signalMode
        await Task.detached {
            engines.drive.setSignalMode(m)
            engines.walk.setSignalMode(m)
        }.value
        await computeRoute(fit: false)
    }

    func mapRegionChanged(_ r: MKCoordinateRegion) {
        lightsTask?.cancel()
        guard showAllLights, let engine, r.span.latitudeDelta < 0.03, r.span.longitudeDelta < 0.03 else {
            viewLights = []
            return
        }
        lightsTask = Task {
            try? await Task.sleep(for: .milliseconds(150))
            guard !Task.isCancelled else { return }
            if let l = try? await Task.detached(operation: { try engine.lights(in: r) }).value, !Task.isCancelled {
                viewLights = l
            }
        }
    }

    func fitRoute() {
        guard !routeCoords.isEmpty else { return }
        var rect = MKMapRect.null
        for c in routeCoords {
            let p = MKMapPoint(c)
            rect = rect.union(MKMapRect(x: p.x, y: p.y, width: 0, height: 0))
        }
        let pad = max(rect.width, rect.height) * 0.25 + 300
        withAnimation { camera = .rect(rect.insetBy(dx: -pad, dy: -pad * 1.6)) }
    }

    // MARK: Navigation

    func startNavigation() {
        guard let track else { return }
        navigating = true
        following = true
        nav = NavState()
        UIApplication.shared.isIdleTimerDisabled = true
        voice.begin()
        if let first = steps.first { voice.say(first.maneuver.instruction, key: "0-depart") }
        _ = track
        startPositionSource(from: 0)
    }

    private func startPositionSource(from along: Double) {
        simulator.stop()
        if simulate, let track {
            location.setNavigating(false, walking: mode == .walk)
            simulator.start(track: track, from: along, speed: mode.simSpeed, multiplier: simMultiplier) { [weak self] loc in
                self?.handleFix(loc)
            }
        } else {
            location.setNavigating(true, walking: mode == .walk)
            location.start()
            if let last = location.last { handleFix(last) }
        }
    }

    func simulationSettingsChanged() {
        guard navigating, !nav.arrived else { return }
        startPositionSource(from: nav.along)
    }

    func stopNavigation() {
        guard navigating else { return }
        navigating = false
        simulator.stop()
        location.setNavigating(false, walking: mode == .walk)
        voice.end()
        UIApplication.shared.isIdleTimerDisabled = false
        if !simulate, let f = location.last { fix = f.coordinate }
        fitRoute()
    }

    func recenter() {
        following = true
        if let f = fix { follow(f, course: nav.lastFix?.course ?? -1) }
    }

    private func follow(_ c: CLLocationCoordinate2D, course: Double) {
        withAnimation(.easeInOut(duration: 0.8)) {
            camera = .camera(MapCamera(
                centerCoordinate: c,
                distance: mode == .walk ? 700 : 1300,
                heading: course >= 0 ? course : 0,
                pitch: mode == .walk ? 0 : 45))
        }
    }

    func handleFix(_ loc: CLLocation) {
        let wasMissing = fix == nil
        fix = loc.coordinate
        guard navigating, let track else {
            // First position with a destination set: route from here.
            if wasMissing && startIsMyLocation && end != nil && route == nil { Task { await computeRoute() } }
            return
        }

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

        if nav.remaining < (mode == .walk ? 15 : 25) {
            if !nav.arrived { voice.say("You have arrived at your destination", key: "arrived") }
            nav.arrived = true
            simulator.stop()
            location.setNavigating(false, walking: mode == .walk)
        } else if p.off < mode.offRouteM {
            voice.update(steps: steps, next: nav.nextStep, toNext: nav.toNext, walking: mode == .walk)
        }

        if following { follow(loc.coordinate, course: loc.course) }

        if p.off > mode.offRouteM && loc.horizontalAccuracy < 60 {
            nav.offRouteFixes += 1
        } else {
            nav.offRouteFixes = 0
        }
        if nav.offRouteFixes >= 3 && !rerouting && !nav.arrived {
            rerouting = true
            nav.offRouteFixes = 0
            Task {
                // Re-route from where you are; keep a typed start out of it.
                let keep = startIsMyLocation
                startIsMyLocation = true
                await computeRoute(fit: false)
                if !keep { startText = "My location"; startResolvedText = startText }
                voice.begin()
                voice.say("Rerouting")
                rerouting = false
            }
        }
    }

    // MARK: Formatting

    static func format(_ c: CLLocationCoordinate2D?) -> String {
        guard let c else { return "" }
        return String(format: "%.5f, %.5f", c.latitude, c.longitude)
    }

    static func parse(_ s: String) -> CLLocationCoordinate2D? { Coordinates.parse(s) }

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
