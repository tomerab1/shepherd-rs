import CoreLocation
import Foundation

/// Device location via CoreLocation.
@MainActor
@Observable
final class LocationTracker: NSObject, CLLocationManagerDelegate {
    var last: CLLocation?
    var status: CLAuthorizationStatus = .notDetermined
    var error: String?
    var onFix: ((CLLocation) -> Void)?
    private let manager = CLLocationManager()
    private var running = false

    override init() {
        super.init()
        manager.delegate = self
        manager.desiredAccuracy = kCLLocationAccuracyBest
        manager.distanceFilter = 2
        status = manager.authorizationStatus
    }

    var authorized: Bool { status == .authorizedAlways || status == .authorized }

    func start() {
        error = nil
        if status == .notDetermined { manager.requestWhenInUseAuthorization() }
        if !running {
            manager.startUpdatingLocation()
            running = true
        }
    }

    func stop() {
        manager.stopUpdatingLocation()
        running = false
    }

    nonisolated func locationManagerDidChangeAuthorization(_ m: CLLocationManager) {
        let s = m.authorizationStatus
        Task { @MainActor in
            self.status = s
            if s == .denied || s == .restricted {
                self.error = "Location access is off. Enable it for Shepherd in System Settings › Privacy & Security › Location Services."
            }
        }
    }

    nonisolated func locationManager(_ m: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let loc = locations.last else { return }
        Task { @MainActor in
            self.last = loc
            self.onFix?(loc)
        }
    }

    nonisolated func locationManager(_ m: CLLocationManager, didFailWithError e: Error) {
        let msg = (e as? CLError)?.code == .denied ? "Location access denied." : "Location unavailable: \(e.localizedDescription)"
        Task { @MainActor in self.error = msg }
    }
}

/// Geometry of the active route, for projecting positions onto it.
struct RouteTrack {
    let coords: [CLLocationCoordinate2D]
    /// Cumulative distance (m) at each coordinate.
    let cum: [Double]
    /// Route distance at which each step's maneuver happens.
    let stepAt: [Double]
    let total: Double
    /// Route's own duration estimate (s), used before we have a measured pace.
    let duration: Double

    init(coords: [CLLocationCoordinate2D], steps: [Step], duration: Double) {
        self.coords = coords
        var cum = [0.0]
        cum.reserveCapacity(coords.count)
        for i in 1..<max(coords.count, 1) {
            cum.append(cum[i - 1] + RouteTrack.dist(coords[i - 1], coords[i]))
        }
        self.cum = cum
        total = cum.last ?? 0
        var at = 0.0
        var starts: [Double] = []
        for s in steps {
            starts.append(at)
            at += s.distance
        }
        // Scale step positions onto the polyline length (they are computed separately).
        let scale = at > 0 ? total / at : 1
        stepAt = starts.map { $0 * scale }
        self.duration = duration
    }

    static func dist(_ a: CLLocationCoordinate2D, _ b: CLLocationCoordinate2D) -> Double {
        let k = 111_320.0
        let dx = (b.longitude - a.longitude) * k * cos((a.latitude + b.latitude) * .pi / 360)
        let dy = (b.latitude - a.latitude) * k
        return (dx * dx + dy * dy).squareRoot()
    }

    /// Projects `p` onto the route near `hint` (segment index). Returns (along m, off-route m, index).
    func project(_ p: CLLocationCoordinate2D, hint: Int) -> (along: Double, off: Double, index: Int) {
        guard coords.count >= 2 else { return (0, 0, 0) }
        let k = 111_320.0
        let cosLat = cos(p.latitude * .pi / 180)
        func xy(_ c: CLLocationCoordinate2D) -> (Double, Double) { (c.longitude * k * cosLat, c.latitude * k) }
        let q = xy(p)
        func scan(_ r: Range<Int>) -> (Double, Double, Int) {
            var best = (Double.infinity, 0.0, 0)
            for i in r {
                let a = xy(coords[i]), b = xy(coords[i + 1])
                let dx = b.0 - a.0, dy = b.1 - a.1
                let len2 = dx * dx + dy * dy
                let t = len2 > 0 ? max(0, min(1, ((q.0 - a.0) * dx + (q.1 - a.1) * dy) / len2)) : 0
                let px = a.0 + t * dx, py = a.1 + t * dy
                let d = ((q.0 - px) * (q.0 - px) + (q.1 - py) * (q.1 - py)).squareRoot()
                if d < best.0 { best = (d, cum[i] + t * (cum[i + 1] - cum[i]), i) }
            }
            return best
        }
        // Look around the last known position first; fall back to the whole route.
        let lo = max(0, hint - 20), hi = min(coords.count - 1, hint + 400)
        var r = scan(lo..<hi)
        if r.0 > 60 { r = scan(0..<(coords.count - 1)) }
        return (r.1, r.0, r.2)
    }
}

/// Live navigation state shown in the UI.
struct NavState {
    var along: Double = 0
    var offRoute: Double = 0
    var index = 0
    var remaining: Double = 0
    /// Smoothed measured speed (m/s); nil until we have movement.
    var speed: Double?
    var remainingTime: Double = 0
    var eta = Date()
    var nextStep = 0
    var toNext: Double = 0
    var arrived = false
    var offRouteFixes = 0
    var lastFix: CLLocation?
}

/// Feeds fake GPS fixes along the route (to try navigation at a desk).
@MainActor
final class RouteSimulator {
    private var timer: Timer?
    private var along = 0.0

    /// Plays the route from `from` meters at `speed` m/s, `multiplier` times faster than
    /// real time. Fixes report the real `speed`, so pace-based estimates stay realistic.
    func start(track: RouteTrack, from: Double = 0, speed: Double, multiplier: Double,
               onFix: @escaping (CLLocation) -> Void) {
        stop()
        along = min(max(0, from), track.total)
        let tick = 0.5
        let emit = { [weak self] in
            guard let self else { return }
            let c = Self.point(track, at: self.along)
            let ahead = Self.point(track, at: min(track.total, self.along + 5))
            onFix(CLLocation(coordinate: c, altitude: 0, horizontalAccuracy: 5, verticalAccuracy: -1,
                             course: Self.bearing(c, ahead), speed: speed, timestamp: Date()))
        }
        emit() // show the position right away instead of "waiting for location"
        let t = Timer(timeInterval: tick, repeats: true) { [weak self] _ in
            MainActor.assumeIsolated {
                guard let self else { return }
                self.along = min(track.total, self.along + speed * multiplier * tick)
                emit()
                if self.along >= track.total { self.stop() }
            }
        }
        // .common keeps it running while the user scrolls the sidebar or drags the map.
        RunLoop.main.add(t, forMode: .common)
        timer = t
    }

    func stop() {
        timer?.invalidate()
        timer = nil
    }

    var running: Bool { timer != nil }

    static func point(_ t: RouteTrack, at d: Double) -> CLLocationCoordinate2D {
        guard t.coords.count > 1 else { return t.coords.first ?? CLLocationCoordinate2D() }
        var i = t.cum.firstIndex(where: { $0 >= d }) ?? (t.cum.count - 1)
        i = max(1, i)
        let span = t.cum[i] - t.cum[i - 1]
        let f = span > 0 ? (d - t.cum[i - 1]) / span : 0
        let a = t.coords[i - 1], b = t.coords[i]
        return CLLocationCoordinate2D(latitude: a.latitude + (b.latitude - a.latitude) * f,
                                      longitude: a.longitude + (b.longitude - a.longitude) * f)
    }

    static func bearing(_ a: CLLocationCoordinate2D, _ b: CLLocationCoordinate2D) -> Double {
        let dx = (b.longitude - a.longitude) * cos(a.latitude * .pi / 180)
        let dy = b.latitude - a.latitude
        return (atan2(dx, dy) * 180 / .pi + 360).truncatingRemainder(dividingBy: 360)
    }
}
