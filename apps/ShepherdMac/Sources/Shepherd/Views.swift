import MapKit
import SwiftUI

@main
struct ShepherdApp: App {
    @NSApplicationDelegateAdaptor(AppDelegate.self) var delegate
    @State private var model = AppModel()

    var body: some Scene {
        WindowGroup("Shepherd") {
            ContentView()
                .environment(model)
                .frame(minWidth: 960, minHeight: 620)
                .task { await model.boot() }
        }
        .commands {
            CommandMenu("Route") {
                Button("Route") { Task { await model.commitText() } }
                    .keyboardShortcut("r")
                Button("Swap Start and Destination") { Task { await model.swap() } }
                    .keyboardShortcut("s", modifiers: [.command, .shift])
                Button("Clear") { model.clear() }
                    .keyboardShortcut("k")
            }
        }
    }
}

final class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
    func applicationWillTerminate(_ notification: Notification) {
        MainActor.assumeIsolated { EngineProcess.shared.stop() }
    }
}

struct ContentView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        NavigationSplitView {
            Sidebar()
                .navigationSplitViewColumnWidth(min: 360, ideal: 400, max: 480)
        } detail: {
            MapPane()
        }
    }
}

// MARK: - Sidebar

struct Sidebar: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        // One scroll view for controls and steps, so expanding sections never pushes
        // content out of the window.
        ScrollView {
        VStack(alignment: .leading, spacing: 0) {
            VStack(alignment: .leading, spacing: 12) {
                StatusRow()
                HStack {
                    Picker("Mode", selection: $model.mode) {
                        ForEach(TravelMode.allCases) { m in
                            Label(m.rawValue, systemImage: m.symbol).tag(m)
                        }
                    }
                    .labelsHidden()
                    .pickerStyle(.segmented)
                    .frame(width: 170)
                    .onChange(of: model.mode) { Task { await model.computeRoute() } }
                    Spacer()
                    Toggle("Avoid Judea & Samaria", isOn: $model.avoidJudeaSamaria)
                        .toggleStyle(.checkbox)
                        .font(.callout)
                        .onChange(of: model.avoidJudeaSamaria) { Task { await model.computeRoute() } }
                }
                HStack {
                    PlaceField(label: "Start", symbol: "circle.circle.fill", tint: .green,
                               text: $model.startText, search: model.startSearch, isStart: true)
                    Button {
                        model.useMyLocation()
                    } label: {
                        if model.waitingForLocation {
                            ProgressView().controlSize(.small)
                        } else {
                            Image(systemName: "location.fill")
                        }
                    }
                    .help("Start from my current location")
                }
                PlaceField(label: "Destination", symbol: "mappin.circle.fill", tint: .red,
                           text: $model.endText, search: model.endSearch, isStart: false)
                if let err = model.location.error {
                    Label(err, systemImage: "location.slash").font(.caption).foregroundStyle(.orange)
                }
                HStack {
                    Button {
                        Task { await model.commitText() }
                    } label: {
                        Label("Route", systemImage: "arrow.triangle.turn.up.right.diamond.fill")
                    }
                    .buttonStyle(.borderedProminent)
                    .keyboardShortcut(.return, modifiers: [])
                    Button { Task { await model.swap() } } label: { Image(systemName: "arrow.up.arrow.down") }
                        .help("Swap start and destination")
                    Button { model.clear() } label: { Image(systemName: "trash") }
                        .help("Clear")
                    Spacer()
                    if model.isRouting { ProgressView().controlSize(.small) }
                }
                VStack(alignment: .leading, spacing: 6) {
                    Text("Clicking the map sets").font(.caption).foregroundStyle(.secondary)
                    Picker("Clicking the map sets", selection: $model.clickMode) {
                        ForEach(ClickMode.allCases) { m in
                            Label(m.rawValue, systemImage: m.symbol).tag(m)
                        }
                    }
                    .labelsHidden()
                    .pickerStyle(.segmented)
                }
                TrafficSettings()
                if !model.incidents.isEmpty {
                    HStack {
                        Image(systemName: "exclamationmark.triangle.fill").foregroundStyle(.orange)
                        Text("\(model.incidents.count) live incident\(model.incidents.count == 1 ? "" : "s")")
                        if let ms = model.lastUpdateMs {
                            Text("· weights updated in \(Int(ms)) ms").foregroundStyle(.secondary)
                        }
                        Spacer()
                        Button("Clear") { Task { await model.clearIncidents() } }
                    }
                    .font(.callout)
                }
                if let err = model.errorMessage {
                    Label(err, systemImage: "exclamationmark.circle.fill")
                        .foregroundStyle(.red)
                        .font(.callout)
                }
                if model.navigating {
                    NavigationCard()
                } else if let r = model.route {
                    Summary(route: r)
                    HStack {
                        Button {
                            model.startNavigation()
                        } label: {
                            Label("Start navigation", systemImage: "location.north.line.fill")
                        }
                        .buttonStyle(.borderedProminent)
                        .tint(.green)
                        Toggle("Simulate", isOn: $model.simulate)
                            .toggleStyle(.checkbox)
                            .help("Move along the route instead of using your real location")
                    }
                }
            }
            .padding()
            Divider()
            StepList()
        }
        }
        // Keep scrolled content below the window's title-bar buttons: a view only extends into
        // the title-bar safe area when it touches it, so a 1 pt inset plus clipping stops that.
        .padding(.top, 1)
        .clipped()
    }
}

struct StatusRow: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        HStack(spacing: 6) {
            switch model.status {
            case .starting:
                ProgressView().controlSize(.small)
                Text("Starting routing engine…")
            case .ready(let h):
                Circle().fill(.green).frame(width: 8, height: 8)
                Text("Engine ready").fontWeight(.medium)
                Text("· \(h.graph.num_segments.formatted()) road segments")
                    .foregroundStyle(.secondary)
            case .failed(let msg):
                Circle().fill(.red).frame(width: 8, height: 8)
                Text(msg).lineLimit(2)
                Button("Retry") { Task { await model.boot() } }
            }
        }
        .font(.callout)
    }
}

/// A start/destination field: type an address or place (with suggestions), or "lat, lon".
struct PlaceField: View {
    let label: String
    let symbol: String
    let tint: Color
    @Binding var text: String
    let search: PlaceSearch
    let isStart: Bool
    @Environment(AppModel.self) private var model
    @FocusState private var focused: Bool

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Image(systemName: symbol).foregroundStyle(tint).frame(width: 18)
                TextField("\(label): address, place or lat, lon", text: $text)
                    .textFieldStyle(.roundedBorder)
                    .focused($focused)
                    .onChange(of: text) { if focused { search.update(text) } }
                    .onSubmit {
                        search.clear()
                        Task { await model.commitText() }
                    }
            }
            if focused && !search.suggestions.isEmpty {
                VStack(alignment: .leading, spacing: 0) {
                    ForEach(search.suggestions, id: \.self) { s in
                        Button {
                            focused = false
                            search.clear()
                            Task { await model.pick(s, forStart: isStart) }
                        } label: {
                            VStack(alignment: .leading, spacing: 1) {
                                Text(s.title).lineLimit(1)
                                if !s.subtitle.isEmpty {
                                    Text(s.subtitle).font(.caption).foregroundStyle(.secondary).lineLimit(1)
                                }
                            }
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .padding(.vertical, 4)
                            .padding(.horizontal, 8)
                            .contentShape(Rectangle())
                        }
                        .buttonStyle(.plain)
                        Divider()
                    }
                }
                .background(.background, in: RoundedRectangle(cornerRadius: 6))
                .overlay(RoundedRectangle(cornerRadius: 6).stroke(.separator))
                .padding(.leading, 26)
            }
        }
    }
}

/// Traffic lights and live traffic settings.
struct TrafficSettings: View {
    @Environment(AppModel.self) private var model
    @State private var expanded = false

    var body: some View {
        @Bindable var model = model
        DisclosureGroup(isExpanded: $expanded) {
            VStack(alignment: .leading, spacing: 8) {
                Picker("Traffic lights", selection: $model.signalMode) {
                    Text("Estimated").tag("estimate")
                    Text("Only with timing data").tag("timed-only")
                    Text("Off").tag("off")
                }
                .onChange(of: model.signalMode) { Task { await model.applySignalMode() } }
                Text("“Only with timing data” ignores lights that have no real timing (uploaded via the admin API).")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Toggle("Show all traffic lights when zoomed in", isOn: $model.showAllLights)
                    .toggleStyle(.checkbox)
                    .onChange(of: model.showAllLights) { if !model.showAllLights { model.viewLights = [] } }
                Divider()
                HStack {
                    SecureField("HERE API key", text: $model.hereKey)
                        .textFieldStyle(.roundedBorder)
                    Button("Save") { Task { await model.saveHereKey() } }
                }
                feedLine.font(.caption)
            }
            .padding(.top, 4)
        } label: {
            HStack {
                Label("Traffic", systemImage: "car.rear.road.lane")
                Spacer()
                if let f = model.feed, f.enabled {
                    Circle().fill(f.ok ? .green : .orange).frame(width: 7, height: 7)
                    Text("live").font(.caption).foregroundStyle(.secondary)
                }
            }
        }
        .task { await model.refreshFeed() }
    }

    @ViewBuilder private var feedLine: some View {
        if let f = model.feed, f.enabled {
            if let e = f.error {
                Label(e, systemImage: "exclamationmark.triangle").foregroundStyle(.orange).lineLimit(3)
            } else if f.polls > 0 {
                let when = Date(timeIntervalSince1970: f.last_poll_unix)
                Text("Live traffic on · \(f.slowed_segments) slowed, \(f.closed_segments) closed · updated \(when.formatted(date: .omitted, time: .shortened)) · every 15 min")
                    .foregroundStyle(.secondary)
            } else {
                Text("Live traffic starting…").foregroundStyle(.secondary)
            }
        } else {
            Text("Live traffic off. Get a free key at developer.here.com (Traffic API v7), paste it here and Save.")
                .foregroundStyle(.secondary)
        }
    }
}

struct Summary: View {
    let route: Route
    @Environment(AppModel.self) private var model

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack(alignment: .firstTextBaseline) {
                Text(AppModel.duration(route.duration))
                    .font(.system(size: 28, weight: .semibold, design: .rounded))
                Text(AppModel.distance(route.distance))
                    .font(.title3)
                    .foregroundStyle(.secondary)
            }
            if let summary = route.legs.first?.summary, !summary.isEmpty {
                Text("via \(summary)").foregroundStyle(.secondary)
            }
            let n = route.traffic_lights?.count ?? 0
            Label("\(n) traffic light\(n == 1 ? "" : "s") on the way (waits included)", systemImage: "light.beacon.max")
                .font(.callout)
                .foregroundStyle(.secondary)
            Text("Computed in \(Int(model.computeMs)) ms · traffic state v\(model.routedMetricVersion)")
                .font(.caption)
                .foregroundStyle(.tertiary)
        }
    }
}

struct NavigationCard: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        let nav = model.nav
        let steps = model.steps
        VStack(alignment: .leading, spacing: 10) {
            if nav.arrived {
                Label("You have arrived", systemImage: "flag.checkered")
                    .font(.title2.weight(.semibold))
            } else if nav.lastFix == nil {
                Label("Waiting for your location…", systemImage: "location.circle")
                    .font(.headline)
            } else if nav.nextStep < steps.count {
                let step = steps[nav.nextStep]
                HStack(alignment: .top, spacing: 12) {
                    Image(systemName: StepList.icon(for: step.maneuver))
                        .font(.system(size: 30, weight: .semibold))
                        .foregroundStyle(.tint)
                        .frame(width: 40)
                    VStack(alignment: .leading, spacing: 2) {
                        Text(AppModel.distance(nav.toNext))
                            .font(.system(size: 26, weight: .bold, design: .rounded))
                        Text(step.maneuver.instruction).font(.headline)
                    }
                }
            }
            Grid(alignment: .leading, horizontalSpacing: 16, verticalSpacing: 4) {
                GridRow {
                    Text("Remaining").foregroundStyle(.secondary)
                    Text(AppModel.distance(nav.remaining)).monospacedDigit()
                }
                GridRow {
                    Text(model.mode == .walk && nav.speed != nil ? "Time left at your pace" : "Time left")
                        .foregroundStyle(.secondary)
                    Text(AppModel.duration(nav.remainingTime)).monospacedDigit()
                }
                GridRow {
                    Text("Arrival").foregroundStyle(.secondary)
                    Text(nav.eta, style: .time).monospacedDigit()
                }
                GridRow {
                    Text("Your speed").foregroundStyle(.secondary)
                    Text(AppModel.speed(nav.speed)).monospacedDigit()
                }
                if nav.offRoute > model.mode.offRouteM {
                    GridRow {
                        Text("Off route").foregroundStyle(.orange)
                        Text("\(Int(nav.offRoute)) m – re-routing…").foregroundStyle(.orange)
                    }
                }
            }
            .font(.callout)
            Button(role: .destructive) {
                model.stopNavigation()
            } label: {
                Label("End navigation", systemImage: "xmark.circle.fill")
            }
        }
        .padding(12)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 10))
    }
}

struct StepList: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        if model.steps.isEmpty {
            ContentUnavailableView(
                "No route yet",
                systemImage: "map",
                description: Text("Type an address, a place or coordinates, or click the map, to set a start and a destination."))
                .padding(.top, 30)
        }
        LazyVStack(alignment: .leading, spacing: 0) {
          ForEach(model.steps) { step in
            HStack(alignment: .top, spacing: 10) {
                Image(systemName: Self.icon(for: step.maneuver))
                    .font(.title3)
                    .frame(width: 26)
                    .foregroundStyle(.tint)
                VStack(alignment: .leading, spacing: 2) {
                    Text(step.maneuver.instruction)
                    if step.distance > 0 {
                        Text(AppModel.distance(step.distance))
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(.vertical, 7)
            .padding(.horizontal, 14)
            .background(model.selectedStep == step.id ? Color.accentColor.opacity(0.18) : .clear)
            .contentShape(Rectangle())
            .onTapGesture { model.focus(step) }
            Divider().padding(.leading, 50)
          }
        }
    }

    static func icon(for m: Maneuver) -> String {
        switch m.type {
        case "depart": return "car.fill"
        case "arrive": return "flag.checkered"
        case "roundabout": return "arrow.triangle.2.circlepath"
        case "merge": return "arrow.merge"
        case "on ramp", "off ramp": return m.modifier?.contains("left") == true ? "arrow.up.left" : "arrow.up.right"
        case "fork": return "arrow.branch"
        default: break
        }
        switch m.modifier ?? "straight" {
        case "left", "sharp left": return "arrow.turn.up.left"
        case "right", "sharp right": return "arrow.turn.up.right"
        case "slight left": return "arrow.up.left"
        case "slight right": return "arrow.up.right"
        case "uturn": return "arrow.uturn.down"
        default: return "arrow.up"
        }
    }
}

// MARK: - Map

struct TrafficLightIcon: View {
    var body: some View {
        VStack(spacing: 1.5) {
            Circle().fill(.red).frame(width: 3.5, height: 3.5)
            Circle().fill(.yellow).frame(width: 3.5, height: 3.5)
            Circle().fill(.green).frame(width: 3.5, height: 3.5)
        }
        .padding(.horizontal, 2.5)
        .padding(.vertical, 3)
        .background(Color.black.opacity(0.85), in: RoundedRectangle(cornerRadius: 3))
        .overlay(RoundedRectangle(cornerRadius: 3).stroke(.white.opacity(0.8), lineWidth: 0.75))
    }
}

struct MapPane: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        MapReader { proxy in
            Map(position: $model.camera) {
                if !model.routeCoords.isEmpty {
                    MapPolyline(coordinates: model.routeCoords)
                        .stroke(.blue.opacity(0.35), style: StrokeStyle(lineWidth: 11, lineCap: .round, lineJoin: .round))
                    MapPolyline(coordinates: model.routeCoords)
                        .stroke(.blue, style: StrokeStyle(lineWidth: 5, lineCap: .round, lineJoin: .round))
                }
                ForEach(Array(model.viewLights.enumerated()), id: \.offset) { _, c in
                    Annotation("", coordinate: c, anchor: .center) {
                        TrafficLightIcon().scaleEffect(0.75).opacity(0.45)
                    }
                }
                ForEach(Array((model.route?.lights ?? []).enumerated()), id: \.offset) { _, c in
                    Annotation("", coordinate: c, anchor: .center) {
                        TrafficLightIcon()
                    }
                }
                if let s = model.start {
                    Marker("Start", systemImage: "circle.circle.fill", coordinate: s).tint(.green)
                }
                if let e = model.end {
                    Marker("Destination", systemImage: "flag.checkered", coordinate: e).tint(.red)
                }
                ForEach(model.incidents) { i in
                    Annotation(i.closed ? "Closed" : "Jam", coordinate: i.coordinate) {
                        Image(systemName: i.closed ? "xmark.octagon.fill" : "car.2.fill")
                            .font(.title2)
                            .foregroundStyle(.white, i.closed ? .red : .orange)
                            .shadow(radius: 2)
                    }
                }
                if let f = model.fix {
                    Annotation("You", coordinate: f) {
                        ZStack {
                            Circle().fill(.blue.opacity(0.2)).frame(width: 34, height: 34)
                            Circle().fill(.white).frame(width: 18, height: 18)
                            Circle().fill(.blue).frame(width: 12, height: 12)
                        }
                    }
                }
                if let id = model.selectedStep, let step = model.steps.first(where: { $0.id == id }) {
                    Annotation("", coordinate: step.maneuver.coordinate) {
                        Circle().fill(.white).stroke(.blue, lineWidth: 3).frame(width: 14, height: 14)
                    }
                }
            }
            .mapStyle(.standard(elevation: .flat, pointsOfInterest: .excludingAll))
            .onMapCameraChange(frequency: .onEnd) { ctx in model.mapRegionChanged(ctx.region) }
            .mapControls {
                MapZoomStepper()
                MapCompass()
                MapScaleView()
            }
            .onTapGesture { point in
                if let c = proxy.convert(point, from: .local) {
                    Task { await model.mapClicked(c) }
                }
            }
            .overlay(alignment: .top) {
                Label(hint, systemImage: model.clickMode.symbol)
                    .font(.callout)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(.regularMaterial, in: Capsule())
                    .padding(.top, 10)
            }
        }
    }

    private var hint: String {
        switch model.clickMode {
        case .start: "Click the map to set the start"
        case .end: "Click the map to set the destination"
        case .close: "Click a road to close it (construction)"
        case .slow: "Click a road to add a traffic jam"
        }
    }
}
