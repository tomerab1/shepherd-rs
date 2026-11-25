import MapKit
import SwiftUI

@main
struct ShepherdPhoneApp: App {
    @State private var model = PhoneModel()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(model)
                .task { await model.boot() }
        }
    }
}

struct ContentView: View {
    @Environment(PhoneModel.self) private var model
    @State private var showSteps = false
    @Namespace private var mapScope

    var body: some View {
        ZStack {
            PhoneMap(scope: mapScope)
                .ignoresSafeArea()
            VStack(spacing: 8) {
                if model.navigating {
                    InstructionBanner()
                } else {
                    SearchCard()
                }
                if let e = model.errorMessage, !model.navigating {
                    Label(e, systemImage: "exclamationmark.triangle.fill")
                        .font(.footnote)
                        .foregroundStyle(.orange)
                        .padding(.horizontal, 12).padding(.vertical, 8)
                        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
                        .onTapGesture { model.errorMessage = nil }
                }
                Spacer()
                // Map buttons, kept clear of the status bar and the cards.
                VStack(spacing: 8) {
                    MapCompass(scope: mapScope)
                    if !model.navigating { MapUserLocationButton(scope: mapScope) }
                }
                .buttonBorderShape(.circle)
                .frame(maxWidth: .infinity, alignment: .trailing)
                if model.navigating {
                    NavBar(showSteps: $showSteps)
                } else if let r = model.route {
                    RouteCard(route: r, showSteps: $showSteps)
                } else if case .ready = model.status {
                    Text("Search for a place, or long-press the map to go there.")
                        .font(.footnote)
                        .padding(.horizontal, 14).padding(.vertical, 8)
                        .background(.regularMaterial, in: Capsule())
                }
            }
            .padding(.horizontal, 10)
            .padding(.bottom, 4)
            if case .loading = model.status {
                ProgressView("Loading maps…")
                    .padding(20)
                    .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 14))
            }
            if case .failed(let msg) = model.status {
                ContentUnavailableView("Engine failed to start", systemImage: "exclamationmark.triangle", description: Text(msg))
                    .background(.regularMaterial)
            }
        }
        .mapScope(mapScope)
        .sheet(isPresented: $showSteps) {
            StepsSheet().presentationDetents([.medium, .large])
        }
    }
}

// MARK: - Search

struct SearchCard: View {
    @Environment(PhoneModel.self) private var model
    @FocusState private var focus: Field?
    enum Field { case start, end }

    var body: some View {
        @Bindable var model = model
        VStack(spacing: 8) {
            HStack(spacing: 8) {
                VStack(spacing: 6) {
                    field("From", symbol: model.startIsMyLocation ? "location.fill" : "circle.circle.fill",
                          tint: model.startIsMyLocation ? .blue : .green, text: $model.startText, search: model.startSearch, which: .start)
                    field("To", symbol: "mappin.circle.fill", tint: .red, text: $model.endText, search: model.endSearch, which: .end)
                }
                Button {
                    Task { await model.swap() }
                } label: {
                    Image(systemName: "arrow.up.arrow.down").font(.body.weight(.semibold))
                }
                .disabled(model.end == nil)
            }
            if let f = focus {
                let search = f == .start ? model.startSearch : model.endSearch
                VStack(alignment: .leading, spacing: 0) {
                    if f == .start && !model.startIsMyLocation {
                        suggestionRow(title: "My location", subtitle: "", symbol: "location.fill") {
                            focus = nil
                            model.useMyLocationAsStart()
                        }
                    }
                    ForEach(search.suggestions, id: \.self) { s in
                        suggestionRow(title: s.title, subtitle: s.subtitle, symbol: "mappin") {
                            focus = nil
                            search.clear()
                            Task { await model.pick(s, forStart: f == .start) }
                        }
                    }
                }
            }
            Picker("Mode", selection: $model.mode) {
                ForEach(TravelMode.allCases) { m in Label(m.rawValue, systemImage: m.symbol).tag(m) }
            }
            .pickerStyle(.segmented)
        }
        .padding(10)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    private func field(_ label: String, symbol: String, tint: Color, text: Binding<String>, search: PlaceSearch, which: Field) -> some View {
        HStack(spacing: 8) {
            Image(systemName: symbol).foregroundStyle(tint).frame(width: 20)
            TextField(label, text: text)
                .focused($focus, equals: which)
                .submitLabel(.route)
                .autocorrectionDisabled()
                .onChange(of: text.wrappedValue) { if focus == which { search.update(text.wrappedValue) } }
                .onSubmit {
                    search.clear()
                    Task { await model.commitText(start: which == .start) }
                }
            if focus == which && !text.wrappedValue.isEmpty {
                Button { text.wrappedValue = "" } label: {
                    Image(systemName: "xmark.circle.fill").foregroundStyle(.tertiary)
                }
            }
        }
        .padding(.horizontal, 10).padding(.vertical, 9)
        .background(Color(.systemBackground).opacity(0.7), in: RoundedRectangle(cornerRadius: 10))
    }

    private func suggestionRow(title: String, subtitle: String, symbol: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            HStack(spacing: 10) {
                Image(systemName: symbol).foregroundStyle(.secondary).frame(width: 20)
                VStack(alignment: .leading, spacing: 1) {
                    Text(title).lineLimit(1)
                    if !subtitle.isEmpty {
                        Text(subtitle).font(.caption).foregroundStyle(.secondary).lineLimit(1)
                    }
                }
                Spacer()
            }
            .padding(.vertical, 8).padding(.horizontal, 6)
            .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
    }
}

// MARK: - Route summary

struct RouteCard: View {
    let route: Route
    @Binding var showSteps: Bool
    @Environment(PhoneModel.self) private var model

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(alignment: .firstTextBaseline) {
                Text(PhoneModel.duration(route.duration))
                    .font(.system(size: 30, weight: .bold, design: .rounded))
                Text(PhoneModel.distance(route.distance))
                    .font(.title3).foregroundStyle(.secondary)
                Spacer()
                OptionsMenu()
            }
            if let s = route.legs.first?.summary, !s.isEmpty {
                Text("via \(s)").font(.subheadline).foregroundStyle(.secondary).lineLimit(1)
            }
            HStack(spacing: 14) {
                let n = route.traffic_lights?.count ?? 0
                Label("\(n) traffic light\(n == 1 ? "" : "s")", systemImage: "light.beacon.max")
                Label("\(Int(model.computeMs)) ms on-device", systemImage: "cpu")
            }
            .font(.caption).foregroundStyle(.secondary)
            HStack(spacing: 10) {
                Button { showSteps = true } label: {
                    Label("Steps", systemImage: "list.bullet").frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
                Button { model.startNavigation() } label: {
                    Label(model.simulate ? "Simulate" : "Go", systemImage: "location.north.line.fill")
                        .font(.headline).frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                Button { model.clear() } label: { Image(systemName: "xmark") }
                    .buttonStyle(.bordered)
            }
            .controlSize(.large)
        }
        .padding(14)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 18))
    }
}

struct OptionsMenu: View {
    @Environment(PhoneModel.self) private var model

    var body: some View {
        @Bindable var model = model
        Menu {
            Toggle("Avoid Judea & Samaria", isOn: $model.avoidJudeaSamaria)
            Picker("Traffic lights", selection: $model.signalMode) {
                Text("Estimate waits").tag("estimate")
                Text("Only with timing data").tag("timed-only")
                Text("Ignore lights").tag("off")
            }
            Toggle("Show traffic lights on map", isOn: $model.showAllLights)
            Divider()
            Toggle("Simulate (move along the route)", isOn: $model.simulate)
            if model.simulate {
                Picker("Simulation speed", selection: $model.simMultiplier) {
                    Text("1×").tag(1.0)
                    Text("5×").tag(5.0)
                    Text("20×").tag(20.0)
                }
            }
        } label: {
            Image(systemName: "slider.horizontal.3").font(.title3).padding(6)
        }
        .onChange(of: model.signalMode) { Task { await model.applySignalMode() } }
        .onChange(of: model.showAllLights) { if !model.showAllLights { model.viewLights = [] } }
        .onChange(of: model.simulate) { model.simulationSettingsChanged() }
        .onChange(of: model.simMultiplier) { model.simulationSettingsChanged() }
    }
}

// MARK: - Navigation

struct InstructionBanner: View {
    @Environment(PhoneModel.self) private var model

    var body: some View {
        let nav = model.nav
        let steps = model.steps
        HStack(spacing: 14) {
            if nav.arrived {
                Image(systemName: "flag.checkered").font(.system(size: 34, weight: .semibold))
                Text("You have arrived").font(.title2.weight(.bold))
            } else if nav.lastFix == nil {
                ProgressView().tint(.white)
                Text("Waiting for your location…").font(.headline)
            } else if nav.nextStep < steps.count {
                let step = steps[nav.nextStep]
                Image(systemName: stepIcon(step.maneuver))
                    .font(.system(size: 38, weight: .bold))
                    .frame(width: 50)
                VStack(alignment: .leading, spacing: 2) {
                    Text(PhoneModel.distance(nav.toNext))
                        .font(.system(size: 30, weight: .bold, design: .rounded))
                        .monospacedDigit()
                    Text(step.maneuver.instruction)
                        .font(.headline)
                        .lineLimit(2)
                }
            }
            Spacer(minLength: 0)
        }
        .foregroundStyle(.white)
        .padding(16)
        .background(nav.offRoute > model.mode.offRouteM ? Color.orange.gradient : Color.green.gradient,
                    in: RoundedRectangle(cornerRadius: 18))
        .shadow(radius: 6, y: 3)
    }
}

struct NavBar: View {
    @Binding var showSteps: Bool
    @Environment(PhoneModel.self) private var model

    var body: some View {
        @Bindable var model = model
        let nav = model.nav
        VStack(spacing: 10) {
            if !model.following {
                Button { model.recenter() } label: {
                    Label("Re-center", systemImage: "location.fill").font(.subheadline.weight(.semibold))
                }
                .buttonStyle(.borderedProminent)
                .frame(maxWidth: .infinity, alignment: .trailing)
            }
            HStack(alignment: .center) {
                Button { model.voice.muted.toggle() } label: {
                    Image(systemName: model.voice.muted ? "speaker.slash.fill" : "speaker.wave.2.fill")
                        .font(.title3).frame(width: 44, height: 44)
                }
                Spacer()
                VStack(spacing: 2) {
                    HStack(alignment: .firstTextBaseline, spacing: 6) {
                        Text(nav.eta, style: .time)
                            .font(.system(size: 26, weight: .bold, design: .rounded))
                        Text("arrival").font(.caption).foregroundStyle(.secondary)
                    }
                    Text("\(PhoneModel.duration(nav.remainingTime)) · \(PhoneModel.distance(nav.remaining)) · \(PhoneModel.speed(nav.speed))")
                        .font(.subheadline)
                        .foregroundStyle(.secondary)
                        .monospacedDigit()
                    if model.simulate {
                        Text("Simulating at \(Int(model.simMultiplier))×").font(.caption2).foregroundStyle(.orange)
                    } else if model.mode == .walk && nav.speed != nil {
                        Text("time left at your pace").font(.caption2).foregroundStyle(.tertiary)
                    }
                }
                Spacer()
                Menu {
                    Button { showSteps = true } label: { Label("Steps", systemImage: "list.bullet") }
                    Toggle("Simulate", isOn: $model.simulate)
                    Button(role: .destructive) { model.stopNavigation() } label: {
                        Label("End navigation", systemImage: "xmark.circle")
                    }
                } label: {
                    Image(systemName: "ellipsis.circle").font(.title3).frame(width: 44, height: 44)
                }
                .onChange(of: model.simulate) { model.simulationSettingsChanged() }
            }
            Button(role: .destructive) { model.stopNavigation() } label: {
                Text(nav.arrived ? "Done" : "End").font(.headline).frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .tint(nav.arrived ? .green : .red)
            .controlSize(.large)
        }
        .padding(14)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 18))
    }
}

struct StepsSheet: View {
    @Environment(PhoneModel.self) private var model

    var body: some View {
        NavigationStack {
            List(Array(model.steps.enumerated()), id: \.element.id) { i, step in
                HStack(spacing: 12) {
                    Image(systemName: stepIcon(step.maneuver))
                        .font(.title3).foregroundStyle(.tint).frame(width: 28)
                    VStack(alignment: .leading, spacing: 2) {
                        Text(step.maneuver.instruction)
                        if step.distance > 0 {
                            Text(PhoneModel.distance(step.distance)).font(.caption).foregroundStyle(.secondary)
                        }
                    }
                }
                .opacity(model.navigating && i < model.nav.nextStep ? 0.4 : 1)
            }
            .listStyle(.plain)
            .navigationTitle("Steps")
            .navigationBarTitleDisplayMode(.inline)
        }
    }
}

func stepIcon(_ m: Maneuver) -> String {
    switch m.type {
    case "depart": return "location.north.fill"
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

struct PhoneMap: View {
    let scope: Namespace.ID
    @Environment(PhoneModel.self) private var model

    var body: some View {
        @Bindable var model = model
        MapReader { proxy in
            Map(position: $model.camera, scope: scope) {
                if !model.routeCoords.isEmpty {
                    MapPolyline(coordinates: model.routeCoords)
                        .stroke(.blue.opacity(0.35), style: StrokeStyle(lineWidth: 12, lineCap: .round, lineJoin: .round))
                    MapPolyline(coordinates: model.routeCoords)
                        .stroke(.blue, style: StrokeStyle(lineWidth: 6, lineCap: .round, lineJoin: .round))
                }
                ForEach(Array(model.viewLights.enumerated()), id: \.offset) { _, c in
                    Annotation("", coordinate: c, anchor: .center) {
                        TrafficLightIcon().scaleEffect(0.75).opacity(0.45)
                    }
                }
                ForEach(Array((model.route?.lights ?? []).enumerated()), id: \.offset) { _, c in
                    Annotation("", coordinate: c, anchor: .center) { TrafficLightIcon() }
                }
                if let s = model.start, !model.startIsMyLocation {
                    Marker("Start", systemImage: "circle.circle.fill", coordinate: s).tint(.green)
                }
                if let e = model.end {
                    Marker(model.endText.isEmpty ? "Destination" : model.endText, systemImage: "flag.checkered", coordinate: e).tint(.red)
                }
                if model.simulate && model.navigating, let f = model.fix {
                    Annotation("", coordinate: f, anchor: .center) {
                        ZStack {
                            Circle().fill(.blue.opacity(0.2)).frame(width: 34, height: 34)
                            Circle().fill(.white).frame(width: 18, height: 18)
                            Circle().fill(.blue).frame(width: 12, height: 12)
                        }
                    }
                } else {
                    UserAnnotation()
                }
            }
            .mapStyle(.standard(elevation: .realistic, pointsOfInterest: .excludingAll))
            .mapControls { MapScaleView() }
            .onMapCameraChange(frequency: .onEnd) { ctx in model.mapRegionChanged(ctx.region) }
            .simultaneousGesture(DragGesture(minimumDistance: 12).onChanged { _ in
                if model.navigating { model.following = false }
            })
            .simultaneousGesture(
                LongPressGesture(minimumDuration: 0.45)
                    .sequenced(before: DragGesture(minimumDistance: 0, coordinateSpace: .local))
                    .onEnded { value in
                        if case .second(true, let drag?) = value, let c = proxy.convert(drag.location, from: .local) {
                            UIImpactFeedbackGenerator(style: .medium).impactOccurred()
                            Task { await model.mapPressed(c) }
                        }
                    })
        }
    }
}
