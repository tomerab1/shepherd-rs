import AVFoundation

/// Spoken turn-by-turn prompts. Each maneuver gets an early heads-up and a "now" prompt;
/// each prompt is spoken once.
@MainActor
final class VoiceGuide: NSObject, AVSpeechSynthesizerDelegate {
    var muted = false {
        didSet { if muted { synth.stopSpeaking(at: .immediate) } }
    }
    private let synth = AVSpeechSynthesizer()
    private var spoken: Set<String> = []
    private let voice = AVSpeechSynthesisVoice(language: "en-US")

    override init() {
        super.init()
        synth.delegate = self
    }

    func begin() {
        spoken = []
        try? AVAudioSession.sharedInstance().setCategory(
            .playback, mode: .voicePrompt, options: [.duckOthers, .interruptSpokenAudioAndMixWithOthers])
    }

    func end() {
        synth.stopSpeaking(at: .immediate)
        spoken = []
        try? AVAudioSession.sharedInstance().setActive(false, options: .notifyOthersOnDeactivation)
    }

    /// Speaks `text`; with a `key`, only the first time.
    func say(_ text: String, key: String? = nil) {
        if let key, !spoken.insert(key).inserted { return }
        guard !muted else { return }
        try? AVAudioSession.sharedInstance().setActive(true)
        let u = AVSpeechUtterance(string: text)
        u.voice = voice
        u.rate = AVSpeechUtteranceDefaultSpeechRate * 1.05
        synth.speak(u)
    }

    /// Called on every position update with the upcoming step and the distance to it.
    func update(steps: [Step], next: Int, toNext: Double, walking: Bool) {
        guard next < steps.count, next > 0 else { return }
        let step = steps[next]
        // Distances (m) at which to announce: heads-up stages, then "now".
        let stages: [Double] = walking ? [120] : [1200, 400]
        let now: Double = walking ? 18 : 70
        if toNext <= now {
            var text = step.maneuver.instruction
            // A second maneuver right after this one: say it too.
            if next + 1 < steps.count, step.distance < (walking ? 40 : 150) {
                text += ", then " + Self.lowerFirst(steps[next + 1].maneuver.instruction)
            }
            say(text, key: "\(next)-now")
            return
        }
        // The nearest stage we're inside; skip stages that would come too close to "now".
        guard let stage = stages.last(where: { toNext <= $0 }), toNext > now * 2 else { return }
        if step.maneuver.type == "arrive" {
            say("In \(Self.spoken(toNext)), you will arrive at your destination", key: "\(next)-\(stage)")
        } else {
            say("In \(Self.spoken(toNext)), \(Self.lowerFirst(step.maneuver.instruction))", key: "\(next)-\(stage)")
        }
    }

    static func spoken(_ m: Double) -> String {
        if m >= 950 {
            let km = (m / 500).rounded() / 2
            return km == 1 ? "1 kilometer" : "\(km.formatted(.number.precision(.fractionLength(0...1)))) kilometers"
        }
        let r = m >= 200 ? (m / 100).rounded() * 100 : (m / 10).rounded() * 10
        return "\(Int(r)) meters"
    }

    static func lowerFirst(_ s: String) -> String {
        guard let f = s.first else { return s }
        return f.lowercased() + s.dropFirst()
    }

    nonisolated func speechSynthesizer(_ s: AVSpeechSynthesizer, didFinish u: AVSpeechUtterance) {
        Task { @MainActor in
            if !self.synth.isSpeaking {
                try? AVAudioSession.sharedInstance().setActive(false, options: .notifyOthersOnDeactivation)
            }
        }
    }
}
