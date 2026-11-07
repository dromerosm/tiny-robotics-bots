// Speech to text (WhisperKit) plus who spoke when (FluidAudio), all on this
// Mac with Core ML. The same steps as tools/transcribe in Python: each word
// goes to the speaker who talks most while it is said, and words in a row
// from one speaker make a turn.
import FluidAudio
import Foundation
import WhisperKit

public struct Word: Codable, Sendable, Hashable {
    public var text: String
    public var start: Double
    public var end: Double
    public var speaker: String
}

public struct Turn: Codable, Sendable, Hashable, Identifiable {
    public var id: Int
    public var speaker: String
    public var start: Double
    public var end: Double
    public var text: String
}

public struct SpeakerSpan: Codable, Sendable, Hashable {
    public var speaker: String
    public var start: Double
    public var end: Double
}

public struct Transcript: Codable, Sendable {
    public var source: String
    public var duration: Double
    public var asrSeconds: Double
    public var diarizationSeconds: Double
    public var speakers: [String]
    public var spans: [SpeakerSpan]
    public var turns: [Turn]
    public var words: [Word]
    /// Names given in the app, by speaker id ("SPEAKER_0" -> "Diego").
    public var names: [String: String] = [:]

    public func name(_ speaker: String) -> String {
        if let n = names[speaker], !n.isEmpty { return n }
        return speaker
    }
}

public enum Stage: Sendable {
    case loadingModels, transcribing, diarizing, done
}

public actor Pipeline {
    public static let whisperModel = "large-v3-v20240930_turbo"
    public static let sampleRate = 16000

    private var whisper: WhisperKit?
    private var diarizer: OfflineDiarizerManager?
    private var diarizerSpeakers: Int?

    public init() {}

    /// 16 kHz mono float samples from any audio file macOS can read.
    public static func load(_ url: URL) throws -> [Float] {
        try AudioConverter().resampleAudioFile(url)
    }

    public func prepare(speakers: Int? = nil) async throws {
        if whisper == nil {
            whisper = try await WhisperKit(WhisperKitConfig(model: Self.whisperModel))
        }
        if diarizer == nil || diarizerSpeakers != speakers {
            var config = OfflineDiarizerConfig()
            config.clustering.numSpeakers = speakers
            let d = OfflineDiarizerManager(config: config)
            try await d.prepareModels()
            diarizer = d
            diarizerSpeakers = speakers
        }
    }

    /// Text only, as one string (for the WER benchmark).
    public func text(_ audio: [Float], language: String = "es") async throws -> String {
        try await prepare()
        let results = try await whisper!.transcribe(audioArray: audio, decodeOptions: options(language, words: false))
        return results.map(\.text).joined(separator: " ")
    }

    /// Who spoke when (for the DER benchmark, and the app).
    public func spans(_ audio: [Float], speakers: Int? = nil) async throws -> [SpeakerSpan] {
        try await prepare(speakers: speakers)
        do {
            let r = try await diarizer!.process(audio: audio)
            return r.segments.map {
                SpeakerSpan(speaker: $0.speakerId, start: Double($0.startTimeSeconds), end: Double($0.endTimeSeconds))
            }
        } catch OfflineDiarizationError.noSpeechDetected {
            return []
        }
    }

    public func run(
        _ audio: [Float], source: String, speakers: Int? = nil, language: String = "es",
        stage: @Sendable (Stage) -> Void = { _ in }
    ) async throws -> Transcript {
        stage(.loadingModels)
        try await prepare(speakers: speakers)
        stage(.transcribing)
        let t0 = Date()
        let results = try await whisper!.transcribe(audioArray: audio, decodeOptions: options(language, words: true))
        let t1 = Date()
        stage(.diarizing)
        let spans = try await self.spans(audio, speakers: speakers)
        let t2 = Date()
        var words: [Word] = results.flatMap(\.segments).flatMap { $0.words ?? [] }.map {
            Word(text: $0.word, start: Double($0.start), end: Double($0.end), speaker: "")
        }
        words = Self.assign(words, spans)
        stage(.done)
        return Transcript(
            source: source, duration: Double(audio.count) / Double(Self.sampleRate),
            asrSeconds: t1.timeIntervalSince(t0), diarizationSeconds: t2.timeIntervalSince(t1),
            speakers: Array(Set(spans.map(\.speaker))).sorted(), spans: spans,
            turns: Self.group(words), words: words)
    }

    private func options(_ language: String, words: Bool) -> DecodingOptions {
        var o = DecodingOptions()
        o.task = .transcribe
        o.language = language
        o.detectLanguage = false
        o.usePrefillPrompt = true
        o.skipSpecialTokens = true
        o.wordTimestamps = words
        o.chunkingStrategy = .vad
        return o
    }

    /// Each word to the speaker with most overlap, or the nearest span.
    public static func assign(_ words: [Word], _ spans: [SpeakerSpan]) -> [Word] {
        words.map { w in
            var w = w
            var best: (String, Double)? = nil
            for s in spans {
                let ov = min(s.end, w.end) - max(s.start, w.start)
                if ov > (best?.1 ?? 0) { best = (s.speaker, ov) }
            }
            if best == nil, !spans.isEmpty {
                let mid = (w.start + w.end) / 2
                let near = spans.min { min(abs($0.start - mid), abs($0.end - mid)) < min(abs($1.start - mid), abs($1.end - mid)) }!
                best = (near.speaker, 0)
            }
            w.speaker = best?.0 ?? "SPEAKER_0"
            return w
        }
    }

    public static func group(_ words: [Word]) -> [Turn] {
        var turns: [Turn] = []
        for w in words {
            if var last = turns.last, last.speaker == w.speaker {
                last.end = w.end
                last.text += w.text
                turns[turns.count - 1] = last
            } else {
                turns.append(Turn(id: turns.count, speaker: w.speaker, start: w.start, end: w.end, text: w.text))
            }
        }
        return turns.map { var t = $0; t.text = t.text.trimmingCharacters(in: .whitespaces); return t }
    }
}

public enum Export {
    /// One line per turn, with the names given in the app.
    public static func markdown(_ t: Transcript, title: String) -> String {
        var out = "# \(title)\n\n"
        out += "\(clock(t.duration)) of audio, \(t.speakers.count) speakers.\n\n"
        for turn in t.turns {
            out += "**\(t.name(turn.speaker))** [\(clock(turn.start))] \(turn.text)\n\n"
        }
        return out
    }

    public static func clock(_ s: Double) -> String {
        let s = Int(s.rounded())
        return s >= 3600
            ? String(format: "%d:%02d:%02d", s / 3600, s / 60 % 60, s % 60)
            : String(format: "%d:%02d", s / 60, s % 60)
    }
}
