// The Swift engines on the same files as tools/transcribe:
//
//   cd ../transcribe && uv run bench.py export     # once: audio and labels
//   swift run -c release scribe-bench ../transcribe/.work/export
//   cd ../transcribe && uv run bench.py score swift
//
// It writes fleurs/hyp-swift.tsv (WhisperKit's text per clip),
// callhome/hyp-swift/<call>.auto.rttm and .oracle.rttm (FluidAudio, with the
// speaker count found by the model, and told the true count), and
// timing-swift.json. bench.py scores them with the same code as Python.
import Foundation
import ScribeKit

let args = CommandLine.arguments
guard args.count >= 2 else {
    print("usage: scribe-bench <export dir> [wer|der]")
    exit(2)
}
let dir = URL(fileURLWithPath: args[1])
let only = args.count > 2 ? args[2] : nil
let fm = FileManager.default
let pipeline = Pipeline()
var timing: [String: Double] = [:]

func wavs(_ folder: URL) throws -> [URL] {
    try fm.contentsOfDirectory(at: folder, includingPropertiesForKeys: nil)
        .filter { $0.pathExtension == "wav" }
        .sorted { $0.lastPathComponent < $1.lastPathComponent }
}

if only == nil || only == "wer" {
    let folder = dir.appending(path: "fleurs")
    try await pipeline.prepare()
    var lines = "", audioS = 0.0, asrS = 0.0
    let files = try wavs(folder)
    for (i, f) in files.enumerated() {
        let audio = try Pipeline.load(f)
        let t0 = Date()
        let text = try await pipeline.text(audio)
        asrS += Date().timeIntervalSince(t0)
        audioS += Double(audio.count) / 16000
        lines += "\(f.deletingPathExtension().lastPathComponent)\t\(text.replacingOccurrences(of: "\t", with: " ").replacingOccurrences(of: "\n", with: " "))\n"
        if (i + 1) % 25 == 0 { print("wer \(i + 1)/\(files.count)") }
    }
    try lines.write(to: folder.appending(path: "hyp-swift.tsv"), atomically: true, encoding: .utf8)
    timing["asr_rtf"] = asrS / audioS
    timing["asr_audio_min"] = audioS / 60
}

if only == nil || only == "der" {
    let folder = dir.appending(path: "callhome")
    let out = folder.appending(path: "hyp-swift")
    try fm.createDirectory(at: out, withIntermediateDirectories: true)
    var audioS = 0.0, diaS = 0.0
    for f in try wavs(folder) {
        let name = f.deletingPathExtension().lastPathComponent
        let ref = try String(contentsOf: folder.appending(path: "\(name).rttm"), encoding: .utf8)
        let k = Set(ref.split(separator: "\n").map { $0.split(separator: " ")[7] }).count
        let audio = try Pipeline.load(f)
        for (mode, num) in [("auto", nil), ("oracle", k)] as [(String, Int?)] {
            let t0 = Date()
            let spans = try await pipeline.spans(audio, speakers: num)
            if mode == "auto" { diaS += Date().timeIntervalSince(t0) }
            let rttm = spans.map {
                String(format: "SPEAKER %@ 1 %.3f %.3f <NA> <NA> %@ <NA> <NA>\n", name, $0.start, $0.end - $0.start, $0.speaker)
            }.joined()
            try rttm.write(to: out.appending(path: "\(name).\(mode).rttm"), atomically: true, encoding: .utf8)
            print("der \(name) \(mode): \(Set(spans.map(\.speaker)).count) speakers (true \(k))")
        }
        audioS += Double(audio.count) / 16000
    }
    timing["diarization_rtf"] = diaS / audioS
    timing["diarization_audio_min"] = audioS / 60
}

let timingURL = dir.appending(path: "timing-swift.json")
var old = (try? JSONSerialization.jsonObject(with: Data(contentsOf: timingURL))) as? [String: Double] ?? [:]
old.merge(timing) { $1 }
try JSONSerialization.data(withJSONObject: old, options: [.prettyPrinted, .sortedKeys]).write(to: timingURL)
print(old)
