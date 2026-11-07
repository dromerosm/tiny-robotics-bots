import AVFoundation
import AppKit
import CryptoKit
import Foundation
import Observation
import ScribeKit

/// One thing to transcribe: a file from this Mac, or a recording in R2.
struct Item: Identifiable, Hashable {
    enum Kind: Hashable { case file(URL), recording(Recording) }
    var kind: Kind

    var id: String {
        switch kind {
        case .file(let u): "file:" + u.path
        case .recording(let r): "rec:" + r.id
        }
    }

    var title: String {
        switch kind {
        case .file(let u): u.lastPathComponent
        case .recording(let r):
            r.date.map { $0.formatted(date: .abbreviated, time: .shortened) } ?? r.id
        }
    }

    /// File name for the saved transcript.
    var key: String {
        switch kind {
        case .file(let u): "file-" + SHA256.hash(data: Data(u.path.utf8)).prefix(8).map { String(format: "%02x", $0) }.joined()
        case .recording(let r): "rec-" + r.id
        }
    }
}

enum Work: Equatable {
    case idle, downloading, running(String), failed(String)
}

@MainActor @Observable
final class AppModel {
    var files: [Item] = []
    var recordings: [Item] = []
    var selection: Item.ID?
    var transcripts: [String: Transcript] = [:] // by Item.key
    var work: [String: Work] = [:]              // by Item.key
    var recError: String?
    var loadingRecordings = false
    var recURL = ""
    var recToken = ""
    private var settingsLoaded = false

    let player = Player()
    private let pipeline = Pipeline()
    private let folder: URL = {
        let f = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)[0]
            .appending(path: "Scribe")
        try? FileManager.default.createDirectory(at: f.appending(path: "audio"), withIntermediateDirectories: true)
        return f
    }()

    init() {
        if let data = try? Data(contentsOf: folder.appending(path: "files.json")),
           let paths = try? JSONDecoder().decode([String].self, from: data) {
            files = paths.map { Item(kind: .file(URL(fileURLWithPath: $0))) }
                .filter { if case .file(let u) = $0.kind { FileManager.default.fileExists(atPath: u.path) } else { false } }
        }
        for item in files { loadSaved(item) }
    }

    var selected: Item? { (files + recordings).first { $0.id == selection } }

    // MARK: files

    func add(_ urls: [URL]) {
        for u in urls where !files.contains(where: { $0.id == "file:" + u.path }) {
            let item = Item(kind: .file(u))
            files.append(item)
            loadSaved(item)
            selection = item.id
        }
        let paths = files.compactMap { if case .file(let u) = $0.kind { u.path } else { nil } }
        try? JSONEncoder().encode(paths).write(to: folder.appending(path: "files.json"))
    }

    func remove(_ item: Item) {
        files.removeAll { $0.id == item.id }
        add([])
    }

    // MARK: recordings in R2

    /// Reads the Worker URL and token from the keychain, away from the main
    /// thread: macOS may ask you first, and the window must not hang meanwhile.
    func loadSettings() async {
        guard !settingsLoaded else { return }
        let (u, t) = await Task.detached { (Settings.get("url") ?? "", Settings.get("token") ?? "") }.value
        recURL = u
        recToken = t
        settingsLoaded = true
    }

    var client: RecClient? {
        guard let u = URL(string: recURL), !recToken.isEmpty else { return nil }
        return RecClient(base: u, token: recToken)
    }

    func saveSettings() {
        Settings.set("url", recURL)
        Settings.set("token", recToken)
    }

    func importSettings() {
        let url = FileManager.default.homeDirectoryForCurrentUser.appending(path: ".secrets/stackchan-rec.env")
        do {
            (recURL, recToken) = try Settings.importEnv(url)
            saveSettings()
            Task { await refresh() }
        } catch {
            recError = error.localizedDescription
        }
    }

    func refresh() async {
        guard let client else {
            recError = "Add the Worker URL and the read token in Settings."
            return
        }
        loadingRecordings = true
        defer { loadingRecordings = false }
        do {
            recordings = try await client.list().map { Item(kind: .recording($0)) }
            recError = nil
            for item in recordings { loadSaved(item) }
        } catch {
            recError = "Could not list the recordings: \(error.localizedDescription)"
        }
    }

    func delete(_ item: Item) async {
        guard case .recording(let r) = item.kind, let client else { return }
        do {
            try await client.delete(r.id)
            recordings.removeAll { $0.id == item.id }
            if selection == item.id { selection = nil }
            if player.item == item.id { player.stop() }
            try? FileManager.default.removeItem(at: folder.appending(path: "audio/stackchan-\(r.id).wav"))
        } catch {
            recError = "Could not delete: \(error.localizedDescription)"
        }
    }

    // MARK: transcripts

    private func savedURL(_ item: Item) -> URL { folder.appending(path: "\(item.key).json") }

    private func loadSaved(_ item: Item) {
        if let data = try? Data(contentsOf: savedURL(item)),
           let t = try? JSONDecoder().decode(Transcript.self, from: data) {
            transcripts[item.key] = t
        }
    }

    func save(_ item: Item) {
        guard let t = transcripts[item.key] else { return }
        try? JSONEncoder().encode(t).write(to: savedURL(item))
    }

    func rename(_ item: Item, speaker: String, to name: String) {
        transcripts[item.key]?.names[speaker] = name
        save(item)
    }

    /// The audio on this Mac: the file itself, or the recording downloaded
    /// once from R2 and kept in Application Support. A recording that was
    /// still uploading is downloaded again.
    func audioFile(_ item: Item) async throws -> URL {
        switch item.kind {
        case .file(let u): return u
        case .recording(let r):
            let cached = folder.appending(path: "audio/stackchan-\(r.id).wav")
            if r.ended, FileManager.default.fileExists(atPath: cached.path) { return cached }
            guard let client else { throw URLError(.userAuthenticationRequired) }
            work[item.key] = .downloading
            defer { if work[item.key] == .downloading { work[item.key] = .idle } }
            return try await client.download(r.id, to: folder.appending(path: "audio"))
        }
    }

    func transcribe(_ item: Item, speakers: Int?) async {
        let key = item.key
        do {
            let url = try await audioFile(item)
            work[key] = .running("Reading the audio")
            let audio = try await Task.detached { try Pipeline.load(url) }.value
            let names = transcripts[key]?.names ?? [:]
            var t = try await pipeline.run(audio, source: url.path, speakers: speakers) { stage in
                let text = switch stage {
                case .loadingModels: "Loading the models (the first time takes a few minutes)"
                case .transcribing: "Speech to text"
                case .diarizing: "Who spoke when"
                case .done: "Done"
                }
                Task { @MainActor in self.work[key] = .running(text) }
            }
            t.names = names
            transcripts[key] = t
            save(item)
            work[key] = .idle
        } catch {
            work[key] = .failed(error.localizedDescription)
        }
    }

    func play(_ item: Item, at time: Double? = nil) async {
        do {
            if player.item != item.id {
                let url = try await audioFile(item)
                player.load(url, item: item.id)
            }
            if let time { player.seek(time) }
            player.play()
        } catch {
            work[item.key] = .failed("Could not play: \(error.localizedDescription)")
        }
    }

    func exportMarkdown(_ item: Item) {
        guard let t = transcripts[item.key] else { return }
        let panel = NSSavePanel()
        panel.nameFieldStringValue = (item.title as NSString).deletingPathExtension + ".md"
        panel.allowedContentTypes = [.init(filenameExtension: "md")!]
        if panel.runModal() == .OK, let url = panel.url {
            try? Export.markdown(t, title: item.title).write(to: url, atomically: true, encoding: .utf8)
        }
    }

    func copyMarkdown(_ item: Item) {
        guard let t = transcripts[item.key] else { return }
        NSPasteboard.general.clearContents()
        NSPasteboard.general.setString(Export.markdown(t, title: item.title), forType: .string)
    }
}

/// Plays one recording at a time, from a file on this Mac.
@MainActor @Observable
final class Player {
    private(set) var item: Item.ID?
    private(set) var playing = false
    private(set) var time: Double = 0
    private(set) var duration: Double = 0
    private var av: AVPlayer?
    private var observer: Any?
    private var endObserver: NSObjectProtocol?

    func load(_ url: URL, item: Item.ID) {
        stop()
        let p = AVPlayer(url: url)
        av = p
        self.item = item
        Task {
            if let d = try? await p.currentItem?.asset.load(.duration) { duration = d.seconds.isFinite ? d.seconds : 0 }
        }
        observer = p.addPeriodicTimeObserver(forInterval: CMTime(seconds: 0.25, preferredTimescale: 600), queue: .main) { [weak self] t in
            MainActor.assumeIsolated { self?.time = t.seconds }
        }
        endObserver = NotificationCenter.default.addObserver(
            forName: .AVPlayerItemDidPlayToEndTime, object: p.currentItem, queue: .main
        ) { [weak self] _ in
            MainActor.assumeIsolated {
                self?.playing = false
                self?.seek(0)
            }
        }
    }

    func play() { av?.play(); playing = av != nil }
    func pause() { av?.pause(); playing = false }
    func toggle() { playing ? pause() : play() }

    func seek(_ seconds: Double) {
        time = seconds
        av?.seek(to: CMTime(seconds: seconds, preferredTimescale: 600), toleranceBefore: .zero, toleranceAfter: .zero)
    }

    func stop() {
        av?.pause()
        if let observer { av?.removeTimeObserver(observer) }
        if let endObserver { NotificationCenter.default.removeObserver(endObserver) }
        observer = nil
        endObserver = nil
        av = nil
        item = nil
        playing = false
        time = 0
        duration = 0
    }
}
