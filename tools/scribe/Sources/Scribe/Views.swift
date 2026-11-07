import ScribeKit
import SwiftUI
import UniformTypeIdentifiers

@main
struct ScribeApp: App {
    @State private var model = AppModel()

    init() {
        // A plain SwiftPM executable starts as a background process: make it a
        // normal app with a Dock icon and a menu bar.
        NSApplication.shared.setActivationPolicy(.regular)
        DispatchQueue.main.async { NSApplication.shared.activate(ignoringOtherApps: true) }
    }

    var body: some Scene {
        WindowGroup("Scribe") {
            ContentView().environment(model).frame(minWidth: 820, minHeight: 520)
        }
        .commands {
            CommandGroup(after: .newItem) {
                Button("Open Audio…") { openPanel(model) }.keyboardShortcut("o")
            }
        }
        Settings { SettingsView().environment(model) }
    }
}

@MainActor func openPanel(_ model: AppModel) {
    let panel = NSOpenPanel()
    panel.allowedContentTypes = [.audio, .movie]
    panel.allowsMultipleSelection = true
    if panel.runModal() == .OK { model.add(panel.urls) }
}

struct ContentView: View {
    @Environment(AppModel.self) private var model
    @State private var dropping = false
    @State private var toDelete: Item?

    var body: some View {
        @Bindable var model = model
        NavigationSplitView {
            List(selection: $model.selection) {
                Section("Files on this Mac") {
                    if model.files.isEmpty {
                        Text("Drop audio files here").foregroundStyle(.secondary)
                    }
                    ForEach(model.files) { item in
                        Row(item: item).contextMenu {
                            Button("Remove from the list") { model.remove(item) }
                        }
                    }
                }
                Section {
                    if let e = model.recError {
                        Text(e).font(.caption).foregroundStyle(.secondary)
                    }
                    ForEach(model.recordings) { item in
                        Row(item: item).contextMenu {
                            Button("Play") { Task { await model.play(item) } }
                            Divider()
                            Button("Delete from R2…", role: .destructive) { toDelete = item }
                        }
                    }
                } header: {
                    HStack {
                        Text("StackChan (R2)")
                        Spacer()
                        if model.loadingRecordings { ProgressView().controlSize(.small) }
                        Button { Task { await model.refresh() } } label: { Image(systemName: "arrow.clockwise") }
                            .buttonStyle(.borderless).help("Get the list of recordings")
                    }
                }
            }
            .navigationSplitViewColumnWidth(min: 240, ideal: 280)
            .toolbar {
                Button { openPanel(model) } label: { Label("Open", systemImage: "plus") }
            }
        } detail: {
            if let item = model.selected {
                DetailView(item: item).id(item.id)
            } else {
                ContentUnavailableView("No recording selected", systemImage: "waveform",
                                       description: Text("Drop an audio file, or pick a StackChan recording."))
            }
        }
        .onDrop(of: [.fileURL], isTargeted: $dropping) { providers in
            for p in providers {
                _ = p.loadObject(ofClass: URL.self) { url, _ in
                    if let url { Task { @MainActor in model.add([url]) } }
                }
            }
            return true
        }
        .overlay {
            if dropping {
                RoundedRectangle(cornerRadius: 12).stroke(.tint, lineWidth: 3).padding(4).allowsHitTesting(false)
            }
        }
        .task {
            await model.loadSettings()
            if model.client != nil { await model.refresh() }
        }
        .confirmationDialog("Delete \(toDelete?.title ?? "") from R2?", isPresented: Binding(
            get: { toDelete != nil }, set: { if !$0 { toDelete = nil } })
        ) {
            Button("Delete", role: .destructive) {
                if let item = toDelete { Task { await model.delete(item) } }
                toDelete = nil
            }
        } message: {
            Text("It can't be undone. A transcript you made stays on this Mac.")
        }
    }
}

struct Row: View {
    @Environment(AppModel.self) private var model
    let item: Item

    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text(item.title).lineLimit(1)
                if case .recording(let r) = item.kind {
                    Text(Export.clock(Double(r.seconds)) + (r.ended ? "" : " · still uploading")
                         + (r.missing > 0 ? " · \(r.missing) gaps" : ""))
                        .font(.caption).foregroundStyle(.secondary)
                }
            }
            Spacer()
            switch model.work[item.key] ?? .idle {
            case .downloading, .running: ProgressView().controlSize(.small)
            case .failed: Image(systemName: "exclamationmark.triangle").foregroundStyle(.orange)
            case .idle:
                if model.transcripts[item.key] != nil {
                    Image(systemName: "text.bubble").foregroundStyle(.secondary).help("Transcribed")
                }
            }
        }
    }
}

struct DetailView: View {
    @Environment(AppModel.self) private var model
    let item: Item
    @State private var autoSpeakers = true
    @State private var speakers = 2
    @State private var confirmDelete = false

    private let palette: [Color] = [.blue, .orange, .green, .purple, .pink, .teal, .brown, .indigo]

    var body: some View {
        let work = model.work[item.key] ?? .idle
        let transcript = model.transcripts[item.key]
        VStack(alignment: .leading, spacing: 0) {
            HStack(alignment: .center, spacing: 12) {
                Toggle("Find the speakers", isOn: $autoSpeakers).toggleStyle(.checkbox)
                if !autoSpeakers {
                    Stepper("\(speakers) speakers", value: $speakers, in: 1...10).fixedSize()
                }
                Spacer()
                switch work {
                case .downloading: ProgressView().controlSize(.small); Text("Downloading").foregroundStyle(.secondary)
                case .running(let s): ProgressView().controlSize(.small); Text(s).foregroundStyle(.secondary)
                case .failed(let e): Text(e).foregroundStyle(.orange).lineLimit(2)
                case .idle: EmptyView()
                }
                Button(transcript == nil ? "Transcribe" : "Transcribe again") {
                    Task { await model.transcribe(item, speakers: autoSpeakers ? nil : speakers) }
                }
                .disabled(work != .idle && !(work.isFailed))
                .keyboardShortcut(.return, modifiers: .command)
            }
            .padding()
            PlayerBar(item: item)
            Divider()
            if let t = transcript {
                TranscriptView(item: item, transcript: t, palette: palette)
            } else {
                ContentUnavailableView("Not transcribed yet", systemImage: "text.bubble",
                                       description: Text("Everything runs on this Mac: WhisperKit for the words, FluidAudio for the speakers."))
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .top)
        .navigationTitle(item.title)
        .toolbar {
            if transcript != nil {
                Button { model.copyMarkdown(item) } label: { Label("Copy", systemImage: "doc.on.doc") }
                    .help("Copy as Markdown")
                Button { model.exportMarkdown(item) } label: { Label("Export", systemImage: "square.and.arrow.up") }
                    .help("Save as Markdown")
            }
            if case .recording = item.kind {
                Button(role: .destructive) { confirmDelete = true } label: { Label("Delete", systemImage: "trash") }
                    .help("Delete this recording from R2")
            }
        }
        .confirmationDialog("Delete this recording from R2?", isPresented: $confirmDelete) {
            Button("Delete", role: .destructive) { Task { await model.delete(item) } }
        } message: {
            Text("It can't be undone. A transcript you made stays on this Mac.")
        }
    }
}

extension Work {
    var isFailed: Bool { if case .failed = self { true } else { false } }
}

struct TranscriptView: View {
    @Environment(AppModel.self) private var model
    let item: Item
    let transcript: Transcript
    let palette: [Color]

    func color(_ speaker: String) -> Color {
        palette[(transcript.speakers.firstIndex(of: speaker) ?? 0) % palette.count]
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 0) {
            HStack(spacing: 16) {
                Text("\(Export.clock(transcript.duration)) of audio · words \(Int(transcript.asrSeconds)) s · speakers \(Int(transcript.diarizationSeconds)) s")
                    .font(.caption).foregroundStyle(.secondary)
                Spacer()
            }
            .padding(.horizontal).padding(.top, 8)
            ScrollView(.horizontal, showsIndicators: false) {
                HStack {
                    ForEach(transcript.speakers, id: \.self) { s in
                        SpeakerField(item: item, speaker: s, name: transcript.names[s] ?? "", color: color(s))
                    }
                }
                .padding(.horizontal).padding(.vertical, 8)
            }
            Divider()
            List(transcript.turns) { turn in
                let now = model.player.item == item.id && model.player.time >= turn.start && model.player.time < turn.end + 0.3
                HStack(alignment: .firstTextBaseline, spacing: 10) {
                    Button(Export.clock(turn.start)) { Task { await model.play(item, at: turn.start) } }
                        .buttonStyle(.borderless).font(.caption.monospacedDigit())
                        .foregroundStyle(now ? Color.accentColor : .secondary)
                        .help("Play from here")
                        .frame(width: 52, alignment: .trailing)
                    Text(transcript.name(turn.speaker)).bold().foregroundStyle(color(turn.speaker))
                        .frame(width: 110, alignment: .leading).lineLimit(1)
                    Text(turn.text).textSelection(.enabled)
                }
                .padding(.vertical, 2)
                .listRowBackground(now ? Color.accentColor.opacity(0.12) : Color.clear)
            }
        }
    }
}

struct SpeakerField: View {
    @Environment(AppModel.self) private var model
    let item: Item
    let speaker: String
    @State var name: String
    let color: Color

    init(item: Item, speaker: String, name: String, color: Color) {
        self.item = item
        self.speaker = speaker
        _name = State(initialValue: name)
        self.color = color
    }

    var body: some View {
        HStack(spacing: 6) {
            Circle().fill(color).frame(width: 10, height: 10)
            TextField(speaker, text: $name)
                .textFieldStyle(.roundedBorder).frame(width: 130)
                .onSubmit { model.rename(item, speaker: speaker, to: name) }
                .onChange(of: name) { _, new in model.rename(item, speaker: speaker, to: new) }
        }
    }
}

struct SettingsView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        Form {
            Section("StackChan recordings (workers/rec)") {
                TextField("Worker URL", text: $model.recURL, prompt: Text("https://…/rec"))
                SecureField("Read token", text: $model.recToken)
                HStack {
                    Button("Import from ~/.secrets/stackchan-rec.env") { model.importSettings() }
                    Spacer()
                    Button("Save") {
                        model.saveSettings()
                        Task { await model.refresh() }
                    }
                    .keyboardShortcut(.defaultAction)
                }
                Text("Kept in your login keychain. This token can read and delete recordings, not add them.")
                    .font(.caption).foregroundStyle(.secondary)
            }
        }
        .formStyle(.grouped)
        .frame(width: 520)
        .padding()
        .task { await model.loadSettings() }
    }
}

/// Play, pause and move through the recording. A StackChan recording is
/// downloaded once, the first time you play or transcribe it.
struct PlayerBar: View {
    @Environment(AppModel.self) private var model
    let item: Item
    @State private var scrub: Double?

    var body: some View {
        let p = model.player
        let mine = p.item == item.id
        let downloading = model.work[item.key] == .downloading
        HStack(spacing: 10) {
            Button {
                if mine { p.toggle() } else { Task { await model.play(item) } }
            } label: {
                Image(systemName: mine && p.playing ? "pause.fill" : "play.fill").frame(width: 16)
            }
            .buttonStyle(.borderless)
            .disabled(downloading)
            .help(mine && p.playing ? "Pause" : "Play")
            if downloading {
                ProgressView().controlSize(.small)
                Text("Downloading").font(.caption).foregroundStyle(.secondary)
            }
            Text(Export.clock(mine ? (scrub ?? p.time) : 0)).font(.caption.monospacedDigit()).foregroundStyle(.secondary)
            Slider(value: Binding(
                get: { mine ? (scrub ?? p.time) : 0 },
                set: { scrub = $0 }
            ), in: 0...max(mine ? p.duration : 1, 1)) { editing in
                if !editing, let s = scrub { p.seek(s); scrub = nil }
            }
            .disabled(!mine)
            Text(Export.clock(mine ? p.duration : 0)).font(.caption.monospacedDigit()).foregroundStyle(.secondary)
        }
        .padding(.horizontal).padding(.bottom, 10)
    }
}
