# Scribe: a Mac app for the recordings

A small native macOS app (SwiftUI) that turns a recording into text and says who spoke when. Everything runs on the Mac, with Core ML. It opens audio files from the Mac and the recordings that StackChan uploads with [14-record](../../examples/stackchan/14-record).

| | |
| --- | --- |
| **Speech to text** | [WhisperKit](https://github.com/argmaxinc/argmax-oss-swift) (MIT), model `large-v3-v20240930_turbo` |
| **Who spoke when** | [FluidAudio](https://github.com/FluidInference/FluidAudio) (Apache 2.0), its offline pipeline: pyannote community-1 converted to Core ML |
| **Needs** | macOS 15 on Apple Silicon, Xcode 16 or later (for `swift build`) |

## What it does

- Drop audio files on the window, or open them with ⌘O.
- Lists the StackChan recordings in R2, through the Worker in [workers/rec](../../workers/rec). It downloads one the first time you play or transcribe it, and keeps it on the Mac.
- Plays the audio: play and pause, a bar to move through it, and the time of each turn is a button that plays from there. The turn you hear is marked.
- Deletes a recording from R2, from the trash button or with a right click on it in the list. It asks first. The transcript stays on the Mac.
- Finds the speakers by itself, or you tell it how many there are.
- Give each speaker a name (`S1` → "Diego"). The names stay when you transcribe again.
- Copy or save the result as Markdown, one line per turn.

Transcripts and downloaded audio stay in `~/Library/Application Support/Scribe/`. The Worker URL and the read token are kept in the login keychain.

## Build and run

```bash
./make-app.sh
open .work/Scribe.app
```

`make-app.sh` builds in release mode and puts `Scribe.app` in `.work/` (not in git). It signs the app with your Apple Development certificate if you have one, or ad hoc if not. It also draws the icon with `make-icon.swift` (a speech bubble with a sound wave, one color per speaker) and turns it into `AppIcon.icns`.

The first time the app reads its token, macOS asks if Scribe may use it. Choose **Always Allow**. With the Apple Development signature it does not ask again after a new build. With an ad hoc signature it asks after every build, because each build looks like a new app.

The first time, open **Scribe → Settings…** and press **Import from ~/.secrets/stackchan-rec.env**, or type the Worker URL (`https://…/rec`) and the read token.

The first transcription downloads the models (about 1.6 GB) and compiles them for this Mac. That took about 80 s on an M4. After that, 63 s of audio took 3.3 s for the words and 1.5 s for the speakers.

## Benchmark against the Python pipeline

The Swift engines run on the same files as [tools/transcribe](../transcribe), and the same Python code scores both:

```bash
cd ../transcribe && uv run bench.py export     # once: the audio and the labels as files
cd ../scribe && swift run -c release scribe-bench ../transcribe/.work/export
cd ../transcribe && uv run bench.py score swift
```

Results on an Apple M4, 2026-11-07:

| | Python (mlx-whisper, pyannote) | Swift (WhisperKit, FluidAudio) |
| --- | --- | --- |
| WER, FLEURS es_419, 200 clips | **2.5 %** (CER 0.95 %) | 3.4 % (CER 1.9 %) |
| Speech to text, time per minute of audio | 13 s | **7 s** |
| DER, CallHome Spanish, 7 calls | **26.8 %** | 40.4 % (35.4 % told the speaker count) |
| Who spoke when, time per minute of audio | 4 s | **0.25 s** |

- Part of the Swift WER is number format: WhisperKit writes "100" where the reference has "cien", and "30" for "treinta por ciento". But it also has real errors, like "80 000" for "800 000".
- FluidAudio is 7 to 13 points worse than pyannote on every call, even when it finds the right number of speakers. CallHome is phone audio with a lot of overlap. FluidAudio's own tests use meetings (AMI), so it may do better on StackChan's room sound. We have not measured that yet.
- Our advice: use the app when you want it fast and easy. When the speakers must be right, use [tools/transcribe](../transcribe).
