# Transcribe and diarize on a Mac

This tool turns a recording into text and says who spoke when. It runs on a Mac with Apple Silicon and nothing goes to the cloud. Later it will handle the recordings StackChan uploads to R2. For now it works on any audio file.

| | |
| --- | --- |
| **Speech to text** | [mlx-whisper](https://github.com/ml-explore/mlx-examples/tree/main/whisper), model `mlx-community/whisper-large-v3-turbo`, on the GPU |
| **Who spoke when** | [pyannote](https://github.com/pyannote/pyannote-audio) `speaker-diarization-community-1`, on MPS |
| **Needs** | macOS on Apple Silicon, [uv](https://docs.astral.sh/uv/), ffmpeg, a Hugging Face token (`hf auth login`) |

## Setup

Accept the terms of these models and datasets on Hugging Face, with the same account as your token:

- [pyannote/speaker-diarization-community-1](https://huggingface.co/pyannote/speaker-diarization-community-1)
- [pyannote/segmentation-3.0](https://huggingface.co/pyannote/segmentation-3.0)
- [talkbank/callhome](https://huggingface.co/datasets/talkbank/callhome), only for the diarization benchmark

Then:

```bash
uv sync
```

## Use

```bash
uv run transcribe.py meeting.wav            # the model finds the speakers
uv run transcribe.py meeting.wav --speakers 3
```

You get `.work/out/meeting.md`, one line per turn, and `meeting.json` with each word, its time and its speaker. Each word goes to the speaker who talks most while it is said.

## Benchmark

```bash
uv run bench.py wer --n 200   # speech to text
uv run bench.py der --n 10    # who spoke when
```

The data (about 1 GB) and the results go to `.work/`, which is not in git.

To score another engine on the same data with the same code (the Swift app in [tools/scribe](../scribe) does this):

```bash
uv run bench.py export        # the audio and the labels as files, in .work/export/
uv run bench.py score swift   # scores .work/export/*/hyp-swift*
```

- **WER** (word error rate) on [Google FLEURS](https://huggingface.co/datasets/google/fleurs), Latin American Spanish, test split. This is read speech at 16 kHz, CC BY 4.0. We use lower case and no punctuation on both sides before scoring.
- **DER** (diarization error rate) on [TalkBank CallHome](https://huggingface.co/datasets/talkbank/callhome) Spanish, shard 0. These are phone calls, CC BY-NC-SA 4.0. We report the full DER: no collar, and overlapping speech counts. It is the strict version, the same one pyannote uses in its own benchmarks.

### Results (Apple M4, 24 GB, 2026-11-07)

| | Data | Audio | Result | Time per minute of audio |
| --- | --- | --- | --- | --- |
| Speech to text | FLEURS es_419, first 200 clips | 41.5 min | **WER 2.5 %**, CER 0.95 % | 13 s |
| Who spoke when | CallHome Spanish, 7 good calls of the first 10 | about 60 min | **DER 26.8 %** | 4 s |

- Almost all the word errors are names: Duvall → Duval, Yakarta → Jakarta, Carpanedo → Carpaneo.
- pyannote found the right number of speakers on its own in all 7 calls. So telling it the number gave the same DER.
- One hour of audio takes about 17 minutes in all.
- These data sets are read speech and phone calls. StackChan hears a room from a distance, with echo and motor noise, so its real numbers will come from our own recordings. Use these numbers as a baseline: if a change makes them worse, you will see it.

### Broken calls in CallHome

Some calls in the Hugging Face copy of CallHome are broken. In shard 0, 4 of the 28 calls (1, 8, 17, 27) have audio 37 to 67 % shorter than their labels. Call 2 has the right length but no speech you can understand: Whisper only makes up a line, and pyannote finds no speech. The first run scored them anyway and gave a DER of 51.7 %.

`bench.py der` now skips a call when its labels go past the end of the audio, or when pyannote finds no speech even with the true number of speakers. It lists those calls in `der.json` under `skipped`, with the reason. The 26.8 % above comes from the 10-call run. We left out its 3 broken calls and weighted each call by how much speech it has.
