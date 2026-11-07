"""Transcribe and diarize one recording on a Mac (Apple Silicon).

    uv run transcribe.py meeting.wav [--speakers 2] [--out .work/out]

Speech to text: mlx-whisper (large-v3-turbo) on the GPU, with word times.
Who spoke when: pyannote speaker-diarization-community-1 on MPS.
Each word goes to the speaker who talks most during that word.
Writes <name>.json (words, turns) and <name>.md (one line per turn).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np

SR = 16000
ASR_MODEL = "mlx-community/whisper-large-v3-turbo"
DIA_MODEL = "pyannote/speaker-diarization-community-1"

_dia = None


def load_audio(path: str | Path) -> np.ndarray:
    """Any audio file -> 16 kHz mono float32, decoded by ffmpeg."""
    raw = subprocess.run(
        ["ffmpeg", "-nostdin", "-v", "error", "-i", str(path),
         "-ac", "1", "-ar", str(SR), "-f", "f32le", "-"],
        check=True, capture_output=True).stdout
    return np.frombuffer(raw, np.float32).copy()


def asr(audio: np.ndarray, language: str = "es") -> dict:
    import mlx_whisper
    return mlx_whisper.transcribe(
        audio, path_or_hf_repo=ASR_MODEL, language=language,
        word_timestamps=True, condition_on_previous_text=False)


def diarize(audio: np.ndarray, speakers: int | None = None):
    """Returns a pyannote Annotation without overlapping turns."""
    global _dia
    import torch
    from pyannote.audio import Pipeline
    if _dia is None:
        _dia = Pipeline.from_pretrained(DIA_MODEL)
        if _dia is None:
            raise SystemExit(f"Cannot load {DIA_MODEL}: accept its terms on huggingface.co")
        _dia.to(torch.device("mps" if torch.backends.mps.is_available() else "cpu"))
    wav = torch.from_numpy(audio).unsqueeze(0)
    out = _dia({"waveform": wav, "sample_rate": SR}, num_speakers=speakers)
    return out


def assign(words: list[dict], turns) -> list[dict]:
    """Give each word the speaker with most overlap (or the nearest turn)."""
    segs = [(t.start, t.end, spk) for t, _, spk in turns.itertracks(yield_label=True)]
    for w in words:
        best, best_ov = None, 0.0
        for s, e, spk in segs:
            ov = min(e, w["end"]) - max(s, w["start"])
            if ov > best_ov:
                best, best_ov = spk, ov
        if best is None and segs:
            mid = (w["start"] + w["end"]) / 2
            best = min(segs, key=lambda x: min(abs(x[0] - mid), abs(x[1] - mid)))[2]
        w["speaker"] = best
    return words


def group(words: list[dict]) -> list[dict]:
    out: list[dict] = []
    for w in words:
        if out and out[-1]["speaker"] == w["speaker"]:
            out[-1]["end"] = w["end"]
            out[-1]["text"] += w["word"]
        else:
            out.append({"speaker": w["speaker"], "start": w["start"],
                        "end": w["end"], "text": w["word"]})
    for t in out:
        t["text"] = t["text"].strip()
    return out


def run(path: Path, speakers: int | None, out_dir: Path) -> dict:
    audio = load_audio(path)
    dur = len(audio) / SR
    t0 = time.perf_counter()
    text = asr(audio)
    t1 = time.perf_counter()
    dia = diarize(audio, speakers)
    t2 = time.perf_counter()
    words = [{"word": w["word"], "start": w["start"], "end": w["end"]}
             for s in text["segments"] for w in s.get("words", [])]
    turns = group(assign(words, dia.exclusive_speaker_diarization))
    result = {"file": str(path), "duration_s": round(dur, 1),
              "asr_s": round(t1 - t0, 1), "diarization_s": round(t2 - t1, 1),
              "speakers": sorted(dia.speaker_diarization.labels()),
              "turns": turns, "words": words}
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"{path.stem}.json").write_text(json.dumps(result, ensure_ascii=False, indent=1))
    md = [f"# {path.name}\n"]
    md += [f"**{t['speaker']}** [{t['start']:.1f}s] {t['text']}\n" for t in turns]
    (out_dir / f"{path.stem}.md").write_text("\n".join(md))
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("audio", type=Path)
    ap.add_argument("--speakers", type=int, help="number of speakers, if you know it")
    ap.add_argument("--out", type=Path, default=Path(".work/out"))
    a = ap.parse_args()
    r = run(a.audio, a.speakers, a.out)
    print(f"{r['duration_s']} s of audio: ASR {r['asr_s']} s, diarization {r['diarization_s']} s, "
          f"{len(r['speakers'])} speakers -> {a.out}/{a.audio.stem}.md")


if __name__ == "__main__":
    main()
