"""Benchmark the pipeline on public Spanish datasets.

    uv run bench.py wer [--n 200]   # speech to text on FLEURS es_419 (test)
    uv run bench.py der [--n 5]     # who spoke when on CallHome Spanish

WER: Google FLEURS, Latin American Spanish, read speech at 16 kHz, CC BY 4.0.
     Both sides are lower case, without punctuation, before scoring.
DER: TalkBank CallHome Spanish, phone calls (8 kHz audio stored at 16 kHz),
     CC BY-NC-SA 4.0 (gated). Some calls in the Hugging Face copy are broken:
     audio shorter than its labels, or no speech in it. They are skipped
     and listed in der.json ("skipped"), not scored.
     Full DER: no collar, overlap scored, as in pyannote's own benchmarks.
     Run twice per call: speakers found by the model, and told the true number.
Data and results go to .work/ (not in git).
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import re
import tarfile
import time
import unicodedata
import urllib.request
from pathlib import Path

import numpy as np

import transcribe as tr

WORK = Path(".work")
HF = "https://huggingface.co/datasets"
FLEURS = f"{HF}/google/fleurs/resolve/main/data/es_419"
CALLHOME = f"{HF}/talkbank/callhome/resolve/main/spa/data-00000-of-00005.parquet"


def fetch(url: str, dest: Path, token: bool = False) -> Path:
    if dest.exists():
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    req = urllib.request.Request(url)
    if token:
        from huggingface_hub import get_token
        req.add_header("Authorization", f"Bearer {get_token()}")
    print(f"downloading {url}")
    tmp = dest.with_suffix(dest.suffix + ".part")
    with urllib.request.urlopen(req) as r, open(tmp, "wb") as f:
        while chunk := r.read(1 << 20):
            f.write(chunk)
    tmp.rename(dest)
    return dest


def norm(s: str) -> str:
    s = unicodedata.normalize("NFC", s.lower())
    s = re.sub(r"[^\w\s']", " ", s)
    return re.sub(r"\s+", " ", s).strip()


def wer(n: int) -> dict:
    import jiwer
    d = WORK / "fleurs"
    tsv = fetch(f"{FLEURS}/test.tsv", d / "test.tsv")
    tgz = fetch(f"{FLEURS}/audio/test.tar.gz", d / "test.tar.gz")
    rows = {}
    with open(tsv, newline="") as f:
        for r in csv.reader(f, delimiter="\t", quoting=csv.QUOTE_NONE):
            rows.setdefault(r[1], r[3])  # file -> normalized transcript
    refs, hyps, audio_s, asr_s = [], [], 0.0, 0.0
    with tarfile.open(tgz, "r|gz") as tar:
        for m in tar:
            name = Path(m.name).name
            if not m.isfile() or name not in rows:
                continue
            raw = tar.extractfile(m).read()
            tmp = d / "cur.wav"
            tmp.write_bytes(raw)
            audio = tr.load_audio(tmp)
            t0 = time.perf_counter()
            hyp = tr.asr(audio)["text"]
            asr_s += time.perf_counter() - t0
            audio_s += len(audio) / tr.SR
            refs.append(norm(rows[name]))
            hyps.append(norm(hyp))
            if len(refs) % 25 == 0:
                print(f"{len(refs)}/{n}  WER so far {jiwer.wer(refs, hyps):.3f}")
            if len(refs) >= n:
                break
    out = {"dataset": "FLEURS es_419 test", "utterances": len(refs),
           "audio_min": round(audio_s / 60, 1), "wer": round(jiwer.wer(refs, hyps), 4),
           "cer": round(jiwer.cer(refs, hyps), 4), "asr_rtf": round(asr_s / audio_s, 3)}
    (WORK / "wer.json").write_text(json.dumps(out, indent=1))
    with open(WORK / "wer-pairs.tsv", "w") as f:
        f.writelines(f"{r}\t{h}\n" for r, h in zip(refs, hyps))
    return out


def der(n: int) -> dict:
    import pyarrow.parquet as pq
    import soundfile as sf
    from pyannote.core import Annotation, Segment
    from pyannote.metrics.diarization import DiarizationErrorRate
    pf = pq.ParquetFile(fetch(CALLHOME, WORK / "callhome" / "spa-0.parquet", token=True))
    print("columns:", pf.schema_arrow.names)
    metrics = {"auto": DiarizationErrorRate(collar=0.0, skip_overlap=False),
               "oracle": DiarizationErrorRate(collar=0.0, skip_overlap=False)}
    calls, skipped, audio_s, dia_s = [], [], 0.0, 0.0
    for batch in pf.iter_batches(batch_size=1):
        r = batch.to_pylist()[0]
        ref = Annotation()
        for s, e, spk in zip(r["timestamps_start"], r["timestamps_end"], r["speakers"]):
            ref[Segment(s, e)] = spk
        wav, sr = sf.read(io.BytesIO(r["audio"]["bytes"]), dtype="float32")
        if wav.ndim > 1:
            wav = wav.mean(axis=1)
        tmp = WORK / "callhome" / "cur.wav"
        sf.write(tmp, wav, sr)
        audio = tr.load_audio(tmp)
        k = len(set(r["speakers"]))
        idx = len(calls) + len(skipped)
        row = {"call": idx, "minutes": round(len(audio) / tr.SR / 60, 1), "speakers": k}
        last = max(r["timestamps_end"])
        if last > 1.05 * len(audio) / tr.SR:
            skipped.append({**row, "reason": f"audio {len(audio) / tr.SR:.0f} s, labels to {last:.0f} s"})
            print("skip", skipped[-1])
            continue
        t0 = time.perf_counter()
        hyps = {"auto": tr.diarize(audio).speaker_diarization}
        auto_s = time.perf_counter() - t0
        hyps["oracle"] = tr.diarize(audio, k).speaker_diarization
        if not hyps["oracle"].labels():
            skipped.append({**row, "reason": "no speech found, even with the true speaker count"})
            print("skip", skipped[-1])
            continue
        row["found"] = len(hyps["auto"].labels())
        for mode, hyp in hyps.items():
            row[f"der_{mode}"] = round(metrics[mode](ref, hyp), 4)
        dia_s += auto_s
        audio_s += len(audio) / tr.SR
        calls.append(row)
        print(row)
        if len(calls) >= n:
            break
    out = {"dataset": "CallHome Spanish (shard 0)", "calls": len(calls),
           "audio_min": round(audio_s / 60, 1),
           "der_auto": round(abs(metrics["auto"]), 4),
           "der_oracle": round(abs(metrics["oracle"]), 4),
           "diarization_rtf": round(dia_s / audio_s, 3), "per_call": calls,
           "skipped": skipped}
    (WORK / "der.json").write_text(json.dumps(out, indent=1))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task", choices=["wer", "der"])
    ap.add_argument("--n", type=int)
    a = ap.parse_args()
    out = wer(a.n or 200) if a.task == "wer" else der(a.n or 5)
    print(json.dumps({k: v for k, v in out.items() if k != "per_call"}, indent=1))


if __name__ == "__main__":
    main()
