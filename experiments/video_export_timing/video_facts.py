"""Prints the hard-gate facts of one or more video files: frame count (by decoding the stream), resolution, frame rate, length, codec, container and size.

    python experiments/video_export_timing/video_facts.py <video.mp4> [...]

Uses the ffmpeg of the imageio-ffmpeg package (no ffprobe on this machine). Prints one JSON object per file.
"""

import json
import re
import subprocess
import sys
from pathlib import Path

import imageio_ffmpeg


def facts(path):
    exe = imageio_ffmpeg.get_ffmpeg_exe()
    info = subprocess.run([exe, "-i", str(path)], capture_output=True, text=True).stderr
    out = {"file": str(path), "size_bytes": Path(path).stat().st_size}
    m = re.search(r"Duration: (\d+):(\d+):([\d.]+)", info)
    out["duration_s"] = round(int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3]), 3) if m else None
    m = re.search(r"Video: (\w+)[^\n]*?(\d{3,5})x(\d{3,5})[^\n]*?([\d.]+) fps", info)
    out["codec"], out["width"], out["height"], out["fps"] = (m[1], int(m[2]), int(m[3]), float(m[4])) if m else (None, None, None, None)
    m = re.search(r"Input #0, ([\w,]+)", info)
    out["container"] = m[1] if m else None
    m = re.search(r"bitrate: (\d+) kb/s", info)
    out["bitrate_kbps"] = int(m[1]) if m else None
    cnt = subprocess.run([exe, "-nostdin", "-i", str(path), "-map", "0:v:0", "-f", "null", "-"], capture_output=True, text=True).stderr   # decodes every frame
    frames = re.findall(r"frame=\s*(\d+)", cnt)
    out["frames"] = int(frames[-1]) if frames else None
    return out


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(json.dumps(facts(p)))
