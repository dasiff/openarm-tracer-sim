"""Contact sheet of two videos: the same frame of each side by side, at 12 evenly spaced times of the whole video and 3 times inside the stance search.

    python experiments/video_export_timing/contact_sheet.py <old.mp4> <new.mp4> <out.png> [--search-frames 685]

Each cell is the old picture on the left and the new one on the right (480 x 270 each) with the frame number and video second above it. Two cells per row.
"""

import argparse

import cv2
import numpy as np


def frame_at(path, index):
    cap = cv2.VideoCapture(path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, index)
    ok, f = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"could not read frame {index} of {path}")
    return f


def main(a):
    cap = cv2.VideoCapture(a.new)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    even = [int(round(i * (n - 1) / 11)) for i in range(12)]
    search = [int(a.search_frames * f) for f in (0.2, 0.55, 0.85)]          # inside the stance search part (the first search_frames frames)
    items = [(i, "search") for i in search] + [(i, "run") for i in even]
    items.sort(key=lambda x: x[0])
    cw, ch, bar = 480, 270, 22
    cells = []
    for idx, kind in items:
        old, new = frame_at(a.old, idx), frame_at(a.new, idx)
        pair = np.hstack([cv2.resize(old, (cw, ch), interpolation=cv2.INTER_AREA), cv2.resize(new, (cw, ch), interpolation=cv2.INTER_AREA)])
        head = np.full((bar, pair.shape[1], 3), 30, np.uint8)
        label = f"frame {idx}  t = {idx / fps:6.2f} s  ({'inside the stance search' if kind == 'search' else 'evenly spaced'})     old (left)  |  new (right)"
        cv2.putText(head, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cell = np.vstack([head, pair])
        cv2.line(cell, (cw, bar), (cw, bar + ch), (0, 200, 255), 1)
        cells.append(cell)
    if len(cells) % 2:
        cells.append(np.zeros_like(cells[0]))
    rows = [np.hstack([cells[i], np.full((cells[i].shape[0], 6, 3), 90, np.uint8), cells[i + 1]]) for i in range(0, len(cells), 2)]
    sheet = np.vstack([np.vstack([r, np.full((6, r.shape[1], 3), 90, np.uint8)]) for r in rows])
    cv2.imwrite(a.out, sheet)
    print(f"{a.out}: {sheet.shape[1]}x{sheet.shape[0]}, frames {[i for i, _ in items]}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("old")
    ap.add_argument("new")
    ap.add_argument("out")
    ap.add_argument("--search-frames", type=int, default=685)
    main(ap.parse_args())
