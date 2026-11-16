"""Compares two videos frame by frame: SSIM (structural similarity, 1.0 = identical) of every frame against the reference, plus the hard-gate facts.

    python experiments/video_export_timing/ssim_compare.py <reference.mp4> <test.mp4> [--out result.json] [--min-frame 0.98] [--min-mean 0.99] [--max-frames N]

SSIM is the standard one (Wang et al. 2004): on the grey picture (OpenCV BGR to grey), 11 x 11 Gaussian window with sigma 1.5, constants C1 = (0.01 * 255)^2 and
C2 = (0.03 * 255)^2, averaged over the whole frame. The frames are decoded with OpenCV's FFmpeg backend. Also counts the frames that are pixel-identical.
The gate (task of 2026-11-16, proposed values): every frame >= --min-frame (0.98) and the mean >= --min-mean (0.99); the near-gate band is 0.98 to 0.985 for
a frame and 0.99 to 0.993 for the mean. Exit status 0 if the gate passes (same frame count, size, rate; every frame and the mean at or above the limits), 1 if not.
"""

import argparse
import json
import sys
import time

import cv2
import numpy as np

C1, C2 = (0.01 * 255) ** 2, (0.03 * 255) ** 2


def ssim_gray(a, b):
    """Mean SSIM of two uint8 grey images of the same shape (Gaussian window 11 x 11, sigma 1.5)."""
    a, b = a.astype(np.float32), b.astype(np.float32)
    blur = lambda x: cv2.GaussianBlur(x, (11, 11), 1.5, borderType=cv2.BORDER_REFLECT)
    mu_a, mu_b = blur(a), blur(b)
    s_aa, s_bb, s_ab = blur(a * a) - mu_a * mu_a, blur(b * b) - mu_b * mu_b, blur(a * b) - mu_a * mu_b
    num = (2 * mu_a * mu_b + C1) * (2 * s_ab + C2)
    den = (mu_a * mu_a + mu_b * mu_b + C1) * (s_aa + s_bb + C2)
    return float(np.mean(num / den))


def video_props(cap):
    return {"width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), "fps": round(cap.get(cv2.CAP_PROP_FPS), 3),
            "frames_in_header": int(cap.get(cv2.CAP_PROP_FRAME_COUNT))}


def compare(ref_path, test_path, min_frame, min_mean, max_frames=None, log=print):
    ref, test = cv2.VideoCapture(ref_path), cv2.VideoCapture(test_path)
    if not ref.isOpened() or not test.isOpened():
        raise RuntimeError("could not open a video")
    pr, pt = video_props(ref), video_props(test)
    scores, identical, n = [], 0, 0
    t0 = time.perf_counter()
    while True:
        ok_r, fr = ref.read()
        ok_t, ft = test.read()
        if not ok_r and not ok_t:
            break
        if ok_r != ok_t:
            scores.append(None)                                        # one video ran out first: frame count differs
            break
        if fr.shape != ft.shape:
            scores.append(None)
            break
        if np.array_equal(fr, ft):
            identical += 1
            scores.append(1.0)
        else:
            scores.append(ssim_gray(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), cv2.cvtColor(ft, cv2.COLOR_BGR2GRAY)))
        n += 1
        if n % 500 == 0:
            log(f"  {n} frames, {time.perf_counter() - t0:.0f} s")
        if max_frames and n >= max_frames:
            break
    ref.release(), test.release()
    valid = [s for s in scores if s is not None]
    count_mismatch = any(s is None for s in scores)
    ranked = sorted(range(len(valid)), key=lambda i: valid[i])
    lowest = [{"frame": i, "ssim": round(valid[i], 5)} for i in ranked[:10]]
    mean = float(np.mean(valid)) if valid else 0.0
    mn = min(valid) if valid else 0.0
    same_props = all(pr[k] == pt[k] for k in ("width", "height", "fps"))
    out = {"reference": ref_path, "test": test_path, "ref_props": pr, "test_props": pt, "frames_compared": len(valid), "frame_count_mismatch": count_mismatch,
           "identical_frames": identical, "mean_ssim": round(mean, 6), "min_ssim": round(mn, 6), "min_frame": ranked[0] if ranked else None, "lowest_10": lowest,
           "frames_below_0_985": sum(1 for s in valid if s < 0.985), "frames_below_min": sum(1 for s in valid if s < min_frame),
           "near_gate": (min_frame <= mn < 0.985) or (min_mean <= mean < 0.993), "gate": {"min_frame": min_frame, "min_mean": min_mean},
           "pass": same_props and not count_mismatch and mn >= min_frame and mean >= min_mean and bool(valid), "seconds": round(time.perf_counter() - t0, 1),
           "per_frame": [round(s, 5) for s in valid]}
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("reference")
    ap.add_argument("test")
    ap.add_argument("--out", default=None)
    ap.add_argument("--min-frame", type=float, default=0.98)
    ap.add_argument("--min-mean", type=float, default=0.99)
    ap.add_argument("--max-frames", type=int, default=None)
    a = ap.parse_args()
    r = compare(a.reference, a.test, a.min_frame, a.min_mean, a.max_frames)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(r, f, indent=1)
    show = {k: v for k, v in r.items() if k != "per_frame"}
    print(json.dumps(show, indent=1))
    print(f"RESULT {'PASS' if r['pass'] else 'FAIL'}: frames {r['frames_compared']} (count mismatch {r['frame_count_mismatch']}), identical {r['identical_frames']}, "
          f"min SSIM {r['min_ssim']:.5f} at frame {r['min_frame']}, mean {r['mean_ssim']:.5f}, near gate {r['near_gate']}")
    sys.exit(0 if r["pass"] else 1)
