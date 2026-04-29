"""Probe OpenCV camera indices on macOS and capture multiple frames per index.

Run from repo root with venv active:
    python scripts/diagnose_cameras.py

For each index, captures N frames over ~3 seconds, computes mean brightness,
and reports whether the feed is alive, dark-but-varying, or all-zero. Saves the
brightest frame to debug-inference/camera_probe/index_{N}.png.
"""

import os
import sys
import time
import cv2
import numpy as np

OUT_DIR = "debug_inference/camera_probe"
MAX_INDEX = 5
FRAMES_PER_INDEX = 30
SETTLE_DELAY_S = 0.05


def probe(index: int) -> dict:
    cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        return {"index": index, "opened": False}

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    frames_read = 0
    means: list[float] = []
    best_frame: np.ndarray | None = None
    best_mean = -1.0

    for _ in range(FRAMES_PER_INDEX):
        ok, frame = cap.read()
        if ok and frame is not None:
            frames_read += 1
            mean = float(frame.mean())
            means.append(mean)
            if mean > best_mean:
                best_mean = mean
                best_frame = frame
        time.sleep(SETTLE_DELAY_S)

    saved_path = None
    if best_frame is not None:
        os.makedirs(OUT_DIR, exist_ok=True)
        saved_path = os.path.join(OUT_DIR, f"index_{index}.png")
        cv2.imwrite(saved_path, best_frame)

    cap.release()

    if not means:
        verdict = "no frames"
    elif max(means) < 1.0:
        verdict = "ALL BLACK (sensor off / no light)"
    elif max(means) < 10.0:
        verdict = "very dark (likely standby/asleep)"
    elif max(means) - min(means) < 0.5:
        verdict = "static (constant frame, likely placeholder)"
    else:
        verdict = "ALIVE"

    return {
        "index": index,
        "opened": True,
        "width": width,
        "height": height,
        "fps": fps,
        "frames_read": frames_read,
        "mean_min": min(means) if means else None,
        "mean_max": max(means) if means else None,
        "verdict": verdict,
        "saved": saved_path,
    }


def main() -> int:
    print(
        f"Probing cv2.VideoCapture indices 0..{MAX_INDEX - 1} via AVFoundation\n"
        f"Reading {FRAMES_PER_INDEX} frames per index (~{FRAMES_PER_INDEX * SETTLE_DELAY_S:.1f}s each)\n"
    )
    results = [probe(i) for i in range(MAX_INDEX)]

    header = f"{'idx':<4}{'opened':<8}{'res':<14}{'fps':<6}{'frames':<8}{'mean_min':<10}{'mean_max':<10}{'verdict':<35}saved"
    print(header)
    print("-" * len(header))
    for r in results:
        if not r["opened"]:
            print(f"{r['index']:<4}{'no':<8}")
            continue
        res = f"{r['width']}x{r['height']}"
        fps = f"{r['fps']:.1f}"
        mn = f"{r['mean_min']:.1f}" if r['mean_min'] is not None else "-"
        mx = f"{r['mean_max']:.1f}" if r['mean_max'] is not None else "-"
        saved = r["saved"] or ""
        print(
            f"{r['index']:<4}{'yes':<8}{res:<14}{fps:<6}{r['frames_read']:<8}"
            f"{mn:<10}{mx:<10}{r['verdict']:<35}{saved}"
        )

    print(
        f"\nALIVE = real video. ALL BLACK = camera open but sensor off.\n"
        f"Open the PNGs under {OUT_DIR}/ to identify which index is the C925e."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
