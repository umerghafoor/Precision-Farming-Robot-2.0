#!/usr/bin/env python3
"""Batch inference on images and videos.

Reads every image/video from ./input/ and writes annotated results to ./output/.

Usage:
    python infer.py [options]

Options:
    --model     Path to YOLO .pt file       (default: best.pt)
    --conf      Confidence threshold 0-1    (default: 0.5)
    --input     Input folder                (default: input)
    --output    Output folder               (default: output)
"""

import sys
import time
from pathlib import Path
import argparse

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"}
VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".m4v", ".wmv", ".flv"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Batch YOLO inference on images and videos")
    p.add_argument("--model",  default="yolo.pt",  help="YOLO model path")
    p.add_argument("--conf",   type=float, default=0.4, help="Confidence threshold")
    p.add_argument("--input",  default="input",    help="Input folder")
    p.add_argument("--output", default="output",   help="Output folder")
    return p.parse_args()


# ──────────────────────────────────────────────────────────────────────────────
# Terminal helpers
# ──────────────────────────────────────────────────────────────────────────────

def _bar(done: int, total: int, width: int = 30) -> str:
    filled = int(width * done / total) if total else 0
    return f"[{'█' * filled}{'░' * (width - filled)}] {done}/{total}"


def print_header(title: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


def print_result(name: str, status: str, detail: str = "") -> None:
    colour = "\033[92m" if "ok" in status.lower() else "\033[91m"
    reset  = "\033[0m"
    tag    = f"{colour}{status:>6}{reset}"
    extra  = f"  {detail}" if detail else ""
    print(f"  {tag}  {name}{extra}")


# ──────────────────────────────────────────────────────────────────────────────
# Image inference
# ──────────────────────────────────────────────────────────────────────────────

def process_image(model, src: Path, dst: Path, conf: float) -> tuple[bool, str]:
    try:
        import cv2
        results = model(str(src), conf=conf, verbose=False)
        if not results:
            return False, "no results"
        frame = results[0].plot()
        dst.parent.mkdir(parents=True, exist_ok=True)
        ok = cv2.imwrite(str(dst), frame)
        if not ok:
            return False, "imwrite failed"
        n = len(results[0].boxes) if results[0].boxes is not None else 0
        return True, f"{n} detection{'s' if n != 1 else ''}"
    except Exception as exc:
        return False, str(exc)


# ──────────────────────────────────────────────────────────────────────────────
# Video inference
# ──────────────────────────────────────────────────────────────────────────────

def process_video(model, src: Path, dst: Path, conf: float) -> tuple[bool, str]:
    try:
        import cv2
    except ImportError:
        return False, "opencv-python not installed"

    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        return False, "cannot open video"

    fps    = cap.get(cv2.CAP_PROP_FPS) or 25.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    dst.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out    = cv2.VideoWriter(str(dst), fourcc, fps, (width, height))

    frame_idx    = 0
    total_dets   = 0
    t_start      = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        results = model(frame, conf=conf, verbose=False)
        annotated = results[0].plot() if results else frame
        out.write(annotated)

        n_dets = len(results[0].boxes) if results and results[0].boxes is not None else 0
        total_dets += n_dets
        frame_idx  += 1

        # Live progress bar (overwrite same line)
        elapsed = time.time() - t_start
        fps_est = frame_idx / elapsed if elapsed > 0 else 0
        bar = _bar(frame_idx, total)
        print(f"\r    {bar}  {fps_est:.1f} fps", end="", flush=True)

    print()  # newline after progress bar
    cap.release()
    out.release()

    return True, f"{frame_idx} frames  {total_dets} total detections"


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main() -> int:
    args = parse_args()

    model_path  = Path(args.model)
    input_dir   = Path(args.input)
    output_dir  = Path(args.output)

    # ── validate paths ──────────────────────────────────────────────────────
    if not model_path.exists():
        print(f"Model not found: {model_path}", file=sys.stderr)
        return 1

    if not input_dir.exists():
        print(f"Input folder not found: {input_dir}", file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    # ── collect files ───────────────────────────────────────────────────────
    all_files = sorted(
        f for f in input_dir.iterdir()
        if f.is_file() and f.suffix.lower() in IMAGE_EXTS | VIDEO_EXTS
    )

    images = [f for f in all_files if f.suffix.lower() in IMAGE_EXTS]
    videos = [f for f in all_files if f.suffix.lower() in VIDEO_EXTS]

    if not all_files:
        print(f"No images or videos found in {input_dir}/")
        return 0

    print(f"\nFound {len(images)} image(s) and {len(videos)} video(s) in {input_dir}/")

    # ── load model ──────────────────────────────────────────────────────────
    try:
        from ultralytics import YOLO
    except ImportError:
        print("Missing dependency: ultralytics\nInstall with: pip install ultralytics", file=sys.stderr)
        return 1

    print(f"Loading model: {model_path}")
    model = YOLO(str(model_path))
    print(f"Model loaded  ({len(model.names)} class{'es' if len(model.names) != 1 else ''}): "
          f"{', '.join(model.names.values())}")

    ok_count = err_count = 0

    # ── images ──────────────────────────────────────────────────────────────
    if images:
        print_header(f"Images  ({len(images)})")
        for i, src in enumerate(images, 1):
            dst = output_dir / src.name
            success, detail = process_image(model, src, dst, args.conf)
            print_result(src.name, "ok" if success else "FAIL", detail)
            if success:
                ok_count += 1
            else:
                err_count += 1

    # ── videos ──────────────────────────────────────────────────────────────
    if videos:
        print_header(f"Videos  ({len(videos)})")
        for src in videos:
            # always write as .mp4
            dst = output_dir / (src.stem + ".mp4")
            print(f"  ▶ {src.name}  →  {dst.name}")
            success, detail = process_video(model, src, dst, args.conf)
            print_result(src.name, "ok" if success else "FAIL", detail)
            if success:
                ok_count += 1
            else:
                err_count += 1

    # ── summary ─────────────────────────────────────────────────────────────
    print(f"\n{'─' * 60}")
    print(f"  Done — {ok_count} succeeded, {err_count} failed")
    print(f"  Output: {output_dir.resolve()}")
    print(f"{'─' * 60}\n")

    return 0 if err_count == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
