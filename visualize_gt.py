#!/usr/bin/env python3
"""Visualize WE3DS ground-truth segmentation masks overlaid on RGB images.

Reads images and their paired segmentation labels, draws coloured overlays
with per-class legends, and writes results to an output folder.

Usage:
    python visualize_gt.py [options]

Options:
    --images      Path to WE3DS images folder          (default: WE3DS/images)
    --masks       Path to SegmentationLabel folder     (default: WE3DS/annotations/segmentation/SegmentationLabel)
    --colors      Path to class_colors.txt             (default: WE3DS/class_colors.txt)
    --names       Path to class_names.txt              (default: WE3DS/class_names.txt)
    --output      Output folder                        (default: output_gt)
    --alpha       Overlay opacity 0-1                  (default: 0.45)
    --files       Specific filenames to process        (e.g. img_01289.png)
    --all         Process all images (default: demo set of 5)
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def load_palette(colors_path: Path, names_path: Path):
    """Return list of (name, (B,G,R)) tuples, one per class index."""
    raw_colors = colors_path.read_text().strip().splitlines()
    raw_names  = names_path.read_text().strip().splitlines()
    palette = []
    for name, color_str in zip(raw_names, raw_colors):
        r, g, b = (int(x) for x in color_str.split(","))
        palette.append((name.strip(), (b, g, r)))   # OpenCV uses BGR
    return palette


def draw_bboxes(image: np.ndarray, mask: np.ndarray,
                palette, crop_ids: set, weed_ids: set) -> tuple[np.ndarray, list[int]]:
    """Draw per-class bounding boxes on a clean copy of the image."""
    canvas  = image.copy()
    present = sorted({int(c) for c in np.unique(mask) if c < len(palette)})

    for cls_id in present:
        if cls_id in (0, 1):        # skip void / soil
            continue

        name, _ = palette[cls_id]
        is_crop = cls_id in crop_ids
        is_weed = cls_id in weed_ids

        if not (is_crop or is_weed):
            continue

        # Bounding box from mask region
        ys, xs = np.where(mask == cls_id)
        if len(xs) == 0:
            continue
        x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())

        color = (0, 200, 0) if is_crop else (0, 60, 220)   # green=crop, red=weed
        tag   = "CROP" if is_crop else "WEED"
        label = f"{tag}: {name}"

        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

        # Label background
        (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ly = max(y1 - 4, th + 4)
        cv2.rectangle(canvas, (x1, ly - th - 4), (x1 + tw + 4, ly + bl - 2), color, -1)
        cv2.putText(canvas, label, (x1 + 2, ly - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    return canvas, present


def draw_legend(canvas: np.ndarray, present: list[int], palette,
                crop_ids: set, weed_ids: set) -> np.ndarray:
    """Draw a bottom legend strip showing which classes appear."""
    h, w = canvas.shape[:2]
    swatch = 18
    pad    = 6
    font   = cv2.FONT_HERSHEY_SIMPLEX
    scale  = 0.42
    thick  = 1

    # Filter out void/soil for the legend
    display = [c for c in present if c not in (0, 1)]
    if not display:
        return canvas

    legend_h = swatch + pad * 2
    strip    = np.zeros((legend_h, w, 3), dtype=np.uint8)

    x = pad
    for cls_id in display:
        name, bgr = palette[cls_id]
        label     = name

        # Colour-code label border: green=crop, red=weed, white=other
        if cls_id in crop_ids:
            border = (0, 200, 0)
            tag    = "[CROP]"
        elif cls_id in weed_ids:
            border = (0, 0, 220)
            tag    = "[WEED]"
        else:
            border = (200, 200, 200)
            tag    = ""

        text    = f"{tag} {label}" if tag else label
        (tw, _), _ = cv2.getTextSize(text, font, scale, thick)
        entry_w = swatch + 4 + tw + pad * 2

        if x + entry_w > w - pad:
            break

        # Swatch
        cv2.rectangle(strip, (x, pad), (x + swatch, pad + swatch), bgr, -1)
        cv2.rectangle(strip, (x, pad), (x + swatch, pad + swatch), border, 1)

        # Label text
        cv2.putText(strip, text, (x + swatch + 4, pad + swatch - 4),
                    font, scale, (230, 230, 230), thick, cv2.LINE_AA)
        x += entry_w

    return np.vstack([canvas, strip])


# ──────────────────────────────────────────────────────────────────────────────
# Core processing
# ──────────────────────────────────────────────────────────────────────────────

CROP_IDS = {2, 5, 6, 11, 14, 15, 18}   # broad bean, buckwheat, pea, corn, soybean, sunflower, sugar beet
WEED_IDS = {3, 4, 7, 8, 9, 10, 12, 13, 16, 17}  # corn spurry, amaranth, fingergrass, wild oat, etc.

# Default demo set – best mixed crop/weed images
DEMO_FILES = [
    "img_01289.png",  # sugar beet + corn cockle (most balanced 50/50)
    "img_01290.png",  # sugar beet + corn cockle (weed clearly separate)
    "img_01293.png",  # sugar beet + corn cockle
    "img_00902.png",  # sugar beet + cornflower
    "img_01864.png",  # soybean + milk thistle
]


def process_file(fname: str, img_dir: Path, mask_dir: Path,
                 palette, out_dir: Path) -> tuple[bool, str]:
    img_path  = img_dir  / fname
    mask_path = mask_dir / fname

    if not img_path.exists():
        return False, f"image not found: {img_path}"
    if not mask_path.exists():
        return False, f"mask not found: {mask_path}"

    image = cv2.imread(str(img_path))
    if image is None:
        return False, "could not read image"

    mask = np.array(Image.open(mask_path))

    annotated, present = draw_bboxes(image, mask, palette, CROP_IDS, WEED_IDS)
    result = draw_legend(annotated, present, palette, CROP_IDS, WEED_IDS)

    out_path = out_dir / fname
    cv2.imwrite(str(out_path), result)
    classes_str = ", ".join(
        f"{'CROP' if c in CROP_IDS else 'WEED' if c in WEED_IDS else 'BG'}:{palette[c][0]}"
        for c in present if c not in (0, 1)
    )
    return True, classes_str or "soil/void only"


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="WE3DS ground-truth overlay visualizer")
    p.add_argument("--images",  default="WE3DS/images")
    p.add_argument("--masks",   default="WE3DS/annotations/segmentation/SegmentationLabel")
    p.add_argument("--colors",  default="WE3DS/class_colors.txt")
    p.add_argument("--names",   default="WE3DS/class_names.txt")
    p.add_argument("--output",  default="output_gt")
    p.add_argument("--alpha",   type=float, default=0.45)
    p.add_argument("--files",   nargs="+", metavar="FILE", help="specific filenames to process")
    p.add_argument("--all",     action="store_true", help="process every image")
    return p.parse_args()


def main() -> int:
    args     = parse_args()
    img_dir  = Path(args.images)
    mask_dir = Path(args.masks)
    out_dir  = Path(args.output)

    for d, name in [(img_dir, "--images"), (mask_dir, "--masks"),
                    (Path(args.colors), "--colors"), (Path(args.names), "--names")]:
        if not d.exists():
            print(f"Path not found ({name}): {d}", file=sys.stderr)
            return 1

    palette = load_palette(Path(args.colors), Path(args.names))
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.files:
        files = args.files
    elif args.all:
        files = sorted(p.name for p in img_dir.iterdir() if p.suffix == ".png")
    else:
        files = DEMO_FILES

    print(f"\nProcessing {len(files)} image(s)  →  {out_dir}/")
    print(f"{'─' * 60}")

    ok = err = 0
    for fname in files:
        success, detail = process_file(fname, img_dir, mask_dir, palette, out_dir)
        status = "\033[92m  ok\033[0m" if success else "\033[91mFAIL\033[0m"
        print(f"  {status}  {fname}  —  {detail}")
        if success:
            ok += 1
        else:
            err += 1

    print(f"{'─' * 60}")
    print(f"  Done — {ok} succeeded, {err} failed")
    print(f"  Output: {out_dir.resolve()}\n")
    return 0 if err == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
