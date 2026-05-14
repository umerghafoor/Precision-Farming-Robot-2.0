"""
WE3DS → YOLOv8s Weed Detection — Local GPU Training
RTX 3070 (8 GB VRAM)

Usage:
    python train_weed_detection.py              # full pipeline
    python train_weed_detection.py --skip-convert   # if labels already built
    python train_weed_detection.py --train-only     # same
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from tqdm import tqdm

# ── Paths ─────────────────────────────────────────────────────────────
SCRIPT_DIR  = Path(__file__).parent.resolve()
DATASET_DIR = SCRIPT_DIR / 'WE3DS'
YOLO_DIR    = SCRIPT_DIR / 'yolo_we3ds'
RUNS_DIR    = SCRIPT_DIR / 'runs'

# ── WE3DS class mapping ────────────────────────────────────────────────
# YOLO classes: 0 = crop, 1 = weed
CROP_INDICES   = {2, 6, 11, 14, 15, 18}  # broad_bean, pea, corn, soybean, sunflower, sugar_beet
IGNORE_INDICES = {0, 1}                   # void, soil
YOLO_CLASSES   = ['crop', 'weed']
MIN_AREA_PX    = 400   # ignore blobs smaller than ~20×20 px


def semantic_to_yolo(idx: int) -> int | None:
    if idx in IGNORE_INDICES:
        return None
    return 0 if idx in CROP_INDICES else 1


# ── Step 1: convert masks → YOLO labels ───────────────────────────────
def convert_dataset():
    mask_dir  = DATASET_DIR / 'annotations' / 'segmentation' / 'SegmentationLabel'
    image_dir = DATASET_DIR / 'images'

    with open(DATASET_DIR / 'train.txt') as f:
        train_ids = [l.strip() for l in f if l.strip()]
    with open(DATASET_DIR / 'test.txt') as f:
        val_ids   = [l.strip() for l in f if l.strip()]

    splits = [('train', train_ids), ('val', val_ids)]

    for split, _ in splits:
        (YOLO_DIR / 'images' / split).mkdir(parents=True, exist_ok=True)
        (YOLO_DIR / 'labels' / split).mkdir(parents=True, exist_ok=True)

    def mask_to_labels(mask_path: Path) -> list[str]:
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            return []
        h, w = mask.shape
        lines = []
        for cls_idx in np.unique(mask):
            yolo_cls = semantic_to_yolo(int(cls_idx))
            if yolo_cls is None:
                continue
            binary = (mask == cls_idx).astype(np.uint8)
            n, labels_im = cv2.connectedComponents(binary)
            for inst in range(1, n):
                pts = np.argwhere(labels_im == inst)
                if len(pts) < MIN_AREA_PX:
                    continue
                r0, c0 = pts.min(axis=0)
                r1, c1 = pts.max(axis=0)
                cx = ((c0 + c1) / 2) / w
                cy = ((r0 + r1) / 2) / h
                bw = (c1 - c0) / w
                bh = (r1 - r0) / h
                lines.append(f'{yolo_cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}')
        return lines

    print('\n── Converting semantic masks → YOLO labels ──────────────────')
    stats = {}
    for split, ids in splits:
        imgs_done = boxes_done = 0
        print(f'\n  {split} ({len(ids)} images)')
        for img_id in tqdm(ids, ncols=80):
            img_src  = image_dir / f'img_{img_id}.png'
            mask_src = mask_dir  / f'img_{img_id}.png'
            if not img_src.exists() or not mask_src.exists():
                continue

            img_dst = YOLO_DIR / 'images' / split / f'img_{img_id}.png'
            if not img_dst.exists():
                img_dst.symlink_to(img_src)

            label_lines = mask_to_labels(mask_src)
            lbl_dst = YOLO_DIR / 'labels' / split / f'img_{img_id}.txt'
            lbl_dst.write_text('\n'.join(label_lines))

            imgs_done  += 1
            boxes_done += len(label_lines)

        stats[split] = (imgs_done, boxes_done)
        print(f'  {split}: {imgs_done} images, {boxes_done} boxes '
              f'(avg {boxes_done/max(imgs_done,1):.1f}/img)')

    # Write dataset.yaml
    cfg = {
        'path' : str(YOLO_DIR),
        'train': 'images/train',
        'val'  : 'images/val',
        'nc'   : len(YOLO_CLASSES),
        'names': YOLO_CLASSES,
    }
    cfg_path = YOLO_DIR / 'dataset.yaml'
    cfg_path.write_text(yaml.dump(cfg, sort_keys=False))
    print(f'\n  Wrote {cfg_path}')
    return cfg_path


# ── Step 2: train ──────────────────────────────────────────────────────
def train(cfg_path: Path):
    from ultralytics import YOLO

    print('\n── Training YOLOv8s ─────────────────────────────────────────')
    print('  GPU   : RTX 3070 (8 GB)')
    print('  Model : yolov8s.pt')
    print('  imgsz : 640  |  batch: 8  |  epochs: 100')
    print('  Classes:', YOLO_CLASSES)
    print()

    model = YOLO('yolov8s.pt')
    model.train(
        data        = str(cfg_path),
        imgsz       = 640,
        epochs      = 100,
        batch       = 8,       # safe for 8 GB VRAM at 640px
        patience    = 20,
        device      = 0,
        project     = str(RUNS_DIR),
        name        = 'we3ds_yolov8s',
        exist_ok    = True,
        # Augmentation
        hsv_h       = 0.015,
        hsv_s       = 0.7,
        hsv_v       = 0.4,
        flipud      = 0.5,
        fliplr      = 0.5,
        mosaic      = 1.0,
        degrees     = 10,
        translate   = 0.1,
        scale       = 0.5,
        # Saving
        save_period = 10,
        workers     = 4,
        cache       = False,   # set 'ram' if you have 32+ GB RAM
    )
    best = RUNS_DIR / 'we3ds_yolov8s' / 'weights' / 'best.pt'
    print(f'\n  Best weights: {best}')
    return best


# ── Step 3: validate ───────────────────────────────────────────────────
def validate(cfg_path: Path, weights: Path):
    from ultralytics import YOLO

    print('\n── Validation ───────────────────────────────────────────────')
    model = YOLO(str(weights))
    m = model.val(data=str(cfg_path), imgsz=640, device=0)
    print(f'  mAP@0.5      : {m.box.map50:.4f}')
    print(f'  mAP@0.5:0.95 : {m.box.map:.4f}')
    print(f'  Precision    : {m.box.mp:.4f}')
    print(f'  Recall       : {m.box.mr:.4f}')


# ── Step 4: export to NCNN (for Raspberry Pi) ──────────────────────────
def export_ncnn(weights: Path):
    from ultralytics import YOLO

    print('\n── Exporting to NCNN (Raspberry Pi) ─────────────────────────')
    model = YOLO(str(weights))
    model.export(format='ncnn', imgsz=640)
    ncnn_dir = weights.parent / 'best_ncnn_model'
    print(f'  NCNN model: {ncnn_dir}')
    print('  Copy this folder to the Raspberry Pi weed-detection-node.')


# ── Entry point ────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--skip-convert', '--train-only', action='store_true',
                        help='Skip mask→label conversion (use existing yolo_we3ds/)')
    parser.add_argument('--no-export', action='store_true',
                        help='Skip NCNN export')
    args = parser.parse_args()

    cfg_path = YOLO_DIR / 'dataset.yaml'

    if not args.skip_convert:
        if not DATASET_DIR.exists():
            sys.exit(f'ERROR: dataset not found at {DATASET_DIR}')
        cfg_path = convert_dataset()
    else:
        if not cfg_path.exists():
            sys.exit(f'ERROR: {cfg_path} not found — run without --skip-convert first')
        print(f'Skipping conversion, using existing {cfg_path}')

    best = train(cfg_path)
    validate(cfg_path, best)

    if not args.no_export:
        export_ncnn(best)

    print('\n✓ Done.')
    print(f'  Weights : {best}')
    print(f'  Results : {RUNS_DIR / "we3ds_yolov8s"}')


if __name__ == '__main__':
    main()
