#!/usr/bin/env python3
"""Simple CLI tool: detect if an input image contains weed.

Usage:
    python detect_weed.py --image /path/to/image.jpg
"""

from pathlib import Path
import argparse
import shutil
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect weed in a single image")
    parser.add_argument("--image", required=True, help="Path to input image")
    parser.add_argument(
        "--model",
        default="yolo.pt",
        help="Path to YOLO model (.pt). Default: yolo.pt",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.4,
        help="Confidence threshold (0.0-1.0). Default: 0.4",
    )
    parser.add_argument(
        "--weed-labels",
        default="weed,weeds",
        help="Comma-separated class names treated as weed. Default: weed,weeds",
    )
    parser.add_argument(
        "--target-labels",
        default=None,
        help=(
            "Comma-separated class names to detect (overrides --weed-labels). "
            "Example: person"
        ),
    )
    parser.add_argument(
        "--positive-output",
        default=None,
        help=(
            "Output label when target is detected. "
            "Default: inferred from --target-labels (or weed)"
        ),
    )
    parser.add_argument(
        "--negative-output",
        default=None,
        help=(
            "Output label when target is not detected. "
            "Default: inferred as not_<positive_output>"
        ),
    )
    parser.add_argument(
        "--annotated-output",
        default=None,
        help=(
            "Path to save annotated image. "
            "Default: <input_stem>_annotated<input_suffix>"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    image_path = Path(args.image)
    model_path = Path(args.model)

    if not image_path.exists() or not image_path.is_file():
        print(f"Image not found: {image_path}", file=sys.stderr)
        return 1

    if not model_path.exists() or not model_path.is_file():
        print(f"Model not found: {model_path}", file=sys.stderr)
        return 1

    if args.annotated_output is not None:
        annotated_output = Path(args.annotated_output)
    else:
        output_suffix = image_path.suffix if image_path.suffix else ".jpg"
        annotated_output = image_path.with_name(f"{image_path.stem}_annotated{output_suffix}")

    try:
        from ultralytics import YOLO
    except ImportError:
        print("Missing dependency: ultralytics", file=sys.stderr)
        print("Install with: pip install ultralytics", file=sys.stderr)
        return 1

    labels_arg = args.target_labels if args.target_labels is not None else args.weed_labels
    parsed_labels = [label.strip().lower() for label in labels_arg.split(",") if label.strip()]
    target_labels = set(parsed_labels)

    if args.positive_output is not None:
        positive_output = args.positive_output
    elif args.target_labels is not None and parsed_labels:
        positive_output = parsed_labels[0]
    else:
        positive_output = "weed"

    negative_output = args.negative_output or f"not_{positive_output}"

    model = YOLO(str(model_path))
    results = model(str(image_path), conf=args.conf, verbose=False)

    try:
        annotated_output.parent.mkdir(parents=True, exist_ok=True)
        if results:
            annotated_frame = results[0].plot()
            from cv2 import imwrite

            saved_ok = imwrite(str(annotated_output), annotated_frame)
            if not saved_ok:
                print(f"Failed to save annotated image: {annotated_output}", file=sys.stderr)
                return 1
        else:
            shutil.copy2(image_path, annotated_output)
    except Exception as exc:
        print(f"Failed to create annotated image: {exc}", file=sys.stderr)
        return 1

    detected_classes = []
    for result in results:
        if result.boxes is None:
            continue
        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = str(model.names.get(class_id, class_id)).lower()
            detected_classes.append(class_name)

    # Simple rule:
    # 1) If any detection label matches target labels -> positive output
    # 2) If model has exactly one class and there is any detection -> positive output
    # 3) Otherwise -> negative output
    has_target_label = any(cls in target_labels for cls in detected_classes)
    single_class_model = isinstance(model.names, dict) and len(model.names) == 1
    has_any_detection = len(detected_classes) > 0

    is_target_detected = has_target_label or (single_class_model and has_any_detection)

    print(positive_output if is_target_detected else negative_output)
    print(f"Annotated image: {annotated_output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
