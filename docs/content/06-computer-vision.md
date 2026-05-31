# Computer Vision — Weed & Marker Detection

The vision stack has three faces: a **ROS2 YOLO node** (live detection on the robot), a **ROS2 ArUco pipeline** (field-marker localisation), and **standalone scripts/notebooks** for training and offline inference.

---

## 1. YOLO detection node (`raspberry-pi/.../yolo_detection/yolo_detection_node.py`)

Runs Ultralytics YOLOv8 inference on the camera stream and publishes annotated frames + structured results.

### Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/camera/color_jpeg` (default) | `CompressedImage` (or `Image`) |
| Pub | `/camera/detection` | annotated image (same transport as input) |
| Pub | `/detections/results` | `std_msgs/String` (JSON) |
| Pub | `/detections/results/legacy` | optional duplicate of results |

The node **auto-negotiates** the input transport: it inspects the live topic types and subscribes as `CompressedImage` when the topic name contains `jpeg`/`compressed` or only a compressed type is advertised, otherwise as raw `Image`. It can also decode the project's custom `gray4;WxH` / `gray8;WxH` packed format.

### Parameters

| Parameter | Default |
|-----------|---------|
| `model_path` | `''` (auto-resolved) |
| `confidence_threshold` | 0.2 |
| `iou_threshold` | 0.45 |
| `input_size` | 640 |
| `camera_topic` | `/camera/color_jpeg` |
| `annotated_topic` | `/camera/detection` |
| `results_topic` | `/detections/results` |
| `enable_visualization` | true (debug logging) |
| `frame_timeout_sec` | 5.0 (watchdog warns if no frames) |

### Model resolution order

1. `model_path` parameter (if it exists on disk)
2. `YOLO_MODEL_PATH` environment variable
3. Search the CWD, the node's directory, and up to 8 parent dirs for `yolo8n.pt`, `best.pt`, then any `yolo*.pt`

### Results JSON

```json
{
  "detections": [
    { "class": "weed", "class_id": 0, "confidence": 0.87, "bbox": [x1, y1, x2, y2] }
  ],
  "timestamp": 1715000000
}
```

`bbox` is pixel coordinates `[x1, y1, x2, y2]`. Bounding-box colours are deterministic per class id. The class list in the project's intended use is `['weed', 'crop']` (the model's own `names` map takes precedence).

```bash
ros2 run yolo_detection yolo_detection_node \
  --ros-args -p model_path:=/path/to/best.pt -p confidence_threshold:=0.5
```

---

## 2. ArUco marker pipeline (`weed-detection-node/`)

A ROS2 package + standalone tooling for **field localisation** using ArUco markers (dictionary `DICT_4X4_50`).

| Node | Subscribes | Publishes | Role |
|------|-----------|-----------|------|
| `aruco_processor` | `camera/raw` | `camera/annotated`, `image/coordinates` (JSON) | Detect markers, report center/corners/ids per frame |
| `image_publisher` | — | `camera/raw` | Publish static/recorded images for testing |
| `image_viewer` / `pretty_viewer` | `camera/annotated` | — | Visualise annotated output |

`aruco_processor` also runs as a CLI tool:

```bash
python3 aruco_processor.py --image field.jpg --output annotated.jpg --show
```

Pre-generated ArUco test scenes live in `weed-detection-node/` and `test_nodes/camera_pub_raw/data/`, so the whole pipeline can be exercised offline without a physical camera. `generate_test_scenes.py` recreates them.

> A second, lighter ROS2 workspace `ros2_ws/src/weed_detection/` contains a `weed_node.py` package variant.

---

## 3. Standalone scripts (repo root)

| Script | Purpose |
|--------|---------|
| `detect_weed.py` | Classify a **single image** as weed / not-weed and save an annotated copy |
| `infer.py` | Batch / video inference helper |
| `train_weed_detection.py` | Train the YOLO model |
| `visualize_gt.py` | Overlay ground-truth labels on images for dataset sanity-checking |

### `detect_weed.py`

```bash
python3 detect_weed.py --image field.jpg --model yolo.pt --conf 0.4
```

Decision logic:
1. If any detected class is in `--weed-labels` (default `weed,weeds`) → positive.
2. Else if the model has exactly one class and there is any detection → positive.
3. Otherwise → negative (`not_<positive>`).

It prints the verdict to stdout and the annotated-image path to stderr. Useful keys: `--target-labels` (override the class set, e.g. `person`), `--positive-output` / `--negative-output`, `--annotated-output`.

### Models & artifacts

- `yolo.pt`, `best.pt` — trained weed-detection weights.
- `yolov8s.pt` — base YOLOv8-small checkpoint (for fine-tuning).
- `Weed_Detection_YOLO_FYP.ipynb`, `train_weed_detection.ipynb` — training notebooks.
- `training_curves.png`, `val_confusion_matrix_normalized.png`, `sample_labels.png`, `laser_targets_demo.png` — training/eval visualisations checked into the repo.

To retrain, open a notebook (or run `train_weed_detection.py`), point it at your dataset, and replace `yolo.pt` / `best.pt` with the new weights.
