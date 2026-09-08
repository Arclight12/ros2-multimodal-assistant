# Custom YOLOv8 ImageNet-LOC Training Design

## Status

Approved by the user on 2026-09-08; revised for Kaggle execution on
2026-09-08.

## Goal

Provide a reproducible way to train the project's custom YOLOv8 tabletop object
detector from a small, configured subset of ImageNet-LOC, while keeping the
existing custom gaze model separate and compatible with the ROS runtime.

## Scope

- Add an ImageNet-LOC XML-to-YOLO dataset converter.
- Select classes through a user-owned tab-separated mapping file containing
  ImageNet synset IDs and the class names exposed to YOLO.
- Create deterministic train/validation splits with a configurable per-class
  image limit.
- Validate image references, annotation dimensions, class membership, and
  bounding-box coordinates before writing labels.
- Keep training on the existing Ultralytics YOLOv8 path and export the best
  weights to `models/object_detection/object_detector.pt`.
- Add focused unit tests for annotation parsing and dataset conversion.
- Make every input and output path a CLI argument so the same commands work
  with Kaggle's read-only `/kaggle/input` datasets and writable
  `/kaggle/working` outputs.
- Document a Kaggle notebook flow that installs dependencies, prepares the
  subset, trains YOLOv8 on GPU when enabled, evaluates it, and exposes the
  exported weights for download.
- Document that ordinary ImageNet classification images are insufficient for
  grasping because they do not provide object locations; ImageNet-LOC or an
  equivalent box-annotated source is required.

## Out of scope

- Automatic ImageNet downloading or credential handling.
- A mandatory Kaggle notebook file; notebook cells in the training guide are
  sufficient and avoid duplicating the CLI implementation.
- Training a gaze model from ImageNet. Gaze remains trained from webcam grid
  samples using the existing MediaPipe-landmark feature pipeline.
- Segmentation, monocular depth, 6D pose estimation, or grasp-pose learning.
- Changes to ROS message contracts or the object detector runtime node.

## Data contract

The converter accepts an extracted ImageNet-LOC-style tree:

- `--images-root`: directory containing JPEG images.
- `--annotations-root`: directory containing XML files with ImageNet object
  annotations.
- `--classes`: UTF-8 text file, one `synset_id<TAB>class_name` per line.
- `--output`: YOLO dataset directory to create.

For Kaggle, the input arguments point into `/kaggle/input/<dataset-name>/` and
the output argument points into `/kaggle/working/`. The converter never writes
under an input path.

Each selected annotation is resolved to an image by the XML filename, then by
the XML path with its extension changed to `.JPEG`, `.jpg`, or `.png`. The
converter writes one YOLO label per source image and includes all valid
selected objects in that image. Coordinates are normalized to `[0, 1]` and
clipped only when a box extends slightly beyond the image edge. Images with no
selected valid boxes are skipped rather than emitted as negatives.

The output contains:

```text
output/
  images/train/
  images/val/
  labels/train/
  labels/val/
  data.yaml
```

The generated YAML uses zero-based class indices in the order of the mapping
file and remains directly consumable by Ultralytics YOLOv8.

## Training contract

The existing `training/object_detection/train.py` remains the only training
entry point. Its default base model stays `yolov8n.pt`, and the prepared YAML
is passed through `--data`. The model is custom because its detection head is
fine-tuned on the selected project classes; no separate model is trained per
object.

The training CLI will expose Ultralytics' Kaggle-relevant controls as options:
`--project`, `--name`, `--device`, `--workers`, and `--exist-ok`. Defaults stay
local-friendly, while the documented Kaggle command writes runs and exported
weights below `/kaggle/working/`.

The Kaggle recipe uses this flow:

```bash
pip install -q ultralytics opencv-contrib-python pyyaml
python training/object_detection/prepare_imagenet_subset.py \
  --images-root /kaggle/input/imagenet-loc-subset/images \
  --annotations-root /kaggle/input/imagenet-loc-subset/annotations \
  --classes /kaggle/input/imagenet-loc-subset/classes.tsv \
  --output /kaggle/working/yolo
python training/object_detection/train.py \
  --data /kaggle/working/yolo/data.yaml \
  --base-model yolov8n.pt \
  --device 0 \
  --project /kaggle/working/runs \
  --output /kaggle/working/models/object_detector.pt
```

If the Kaggle notebook has no GPU, `--device cpu` remains supported but will
be slower. The user downloads `/kaggle/working/models/object_detector.pt` and
configures that file as `object_model_path` in the ROS workspace.

## Verification

- Unit tests cover class-file parsing, XML parsing, image resolution, box
  normalization/clipping, invalid-box rejection, and deterministic splitting.
- A compile check runs without requiring Ultralytics or ImageNet data.
- A real training smoke test is documented but remains hardware/data dependent
  and is excluded from the normal test suite.
