# Custom YOLO26 ImageNet-LOC Training Design

## Status

Approved by the user on 2026-09-08; revised for Modal execution on
2026-09-08.

## Goal

Provide a reproducible way to train the project's custom YOLO26 tabletop object
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
- Fine-tune the pretrained Ultralytics `yolo26n.pt` detector and export the best
  weights to `models/object_detection/object_detector.pt`.
- Add focused unit tests for annotation parsing and dataset conversion.
- Add a Modal training entry point that mounts a persistent Modal Volume,
  prepares the subset remotely, trains on an L4 GPU, and persists the model
  and metrics for download.
- Document the Modal Volume upload, training, and download commands.
- Document that ordinary ImageNet classification images are insufficient for
  grasping because they do not provide object locations; ImageNet-LOC or an
  equivalent box-annotated source is required.

## Out of scope

- Automatic downloading of ImageNet image data from ImageNet. The user must
  obtain the data under its terms and upload it to the Modal Volume.
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

For Modal, the input arguments point into the read-only source area of the
mounted volume and the output argument points into its writable output area.
The converter never writes under the source path.

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
file and remains directly consumable by Ultralytics YOLO26.

## Training contract

The existing `training/object_detection/train.py` remains the local training
entry point. Its default base model becomes `yolo26n.pt`, and the prepared YAML
is passed through `--data`. The model is custom because its detection head is
fine-tuned on the selected project classes; no separate model is trained per
object.

The Modal entry point will use a persistent volume named
`ros-yolo-training`, mounted at `/mnt/data`, and a GPU function with a long
training timeout. The function will read the source subset from
`/mnt/data/imagenet-loc`, write prepared data and run artifacts under
`/mnt/data/output`, and save the final weights at
`/mnt/data/output/models/object_detector.pt`.

The Modal recipe uses this flow:

```bash
python -m pip install modal
python -m modal setup
python -m modal volume create ros-yolo-training
python -m modal volume put ros-yolo-training ./imagenet-loc /imagenet-loc
python -m modal run training/object_detection/modal_train.py
python -m modal volume get ros-yolo-training /output/models/object_detector.pt ./models/object_detection/object_detector.pt
```

The user supplies `classes.tsv` inside the uploaded `imagenet-loc` directory.
The Modal job filters to those classes automatically, so unrelated ImageNet
classes are never copied into the training set.

## Verification

- Unit tests cover class-file parsing, XML parsing, image resolution, box
  normalization/clipping, invalid-box rejection, and deterministic splitting.
- A compile check runs without requiring Ultralytics or ImageNet data.
- Modal app import and dry-run argument validation are tested without starting
  a remote GPU job.
- A real training run remains cloud-, account-, and data-dependent and is
  excluded from the normal test suite.
