# Model training

Training is deliberately outside the ROS packages. Runtime nodes load exported
weights and degrade only the affected capability when a weight file is absent.

## Gaze

Collect a 5x5 grid from the laptop webcam. The collector waits for the user to
look at each displayed normalized target, captures several frames, and records
session, lighting, head-position, and timestamp metadata:

```bash
python3 training/gaze/collect_data.py --camera-index 0 \
  --output datasets/gaze/raw --grid-size 5 --frames-per-point 10 \
  --lighting office
python3 training/gaze/preprocess.py \
  --labels datasets/gaze/raw/SESSION/labels.jsonl \
  --output datasets/gaze/gaze.npz
python3 training/gaze/train.py --data datasets/gaze/gaze.npz \
  --output models/gaze/gaze_model.pt
python3 training/gaze/evaluate.py --data datasets/gaze/gaze.npz \
  --model models/gaze/gaze_model.pt --workspace-width-m 0.4 \
  --workspace-height-m 0.3
```

To train without collecting your own data, use the public MPIIFaceGaze dataset
(CC BY-NC-SA 4.0, non-commercial use). One command downloads it into a Modal
volume, extracts features, trains on a T4, and saves the model locally:

```bash
python -m modal run training/gaze/modal_train.py
```

Public data is laptop-screen gaze, so expect extra error on a tabletop; add a
short collected session to fine-tune for your geometry.

The model regresses a two-coordinate gaze point from head-relative face-mesh
landmarks and iris offsets (`assistant_interaction/gaze_features.py`, shared by
training and the gaze node). Inputs are standardized inside the exported model. Add sessions with different seating and
lighting rather than assuming a fixed face pose. Evaluation reports normalized
mean error and physical tabletop error in centimetres.

## Object detection

The detector is fine-tuned remotely with the recent Ultralytics YOLO11 nano
checkpoint (`yolo11n.pt`) on Modal's L40S GPU. Use ImageNet-LOC or another ImageNet
subset with Pascal VOC XML bounding boxes; classification-only images cannot
localize objects for grasping.

Create only this small manifest locally; the images do not need to be downloaded
to your computer:

```text
imagenet-loc/
  images/<synset-id>/*.JPEG
  annotations/<synset-id>/*.xml
  classes.tsv
```

`classes.tsv` selects the only classes that enter training. Use one
`synset-id<TAB>class-name` per line, using the synset IDs from your extracted
ImageNet directories and XML files:

```text
<cup-synset-id>	cup
<bottle-synset-id>	bottle
<ball-synset-id>	ball
```

The converter automatically ignores every other ImageNet class, limits the
number of images per class, converts XML boxes to YOLO labels, and creates a
deterministic train/validation split. The Modal job uses the `Kaggle_Secret`
secret to download only those matching JPEGs directly into the Modal volume.

Install and authenticate Modal:

```bash
python -m pip install modal
python -m modal setup
python -m modal volume create ros-yolo-training
```

The volume must contain `imagenet-loc/annotations/` and
`imagenet-loc/classes.tsv`. Upload the small ImageNet-LOC bounding-box archive
or selected XML files once; do not upload the 160 GB image archive. Ensure the
Modal secret named `Kaggle_Secret` contains `KAGGLE_KEY` (or
`KAGGLE_API_TOKEN`).

Run the remote fine-tuning job:

```bash
python -m modal run training/object_detection/modal_train.py \
  --epochs 100 --batch-size 32 --image-size 640 \
  --max-images-per-class 500
```

The job uses an L40S GPU, automatically downloads the pretrained `yolo11n.pt`
checkpoint inside the Modal container, prepares the selected classes, trains,
and saves:

```text
/output/models/object_detector.pt
/output/runs/yolo11n/results.csv
```

Download the trained detector into the ROS workspace:

```bash
python -m modal volume get ros-yolo-training \
  /output/models/object_detector.pt \
  models/object_detection/object_detector.pt
```

For a repeat run with an existing prepared output, add `--force`. The ROS
detector accepts the exported weights through `perception.yaml`; training is
never run inside a ROS callback. The older local capture/preparation scripts
remain useful for collecting phone-camera images for a later domain-specific
fine-tuning pass.
