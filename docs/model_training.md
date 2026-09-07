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

The first model fuses selected left-eye, right-eye, and head landmarks into a
two-coordinate regression head. Add sessions with different seating and
lighting rather than assuming a fixed face pose. Evaluation reports normalized
mean error and physical tabletop error in centimetres.

## Object detection

Capture images from the actual fixed phone camera. Annotate the generated YOLO
text files with the classes used by the project; the class list is data, not
hardcoded source:

```bash
python3 training/object_detection/collect_data.py \
  --source http://PHONE:8080/video --output datasets/object_detection/raw \
  --frames 500
printf 'cup\nbottle\ncube\n' > datasets/object_detection/classes.txt
python3 training/object_detection/prepare_dataset.py \
  --input datasets/object_detection/raw \
  --output datasets/object_detection/yolo \
  --classes datasets/object_detection/classes.txt
python3 training/object_detection/train.py \
  --data datasets/object_detection/yolo/data.yaml \
  --output models/object_detection/object_detector.pt
python3 training/object_detection/evaluate.py \
  --model models/object_detection/object_detector.pt \
  --data datasets/object_detection/yolo/data.yaml
```

Install `ultralytics` for training/evaluation. The ROS detector accepts the
exported YOLO-family weights through `perception.yaml`; training is never run
inside a ROS callback.
