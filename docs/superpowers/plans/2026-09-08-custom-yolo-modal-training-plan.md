# Custom YOLO26 Modal Training Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert an ImageNet-LOC subset into YOLO labels, fine-tune pretrained `yolo26n.pt` on Modal's L40S GPU, and persist the trained detector for the ROS runtime.

**Architecture:** A pure Python converter parses ImageNet XML annotations, filters the synsets listed in `classes.tsv`, copies only selected images, and writes a deterministic YOLO dataset. A Modal app mounts one persistent Volume, runs the converter and existing training entry point remotely, commits the output, and exposes the resulting weights through `modal volume get`.

**Tech Stack:** Python 3.11, stdlib XML/path utilities, PyYAML, Ultralytics YOLO26, Modal App/Function/Image/Volume, pytest.

**Spec:** `docs/superpowers/specs/2026-09-08-custom-yolo-modal-training-design.md`

## Global Constraints

- Fine-tune the pretrained Ultralytics `yolo26n.pt` detector and export the best weights to `models/object_detection/object_detector.pt`.
- The user must obtain ImageNet image data under its terms and upload it to the Modal Volume.
- The Modal entry point uses a persistent volume named `ros-yolo-training`, mounted at `/mnt/data`, with input under `/mnt/data/imagenet-loc` and output under `/mnt/data/output`.
- Modal training uses an L40S GPU and a long timeout; normal tests never start a remote GPU job.
- Unrelated ImageNet classes are not copied into the prepared dataset.
- No ROS runtime interfaces or motion logic change.

---

### Task 1: Build the ImageNet-LOC to YOLO converter

**Files:**
- Create: `training/object_detection/imagenet_subset.py`
- Create: `training/object_detection/prepare_imagenet_subset.py`
- Create: `tests/test_imagenet_subset.py`

**Interfaces:**
- `load_class_mapping(path: Path) -> ClassMapping` reads one `synset_id<TAB>class_name` per line and assigns class IDs in file order.
- `parse_annotation(path: Path, mapping: ClassMapping) -> Annotation` parses ImageNet XML and keeps only selected objects.
- `to_yolo_line(box: Box, width: int, height: int) -> str | None` clips valid coordinates and returns `class_id center_x center_y width height` or `None` for an unusable box.
- `prepare_dataset(images_root: Path, annotations_root: Path, classes_path: Path, output: Path, validation_fraction: float, max_images_per_class: int, seed: int, force: bool) -> PreparationSummary` writes `images/{train,val}`, `labels/{train,val}`, and `data.yaml`.

- [ ] **Step 1: Write failing tests for mapping and XML parsing**

```python
def test_class_mapping_preserves_file_order(tmp_path):
    path = tmp_path / "classes.tsv"
    path.write_text("n1\\tcup\\nn2\\tbottle\\n", encoding="utf-8")
    mapping = load_class_mapping(path)
    assert mapping.synset_to_index == {"n1": 0, "n2": 1}
    assert mapping.names == ("cup", "bottle")


def test_parse_annotation_filters_unselected_objects(tmp_path):
    xml = tmp_path / "item.xml"
    xml.write_text(
        """<annotation><filename>item.JPEG</filename><size><width>100</width><height>80</height></size>
        <object><name>n1</name><bndbox><xmin>10</xmin><ymin>20</ymin><xmax>60</xmax><ymax>70</ymax></bndbox></object>
        <object><name>n9</name><bndbox><xmin>1</xmin><ymin>1</ymin><xmax>2</xmax><ymax>2</ymax></bndbox></object>
        </annotation>""",
        encoding="utf-8",
    )
    annotation = parse_annotation(xml, ClassMapping({"n1": 0}, ("cup",)))
    assert annotation.image_name == "item.JPEG"
    assert len(annotation.boxes) == 1
    assert annotation.boxes[0].class_id == 0
```

- [ ] **Step 2: Run the focused tests and verify they fail because the converter module is missing**

Run: `python -m pytest tests/test_imagenet_subset.py -q`

Expected: collection failure reporting missing `training.object_detection.imagenet_subset`.

- [ ] **Step 3: Implement the dataclasses, mapping parser, XML parser, and YOLO normalization**

Use `xml.etree.ElementTree`, `pathlib`, and `dataclasses`. Reject duplicate synsets or names, non-positive image dimensions, non-finite coordinates, and boxes whose clipped right/bottom edge is not greater than its left/top edge. Ignore unselected XML objects.

- [ ] **Step 4: Add failing tests for image resolution, per-class limiting, and deterministic split output**

```python
def test_prepare_dataset_filters_classes_and_writes_yolo_layout(tmp_path):
    images = tmp_path / "images" / "n1"
    annotations = tmp_path / "annotations" / "n1"
    images.mkdir(parents=True)
    annotations.mkdir(parents=True)
    (images / "one.JPEG").write_bytes(b"image")
    (annotations / "one.xml").write_text(XML_WITH_ONE_SELECTED_BOX, encoding="utf-8")
    classes = tmp_path / "classes.tsv"
    classes.write_text("n1\\tcup\\n", encoding="utf-8")

    summary = prepare_dataset(
        tmp_path / "images", tmp_path / "annotations", classes,
        tmp_path / "yolo", 0.5, 1, 7, False,
    )

    assert summary.image_count == 1
    assert (tmp_path / "yolo" / "data.yaml").is_file()
    assert list((tmp_path / "yolo" / "labels").rglob("*.txt"))
```

- [ ] **Step 5: Run the new tests and verify the dataset behavior is still missing**

Run: `python -m pytest tests/test_imagenet_subset.py -q`

Expected: failures for `prepare_dataset` or its missing output behavior.

- [ ] **Step 6: Implement deterministic selection, copying, labels, and `data.yaml`**

Resolve images from the XML filename, matching relative paths and common `.JPEG`, `.jpg`, and `.png` extensions. Shuffle with `random.Random(seed)`, enforce `max_images_per_class` when it is positive, split with the configured validation fraction, write unique sequential filenames to avoid basename collisions, and require `--force` before clearing a non-empty output directory.

- [ ] **Step 7: Add the CLI and run the focused tests**

Expose `--images-root`, `--annotations-root`, `--classes`, `--output`, `--validation-fraction`, `--max-images-per-class`, `--seed`, and `--force`. Run `python -m pytest tests/test_imagenet_subset.py -q`; expected result: PASS.

- [ ] **Step 8: Commit the converter slice**

```bash
git add training/object_detection/imagenet_subset.py training/object_detection/prepare_imagenet_subset.py tests/test_imagenet_subset.py
git commit -m "feat: convert imagenet boxes to yolo data"
```

### Task 2: Make local YOLO training configurable for YOLO26n

**Files:**
- Modify: `training/object_detection/train.py`
- Create: `tests/test_train_cli.py`

**Interfaces:**
- `build_parser() -> argparse.ArgumentParser` exposes the training options.
- `train_detector(args: argparse.Namespace) -> Path` trains the selected Ultralytics model and returns the copied best-weight path.

- [ ] **Step 1: Write a failing parser test**

```python
def test_training_defaults_to_yolo26_nano():
    args = build_parser().parse_args(["--data", "data.yaml"])
    assert args.base_model == "yolo26n.pt"
    assert args.device is None
    assert args.workers == 2
```

- [ ] **Step 2: Run the test and verify the existing default is `yolov8n.pt`**

Run: `python -m pytest tests/test_train_cli.py::test_training_defaults_to_yolo26_nano -q`

Expected: FAIL because the current default is `yolov8n.pt` and no parser function is exported.

- [ ] **Step 3: Implement the parser and training function**

Keep the existing Ultralytics API, change the default to `yolo26n.pt`, and add `--batch-size`, `--project`, `--name`, `--device`, `--workers`, `--patience`, and `--exist-ok`. Pass them to `model.train`; pass `None` for an unspecified device so local Ultralytics auto-selects. Continue copying `best.pt` to the explicit `--output` path.

- [ ] **Step 4: Run the focused tests and compile check**

Run: `python -m pytest tests/test_train_cli.py -q` and `python -m py_compile training/object_detection/train.py`.

Expected: PASS with no Ultralytics import required during parser tests.

- [ ] **Step 5: Commit the training CLI slice**

```bash
git add training/object_detection/train.py tests/test_train_cli.py
git commit -m "feat: configure yolo26 nano training"
```

### Task 3: Add the Modal L40S training app

**Files:**
- Create: `training/object_detection/modal_train.py`
- Modify: `requirements.txt`
- Create: `tests/test_modal_train.py`

**Interfaces:**
- `validate_modal_paths(images_root: Path, annotations_root: Path, classes_path: Path) -> None` checks the mounted source before starting a GPU job.
- `train_remote(...) -> str` is the Modal Function that prepares data, fine-tunes YOLO26n, commits the Volume, and returns the persisted weights path.
- `main(...)` is the Modal local entrypoint used by `modal run`.

- [ ] **Step 1: Write failing tests for path validation and default paths**

```python
def test_modal_defaults_use_l40s_paths():
    assert DEFAULT_MODEL == "yolo26n.pt"
    assert DEFAULT_VOLUME_NAME == "ros-yolo-training"
    assert DEFAULT_GPU == "L40S"


def test_validate_modal_paths_rejects_missing_classes(tmp_path):
    with pytest.raises(FileNotFoundError):
        validate_modal_paths(tmp_path / "images", tmp_path / "annotations", tmp_path / "classes.tsv")
```

- [ ] **Step 2: Run the tests and verify the Modal module is missing**

Run: `python -m pytest tests/test_modal_train.py -q`

Expected: collection failure because `modal_train.py` does not exist.

- [ ] **Step 3: Implement the Modal app using the current SDK**

Define an optional local import of `modal` so unit tests can run without the package. When Modal is available, create `modal.App`, a `modal.Volume.from_name("ros-yolo-training", create_if_missing=True)`, and an Image based on `modal.Image.debian_slim(python_version="3.11")` with `ultralytics`, `opencv-python-headless`, and `pyyaml` installed. Add the local object-detection directory with `Image.add_local_dir`.

Decorate `train_remote` with `gpu="L40S"`, `timeout=86400`, and `volumes={"/mnt/data": volume}`. Inside the function import the copied converter and training modules, prepare `/mnt/data/output/yolo`, train with `device="0"`, save `/mnt/data/output/models/object_detector.pt`, call `volume.commit()`, and return the path. Register a typed `@app.local_entrypoint()` with defaults for the mounted paths, model, epochs, batch size, image size, and per-class limit so `modal run training/object_detection/modal_train.py --help` exposes the controls.

- [ ] **Step 4: Add the Modal dependency and run local tests**

Add `modal` under the training-only dependencies. Run `python -m pytest tests/test_modal_train.py -q` and `python -m py_compile training/object_detection/modal_train.py`; expected result: PASS and compile success without starting a remote function.

- [ ] **Step 5: Commit the Modal slice**

```bash
git add training/object_detection/modal_train.py requirements.txt tests/test_modal_train.py
git commit -m "feat: train yolo26 on modal l40s"
```

### Task 4: Replace Kaggle documentation with the Modal runbook

**Files:**
- Modify: `docs/model_training.md`
- Modify: `requirements.txt`

**Interfaces:**
- The documented commands upload the user-provided ImageNet-LOC subset to `ros-yolo-training`, invoke the Modal app on L40S, and download `object_detector.pt` into the ROS `models/object_detection` directory.

- [ ] **Step 1: Replace the object-detection section**

Document the required `imagenet-loc/` layout, `classes.tsv` format, ImageNet access limitation, Modal authentication via `modal setup`, volume upload, `modal run`, optional overrides, and `modal volume get`. Remove all Kaggle paths and commands.

- [ ] **Step 2: Add a local documentation consistency check**

Run: `rg -n "Kaggle|kaggle|yolov8n|YOLOv8" docs training requirements.txt README.md PROJECT_SPEC.md`

Expected: no matches in the changed training documentation or code; historical design references may be absent because the spec was renamed.

- [ ] **Step 3: Commit the runbook**

```bash
git add docs/model_training.md requirements.txt
git commit -m "docs: document modal yolo training"
```

### Task 5: Full verification and optional remote run

**Files:**
- No new files.

- [ ] **Step 1: Run the complete hardware-free test and compile checks**

Run:

```bash
python -m pytest -q
python -m compileall -q training
```

Expected: all tests pass and compilation exits successfully.

- [ ] **Step 2: Validate Modal CLI availability without starting training**

Run: `modal run training/object_detection/modal_train.py --help`

Expected: help output lists the local-entrypoint options, with no GPU allocation.

- [ ] **Step 3: Check Modal authentication and the training volume**

Run: `modal volume ls ros-yolo-training`.

If authentication is unavailable or the volume does not contain `imagenet-loc/images`, `imagenet-loc/annotations`, and `imagenet-loc/classes.tsv`, report the exact blocker instead of starting a doomed GPU job.

- [ ] **Step 4: Run the real L40S training job when the volume is ready**

Run: `modal run training/object_detection/modal_train.py --epochs 100 --batch-size 32 --image-size 640 --max-images-per-class 500`.

Expected: Modal provisions an L40S, filters the configured classes, trains `yolo26n.pt`, commits `/output/models/object_detector.pt`, and prints the persisted output path.

- [ ] **Step 5: Download and verify the exported weights**

Run:

```bash
modal volume get ros-yolo-training /output/models/object_detector.pt models/object_detection/object_detector.pt
python -c "from pathlib import Path; p=Path('models/object_detection/object_detector.pt'); assert p.is_file() and p.stat().st_size > 0; print(p)"
```

Expected: a non-empty weight file exists at the ROS runtime path. Do not commit it because model artifacts are ignored.
