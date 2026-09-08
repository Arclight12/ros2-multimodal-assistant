from __future__ import annotations

from pathlib import Path

import pytest

from training.object_detection.imagenet_subset import (
    Annotation,
    Box,
    ClassMapping,
    load_class_mapping,
    parse_annotation,
    prepare_dataset,
    to_yolo_line,
)


def test_class_mapping_preserves_file_order(tmp_path: Path) -> None:
    path = tmp_path / "classes.tsv"
    path.write_text("n1\tcup\nn2\tbottle\n", encoding="utf-8")

    mapping = load_class_mapping(path)

    assert mapping.synset_to_index == {"n1": 0, "n2": 1}
    assert mapping.names == ("cup", "bottle")


def test_class_mapping_rejects_duplicates(tmp_path: Path) -> None:
    path = tmp_path / "classes.tsv"
    path.write_text("n1\tcup\nn1\tbottle\n", encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate synset"):
        load_class_mapping(path)


def test_parse_annotation_filters_unselected_objects(tmp_path: Path) -> None:
    xml = tmp_path / "item.xml"
    xml.write_text(
        """<annotation><filename>item.JPEG</filename><size><width>100</width><height>80</height></size>
        <object><name>n1</name><bndbox><xmin>10</xmin><ymin>20</ymin><xmax>60</xmax><ymax>70</ymax></bndbox></object>
        <object><name>n9</name><bndbox><xmin>1</xmin><ymin>1</ymin><xmax>2</xmax><ymax>2</ymax></bndbox></object>
        </annotation>""",
        encoding="utf-8",
    )

    annotation = parse_annotation(xml, ClassMapping({"n1": 0}, ("cup",)))

    assert annotation == Annotation(
        image_name="item.JPEG",
        width=100,
        height=80,
        boxes=(Box(0, 10.0, 20.0, 60.0, 70.0),),
    )


def test_to_yolo_line_clips_and_normalizes_box() -> None:
    line = to_yolo_line(Box(2, -10.0, 10.0, 110.0, 50.0), 100, 80)

    assert line == "2 0.500000 0.375000 1.000000 0.500000"


def test_to_yolo_line_rejects_empty_box() -> None:
    assert to_yolo_line(Box(0, 10.0, 10.0, 10.0, 20.0), 100, 80) is None


def test_prepare_dataset_filters_classes_and_writes_yolo_layout(
    tmp_path: Path,
) -> None:
    images = tmp_path / "images" / "n1"
    annotations = tmp_path / "annotations" / "n1"
    images.mkdir(parents=True)
    annotations.mkdir(parents=True)
    (images / "one.JPEG").write_bytes(b"image")
    (annotations / "one.xml").write_text(
        """<annotation><filename>one.JPEG</filename><size><width>100</width><height>80</height></size>
        <object><name>n1</name><bndbox><xmin>10</xmin><ymin>20</ymin><xmax>60</xmax><ymax>70</ymax></bndbox></object>
        <object><name>n9</name><bndbox><xmin>1</xmin><ymin>1</ymin><xmax>2</xmax><ymax>2</ymax></bndbox></object>
        </annotation>""",
        encoding="utf-8",
    )
    classes = tmp_path / "classes.tsv"
    classes.write_text("n1\tcup\n", encoding="utf-8")

    summary = prepare_dataset(
        tmp_path / "images",
        tmp_path / "annotations",
        classes,
        tmp_path / "yolo",
        0.5,
        1,
        7,
        False,
    )

    assert summary.image_count == 1
    assert (tmp_path / "yolo" / "data.yaml").is_file()
    labels = list((tmp_path / "yolo" / "labels").rglob("*.txt"))
    assert len(labels) == 1
    assert labels[0].read_text(encoding="utf-8").strip().startswith("0 ")


def test_prepare_dataset_requires_force_for_nonempty_output(tmp_path: Path) -> None:
    output = tmp_path / "yolo"
    output.mkdir()
    (output / "old.txt").write_text("old", encoding="utf-8")
    classes = tmp_path / "classes.tsv"
    classes.write_text("n1\tcup\n", encoding="utf-8")

    with pytest.raises(FileExistsError, match="--force"):
        prepare_dataset(
            tmp_path / "images",
            tmp_path / "annotations",
            classes,
            output,
            0.2,
            0,
            7,
            False,
        )
