from pathlib import Path

from training.object_detection.kaggle_subset import select_kaggle_images


def test_select_kaggle_images_uses_selected_synsets_and_cap(tmp_path: Path) -> None:
    annotations = tmp_path / "annotations"
    for synset in ("n1", "n2", "ignored"):
        directory = annotations / synset
        directory.mkdir(parents=True)
        for index in range(3):
            (directory / f"{synset}_{index}.xml").write_text("<annotation/>", encoding="utf-8")
    classes = tmp_path / "classes.tsv"
    classes.write_text("n1\tcup\nn2\tbottle\n", encoding="utf-8")

    selected = select_kaggle_images(annotations, classes, max_images_per_class=2, seed=7)

    assert len(selected) == 4
    assert {item.synset for item in selected} == {"n1", "n2"}
    assert all(
        item.relative_image.parts[:4] == ("ILSVRC", "Data", "CLS-LOC", "train")
        for item in selected
    )
    assert all(item.relative_image.suffix == ".JPEG" for item in selected)
