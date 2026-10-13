from assistant_interaction.selection_logic import (
    DetectionObservation,
    GazeObservation,
    SelectionConfig,
    TemporalSelector,
    nearest_valid_detection,
    select_by_class,
)


def detection(object_id, x, y, confidence=0.9):
    return DetectionObservation(
        object_id=object_id,
        class_name=object_id,
        confidence=confidence,
        image_x=x,
        image_y=y,
        image_width=1000,
        image_height=1000,
    )


def test_nearest_valid_detection_is_selected():
    result = nearest_valid_detection(
        GazeObservation(0.68, 0.34, 0.95),
        [detection("cup", 680, 340), detection("ball", 100, 100)],
        SelectionConfig(),
    )

    assert result is not None
    assert result.object_id == "cup"


def test_low_confidence_and_far_objects_are_rejected():
    config = SelectionConfig(
        minimum_gaze_confidence=0.8,
        minimum_detection_confidence=0.8,
        maximum_gaze_object_distance=0.1,
    )

    assert (
        nearest_valid_detection(
            GazeObservation(0.5, 0.5, 0.7), [detection("cup", 500, 500)], config
        )
        is None
    )
    assert (
        nearest_valid_detection(
            GazeObservation(0.5, 0.5, 0.9), [detection("cup", 500, 500, 0.7)], config
        )
        is None
    )
    assert (
        nearest_valid_detection(
            GazeObservation(0.5, 0.5, 0.9), [detection("cup", 900, 900)], config
        )
        is None
    )


def test_temporal_selector_waits_then_confirms_same_object():
    selector = TemporalSelector(hold_seconds=1.0)
    candidate = detection("cup", 500, 500)

    assert selector.update(candidate, now=0.0) is None
    assert selector.update(candidate, now=0.9) is None
    confirmed = selector.update(candidate, now=1.0)

    assert confirmed is not None
    assert confirmed.object_id == "cup"
    assert selector.update(candidate, now=2.0) is None


def test_temporal_selector_resets_when_target_changes():
    selector = TemporalSelector(hold_seconds=1.0)
    cup = detection("cup", 500, 500)
    ball = detection("ball", 500, 500)

    selector.update(cup, now=0.0)
    assert selector.update(ball, now=1.0) is None
    assert selector.update(ball, now=1.9) is None
    assert selector.update(ball, now=2.0).object_id == "ball"


def test_voice_class_picks_most_confident_valid_detection():
    cup_a = DetectionObservation("cup_0", "cup", 0.8, 1, 1, 10, 10)
    cup_b = DetectionObservation("cup_1", "cup", 0.95, 1, 1, 10, 10)
    weak = DetectionObservation("book_0", "book", 0.3, 1, 1, 10, 10)
    config = SelectionConfig()

    assert select_by_class("cup", [cup_a, cup_b, weak], config) is cup_b
    assert select_by_class("book", [weak], config) is None  # low confidence
    assert select_by_class("ball", [cup_a], config) is None  # not visible
