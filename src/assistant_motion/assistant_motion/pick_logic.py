"""Pure pick gating: a selected object class maps to one slot label at a time.

A class label is its index in ``pick_classes``, which the user edits (see
``config/pick.yaml``); the Arduino owns the pose of each label. No ROS here, so
the timing rules can be checked with plain Python.
"""

from __future__ import annotations


class PickGate:
    """Let one selected class through per pick.

    ``accept`` returns the label of a pickable class when no pick is running and
    the re-arm gap since the last pick has passed. ``release`` ends the pick, so
    a selection that is still held right after the arm finished is not picked
    again.
    """

    def __init__(self, pick_classes: list[str], rearm_gap_s: float) -> None:
        self._classes = list(pick_classes)
        self._rearm_gap_s = float(rearm_gap_s)
        self._busy = False
        self._ready_at = 0.0

    @property
    def busy(self) -> bool:
        """Whether a label has been issued and not released yet."""
        return self._busy

    @property
    def classes(self) -> list[str]:
        """The pick classes in label order."""
        return list(self._classes)

    def label_of(self, class_name: str) -> int | None:
        """Return the class label, or None when the class is not pickable."""
        try:
            return self._classes.index(class_name)
        except ValueError:
            return None

    def accept(self, now: float, class_name: str) -> int | None:
        """Return the label to send for a selected class, or None."""
        label = self.label_of(class_name)
        if label is None or self._busy or float(now) < self._ready_at:
            return None
        self._busy = True
        return label

    def release(self, now: float) -> None:
        """End the current pick after an arm reply or a command timeout."""
        self._busy = False
        self._ready_at = float(now) + self._rearm_gap_s
