"""Draw the gaze -> detection -> selection flow on a workspace frame.

Pure OpenCV/NumPy so it is testable without ROS. Boxes are dicts with
``object_id, class_name, confidence, x, y, w, h`` (pixel centre and size).
"""

from __future__ import annotations

from typing import Any

GREEN = (0, 200, 0)
YELLOW = (0, 220, 255)
RED = (0, 0, 255)
WHITE = (255, 255, 255)


def describe_selection(selection: dict[str, Any]) -> str:
    """One readable block: what was picked and every coordinate we have."""
    lines = [
        f"SELECTED {selection['object_id']} ({selection['class_name']}) "
        f"via {selection['source']}, confidence {selection['confidence']:.2f}",
        f"  image px   : ({selection['image_x']:.0f}, {selection['image_y']:.0f})",
    ]
    if selection.get("workspace_xy") is not None:
        wx, wy = selection["workspace_xy"]
        lines.append(f"  workspace m: ({wx:.3f}, {wy:.3f})")
    else:
        lines.append("  workspace m: n/a (workspace not calibrated)")
    if selection.get("robot_xyz") is not None:
        rx, ry, rz = selection["robot_xyz"]
        lines.append(f"  robot base : ({rx:.3f}, {ry:.3f}, {rz:.3f}) m")
    else:
        lines.append("  robot base : n/a (no target pose; calibrate first)")
    return "\n".join(lines)


def render_overlay(
    cv2: Any,
    frame: Any,
    boxes: list[dict[str, Any]],
    gaze: tuple[float, float, bool] | None,
    selected_id: str | None,
) -> Any:
    """Return a copy of ``frame`` with boxes, the gaze dot and the target marked."""
    out = frame.copy()
    height, width = out.shape[:2]
    for box in boxes:
        chosen = box["object_id"] == selected_id
        color = GREEN if chosen else YELLOW
        x0 = int(box["x"] - box["w"] / 2)
        y0 = int(box["y"] - box["h"] / 2)
        x1 = int(box["x"] + box["w"] / 2)
        y1 = int(box["y"] + box["h"] / 2)
        cv2.rectangle(out, (x0, y0), (x1, y1), color, 6 if chosen else 3)
        label = f"{box['class_name']} {box['confidence']:.2f}"
        cv2.putText(
            out,
            label,
            (x0, max(24, y0 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.7, width / 1500),
            color,
            2,
        )
        cv2.circle(out, (int(box["x"]), int(box["y"])), 8, color, -1)
    if gaze is not None:
        gx, gy, valid = gaze
        color = RED if valid else (128, 128, 128)
        center = (
            int(min(max(gx, 0.0), 1.0) * width),
            int(min(max(gy, 0.0), 1.0) * height),
        )
        cv2.circle(out, center, 22, color, 4)
        cv2.circle(out, center, 4, color, -1)
    return out
