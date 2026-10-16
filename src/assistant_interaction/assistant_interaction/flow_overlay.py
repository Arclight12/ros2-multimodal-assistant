"""Draw the gaze -> detection -> selection flow on a workspace frame.

Pure OpenCV/NumPy so it is testable without ROS. Boxes are dicts with
``object_id, class_name, confidence, x, y, w, h`` (pixel centre and size).
An optional ``selection`` adds a thick box plus coordinate banner for the pick.
"""

from __future__ import annotations

from typing import Any

GREEN = (0, 200, 0)
YELLOW = (0, 220, 255)
RED = (0, 0, 255)
MAGENTA = (255, 0, 255)


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


def _banner_lines(selection: dict[str, Any]) -> list[str]:
    """Four short lines: what was picked, image px, workspace m, robot m."""
    point = selection.get("workspace_xy")
    workspace = (
        f"workspace m ({point[0]:.3f}, {point[1]:.3f})"
        if point is not None
        else "workspace n/a"
    )
    p = selection.get("robot_xyz")
    robot = (
        f"robot m ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"
        if p is not None
        else "robot n/a"
    )
    return [
        f"{str(selection['source']).upper()} {selection['object_id']} "
        f"({selection['class_name']}) {selection['confidence']:.2f}",
        f"img px ({selection['image_x']:.0f}, {selection['image_y']:.0f})",
        workspace,
        robot,
    ]


def _draw_banner(
    cv2: Any,
    out: Any,
    lines: list[str],
    anchor: tuple[int, int],
    color: tuple[int, int, int],
    width: int,
    height: int,
) -> None:
    """Filled text block centred on ``anchor``: above it, or below if it clips."""
    scale = max(0.6, width / 1600)
    thickness = 2
    sizes = [
        cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0]
        for line in lines
    ]
    line_h = max(h for _, h in sizes) + 12
    block_w = max(w for w, _ in sizes) + 16
    block_h = line_h * len(lines) + 8
    x0 = int(min(max(anchor[0] - block_w // 2, 0), max(width - block_w, 0)))
    top = int(anchor[1] - block_h)
    if top < 0:
        top = int(anchor[1])
    top = int(min(max(top, 0), max(height - block_h, 0)))
    cv2.rectangle(out, (x0, top), (x0 + block_w, top + block_h), color, -1)
    for index, (text, (_, text_h)) in enumerate(zip(lines, sizes, strict=True)):
        cv2.putText(
            out,
            text,
            (x0 + 8, top + 8 + line_h * index + text_h),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 0, 0),
            thickness,
        )


def render_overlay(
    cv2: Any,
    frame: Any,
    boxes: list[dict[str, Any]],
    gaze: tuple[float, float, bool] | None,
    selected_id: str | None,
    selection: dict[str, Any] | None = None,
) -> Any:
    """Return a copy of ``frame`` with boxes, the gaze dot and the target marked.

    ``selection`` (optional) is the freshest pick: its box gets a thick border in
    the source colour plus a coordinate banner, or a crosshair when its box has
    already left ``boxes``.
    """
    out = frame.copy()
    height, width = out.shape[:2]
    mean = float(out.mean())
    if mean < 70 or mean > 215:
        problem = "DARK" if mean < 70 else "OVEREXPOSED"
        cv2.putText(
            out,
            f"{problem} FRAME (mean {mean:.0f}) - fix phone lighting/aim",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.8, width / 1200),
            (0, 140, 255),
            3,
        )
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
    if selection is not None:
        color = MAGENTA if str(selection["source"]).lower() == "voice" else GREEN
        match = next(
            (box for box in boxes if box["object_id"] == selection["object_id"]), None
        )
        if match is not None:
            x0 = int(match["x"] - match["w"] / 2)
            y0 = int(match["y"] - match["h"] / 2)
            x1 = int(match["x"] + match["w"] / 2)
            y1 = int(match["y"] + match["h"] / 2)
            cv2.rectangle(out, (x0, y0), (x1, y1), color, 8)
            anchor = ((x0 + x1) // 2, y0)
        else:
            half = 15
            cx = int(selection["image_x"])
            cy = int(selection["image_y"])
            cv2.line(out, (cx - half, cy), (cx + half, cy), color, 4)
            cv2.line(out, (cx, cy - half), (cx, cy + half), color, 4)
            anchor = (cx, cy)
        _draw_banner(cv2, out, _banner_lines(selection), anchor, color, width, height)
    return out
