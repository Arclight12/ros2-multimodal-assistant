#!/usr/bin/env python3
"""Fast interactive dataset annotator for the 6 workspace objects.

Features:
- Single Click: Auto-snaps a tight bounding box around the clicked object
- Click & Drag: Manually draw/adjust any bounding box
- Press 1-6 to switch active class
- Press SPACE / ENTER to save & advance to the next image
- Press 'b' to go back, 'c' to clear current image, 'q' to finish
"""

from __future__ import annotations
import argparse
from pathlib import Path
import cv2
import numpy as np

CLASSES = [
    "sharpener",
    "eraser",
    "thermocol",
    "block_a",
    "block_b",
    "block_c",
]

COLORS = [
    (0, 220, 0),    # sharpener - green
    (40, 40, 220),  # eraser - red/dark
    (255, 180, 0),  # thermocol - cyan/blue
    (240, 240, 240),# block_a - white
    (255, 120, 0),  # block_b - light blue
    (0, 240, 255),  # block_c - yellow
]


def snap_box(image: np.ndarray, click_x: int, click_y: int, pad: int = 55) -> tuple[int, int, int, int]:
    """Snap a tight bounding box around object at click coordinates."""
    h, w = image.shape[:2]
    x1, y1 = max(0, click_x - pad), max(0, click_y - pad)
    x2, y2 = min(w, click_x + pad), min(h, click_y + pad)
    roi = image[y1:y2, x1:x2]
    if roi.size == 0:
        return click_x - 25, click_y - 25, 50, 50

    # Cardboard background color estimate from border of ROI
    border_pixels = np.vstack([roi[0, :], roi[-1, :], roi[:, 0], roi[:, -1]])
    bg = np.median(border_pixels, axis=0)
    diff = np.linalg.norm(roi.astype(np.float32) - bg, axis=2)
    mask = (diff > 22).astype(np.uint8) * 255

    # Connected component / contour containing click
    cx, cy = click_x - x1, click_y - y1
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    for c in cnts:
        if cv2.pointPolygonTest(c, (cx, cy), False) >= 0:
            best_cnt = c
            break
        elif cv2.contourArea(c) > 250:
            best_cnt = c

    if best_cnt is not None:
        bx, by, bw, bh = cv2.boundingRect(best_cnt)
        return x1 + max(0, bx - 4), y1 + max(0, by - 4), bw + 8, bh + 8
    return click_x - 25, click_y - 25, 50, 50


class Annotator:
    def __init__(self, images_dir: Path, labels_dir: Path):
        self.images_dir = images_dir
        self.labels_dir = labels_dir
        self.image_paths = sorted(images_dir.glob("*.jpg"))
        self.current_idx = 0
        self.active_class_idx = 0

        self.boxes: list[tuple[int, int, int, int, int]] = []  # (cls_id, x, y, w, h)
        self.drag_start = None
        self.drag_current = None
        self.is_dragging = False

        self.window_name = "Fast Dataset Annotator - [SPACE: Next | 1-6: Class | c: Clear | q: Done]"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.drag_current = (x, y)
            self.is_dragging = False

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drag_start is not None:
                dx = abs(x - self.drag_start[0])
                dy = abs(y - self.drag_start[1])
                if dx > 8 or dy > 8:
                    self.is_dragging = True
                    self.drag_current = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            if self.drag_start is not None:
                if self.is_dragging:
                    # Manual drag box
                    x1 = min(self.drag_start[0], x)
                    y1 = min(self.drag_start[1], y)
                    w = abs(x - self.drag_start[0])
                    h = abs(y - self.drag_start[1])
                    if w > 10 and h > 10:
                        self.boxes.append((self.active_class_idx, x1, y1, w, h))
                        self.active_class_idx = (self.active_class_idx + 1) % len(CLASSES)
                else:
                    # Single click -> auto-snap
                    cur_img = cv2.imread(str(self.image_paths[self.current_idx]))
                    bx, by, bw, bh = snap_box(cur_img, x, y)
                    self.boxes.append((self.active_class_idx, bx, by, bw, bh))
                    self.active_class_idx = (self.active_class_idx + 1) % len(CLASSES)

                self.drag_start = None
                self.drag_current = None
                self.is_dragging = False

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click removes last box
            if self.boxes:
                self.boxes.pop()
                if self.active_class_idx > 0:
                    self.active_class_idx -= 1

    def load_annotations(self, img_path: Path, img_shape: tuple[int, int]):
        label_file = self.labels_dir / f"{img_path.stem}.txt"
        self.boxes = []
        if label_file.exists():
            h, w = img_shape[:2]
            for line in label_file.read_text().splitlines():
                parts = line.strip().split()
                if len(parts) == 5:
                    cid = int(parts[0])
                    xc, yc, nw, nh = map(float, parts[1:])
                    bx = int((xc - nw / 2) * w)
                    by = int((yc - nh / 2) * h)
                    bw = int(nw * w)
                    bh = int(nh * h)
                    self.boxes.append((cid, bx, by, bw, bh))

    def save_annotations(self, img_path: Path, img_shape: tuple[int, int]):
        label_file = self.labels_dir / f"{img_path.stem}.txt"
        h, w = img_shape[:2]
        lines = []
        for cid, bx, by, bw, bh in self.boxes:
            xc = (bx + bw / 2.0) / w
            yc = (by + bh / 2.0) / h
            nw = bw / float(w)
            nh = bh / float(h)
            lines.append(f"{cid} {xc:.6f} {yc:.6f} {nw:.6f} {nh:.6f}")
        label_file.write_text("\n".join(lines) + ("\n" if lines else ""))

    def run(self):
        if not self.image_paths:
            print("No images found in", self.images_dir)
            return

        while 0 <= self.current_idx < len(self.image_paths):
            p = self.image_paths[self.current_idx]
            img = cv2.imread(str(p))
            if img is None:
                self.current_idx += 1
                continue

            h, w = img.shape[:2]
            self.load_annotations(p, img.shape)

            while True:
                display = img.copy()

                # Draw saved boxes
                for cid, bx, by, bw, bh in self.boxes:
                    col = COLORS[cid % len(COLORS)]
                    name = CLASSES[cid % len(CLASSES)]
                    cv2.rectangle(display, (bx, by), (bx + bw, by + bh), col, 2)
                    cv2.putText(
                        display,
                        name,
                        (bx, max(18, by - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        col,
                        2,
                    )

                # Draw dragging box preview
                if self.is_dragging and self.drag_start and self.drag_current:
                    x1 = min(self.drag_start[0], self.drag_current[0])
                    y1 = min(self.drag_start[1], self.drag_current[1])
                    bw = abs(self.drag_current[0] - self.drag_start[0])
                    bh = abs(self.drag_current[1] - self.drag_start[1])
                    cv2.rectangle(display, (x1, y1), (x1 + bw, y1 + bh), (0, 255, 255), 1)

                # Header UI
                cv2.rectangle(display, (0, 0), (w, 55), (30, 30, 30), -1)
                cur_cls_name = CLASSES[self.active_class_idx]
                cur_col = COLORS[self.active_class_idx]
                title = f"[{self.current_idx + 1}/{len(self.image_paths)}] Next Class: {cur_cls_name.upper()} | Boxes: {len(self.boxes)}"
                cv2.putText(display, title, (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.85, cur_col, 2)

                # Footer help
                cv2.rectangle(display, (0, h - 30), (w, h), (20, 20, 20), -1)
                help_str = "CLICK: Auto-snap box | DRAG: Custom box | RIGHT-CLICK: Undo | 1-6: Class | SPACE: Next | b: Back | q: Save&Done"
                cv2.putText(display, help_str, (15, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

                cv2.imshow(self.window_name, display)
                key = cv2.waitKey(20) & 0xFF

                if key in (27, ord("q")):  # ESC / q
                    self.save_annotations(p, img.shape)
                    cv2.destroyAllWindows()
                    print(f"Annotated up to image {self.current_idx + 1}")
                    return

                elif key in (32, 13):  # SPACE / ENTER -> Save & Next
                    self.save_annotations(p, img.shape)
                    self.current_idx += 1
                    self.active_class_idx = 0
                    break

                elif key == ord("b"):  # Back
                    self.save_annotations(p, img.shape)
                    self.current_idx = max(0, self.current_idx - 1)
                    self.active_class_idx = 0
                    break

                elif key == ord("c"):  # Clear
                    self.boxes = []
                    self.active_class_idx = 0

                elif ord("1") <= key <= ord("6"):
                    self.active_class_idx = key - ord("1")

        cv2.destroyAllWindows()
        print("All annotations saved!")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("datasets/object_detection/raw/images"))
    parser.add_argument("--labels", type=Path, default=Path("datasets/object_detection/raw/labels"))
    args = parser.parse_args()

    # Save classes.txt
    classes_file = Path("datasets/object_detection/classes.txt")
    classes_file.parent.mkdir(parents=True, exist_ok=True)
    classes_file.write_text("\n".join(CLASSES) + "\n")

    annotator = Annotator(args.images, args.labels)
    annotator.run()


if __name__ == "__main__":
    main()
