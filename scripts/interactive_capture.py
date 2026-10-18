#!/usr/bin/env python3
"""Interactive dataset capture tool with live visual display and audio cues.

Controls:
  SPACE: Capture current frame (plays shutter sound and flashes green)
  a:     Toggle automated countdown mode (countdown with audio beeps)
  q/ESC: Save and quit
"""

import argparse
import subprocess
import time
from pathlib import Path
import cv2
import numpy as np


def play_sound(sound_name: str) -> None:
    """Play a desktop system sound asynchronously."""
    try:
        subprocess.Popen(
            ["canberra-gtk-play", "-i", sound_name],
            stdout=cv2.DEVNULL if hasattr(cv2, "DEVNULL") else None,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def play_voice(text: str) -> None:
    """Speak text asynchronously."""
    try:
        subprocess.Popen(["spd-say", "-t", "female2", text], stderr=subprocess.DEVNULL)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://192.168.1.2:8080/video")
    parser.add_argument("--output", type=Path, default=Path("datasets/object_detection/raw"))
    parser.add_argument("--target", type=int, default=40)
    args = parser.parse_args()

    image_dir = args.output / "images"
    label_dir = args.output / "labels"
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)

    # Find existing count
    existing = len(list(image_dir.glob("frame_*.jpg")))
    captured_count = existing

    cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        print(f"Error: Could not open camera stream at {args.source}")
        return

    window_name = "Dataset Collector - [SPACE: Capture | a: Auto Timer | q: Quit]"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1024, 576)

    auto_mode = False
    countdown_start = time.time()
    countdown_duration = 3.5  # seconds
    flash_timer = 0.0

    print("\n" + "=" * 60)
    print("Interactive Dataset Collector Started!")
    print(f"Goal: {args.target} clean frames (currently have {existing})")
    print("Press SPACE to capture a frame whenever your hands are off.")
    print("Press 'a' to toggle automated 3-second timer with audio beeps.")
    print("Press 'q' or ESC to exit.")
    print("=" * 60 + "\n")

    play_sound("bell")

    last_spoken_sec = -1

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.02)
            continue

        display = frame.copy()
        h, w = display.shape[:2]
        now = time.time()

        # Handle auto-countdown mode
        if auto_mode:
            elapsed = now - countdown_start
            remaining = countdown_duration - elapsed
            sec = int(np.ceil(remaining))

            if sec > 0 and sec != last_spoken_sec:
                last_spoken_sec = sec
                play_sound("bell")

            if remaining <= 0:
                # Capture frame!
                captured_count += 1
                img_path = image_dir / f"frame_{captured_count:06d}.jpg"
                lbl_path = label_dir / f"frame_{captured_count:06d}.txt"
                cv2.imwrite(str(img_path), frame)
                lbl_path.touch()
                play_sound("camera-shutter")
                flash_timer = now + 0.3
                countdown_start = now
                last_spoken_sec = -1
                if captured_count >= args.target:
                    print(f"Reached target {args.target} frames!")
                    play_sound("complete")
                    break
            else:
                banner_text = f"AUTO CAPTURING IN {remaining:.1f}s - HANDS OFF!"
                cv2.rectangle(display, (0, 0), (w, 60), (0, 0, 200), -1)
                cv2.putText(
                    display,
                    banner_text,
                    (20, 42),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.1,
                    (255, 255, 255),
                    3,
                )
        else:
            # Manual mode instructions overlay
            cv2.rectangle(display, (0, 0), (w, 55), (40, 40, 40), -1)
            cv2.putText(
                display,
                f"Press SPACE to Capture | Count: {captured_count}/{args.target}",
                (20, 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )

        # Flash green indicator on capture
        if now < flash_timer:
            cv2.rectangle(display, (0, 0), (w, h), (0, 255, 0), 12)
            cv2.putText(
                display,
                "CAPTURED!",
                (w // 2 - 120, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),
                4,
            )

        # Bottom status bar
        cv2.rectangle(display, (0, h - 35), (w, h), (20, 20, 20), -1)
        status_msg = f"Mode: {'AUTO-COUNTDOWN' if auto_mode else 'MANUAL (SPACEBAR)'} | Frames: {captured_count} | Press 'q' to Finish"
        cv2.putText(
            display,
            status_msg,
            (15, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 200, 200),
            1,
        )

        cv2.imshow(window_name, display)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):  # ESC or q
            break
        elif key == 32:  # SPACE
            captured_count += 1
            img_path = image_dir / f"frame_{captured_count:06d}.jpg"
            lbl_path = label_dir / f"frame_{captured_count:06d}.txt"
            cv2.imwrite(str(img_path), frame)
            lbl_path.touch()
            play_sound("camera-shutter")
            flash_timer = now + 0.3
            print(f"Captured {img_path.name} ({captured_count}/{args.target})")
            if captured_count >= args.target:
                print(f"Reached target {args.target} frames!")
                play_sound("complete")
                break
        elif key == ord("a"):  # Toggle auto mode
            auto_mode = not auto_mode
            countdown_start = now
            last_spoken_sec = -1
            play_sound("dialog-information")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nDone! Total frames in dataset: {captured_count}")


if __name__ == "__main__":
    main()
