#!/usr/bin/env python3
"""Check the phone microphone and the Vosk voice grammar without starting ROS.

Streams the phone's IP Webcam audio, runs the same Vosk recogniser the voice
node uses, and prints each heard phrase with the object class it maps to.

Examples:
  python scripts/check_voice.py --url http://192.168.1.10:8080/audio.wav
  python scripts/check_voice.py --url ... --seconds 30
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_interaction"))

from assistant_interaction.audio_stream import (  # noqa: E402
    open_wav_stream,
    run_recognizer,
)
from assistant_interaction.voice_command import (  # noqa: E402
    grammar_json,
    match_class,
)

MODEL_URL = "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip"
DEFAULT_MODEL = "models/voice/vosk-model-small-en-us-0.15"
STREAM_HINT = (
    "Check that IP Webcam is started on the phone, the URL is "
    "http://<phone-ip>:8080/audio.wav, the laptop and phone share Wi-Fi, and "
    "the app has microphone permission."
)


def parse_args() -> argparse.Namespace:
    """Parse and return the diagnostic CLI arguments."""
    parser = argparse.ArgumentParser(
        description="Verify the phone microphone and the Vosk voice grammar.",
    )
    parser.add_argument(
        "--url",
        required=True,
        help="phone audio URL, e.g. http://192.168.1.10:8080/audio.wav",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help=f"unpacked Vosk model directory (default: {DEFAULT_MODEL})",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=15.0,
        help="how long to listen (default: 15)",
    )
    return parser.parse_args()


def main() -> int:
    """Run the recogniser for the requested time and return an exit code."""
    args = parse_args()
    try:
        import vosk
    except ImportError:
        print(
            "Vosk is required: python3 -m pip install vosk", file=sys.stderr
        )
        return 2
    model_dir = Path(args.model)
    if not model_dir.is_dir():
        print(
            f"Vosk model not found at {model_dir}. Download {MODEL_URL} and "
            "unzip it into models/voice/ so that the folder above exists."
        )
        return 2

    try:
        stream, fmt = open_wav_stream(args.url)
    except (ConnectionError, ValueError, OSError) as exc:
        print(f"FAIL: could not open {args.url!r}: {exc}. {STREAM_HINT}")
        return 1
    print(f"stream format: {fmt}")

    recognizer = vosk.KaldiRecognizer(
        vosk.Model(str(model_dir)), 16000, grammar_json()
    )
    heard = 0

    def on_text(text: str) -> None:
        nonlocal heard
        if not text.strip():
            return
        heard += 1
        matched = match_class(text) or "no object matched"
        print(f"heard: {text!r} -> {matched}")

    deadline = time.monotonic() + args.seconds
    print(f"listening for {args.seconds:g}s, say e.g. 'pick up the cup' ...")
    try:
        run_recognizer(
            stream,
            fmt,
            recognizer,
            on_text,
            should_stop=lambda: time.monotonic() >= deadline,
        )
    except (ConnectionError, ValueError, OSError) as exc:
        print(f"FAIL: audio stream error: {exc}. {STREAM_HINT}")
        return 1
    finally:
        stream.close()
    print(f"done: {heard} phrase(s) heard")
    return 0 if heard else 1


if __name__ == "__main__":
    raise SystemExit(main())
