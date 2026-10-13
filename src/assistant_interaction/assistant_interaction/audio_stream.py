"""Phone microphone audio for the offline Vosk recogniser.

The Android "IP Webcam" app serves microphone audio as an endless WAV over
HTTP. These helpers parse that header, convert PCM to the 16 kHz mono
16-bit layout Vosk expects, and feed chunks to a recogniser. No ROS or vosk
imports, so the logic stays testable without hardware.
"""

from __future__ import annotations

import json
import struct
import urllib.request
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

TARGET_RATE = 16000
PCM_TAG = 1
EXTENSIBLE_TAG = 0xFFFE
MAX_HEADER_CHUNK = 1 << 16


@dataclass(frozen=True)
class WavFormat:
    """Sample layout of a PCM WAV stream."""

    sample_rate: int
    channels: int
    bits: int


def _read_exact(stream: Any, size: int) -> bytes:
    """Read exactly ``size`` bytes, raising ``ValueError`` on early EOF."""
    data = b""
    while len(data) < size:
        part = stream.read(size - len(data))
        if not part:
            raise ValueError("truncated WAV header")
        data += part
    return data


def parse_wav_header(stream: Any) -> WavFormat:
    """Consume a WAV header and return its format.

    Stops at the first PCM byte of the ``data`` chunk. The data size is
    ignored because live streams advertise 0 or 0xFFFFFFFF.
    """
    riff = _read_exact(stream, 12)
    if riff[:4] != b"RIFF" or riff[8:12] != b"WAVE":
        raise ValueError("stream is not a RIFF/WAVE file")
    fmt_body = None
    while True:
        chunk_id, size = struct.unpack("<4sI", _read_exact(stream, 8))
        if chunk_id == b"data":
            break
        if size > MAX_HEADER_CHUNK:
            raise ValueError(f"oversized WAV chunk {chunk_id!r}")
        body = _read_exact(stream, size + (size & 1))
        if chunk_id == b"fmt ":
            fmt_body = body[:size]
    if fmt_body is None:
        raise ValueError("WAV data chunk found before fmt chunk")
    if len(fmt_body) < 16:
        raise ValueError("WAV fmt chunk is too short")
    tag, channels, rate, _, _, bits = struct.unpack("<HHIIHH", fmt_body[:16])
    if tag not in (PCM_TAG, EXTENSIBLE_TAG):
        raise ValueError(f"WAV format tag {tag:#x} is not PCM")
    if bits != 16:
        raise ValueError(f"WAV must be 16-bit PCM, got {bits}-bit")
    if channels < 1 or rate < 1:
        raise ValueError("WAV fmt chunk has no channels or sample rate")
    return WavFormat(rate, channels, bits)


def open_wav_stream(
    url: str, timeout_s: float = 5.0
) -> tuple[Any, WavFormat]:
    """Open an HTTP WAV stream and return ``(response, format)``."""
    response = urllib.request.urlopen(url, timeout=timeout_s)
    try:
        fmt = parse_wav_header(response)
    except BaseException:
        response.close()
        raise
    return response, fmt


def to_mono_16k(pcm: bytes, fmt: WavFormat) -> bytes:
    """Convert interleaved 16-bit PCM to 16 kHz mono int16 little-endian."""
    frame_bytes = 2 * fmt.channels
    usable = len(pcm) - len(pcm) % frame_bytes
    if usable == 0:
        return b""
    samples = np.frombuffer(pcm[:usable], dtype="<i2")
    mono = samples.reshape(-1, fmt.channels).astype(np.float64).mean(axis=1)
    if fmt.sample_rate != TARGET_RATE:
        # shortcut: per-chunk resample ignores chunk-edge phase, negligible speech
        count = round(len(mono) * TARGET_RATE / fmt.sample_rate)
        src_t = np.arange(len(mono)) / fmt.sample_rate
        dst_t = np.arange(count) / TARGET_RATE
        mono = np.interp(dst_t, src_t, mono)
    return np.rint(mono).astype("<i2").tobytes()


def run_recognizer(
    stream: Any,
    fmt: WavFormat,
    recognizer: Any,
    on_text: Callable[[str], None],
    chunk_ms: int = 200,
    should_stop: Callable[[], bool] = lambda: False,
) -> None:
    """Feed live audio to ``recognizer`` until ``should_stop()`` is true.

    Raises ``ConnectionError`` when the stream ends so the caller can
    reconnect. Each non-empty recognised result goes to ``on_text``.
    """
    frames = max(1, fmt.sample_rate * chunk_ms // 1000)
    nbytes = frames * fmt.channels * 2
    while not should_stop():
        raw = b""
        # Loop so a short read never splits a frame and misaligns the rest.
        while len(raw) < nbytes:
            part = stream.read(nbytes - len(raw))
            if not part:
                break
            raw += part
        if not raw:
            raise ConnectionError("audio stream ended")
        pcm = to_mono_16k(raw, fmt)
        if recognizer.AcceptWaveform(pcm):
            text = json.loads(recognizer.Result()).get("text", "")
            if text:
                on_text(text)
