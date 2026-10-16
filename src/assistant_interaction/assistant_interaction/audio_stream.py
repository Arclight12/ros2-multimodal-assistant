"""Phone microphone audio, cut into spoken utterances.

The Android "IP Webcam" app serves microphone audio as an endless WAV over
HTTP. These helpers parse that header, convert PCM to 16 kHz mono 16-bit,
and cut the stream into utterances by loudness. No ROS imports, so the
logic stays testable without hardware.
"""

from __future__ import annotations

import struct
import urllib.request
from dataclasses import dataclass
from typing import Any

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


def open_wav_stream(url: str, timeout_s: float = 5.0) -> tuple[Any, WavFormat]:
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


class Segmenter:
    """Cut a stream of equal PCM chunks into utterances by loudness.

    A chunk is speech when its RMS exceeds both ``min_rms`` and three times
    the running background level. An utterance ends after ``end_chunks`` quiet
    chunks or ``max_chunks`` in total; ones shorter than ``min_chunks`` are
    dropped as clicks. The last ``preroll`` quiet chunks are kept so the first
    syllable is not clipped.
    """

    def __init__(
        self,
        min_rms: float = 500.0,
        end_chunks: int = 6,
        min_chunks: int = 3,
        max_chunks: int = 60,
        preroll: int = 2,
    ) -> None:
        self._min_rms = min_rms
        self._end = end_chunks
        self._min = min_chunks
        self._max = max_chunks
        self._preroll = preroll
        self._noise = 0.0
        self._quiet: list[bytes] = []
        self._speech: list[bytes] = []
        self._silent = 0
        self._lead = 0

    def feed(self, chunk: bytes) -> bytes | None:
        """Add one 16 kHz mono chunk; return finished utterance PCM, else None."""
        samples = np.frombuffer(chunk, dtype="<i2").astype(np.float64)
        rms = float(np.sqrt(np.mean(samples**2))) if len(samples) else 0.0
        loud = rms > max(self._min_rms, 3.0 * self._noise)
        if not self._speech:
            if not loud:
                self._noise = 0.95 * self._noise + 0.05 * rms
                self._quiet = (self._quiet + [chunk])[-self._preroll :]
                return None
            self._lead = len(self._quiet)
            self._speech = self._quiet + [chunk]
            self._quiet = []
            self._silent = 0
            return None
        self._speech.append(chunk)
        self._silent = 0 if loud else self._silent + 1
        if self._silent < self._end and len(self._speech) < self._max:
            return None
        voiced = len(self._speech) - self._lead - self._silent
        pcm = b"".join(self._speech) if voiced >= self._min else None
        self._speech = []
        return pcm
