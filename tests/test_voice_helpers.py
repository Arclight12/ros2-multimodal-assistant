"""Tests for the phone WAV parser, audio conversion and voice matching."""

import io
import json
import struct

import numpy as np
import pytest

from assistant_interaction.audio_stream import (
    WavFormat,
    parse_wav_header,
    run_recognizer,
    to_mono_16k,
)
from assistant_interaction.voice_command import grammar_json, match_class


def make_wav(
    channels=1, rate=44100, bits=16, tag=1, payload=b"", list_chunk=False
):
    """Build a live-style WAV whose data chunk size is 0xFFFFFFFF."""
    block_align = channels * bits // 8
    fmt = struct.pack(
        "<HHIIHH", tag, channels, rate, rate * block_align, block_align, bits
    )
    body = b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt
    if list_chunk:
        # Odd size (3) exercises the pad byte before the data chunk.
        body += b"LIST" + struct.pack("<I", 3) + b"abc" + b"\x00"
    body += b"data" + struct.pack("<I", 0xFFFFFFFF) + payload
    return b"RIFF" + struct.pack("<I", 0xFFFFFFFF) + body


@pytest.mark.parametrize(
    "channels, rate, list_chunk",
    [(1, 44100, False), (2, 48000, True)],
)
def test_parses_header(channels, rate, list_chunk):
    wav = make_wav(channels, rate, list_chunk=list_chunk)
    assert parse_wav_header(io.BytesIO(wav)) == WavFormat(rate, channels, 16)


def test_accepts_extensible_format_tag():
    fmt = parse_wav_header(io.BytesIO(make_wav(tag=0xFFFE)))
    assert fmt == WavFormat(44100, 1, 16)


@pytest.mark.parametrize("list_chunk", [False, True])
def test_stream_left_at_first_pcm_byte(list_chunk):
    payload = b"\x01\x02\x03\x04"
    wav = make_wav(2, 48000, payload=payload, list_chunk=list_chunk)
    stream = io.BytesIO(wav)
    parse_wav_header(stream)
    assert stream.read() == payload


def test_rejects_non_wav():
    with pytest.raises(ValueError, match="RIFF"):
        parse_wav_header(io.BytesIO(b"OggS" + bytes(40)))


def test_rejects_8_bit():
    with pytest.raises(ValueError, match="16-bit"):
        parse_wav_header(io.BytesIO(make_wav(bits=8, rate=8000)))


def test_rejects_non_pcm_tag():
    with pytest.raises(ValueError, match="PCM"):
        parse_wav_header(io.BytesIO(make_wav(tag=3)))


@pytest.mark.parametrize("cut", [5, 30])
def test_rejects_truncated_header(cut):
    with pytest.raises(ValueError, match="truncated"):
        parse_wav_header(io.BytesIO(make_wav()[:cut]))


def test_stereo_44100_resamples_to_16k_mono():
    frames = 4410  # 0.1 s
    left = np.full(frames, 1000, dtype="<i2")
    right = np.full(frames, 3000, dtype="<i2")
    pcm = np.column_stack([left, right]).tobytes()
    out = to_mono_16k(pcm, WavFormat(44100, 2, 16))
    samples = np.frombuffer(out, dtype="<i2")
    assert abs(len(samples) - 1600) <= 2
    assert np.all(samples == 2000)


def test_16k_mono_passthrough():
    pcm = np.array([1, -2, 300, -32768, 32767], dtype="<i2").tobytes()
    assert to_mono_16k(pcm, WavFormat(16000, 1, 16)) == pcm


def test_silence_stays_silent():
    out = to_mono_16k(bytes(4410 * 4), WavFormat(44100, 2, 16))
    assert not np.frombuffer(out, dtype="<i2").any()


def test_drops_trailing_partial_frame():
    samples = np.array([5, 6, 7], dtype="<i2").tobytes()
    out = to_mono_16k(samples + b"\x7f", WavFormat(16000, 1, 16))
    assert out == samples


def test_empty_input_gives_empty_output():
    assert to_mono_16k(b"", WavFormat(44100, 2, 16)) == b""


class FakeRecognizer:
    """Accepts every third chunk and returns a fixed result text."""

    def __init__(self, text="pick up the cup"):
        self.calls = 0
        self.text = text

    def AcceptWaveform(self, data):
        self.calls += 1
        return self.calls % 3 == 0

    def Result(self):
        return json.dumps({"text": self.text})


# 100 ms of 16 kHz mono 16-bit audio.
CHUNK_BYTES = 3200


def test_run_recognizer_reports_text_then_raises_at_end():
    texts = []
    stream = io.BytesIO(bytes(CHUNK_BYTES * 9))
    with pytest.raises(ConnectionError, match="audio stream ended"):
        run_recognizer(
            stream, WavFormat(16000, 1, 16), FakeRecognizer(),
            texts.append, chunk_ms=100,
        )
    assert texts == ["pick up the cup"] * 3


def test_run_recognizer_ignores_empty_text():
    texts = []
    rec = FakeRecognizer(text="")
    run_recognizer(
        io.BytesIO(bytes(CHUNK_BYTES * 9)), WavFormat(16000, 1, 16), rec,
        texts.append, chunk_ms=100, should_stop=lambda: rec.calls >= 9,
    )
    assert texts == []


@pytest.mark.parametrize(
    "text, expected",
    [
        ("pick up the cup", "cup"),
        ("give me the water bottle please", "water_bottle"),
        ("bottle", "water_bottle"),
        ("cup and book", None),
        ("hello", None),
        ("", None),
    ],
)
def test_match_class(text, expected):
    assert match_class(text) == expected


def test_grammar_json_lists_unique_words():
    words = json.loads(grammar_json())
    assert isinstance(words, list)
    assert "[unk]" in words
    assert "water" in words and "bottle" in words
    assert len(words) == len(set(words))
