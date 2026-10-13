"""Hardware-free tests for the camera open and reconnect helpers."""

import pytest

from assistant_interaction.camera_device import (
    open_webcam,
    read_frame as read_webcam_frame,
    resolve_backend,
    retry_delay_s as webcam_retry_delay_s,
)
from assistant_perception.camera_stream import (
    open_mjpeg_stream,
    read_frame,
    retry_delay_s,
)


class FakeCapture:
    """Minimal capture double recording property writes and releases."""

    def __init__(self, opened=True):
        self._opened = opened
        self.properties = []
        self.released = False

    def isOpened(self):
        return self._opened

    def read(self):
        return True, object()

    def set(self, property_id, value):
        self.properties.append((property_id, value))
        return True

    def release(self):
        self.released = True


class FailingCapture(FakeCapture):
    """Capture whose read raises, simulating a driver or codec failure."""

    def read(self):
        raise RuntimeError("device pulled")


class FakeCv2:
    """OpenCV module double exposing capture constants and call recording."""

    CAP_ANY = 0
    CAP_V4L2 = 200
    CAP_DSHOW = 700
    CAP_MSMF = 1400
    CAP_FFMPEG = 1900
    CAP_PROP_OPEN_TIMEOUT_MSEC = 53
    CAP_PROP_READ_TIMEOUT_MSEC = 54
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4

    def __init__(self, capture=None, reject_params=False):
        self.capture = FakeCapture() if capture is None else capture
        self.reject_params = reject_params
        self.calls = []

    def VideoCapture(self, *args):
        self.calls.append(args)
        if self.reject_params and len(args) == 3:
            raise TypeError("no parameters overload")
        return self.capture


def test_retry_delay_doubles_and_caps():
    delays = [retry_delay_s(attempt, 1.0, 30.0) for attempt in range(1, 8)]
    assert delays == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0]


def test_webcam_retry_delay_matches_workspace_backoff():
    assert webcam_retry_delay_s(3, 0.5, 12.0) == retry_delay_s(3, 0.5, 12.0)


def test_open_mjpeg_stream_uses_ffmpeg_backend_with_timeouts():
    fake = FakeCv2()
    capture = open_mjpeg_stream(fake, "http://phone:8080/video", 4000, 8000)

    assert fake.calls == [
        (
            "http://phone:8080/video",
            FakeCv2.CAP_FFMPEG,
            [53, 4000, 54, 8000],
        )
    ]
    assert capture is fake.capture


def test_open_mjpeg_stream_falls_back_without_params_overload():
    fake = FakeCv2(reject_params=True)
    capture = open_mjpeg_stream(fake, "http://phone:8080/video")

    assert fake.calls[-1] == ("http://phone:8080/video", FakeCv2.CAP_FFMPEG)
    assert capture is fake.capture


def test_open_mjpeg_stream_returns_none_when_not_opened():
    fake = FakeCv2(capture=FakeCapture(opened=False))

    assert open_mjpeg_stream(fake, "http://phone:8080/video") is None
    assert fake.capture.released


def test_read_frame_reports_driver_exception_as_failed_read():
    success, frame = read_frame(FailingCapture())

    assert success is False
    assert frame is None


def test_webcam_read_frame_reports_driver_exception_as_failed_read():
    assert read_webcam_frame(FailingCapture()) == (False, None)


def test_resolve_backend_maps_names():
    assert resolve_backend(FakeCv2(), "auto") == FakeCv2.CAP_ANY
    assert resolve_backend(FakeCv2(), "dshow") == FakeCv2.CAP_DSHOW
    assert resolve_backend(FakeCv2(), "MSMF") == FakeCv2.CAP_MSMF
    assert resolve_backend(FakeCv2(), "v4l2") == FakeCv2.CAP_V4L2


def test_resolve_backend_rejects_unknown_names():
    with pytest.raises(ValueError, match="unknown camera backend"):
        resolve_backend(FakeCv2(), "banana")


def test_open_webcam_uses_backend_and_requests_capture_size():
    fake = FakeCv2()
    capture = open_webcam(fake, 1, "dshow", width=640, height=480)

    assert fake.calls == [(1, FakeCv2.CAP_DSHOW)]
    assert capture.properties == [
        (FakeCv2.CAP_PROP_FRAME_WIDTH, 640),
        (FakeCv2.CAP_PROP_FRAME_HEIGHT, 480),
    ]


def test_open_webcam_keeps_driver_defaults_when_size_is_zero():
    fake = FakeCv2()

    assert open_webcam(fake, 0, "auto").properties == []


def test_open_webcam_accepts_device_path_and_rejects_blank():
    fake = FakeCv2()
    open_webcam(fake, "/dev/video2", "v4l2")
    assert fake.calls == [("/dev/video2", FakeCv2.CAP_V4L2)]

    blank = FakeCv2()
    assert open_webcam(blank, "   ") is None
    assert blank.calls == []


def test_open_webcam_returns_none_when_not_opened():
    fake = FakeCv2(capture=FakeCapture(opened=False))

    assert open_webcam(fake, 0, "auto") is None
    assert fake.capture.released
