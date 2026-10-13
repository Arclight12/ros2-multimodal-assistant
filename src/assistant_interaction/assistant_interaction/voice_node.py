"""Phone-microphone voice commands: Vosk speech to an object class name."""

from __future__ import annotations

import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio_stream import open_wav_stream, run_recognizer
from .camera_device import retry_delay_s
from .voice_command import grammar_json, match_class

try:
    import vosk
except ImportError:  # pragma: no cover - optional runtime dependency
    vosk = None


class VoiceNode(Node):
    """Stream phone audio into Vosk and publish the spoken object class."""

    def __init__(self) -> None:
        super().__init__("voice_node")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("voice_stream_url", "")
        self.declare_parameter(
            "vosk_model_path", "models/voice/vosk-model-small-en-us-0.15"
        )
        self.declare_parameter("voice_open_timeout_s", 5.0)
        self.declare_parameter("voice_chunk_ms", 200)
        self.declare_parameter("voice_reconnect_base_delay_s", 1.0)
        self.declare_parameter("voice_reconnect_max_delay_s", 30.0)
        self._url = str(self.get_parameter("voice_stream_url").value)
        self._open_timeout = float(
            self.get_parameter("voice_open_timeout_s").value
        )
        self._chunk_ms = int(self.get_parameter("voice_chunk_ms").value)
        self._base_delay = float(
            self.get_parameter("voice_reconnect_base_delay_s").value
        )
        self._max_delay = float(
            self.get_parameter("voice_reconnect_max_delay_s").value
        )
        self._publisher = self.create_publisher(
            String, "/voice/object_id", 10
        )
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._model = None
        if bool(self.get_parameter("mock_mode").value):
            self.get_logger().info(
                "Voice node is in mock mode; no audio is processed."
            )
            return
        if self._start_recognizer():
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def _start_recognizer(self) -> bool:
        """Load the Vosk model; log the reason and return False if unusable."""
        if not self._url:
            self.get_logger().error(
                "voice_stream_url is empty; voice input is disabled."
            )
            return False
        if vosk is None:
            self.get_logger().error(
                "vosk is not installed; voice input is disabled."
            )
            return False
        path = os.path.expanduser(
            str(self.get_parameter("vosk_model_path").value)
        )
        if not os.path.isdir(path):
            self.get_logger().error(
                f"Vosk model not found at '{path}'; see docs/voice.md."
            )
            return False
        vosk.SetLogLevel(-1)
        self._model = vosk.Model(path)
        return True

    def _run(self) -> None:
        """Reconnecting audio loop; runs in a background thread."""
        attempt = 0
        while not self._stop.is_set():
            try:
                stream, fmt = open_wav_stream(self._url, self._open_timeout)
            except (OSError, ValueError) as exc:
                attempt += 1
                self.get_logger().warning(
                    f"Cannot open phone audio '{self._url}': {exc}"
                )
                self._stop.wait(self._delay(attempt))
                continue
            attempt = 0
            self.get_logger().info(
                f"Listening on phone audio ({fmt.sample_rate} Hz)."
            )
            try:
                recognizer = vosk.KaldiRecognizer(
                    self._model, 16000, grammar_json()
                )
                run_recognizer(
                    stream,
                    fmt,
                    recognizer,
                    self._on_text,
                    chunk_ms=self._chunk_ms,
                    should_stop=self._stop.is_set,
                )
            except (OSError, ValueError) as exc:  # includes ConnectionError
                self.get_logger().warning(f"Phone audio lost: {exc}")
            finally:
                stream.close()
            attempt += 1
            self._stop.wait(self._delay(attempt))

    def _delay(self, attempt: int) -> float:
        return retry_delay_s(attempt, self._base_delay, self._max_delay)

    def _on_text(self, text: str) -> None:
        """Publish the object class named in one recognised phrase."""
        class_name = match_class(text)
        if class_name is None:
            self.get_logger().info(f"Heard '{text}': no object matched.")
            return
        self.get_logger().info(f"Heard '{text}' -> {class_name}")
        self._publisher.publish(String(data=class_name))

    def shutdown_callback(self) -> None:
        """Stop the audio thread."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def main(args: list[str] | None = None) -> None:
    """Run the voice node."""
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_callback()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
