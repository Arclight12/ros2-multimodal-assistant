"""Phone-microphone voice commands: NVIDIA speech NIM to an object class."""

from __future__ import annotations

import os
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio_stream import TARGET_RATE, Segmenter, open_wav_stream, to_mono_16k
from .camera_device import retry_delay_s
from .voice_command import match_class

try:
    import riva.client
except ImportError:  # pragma: no cover - optional runtime dependency
    riva = None


class VoiceNode(Node):
    """Cut phone audio into utterances and publish the object each one names."""

    def __init__(self) -> None:
        super().__init__("voice_node")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("voice_stream_url", "")
        self.declare_parameter("nvidia_server", "grpc.nvcf.nvidia.com:443")
        # nemotron-asr-streaming (English). Streaming-only: it rejects one-shot calls.
        self.declare_parameter(
            "nvidia_function_id", "bb0837de-8c7b-481f-9ec8-ef5663e9c1fa"
        )
        self.declare_parameter("language_code", "en-US")
        self.declare_parameter("voice_open_timeout_s", 5.0)
        self.declare_parameter("voice_chunk_ms", 100)
        self.declare_parameter("voice_min_rms", 500.0)
        self.declare_parameter("voice_end_silence_ms", 600)
        self.declare_parameter("voice_max_utterance_s", 6.0)
        self.declare_parameter("voice_reconnect_base_delay_s", 1.0)
        self.declare_parameter("voice_reconnect_max_delay_s", 30.0)
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self._url = str(get("voice_stream_url"))
        self._open_timeout = float(get("voice_open_timeout_s"))
        self._chunk_ms = int(get("voice_chunk_ms"))
        self._segmenter_args = dict(
            min_rms=float(get("voice_min_rms")),
            end_chunks=int(get("voice_end_silence_ms")) // self._chunk_ms,
            max_chunks=int(float(get("voice_max_utterance_s")) * 1000)
            // self._chunk_ms,
        )
        self._base_delay = float(get("voice_reconnect_base_delay_s"))
        self._max_delay = float(get("voice_reconnect_max_delay_s"))
        self._publisher = self.create_publisher(String, "/voice/object_id", 10)
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        if bool(get("mock_mode")):
            self.get_logger().info("Voice node is in mock mode; no audio is processed.")
            return
        key = os.environ.get("NVIDIA_API_KEY", "")
        if not self._url or not key:
            self.get_logger().error(
                "voice_stream_url or NVIDIA_API_KEY is missing; voice is disabled."
            )
            return
        if riva is None:
            self.get_logger().error(
                "nvidia-riva-client is not installed; voice is disabled."
            )
            return
        auth = riva.client.Auth(
            None,
            True,
            str(get("nvidia_server")),
            [
                ["function-id", str(get("nvidia_function_id"))],
                ["authorization", f"Bearer {key}"],
            ],
        )
        self._asr = riva.client.ASRService(auth)
        self._config = riva.client.StreamingRecognitionConfig(
            config=riva.client.RecognitionConfig(
                encoding=riva.client.AudioEncoding.LINEAR_PCM,
                language_code=str(get("language_code")),
                sample_rate_hertz=TARGET_RATE,
                audio_channel_count=1,
                max_alternatives=1,
            ),
            interim_results=False,
        )
        # One slot: a newer utterance replaces one still waiting for the API.
        self._utterances: queue.Queue = queue.Queue(maxsize=1)
        for target in (self._listen, self._ask_loop):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)

    def _listen(self) -> None:
        """Reconnecting audio loop: cut utterances and queue them."""
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
            self.get_logger().info(f"Listening on phone audio ({fmt.sample_rate} Hz).")
            segmenter = Segmenter(**self._segmenter_args)
            nbytes = max(1, fmt.sample_rate * self._chunk_ms // 1000) * fmt.channels * 2
            try:
                while not self._stop.is_set():
                    raw = b""
                    # Loop so a short read never splits a frame.
                    while len(raw) < nbytes:
                        part = stream.read(nbytes - len(raw))
                        if not part:
                            raise ConnectionError("audio stream ended")
                        raw += part
                    pcm = segmenter.feed(to_mono_16k(raw, fmt))
                    if pcm is not None:
                        self._submit(pcm)
            except (OSError, ValueError) as exc:  # includes ConnectionError
                self.get_logger().warning(f"Phone audio lost: {exc}")
            finally:
                stream.close()
            attempt += 1
            self._stop.wait(self._delay(attempt))

    def _submit(self, pcm: bytes) -> None:
        try:
            self._utterances.get_nowait()
        except queue.Empty:
            pass
        self._utterances.put(pcm)

    def _ask_loop(self) -> None:
        """Transcribe each utterance and publish the object class it names."""
        while not self._stop.is_set():
            try:
                pcm = self._utterances.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                text = self._transcribe(pcm)
            except Exception as exc:  # noqa: BLE001 - grpc raises its own types
                self.get_logger().warning(f"NVIDIA speech request failed: {exc}")
                continue
            class_name = match_class(text)
            if class_name is None:
                self.get_logger().info(f"Heard '{text}': no object matched.")
                continue
            self.get_logger().info(f"Heard '{text}' -> {class_name}")
            self._publisher.publish(String(data=class_name))

    def _transcribe(self, pcm: bytes) -> str:
        """Stream one utterance in 100 ms chunks; the channel stays open."""
        step = TARGET_RATE // 10 * 2
        chunks = (pcm[i : i + step] for i in range(0, len(pcm), step))
        return " ".join(
            result.alternatives[0].transcript.strip()
            for response in self._asr.streaming_response_generator(chunks, self._config)
            for result in response.results
            if result.is_final
        ).strip()

    def _delay(self, attempt: int) -> float:
        return retry_delay_s(attempt, self._base_delay, self._max_delay)

    def shutdown_callback(self) -> None:
        """Stop the worker threads."""
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)


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
