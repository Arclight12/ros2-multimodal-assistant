"""Voice input node for the multimodal assistant.

Captures audio from a microphone and converts speech to a target object name
using offline speech recognition (Vosk). The recognized object is published on
the ``/voice/object_id`` topic for the selection manager to consume.

This node contains no gaze, motion, or planning logic.

Topics
------
* Publishes ``/voice/object_id`` (``assistant_msgs/ObjectSelection``).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from assistant_msgs.msg import ObjectSelection


class VoiceNode(Node):
    """Recognize spoken object names and publish them for selection."""

    def __init__(self) -> None:
        """Initialize the voice input node."""
        super().__init__("voice_node")

        self.declare_parameter("audio_device_index", 0)
        self.declare_parameter("vosk_model_path", "models/vosk-model-small-en-us")
        self.declare_parameter("sample_rate", 16000)

        self._audio_device_index: int = (
            self.get_parameter("audio_device_index").get_parameter_value().integer_value
        )
        self._vosk_model_path: str = (
            self.get_parameter("vosk_model_path").get_parameter_value().string_value
        )
        self._sample_rate: int = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )

        self._publisher = self.create_publisher(
            ObjectSelection, "/voice/object_id", 10
        )

        self._recognizer = None
        self._try_load_recognizer()

        self.get_logger().info(
            f"VoiceNode initialized | device={self._audio_device_index} "
            f"| model={self._vosk_model_path}"
        )

        # Start the audio capture loop as a background thread.
        self._setup_audio_loop()

    def _try_load_recognizer(self) -> None:
        """Attempt to load the Vosk recognizer.

        This is an integration point. If the model is not yet present, the node
        logs a warning and continues running without making the system crash.
        """
        # TODO(interaction-team): instantiate Vosk recognizer here once the
        # model is downloaded and available at ``self._vosk_model_path``.
        self.get_logger().warn(
            "Vosk model not loaded. Voice recognition is a placeholder (TODO)."
        )

    def _setup_audio_loop(self) -> None:
        """Configure the background capture loop.

        TODO(interaction-team): open the microphone stream, read audio chunks and
        feed them to the recognizer. This method is a placeholder.
        """
        self.get_logger().info("Audio loop configured (placeholder).")

    def _publish_object(self, object_id: str) -> None:
        """Publish a recognized object selection on ``/voice/object_id``.

        :param object_id: The recognized object identifier.
        """
        msg = ObjectSelection()
        msg.object_id = object_id
        msg.source = "voice"
        self._publisher.publish(msg)
        self.get_logger().info(f"Voice selection: {object_id}")

    def shutdown_callback(self) -> None:
        """Perform clean shutdown of the audio stream and node."""
        self.get_logger().info("Shutting down voice node.")
        # TODO(interaction-team): stop and release the audio stream here.


def main(args: list[str] | None = None) -> None:
    """Entry point for the voice node executable."""
    rclpy.init(args=args)
    node = VoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl+C).")
    finally:
        node.shutdown_callback()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
