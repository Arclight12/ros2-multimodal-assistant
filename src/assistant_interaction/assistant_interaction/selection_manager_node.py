"""Selection manager node: arbitrates between voice and gaze selections.

The selection manager subscribes to both the voice and gaze object-id topics.
When a selection arrives, it resolves any conflict according to the project
rule that **voice takes priority** during disagreements. The final resolved
selection is published on ``/final_selection`` for the motion planner.

No motion planning or perception logic lives here.

Topics
------
* Subscribes to ``/voice/object_id`` and ``/gaze/object_id``.
* Publishes ``/final_selection`` (``assistant_msgs/ObjectSelection``).
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from assistant_msgs.msg import ObjectSelection

# Valid sources defined by the interface contract.
_SOURCE_VOICE = "voice"
_SOURCE_GAZE = "gaze"


class SelectionManagerNode(Node):
    """Resolve incoming voice/gaze selections into a single final selection."""

    def __init__(self) -> None:
        """Initialize the selection manager node."""
        super().__init__("selection_manager_node")

        self.declare_parameter("confirmation_timeout", 1.5)

        self._confirmation_timeout: float = (
            self.get_parameter("confirmation_timeout")
            .get_parameter_value()
            .double_value
        )

        # Latest candidate object per source.
        self._pending_selection: Optional[str] = None
        self._pending_source: Optional[str] = None

        self._publisher = self.create_publisher(
            ObjectSelection, "/final_selection", 10
        )

        self._voice_sub = self.create_subscription(
            ObjectSelection, "/voice/object_id", self._on_voice, 10
        )
        self._gaze_sub = self.create_subscription(
            ObjectSelection, "/gaze/object_id", self._on_gaze, 10
        )

        self.get_logger().info("SelectionManagerNode initialized.")

    def _on_voice(self, msg: ObjectSelection) -> None:
        """Handle an incoming voice selection.

        Voice is the highest-priority source. When received, it immediately wins
        over any pending gaze selection.

        :param msg: The voice selection message.
        """
        self.get_logger().info(
            f"Received voice selection for object '{msg.object_id}'."
        )
        self._resolve_conflict(msg, priority=_SOURCE_VOICE)

    def _on_gaze(self, msg: ObjectSelection) -> None:
        """Handle an incoming gaze selection.

        Gaze is lower priority than voice. If a voice selection is already
        pending, the gaze selection is ignored.

        :param msg: The gaze selection message.
        """
        self.get_logger().info(
            f"Received gaze selection for object '{msg.object_id}'."
        )
        self._resolve_conflict(msg, priority=_SOURCE_GAZE)

    def _resolve_conflict(self, msg: ObjectSelection, priority: str) -> None:
        """Apply the arbitration rule and publish a final selection.

        Rule: voice wins over gaze. Gaze is only accepted when no conflicting
        voice selection exists.

        :param msg: The incoming selection message.
        :param priority: The source priority of this message.
        """
        if priority == _SOURCE_GAZE and self._pending_source == _SOURCE_VOICE:
            self.get_logger().info(
                "Conflict detected: voice has priority over gaze. "
                f"Ignoring gaze '{msg.object_id}'."
            )
            return

        self._pending_selection = msg.object_id
        self._pending_source = msg.source

        selected = ObjectSelection()
        selected.object_id = msg.object_id
        selected.source = msg.source
        self._publisher.publish(selected)
        self.get_logger().info(
            f"Final selection published: '{selected.object_id}' (source={selected.source})."
        )

    def shutdown_callback(self) -> None:
        """Perform clean shutdown of the selection manager."""
        self.get_logger().info("Shutting down selection manager node.")


def main(args: list[str] | None = None) -> None:
    """Entry point for the selection manager node executable."""
    rclpy.init(args=args)
    node = SelectionManagerNode()

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
