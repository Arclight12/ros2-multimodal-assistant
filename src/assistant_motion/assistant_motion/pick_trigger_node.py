"""Turn the object chosen by gaze or voice into a slot label and, if allowed, a pick.

It subscribes to ``/final_selection`` (the selection manager's gaze/voice
choice) and publishes the label of the selected class on ``/object_label``. It
sends the matching pick command to ``arm_controller_node`` only when
``auto_pick`` is true, because a wrong label moves the arm with no way back.
"""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from assistant_msgs.msg import ObjectSelection

from .kinematics import compute_arm_pulses
from .pick_logic import PickGate

# arm_controller_node appends this to the id when a command failed.
DONE_ERROR_SUFFIX = ":error"


class PickTriggerNode(Node):
    """Publish one slot label per stable object, and optionally pick it."""

    def __init__(self) -> None:
        super().__init__("pick_trigger_node")
        # Trained YOLO classes for tabletop manipulation
        self.declare_parameter(
            "pick_classes",
            ["sharpener", "eraser", "thermocol", "block_a", "block_b", "block_c"],
        )
        self.declare_parameter("rearm_gap_s", 3.0)
        # Safety default: labels are published, but the arm does not move.
        self.declare_parameter("auto_pick", False)
        self.declare_parameter("arm_command_timeout_s", 40.0)
        # system.yaml sets this for every node; here it makes the node a dry run.
        self.declare_parameter("mock_mode", True)
        self._pick_classes = [
            str(name) for name in self.get_parameter("pick_classes").value
        ]
        self._auto_pick = bool(self.get_parameter("auto_pick").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._command_timeout = float(self.get_parameter("arm_command_timeout_s").value)
        self._gate = PickGate(
            self._pick_classes, float(self.get_parameter("rearm_gap_s").value)
        )
        self._command_id = 0
        self._pending_id: int | None = None
        self._release_at: float | None = None
        self._last_status: str | None = None
        self._label_publisher = self.create_publisher(Int32, "/object_label", 10)
        self._command_publisher = self.create_publisher(String, "/arm/command", 10)
        self.create_subscription(
            ObjectSelection, "/final_selection", self._on_selection, 10
        )
        self.create_subscription(String, "/arm/done", self._on_arm_done, 10)
        self.create_subscription(String, "/arm/status", self._on_arm_status, 10)
        self.create_timer(0.2, self._check_timeout)
        if not self._auto_pick:
            self.get_logger().info(
                "auto_pick is false: labels are published but the arm is not "
                "commanded. Set auto_pick:=true to pick."
            )
        elif self._mock_mode:
            self.get_logger().warning(
                "auto_pick is true but mock_mode is on: labels are published and "
                "no arm command is sent."
            )
        else:
            self.get_logger().warning(
                "auto_pick is true: a gaze or voice selection will move the arm."
            )

    def _on_selection(self, message: ObjectSelection) -> None:
        """Map the selected class to a label and act on it."""
        class_name = message.class_name
        label = self._gate.accept(time.monotonic(), class_name)
        if label is None:
            if self._gate.label_of(class_name) is None:
                self.get_logger().warning(
                    f"Selected '{class_name}' has no slot; pick_classes is "
                    f"{self._pick_classes}."
                )
            return
        self.get_logger().info(
            f"Pick label {label} ({class_name}) selected by {message.source}"
        )
        self._label_publisher.publish(Int32(data=label))
        # Also when nothing is sent: the slot is busy until the arm answers or
        # arm_command_timeout_s passes.
        self._release_at = time.monotonic() + self._command_timeout
        if not self._auto_pick:
            return

        cmd_payload: dict[str, Any] | None = None
        if message.has_workspace_pose:
            ws_x = float(message.workspace_center.x)
            ws_y = float(message.workspace_center.y)
            try:
                base_p, ch1_p, ch2_p, grip_p, dist, angle_deg = compute_arm_pulses(
                    class_name, ws_x, ws_y
                )
                self.get_logger().info(
                    f"Continuous 2D pick pose for '{class_name}' at ({ws_x:.3f}, {ws_y:.3f}) m: "
                    f"dist={dist*100:.1f}cm, azimuth={angle_deg:.1f}deg -> "
                    f"base={base_p}, ch1={ch1_p}, ch2={ch2_p}, gripper={grip_p}"
                )
                self._command_id += 1
                self._pending_id = self._command_id
                cmd_payload = {
                    "id": self._command_id,
                    "command": "pick",
                    "base": base_p,
                    "link1": ch1_p,
                    "link2": ch2_p,
                    "gripper": grip_p,
                }
            except ValueError as exc:
                self.get_logger().warning(
                    f"Continuous kinematics rejected target '{class_name}': {exc}"
                )

        if cmd_payload is None:
            if label < 4:
                self._command_id += 1
                self._pending_id = self._command_id
                cmd_payload = {
                    "id": self._command_id,
                    "command": "pick",
                    "slot": label,
                }
            else:
                self.get_logger().error(
                    f"Cannot pick '{class_name}': no valid workspace pose and slot index {label} >= 4"
                )
                self._release_pick(time.monotonic())
                return

        if self._mock_mode:
            self.get_logger().info(
                f"mock_mode: pick payload {cmd_payload} logged only, no arm command sent."
            )
            return

        self._command_publisher.publish(String(data=json.dumps(cmd_payload)))

    def _on_arm_done(self, message: String) -> None:
        """Release the tracker when the pick we sent has finished or failed."""
        command_id, ok = _parse_done(message.data)
        if command_id is None:
            self.get_logger().warning(
                f"Ignoring unreadable /arm/done payload {message.data!r}"
            )
            return
        if not self._gate.busy or command_id != self._pending_id:
            return
        if not ok:
            self.get_logger().error(
                f"Arm reported pick {command_id} failed; releasing the picker."
            )
        self._release_pick(time.monotonic())

    def _on_arm_status(self, message: String) -> None:
        """Surface the arm's availability, which the pick commands depend on."""
        if message.data == self._last_status:
            return
        self._last_status = message.data
        if message.data in ("available", "mock"):
            self.get_logger().info(f"Arm status: {message.data}")
        else:
            self.get_logger().warning(
                f"Arm status: {message.data}; pick commands are refused."
            )

    def _check_timeout(self) -> None:
        """Release a pick that never got an /arm/done (arm idle or lost reply)."""
        if self._release_at is None or not self._gate.busy:
            return
        now = time.monotonic()
        if now < self._release_at:
            return
        self.get_logger().warning(
            f"No /arm/done within {self._command_timeout:.0f}s; releasing the picker."
        )
        self._release_pick(now)

    def _release_pick(self, now: float) -> None:
        """End the current pick and wait for the re-arm gap."""
        self._gate.release(now)
        self._pending_id = None
        self._release_at = None

    def shutdown_callback(self) -> None:
        """Provide a consistent node shutdown hook."""


def _parse_done(payload: str) -> tuple[int | None, bool]:
    """Split an ``<id>`` or ``<id>:error`` /arm/done payload.

    The arm controller publishes the command id with no whitespace, and appends
    ``:error`` when the Arduino refused the command. Anything else is ignored.
    """
    text = payload.strip()
    ok = not text.endswith(DONE_ERROR_SUFFIX)
    raw = text[: -len(DONE_ERROR_SUFFIX)] if not ok else text
    try:
        return int(raw), ok
    except ValueError:
        return None, ok


def main(args: list[str] | None = None) -> None:
    """Run the pick trigger node."""
    rclpy.init(args=args)
    node = PickTriggerNode()
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
