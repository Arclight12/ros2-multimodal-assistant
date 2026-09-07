"""Hardware-isolated arm command adapter with deterministic mock mode."""

from __future__ import annotations

import json
import math
import os
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - optional hardware dependency
    serial = None


class ArmControllerNode(Node):
    """Translate abstract motion commands into mock or serial operations."""

    def __init__(self) -> None:
        super().__init__("arm_controller_node")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("mock_mode", True)
        self._serial_port = os.path.expanduser(
            str(self.get_parameter("serial_port").value)
        )
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._serial = None
        self._status_publisher = self.create_publisher(
            String, "/arm/status", 10
        )
        self._subscription = self.create_subscription(
            String, "/arm/command", self._on_command, 10
        )
        self._open_hardware()

    def _open_hardware(self) -> None:
        """Open serial only when explicitly outside mock mode."""
        if self._mock_mode:
            self._publish_status("mock")
            self.get_logger().warning(
                "Arm controller is in mock mode; no commands reach hardware."
            )
            return
        if serial is None:
            self.get_logger().error(
                "pyserial is unavailable; arm hardware is disabled."
            )
            self._publish_status("unavailable")
            return
        if not self._serial_port:
            self.get_logger().error(
                "serial_port is empty; arm hardware is disabled."
            )
            self._publish_status("unavailable")
            return
        try:
            self._serial = serial.Serial(
                self._serial_port, self._baudrate, timeout=1.0
            )
        except (OSError, serial.SerialException) as exc:
            self.get_logger().error(f"Unable to open arm serial port: {exc}")
            self._publish_status("unavailable")
            return
        self._publish_status("available")

    def _on_command(self, message: String) -> None:
        """Validate and execute one hardware-neutral command."""
        try:
            command = json.loads(message.data)
            frame = self._command_frame(command)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Rejected malformed arm command: {exc}")
            return
        if self._mock_mode:
            self.get_logger().info(f"MOCK arm command: {frame.strip()}")
            return
        if self._serial is None:
            self.get_logger().error(
                "Rejected arm command: hardware is unavailable."
            )
            return
        self._serial.write(frame.encode("ascii"))

    @staticmethod
    def _command_frame(command: Any) -> str:
        """Convert allowlisted JSON commands to the simple serial protocol."""
        if not isinstance(command, dict) or not isinstance(
            command.get("command"), str
        ):
            raise ValueError(
                "command must be an object with a string command"
            )
        name = command["command"]
        if name in {"plan", "home", "open_gripper", "close_gripper"}:
            markers = {
                "plan": "P",
                "home": "H",
                "open_gripper": "O",
                "close_gripper": "C",
            }
            return markers[name] + "\n"
        if name != "move_to":
            raise ValueError(f"unknown command {name!r}")
        pose = command.get("pose")
        if not isinstance(pose, (list, tuple)) or len(pose) != 3:
            raise ValueError("move_to requires a three-value pose")
        values = tuple(float(value) for value in pose)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("move_to pose must be finite")
        return "G," + ",".join(str(value) for value in values) + "\n"

    def _publish_status(self, status: str) -> None:
        """Publish current hardware availability."""
        self._status_publisher.publish(String(data=status))

    def shutdown_callback(self) -> None:
        """Close the serial port."""
        if self._serial is not None:
            self._serial.close()


def main(args: list[str] | None = None) -> None:
    """Run the arm controller node."""
    rclpy.init(args=args)
    node = ArmControllerNode()
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
