"""Hardware-isolated arm command adapter with deterministic mock mode."""

from __future__ import annotations

import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - optional hardware dependency
    serial = None

from .arm_protocol import command_frame, parse_command

# The Uno resets when the port opens, so wait for its READY line.
READY_TIMEOUT_S = 6.0


class ArmControllerNode(Node):
    """Translate abstract motion commands into mock or serial operations."""

    def __init__(self) -> None:
        super().__init__("arm_controller_node")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("servo_offsets_deg", [90.0, 90.0, 90.0, 90.0])
        self.declare_parameter("servo_signs", [1.0, 1.0, 1.0, 1.0])
        self._serial_port = os.path.expanduser(
            str(self.get_parameter("serial_port").value)
        )
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._offsets = [
            float(v) for v in self.get_parameter("servo_offsets_deg").value
        ]
        self._signs = [float(v) for v in self.get_parameter("servo_signs").value]
        self._serial = None
        self._status = "unavailable"
        self._opened_at = 0.0
        self._pending: deque = deque()
        self._status_publisher = self.create_publisher(String, "/arm/status", 10)
        self._done_publisher = self.create_publisher(String, "/arm/done", 10)
        self._subscription = self.create_subscription(
            String, "/arm/command", self._on_command, 10
        )
        self._open_hardware()
        # Republished so late subscribers (the planner) still see the state.
        self.create_timer(0.05, self._poll)
        self.create_timer(1.0, self._publish_status)

    def _open_hardware(self) -> None:
        """Open serial only when explicitly outside mock mode."""
        if self._mock_mode:
            self._status = "mock"
            self.get_logger().warning(
                "Arm controller is in mock mode; no commands reach hardware."
            )
            return
        if serial is None:
            self.get_logger().error(
                "pyserial is unavailable; arm hardware is disabled."
            )
            return
        if not self._serial_port:
            self.get_logger().error("serial_port is empty; arm hardware is disabled.")
            return
        try:
            self._serial = serial.Serial(
                self._serial_port, self._baudrate, timeout=0.05
            )
        except (OSError, serial.SerialException) as exc:
            self.get_logger().error(f"Unable to open arm serial port: {exc}")
            return
        self._opened_at = time.monotonic()
        self._status = "connecting"

    def _on_command(self, message: String) -> None:
        """Validate and execute one hardware-neutral command."""
        try:
            command = parse_command(message.data)
            frame = command_frame(command, self._offsets, self._signs)
        except ValueError as exc:  # includes json.JSONDecodeError
            self.get_logger().error(f"Rejected malformed arm command: {exc}")
            self._publish_done(command_id_of(message.data), ok=False)
            return
        command_id = command.get("id")
        if self._mock_mode:
            self.get_logger().info(f"MOCK arm command: {frame.strip()}")
            self._publish_done(command_id, ok=True)
            return
        if self._status != "available":
            self.get_logger().error("Rejected arm command: hardware is unavailable.")
            self._publish_done(command_id, ok=False)
            return
        self._pending.append(command_id)
        self._serial.write(frame.encode("ascii"))

    def _poll(self) -> None:
        """Read Arduino replies and watch for the boot handshake."""
        if self._serial is None:
            return
        try:
            while self._serial.in_waiting:
                # shortcut: a partial line is dropped on read timeout.
                line = self._serial.readline().decode("ascii", "ignore")
                self._handle_line(line.strip())
        except (OSError, serial.SerialException) as exc:
            self.get_logger().error(f"Arm serial link lost: {exc}")
            self._serial.close()
            self._serial = None
            self._status = "unavailable"
            while self._pending:
                self._publish_done(self._pending.popleft(), ok=False)
            return
        if (
            self._status == "connecting"
            and time.monotonic() - self._opened_at > READY_TIMEOUT_S
        ):
            self.get_logger().error(
                "Arduino did not send READY; is the arm firmware flashed?"
            )
            self._status = "unavailable"

    def _handle_line(self, line: str) -> None:
        """React to one Arduino reply line."""
        if line == "READY":
            self._status = "available"
            self.get_logger().info("Arm hardware is ready.")
        elif line == "OK" and self._pending:
            self._publish_done(self._pending.popleft(), ok=True)
        elif line.startswith("ERR") and self._pending:
            self.get_logger().error(f"Arduino rejected command: {line}")
            self._publish_done(self._pending.popleft(), ok=False)

    def _publish_done(self, command_id, ok: bool) -> None:
        """Acknowledge one command so the planner can sequence stages."""
        if command_id is None:
            return
        suffix = "" if ok else ":error"
        self._done_publisher.publish(String(data=f"{command_id}{suffix}"))

    def _publish_status(self) -> None:
        """Publish current hardware availability."""
        self._status_publisher.publish(String(data=self._status))

    def shutdown_callback(self) -> None:
        """Close the serial port."""
        if self._serial is not None:
            self._serial.close()


def command_id_of(payload: str):
    """Best-effort id extraction so malformed commands still get a reply."""
    try:
        return parse_command(payload).get("id")
    except ValueError:
        return None


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
