"""Hardware-isolated arm command adapter with deterministic mock mode.

Every frame built by :mod:`arm_protocol` goes out one command at a time. The
commands behave the same way, including ``pick`` (``D,<slot>``): each waits for
the Arduino's ``OK`` / ``CYCLE_COMPLETE`` or ``ERR,<reason>`` / ``CYCLE_ABORTED``
before another command is accepted, and reports it on ``/arm/done`` as ``<id>``
or ``<id>:error``. Progress lines (``PICKUP_START`` ...) are ignored."""

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
from .workspace_checks import clamp_command, load_channel_limits

# The Uno resets when the port opens, so wait for its READY line.
READY_TIMEOUT_S = 6.0


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
        # robot.yaml channel_limits: last ROS-side clamp before the Arduino,
        # which keeps its own out_of_range check as the final gate.
        self._channel_limits = load_channel_limits(self)
        self._clamp_logged = False
        self._serial = None
        self._status = "unavailable"
        self._opened_at = 0.0
        self._pending: deque = deque()
        self._rx = b""
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
        self._rx = b""
        self._status = "connecting"

    def _on_command(self, message: String) -> None:
        """Validate, clamp and execute one hardware-neutral command."""
        try:
            command = parse_command(message.data)
            command, notes = clamp_command(command, self._channel_limits)
            if notes and not self._clamp_logged:
                self._clamp_logged = True
                self.get_logger().warning(
                    "Clamped an arm command into the robot.yaml channel limits: "
                    + ", ".join(notes)
                )
            frame = command_frame(command)
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
        if command.get("command") == "stop":
            # The Arduino never replies to a freeze; the command it interrupts
            # answers ERR,stopped instead. So a stop is acknowledged here and
            # must not join the queue of commands waiting for a reply.
            self._serial.write(frame.encode("ascii"))
            self._serial.flush()
            self._publish_done(command_id, ok=True)
            return
        if self._pending:
            # The firmware discards everything but 'x' while a move runs, so a
            # second command would never be answered and would shift every
            # later OK onto the wrong id. Refuse it instead.
            self.get_logger().error("Rejected arm command: previous one still running.")
            self._publish_done(command_id, ok=False)
            return
        self._pending.append(command_id)
        self.get_logger().info(f"Sending serial command to arm: {frame.strip()!r}")
        self._serial.write(frame.encode("ascii"))
        self._serial.flush()

    def _poll(self) -> None:
        """Read Arduino replies and watch for the boot handshake."""
        if self._serial is None:
            return
        try:
            waiting = self._serial.in_waiting
            if waiting:
                # Buffer raw bytes so a reply split across reads is not lost.
                self._rx += self._serial.read(waiting)
            while b"\n" in self._rx:
                raw, self._rx = self._rx.split(b"\n", 1)
                self._handle_line(raw.decode("ascii", "ignore").strip())
            if len(self._rx) > 256:  # no newline in a long run: line noise
                self._rx = b""
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
        if not line:
            return
        self.get_logger().info(f"Arduino replied: {line}")
        if line in ("READY", "ARM_READY"):
            self._status = "available"
            self.get_logger().info("Arm hardware is ready.")
        elif line in ("OK", "CYCLE_COMPLETE") and self._pending:
            self._publish_done(self._pending.popleft(), ok=True)
        elif line == "CYCLE_ABORTED":
            self.get_logger().error("Arduino aborted the pick cycle.")
            if self._pending:
                self._publish_done(self._pending.popleft(), ok=False)
        elif line == "ERR,bad_calibration":
            self.get_logger().error("Arduino refused its calibration; arm disabled.")
            self._status = "unavailable"
        elif line == "ERR,pca9685_not_found":
            self.get_logger().error(
                "PCA9685 servo driver board not detected on I2C (0x40)! "
                "Check PCA9685 power supply and wiring: VCC (5V), GND, SDA (A4), SCL (A5)."
            )
            self._status = "unavailable"
        elif line.startswith("ERR"):
            self.get_logger().error(f"Arduino reported: {line}")
            if self._pending:
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
