"""Arm controller node: serial abstraction for the Arduino-controlled arm.

This node provides a thin serial communication layer toward the Arduino Uno
that drives the servo-based robotic arm and gripper. It translates high-level
commands (move to pose, open/close gripper) into serial protocol messages that
the Arduino firmware will interpret.

The actual Arduino firmware and MoveIt are intentionally not implemented here;
this node is a communication placeholder.

This node contains no perception or interaction logic.

Topics
------
* Subscribes to ``/final_selection`` (``assistant_msgs/ObjectSelection``) for
  activation, though actual actuation is driven by the motion planner.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from assistant_msgs.msg import ObjectSelection

# Protocol markers for simplicity. Placeholder framing for the serial link.
_CMD_GOTO = "G"
_CMD_GRIPPER_OPEN = "O"
_CMD_GRIPPER_CLOSE = "C"


class ArmControllerNode(Node):
    """Abstraction layer for serial communication with the Arduino arm."""

    def __init__(self) -> None:
        """Initialize the arm controller node."""
        super().__init__("arm_controller_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        self._serial_port: str = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self._baudrate: int = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )

        self._serial = None
        self._try_open_serial()

        self._selection_sub = self.create_subscription(
            ObjectSelection, "/final_selection", self._on_final_selection, 10
        )

        self.get_logger().info(
            f"ArmControllerNode initialized | port={self._serial_port} | "
            f"baud={self._baudrate}"
        )

    def _try_open_serial(self) -> None:
        """Attempt to open the serial port to the Arduino.

        If no device is present the node logs a warning and continues running so
        the rest of the system can still be exercised.
        """
        # TODO(motion-team): use pyserial to open ``self._serial_port`` at
        # ``self._baudrate`` once hardware is connected.
        self.get_logger().warn(
            f"Serial connection to {self._serial_port} not established (placeholder)."
        )

    def _on_final_selection(self, msg: ObjectSelection) -> None:
        """Respond to a final selection by queuing a command.

        :param msg: The final selection message.
        """
        self.get_logger().info(f"Arm controller noted final selection: {msg.object_id}")

    def _send_command(self, command: str, arguments: list[float] | None = None) -> None:
        """Send a serial command frame to the Arduino.

        :param command: The single-character command marker.
        :param arguments: Optional numeric arguments for the command.
        """
        parts = [command]
        if arguments:
            parts.extend(str(arg) for arg in arguments)
        frame = ",".join(parts) + "\n"
        # TODO(motion-team): write the frame to the opened serial port.
        self.get_logger().debug(f"Serial frame (not sent - placeholder): {frame!r}")

    def _move_to_pose(self, x: float, y: float, z: float) -> None:
        """Command the arm to a Cartesian position.

        :param x: X coordinate.
        :param y: Y coordinate.
        :param z: Z coordinate.
        """
        self._send_command(_CMD_GOTO, [x, y, z])

    def _set_gripper(self, closed: bool) -> None:
        """Open or close the gripper.

        :param closed: True to close the gripper, False to open it.
        """
        self._send_command(_CMD_GRIPPER_CLOSE if closed else _CMD_GRIPPER_OPEN)

    def shutdown_callback(self) -> None:
        """Perform clean shutdown, closing the serial connection."""
        self.get_logger().info("Shutting down arm controller node.")
        if self._serial is not None:
            # TODO(motion-team): close the serial port gracefully here.
            pass


def main(args: list[str] | None = None) -> None:
    """Entry point for the arm controller node executable."""
    rclpy.init(args=args)
    node = ArmControllerNode()

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
