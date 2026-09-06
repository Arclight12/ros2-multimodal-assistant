"""Motion planner node: plans and supervises grasps.

The motion planner subscribes to the final object selection and looks up the
object's target pose from the runtime ``object_map.json``. It provides an action server
``/execute_grasp`` that steps through planning, motion, and grasping using a
MoveIt 2 integration point (currently a skeleton).

This node contains no perception or interaction logic.

Topics
------
* Subscribes to ``/final_selection`` (``assistant_msgs/ObjectSelection``).

Actions
-------
* ``/execute_grasp`` (``assistant_msgs/ExecuteGrasp``).
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from assistant_msgs.action import ExecuteGrasp
from assistant_msgs.msg import ObjectSelection


class MotionPlannerNode(Node):
    """Plan and execute arm motion toward a selected object."""

    def __init__(self) -> None:
        """Initialize the motion planner node."""
        super().__init__("motion_planner_node")

        self.declare_parameter(
            "object_map_path",
            "~/.ros/ros2_multimodal_assistant/object_map.json",
        )
        self.declare_parameter("moveit_enabled", False)
        self.declare_parameter("mock_mode", True)

        self._object_map_path: str = os.path.expanduser(
            self.get_parameter("object_map_path").get_parameter_value().string_value
        )
        self._moveit_enabled: bool = (
            self.get_parameter("moveit_enabled").get_parameter_value().bool_value
        )
        self._mock_mode: bool = (
            self.get_parameter("mock_mode").get_parameter_value().bool_value
        )

        self._object_map: Dict[str, Any] = {}
        self._object_map_meta: Dict[str, Any] = {}

        # Subscriber to the final selection output from the interaction layer.
        self._selection_sub = self.create_subscription(
            ObjectSelection, "/final_selection", self._on_final_selection, 10
        )

        # Action server exposing the grasp capability.
        self._action_server = ActionServer(
            self,
            ExecuteGrasp,
            "/execute_grasp",
            self._execute_grasp_callback,
        )

        self.get_logger().info(
            f"MotionPlannerNode initialized | moveit_enabled={self._moveit_enabled} "
            f"| mock_mode={self._mock_mode} "
            f"| object_map={self._object_map_path}"
        )

    def _load_object_map(self) -> None:
        """(Re)load the object map and its provenance metadata from disk.

        The runtime map file follows the ``{_meta, objects}`` schema. The
        ``_meta`` block records whether the coordinates are simulated (mock)
        and whether they have been calibrated to the real workspace.

        Reloading happens on demand at grasp time so that a scan performed
        after node startup is always reflected, and the planner never depends
        on a startup-time race with the workspace mapper.
        """
        path = Path(self._object_map_path)
        if not path.exists():
            self.get_logger().warn(f"Object map not found at {path}")
            self._object_map, self._object_map_meta = {}, {}
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                document = json.load(handle)
        except (json.JSONDecodeError, OSError) as exc:
            self.get_logger().warn(f"Failed to read object map {path}: {exc}")
            self._object_map, self._object_map_meta = {}, {}
            return
        meta = document.get("_meta", {})
        objects = document.get("objects", document)
        if "objects" not in document:
            self.get_logger().info("Object map uses legacy flat schema (no _meta).")
        if not isinstance(objects, dict):
            self.get_logger().warn(f"Object map 'objects' section malformed in {path}")
            objects = {}
        if meta.get("mock", True):
            self.get_logger().warn(
                "Loaded object map is marked as MOCK/UNcalibrated. "
                "No real arm motion will be attempted."
            )
        self._object_map = dict(objects)
        self._object_map_meta = dict(meta)

    def _on_final_selection(self, msg: ObjectSelection) -> None:
        """React to a newly confirmed final selection.

        :param msg: The final selection message.
        """
        self.get_logger().info(
            f"Final selection received: '{msg.object_id}'. "
            f"Ready to grasp when requested via /execute_grasp."
        )

    def _get_feedback(self) -> ExecuteGrasp.Feedback:
        """Instantiate a fresh feedback message.

        :return: An empty feedback message.
        """
        return ExecuteGrasp.Feedback()

    def _execute_grasp_callback(self, goal_handle: Any) -> ExecuteGrasp.Result:
        """Handle the ``/execute_grasp`` action goal.

        :param goal_handle: The action goal handle.
        :return: The action result.
        """
        goal = goal_handle.request
        self.get_logger().info(f"Received grasp goal for '{goal.object_id}'.")

        feedback = self._get_feedback()
        feedback.current_state = "lookup_pose"
        goal_handle.publish_feedback(feedback)

        pose = self._lookup_pose(goal.object_id)
        if pose is None:
            result = ExecuteGrasp.Result()
            result.success = False
            result.message = f"Object '{goal.object_id}' not found in object map."
            goal_handle.abort()
            return result

        if (
            self._moveit_enabled
            and not self._object_map_meta.get("calibrated", False)
        ):
            result = ExecuteGrasp.Result()
            result.success = False
            result.message = (
                f"Refusing grasp of '{goal.object_id}': object map is not "
                "calibrated. Run a real (non-mock) calibrated scan first."
            )
            goal_handle.abort()
            return result

        feedback.current_state = "planning"
        goal_handle.publish_feedback(feedback)

        if self._moveit_enabled:
            feedback.current_state = "moving"
            goal_handle.publish_feedback(feedback)
            # TODO(motion-team): integrate MoveIt 2 planning and execution here.
            self.get_logger().info("MoveIt planning/execution is a placeholder (TODO).")
        elif self._mock_mode:
            feedback.current_state = "executing_simulated"
            goal_handle.publish_feedback(feedback)
            self.get_logger().warn(
                "mock_mode=True: grasp is SIMULATED. No arm motion performed."
            )
        else:
            self.get_logger().warn(
                "moveit_enabled=False and mock_mode=False; skipping motion. "
                "Placeholder path."
            )

        result = ExecuteGrasp.Result()
        result.success = True
        if self._mock_mode:
            result.message = (
                f"Mock grasp of '{goal.object_id}' completed (SIMULATED, "
                "no robot motion)."
            )
        else:
            result.message = f"Grasp of '{goal.object_id}' completed (skeleton)."
        goal_handle.succeed()
        return result

    def _lookup_pose(self, object_id: str) -> Optional[Dict[str, Any]]:
        """Look up an object's pose in the current object map.

        The map is (re)loaded from disk so a scan performed after node startup
        is always reflected.

        :param object_id: The object identifier.
        :return: The object's pose dict, or None if not found.
        """
        self._load_object_map()
        return self._object_map.get(object_id)

    def shutdown_callback(self) -> None:
        """Perform clean shutdown of the motion planner."""
        self.get_logger().info("Shutting down motion planner node.")


def main(args: list[str] | None = None) -> None:
    """Entry point for the motion planner node executable."""
    rclpy.init(args=args)
    node = MotionPlannerNode()

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
