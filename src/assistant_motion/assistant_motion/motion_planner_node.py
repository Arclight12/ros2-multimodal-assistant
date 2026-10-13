"""Guarded grasp orchestration for live selected detections."""

from __future__ import annotations

import json
import threading
import time
from typing import Any

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from assistant_msgs.action import ExecuteGrasp
from assistant_msgs.msg import DetectedObjectArray, ObjectSelection

from .arm_kinematics import ArmGeometry, solve_ik
from .grasp_safety import GraspSafetyState, validate_real_grasp
from .workspace_checks import valid_pose


class MotionPlannerNode(Node):
    """Validate a live target and orchestrate an explicit grasp action."""

    def __init__(self) -> None:
        super().__init__("motion_planner_node")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("workspace_width_m", 0.4)
        self.declare_parameter("workspace_height_m", 0.3)
        self.declare_parameter("detection_timeout_seconds", 1.0)
        self.declare_parameter("pregrasp_offset_m", 0.04)
        self.declare_parameter("arm_command_timeout_s", 15.0)
        defaults = ArmGeometry()
        self.declare_parameter(
            "arm_shoulder_height_m", defaults.shoulder_height_m
        )
        self.declare_parameter("arm_upper_arm_m", defaults.upper_arm_m)
        self.declare_parameter("arm_forearm_m", defaults.forearm_m)
        self.declare_parameter("arm_gripper_m", defaults.gripper_m)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._motion_enabled = bool(
            self.get_parameter("motion_enabled").value
        )
        self._workspace_width = float(
            self.get_parameter("workspace_width_m").value
        )
        self._workspace_height = float(
            self.get_parameter("workspace_height_m").value
        )
        self._detection_timeout = float(
            self.get_parameter("detection_timeout_seconds").value
        )
        self._pregrasp_offset = float(
            self.get_parameter("pregrasp_offset_m").value
        )
        self._command_timeout = float(
            self.get_parameter("arm_command_timeout_s").value
        )
        self._geometry = ArmGeometry(
            shoulder_height_m=float(
                self.get_parameter("arm_shoulder_height_m").value
            ),
            upper_arm_m=float(self.get_parameter("arm_upper_arm_m").value),
            forearm_m=float(self.get_parameter("arm_forearm_m").value),
            gripper_m=float(self.get_parameter("arm_gripper_m").value),
        )
        self._command_id = 0
        self._done: set[str] = set()
        self._done_cond = threading.Condition()
        # Reentrant so /arm/done is handled while a grasp action blocks.
        self._group = ReentrantCallbackGroup()
        self._detections: DetectedObjectArray | None = None
        self._selection: ObjectSelection | None = None
        self._hardware_available = False
        self._command_publisher = self.create_publisher(
            String, "/arm/command", 10
        )
        self._detection_subscription = self.create_subscription(
            DetectedObjectArray,
            "/object_detections",
            self._on_detections,
            10,
        )
        self._selection_subscription = self.create_subscription(
            ObjectSelection,
            "/final_selection",
            self._on_selection,
            10,
        )
        self._status_subscription = self.create_subscription(
            String,
            "/arm/status",
            self._on_arm_status,
            10,
            callback_group=self._group,
        )
        self._done_subscription = self.create_subscription(
            String,
            "/arm/done",
            self._on_arm_done,
            10,
            callback_group=self._group,
        )
        self._action_server = ActionServer(
            self,
            ExecuteGrasp,
            "/execute_grasp",
            self._execute_grasp_callback,
            callback_group=self._group,
        )

    def _on_detections(self, message: DetectedObjectArray) -> None:
        """Cache the newest live scene."""
        self._detections = message

    def _on_selection(self, message: ObjectSelection) -> None:
        """Cache the newest authoritative selection."""
        self._selection = message

    def _on_arm_status(self, message: String) -> None:
        """Update hardware availability from the isolated arm node."""
        self._hardware_available = message.data == "available"

    def _on_arm_done(self, message: String) -> None:
        """Wake the grasp sequence when the arm acknowledges a command."""
        with self._done_cond:
            self._done.add(message.data)
            self._done_cond.notify_all()

    def _execute_grasp_callback(
        self, goal_handle: Any
    ) -> ExecuteGrasp.Result:
        """Run the validated grasp sequence."""
        goal = goal_handle.request
        self._feedback(goal_handle, "target_received")
        detection = self._find_live_detection(goal.object_id)
        if detection is None:
            return self._abort(
                goal_handle, "selected object is not in live detections"
            )
        if self._mock_mode:
            return self._run_mock_grasp(goal_handle, goal.object_id)

        self._feedback(goal_handle, "validating")
        pose = self._pose_tuple(detection, goal)
        stages = self._plan_stages(pose) if valid_pose(pose) else None
        state = GraspSafetyState(
            mock_mode=False,
            workspace_calibrated=bool(
                self._detections and self._detections.calibrated
            )
            and bool(
                detection.has_workspace_pose and detection.has_robot_pose
            ),
            stable_selection=bool(
                self._selection
                and self._selection.object_id == goal.object_id
            ),
            target_pose_valid=valid_pose(pose),
            target_in_workspace=self._within_workspace(detection),
            target_reachable=stages is not None,
            hardware_available=self._hardware_available,
            motion_enabled=self._motion_enabled,
        )
        decision = validate_real_grasp(state)
        if not decision.allowed:
            return self._abort(
                goal_handle, f"refusing real grasp: {decision.reason}"
            )

        self._done.clear()
        for stage, command in stages:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, "grasp canceled")
            self._feedback(goal_handle, stage)
            error = self._send_and_wait(command)
            if error:
                return self._abort(goal_handle, f"{stage} failed: {error}")
        self._feedback(goal_handle, "complete")
        goal_handle.succeed()
        return self._result(True, f"Grasp of '{goal.object_id}' completed.")

    def _run_mock_grasp(
        self, goal_handle: Any, object_id: str
    ) -> ExecuteGrasp.Result:
        """Exercise every grasp stage without publishing hardware commands."""
        for stage in (
            "validating",
            "opening_gripper",
            "moving_to_pregrasp",
            "approaching",
            "closing_gripper",
            "lifting",
            "returning",
            "complete",
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, "mock grasp canceled")
            self._feedback(goal_handle, stage)
        goal_handle.succeed()
        return self._result(
            True,
            f"Mock grasp of '{object_id}' completed "
            "(SIMULATED, no robot motion).",
        )

    def _find_live_detection(self, object_id: str):
        """Return an object while the latest live frame is current."""
        if not object_id or self._detections is None:
            return None
        stamp = self._detections.header.stamp
        if stamp.sec or stamp.nanosec:
            age = (
                self.get_clock().now().nanoseconds
                - (stamp.sec * 1_000_000_000 + stamp.nanosec)
            ) / 1_000_000_000.0
            if age > self._detection_timeout:
                return None
        return next(
            (
                item
                for item in self._detections.objects
                if item.object_id == object_id
            ),
            None,
        )

    def _pose_tuple(
        self, detection, goal
    ) -> tuple[float, float, float] | None:
        """Prefer live mapped pose and use a goal pose only as fallback."""
        if detection.has_robot_pose:
            pose = detection.robot_pose.position
            return (float(pose.x), float(pose.y), float(pose.z))
        if goal.has_target_pose:
            pose = goal.target_pose.position
            return (float(pose.x), float(pose.y), float(pose.z))
        return None

    def _within_workspace(self, detection) -> bool:
        """Validate the workspace-local position supplied by perception."""
        if not detection.has_workspace_pose:
            return False
        return (
            0.0 <= detection.workspace_center.x <= self._workspace_width
            and 0.0 <= detection.workspace_center.y <= self._workspace_height
        )

    def _plan_stages(self, pose) -> list[tuple[str, dict]] | None:
        """Solve IK for every waypoint up front; None if any is unreachable."""
        above = (pose[0], pose[1], pose[2] + self._pregrasp_offset)
        pregrasp = solve_ik(*above, geometry=self._geometry)
        grasp = solve_ik(*pose, geometry=self._geometry)
        if pregrasp is None or grasp is None:
            return None

        def move(angles) -> dict:
            return {
                "command": "move_joints",
                "angles": [
                    angles.base,
                    angles.shoulder,
                    angles.elbow,
                    angles.wrist,
                ],
            }

        return [
            ("opening_gripper", {"command": "open_gripper"}),
            ("moving_to_pregrasp", move(pregrasp)),
            ("approaching", move(grasp)),
            ("closing_gripper", {"command": "close_gripper"}),
            ("lifting", move(pregrasp)),
            ("returning", {"command": "home"}),
        ]

    def _send_and_wait(self, command: dict) -> str | None:
        """Publish one arm command; return an error, or None once acked."""
        self._command_id += 1
        command_id = str(self._command_id)
        self._publish_command({**command, "id": self._command_id})
        deadline = time.monotonic() + self._command_timeout
        with self._done_cond:
            while True:
                if command_id in self._done:
                    return None
                if f"{command_id}:error" in self._done:
                    return "arm rejected the command"
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return "arm did not acknowledge in time"
                self._done_cond.wait(remaining)

    def _publish_command(self, command: dict) -> None:
        """Send a hardware-neutral command to the arm adapter."""
        self._command_publisher.publish(String(data=json.dumps(command)))

    @staticmethod
    def _feedback(goal_handle: Any, stage: str) -> None:
        feedback = ExecuteGrasp.Feedback()
        feedback.current_state = stage
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(success: bool, message: str) -> ExecuteGrasp.Result:
        result = ExecuteGrasp.Result()
        result.success = success
        result.message = message
        return result

    def _abort(self, goal_handle: Any, message: str) -> ExecuteGrasp.Result:
        goal_handle.abort()
        return self._result(False, message)

    def shutdown_callback(self) -> None:
        """Provide a consistent node shutdown hook."""


def main(args: list[str] | None = None) -> None:
    """Run the motion planner node."""
    rclpy.init(args=args)
    node = MotionPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_callback()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
