"""Launch the complete gaze-guided manipulation graph."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return the nine-node mock-safe system launch description."""
    bringup_dir = get_package_share_directory("assistant_bringup")
    config_dir = os.path.join(bringup_dir, "config")
    config = {
        name: os.path.join(config_dir, name)
        for name in (
            "system.yaml",
            "cameras.yaml",
            "perception.yaml",
            "gaze.yaml",
            "workspace.yaml",
            "robot.yaml",
        )
    }

    def node(package, executable, name, parameters):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=[config["system.yaml"]]
            + [config[item] for item in parameters],
        )

    return LaunchDescription(
        [
            node(
                "assistant_perception",
                "workspace_camera_node",
                "workspace_camera_node",
                ["cameras.yaml"],
            ),
            node(
                "assistant_perception",
                "object_detector_node",
                "object_detector_node",
                ["perception.yaml"],
            ),
            node(
                "assistant_perception",
                "workspace_mapper_node",
                "workspace_mapper_node",
                ["perception.yaml", "workspace.yaml"],
            ),
            node(
                "assistant_perception",
                "calibration_node",
                "calibration_node",
                ["perception.yaml", "workspace.yaml"],
            ),
            node(
                "assistant_interaction",
                "gaze_camera_node",
                "gaze_camera_node",
                ["cameras.yaml"],
            ),
            node(
                "assistant_interaction",
                "gaze_estimation_node",
                "gaze_estimation_node",
                ["gaze.yaml"],
            ),
            node(
                "assistant_interaction",
                "selection_manager_node",
                "selection_manager_node",
                ["gaze.yaml"],
            ),
            node(
                "assistant_motion",
                "motion_planner_node",
                "motion_planner_node",
                ["workspace.yaml", "robot.yaml"],
            ),
            node(
                "assistant_motion",
                "arm_controller_node",
                "arm_controller_node",
                ["robot.yaml"],
            ),
        ]
    )
