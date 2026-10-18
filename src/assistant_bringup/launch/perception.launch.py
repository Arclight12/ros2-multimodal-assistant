"""Launch only the workspace camera, calibration, and mapping perception nodes."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Return the perception-only launch description for camera and calibration testing."""
    bringup_dir = get_package_share_directory("assistant_bringup")
    config_dir = os.path.join(bringup_dir, "config")
    config = {
        name: os.path.join(config_dir, name)
        for name in (
            "system.yaml",
            "cameras.yaml",
            "perception.yaml",
            "workspace.yaml",
        )
    }

    def node(package, executable, name, parameters, overrides=None):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=[config["system.yaml"]]
            + [config[item] for item in parameters]
            + [
                {
                    "mock_mode": ParameterValue(
                        LaunchConfiguration("mock_mode"), value_type=bool
                    )
                }
            ]
            + (overrides or []),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mock_mode", default_value="false"),
            DeclareLaunchArgument(
                "workspace_camera_url",
                default_value="http://192.168.1.2:8080/video",
                description="Phone workspace MJPEG stream URL",
            ),
            node(
                "assistant_perception",
                "workspace_camera_node",
                "workspace_camera_node",
                ["cameras.yaml"],
                [
                    {
                        "camera_url": ParameterValue(
                            LaunchConfiguration("workspace_camera_url"),
                            value_type=str,
                        )
                    }
                ],
            ),
            node(
                "assistant_perception",
                "object_detector_node",
                "object_detector_node",
                ["perception.yaml"],
            ),
            node(
                "assistant_perception",
                "calibration_node",
                "calibration_node",
                ["perception.yaml", "workspace.yaml"],
            ),
            node(
                "assistant_perception",
                "workspace_mapper_node",
                "workspace_mapper_node",
                ["perception.yaml", "workspace.yaml"],
            ),
        ]
    )
