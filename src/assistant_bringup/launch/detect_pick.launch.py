"""Launch the gaze/voice pick path on one machine (the Pi).

Detector, gaze, voice and the selection manager choose an object;
``pick_trigger_node`` turns its class into a slot label and
``arm_controller_node`` sends it to the Arduino. The motion planner and
workspace calibration are not started. The arm serial port comes from
``robot.yaml``.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Return the nine-node gaze/voice pick launch description."""
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
            "voice.yaml",
            "pick.yaml",
        )
    }

    def node(package, executable, name, parameters, overrides=None, condition=None):
        kwargs = {}
        if condition is not None:
            kwargs["condition"] = condition
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
            **kwargs,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mock_mode", default_value="false"),
            DeclareLaunchArgument(
                "workspace_camera_url",
                default_value="http://192.168.1.2:8080/video",
            ),
            DeclareLaunchArgument(
                "voice_stream_url",
                default_value="http://192.168.1.2:8080/audio.wav",
            ),
            DeclareLaunchArgument(
                "enable_gaze",
                default_value="false",
                description="Enable webcam-based gaze tracking",
            ),
            DeclareLaunchArgument("gaze_camera_index", default_value="0"),
            DeclareLaunchArgument("gaze_camera_device", default_value=""),
            DeclareLaunchArgument("gaze_camera_backend", default_value="auto"),
            DeclareLaunchArgument("auto_pick", default_value="true"),
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
                "workspace_mapper_node",
                "workspace_mapper_node",
                ["perception.yaml", "workspace.yaml"],
            ),
            node(
                "assistant_interaction",
                "gaze_camera_node",
                "gaze_camera_node",
                ["cameras.yaml"],
                [
                    {
                        "gaze_camera_index": ParameterValue(
                            LaunchConfiguration("gaze_camera_index"),
                            value_type=int,
                        ),
                        "gaze_camera_device": ParameterValue(
                            LaunchConfiguration("gaze_camera_device"),
                            value_type=str,
                        ),
                        "gaze_camera_backend": ParameterValue(
                            LaunchConfiguration("gaze_camera_backend"),
                            value_type=str,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("enable_gaze")),
            ),
            node(
                "assistant_interaction",
                "gaze_estimation_node",
                "gaze_estimation_node",
                ["gaze.yaml"],
                condition=IfCondition(LaunchConfiguration("enable_gaze")),
            ),
            node(
                "assistant_interaction",
                "selection_manager_node",
                "selection_manager_node",
                ["gaze.yaml"],
            ),
            node(
                "assistant_interaction",
                "voice_node",
                "voice_node",
                ["voice.yaml"],
                [
                    {
                        "voice_stream_url": ParameterValue(
                            LaunchConfiguration("voice_stream_url"), value_type=str
                        )
                    }
                ],
            ),
            node(
                "assistant_motion",
                "pick_trigger_node",
                "pick_trigger_node",
                ["pick.yaml"],
                [
                    {
                        "auto_pick": ParameterValue(
                            LaunchConfiguration("auto_pick"), value_type=bool
                        )
                    }
                ],
            ),
            node(
                "assistant_motion",
                "arm_controller_node",
                "arm_controller_node",
                ["robot.yaml"],
            ),
        ]
    )
