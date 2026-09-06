import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Generate the system launch description for the multimodal assistant.

    Launches all nodes from the perception, interaction, and motion packages.
    Parameters are loaded from ``config/system.yaml`` (shared across nodes).

    :return: A populated LaunchDescription.
    """
    bringup_dir = get_package_share_directory("assistant_bringup")

    config_path = os.path.join(bringup_dir, "config", "system.yaml")
    example_map_path = os.path.join(
        bringup_dir, "config", "object_map_example.json"
    )

    return LaunchDescription(
        [
            # --- Perception layer ---
            Node(
                package="assistant_perception",
                executable="workspace_mapper_node",
                name="workspace_mapper_node",
                output="screen",
                parameters=[
                    config_path,
                    {"example_map_path": example_map_path},
                ],
            ),
            # --- Interaction layer ---
            Node(
                package="assistant_interaction",
                executable="voice_node",
                name="voice_node",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="assistant_interaction",
                executable="gaze_node",
                name="gaze_node",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="assistant_interaction",
                executable="selection_manager_node",
                name="selection_manager_node",
                output="screen",
                parameters=[config_path],
            ),
            # --- Motion layer ---
            Node(
                package="assistant_motion",
                executable="motion_planner_node",
                name="motion_planner_node",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="assistant_motion",
                executable="arm_controller_node",
                name="arm_controller_node",
                output="screen",
                parameters=[config_path],
            ),
        ]
    )