"""
tag_tracker.launch.py
---------------------
Launch the C++ tag_tracker_node.

Usage:
  ros2 launch tag_tracker tag_tracker.launch.py

Overrides (all optional):
  config_file:=<path>          default: share/tag_tracker/config/tag_tracker.yaml
  image_topic:=<topic>         default: /camera/image_raw
  tag_family:=<family>         default: tag36h11
  publish_debug_image:=true    default: true
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share    = get_package_share_directory("tag_tracker")
    default_cfg  = os.path.join(pkg_share, "config", "tag_tracker.yaml")

    # ── Launch arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "config_file", default_value=default_cfg,
            description="Full path to the YAML config file"),
        DeclareLaunchArgument(
            "tag_family", default_value="tag36h11",
            description="AprilTag family string"),
        DeclareLaunchArgument(
            "publish_debug_image", default_value="true",
            description="Publish annotated debug image"),
    ]

    # ── Node ──────────────────────────────────────────────────────────────
    node = Node(
        package    = "tag_tracker",
        executable = "tag_tracker_node",
        name       = "tag_tracker_node",
        output     = "screen",
        parameters = [
            LaunchConfiguration("config_file"),
            {
                "tag_family":          LaunchConfiguration("tag_family"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            },
        ],
        # Uncomment to remap topics:
        # remappings=[("/camera/image_raw", "/my_robot/camera/image_raw")],
    )

    return LaunchDescription(args + [node])
