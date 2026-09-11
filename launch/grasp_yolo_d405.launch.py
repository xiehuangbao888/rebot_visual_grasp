"""YOLO grasp tuned for wrist D405 (RealSense topics).

Gemini2 defaults stay in grasp_yolo.launch.py.
D405 RealSense node usually publishes under /camera/camera/...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # RealSense default: /{camera_namespace}/{camera_name}/...
            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "color_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "optical_frame",
                default_value="camera_color_optical_frame",
            ),
            DeclareLaunchArgument(
                "yolo_model",
                default_value="/home/ubuntu/rebot_visual_model/yoloe-26s-seg.pt",
            ),
            DeclareLaunchArgument("target_class", default_value="cube"),
            DeclareLaunchArgument("place_class", default_value="box"),
            DeclareLaunchArgument("enable_place_detect", default_value="true"),
            DeclareLaunchArgument("place_z_offset_m", default_value="0.1"),
            DeclareLaunchArgument("grasp_x_offset_m", default_value="-0.04"),
            DeclareLaunchArgument("grasp_y_offset_m", default_value="0.0"),
            DeclareLaunchArgument("grasp_z_offset_m", default_value="0.02"),
            DeclareLaunchArgument("yolo_device", default_value="0"),
            DeclareLaunchArgument("conf_threshold", default_value="0.4"),
            DeclareLaunchArgument("orientation_mode", default_value="seeed"),
            DeclareLaunchArgument("grasp_pitch_deg", default_value="90.0"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("pregrasp_mode", default_value="tool_x"),
            DeclareLaunchArgument("grasp_approach_mode", default_value="camera"),
            DeclareLaunchArgument("insertion_depth_m", default_value="0.015"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("move_to_observation_on_start", default_value="true"),
            DeclareLaunchArgument("auto_publish_on_detect", default_value="true"),
            DeclareLaunchArgument("open_gripper_on_g", default_value="false"),
            DeclareLaunchArgument("trigger_grasp_on_g", default_value="false"),
            Node(
                package="rebot_visual_grasp",
                executable="grasp_yolo",
                output="screen",
                parameters=[
                    {
                        "color_topic": LaunchConfiguration("color_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "color_info_topic": LaunchConfiguration("color_info_topic"),
                        "optical_frame": LaunchConfiguration("optical_frame"),
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "custom_classes": ["cube", "box", "pen"],
                        "target_class": LaunchConfiguration("target_class"),
                        "place_class": LaunchConfiguration("place_class"),
                        "enable_place_detect": LaunchConfiguration("enable_place_detect"),
                        "place_z_offset_m": LaunchConfiguration("place_z_offset_m"),
                        "yolo_device": LaunchConfiguration("yolo_device"),
                        "conf_threshold": LaunchConfiguration("conf_threshold"),
                        "orientation_mode": LaunchConfiguration("orientation_mode"),
                        "grasp_pitch_deg": LaunchConfiguration("grasp_pitch_deg"),
                        "pregrasp_offset_m": LaunchConfiguration("pregrasp_offset_m"),
                        "pregrasp_mode": LaunchConfiguration("pregrasp_mode"),
                        "grasp_approach_mode": LaunchConfiguration("grasp_approach_mode"),
                        "insertion_depth_m": LaunchConfiguration("insertion_depth_m"),
                        "grasp_x_offset_m": LaunchConfiguration("grasp_x_offset_m"),
                        "grasp_y_offset_m": LaunchConfiguration("grasp_y_offset_m"),
                        "grasp_z_offset_m": LaunchConfiguration("grasp_z_offset_m"),
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "move_to_observation_on_start": LaunchConfiguration(
                            "move_to_observation_on_start"
                        ),
                        "auto_publish_on_detect": LaunchConfiguration(
                            "auto_publish_on_detect"
                        ),
                        "open_gripper_on_g": LaunchConfiguration("open_gripper_on_g"),
                        "trigger_grasp_on_g": LaunchConfiguration("trigger_grasp_on_g"),
                        "use_yoloe": True,
                    }
                ],
            ),
        ]
    )
