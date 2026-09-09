from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("color_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("depth_scale", default_value="0.001"),
            DeclareLaunchArgument("grasp_roll_deg", default_value="180.0"),
            DeclareLaunchArgument("grasp_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("grasp_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("move_to_observation_on_start", default_value="true"),
            DeclareLaunchArgument("observation_x", default_value="0.205"),
            DeclareLaunchArgument("observation_y", default_value="0.007"),
            DeclareLaunchArgument("observation_z", default_value="0.312"),
            DeclareLaunchArgument("observation_qx", default_value="-0.025"),
            DeclareLaunchArgument("observation_qy", default_value="0.434"),
            DeclareLaunchArgument("observation_qz", default_value="0.031"),
            DeclareLaunchArgument("observation_qw", default_value="0.900"),
            DeclareLaunchArgument("observation_duration", default_value="3.0"),
            Node(
                package="rebot_visual_grasp",
                executable="grasp_click",
                output="screen",
                parameters=[
                    {
                        "color_topic": LaunchConfiguration("color_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "color_info_topic": LaunchConfiguration("color_info_topic"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "pregrasp_offset_m": LaunchConfiguration("pregrasp_offset_m"),
                        "depth_scale": LaunchConfiguration("depth_scale"),
                        "grasp_roll_deg": LaunchConfiguration("grasp_roll_deg"),
                        "grasp_pitch_deg": LaunchConfiguration("grasp_pitch_deg"),
                        "grasp_yaw_deg": LaunchConfiguration("grasp_yaw_deg"),
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "move_to_observation_on_start": LaunchConfiguration(
                            "move_to_observation_on_start"
                        ),
                        "observation_x": LaunchConfiguration("observation_x"),
                        "observation_y": LaunchConfiguration("observation_y"),
                        "observation_z": LaunchConfiguration("observation_z"),
                        "observation_qx": LaunchConfiguration("observation_qx"),
                        "observation_qy": LaunchConfiguration("observation_qy"),
                        "observation_qz": LaunchConfiguration("observation_qz"),
                        "observation_qw": LaunchConfiguration("observation_qw"),
                        "observation_duration": LaunchConfiguration("observation_duration"),
                    }
                ],
            ),
        ]
    )
