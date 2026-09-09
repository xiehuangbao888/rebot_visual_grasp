from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/camera/depth/points"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("table_z_min", default_value="0.02"),
            DeclareLaunchArgument("table_z_max", default_value="0.35"),
            DeclareLaunchArgument("grasp_roll_deg", default_value="180.0"),
            DeclareLaunchArgument("grasp_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("grasp_yaw_deg", default_value="0.0"),
            Node(
                package="rebot_visual_grasp",
                executable="grasp_marker",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "pregrasp_offset_m": LaunchConfiguration("pregrasp_offset_m"),
                        "table_z_min": LaunchConfiguration("table_z_min"),
                        "table_z_max": LaunchConfiguration("table_z_max"),
                        "grasp_roll_deg": LaunchConfiguration("grasp_roll_deg"),
                        "grasp_pitch_deg": LaunchConfiguration("grasp_pitch_deg"),
                        "grasp_yaw_deg": LaunchConfiguration("grasp_yaw_deg"),
                    }
                ],
            ),
        ]
    )
