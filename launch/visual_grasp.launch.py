from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rebot_visual_grasp")
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, "/launch/grasp_click.launch.py"]),
                launch_arguments={
                    "arm_namespace": LaunchConfiguration("arm_namespace"),
                    "move_to_observation_on_start": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, "/launch/grasp_execute.launch.py"]),
                launch_arguments={
                    "arm_namespace": LaunchConfiguration("arm_namespace"),
                }.items(),
            ),
        ]
    )
