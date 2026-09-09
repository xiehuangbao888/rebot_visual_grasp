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
            DeclareLaunchArgument("yolo_model", default_value="/home/ubuntu/ultralytics-main/yoloe-26s-seg.pt"),
            DeclareLaunchArgument("target_class", default_value="yellow banana"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, "/launch/grasp_yolo.launch.py"]),
                launch_arguments={
                    "arm_namespace": LaunchConfiguration("arm_namespace"),
                    "yolo_model": LaunchConfiguration("yolo_model"),
                    "target_class": LaunchConfiguration("target_class"),
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
