"""RS arm bringup + D435i mount/camera URDF for RViz (no RealSense driver).

Camera driver separately:
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

  ros2 launch rebot_visual_grasp bringup_with_d435i.launch.py use_rviz:=true
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vis_share = FindPackageShare("rebot_visual_grasp")
    xacro_file = PathJoinSubstitution(
        [
            vis_share,
            "description",
            "urdf",
            "rebotarm_rs_with_d435i.urdf.xacro",
        ]
    )
    rviz_urdf_compat = PathJoinSubstitution(
        [bringup_share, "launch", "rviz_urdf_compat.py"]
    )

    hardware_config = LaunchConfiguration("hardware_config")
    model = LaunchConfiguration("model")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    disable_after_safe_home = LaunchConfiguration("disable_after_safe_home")

    robot_description = ParameterValue(
        Command(
            [
                "bash",
                "-c",
                [
                    "xacro ",
                    xacro_file,
                    " | python3 ",
                    rviz_urdf_compat,
                    " -",
                ],
            ]
        ),
        value_type=str,
    )
    rviz_config = PathJoinSubstitution(
        [vis_share, "rviz", "bringup_with_camera.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "rebotarm_hardware.yaml"]
                ),
            ),
            DeclareLaunchArgument("model", default_value="rs"),
            DeclareLaunchArgument("channel", default_value="can0"),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            LogInfo(
                msg=(
                    "[bringup_with_d435i] RS bringup + D435i URDF/RViz only. "
                    "Start camera: ros2 launch realsense2_camera rs_launch.py "
                    "align_depth.enable:=true"
                )
            ),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                on_exit=Shutdown(reason="reBotArmController exited"),
                parameters=[
                    {
                        "hardware_config": hardware_config,
                        "model": model,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "frame_id": frame_id,
                        "ee_frame_id": ee_frame_id,
                        "disable_after_safe_home": ParameterValue(
                            disable_after_safe_home,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
