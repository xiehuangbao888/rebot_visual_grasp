"""Launch grasp_execute and trigger /grasp_execute after a short delay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _delayed_trigger(context, *args, **kwargs):
    delay = float(context.launch_configurations.get("trigger_delay_s", "3.0"))
    return [
        TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/grasp_execute",
                        "std_srvs/srv/Trigger",
                        "{}",
                    ],
                    output="screen",
                )
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("trigger_delay_s", default_value="3.0"),
            DeclareLaunchArgument("require_place_pose", default_value="true"),
            DeclareLaunchArgument("pregrasp_duration", default_value="4.0"),
            DeclareLaunchArgument("grasp_duration", default_value="3.0"),
            DeclareLaunchArgument("observation_duration", default_value="4.0"),
            DeclareLaunchArgument("place_duration", default_value="4.0"),
            Node(
                package="rebot_visual_grasp",
                executable="grasp_execute",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "require_place_pose": LaunchConfiguration("require_place_pose"),
                        "pregrasp_duration": LaunchConfiguration("pregrasp_duration"),
                        "grasp_duration": LaunchConfiguration("grasp_duration"),
                        "observation_duration": LaunchConfiguration(
                            "observation_duration"
                        ),
                        "place_duration": LaunchConfiguration("place_duration"),
                    }
                ],
            ),
            OpaqueFunction(function=_delayed_trigger),
        ]
    )
