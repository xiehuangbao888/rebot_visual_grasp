from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("pregrasp_duration", default_value="4.0"),
            DeclareLaunchArgument("grasp_duration", default_value="3.0"),
            DeclareLaunchArgument("observation_duration", default_value="4.0"),
            DeclareLaunchArgument("place_duration", default_value="4.0"),
            DeclareLaunchArgument("check_ik", default_value="false"),
            DeclareLaunchArgument("open_gripper_before_motion", default_value="true"),
            DeclareLaunchArgument("require_place_pose", default_value="true"),
            DeclareLaunchArgument("open_gripper_at_place", default_value="true"),
            DeclareLaunchArgument("return_observation_after_place", default_value="true"),
            DeclareLaunchArgument("close_gripper_after_place", default_value="true"),
            DeclareLaunchArgument(
                "place_use_observation_orientation", default_value="true"
            ),
            DeclareLaunchArgument("gripper_timeout", default_value="5.0"),
            DeclareLaunchArgument("observation_x", default_value="0.205"),
            DeclareLaunchArgument("observation_y", default_value="0.007"),
            DeclareLaunchArgument("observation_z", default_value="0.312"),
            DeclareLaunchArgument("observation_qx", default_value="-0.025"),
            DeclareLaunchArgument("observation_qy", default_value="0.434"),
            DeclareLaunchArgument("observation_qz", default_value="0.031"),
            DeclareLaunchArgument("observation_qw", default_value="0.900"),
            Node(
                package="rebot_visual_grasp",
                executable="grasp_execute",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "pregrasp_duration": LaunchConfiguration("pregrasp_duration"),
                        "grasp_duration": LaunchConfiguration("grasp_duration"),
                        "observation_duration": LaunchConfiguration(
                            "observation_duration"
                        ),
                        "place_duration": LaunchConfiguration("place_duration"),
                        "check_ik": LaunchConfiguration("check_ik"),
                        "open_gripper_before_motion": LaunchConfiguration(
                            "open_gripper_before_motion"
                        ),
                        "require_place_pose": LaunchConfiguration("require_place_pose"),
                        "open_gripper_at_place": LaunchConfiguration(
                            "open_gripper_at_place"
                        ),
                        "return_observation_after_place": LaunchConfiguration(
                            "return_observation_after_place"
                        ),
                        "close_gripper_after_place": LaunchConfiguration(
                            "close_gripper_after_place"
                        ),
                        "place_use_observation_orientation": LaunchConfiguration(
                            "place_use_observation_orientation"
                        ),
                        "gripper_timeout": LaunchConfiguration("gripper_timeout"),
                        "observation_x": LaunchConfiguration("observation_x"),
                        "observation_y": LaunchConfiguration("observation_y"),
                        "observation_z": LaunchConfiguration("observation_z"),
                        "observation_qx": LaunchConfiguration("observation_qx"),
                        "observation_qy": LaunchConfiguration("observation_qy"),
                        "observation_qz": LaunchConfiguration("observation_qz"),
                        "observation_qw": LaunchConfiguration("observation_qw"),
                    }
                ],
            ),
        ]
    )
