from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vis_share = FindPackageShare("rebot_visual_grasp")
    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "RS", "urdf", "ReBot_Arm_RS.urdf"]
    )
    rviz_urdf_compat = PathJoinSubstitution(
        [bringup_share, "launch", "rviz_urdf_compat.py"]
    )
    rviz_config = PathJoinSubstitution([vis_share, "rviz", "align_mount.rviz"])
    robot_description = ParameterValue(
        Command(["python3 ", rviz_urdf_compat, " ", urdf_file]),
        value_type=str,
    )
    assembly = LaunchConfiguration("assembly")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "assembly",
                default_value="gemini2",
                description="gemini2 / d435i share mount; d405 is independent",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="rebot_visual_grasp",
                executable="align_mount",
                output="screen",
                emulate_tty=True,
                parameters=[{"assembly": assembly}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
