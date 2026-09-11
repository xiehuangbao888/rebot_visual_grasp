from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vis_share = FindPackageShare("rebot_visual_grasp")
    xacro_file = PathJoinSubstitution(
        [
            vis_share,
            "description",
            "urdf",
            "rebotarm_rs_with_gemini2.urdf.xacro",
        ]
    )
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    rviz_config = PathJoinSubstitution(
        [vis_share, "rviz", "display_arm_camera.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
