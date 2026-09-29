from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    vis_share = FindPackageShare("rebot_visual_grasp")
    assembly = LaunchConfiguration("assembly").perform(context).strip().lower() or "gemini2"
    xacro_name = {
        "gemini2": "rebotarm_rs_with_gemini2.urdf.xacro",
        "d405": "rebotarm_rs_with_d405.urdf.xacro",
    }.get(assembly)
    if xacro_name is None:
        raise RuntimeError(f"unknown assembly={assembly!r}; use gemini2 or d405")

    xacro_file = PathJoinSubstitution([vis_share, "description", "urdf", xacro_name])
    rviz_config = PathJoinSubstitution([vis_share, "rviz", "align_camera.rviz"])
    # Same as Gemini: camera only on the interactive marker (avoid double D405).
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " with_camera:=false"]),
        value_type=str,
    )
    return [
        LogInfo(msg=f"[align_camera] assembly={assembly} xacro={xacro_name} with_camera:=false"),
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
            executable="align_camera",
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


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "assembly",
                default_value="gemini2",
                description="gemini2 (default, existing) or d405",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
