from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    # Import here so launch works after `source install/setup.bash`.
    from rebot_visual_grasp.paths import resolve_assembly, writable_xacro

    vis_share = FindPackageShare("rebot_visual_grasp")
    assembly = LaunchConfiguration("assembly").perform(context).strip().lower() or "gemini2"
    assembly = resolve_assembly(assembly)
    # Prefer src xacro (same file align_mount writes), not a stale install copy.
    xacro_path = str(writable_xacro(assembly))

    rviz_config = PathJoinSubstitution([vis_share, "rviz", "align_camera.rviz"])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_path, " with_camera:=false"]),
        value_type=str,
    )
    return [
        LogInfo(
            msg=(
                f"[align_camera] assembly={assembly} "
                f"xacro={xacro_path} with_camera:=false"
            )
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
                description="gemini2 (default), d405, or d435i",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
