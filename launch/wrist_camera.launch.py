"""Start wrist Gemini 2 + publish mount/camera TF without modifying bringup.

Upstream bringup stays unchanged, e.g.:
  ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true

Then in another terminal:
  ros2 launch rebot_visual_grasp wrist_camera.launch.py
"""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf_from_xyz_rpy(
    *,
    xyz: list[float],
    rpy: list[float],
    parent: str,
    child: str,
    name: str,
) -> Node:
    # Humble static_transform_publisher: x y z yaw pitch roll parent child
    # (yaw/pitch/roll == rpy order when using this argv form)
    x, y, z = (float(v) for v in xyz)
    roll, pitch, yaw = (float(v) for v in rpy)
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--roll",
            str(roll),
            "--pitch",
            str(pitch),
            "--yaw",
            str(yaw),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        output="screen",
    )


def _load_extrinsics(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if "mount" not in data or "camera" not in data:
        raise RuntimeError(f"invalid extrinsics file (need mount/camera): {path}")
    return data


def _setup(context, *args, **kwargs):
    actions = []
    cfg = Path(LaunchConfiguration("extrinsics_file").perform(context)).expanduser()
    if not cfg.is_file():
        share = Path(get_package_share_directory("rebot_visual_grasp"))
        cfg = share / "config" / "wrist_extrinsics.yaml"
    extrinsics = _load_extrinsics(cfg)

    mount = extrinsics["mount"]
    camera = extrinsics["camera"]
    actions.append(
        _static_tf_from_xyz_rpy(
            xyz=list(mount["xyz"]),
            rpy=list(mount["rpy"]),
            parent=str(mount.get("parent_frame", "gripper_end")),
            child=str(mount.get("child_frame", "camera_mount_link")),
            name="tf_gripper_to_mount",
        )
    )
    actions.append(
        _static_tf_from_xyz_rpy(
            xyz=list(camera["xyz"]),
            rpy=list(camera["rpy"]),
            parent=str(camera.get("parent_frame", "camera_mount_link")),
            child=str(camera.get("child_frame", "camera_link")),
            name="tf_mount_to_camera",
        )
    )

    start_driver = LaunchConfiguration("start_camera_driver").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    if start_driver:
        try:
            orbbec_share = get_package_share_directory("orbbec_camera")
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "orbbec_camera not found. Install Orbbec ROS2 in another workspace "
                "and source it, or run with start_camera_driver:=false and launch "
                "gemini2.launch.py yourself."
            ) from exc
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(orbbec_share) / "launch" / "gemini2.launch.py")
                )
            )
        )
    return actions


def generate_launch_description():
    share = get_package_share_directory("rebot_visual_grasp")
    default_extrinsics = str(Path(share) / "config" / "wrist_extrinsics.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "extrinsics_file",
                default_value=default_extrinsics,
                description="YAML with mount/camera xyz+rpy relative to gripper_end",
            ),
            DeclareLaunchArgument(
                "start_camera_driver",
                default_value="true",
                description="If true, also include orbbec_camera gemini2.launch.py",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
