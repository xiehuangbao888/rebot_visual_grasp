from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

ASSEMBLIES = {
    "gemini2": {
        "xacro": "rebotarm_rs_with_gemini2.urdf.xacro",
        "mount_mesh": "D435_Gemini2_Mount.stl",
        "extrinsics": "wrist_extrinsics.yaml",
        "camera_mesh": "package://orbbec_description/meshes/gemini2/base_link.STL",
        "camera_mesh_package": "orbbec_description",
        "camera_mesh_relpath": "meshes/gemini2/base_link.STL",
        "camera_mesh_scale": 1.0,
        "camera_mesh_offset": (-0.01491, -0.025, -0.0125),
        "camera_mesh_rpy": (0.0, 0.0, 0.0),
        "camera_label": "Gemini 2",
    },
    "d405": {
        "xacro": "rebotarm_rs_with_d405.urdf.xacro",
        "mount_mesh": "D405_305_Mount.stl",
        "extrinsics": "wrist_extrinsics_d405.yaml",
        # Same visual origin as realsense2_description _d405.urdf.xacro (bottom_screw frame).
        "camera_mesh": "package://realsense2_description/meshes/d405.stl",
        "camera_mesh_package": "realsense2_description",
        "camera_mesh_relpath": "meshes/d405.stl",
        "camera_mesh_scale": 0.001,
        "camera_mesh_offset": (0.01465, 0.0, 0.021),
        "camera_mesh_rpy": (1.57079632679, 0.0, 1.57079632679),
        "camera_label": "D405",
    },
    "d435i": {
        "xacro": "rebotarm_rs_with_d435i.urdf.xacro",
        # Same physical mount as Gemini2.
        "mount_mesh": "D435_Gemini2_Mount.stl",
        "extrinsics": "wrist_extrinsics_d435i.yaml",
        "camera_mesh": "package://realsense2_description/meshes/d435.dae",
        "camera_mesh_package": "realsense2_description",
        "camera_mesh_relpath": "meshes/d435.dae",
        "camera_mesh_scale": 1.0,
        "camera_mesh_offset": (0.0149, 0.0, 0.0125),
        "camera_mesh_rpy": (1.57079632679, 0.0, 1.57079632679),
        "camera_label": "D435i",
    },
}

# Assemblies that share the same physical wrist mount bracket (not D405).
SHARED_MOUNT_ASSEMBLIES = ("gemini2", "d435i")



def resolve_assembly(name: str | None = None) -> str:
    key = (name or os.environ.get("REBOT_CAMERA_ASSEMBLY") or "gemini2").strip().lower()
    if key not in ASSEMBLIES:
        raise ValueError(f"unknown assembly {key!r}; expected one of {sorted(ASSEMBLIES)}")
    return key


def assembly_cfg(name: str | None = None) -> dict:
    return ASSEMBLIES[resolve_assembly(name)]


def package_share() -> Path:
    return Path(get_package_share_directory("rebot_visual_grasp"))


def share_xacro(assembly: str | None = None) -> Path:
    cfg = assembly_cfg(assembly)
    return package_share() / "description" / "urdf" / cfg["xacro"]


def mount_mesh_uri(assembly: str | None = None) -> str:
    cfg = assembly_cfg(assembly)
    return f"package://rebot_visual_grasp/description/meshes/{cfg['mount_mesh']}"


def writable_xacro(assembly: str | None = None) -> Path:
    """Prefer the source xacro so alignment edits survive colcon install."""
    cfg = assembly_cfg(assembly)
    share = package_share()
    workspace = share.parents[3]
    src = workspace / "src" / "rebot_visual_grasp" / "description" / "urdf" / cfg["xacro"]
    if src.is_file():
        return src
    return share_xacro(assembly)


def writable_extrinsics(assembly: str | None = None) -> Path:
    cfg = assembly_cfg(assembly)
    share = package_share()
    workspace = share.parents[3]
    src = workspace / "src" / "rebot_visual_grasp" / "config" / cfg["extrinsics"]
    if src.is_file():
        return src
    return share / "config" / cfg["extrinsics"]
