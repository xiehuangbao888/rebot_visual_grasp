from pathlib import Path

from ament_index_python.packages import get_package_share_directory

XACRO_NAME = "rebotarm_rs_with_gemini2.urdf.xacro"
MESH_NAME = "D435_Gemini2_Mount.stl"


def package_share() -> Path:
    return Path(get_package_share_directory("rebot_visual_grasp"))


def share_xacro() -> Path:
    return package_share() / "description" / "urdf" / XACRO_NAME


def mount_mesh_uri() -> str:
    return f"package://rebot_visual_grasp/description/meshes/{MESH_NAME}"


def writable_xacro() -> Path:
    """Prefer the source xacro so alignment edits survive colcon install."""
    share = package_share()
    workspace = share.parents[3]
    src = workspace / "src" / "rebot_visual_grasp" / "description" / "urdf" / XACRO_NAME
    if src.is_file():
        return src
    return share_xacro()
