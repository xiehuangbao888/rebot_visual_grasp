"""Coordinate transforms for Seeed-style visual grasp poses."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

_ROT_X_PI = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=np.float64,
)


def _nearest_rotation_matrix(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError(f"rotation matrix must be (3, 3), got {rotation.shape}")
    if not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix contains non-finite values")

    u, _, vt = np.linalg.svd(rotation)
    ortho = u @ vt
    if np.linalg.det(ortho) < 0.0:
        u[:, -1] *= -1.0
        ortho = u @ vt
    return ortho.astype(np.float64)


def rotation_matrix_to_euler_zyx(rotation: np.ndarray) -> np.ndarray:
    rotation = _nearest_rotation_matrix(rotation)
    sy = math.sqrt(float(rotation[0, 0] ** 2 + rotation[1, 0] ** 2))
    if sy > 1e-6:
        rx = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        ry = math.atan2(float(-rotation[2, 0]), sy)
        rz = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        rx = math.atan2(float(-rotation[1, 2]), float(rotation[1, 1]))
        ry = math.atan2(float(-rotation[2, 0]), sy)
        rz = 0.0
    return np.array([rx, ry, rz], dtype=np.float64)


def canonicalize_parallel_gripper_tcp_rotation(rotation: np.ndarray) -> np.ndarray:
    rotation = _nearest_rotation_matrix(rotation)
    alt = rotation @ _ROT_X_PI
    roll = float(rotation_matrix_to_euler_zyx(rotation)[0])
    alt_roll = float(rotation_matrix_to_euler_zyx(alt)[0])
    return alt if abs(alt_roll) < abs(roll) else rotation


def grasp_axes_to_rebot_tcp_rotation(
    grip_axis: np.ndarray,
    open_axis: np.ndarray,
    approach_axis: np.ndarray,
) -> np.ndarray:
    grip = np.asarray(grip_axis, dtype=np.float64)
    open_vec = np.asarray(open_axis, dtype=np.float64)
    approach = np.asarray(approach_axis, dtype=np.float64)

    grip /= max(float(np.linalg.norm(grip)), 1e-8)
    open_vec /= max(float(np.linalg.norm(open_vec)), 1e-8)
    approach /= max(float(np.linalg.norm(approach)), 1e-8)

    tcp_x = -approach
    tcp_y = open_vec - float(np.dot(open_vec, tcp_x)) * tcp_x
    tcp_y /= max(float(np.linalg.norm(tcp_y)), 1e-8)
    tcp_z = np.cross(tcp_x, tcp_y)
    tcp_z /= max(float(np.linalg.norm(tcp_z)), 1e-8)

    if float(np.dot(tcp_z, grip)) < 0.0:
        tcp_y = -tcp_y
        tcp_z = -tcp_z

    matrix = np.column_stack([tcp_x, tcp_y, tcp_z]).astype(np.float64)
    if np.linalg.det(matrix) < 0.0:
        matrix[:, 2] *= -1.0
    return matrix


def mat4_to_pose6d(transform: np.ndarray) -> tuple[float, float, float, float, float, float]:
    x, y, z = float(transform[0, 3]), float(transform[1, 3]), float(transform[2, 3])
    roll, pitch, yaw = rotation_matrix_to_euler_zyx(transform[:3, :3])
    return x, y, z, roll, pitch, yaw


def pose6d_to_geometry_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def _make_grasp_base_transform(
    position_cam: np.ndarray,
    tcp_rotation_cam: np.ndarray,
    cam_to_base: np.ndarray,
) -> np.ndarray:
    grasp_cam = np.eye(4, dtype=np.float64)
    grasp_cam[:3, :3] = np.asarray(tcp_rotation_cam, dtype=np.float64)
    grasp_cam[:3, 3] = np.asarray(position_cam, dtype=np.float64)

    grasp_base = np.asarray(cam_to_base, dtype=np.float64) @ grasp_cam
    grasp_base[:3, :3] = canonicalize_parallel_gripper_tcp_rotation(grasp_base[:3, :3])
    return grasp_base


def _offset_along_tool_x(transform: np.ndarray, offset_m: float) -> np.ndarray:
    offset = transform.copy()
    offset[:3, 3] = transform[:3, 3] - transform[:3, 0] * float(offset_m)
    return offset


def _offset_along_world_z(transform: np.ndarray, offset_m: float) -> np.ndarray:
    """Raise/lower along base-frame +Z (straight above the grasp on a tabletop)."""
    offset = transform.copy()
    offset[2, 3] = transform[2, 3] + float(offset_m)
    return offset


def force_top_down_tcp_in_base(grasp_base: np.ndarray) -> np.ndarray:
    """Keep horizontal jaw opening; force approach along world +Z (tip toward table).

    Seeed sets approach along the camera ray, so eye-in-hand views make the TCP tilt.
    For tabletop pick we usually want the gripper tip perpendicular to the desk.
    """
    out = np.asarray(grasp_base, dtype=np.float64).copy()
    # tcp_y ≈ open axis after Seeed mapping; fall back to tcp_z / X.
    open_axis = out[:3, 1].copy()
    open_axis[2] = 0.0
    norm = float(np.linalg.norm(open_axis))
    if norm < 1e-6:
        open_axis = out[:3, 2].copy()
        open_axis[2] = 0.0
        norm = float(np.linalg.norm(open_axis))
    if norm < 1e-6:
        open_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        open_axis /= norm

    approach = np.array([0.0, 0.0, 1.0], dtype=np.float64)  # object → gripper (up)
    grip = np.cross(open_axis, approach)
    grip_norm = float(np.linalg.norm(grip))
    if grip_norm < 1e-6:
        open_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        grip = np.cross(open_axis, approach)
        grip_norm = float(np.linalg.norm(grip))
    grip /= max(grip_norm, 1e-8)

    tcp = grasp_axes_to_rebot_tcp_rotation(grip, open_axis, approach)
    out[:3, :3] = canonicalize_parallel_gripper_tcp_rotation(tcp)
    return out


def transform_grasp_pose_to_base(
    position_cam: np.ndarray,
    tcp_rotation_cam: np.ndarray,
    cam_to_base: np.ndarray,
    pregrasp_offset_m: float,
    insertion_depth_m: float = 0.0,
    pregrasp_mode: str = "tool_x",
    approach_mode: str = "camera",
) -> tuple[Pose, Pose]:
    grasp_base = _make_grasp_base_transform(position_cam, tcp_rotation_cam, cam_to_base)

    approach = str(approach_mode).strip().lower()
    if approach in {"world_z", "vertical", "top_down", "table"}:
        grasp_base = force_top_down_tcp_in_base(grasp_base)

    grasp_base = _offset_along_tool_x(grasp_base, -insertion_depth_m)

    mode = str(pregrasp_mode).strip().lower()
    if mode in {"world_z", "vertical", "top_down", "table"}:
        pregrasp_base = _offset_along_world_z(grasp_base, pregrasp_offset_m)
    else:
        # Default Seeed: retreat along tool X (tilted approach).
        pregrasp_base = _offset_along_tool_x(grasp_base, pregrasp_offset_m)

    gx, gy, gz, gr, gp, gyaw = mat4_to_pose6d(grasp_base)
    px, py, pz, pr, pp, pyaw = mat4_to_pose6d(pregrasp_base)
    return (
        pose6d_to_geometry_pose(gx, gy, gz, gr, gp, gyaw),
        pose6d_to_geometry_pose(px, py, pz, pr, pp, pyaw),
    )
