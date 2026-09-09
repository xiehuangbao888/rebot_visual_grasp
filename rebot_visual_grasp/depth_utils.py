"""Depth image helpers and optical/base point transforms."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from geometry_msgs.msg import PointStamped, TransformStamped
from tf_transformations import quaternion_matrix
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point


def depth_value_to_meters(raw: float, encoding: str, depth_scale: float) -> float:
    if not math.isfinite(raw):
        return float("nan")
    enc = encoding.lower()
    if enc in ("32fc1", "64fc1"):
        return float(raw)
    return float(raw) * depth_scale


def sample_depth_m(
    depth_img: np.ndarray,
    depth_encoding: str,
    depth_scale: float,
    u: int,
    v: int,
    *,
    patch_radius: int,
    depth_min_m: float,
    depth_max_m: float,
) -> Optional[float]:
    if depth_img.ndim != 2:
        return None
    h, w = depth_img.shape[:2]
    u = int(np.clip(u, 0, w - 1))
    v = int(np.clip(v, 0, h - 1))
    r = max(patch_radius, 0)
    u0, u1 = max(u - r, 0), min(u + r + 1, w)
    v0, v1 = max(v - r, 0), min(v + r + 1, h)
    patch = depth_img[v0:v1, u0:u1].astype(np.float64).reshape(-1)
    values = [
        depth_value_to_meters(val, depth_encoding, depth_scale)
        for val in patch
    ]
    valid = [val for val in values if depth_min_m <= val <= depth_max_m]
    if not valid:
        return None
    return float(np.median(valid))


def sample_depth_in_box(
    depth_img: np.ndarray,
    depth_encoding: str,
    depth_scale: float,
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    *,
    depth_min_m: float,
    depth_max_m: float,
    quantile: float = 0.5,
) -> Optional[tuple[float, int, int]]:
    h, w = depth_img.shape[:2]
    x1 = int(np.clip(min(x1, x2), 0, w - 1))
    x2 = int(np.clip(max(x1, x2), 0, w - 1))
    y1 = int(np.clip(min(y1, y2), 0, h - 1))
    y2 = int(np.clip(max(y1, y2), 0, h - 1))
    patch = depth_img[y1 : y2 + 1, x1 : x2 + 1].astype(np.float64).reshape(-1)
    values = [
        depth_value_to_meters(val, depth_encoding, depth_scale)
        for val in patch
    ]
    valid = [val for val in values if depth_min_m <= val <= depth_max_m]
    if not valid:
        return None
    depth_m = float(np.quantile(valid, quantile))
    u = (x1 + x2) // 2
    v = (y1 + y2) // 2
    return depth_m, u, v


def pixel_to_optical(
    camera_info: CameraInfo,
    u: int,
    v: int,
    depth_m: float,
) -> Optional[tuple[float, float, float]]:
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if fx <= 0.0 or fy <= 0.0:
        return None
    x = (float(u) - cx) * depth_m / fx
    y = (float(v) - cy) * depth_m / fy
    return x, y, depth_m


def optical_points_to_base_yaw(
    node: Node,
    tf_buffer,
    base_frame: str,
    optical_frame: str,
    center_uv: tuple[int, int],
    axis_uv: tuple[int, int],
    depth_m: float,
    camera_info: CameraInfo,
    stamp,
) -> Optional[float]:
    center = pixel_to_optical(camera_info, center_uv[0], center_uv[1], depth_m)
    axis_pt = pixel_to_optical(camera_info, axis_uv[0], axis_uv[1], depth_m)
    if center is None or axis_pt is None:
        return None

    def to_base(x: float, y: float, z: float) -> Optional[tuple[float, float, float]]:
        pt = PointStamped()
        pt.header.stamp = stamp
        pt.header.frame_id = optical_frame
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        try:
            timeout = Duration(seconds=0.5)
            try:
                transform = tf_buffer.lookup_transform(base_frame, optical_frame, stamp, timeout=timeout)
            except Exception:
                transform = tf_buffer.lookup_transform(base_frame, optical_frame, Time(), timeout=timeout)
            out = do_transform_point(pt, transform)
        except Exception as exc:
            node.get_logger().warning(f"TF failed for yaw: {exc}")
            return None
        return float(out.point.x), float(out.point.y), float(out.point.z)

    c = to_base(*center)
    a = to_base(*axis_pt)
    if c is None or a is None:
        return None
    return math.atan2(a[1] - c[1], a[0] - c[0])


def transform_to_matrix4(transform: TransformStamped) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    matrix[0, 3] = float(translation.x)
    matrix[1, 3] = float(translation.y)
    matrix[2, 3] = float(translation.z)
    return matrix.astype(np.float64)


def lookup_cam_to_base_matrix(
    node: Node,
    tf_buffer,
    base_frame: str,
    optical_frame: str,
    stamp,
) -> Optional[np.ndarray]:
    try:
        timeout = Duration(seconds=0.5)
        try:
            transform = tf_buffer.lookup_transform(base_frame, optical_frame, stamp, timeout=timeout)
        except Exception:
            transform = tf_buffer.lookup_transform(base_frame, optical_frame, Time(), timeout=timeout)
    except Exception as exc:
        node.get_logger().warning(f"TF lookup failed: {exc}")
        return None
    return transform_to_matrix4(transform)


def depth_image_to_mm(
    depth_img: np.ndarray,
    depth_encoding: str,
    depth_scale: float,
) -> np.ndarray:
    values = depth_img.astype(np.float64)
    enc = depth_encoding.lower()
    if enc in ("32fc1", "64fc1"):
        return np.where(values > 0.0, values * 1000.0, 0.0).astype(np.float32)
    return np.where(values > 0.0, values * depth_scale * 1000.0, 0.0).astype(np.float32)


def optical_point_to_base(
    node: Node,
    tf_buffer,
    base_frame: str,
    optical_frame: str,
    u: int,
    v: int,
    depth_m: float,
    camera_info: CameraInfo,
    stamp,
) -> Optional[tuple[float, float, float]]:
    optical = pixel_to_optical(camera_info, u, v, depth_m)
    if optical is None:
        return None
    pt = PointStamped()
    pt.header.stamp = stamp
    pt.header.frame_id = optical_frame
    pt.point.x, pt.point.y, pt.point.z = optical
    try:
        timeout = Duration(seconds=0.5)
        try:
            transform = tf_buffer.lookup_transform(base_frame, optical_frame, stamp, timeout=timeout)
        except Exception:
            transform = tf_buffer.lookup_transform(base_frame, optical_frame, Time(), timeout=timeout)
        out = do_transform_point(pt, transform)
    except Exception as exc:
        node.get_logger().warning(f"TF failed: {exc}")
        return None
    return float(out.point.x), float(out.point.y), float(out.point.z)
