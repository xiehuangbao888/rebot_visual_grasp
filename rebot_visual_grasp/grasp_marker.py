#!/usr/bin/env python3
"""Subscribe to depth cloud, transform to base_link, publish grasp RViz markers."""

from __future__ import annotations

from typing import Iterable

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import MarkerArray

from rebot_visual_grasp.grasp_utils import publish_grasp_targets


class GraspMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_marker")

        self.declare_parameter("cloud_topic", "/camera/depth/points")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pregrasp_offset_m", 0.08)
        self.declare_parameter("table_z_min", 0.02)
        self.declare_parameter("table_z_max", 0.35)
        self.declare_parameter("workspace_x_min", -0.35)
        self.declare_parameter("workspace_x_max", 0.45)
        self.declare_parameter("workspace_y_min", -0.35)
        self.declare_parameter("workspace_y_max", 0.35)
        self.declare_parameter("grasp_roll_deg", 180.0)
        self.declare_parameter("grasp_pitch_deg", 0.0)
        self.declare_parameter("grasp_yaw_deg", 0.0)
        self.declare_parameter("min_points", 80)
        self.declare_parameter("process_hz", 2.0)

        self._cloud_topic = self.get_parameter("cloud_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._pregrasp_offset = float(self.get_parameter("pregrasp_offset_m").value)
        self._table_z_min = float(self.get_parameter("table_z_min").value)
        self._table_z_max = float(self.get_parameter("table_z_max").value)
        self._x_min = float(self.get_parameter("workspace_x_min").value)
        self._x_max = float(self.get_parameter("workspace_x_max").value)
        self._y_min = float(self.get_parameter("workspace_y_min").value)
        self._y_max = float(self.get_parameter("workspace_y_max").value)
        self._roll_deg = float(self.get_parameter("grasp_roll_deg").value)
        self._pitch_deg = float(self.get_parameter("grasp_pitch_deg").value)
        self._yaw_deg = float(self.get_parameter("grasp_yaw_deg").value)
        self._min_points = int(self.get_parameter("min_points").value)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(MarkerArray, "/grasp_markers", 10)
        self._grasp_pose_pub = self.create_publisher(PoseStamped, "/grasp_pose", 10)
        self._pregrasp_pose_pub = self.create_publisher(PoseStamped, "/pregrasp_pose", 10)

        process_hz = max(float(self.get_parameter("process_hz").value), 0.1)
        self._latest_cloud: PointCloud2 | None = None
        self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / process_hz, self._process_latest_cloud)

        self.get_logger().info(
            f"Grasp marker ready: cloud={self._cloud_topic}, base={self._base_frame}, "
            f"pregrasp_offset={self._pregrasp_offset:.2f} m"
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _lookup_transform(self, cloud: PointCloud2):
        timeout = Duration(seconds=0.5)
        try:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                cloud.header.frame_id,
                cloud.header.stamp,
                timeout=timeout,
            )
        except Exception:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                cloud.header.frame_id,
                Time(),
                timeout=timeout,
            )

    @staticmethod
    def _filter_points(points: Iterable[tuple[float, float, float]]) -> np.ndarray:
        xyz = np.asarray(list(points), dtype=np.float64)
        if xyz.size == 0:
            return xyz.reshape(0, 3)
        valid = np.isfinite(xyz).all(axis=1)
        return xyz[valid]

    def _select_grasp_point(self, xyz: np.ndarray) -> np.ndarray | None:
        if xyz.shape[0] < self._min_points:
            return None

        mask = (
            (xyz[:, 0] >= self._x_min)
            & (xyz[:, 0] <= self._x_max)
            & (xyz[:, 1] >= self._y_min)
            & (xyz[:, 1] <= self._y_max)
            & (xyz[:, 2] >= self._table_z_min)
            & (xyz[:, 2] <= self._table_z_max)
        )
        filtered = xyz[mask]
        if filtered.shape[0] < self._min_points:
            return None

        # Centroid of points in the workspace/table band (MVP target).
        return filtered.mean(axis=0)

    def _process_latest_cloud(self) -> None:
        cloud = self._latest_cloud
        if cloud is None:
            return

        try:
            transform = self._lookup_transform(cloud)
            cloud_base = do_transform_cloud(cloud, transform)
        except Exception as exc:
            self.get_logger().warning(f"TF lookup failed ({cloud.header.frame_id} -> {self._base_frame}): {exc}")
            return

        raw_points = pc2.read_points(cloud_base, field_names=("x", "y", "z"), skip_nans=True)
        xyz = self._filter_points(raw_points)
        grasp_xyz = self._select_grasp_point(xyz)
        if grasp_xyz is None:
            self.get_logger().debug("Not enough valid points for grasp target")
            return

        gx, gy, gz = (float(v) for v in grasp_xyz)
        stamp = self.get_clock().now().to_msg()
        publish_grasp_targets(
            base_frame=self._base_frame,
            stamp=stamp,
            gx=gx,
            gy=gy,
            gz=gz,
            pregrasp_offset_m=self._pregrasp_offset,
            roll_deg=self._roll_deg,
            pitch_deg=self._pitch_deg,
            yaw_deg=self._yaw_deg,
            marker_pub=self._marker_pub,
            grasp_pose_pub=self._grasp_pose_pub,
            pregrasp_pose_pub=self._pregrasp_pose_pub,
        )
        pregrasp_z = gz + self._pregrasp_offset

        self.get_logger().info(
            f"grasp=({gx:.3f}, {gy:.3f}, {gz:.3f}) "
            f"pregrasp_z={pregrasp_z:.3f} points={int(xyz.shape[0])}",
            throttle_duration_sec=2.0,
        )


def main() -> None:
    rclpy.init()
    node = GraspMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
