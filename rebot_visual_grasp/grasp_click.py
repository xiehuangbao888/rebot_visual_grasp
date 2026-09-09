#!/usr/bin/env python3
"""Click on color image to pick a grasp point from depth, publish RViz markers."""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from rebot_visual_grasp.arm_motion import call_enable, make_pose, move_to_pose
from rebot_visual_grasp.grasp_utils import publish_grasp_targets


class GraspClickNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_click")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("pregrasp_offset_m", 0.08)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_min_m", 0.05)
        self.declare_parameter("depth_max_m", 1.5)
        self.declare_parameter("depth_patch_radius", 2)
        self.declare_parameter("grasp_roll_deg", 180.0)
        self.declare_parameter("grasp_pitch_deg", 0.0)
        self.declare_parameter("grasp_yaw_deg", 0.0)
        self.declare_parameter("window_name", "grasp_click")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("move_to_observation_on_start", True)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("observation_x", 0.205)
        self.declare_parameter("observation_y", 0.007)
        self.declare_parameter("observation_z", 0.312)
        self.declare_parameter("observation_qx", -0.025)
        self.declare_parameter("observation_qy", 0.434)
        self.declare_parameter("observation_qz", 0.031)
        self.declare_parameter("observation_qw", 0.900)
        self.declare_parameter("observation_duration", 3.0)

        self._color_topic = self.get_parameter("color_topic").value
        self._depth_topic = self.get_parameter("depth_topic").value
        self._color_info_topic = self.get_parameter("color_info_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._optical_frame = self.get_parameter("optical_frame").value
        self._pregrasp_offset = float(self.get_parameter("pregrasp_offset_m").value)
        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._depth_min = float(self.get_parameter("depth_min_m").value)
        self._depth_max = float(self.get_parameter("depth_max_m").value)
        self._patch_radius = int(self.get_parameter("depth_patch_radius").value)
        self._roll_deg = float(self.get_parameter("grasp_roll_deg").value)
        self._pitch_deg = float(self.get_parameter("grasp_pitch_deg").value)
        self._yaw_deg = float(self.get_parameter("grasp_yaw_deg").value)
        self._window_name = self.get_parameter("window_name").value
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._move_to_observation = bool(self.get_parameter("move_to_observation_on_start").value)
        self._auto_enable = bool(self.get_parameter("auto_enable").value)
        self._observation_pose = make_pose(
            float(self.get_parameter("observation_x").value),
            float(self.get_parameter("observation_y").value),
            float(self.get_parameter("observation_z").value),
            float(self.get_parameter("observation_qx").value),
            float(self.get_parameter("observation_qy").value),
            float(self.get_parameter("observation_qz").value),
            float(self.get_parameter("observation_qw").value),
        )
        self._observation_duration = float(self.get_parameter("observation_duration").value)

        self._bridge = CvBridge()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._color_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None
        self._color_info: Optional[CameraInfo] = None
        self._pending_click: Optional[tuple[int, int]] = None
        self._last_click_uv: Optional[tuple[int, int]] = None
        self._last_grasp_base: Optional[tuple[float, float, float]] = None
        self._display_bgr: Optional[np.ndarray] = None

        self._marker_pub = self.create_publisher(MarkerArray, "/grasp_markers", 10)
        self._grasp_pose_pub = self.create_publisher(PoseStamped, "/grasp_pose", 10)
        self._pregrasp_pose_pub = self.create_publisher(PoseStamped, "/pregrasp_pose", 10)
        self._execute_cli = self.create_client(Trigger, "/grasp_execute")

        self.create_subscription(Image, self._color_topic, self._color_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self._depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self._color_info_topic, self._info_callback, qos_profile_sensor_data)
        self.create_timer(0.03, self._ui_tick)

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._window_name, self._mouse_callback, self)

        self.get_logger().info(
            f"Click grasp ready. Left-click target, press 'g' to execute grasp. "
            f"color={self._color_topic}, depth={self._depth_topic}"
        )

    def move_to_observation_pose(self) -> bool:
        if self._auto_enable and not call_enable(self, self._arm_namespace):
            return False
        return move_to_pose(
            self,
            self._observation_pose,
            self._observation_duration,
            namespace=self._arm_namespace,
            label="observation_pose",
        )

    def _color_callback(self, msg: Image) -> None:
        self._color_msg = msg
        try:
            encoding = "bgr8" if msg.encoding.lower() in ("rgb8", "bgr8") else "passthrough"
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            if msg.encoding.lower() == "rgb8":
                bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
            self._display_bgr = bgr.copy()
        except Exception as exc:
            self.get_logger().warning(f"Color decode failed: {exc}")

    def _depth_callback(self, msg: Image) -> None:
        self._depth_msg = msg

    def _info_callback(self, msg: CameraInfo) -> None:
        self._color_info = msg
        if self._optical_frame == "camera_color_optical_frame" and msg.header.frame_id:
            self._optical_frame = msg.header.frame_id

    @staticmethod
    def _mouse_callback(event: int, x: int, y: int, _flags: int, node: "GraspClickNode") -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            node._pending_click = (x, y)

    def _lookup_transform(self, stamp):
        timeout = Duration(seconds=0.5)
        try:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                self._optical_frame,
                stamp,
                timeout=timeout,
            )
        except Exception:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                self._optical_frame,
                Time(),
                timeout=timeout,
            )

    @staticmethod
    def _depth_value_to_meters(raw: float, encoding: str, depth_scale: float) -> float:
        if not math.isfinite(raw):
            return float("nan")
        enc = encoding.lower()
        if enc in ("32fc1", "64fc1"):
            return float(raw)
        return float(raw) * depth_scale

    def _sample_depth_m(self, u: int, v: int) -> Optional[float]:
        if self._depth_msg is None or self._color_msg is None:
            return None

        depth = self._bridge.imgmsg_to_cv2(self._depth_msg, desired_encoding="passthrough")
        if depth.ndim != 2:
            return None

        color_h, color_w = self._color_msg.height, self._color_msg.width
        depth_h, depth_w = depth.shape[:2]
        u_d = int(round(u * depth_w / max(color_w, 1)))
        v_d = int(round(v * depth_h / max(color_h, 1)))

        r = max(self._patch_radius, 0)
        u0 = max(u_d - r, 0)
        u1 = min(u_d + r + 1, depth_w)
        v0 = max(v_d - r, 0)
        v1 = min(v_d + r + 1, depth_h)
        patch = depth[v0:v1, u0:u1].astype(np.float64).reshape(-1)

        values = [
            self._depth_value_to_meters(v, self._depth_msg.encoding, self._depth_scale)
            for v in patch
        ]
        valid = [v for v in values if self._depth_min <= v <= self._depth_max]
        if not valid:
            return None
        return float(np.median(valid))

    def _pixel_to_optical(self, u: int, v: int, depth_m: float) -> Optional[tuple[float, float, float]]:
        if self._color_info is None:
            return None
        fx = float(self._color_info.k[0])
        fy = float(self._color_info.k[4])
        cx = float(self._color_info.k[2])
        cy = float(self._color_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None

        x = (float(u) - cx) * depth_m / fx
        y = (float(v) - cy) * depth_m / fy
        z = depth_m
        return x, y, z

    def _process_click(self, u: int, v: int) -> None:
        depth_m = self._sample_depth_m(u, v)
        if depth_m is None:
            self.get_logger().warning(f"No valid depth near pixel ({u}, {v})")
            return

        optical_xyz = self._pixel_to_optical(u, v, depth_m)
        if optical_xyz is None:
            self.get_logger().warning("CameraInfo missing; cannot unproject pixel")
            return

        stamp = self._color_msg.header.stamp if self._color_msg is not None else self.get_clock().now().to_msg()
        pt_optical = PointStamped()
        pt_optical.header.stamp = stamp
        pt_optical.header.frame_id = self._optical_frame
        pt_optical.point.x, pt_optical.point.y, pt_optical.point.z = optical_xyz

        try:
            transform = self._lookup_transform(stamp)
            pt_base = do_transform_point(pt_optical, transform)
        except Exception as exc:
            self.get_logger().warning(
                f"TF failed ({self._optical_frame} -> {self._base_frame}): {exc}"
            )
            return

        gx = float(pt_base.point.x)
        gy = float(pt_base.point.y)
        gz = float(pt_base.point.z)
        out_stamp = self.get_clock().now().to_msg()

        publish_grasp_targets(
            base_frame=self._base_frame,
            stamp=out_stamp,
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

        self._last_click_uv = (u, v)
        self._last_grasp_base = (gx, gy, gz)
        self.get_logger().info(
            f"click=({u},{v}) depth={depth_m:.3f} m -> base_link grasp=({gx:.3f}, {gy:.3f}, {gz:.3f})"
        )

    def _ui_tick(self) -> None:
        if self._pending_click is not None:
            u, v = self._pending_click
            self._pending_click = None
            self._process_click(u, v)

        if self._display_bgr is None:
            return

        frame = self._display_bgr.copy()
        if self._last_click_uv is not None:
            u, v = self._last_click_uv
            cv2.circle(frame, (u, v), 8, (0, 255, 255), 2)
            if self._last_grasp_base is not None:
                gx, gy, gz = self._last_grasp_base
                cv2.putText(
                    frame,
                    f"base ({gx:.3f}, {gy:.3f}, {gz:.3f})",
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
        cv2.putText(
            frame,
            "Left click: target | g: execute grasp",
            (10, frame.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("g"), ord("G")):
            self._trigger_grasp_execute()

    def _trigger_grasp_execute(self) -> None:
        if not self._execute_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning(
                "grasp_execute not available; launch rebot_visual_grasp grasp_execute.launch.py"
            )
            return
        future = self._execute_cli.call_async(Trigger.Request())
        future.add_done_callback(self._execute_done)

    def _execute_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"grasp_execute failed: {exc}")
            return
        if result.success:
            self.get_logger().info(f"grasp_execute: {result.message}")
        else:
            self.get_logger().error(f"grasp_execute: {result.message}")


def main() -> None:
    rclpy.init()
    node = GraspClickNode()
    try:
        if node._move_to_observation:
            node.get_logger().info("Moving to observation pose before click UI...")
            if not node.move_to_observation_pose():
                node.get_logger().error(
                    "Observation move failed; fix pose/enable or set move_to_observation_on_start:=false"
                )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
