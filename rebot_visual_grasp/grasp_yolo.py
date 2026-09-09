#!/usr/bin/env python3
"""YOLO detection + depth -> 6D grasp pose for reBot visual grasp."""

from __future__ import annotations

import math
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

from rebotarm_msgs.srv import GripperCommand
from rebot_visual_grasp.arm_motion import call_enable, make_pose, move_to_pose
from rebot_visual_grasp.depth_utils import (
    depth_image_to_mm,
    lookup_cam_to_base_matrix,
    optical_point_to_base,
    optical_points_to_base_yaw,
    sample_depth_in_box,
)
from rebot_visual_grasp.grasp_transforms import (
    rotation_matrix_to_euler_zyx,
    transform_grasp_pose_to_base,
)
from rebot_visual_grasp.grasp_utils import publish_grasp_poses, publish_grasp_targets, publish_place_pose
from rebot_visual_grasp.ordinary_grasp import (
    GraspPose,
    draw_grasp,
    estimate_grasps,
    select_best_grasp,
)
from rebot_visual_grasp.yolo_loader import load_yolo_model, predict_kwargs, ultralytics_available


class Detection:
    __slots__ = (
        "class_name",
        "confidence",
        "x1",
        "y1",
        "x2",
        "y2",
        "center_u",
        "center_v",
        "short_axis_uv",
        "is_obb",
    )

    def __init__(
        self,
        class_name: str,
        confidence: float,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        center_u: int,
        center_v: int,
        short_axis_uv: tuple[int, int],
        is_obb: bool,
    ) -> None:
        self.class_name = class_name
        self.confidence = confidence
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.center_u = center_u
        self.center_v = center_v
        self.short_axis_uv = short_axis_uv
        self.is_obb = is_obb


class GraspYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_yolo")
        if not ultralytics_available():
            raise RuntimeError(
                "ultralytics is not installed. Run: pip install -U ultralytics"
            )

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("ee_frame", "gripper_end")
        self.declare_parameter("yolo_model", "/home/ubuntu/ultralytics-main/yoloe-26s-seg.pt")
        self.declare_parameter("custom_classes", ["pen"])
        self.declare_parameter("yolo_device", "cpu")
        self.declare_parameter("use_yoloe", True)
        self.declare_parameter("target_class", "pen")
        self.declare_parameter("place_class", "box")
        self.declare_parameter("enable_place_detect", True)
        self.declare_parameter("place_z_offset_m", 0.05)
        self.declare_parameter("conf_threshold", 0.4)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_min_m", 0.05)
        self.declare_parameter("depth_max_m", 1.5)
        self.declare_parameter("depth_quantile", 0.5)
        self.declare_parameter("pregrasp_offset_m", 0.08)
        self.declare_parameter("pregrasp_mode", "tool_x")
        self.declare_parameter("grasp_approach_mode", "camera")
        self.declare_parameter("insertion_depth_m", 0.015)
        # base_link trim: negative X pulls grasp back toward the arm (fix "偏前").
        self.declare_parameter("grasp_x_offset_m", 0.0)
        self.declare_parameter("grasp_y_offset_m", 0.0)
        self.declare_parameter("grasp_z_offset_m", 0.03)
        self.declare_parameter("min_base_z_m", 0.0)
        self.declare_parameter("orientation_mode", "seeed")
        self.declare_parameter("grasp_roll_deg", 0.0)
        self.declare_parameter("grasp_pitch_deg", 90.0)
        self.declare_parameter("grasp_yaw_deg", 0.0)
        self.declare_parameter("infer_every_n_frames", 3)
        self.declare_parameter("window_name", "grasp_yolo")
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
        self.declare_parameter("trigger_grasp_on_g", False)
        self.declare_parameter("open_gripper_on_g", False)
        self.declare_parameter("auto_publish_on_detect", True)
        self.declare_parameter("print_pose_every_s", 1.0)
        self.declare_parameter("gripper_timeout", 5.0)

        self._target_class = str(self.get_parameter("target_class").value).strip().lower()
        self._place_class = str(self.get_parameter("place_class").value).strip().lower()
        self._enable_place_detect = bool(self.get_parameter("enable_place_detect").value)
        self._place_z_offset = float(self.get_parameter("place_z_offset_m").value)
        self._conf_threshold = float(self.get_parameter("conf_threshold").value)
        custom_classes_raw = self.get_parameter("custom_classes").value
        if isinstance(custom_classes_raw, (list, tuple)):
            self._custom_classes = [str(name).strip() for name in custom_classes_raw if str(name).strip()]
        else:
            self._custom_classes = [
                part.strip() for part in str(custom_classes_raw).split(",") if part.strip()
            ]
        # YOLOE set_classes must include grasp + place classes we filter on.
        for needed in (self._target_class, self._place_class if self._enable_place_detect else ""):
            if not needed:
                continue
            lowers = {name.lower() for name in self._custom_classes}
            if needed not in lowers:
                self._custom_classes = [needed, *self._custom_classes]
        raw_device = str(self.get_parameter("yolo_device").value).strip()
        self._predict_kwargs = predict_kwargs(raw_device, self._conf_threshold)
        self._yolo_device = str(self._predict_kwargs.get("device", "cpu"))
        self._use_yoloe = bool(self.get_parameter("use_yoloe").value)
        if raw_device.lower() != self._yolo_device:
            self.get_logger().info(
                f"yolo_device mapped '{raw_device}' → '{self._yolo_device}'"
            )
        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._depth_min = float(self.get_parameter("depth_min_m").value)
        self._depth_max = float(self.get_parameter("depth_max_m").value)
        self._depth_quantile = float(self.get_parameter("depth_quantile").value)
        self._pregrasp_offset = float(self.get_parameter("pregrasp_offset_m").value)
        self._pregrasp_mode = str(self.get_parameter("pregrasp_mode").value).strip().lower()
        self._grasp_approach_mode = str(
            self.get_parameter("grasp_approach_mode").value
        ).strip().lower()
        self._insertion_depth = float(self.get_parameter("insertion_depth_m").value)
        self._grasp_x_offset = float(self.get_parameter("grasp_x_offset_m").value)
        self._grasp_y_offset = float(self.get_parameter("grasp_y_offset_m").value)
        self._grasp_z_offset = float(self.get_parameter("grasp_z_offset_m").value)
        self._min_base_z = float(self.get_parameter("min_base_z_m").value)
        self._orientation_mode = str(self.get_parameter("orientation_mode").value)
        self._roll_deg = float(self.get_parameter("grasp_roll_deg").value)
        self._pitch_deg = float(self.get_parameter("grasp_pitch_deg").value)
        self._yaw_deg = float(self.get_parameter("grasp_yaw_deg").value)
        self._infer_every_n = max(int(self.get_parameter("infer_every_n_frames").value), 1)
        self._window_name = self.get_parameter("window_name").value
        self._base_frame = self.get_parameter("base_frame").value
        self._optical_frame = self.get_parameter("optical_frame").value
        self._ee_frame = self.get_parameter("ee_frame").value
        self._trigger_grasp_on_g = bool(self.get_parameter("trigger_grasp_on_g").value)
        self._open_gripper_on_g = bool(self.get_parameter("open_gripper_on_g").value)
        self._auto_publish_on_detect = bool(
            self.get_parameter("auto_publish_on_detect").value
        )
        self._print_pose_every_s = max(
            float(self.get_parameter("print_pose_every_s").value), 0.0
        )
        self._gripper_timeout = float(self.get_parameter("gripper_timeout").value)
        self._last_pose_print_time = 0.0
        self._last_det_summary_time = 0.0

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

        model_path = str(self.get_parameter("yolo_model").value)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self._model = load_yolo_model(
            model_path,
            custom_classes=self._custom_classes,
            use_yoloe=self._use_yoloe,
        )
        if self._custom_classes:
            self.get_logger().info(f"YOLO custom classes: {self._custom_classes}")

        self._bridge = CvBridge()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._color_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None
        self._color_info: Optional[CameraInfo] = None
        self._display_bgr: Optional[np.ndarray] = None
        self._depth_img: Optional[np.ndarray] = None
        self._frame_counter = 0
        self._frozen = False
        # Avoid YOLO predict while observation move is still spinning the executor.
        self._inference_enabled = not self._move_to_observation
        self._detections: list[Detection] = []
        self._best: Optional[Detection] = None
        self._last_results: list[Any] = []
        self._last_seeed_grasps: list[GraspPose] = []
        self._best_seeed: Optional[GraspPose] = None
        self._best_place_seeed: Optional[GraspPose] = None
        self._last_grasp_base: Optional[tuple[float, float, float]] = None
        self._last_place_base: Optional[tuple[float, float, float]] = None
        self._last_place_print_time = 0.0

        self._marker_pub = self.create_publisher(MarkerArray, "/grasp_markers", 10)
        self._place_marker_pub = self.create_publisher(MarkerArray, "/place_markers", 10)
        self._grasp_pose_pub = self.create_publisher(PoseStamped, "/grasp_pose", 10)
        self._pregrasp_pose_pub = self.create_publisher(PoseStamped, "/pregrasp_pose", 10)
        self._place_pose_pub = self.create_publisher(PoseStamped, "/place_pose", 10)
        self._execute_cli = self.create_client(Trigger, "/grasp_execute")
        self._enable_cli = self.create_client(Trigger, f"/{self._arm_namespace}/enable")
        self._open_cli = self.create_client(
            GripperCommand, f"/{self._arm_namespace}/gripper/open"
        )

        self.create_subscription(Image, self.get_parameter("color_topic").value, self._color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("depth_topic").value, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter("color_info_topic").value, self._info_cb, qos_profile_sensor_data)
        self.create_timer(0.03, self._ui_tick)

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            "YOLO detect-only ready: after observation pose, auto-detect → "
            "publish markers + print 6D (no arm/gripper motion). "
            "G=freeze frame, R=live, Q=quit. "
            f"class filter='{self._target_class or '*'}', "
            f"place_class='{self._place_class if self._enable_place_detect else 'off'}', "
            f"orientation={self._orientation_mode}, "
            f"pregrasp_mode={self._pregrasp_mode}, "
            f"grasp_approach_mode={self._grasp_approach_mode}, "
            f"auto_publish={self._auto_publish_on_detect}, "
            f"grasp_xyz_offset=({self._grasp_x_offset:.3f}, "
            f"{self._grasp_y_offset:.3f}, {self._grasp_z_offset:.3f})m, "
            f"place_z_offset={self._place_z_offset:.3f}m, "
            f"topics=/grasp_pose,/pregrasp_pose,/place_pose, "
            f"open_gripper_on_g={self._open_gripper_on_g}, "
            f"trigger_grasp_on_g={self._trigger_grasp_on_g}"
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

    def _color_cb(self, msg: Image) -> None:
        self._color_msg = msg
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._display_bgr = bgr.copy()
        except Exception:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if msg.encoding.lower() == "rgb8":
                self._display_bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
            else:
                self._display_bgr = bgr.copy()

    def _depth_cb(self, msg: Image) -> None:
        self._depth_msg = msg
        self._depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _info_cb(self, msg: CameraInfo) -> None:
        self._color_info = msg
        if self._optical_frame == "camera_color_optical_frame" and msg.header.frame_id:
            self._optical_frame = msg.header.frame_id

    def _scale_uv_to_depth(self, u: int, v: int) -> tuple[int, int]:
        if self._color_msg is None or self._depth_img is None:
            return u, v
        color_h, color_w = self._color_msg.height, self._color_msg.width
        depth_h, depth_w = self._depth_img.shape[:2]
        u_d = int(round(u * depth_w / max(color_w, 1)))
        v_d = int(round(v * depth_h / max(color_h, 1)))
        return u_d, v_d

    def _scale_box_to_depth(self, det: Detection) -> tuple[int, int, int, int]:
        u1, v1 = self._scale_uv_to_depth(det.x1, det.y1)
        u2, v2 = self._scale_uv_to_depth(det.x2, det.y2)
        return min(u1, u2), min(v1, v2), max(u1, u2), max(v1, v2)

    def _parse_results(self, results: Any) -> list[Detection]:
        detections: list[Detection] = []
        if not results:
            return detections
        result = results[0]
        names = result.names

        if getattr(result, "obb", None) is not None and len(result.obb) > 0:
            for item in result.obb:
                conf = float(item.conf.item())
                if conf < self._conf_threshold:
                    continue
                cls_id = int(item.cls.item())
                class_name = str(names.get(cls_id, cls_id))
                if not self._class_allowed(class_name):
                    continue
                cx, cy, w, h, rot = (float(v) for v in item.xywhr[0].tolist())
                half_w = w * 0.5
                half_h = h * 0.5
                cos_r = math.cos(rot)
                sin_r = math.sin(rot)
                corners = []
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    lx = sx * half_w
                    ly = sy * half_h
                    x = cx + lx * cos_r - ly * sin_r
                    y = cy + lx * sin_r + ly * cos_r
                    corners.append((x, y))
                xs = [c[0] for c in corners]
                ys = [c[1] for c in corners]
                x1, y1, x2, y2 = int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))
                if w <= h:
                    axis_len = half_w
                    axis_angle = rot
                else:
                    axis_len = half_h
                    axis_angle = rot + math.pi * 0.5
                axis_u = int(round(cx + axis_len * math.cos(axis_angle)))
                axis_v = int(round(cy + axis_len * math.sin(axis_angle)))
                detections.append(
                    Detection(
                        class_name=class_name,
                        confidence=conf,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                        center_u=int(round(cx)),
                        center_v=int(round(cy)),
                        short_axis_uv=(axis_u, axis_v),
                        is_obb=True,
                    )
                )
            return detections

        boxes = result.boxes
        if boxes is None:
            return detections
        for box in boxes:
            conf = float(box.conf.item())
            if conf < self._conf_threshold:
                continue
            cls_id = int(box.cls.item())
            class_name = str(names.get(cls_id, cls_id))
            if not self._class_allowed(class_name):
                continue
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            w = max(x2 - x1, 1)
            h = max(y2 - y1, 1)
            if w >= h:
                axis_u = x2
                axis_v = cy
            else:
                axis_u = cx
                axis_v = y2
            detections.append(
                Detection(
                    class_name=class_name,
                    confidence=conf,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    center_u=cx,
                    center_v=cy,
                    short_axis_uv=(axis_u, axis_v),
                    is_obb=False,
                )
            )
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections

    def _camera_matrix(self) -> Optional[np.ndarray]:
        if self._color_info is None:
            return None
        return np.array(self._color_info.k, dtype=np.float32).reshape(3, 3)

    def _depth_to_mm(self) -> Optional[np.ndarray]:
        if self._depth_img is None or self._depth_msg is None:
            return None
        return depth_image_to_mm(self._depth_img, self._depth_msg.encoding, self._depth_scale)

    def _class_allowed(self, class_name: str) -> bool:
        """Keep grasp target and place class; empty target_class keeps all."""
        name = class_name.lower()
        if not self._target_class:
            return True
        if name == self._target_class:
            return True
        if self._enable_place_detect and self._place_class and name == self._place_class:
            return True
        return False

    def _update_seeed_grasps(self, results: list[Any]) -> None:
        depth_mm = self._depth_to_mm()
        k_matrix = self._camera_matrix()
        if depth_mm is None or k_matrix is None:
            self._last_seeed_grasps = []
            self._best_seeed = None
            self._best_place_seeed = None
            return
        self._last_seeed_grasps = estimate_grasps(
            results,
            depth_mm,
            k_matrix,
            depth_quantile=self._depth_quantile,
        )
        self._best_seeed = select_best_grasp(
            self._last_seeed_grasps,
            target_class=self._target_class,
        )
        if self._enable_place_detect and self._place_class:
            self._best_place_seeed = select_best_grasp(
                self._last_seeed_grasps,
                target_class=self._place_class,
            )
        else:
            self._best_place_seeed = None

    def _detection_summary(self) -> str:
        if not self._last_seeed_grasps:
            return "none"
        parts: list[str] = []
        for grasp in self._last_seeed_grasps:
            status = "ok" if grasp.is_valid else (grasp.rejected_reason or "bad")
            parts.append(f"{grasp.class_name}:{grasp.conf:.2f}/{status}")
        return " | ".join(parts)

    def _run_inference(self) -> None:
        if self._display_bgr is None:
            return
        results = self._model.predict(
            self._display_bgr,
            **self._predict_kwargs,
        )
        self._last_results = results
        if self._orientation_mode == "seeed":
            self._update_seeed_grasps(results)
            if self._should_print_det_summary():
                self.get_logger().info(f"YOLO dets: {self._detection_summary()}")
        else:
            self._detections = self._parse_results(results)
            self._best = self._detections[0] if self._detections else None
        if self._auto_publish_on_detect:
            self._publish_current_grasp(force_log=False)
            self._publish_current_place(force_log=False)

    def _should_print_det_summary(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._print_pose_every_s <= 0.0:
            return True
        if (now - self._last_det_summary_time) >= self._print_pose_every_s:
            self._last_det_summary_time = now
            return True
        return False

    def _should_print_pose(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._print_pose_every_s <= 0.0:
            return True
        if (now - self._last_pose_print_time) >= self._print_pose_every_s:
            self._last_pose_print_time = now
            return True
        return False

    def _should_print_place_pose(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._print_pose_every_s <= 0.0:
            return True
        if (now - self._last_place_print_time) >= self._print_pose_every_s:
            self._last_place_print_time = now
            return True
        return False

    def _log_6d(
        self,
        label: str,
        *,
        class_name: str,
        conf: float,
        grasp_pose,
        pregrasp_pose=None,
        extra: str = "",
    ) -> None:
        g = grasp_pose.position
        o = grasp_pose.orientation
        msg = (
            f"{label} '{class_name}' conf={conf:.2f} "
            f"grasp_xyz=({g.x:.3f}, {g.y:.3f}, {g.z:.3f}) "
            f"grasp_quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
        )
        if pregrasp_pose is not None:
            p = pregrasp_pose.position
            po = pregrasp_pose.orientation
            msg += (
                f" pregrasp_xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"pregrasp_quat=({po.x:.3f}, {po.y:.3f}, {po.z:.3f}, {po.w:.3f})"
            )
        if extra:
            msg += f" {extra}"
        self.get_logger().info(msg)

    def _publish_current_grasp(self, *, force_log: bool = True) -> bool:
        if self._orientation_mode == "seeed":
            return self._publish_cached_seeed_grasp(force_log=force_log)
        if self._best is None:
            return False
        return self._publish_detection_grasp(self._best, force_log=force_log)

    def _publish_current_place(self, *, force_log: bool = True) -> bool:
        if not self._enable_place_detect or not self._place_class:
            return False
        if self._orientation_mode == "seeed":
            return self._publish_cached_place_pose(force_log=force_log)
        return False

    def _publish_cached_place_pose(self, *, force_log: bool = True) -> bool:
        place = self._best_place_seeed
        if place is None:
            return False
        if self._color_info is None or self._depth_msg is None:
            return False

        stamp = (
            self._color_msg.header.stamp
            if self._color_msg is not None
            else self.get_clock().now().to_msg()
        )
        cam_to_base = lookup_cam_to_base_matrix(
            self,
            self._tf_buffer,
            self._base_frame,
            self._optical_frame,
            stamp,
        )
        if cam_to_base is None:
            return False

        place_pose, _pregrasp = transform_grasp_pose_to_base(
            place.position,
            place.tcp_rotation,
            cam_to_base,
            self._pregrasp_offset,
            0.0,
            pregrasp_mode=self._pregrasp_mode,
            approach_mode=self._grasp_approach_mode,
        )
        # Place point: box surface + vertical offset (drop height above box).
        place_pose.position.x += self._grasp_x_offset
        place_pose.position.y += self._grasp_y_offset
        place_pose.position.z += self._place_z_offset
        if place_pose.position.z < self._min_base_z:
            if force_log:
                self.get_logger().warning(
                    f"Place z={place_pose.position.z:.3f} below min_base_z={self._min_base_z:.3f}"
                )
            return False

        publish_place_pose(
            base_frame=self._base_frame,
            stamp=stamp,
            place_pose=place_pose,
            marker_pub=self._place_marker_pub,
            place_pose_pub=self._place_pose_pub,
        )
        self._last_place_base = (
            float(place_pose.position.x),
            float(place_pose.position.y),
            float(place_pose.position.z),
        )
        if force_log or self._should_print_place_pose():
            self._log_6d(
                "Place 6D",
                class_name=place.class_name,
                conf=float(place.conf),
                grasp_pose=place_pose,
                extra=f"topic=/place_pose place_z_offset={self._place_z_offset:.3f}",
            )
        return True

    def _lookup_ee_rpy_deg(self) -> Optional[tuple[float, float, float]]:
        try:
            timeout = Duration(seconds=0.5)
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame, self._ee_frame, Time(), timeout=timeout
                )
            except Exception:
                return None
            q = transform.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        except Exception:
            return None

    def _detection_to_grasp(self, det: Detection) -> Optional[tuple[float, float, float, float, float, float]]:
        if self._depth_img is None or self._depth_msg is None or self._color_info is None:
            return None
        x1, y1, x2, y2 = self._scale_box_to_depth(det)
        sampled = sample_depth_in_box(
            self._depth_img,
            self._depth_msg.encoding,
            self._depth_scale,
            x1,
            y1,
            x2,
            y2,
            depth_min_m=self._depth_min,
            depth_max_m=self._depth_max,
            quantile=self._depth_quantile,
        )
        if sampled is None:
            return None
        depth_m, _, _ = sampled
        u, v = det.center_u, det.center_v
        stamp = self._color_msg.header.stamp if self._color_msg is not None else self.get_clock().now().to_msg()
        base_xyz = optical_point_to_base(
            self,
            self._tf_buffer,
            self._base_frame,
            self._optical_frame,
            u,
            v,
            depth_m,
            self._color_info,
            stamp,
        )
        if base_xyz is None:
            return None
        gx, gy, gz = base_xyz

        if self._orientation_mode == "gripper_tf":
            rpy = self._lookup_ee_rpy_deg()
            if rpy is None:
                return None
            roll_deg, pitch_deg, yaw_deg = rpy
        elif self._orientation_mode == "yaw_from_obb":
            yaw = optical_points_to_base_yaw(
                self,
                self._tf_buffer,
                self._base_frame,
                self._optical_frame,
                (u, v),
                det.short_axis_uv,
                depth_m,
                self._color_info,
                stamp,
            )
            roll_deg = self._roll_deg
            pitch_deg = self._pitch_deg
            yaw_deg = math.degrees(yaw) if yaw is not None else self._yaw_deg
        else:
            roll_deg = self._roll_deg
            pitch_deg = self._pitch_deg
            yaw_deg = self._yaw_deg

        return gx, gy, gz, roll_deg, pitch_deg, yaw_deg

    def _publish_seeed_grasp(self) -> bool:
        if self._display_bgr is None:
            return False
        results = self._model.predict(
            self._display_bgr,
            **self._predict_kwargs,
        )
        self._last_results = results
        self._update_seeed_grasps(results)
        return self._publish_cached_seeed_grasp(force_log=True)

    def _publish_cached_seeed_grasp(self, *, force_log: bool = True) -> bool:
        grasp = self._best_seeed
        if grasp is None:
            return False
        if self._color_info is None or self._depth_msg is None:
            return False

        stamp = self._color_msg.header.stamp if self._color_msg is not None else self.get_clock().now().to_msg()
        cam_to_base = lookup_cam_to_base_matrix(
            self,
            self._tf_buffer,
            self._base_frame,
            self._optical_frame,
            stamp,
        )
        if cam_to_base is None:
            return False

        grasp_pose, pregrasp_pose = transform_grasp_pose_to_base(
            grasp.position,
            grasp.tcp_rotation,
            cam_to_base,
            self._pregrasp_offset,
            self._insertion_depth,
            pregrasp_mode=self._pregrasp_mode,
            approach_mode=self._grasp_approach_mode,
        )
        grasp_pose.position.x += self._grasp_x_offset
        grasp_pose.position.y += self._grasp_y_offset
        pregrasp_pose.position.x += self._grasp_x_offset
        pregrasp_pose.position.y += self._grasp_y_offset
        if self._grasp_z_offset != 0.0:
            grasp_pose.position.z += self._grasp_z_offset
            pregrasp_pose.position.z += self._grasp_z_offset
        if grasp_pose.position.z < self._min_base_z:
            if force_log:
                self.get_logger().warning(
                    f"Grasp z={grasp_pose.position.z:.3f} below min_base_z={self._min_base_z:.3f}"
                )
            return False

        publish_grasp_poses(
            base_frame=self._base_frame,
            stamp=stamp,
            grasp_pose=grasp_pose,
            pregrasp_pose=pregrasp_pose,
            marker_pub=self._marker_pub,
            grasp_pose_pub=self._grasp_pose_pub,
            pregrasp_pose_pub=self._pregrasp_pose_pub,
        )
        self._last_grasp_base = (
            float(grasp_pose.position.x),
            float(grasp_pose.position.y),
            float(grasp_pose.position.z),
        )
        if force_log or self._should_print_pose():
            tcp_rpy = rotation_matrix_to_euler_zyx(
                np.array(
                    [
                        [grasp.tcp_rotation[0, 0], grasp.tcp_rotation[0, 1], grasp.tcp_rotation[0, 2]],
                        [grasp.tcp_rotation[1, 0], grasp.tcp_rotation[1, 1], grasp.tcp_rotation[1, 2]],
                        [grasp.tcp_rotation[2, 0], grasp.tcp_rotation[2, 1], grasp.tcp_rotation[2, 2]],
                    ]
                )
            )
            self._log_6d(
                "Seeed 6D",
                class_name=grasp.class_name,
                conf=float(grasp.conf),
                grasp_pose=grasp_pose,
                pregrasp_pose=pregrasp_pose,
                extra=(
                    f"cam_tcp_rpy=({math.degrees(tcp_rpy[0]):.1f}, "
                    f"{math.degrees(tcp_rpy[1]):.1f}, {math.degrees(tcp_rpy[2]):.1f})"
                ),
            )
        return True

    def _publish_detection_grasp(self, det: Detection, *, force_log: bool = True) -> bool:
        grasp = self._detection_to_grasp(det)
        if grasp is None:
            if force_log:
                self.get_logger().warning(
                    f"Failed to build grasp pose for '{det.class_name}' "
                    f"({det.confidence:.2f})"
                )
            return False
        gx, gy, gz, roll_deg, pitch_deg, yaw_deg = grasp
        gx += self._grasp_x_offset
        gy += self._grasp_y_offset
        gz += self._grasp_z_offset
        stamp = self.get_clock().now().to_msg()
        grasp_pose, pregrasp_pose = publish_grasp_targets(
            base_frame=self._base_frame,
            stamp=stamp,
            gx=gx,
            gy=gy,
            gz=gz,
            pregrasp_offset_m=self._pregrasp_offset,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            marker_pub=self._marker_pub,
            grasp_pose_pub=self._grasp_pose_pub,
            pregrasp_pose_pub=self._pregrasp_pose_pub,
        )
        self._last_grasp_base = (gx, gy, gz)
        if force_log or self._should_print_pose():
            self._log_6d(
                "YOLO 6D",
                class_name=det.class_name,
                conf=float(det.confidence),
                grasp_pose=grasp_pose,
                pregrasp_pose=pregrasp_pose,
                extra=f"rpy_deg=({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f})",
            )
        return True

    def _open_gripper_async(self) -> None:
        """Open jaw only; no arm motion. Async so UI timer does not deadlock."""
        if self._auto_enable:
            if not self._enable_cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warning("enable not available; trying open anyway")
                self._call_open_gripper()
                return
            fut = self._enable_cli.call_async(Trigger.Request())
            fut.add_done_callback(self._enable_then_open)
            return
        self._call_open_gripper()

    def _enable_then_open(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"enable failed: {exc}")
            return
        if result is None or not result.success:
            msg = result.message if result is not None else "no response"
            self.get_logger().error(f"enable failed: {msg}")
            return
        self.get_logger().info(result.message or "enabled")
        self._call_open_gripper()

    def _call_open_gripper(self) -> None:
        if not self._open_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().error("gripper/open not available")
            return
        req = GripperCommand.Request()
        req.timeout = float(self._gripper_timeout)
        fut = self._open_cli.call_async(req)
        fut.add_done_callback(self._open_gripper_done)

    def _open_gripper_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"gripper open failed: {exc}")
            return
        if result is None or not result.success:
            msg = result.message if result is not None else "no response"
            self.get_logger().error(f"gripper open failed: {msg}")
            return
        self.get_logger().info(
            f"gripper opened (reached={result.reached_position:.3f}) — "
            "arm still at observation; call /grasp_execute when ready"
        )

    def _trigger_grasp_execute(self) -> None:
        if not self._execute_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning("grasp_execute not available")
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

    def _draw_overlay(self, frame: np.ndarray) -> None:
        if self._orientation_mode == "seeed":
            for grasp in self._last_seeed_grasps:
                name = grasp.class_name.lower()
                if self._place_class and name == self._place_class:
                    color = (0, 180, 255)  # place = orange
                elif self._target_class and name == self._target_class:
                    color = (0, 255, 0) if grasp.is_valid else (0, 165, 255)
                else:
                    color = (180, 180, 180)
                draw_grasp(frame, grasp, color=color)
            cv2.putText(
                frame,
                f"dets: {self._detection_summary()}",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
        for det in self._detections:
            color = (0, 255, 0) if det is self._best else (255, 180, 0)
            cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), color, 2)
            label = f"{det.class_name} {det.confidence:.2f}"
            cv2.putText(
                frame,
                label,
                (det.x1, max(det.y1 - 8, 16)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.circle(frame, (det.center_u, det.center_v), 4, (0, 255, 255), -1)
            cv2.line(
                frame,
                (det.center_u, det.center_v),
                det.short_axis_uv,
                (255, 0, 255),
                2,
            )
        if self._last_grasp_base is not None:
            gx, gy, gz = self._last_grasp_base
            cv2.putText(
                frame,
                f"grasp base ({gx:.3f}, {gy:.3f}, {gz:.3f})",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        if self._last_place_base is not None:
            px, py, pz = self._last_place_base
            cv2.putText(
                frame,
                f"place base ({px:.3f}, {py:.3f}, {pz:.3f})",
                (10, 54),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 180, 0),
                2,
                cv2.LINE_AA,
            )
        state = "FROZEN" if self._frozen else "LIVE"
        cv2.putText(
            frame,
            f"{state} | grasp+place 6D | /place_pose | G=freeze R=live Q=quit",
            (10, frame.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def _ui_tick(self) -> None:
        if self._display_bgr is None:
            return

        if not self._frozen and self._inference_enabled:
            self._frame_counter += 1
            if self._frame_counter % self._infer_every_n == 0:
                self._run_inference()

        frame = self._display_bgr.copy()
        self._draw_overlay(frame)
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("g"), ord("G")):
            # Freeze frame and force one publish/print; never move arm by default.
            self._frozen = True
            if self._orientation_mode == "seeed":
                published = self._publish_seeed_grasp()
                self._publish_current_place(force_log=True)
            elif self._best is None:
                self.get_logger().warning("No YOLO detection to publish")
                published = False
            else:
                self._run_inference()
                published = self._publish_detection_grasp(self._best, force_log=True)
            if published:
                if self._open_gripper_on_g:
                    self.get_logger().info(
                        "Opening gripper only (no pregrasp/grasp motion yet)"
                    )
                    self._open_gripper_async()
                if self._trigger_grasp_on_g:
                    self._trigger_grasp_execute()
        elif key in (ord("r"), ord("R")):
            self._frozen = False
        elif key in (ord("q"), ord("Q"), 27):
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = GraspYoloNode()
    try:
        if node._move_to_observation:
            node.get_logger().info("Moving to observation pose before YOLO...")
            if not node.move_to_observation_pose():
                node.get_logger().error("Observation move failed")
        node._inference_enabled = True
        node.get_logger().info(
            f"YOLO inference enabled on device='{node._yolo_device}'"
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
