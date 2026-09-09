"""Shared grasp pose / RViz marker helpers."""

from __future__ import annotations

import math

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import ColorRGBA
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def quat_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
    return quaternion_from_euler(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )


def make_pose(x: float, y: float, z: float, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    qx, qy, qz, qw = quat_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def arrow_marker(
    marker_id: int,
    frame_id: str,
    stamp,
    pose: Pose,
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "grasp"
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.012
    marker.scale.y = 0.024
    marker.scale.z = 0.03
    marker.color = color
    marker.lifetime.sec = 0
    return marker


def sphere_marker(
    marker_id: int,
    frame_id: str,
    stamp,
    x: float,
    y: float,
    z: float,
    color: ColorRGBA,
    *,
    diameter: float = 0.025,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "grasp"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = diameter
    marker.color = color
    marker.lifetime.sec = 0
    return marker


def publish_grasp_poses(
    *,
    base_frame: str,
    stamp,
    grasp_pose: Pose,
    pregrasp_pose: Pose,
    marker_pub,
    grasp_pose_pub,
    pregrasp_pose_pub,
) -> None:
    grasp_msg = PoseStamped()
    grasp_msg.header.stamp = stamp
    grasp_msg.header.frame_id = base_frame
    grasp_msg.pose = grasp_pose
    grasp_pose_pub.publish(grasp_msg)

    pregrasp_msg = PoseStamped()
    pregrasp_msg.header.stamp = stamp
    pregrasp_msg.header.frame_id = base_frame
    pregrasp_msg.pose = pregrasp_pose
    pregrasp_pose_pub.publish(pregrasp_msg)

    red = ColorRGBA(r=1.0, g=0.15, b=0.15, a=0.95)
    green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.95)
    yellow = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.95)

    markers = MarkerArray()
    markers.markers.append(arrow_marker(0, base_frame, stamp, grasp_pose, red))
    markers.markers.append(arrow_marker(1, base_frame, stamp, pregrasp_pose, green))
    markers.markers.append(
        sphere_marker(
            2,
            base_frame,
            stamp,
            grasp_pose.position.x,
            grasp_pose.position.y,
            grasp_pose.position.z,
            yellow,
        )
    )
    marker_pub.publish(markers)


def publish_grasp_targets(
    *,
    base_frame: str,
    stamp,
    gx: float,
    gy: float,
    gz: float,
    pregrasp_offset_m: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    marker_pub,
    grasp_pose_pub,
    pregrasp_pose_pub,
) -> tuple[Pose, Pose]:
    pregrasp_z = gz + pregrasp_offset_m
    grasp_pose = make_pose(gx, gy, gz, roll_deg, pitch_deg, yaw_deg)
    pregrasp_pose = make_pose(gx, gy, pregrasp_z, roll_deg, pitch_deg, yaw_deg)

    grasp_msg = PoseStamped()
    grasp_msg.header.stamp = stamp
    grasp_msg.header.frame_id = base_frame
    grasp_msg.pose = grasp_pose
    grasp_pose_pub.publish(grasp_msg)

    pregrasp_msg = PoseStamped()
    pregrasp_msg.header.stamp = stamp
    pregrasp_msg.header.frame_id = base_frame
    pregrasp_msg.pose = pregrasp_pose
    pregrasp_pose_pub.publish(pregrasp_msg)

    red = ColorRGBA(r=1.0, g=0.15, b=0.15, a=0.95)
    green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.95)
    yellow = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.95)

    markers = MarkerArray()
    markers.markers.append(arrow_marker(0, base_frame, stamp, grasp_pose, red))
    markers.markers.append(arrow_marker(1, base_frame, stamp, pregrasp_pose, green))
    markers.markers.append(sphere_marker(2, base_frame, stamp, gx, gy, gz, yellow))
    marker_pub.publish(markers)
    return grasp_pose, pregrasp_pose


def publish_place_pose(
    *,
    base_frame: str,
    stamp,
    place_pose: Pose,
    marker_pub,
    place_pose_pub,
) -> None:
    """Publish place/box pose and a blue RViz arrow+sphere."""
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = base_frame
    msg.pose = place_pose
    place_pose_pub.publish(msg)

    blue = ColorRGBA(r=0.2, g=0.45, b=1.0, a=0.95)
    cyan = ColorRGBA(r=0.1, g=0.9, b=0.95, a=0.95)
    markers = MarkerArray()
    arrow = arrow_marker(10, base_frame, stamp, place_pose, blue)
    arrow.ns = "place"
    markers.markers.append(arrow)
    sphere = sphere_marker(
        11,
        base_frame,
        stamp,
        place_pose.position.x,
        place_pose.position.y,
        place_pose.position.z,
        cyan,
        diameter=0.03,
    )
    sphere.ns = "place"
    markers.markers.append(sphere)
    marker_pub.publish(markers)
