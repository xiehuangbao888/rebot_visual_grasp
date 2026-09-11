from __future__ import annotations

import math
import re
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from rebot_visual_grasp.paths import (
    assembly_cfg,
    resolve_assembly,
    writable_extrinsics,
    writable_xacro,
)

POSE_PATH = Path("/tmp/camera_pose.txt")


def quat_to_rpy(q: Quaternion):
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def format_params(pose):
    r, p, y = quat_to_rpy(pose.orientation)
    xyz = f"{pose.position.x:.4f} {pose.position.y:.4f} {pose.position.z:.4f}"
    rpy = f"{r:.4f} {p:.4f} {y:.4f}"
    return xyz, rpy


def _parse_xyz_rpy_from_xacro(text: str) -> Pose | None:
    xyz_m = re.search(r'<xacro:property name="camera_xyz" value="([^"]*)"/>', text)
    rpy_m = re.search(r'<xacro:property name="camera_rpy" value="([^"]*)"/>', text)
    if not xyz_m or not rpy_m:
        return None
    try:
        x, y, z = [float(v) for v in xyz_m.group(1).split()]
        roll, pitch, yaw = [float(v) for v in rpy_m.group(1).split()]
    except ValueError:
        return None
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def load_current_camera_pose(assembly: str) -> Pose:
    """Start from the already-saved camera↔mount assembly in xacro."""
    xacro_path = writable_xacro(assembly)
    if xacro_path.is_file():
        pose = _parse_xyz_rpy_from_xacro(xacro_path.read_text())
        if pose is not None:
            return pose
    pose = Pose()
    pose.orientation.w = 1.0
    return pose


def _write_extrinsics_camera(assembly: str, xyz: str, rpy: str) -> None:
    path = writable_extrinsics(assembly)
    if not path.is_file():
        return
    xs = [float(v) for v in xyz.split()]
    rs = [float(v) for v in rpy.split()]
    text = path.read_text()
    text = re.sub(
        r"(camera:\n(?:.*\n)*?\s+xyz:\s*)\[[^\]]*\]",
        rf"\g<1>[{xs[0]:.4f}, {xs[1]:.4f}, {xs[2]:.4f}]",
        text,
        count=1,
    )
    text = re.sub(
        r"(camera:\n(?:.*\n)*?\s+rpy:\s*)\[[^\]]*\]",
        rf"\g<1>[{rs[0]:.4f}, {rs[1]:.4f}, {rs[2]:.4f}]",
        text,
        count=1,
    )
    path.write_text(text)


def write_params(assembly: str, xyz, rpy):
    text = (
        f'<xacro:property name="camera_xyz" value="{xyz}"/>\n'
        f'<xacro:property name="camera_rpy" value="{rpy}"/>\n'
    )
    POSE_PATH.write_text(text)
    xacro_path = writable_xacro(assembly)
    if xacro_path.exists():
        src = xacro_path.read_text()
        src = re.sub(
            r'(<xacro:property name="camera_xyz" value=")[^"]*("/>)',
            rf"\g<1>{xyz}\2",
            src,
            count=1,
        )
        src = re.sub(
            r'(<xacro:property name="camera_rpy" value=")[^"]*("/>)',
            rf"\g<1>{rpy}\2",
            src,
            count=1,
        )
        xacro_path.write_text(src)
    _write_extrinsics_camera(assembly, xyz, rpy)
    return text


class AlignCamera(Node):
    def __init__(self):
        super().__init__("align_camera")
        self.declare_parameter("assembly", "gemini2")
        self.assembly = resolve_assembly(
            self.get_parameter("assembly").get_parameter_value().string_value
        )
        self._cfg = assembly_cfg(self.assembly)
        self._initial_pose = load_current_camera_pose(self.assembly)
        self.server = InteractiveMarkerServer(self, "/align_camera")
        self._insert_marker()
        self.server.applyChanges()
        xyz, rpy = format_params(self._initial_pose)
        label = self._cfg["camera_label"]
        self.get_logger().info(
            f"assembly={self.assembly} xacro={writable_xacro(self.assembly)}\n"
            f"Starts from current assembly (camera relative to camera_mount_link):\n"
            f"  camera_xyz=\"{xyz}\"\n"
            f"  camera_rpy=\"{rpy}\"\n"
            f"Click Interact, drag the {label} / cyan box to fine-tune. "
            "Release mouse to overwrite camera_xyz / camera_rpy."
        )

    def _camera_mesh(self):
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
        from pathlib import Path as _Path

        mesh = Marker()
        mesh.type = Marker.MESH_RESOURCE
        mesh.mesh_use_embedded_materials = False
        # Prefer file:// so RViz still finds the mesh if package:// lookup fails.
        resource = self._cfg["camera_mesh"]
        pkg = self._cfg.get("camera_mesh_package")
        rel = self._cfg.get("camera_mesh_relpath")
        if pkg and rel:
            try:
                abs_mesh = _Path(get_package_share_directory(pkg)) / rel
                if abs_mesh.is_file():
                    resource = abs_mesh.resolve().as_uri()
            except PackageNotFoundError:
                pass
        mesh.mesh_resource = resource
        ox, oy, oz = self._cfg["camera_mesh_offset"]
        mesh.pose.position.x = ox
        mesh.pose.position.y = oy
        mesh.pose.position.z = oz
        roll, pitch, yaw = self._cfg.get("camera_mesh_rpy", (0.0, 0.0, 0.0))
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        mesh.pose.orientation.x = qx
        mesh.pose.orientation.y = qy
        mesh.pose.orientation.z = qz
        mesh.pose.orientation.w = qw
        s = float(self._cfg["camera_mesh_scale"])
        mesh.scale.x = mesh.scale.y = mesh.scale.z = s
        mesh.color.r, mesh.color.g, mesh.color.b, mesh.color.a = 0.75, 0.75, 0.8, 1.0
        self.get_logger().info(f"align camera mesh resource: {resource}")
        return mesh

    def _grab_box(self):
        box = Marker()
        box.type = Marker.CUBE
        ox, oy, oz = self._cfg["camera_mesh_offset"]
        box.pose.position.x = ox
        box.pose.position.y = oy
        box.pose.position.z = oz
        # Large always-visible handle so the scene is never "empty" if STL fails.
        box.scale.x = 0.05
        box.scale.y = 0.05
        box.scale.z = 0.03
        box.color.r, box.color.g, box.color.b, box.color.a = 0.2, 0.8, 1.0, 0.55
        return box

    def _insert_marker(self):
        label = self._cfg["camera_label"]
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "camera_mount_link"
        int_marker.name = self.assembly
        int_marker.description = f"DRAG {label} (from current assembly)"
        int_marker.scale = 0.12
        int_marker.pose = self._initial_pose

        look = InteractiveMarkerControl()
        look.always_visible = True
        look.interaction_mode = InteractiveMarkerControl.NONE
        look.markers.append(self._camera_mesh())
        int_marker.controls.append(look)

        grab = InteractiveMarkerControl()
        grab.always_visible = True
        grab.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        grab.markers.append(self._grab_box())
        int_marker.controls.append(grab)

        for axis, vec in (("x", (1.0, 0.0, 0.0)), ("y", (0.0, 1.0, 0.0)), ("z", (0.0, 0.0, 1.0))):
            move = InteractiveMarkerControl()
            move.name = f"move_{axis}"
            move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            move.orientation.w = 1.0
            move.orientation.x, move.orientation.y, move.orientation.z = vec
            int_marker.controls.append(move)

            rot = InteractiveMarkerControl()
            rot.name = f"rotate_{axis}"
            rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            rot.orientation.w = 1.0
            rot.orientation.x, rot.orientation.y, rot.orientation.z = vec
            int_marker.controls.append(rot)

        self.server.insert(int_marker, feedback_callback=self._on_feedback)

    def _on_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        xyz, rpy = format_params(feedback.pose)
        text = write_params(self.assembly, xyz, rpy)
        self.get_logger().info(
            "Updated camera pose relative to camera_mount_link and wrote xacro:\n" + text
        )


def main():
    rclpy.init()
    node = AlignCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
