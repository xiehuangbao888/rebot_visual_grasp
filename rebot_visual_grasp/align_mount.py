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

from rebot_visual_grasp.paths import mount_mesh_uri, resolve_assembly, writable_extrinsics, writable_xacro

POSE_PATH = Path("/tmp/mount_pose.txt")


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


def _write_extrinsics_mount(assembly: str, xyz: str, rpy: str) -> None:
    path = writable_extrinsics(assembly)
    if not path.is_file():
        return
    xs = [float(v) for v in xyz.split()]
    rs = [float(v) for v in rpy.split()]
    text = path.read_text()
    text = re.sub(
        r"(mount:\n(?:.*\n)*?\s+xyz:\s*)\[[^\]]*\]",
        rf"\g<1>[{xs[0]:.4f}, {xs[1]:.4f}, {xs[2]:.4f}]",
        text,
        count=1,
    )
    text = re.sub(
        r"(mount:\n(?:.*\n)*?\s+rpy:\s*)\[[^\]]*\]",
        rf"\g<1>[{rs[0]:.4f}, {rs[1]:.4f}, {rs[2]:.4f}]",
        text,
        count=1,
    )
    path.write_text(text)


def load_current_mount_pose(assembly: str) -> Pose:
    xacro_path = writable_xacro(assembly)
    pose = Pose()
    pose.orientation.w = 1.0
    if not xacro_path.is_file():
        return pose
    text = xacro_path.read_text()
    xyz_m = re.search(r'<xacro:property name="mount_xyz" value="([^"]*)"/>', text)
    rpy_m = re.search(r'<xacro:property name="mount_rpy" value="([^"]*)"/>', text)
    if not xyz_m or not rpy_m:
        return pose
    try:
        x, y, z = [float(v) for v in xyz_m.group(1).split()]
        roll, pitch, yaw = [float(v) for v in rpy_m.group(1).split()]
    except ValueError:
        return pose
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    return pose


def write_params(assembly: str, xyz, rpy):
    text = (
        f'<xacro:property name="mount_xyz" value="{xyz}"/>\n'
        f'<xacro:property name="mount_rpy" value="{rpy}"/>\n'
    )
    POSE_PATH.write_text(text)
    xacro_path = writable_xacro(assembly)
    if xacro_path.exists():
        src = xacro_path.read_text()
        src = re.sub(
            r'(<xacro:property name="mount_xyz" value=")[^"]*("/>)',
            rf"\g<1>{xyz}\2",
            src,
            count=1,
        )
        src = re.sub(
            r'(<xacro:property name="mount_rpy" value=")[^"]*("/>)',
            rf"\g<1>{rpy}\2",
            src,
            count=1,
        )
        xacro_path.write_text(src)
    _write_extrinsics_mount(assembly, xyz, rpy)
    return text


class AlignMount(Node):
    def __init__(self):
        super().__init__("align_mount")
        self.declare_parameter("assembly", "gemini2")
        self.declare_parameter("parent_frame", "gripper_end")
        self.declare_parameter("mesh", "")
        self.assembly = resolve_assembly(
            self.get_parameter("assembly").get_parameter_value().string_value
        )
        self.parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        mesh_param = self.get_parameter("mesh").get_parameter_value().string_value
        self.mesh = mesh_param or mount_mesh_uri(self.assembly)
        self._initial_pose = load_current_mount_pose(self.assembly)

        self.server = InteractiveMarkerServer(self, "/align_mount")
        self._insert_marker()
        self.server.applyChanges()
        xyz, rpy = format_params(self._initial_pose)
        self.get_logger().info(
            f"assembly={self.assembly} mesh={self.mesh} xacro={writable_xacro(self.assembly)}\n"
            f"Starts from current mount pose:\n  mount_xyz=\"{xyz}\"\n  mount_rpy=\"{rpy}\"\n"
            "Click Interact, then drag the RED BOX or the colored arrows/rings. "
            "Do not try to drag the arm itself. Release mouse to export pose."
        )

    def _mesh_visual(self):
        mesh = Marker()
        mesh.type = Marker.MESH_RESOURCE
        mesh.mesh_resource = self.mesh
        mesh.mesh_use_embedded_materials = False
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 0.001
        mesh.color.r, mesh.color.g, mesh.color.b, mesh.color.a = 0.85, 0.15, 0.15, 1.0
        return mesh

    def _grab_box(self):
        box = Marker()
        box.type = Marker.CUBE
        box.pose.position.x = 0.0
        box.pose.position.y = 0.049
        box.pose.position.z = -0.009
        box.scale.x = 0.10
        box.scale.y = 0.06
        box.scale.z = 0.04
        box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.1, 0.1, 0.45
        return box

    def _insert_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.parent_frame
        int_marker.name = "camera_mount"
        int_marker.description = "DRAG THIS (red box / arrows)"
        int_marker.scale = 0.18
        int_marker.pose = self._initial_pose

        look = InteractiveMarkerControl()
        look.always_visible = True
        look.interaction_mode = InteractiveMarkerControl.NONE
        look.markers.append(self._mesh_visual())
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
            "Exported pose relative to gripper_end and wrote xacro:\n" + text
        )


def main():
    rclpy.init()
    node = AlignMount()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
