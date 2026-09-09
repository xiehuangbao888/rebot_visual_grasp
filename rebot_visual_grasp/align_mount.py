import math
import re
from pathlib import Path

import rclpy
from geometry_msgs.msg import Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from rebot_visual_grasp.paths import mount_mesh_uri, writable_xacro

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


def write_params(xyz, rpy):
    text = (
        f'<xacro:property name="mount_xyz" value="{xyz}"/>\n'
        f'<xacro:property name="mount_rpy" value="{rpy}"/>\n'
    )
    POSE_PATH.write_text(text)
    xacro_path = writable_xacro()
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
    return text


class AlignMount(Node):
    def __init__(self):
        super().__init__("align_mount")
        self.declare_parameter("parent_frame", "gripper_end")
        self.declare_parameter("mesh", mount_mesh_uri())
        self.parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        self.mesh = self.get_parameter("mesh").get_parameter_value().string_value

        self.server = InteractiveMarkerServer(self, "/align_mount")
        self._insert_marker()
        self.server.applyChanges()
        self.get_logger().info(
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
        text = write_params(xyz, rpy)
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
