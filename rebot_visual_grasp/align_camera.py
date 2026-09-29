from __future__ import annotations

import math
import os
import re
import select
import termios
import threading
import tty
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

# Parent-frame (camera_mount_link) steps — same as align_mount.
STEP_XYZ = 0.0001  # 0.1 mm
STEP_XYZ_FINE = 0.00005  # 0.05 mm
STEP_YAW = math.radians(1.0)
STEP_YAW_FINE = math.radians(0.2)

HELP = """
Keyboard (parent frame = camera_mount_link):
  A / D   : Y- / Y+     (左右)
  W / S   : Z+ / Z-     (上下)
  R / F   : X+ / X-     (前后，补全 xyz)
  Q / E   : yaw- / yaw+ (绕 Z 旋转；不是 xyz 平移)
  Shift+键: 细调 (0.05 mm / 0.2 deg)
  P       : 打印当前 camera_xyz / camera_rpy
  H       : 再打印本帮助
每次有效按键都会立刻写回 xacro。鼠标拖动仍可用。
"""

try:
    from pynput import keyboard as _pynput_keyboard

    _HAS_PYNPUT = True
except ImportError:
    _pynput_keyboard = None
    _HAS_PYNPUT = False


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


def _copy_pose(pose: Pose) -> Pose:
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z
    out.orientation.x = pose.orientation.x
    out.orientation.y = pose.orientation.y
    out.orientation.z = pose.orientation.z
    out.orientation.w = pose.orientation.w
    return out


class AlignCamera(Node):
    def __init__(self):
        super().__init__("align_camera")
        self.declare_parameter("assembly", "gemini2")
        self.assembly = resolve_assembly(
            self.get_parameter("assembly").get_parameter_value().string_value
        )
        self._cfg = assembly_cfg(self.assembly)
        self._marker_name = self.assembly
        self._pose = load_current_camera_pose(self.assembly)
        self._pose_lock = threading.Lock()
        self._key_queue: list[str] = []
        self._key_lock = threading.Lock()
        self._kb_stop = threading.Event()
        self._tty_fd = None
        self._tty_old = None
        self._pynput_listener = None
        self._shift_down = False

        self.server = InteractiveMarkerServer(self, "/align_camera")
        self._insert_marker()
        self.server.applyChanges()
        xyz, rpy = format_params(self._pose)
        label = self._cfg["camera_label"]
        self.get_logger().info(
            f"assembly={self.assembly} xacro={writable_xacro(self.assembly)}\n"
            f"Starts from current assembly (camera relative to camera_mount_link):\n"
            f"  camera_xyz=\"{xyz}\"\n"
            f"  camera_rpy=\"{rpy}\"\n"
            f"Click Interact to drag the {label} / cyan box, OR use keyboard."
            + HELP
        )
        self.create_timer(0.05, self._flush_keys)
        self._start_keyboard()

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
        int_marker.name = self._marker_name
        int_marker.description = f"KEYS WASD/RF/QE or drag {label}"
        int_marker.scale = 0.12
        int_marker.pose = _copy_pose(self._pose)

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
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            with self._pose_lock:
                self._pose = _copy_pose(feedback.pose)
            return
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        with self._pose_lock:
            self._pose = _copy_pose(feedback.pose)
            pose = _copy_pose(self._pose)
        xyz, rpy = format_params(pose)
        text = write_params(self.assembly, xyz, rpy)
        self.get_logger().info(
            "Updated camera pose relative to camera_mount_link and wrote xacro:\n" + text
        )

    def _apply_pose(self, pose: Pose, save: bool = True):
        with self._pose_lock:
            self._pose = _copy_pose(pose)
            cur = _copy_pose(self._pose)
        self.server.setPose(self._marker_name, cur)
        self.server.applyChanges()
        if save:
            xyz, rpy = format_params(cur)
            text = write_params(self.assembly, xyz, rpy)
            self.get_logger().info("Key nudge → wrote xacro:\n" + text)

    def _nudge(self, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        with self._pose_lock:
            pose = _copy_pose(self._pose)
        pose.position.x += dx
        pose.position.y += dy
        pose.position.z += dz
        if dyaw != 0.0:
            roll, pitch, yaw = quat_to_rpy(pose.orientation)
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw + dyaw)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
        self._apply_pose(pose, save=True)

    def _handle_key(self, ch: str):
        fine = ch.isupper() and ch.isalpha()
        key = ch.lower()
        step = STEP_XYZ_FINE if fine else STEP_XYZ
        yaw = STEP_YAW_FINE if fine else STEP_YAW

        if key == "a":
            self._nudge(dy=-step)
        elif key == "d":
            self._nudge(dy=+step)
        elif key == "w":
            self._nudge(dz=+step)
        elif key == "s":
            self._nudge(dz=-step)
        elif key == "r":
            self._nudge(dx=+step)
        elif key == "f":
            self._nudge(dx=-step)
        elif key == "q":
            self._nudge(dyaw=-yaw)
        elif key == "e":
            self._nudge(dyaw=+yaw)
        elif key == "p":
            with self._pose_lock:
                pose = _copy_pose(self._pose)
            xyz, rpy = format_params(pose)
            self.get_logger().info(f"current camera_xyz=\"{xyz}\" camera_rpy=\"{rpy}\"")
        elif key == "h":
            self.get_logger().info(HELP)

    def _flush_keys(self):
        with self._key_lock:
            keys = self._key_queue
            self._key_queue = []
        for ch in keys:
            self._handle_key(ch)

    def _enqueue(self, ch: str):
        with self._key_lock:
            self._key_queue.append(ch)

    def _start_keyboard(self):
        if _HAS_PYNPUT:
            self._start_pynput()
            return
        self._start_tty_keyboard()

    def _start_pynput(self):
        assert _pynput_keyboard is not None

        def on_press(key):
            try:
                if key in (
                    _pynput_keyboard.Key.shift,
                    _pynput_keyboard.Key.shift_l,
                    _pynput_keyboard.Key.shift_r,
                ):
                    self._shift_down = True
                    return
                ch = key.char
            except AttributeError:
                return
            if not ch or not ch.isalpha():
                return
            out = ch.upper() if self._shift_down else ch.lower()
            self._enqueue(out)

        def on_release(key):
            if key in (
                _pynput_keyboard.Key.shift,
                _pynput_keyboard.Key.shift_l,
                _pynput_keyboard.Key.shift_r,
            ):
                self._shift_down = False

        self._pynput_listener = _pynput_keyboard.Listener(
            on_press=on_press, on_release=on_release
        )
        self._pynput_listener.daemon = True
        self._pynput_listener.start()
        self.get_logger().info(
            "Keyboard ready via pynput (works while RViz is focused)."
        )

    def _start_tty_keyboard(self):
        try:
            fd = os.open("/dev/tty", os.O_RDONLY | os.O_NOCTTY)
        except OSError:
            self.get_logger().warn(
                "Cannot open /dev/tty for keyboard; use mouse drag only."
            )
            return
        try:
            old = termios.tcgetattr(fd)
        except termios.error:
            os.close(fd)
            self.get_logger().warn("tty is not a terminal; use mouse drag only.")
            return
        self._tty_fd = fd
        self._tty_old = old
        tty.setcbreak(fd)
        thread = threading.Thread(
            target=self._keyboard_loop, name="align_camera_kb", daemon=True
        )
        thread.start()
        self.get_logger().info(
            "Keyboard ready on this terminal (/dev/tty). Focus the launch terminal to type."
        )

    def _keyboard_loop(self):
        fd = self._tty_fd
        assert fd is not None
        while not self._kb_stop.is_set() and rclpy.ok():
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
            except (OSError, ValueError):
                break
            if not r:
                continue
            try:
                data = os.read(fd, 8)
            except OSError:
                break
            if not data:
                continue
            if data[0] == 0x1B:
                continue
            for b in data:
                if 32 <= b < 127:
                    self._enqueue(chr(b))

    def destroy_node(self):
        self._kb_stop.set()
        if self._pynput_listener is not None:
            try:
                self._pynput_listener.stop()
            except Exception:
                pass
            self._pynput_listener = None
        if self._tty_fd is not None and self._tty_old is not None:
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._tty_old)
            except termios.error:
                pass
            try:
                os.close(self._tty_fd)
            except OSError:
                pass
            self._tty_fd = None
        super().destroy_node()


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
