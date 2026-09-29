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
    SHARED_MOUNT_ASSEMBLIES,
    mount_mesh_uri,
    resolve_assembly,
    share_xacro,
    writable_extrinsics,
    writable_xacro,
)

POSE_PATH = Path("/tmp/mount_pose.txt")

# Parent-frame (gripper_end) steps.
STEP_XYZ = 0.0001  # 0.1 mm
STEP_XYZ_FINE = 0.00005  # 0.05 mm
STEP_YAW = math.radians(1.0)
STEP_YAW_FINE = math.radians(0.2)

HELP = """
Keyboard (parent frame = gripper_end):
  A / D   : Y- / Y+     (左右)
  W / S   : Z+ / Z-     (上下)
  R / F   : X+ / X-     (前后，补全 xyz)
  Q / E   : yaw- / yaw+ (绕 Z 旋转；不是 xyz 平移)
  Shift+键: 细调 (0.05 mm / 0.2 deg)
  P       : 打印当前 mount_xyz / mount_rpy
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


def _patch_xacro_mount(xacro_path, xyz: str, rpy: str) -> None:
    if not xacro_path.exists():
        return
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


def write_params(assembly: str, xyz, rpy):
    """Write mount pose.

    gemini2 and d435i share the same physical mount → sync those only.
    d405 has its own bracket → never overwrite it from gemini2/d435i.
    """
    text = (
        f'<xacro:property name="mount_xyz" value="{xyz}"/>\n'
        f'<xacro:property name="mount_rpy" value="{rpy}"/>\n'
    )
    POSE_PATH.write_text(text)
    targets = (
        list(SHARED_MOUNT_ASSEMBLIES)
        if assembly in SHARED_MOUNT_ASSEMBLIES
        else [assembly]
    )
    for name in targets:
        src_path = writable_xacro(name)
        _patch_xacro_mount(src_path, xyz, rpy)
        try:
            share_path = share_xacro(name)
            if share_path.resolve() != src_path.resolve():
                _patch_xacro_mount(share_path, xyz, rpy)
        except Exception:
            pass
        _write_extrinsics_mount(name, xyz, rpy)
    return text


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
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        qx,
        qy,
        qz,
        qw,
    )
    return pose


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
        self._pose = load_current_mount_pose(self.assembly)
        self._pose_lock = threading.Lock()
        self._key_queue: list[str] = []
        self._key_lock = threading.Lock()
        self._kb_stop = threading.Event()
        self._tty_fd = None
        self._tty_old = None
        self._pynput_listener = None
        self._shift_down = False

        self.server = InteractiveMarkerServer(self, "/align_mount")
        self._insert_marker()
        self.server.applyChanges()
        xyz, rpy = format_params(self._pose)
        self.get_logger().info(
            f"assembly={self.assembly} mesh={self.mesh} xacro={writable_xacro(self.assembly)}\n"
            f"Starts from current mount pose:\n  mount_xyz=\"{xyz}\"\n  mount_rpy=\"{rpy}\"\n"
            "Click Interact to drag, OR use keyboard (see help below)."
            + HELP
        )
        self.create_timer(0.05, self._flush_keys)
        self._start_keyboard()

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
        int_marker.description = "KEYS: WASD/RF/QE (or drag)"
        int_marker.scale = 0.18
        int_marker.pose = _copy_pose(self._pose)

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
            "Exported pose relative to gripper_end and wrote xacro:\n" + text
        )

    def _apply_pose(self, pose: Pose, save: bool = True):
        with self._pose_lock:
            self._pose = _copy_pose(pose)
            cur = _copy_pose(self._pose)
        self.server.setPose("camera_mount", cur)
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
            self.get_logger().info(f"current mount_xyz=\"{xyz}\" mount_rpy=\"{rpy}\"")
        elif key == "h":
            self.get_logger().info(HELP)
        # ignore other keys (including launch/rviz noise)

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
            # Ignore while typing into other text fields if char is None.
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
        thread = threading.Thread(target=self._keyboard_loop, name="align_mount_kb", daemon=True)
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
    node = AlignMount()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
