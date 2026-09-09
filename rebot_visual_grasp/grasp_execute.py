#!/usr/bin/env python3
"""Execute open-loop grasp-and-place from locked poses.

Before motion, locks the latest:
  /pregrasp_pose, /grasp_pose, /place_pose

Sequence:
  1) open gripper
  2) move_to_pose pregrasp
  3) move_to_pose grasp
  4) close gripper
  5) move_to_pose observation (carry object)
  6) move_to_pose place (above box)
  7) open gripper (release into box)
  8) move_to_pose observation
  9) close gripper
"""

from __future__ import annotations

import copy
import threading

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.srv import GripperCommand, MoveToPoseIK
from std_srvs.srv import Trigger


def _make_pose(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def _fmt_xyz(pose: Pose) -> str:
    p = pose.position
    return f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"


class GraspExecuteNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_execute")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("pregrasp_duration", 4.0)
        self.declare_parameter("grasp_duration", 3.0)
        self.declare_parameter("observation_duration", 4.0)
        self.declare_parameter("place_duration", 4.0)
        self.declare_parameter("check_ik", False)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("gripper_timeout", 5.0)
        self.declare_parameter("open_gripper_before_motion", True)
        self.declare_parameter("require_place_pose", True)
        self.declare_parameter("open_gripper_at_place", True)
        self.declare_parameter("return_observation_after_place", True)
        self.declare_parameter("close_gripper_after_place", True)
        self.declare_parameter("place_use_observation_orientation", True)
        # Same defaults as grasp_yolo observation pose
        self.declare_parameter("observation_x", 0.205)
        self.declare_parameter("observation_y", 0.007)
        self.declare_parameter("observation_z", 0.312)
        self.declare_parameter("observation_qx", -0.025)
        self.declare_parameter("observation_qy", 0.434)
        self.declare_parameter("observation_qz", 0.031)
        self.declare_parameter("observation_qw", 0.900)

        ns = str(self.get_parameter("arm_namespace").value).strip("/")
        self._pregrasp_duration = float(self.get_parameter("pregrasp_duration").value)
        self._grasp_duration = float(self.get_parameter("grasp_duration").value)
        self._observation_duration = float(self.get_parameter("observation_duration").value)
        self._place_duration = float(self.get_parameter("place_duration").value)
        self._check_ik = bool(self.get_parameter("check_ik").value)
        self._auto_enable = bool(self.get_parameter("auto_enable").value)
        self._gripper_timeout = float(self.get_parameter("gripper_timeout").value)
        self._open_gripper_before_motion = bool(
            self.get_parameter("open_gripper_before_motion").value
        )
        self._require_place_pose = bool(self.get_parameter("require_place_pose").value)
        self._open_gripper_at_place = bool(self.get_parameter("open_gripper_at_place").value)
        self._return_observation_after_place = bool(
            self.get_parameter("return_observation_after_place").value
        )
        self._close_gripper_after_place = bool(
            self.get_parameter("close_gripper_after_place").value
        )
        self._place_use_observation_orientation = bool(
            self.get_parameter("place_use_observation_orientation").value
        )
        self._observation_pose = _make_pose(
            float(self.get_parameter("observation_x").value),
            float(self.get_parameter("observation_y").value),
            float(self.get_parameter("observation_z").value),
            float(self.get_parameter("observation_qx").value),
            float(self.get_parameter("observation_qy").value),
            float(self.get_parameter("observation_qz").value),
            float(self.get_parameter("observation_qw").value),
        )

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._latest_grasp: PoseStamped | None = None
        self._latest_pregrasp: PoseStamped | None = None
        self._latest_place: PoseStamped | None = None
        self._last_move_error = ""

        self._enable_cli = self.create_client(Trigger, f"/{ns}/enable", callback_group=self._cb_group)
        self._open_cli = self.create_client(
            GripperCommand, f"/{ns}/gripper/open", callback_group=self._cb_group
        )
        self._close_cli = self.create_client(
            GripperCommand, f"/{ns}/gripper/close", callback_group=self._cb_group
        )
        self._ik_cli = self.create_client(
            MoveToPoseIK, f"/{ns}/move_to_pose_ik", callback_group=self._cb_group
        )
        self._move_cli = ActionClient(
            self, MoveToPose, f"/{ns}/move_to_pose", callback_group=self._cb_group
        )

        self.create_subscription(
            PoseStamped,
            "/grasp_pose",
            self._grasp_callback,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PoseStamped,
            "/pregrasp_pose",
            self._pregrasp_callback,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PoseStamped,
            "/place_pose",
            self._place_callback,
            10,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger,
            "/grasp_execute",
            self._execute_service,
            callback_group=self._cb_group,
        )

        obs = self._observation_pose.position
        self.get_logger().info(
            f"Grasp-place execute ready on /grasp_execute (arm={ns}). "
            "Locks grasp+place first, then: "
            "open → pregrasp → grasp → close → observation → place → "
            "open → observation → close. "
            f"require_place={self._require_place_pose}, "
            f"observation=({obs.x:.3f}, {obs.y:.3f}, {obs.z:.3f})"
        )

    def _grasp_callback(self, msg: PoseStamped) -> None:
        self._latest_grasp = msg

    def _pregrasp_callback(self, msg: PoseStamped) -> None:
        self._latest_pregrasp = msg

    def _place_callback(self, msg: PoseStamped) -> None:
        self._latest_place = msg

    def _execute_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._lock.acquire(blocking=False):
            response.success = False
            response.message = "grasp already running"
            return response

        missing = []
        if self._latest_grasp is None:
            missing.append("/grasp_pose")
        if self._latest_pregrasp is None:
            missing.append("/pregrasp_pose")
        if self._require_place_pose and self._latest_place is None:
            missing.append("/place_pose")
        if missing:
            self._lock.release()
            response.success = False
            response.message = (
                "missing " + ", ".join(missing) + "; wait until YOLO published grasp and place"
            )
            return response

        try:
            ok, message = self._run_grasp()
            response.success = ok
            response.message = message
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        finally:
            self._lock.release()
        return response

    def _run_grasp(self) -> tuple[bool, str]:
        # Lock targets at trigger time so live YOLO updates cannot change mid-motion.
        pregrasp = copy.deepcopy(self._latest_pregrasp.pose)
        grasp = copy.deepcopy(self._latest_grasp.pose)
        observation = copy.deepcopy(self._observation_pose)
        place = None
        if self._latest_place is not None:
            place = copy.deepcopy(self._latest_place.pose)
            if self._place_use_observation_orientation:
                place.orientation = copy.deepcopy(observation.orientation)

        self.get_logger().info(
            "Confirmed targets before motion: "
            f"pregrasp={_fmt_xyz(pregrasp)} "
            f"grasp={_fmt_xyz(grasp)} "
            f"place={_fmt_xyz(place) if place is not None else 'n/a'} "
            f"pregrasp_quat=({pregrasp.orientation.x:.3f}, {pregrasp.orientation.y:.3f}, "
            f"{pregrasp.orientation.z:.3f}, {pregrasp.orientation.w:.3f})"
        )

        if self._auto_enable and not self._call_trigger(self._enable_cli, "enable"):
            return False, "enable failed"

        if self._check_ik:
            checks = [
                ("pregrasp", pregrasp),
                ("grasp", grasp),
                ("observation", observation),
            ]
            if place is not None:
                checks.append(("place", place))
            for label, pose in checks:
                if not self._check_pose_ik(pose, label):
                    return False, f"IK failed for {label}"

        # 1) open gripper
        if self._open_gripper_before_motion:
            if not self._call_gripper(self._open_cli, "open"):
                return False, "gripper open failed"
        else:
            self.get_logger().info("Skipping gripper open")

        # 2) pregrasp
        if not self._move_to_pose(pregrasp, self._pregrasp_duration, "pregrasp"):
            return False, self._last_move_error or "move_to_pose pregrasp failed"

        # 3) grasp
        if not self._move_to_pose(grasp, self._grasp_duration, "grasp"):
            return False, self._last_move_error or "move_to_pose grasp failed"

        # 4) close gripper
        if not self._call_gripper(self._close_cli, "close"):
            return False, "gripper close failed"

        # 5) return to observation with object
        if not self._move_to_pose(observation, self._observation_duration, "observation"):
            return False, self._last_move_error or "move_to_pose observation failed"

        # 6) move above box (place)
        if place is not None:
            if not self._move_to_pose(place, self._place_duration, "place"):
                return False, self._last_move_error or "move_to_pose place failed"
            # 7) release into box
            if self._open_gripper_at_place:
                if not self._call_gripper(self._open_cli, "open"):
                    return False, "gripper open at place failed"
            # 8) return to observation after place
            if self._return_observation_after_place:
                if not self._move_to_pose(
                    observation, self._observation_duration, "observation_after_place"
                ):
                    return False, (
                        self._last_move_error
                        or "move_to_pose observation after place failed"
                    )
            # 9) close gripper at observation
            if self._close_gripper_after_place:
                if not self._call_gripper(self._close_cli, "close"):
                    return False, "gripper close after place failed"

        return True, (
            "grasp-place complete: "
            "open→pregrasp→grasp→close→observation→place→open→observation→close"
        )

    def _check_pose_ik(self, pose: Pose, label: str) -> bool:
        if not self._ik_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("move_to_pose_ik unavailable; skipping IK check")
            return True
        request = MoveToPoseIK.Request()
        request.target_pose = pose
        future = self._ik_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            self.get_logger().error(f"IK check failed for {label}: {message}")
            return False
        return True

    def _move_to_pose(self, pose: Pose, duration: float, label: str) -> bool:
        self._last_move_error = ""
        if not self._move_cli.wait_for_server(timeout_sec=5.0):
            self._last_move_error = f"{label}: move_to_pose action not available"
            self.get_logger().error(self._last_move_error)
            return False

        goal = MoveToPose.Goal()
        goal.target_pose = pose
        goal.duration = max(float(duration), 0.2)

        send_future = self._move_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._last_move_error = (
                f"{label}: goal rejected (arm busy/disabled? "
                "check controller state TRAJ_RUNNING/GRAVITY_COMP)"
            )
            self.get_logger().error(self._last_move_error)
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=max(duration + 5.0, 10.0)
        )
        result = result_future.result()
        if result is None:
            self._last_move_error = f"{label}: no result"
            self.get_logger().error(self._last_move_error)
            return False
        action_result = result.result
        if not action_result.success:
            detail = action_result.message or "unknown"
            self._last_move_error = f"{label}: {detail}"
            self.get_logger().error(self._last_move_error)
            return False
        self.get_logger().info(f"{label}: {action_result.message}")
        return True

    def _call_trigger(self, client, label: str) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{label} service not available")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            self.get_logger().error(f"{label} failed: {message}")
            return False
        return True

    def _call_gripper(self, client, label: str) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"gripper {label} service not available")
            return False
        request = GripperCommand.Request()
        request.timeout = self._gripper_timeout
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._gripper_timeout + 5.0
        )
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            self.get_logger().error(f"gripper {label} failed: {message}")
            return False
        self.get_logger().info(
            f"gripper {label} ok (reached={result.reached_position:.3f})"
        )
        return True


def main() -> None:
    rclpy.init()
    node = GraspExecuteNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
