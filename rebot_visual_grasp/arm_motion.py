"""Helpers to call reBotArm controller enable and move_to_pose."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.srv import GripperCommand
from std_srvs.srv import Trigger


def call_enable(node: Node, namespace: str = "rebotarm", timeout_sec: float = 5.0) -> bool:
    client = node.create_client(Trigger, f"/{namespace.strip('/')}/enable")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error("enable service not available")
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    result = future.result()
    if result is None or not result.success:
        message = result.message if result is not None else "no response"
        node.get_logger().error(f"enable failed: {message}")
        return False
    node.get_logger().info(result.message or "enabled")
    return True


def open_gripper(
    node: Node,
    namespace: str = "rebotarm",
    *,
    timeout: float = 5.0,
) -> bool:
    """Open gripper only; does not move the arm."""
    ns = namespace.strip("/")
    client = node.create_client(GripperCommand, f"/{ns}/gripper/open")
    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error("gripper open service not available")
        return False
    request = GripperCommand.Request()
    request.timeout = float(timeout)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout + 2.0)
    result = future.result()
    if result is None or not result.success:
        message = result.message if result is not None and hasattr(result, "message") else "no response"
        # GripperCommand may not have message; use success only
        if result is None:
            node.get_logger().error("gripper open failed: no response")
        else:
            node.get_logger().error(
                f"gripper open failed (reached={getattr(result, 'reached_position', '?')})"
            )
        return False
    node.get_logger().info(
        f"gripper opened (reached={getattr(result, 'reached_position', 0.0):.3f})"
    )
    return True


def move_to_pose(
    node: Node,
    pose: Pose,
    duration: float,
    *,
    namespace: str = "rebotarm",
    label: str = "move_to_pose",
) -> bool:
    ns = namespace.strip("/")
    client = ActionClient(node, MoveToPose, f"/{ns}/move_to_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f"{label}: move_to_pose action not available")
        return False

    goal = MoveToPose.Goal()
    goal.target_pose = pose
    goal.duration = max(float(duration), 0.2)

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error(f"{label}: goal rejected")
        return False

    wait_sec = max(float(duration) + 5.0, 10.0)
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=wait_sec)
    result = result_future.result()
    if result is None:
        node.get_logger().error(f"{label}: no result")
        return False
    action_result = result.result
    if not action_result.success:
        node.get_logger().error(f"{label}: {action_result.message}")
        return False
    node.get_logger().info(f"{label}: {action_result.message}")
    return True


def make_pose(
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
