#!/usr/bin/env python3
"""Receive spr_decision goals and emulate only their results, without Nav2."""

from __future__ import annotations

import copy
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_interfaces.msg import GameState
from std_msgs.msg import UInt8
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


def normalized_pose(source: PoseStamped, frame_id: str, stamp) -> PoseStamped:
    pose = copy.deepcopy(source)
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    q = pose.pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-9:
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
    else:
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
    return pose


class DecisionResultMock(Node):
    """Mock NavigateToPose and visualize the behavior tree's requested result."""

    def __init__(self) -> None:
        super().__init__("decision_result_mock")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("result_mode", "manual_pose")
        self.declare_parameter("result_delay_sec", 1.0)
        self.declare_parameter("xy_tolerance", 0.4)
        self.declare_parameter("check_goal_yaw", False)
        self.declare_parameter("yaw_tolerance_deg", 15.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "chassis")
        self.declare_parameter("robot_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("publish_robot_tf", True)
        self.declare_parameter("initial_pose_topic", "initialpose")
        self.declare_parameter("goal_topic", "decision_goal")
        self.declare_parameter("marker_topic", "decision_markers")
        self.declare_parameter("max_goal_history", 30)

        self._mode = str(self.get_parameter("result_mode").value).strip().lower()
        if self._mode not in {"manual_pose", "hold", "succeed", "abort"}:
            raise ValueError(
                "result_mode must be manual_pose, hold, succeed, or abort"
            )
        self._delay = max(0.0, float(self.get_parameter("result_delay_sec").value))
        self._xy_tolerance = max(0.0, float(self.get_parameter("xy_tolerance").value))
        self._check_goal_yaw = bool(self.get_parameter("check_goal_yaw").value)
        self._yaw_tolerance = math.radians(
            max(0.0, float(self.get_parameter("yaw_tolerance_deg").value))
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._robot_pose = [float(v) for v in self.get_parameter("robot_pose").value]
        if len(self._robot_pose) != 3:
            raise ValueError("robot_pose must be [x, y, yaw]")
        self._publish_robot_tf_enabled = bool(
            self.get_parameter("publish_robot_tf").value
        )
        self._history_limit = max(1, int(self.get_parameter("max_goal_history").value))

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), latched
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), latched
        )

        self._lock = threading.Lock()
        self._sequence = 0
        self._latest_sequence = 0
        self._latest_goal: PoseStamped | None = None
        self._history: list[PoseStamped] = []
        self._nav_state = "等待行为树目标"
        self._spin_value: int | None = None
        self._posture_value: int | None = None
        self._game_state: GameState | None = None

        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(UInt8, "spin_or_not", self._spin_callback, output_qos)
        self.create_subscription(UInt8, "posture", self._posture_callback, output_qos)
        self.create_subscription(GameState, "game_state", self._game_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            self._initial_pose_callback,
            10,
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.5, self._publish_visualization)

        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            "决策结果接收器已启动: mode=%s, action=%s"
            % (self._mode, self.get_parameter("action_name").value)
        )

    def _goal_callback(self, goal_request):
        del goal_request
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _spin_callback(self, msg: UInt8) -> None:
        with self._lock:
            self._spin_value = int(msg.data)

    def _posture_callback(self, msg: UInt8) -> None:
        with self._lock:
            self._posture_value = int(msg.data)

    def _game_callback(self, msg: GameState) -> None:
        with self._lock:
            self._game_state = msg

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            self._robot_pose = [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw,
            ]
        self.get_logger().info(
            "手动机器人位姿: x=%.3f, y=%.3f, yaw=%.1f deg"
            % (self._robot_pose[0], self._robot_pose[1], math.degrees(yaw))
        )
        self._publish_visualization()

    def _set_goal(self, source: PoseStamped) -> int:
        goal = normalized_pose(source, self._map_frame, self.get_clock().now().to_msg())
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
            self._latest_sequence = sequence
            self._latest_goal = goal
            if not self._history or (
                abs(self._history[-1].pose.position.x - goal.pose.position.x) > 1.0e-6
                or abs(self._history[-1].pose.position.y - goal.pose.position.y) > 1.0e-6
            ):
                self._history.append(copy.deepcopy(goal))
                self._history = self._history[-self._history_limit :]
            if self._mode == "manual_pose":
                self._nav_state = "等待 2D Pose Estimate 到达目标"
            elif self._mode == "hold":
                self._nav_state = "保持执行"
            else:
                self._nav_state = "等待模拟结果"
        self._goal_pub.publish(goal)
        self._publish_visualization()
        self.get_logger().info(
            "行为树目标 #%d: x=%.3f, y=%.3f, mode=%s"
            % (sequence, goal.pose.position.x, goal.pose.position.y, self._mode)
        )
        return sequence

    def _manual_goal_reached(self, goal: PoseStamped) -> bool:
        with self._lock:
            robot_pose = list(self._robot_pose)
        distance = math.hypot(
            goal.pose.position.x - robot_pose[0], goal.pose.position.y - robot_pose[1]
        )
        if distance > self._xy_tolerance:
            return False
        if not self._check_goal_yaw:
            return True
        goal_yaw = yaw_from_quaternion(goal.pose.orientation)
        return abs(angle_error(goal_yaw, robot_pose[2])) <= self._yaw_tolerance

    def _execute_callback(self, goal_handle):
        sequence = self._set_goal(goal_handle.request.pose)
        start = time.monotonic()
        result = NavigateToPose.Result()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self._lock:
                    if sequence == self._latest_sequence:
                        self._nav_state = "已取消"
                self._publish_visualization()
                return result

            with self._lock:
                latest = sequence == self._latest_sequence
                goal = copy.deepcopy(self._latest_goal)
            if not latest:
                if goal_handle.is_active:
                    goal_handle.abort()
                return result

            elapsed = time.monotonic() - start
            reached = (
                self._mode == "manual_pose"
                and goal is not None
                and self._manual_goal_reached(goal)
            )
            if reached or (self._mode != "hold" and self._mode != "manual_pose" and elapsed >= self._delay):
                if reached or self._mode == "succeed":
                    goal_handle.succeed()
                    state = "手动到达，返回成功" if reached else "模拟成功"
                else:
                    goal_handle.abort()
                    state = "模拟失败"
                with self._lock:
                    if sequence == self._latest_sequence:
                        self._nav_state = state
                self.get_logger().info("目标 #%d %s" % (sequence, state))
                self._publish_visualization()
                return result

            feedback = NavigateToPose.Feedback()
            feedback.current_pose = self._robot_pose_stamped()
            if goal is not None:
                feedback.distance_remaining = math.hypot(
                    goal.pose.position.x - feedback.current_pose.pose.position.x,
                    goal.pose.position.y - feedback.current_pose.pose.position.y,
                )
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        if goal_handle.is_active:
            goal_handle.abort()
        return result

    def _robot_pose_stamped(self) -> PoseStamped:
        with self._lock:
            robot_pose = list(self._robot_pose)
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = robot_pose[0]
        pose.pose.position.y = robot_pose[1]
        qx, qy, qz, qw = quaternion_from_yaw(robot_pose[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _publish_tf(self, stamp) -> None:
        if not self._publish_robot_tf_enabled:
            return
        with self._lock:
            robot_pose = list(self._robot_pose)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._robot_frame
        transform.transform.translation.x = robot_pose[0]
        transform.transform.translation.y = robot_pose[1]
        qx, qy, qz, qw = quaternion_from_yaw(robot_pose[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    def _status_text(self) -> str:
        posture_names = {1: "进攻", 2: "防御", 3: "移动"}
        with self._lock:
            nav_state = self._nav_state
            spin_value = self._spin_value
            posture_value = self._posture_value
            game = copy.deepcopy(self._game_state)
        posture = posture_names.get(posture_value, "未发布")
        spin = "开" if spin_value == 1 else ("关" if spin_value == 0 else "未发布")
        lines = ["决策: %s | 姿态: %s | 小陀螺: %s" % (nav_state, posture, spin)]
        if game is not None:
            lines.append(
                "比赛阶段=%d  剩余=%ds  HP=%d  弹量=%d"
                % (
                    game.game_progress,
                    game.stage_remain_time,
                    game.current_hp,
                    game.projectile_allowance_17mm,
                )
            )
        return "\n".join(lines)

    def _publish_visualization(self) -> None:
        now = self.get_clock().now().to_msg()
        self._publish_tf(now)
        with self._lock:
            goal = copy.deepcopy(self._latest_goal)
            history = copy.deepcopy(self._history)

        markers = MarkerArray()
        robot_pose = self._robot_pose_stamped()

        robot = Marker()
        robot.header.frame_id = self._map_frame
        robot.header.stamp = now
        robot.ns = "decision_sim"
        robot.id = 1
        robot.type = Marker.CYLINDER
        robot.action = Marker.ADD
        robot.pose = robot_pose.pose
        robot.pose.position.z = 0.2
        robot.scale.x = 0.65
        robot.scale.y = 0.65
        robot.scale.z = 0.4
        robot.color.r = 0.1
        robot.color.g = 0.35
        robot.color.b = 1.0
        robot.color.a = 0.8
        markers.markers.append(robot)

        trail = Marker()
        trail.header.frame_id = self._map_frame
        trail.header.stamp = now
        trail.ns = "decision_sim"
        trail.id = 2
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.scale.x = 0.08
        trail.color.r = 1.0
        trail.color.g = 0.55
        trail.color.b = 0.0
        trail.color.a = 0.8
        trail.points = [item.pose.position for item in history]
        markers.markers.append(trail)

        if goal is not None:
            arrow = Marker()
            arrow.header.frame_id = self._map_frame
            arrow.header.stamp = now
            arrow.ns = "decision_sim"
            arrow.id = 3
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = goal.pose
            arrow.pose.position.z = 0.15
            arrow.scale.x = 1.0
            arrow.scale.y = 0.2
            arrow.scale.z = 0.2
            arrow.color.r = 1.0
            arrow.color.g = 0.1
            arrow.color.b = 0.1
            arrow.color.a = 1.0
            markers.markers.append(arrow)

            goal_text = Marker()
            goal_text.header.frame_id = self._map_frame
            goal_text.header.stamp = now
            goal_text.ns = "decision_sim"
            goal_text.id = 4
            goal_text.type = Marker.TEXT_VIEW_FACING
            goal_text.action = Marker.ADD
            goal_text.pose = goal.pose
            goal_text.pose.position.z = 0.8
            goal_text.scale.z = 0.42
            goal_text.color.r = 1.0
            goal_text.color.g = 0.15
            goal_text.color.b = 0.15
            goal_text.color.a = 1.0
            goal_text.text = "目标 (%.2f, %.2f)" % (
                goal.pose.position.x,
                goal.pose.position.y,
            )
            markers.markers.append(goal_text)

        status = Marker()
        status.header.frame_id = self._map_frame
        status.header.stamp = now
        status.ns = "decision_sim"
        status.id = 5
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = robot_pose.pose.position.x
        status.pose.position.y = robot_pose.pose.position.y
        status.pose.position.z = 1.2
        status.pose.orientation.w = 1.0
        status.scale.z = 0.38
        status.color.r = 0.1
        status.color.g = 1.0
        status.color.b = 0.2
        status.color.a = 1.0
        status.text = self._status_text()
        markers.markers.append(status)
        self._marker_pub.publish(markers)

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionResultMock()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
