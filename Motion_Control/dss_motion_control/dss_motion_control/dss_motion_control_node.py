#!/usr/bin/env python3
"""DSS 경로 추종 제어 노드 — Stanley·Pure Pursuit 액션 서버 + RViz 목표점 브리지.

ADR: docs/adr/2026-08-17-dss-motion-control.md
파라미터(캘리브레이션 대상 — D8): wheelbase·max_steer_deg·steer_sign 은
저속 실측으로 확정한다.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

from dss_motion_interfaces.action import MotionPurePursuit, MotionStanley

from dss_motion_control import geometry as geo
from dss_motion_control.tracking_server import (
    DssCommander, PurePursuitServer, StanleyServer)


class GoalPoseBridge:
    """RViz "2D Goal Pose"(/goal_pose) → 직선 경로 → 기본 제어기 액션 (ADR D7)."""

    def __init__(self, node, tf_buffer):
        self.node = node
        self.tf_buffer = tf_buffer
        self.default_controller = node.get_parameter('default_controller').value
        self.cruise_speed = node.get_parameter('goal_cruise_speed').value
        self.accel = node.get_parameter('goal_acceleration').value
        self.spacing = node.get_parameter('goal_waypoint_spacing').value
        self.stanley_client = ActionClient(node, MotionStanley, 'motion_stanley')
        self.pp_client = ActionClient(node, MotionPurePursuit, 'motion_pure_pursuit')
        self.sub = node.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal, 10)

    def _pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def on_goal(self, msg):
        cur = self._pose()
        if cur is None:
            self.node.get_logger().error('goal_pose: 현재 포즈(TF) 없음 — 무시')
            return
        gx, gy = msg.pose.position.x, msg.pose.position.y
        pts = geo.straight_path(cur[0], cur[1], gx, gy, self.spacing)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        dist = math.hypot(gx - cur[0], gy - cur[1])
        self.node.get_logger().info(
            f'goal_pose 수신 → {self.default_controller} 직선 {dist:.1f} m '
            f'({len(pts)} wpts, v={self.cruise_speed})')

        if self.default_controller == 'pure_pursuit':
            goal = MotionPurePursuit.Goal()
            goal.lookahead_distance = 0.0   # 노드 기본값 사용
            client = self.pp_client
        else:
            goal = MotionStanley.Goal()
            client = self.stanley_client
        goal.path = path
        goal.max_linear_speed = self.cruise_speed
        goal.acceleration = self.accel
        goal.goal_tolerance = 0.0           # 노드 기본값 사용
        if not client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error('액션 서버 미기동')
            return
        client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = Node('dss_motion_control')

    # 파라미터 일괄 선언 (제어·차량·캘리브레이션·목표점 UX)
    defaults = {
        'control_rate_hz': 20.0,
        'wheelbase': 2.7,                # m — 자전거 모델 축거 가정
        # 실측 캘리브레이션(D8): steer +→우회전, -→좌회전 (대칭 확인).
        # 정상 곡률 2회 실측 κ→δ 역산: 유효 조향각 ≈ 9~10°/steer 1.0.
        'max_steer_deg': 10.0,
        'steer_sign': -1.0,              # δ>0(좌) 를 DSS 좌회전으로 보내려면 부호 반전
        'default_goal_tolerance': 1.5,   # m
        'lateral_abort_dist': 4.0,       # m
        'max_timeout_sec': 180.0,
        'min_speed': 0.8,                # m/s — 정지 마찰 극복
        'watchdog_timeout_sec': 2.0,
        'watchdog_jump_threshold': 2.0,  # m — 측위 점프 판정
        'stall_timeout_sec': 4.0,        # s — 명령>0·실속도≈0 지속 시 충돌 판정
        'throttle_kp': 0.08,
        'throttle_ki': 0.02,
        'throttle_deadband': 0.15,       # 실측: ~0.2 미만 무동작 — 피드포워드 보상
        'brake_kp': 0.3,
        'throttle_max': 0.35,            # 오늘 실측: 0.3 ≈ 5~6 m/s
        'stanley_k': 1.0,
        'stanley_k_soft': 1.0,
        'pp_lookahead': 8.0,             # m — 차량 속도 스케일
        'default_controller': 'pure_pursuit',
        'goal_cruise_speed': 3.0,        # m/s — goal_pose 브리지 기본
        'goal_acceleration': 1.0,        # m/s^2
        'goal_waypoint_spacing': 2.0,    # m
    }
    for name, val in defaults.items():
        node.declare_parameter(name, val)

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    commander = DssCommander(node)

    StanleyServer(node, 'motion_stanley', MotionStanley, commander, tf_buffer)
    PurePursuitServer(node, 'motion_pure_pursuit', MotionPurePursuit,
                      commander, tf_buffer)
    GoalPoseBridge(node, tf_buffer)

    node.get_logger().info(
        'dss_motion_control 준비 — 액션 motion_stanley / motion_pure_pursuit, '
        '/goal_pose 브리지 활성')

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
