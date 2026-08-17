#!/usr/bin/env python3
"""CLI 테스트 클라이언트 — 목표 (x, y) 로 직선 경로 추종 액션 호출.

사용:
    ros2 run dss_motion_control send_goal 50 0
    ros2 run dss_motion_control send_goal 50 0 stanley 3.0
    (인자: x y [stanley|pure_pursuit] [속도 m/s])
"""

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

from dss_motion_interfaces.action import MotionPurePursuit, MotionStanley

from dss_motion_control import geometry as geo


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if len(argv) < 2:
        print(__doc__)
        return 1
    gx, gy = float(argv[0]), float(argv[1])
    controller = argv[2] if len(argv) > 2 else 'pure_pursuit'
    speed = float(argv[3]) if len(argv) > 3 else 3.0

    rclpy.init()
    node = Node('send_goal')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    # 현재 위치 (TF) 취득
    cur = None
    deadline = node.get_clock().now().nanoseconds + int(5e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = tf_buffer.lookup_transform('map', 'base_link', Time())
            cur = (tf.transform.translation.x, tf.transform.translation.y)
            break
        except Exception:
            continue
    if cur is None:
        print('현재 포즈(TF map→base_link) 없음 — 측위 스택을 먼저 기동하세요')
        return 2

    pts = geo.straight_path(cur[0], cur[1], gx, gy, 2.0)
    path = Path()
    path.header.frame_id = 'map'
    for x, y in pts:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)

    if controller == 'stanley':
        action_type, action_name = MotionStanley, 'motion_stanley'
    else:
        action_type, action_name = MotionPurePursuit, 'motion_pure_pursuit'

    goal = action_type.Goal()
    goal.path = path
    goal.max_linear_speed = speed
    goal.acceleration = 1.0
    goal.goal_tolerance = 0.0

    dist = math.hypot(gx - cur[0], gy - cur[1])
    print(f'{controller}: ({cur[0]:.1f},{cur[1]:.1f}) → ({gx:.1f},{gy:.1f}) '
          f'{dist:.1f} m, v={speed} m/s, {len(pts)} wpts')

    client = ActionClient(node, action_type, action_name)
    if not client.wait_for_server(timeout_sec=3.0):
        print('액션 서버 미기동 — dss_motion_control 노드를 먼저 실행하세요')
        return 3

    def on_feedback(msg):
        fb = msg.feedback
        print(f'  잔여 {fb.remaining_distance:6.1f} m | CTE {fb.cross_track_error:+.2f} m'
              f' | v {fb.current_speed:4.1f} m/s | steer {fb.steer_command:+.2f}')

    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        print('goal 거부됨')
        return 4

    # 클라이언트가 죽어도 서버 액션이 계속 달리는 좀비 goal 방지 —
    # SIGINT/SIGTERM(timeout 포함) 수신 시 cancel 을 보내고 잠시 spin 후 종료.
    import signal as _signal

    def _cancel_and_exit(signum, frame):
        print(f'\n시그널 {signum} — goal 취소 전송')
        fut = gh.cancel_goal_async()
        end = time.time() + 2.0
        while rclpy.ok() and not fut.done() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        raise SystemExit(6)

    _signal.signal(_signal.SIGINT, _cancel_and_exit)
    _signal.signal(_signal.SIGTERM, _cancel_and_exit)

    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    print(f'결과: status={result.status} 주행 {result.actual_distance:.1f} m '
          f'최종 CTE {result.final_cross_track_error:+.2f} m '
          f'{result.elapsed_time:.1f} s')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result.status == 0 else 5


if __name__ == '__main__':
    sys.exit(main())
