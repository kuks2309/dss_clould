"""공통 경로 추종 액션 서버 + Stanley/Pure Pursuit 서브클래스 (ADR D9).

출력은 기존 확립 제어 표면 /dss/control (dss_ros2_bridge/DssControl) — DSSControlNode
가 UDP 로 릴레이하며 500 ms 무수신 dead-man 을 가진다. 유휴 시에는 발행하지 않는다
(jog 등 타 발행자와의 경합 회피 — 동시 기동 금지 규율은 jog ADR 계승).
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time

from dss_ros2_bridge.msg import DssControl

from dss_motion_control import geometry as geo
from dss_motion_control.motion_profile import TrapezoidalProfile, DONE
from dss_motion_control.watchdog import LocalizationWatchdog

# 서버 간 상호 배제 — 원본 g_active_action CAS 의 파이썬 판
_ACTIVE_LOCK = threading.Lock()
_ACTIVE_NAME = [None]


class SpeedEstimator:
    """TF 포즈 미분 + 재귀 이동평균으로 차속 추정."""

    def __init__(self, window=5):
        self.window = window
        self.buf = []
        self.prev = None    # (x, y, t)

    def update(self, x, y, t):
        if self.prev is not None:
            dt = t - self.prev[2]
            if dt > 1e-3:
                v = math.hypot(x - self.prev[0], y - self.prev[1]) / dt
                self.buf.append(v)
                if len(self.buf) > self.window:
                    self.buf.pop(0)
        self.prev = (x, y, t)
        return sum(self.buf) / len(self.buf) if self.buf else 0.0


class DssCommander:
    """속도 오차 PI → throttle/brake, 조향 정규화 → /dss/control 발행."""

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(DssControl, '/dss/control', 10)
        self.kp = node.get_parameter('throttle_kp').value
        self.ki = node.get_parameter('throttle_ki').value
        self.kb = node.get_parameter('brake_kp').value
        self.throttle_max = node.get_parameter('throttle_max').value
        # DSS 스로틀 실측 데드밴드(~0.2 미만 무동작) 보상 피드포워드
        self.throttle_ff = node.get_parameter('throttle_deadband').value
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def command(self, v_cmd, v_est, steer_norm, dt):
        err = v_cmd - v_est
        msg = DssControl()
        msg.steer = float(max(-1.0, min(1.0, steer_norm)))
        # 기어 실측: +1=전진(D)·0=유지·-1=후진. 0(유지)은 직전 후진 기어를
        # 물려받아 무동작을 만든다 — 전진을 명시한다.
        msg.target_gear = 1.0
        if err >= -0.3:
            self.integral = max(0.0, min(0.5, self.integral + err * dt * self.ki))
            base = self.throttle_ff if v_cmd > 0.1 else 0.0
            msg.throttle = float(max(0.0, min(self.throttle_max,
                                              base + self.kp * err + self.integral)))
            msg.brake = 0.0
        else:
            # 큰 감속 요구 — 스로틀 차단 + 비례 브레이크
            self.integral = 0.0
            msg.throttle = 0.0
            msg.brake = float(max(0.0, min(1.0, self.kb * (-err))))
        self.pub.publish(msg)
        return msg

    def stop_and_release(self, v_est_fn, hold_brake=0.6, timeout=5.0, rate_hz=20.0):
        """정지 시퀀스: 브레이크 유지 → 정지 확인 → 0 명령 수 회 후 릴리스."""
        period = 1.0 / rate_hz
        deadline = time.time() + timeout
        while time.time() < deadline and v_est_fn() > 0.2:
            msg = DssControl()
            msg.brake = hold_brake
            self.pub.publish(msg)
            time.sleep(period)
        for _ in range(5):
            self.pub.publish(DssControl())
            time.sleep(period)


class PathTrackingServer:
    """가드·프로파일·워치독·피드백 공통 루프. 서브클래스가 steer() 구현."""

    STATUS_OK, STATUS_CANCEL, STATUS_INVALID = 0, -1, -2
    STATUS_TIMEOUT, STATUS_SAFETY = -3, -4

    def __init__(self, node, action_name, action_type, commander, tf_buffer):
        self.node = node
        self.name = action_name
        self.action_type = action_type
        self.commander = commander
        self.tf_buffer = tf_buffer
        self.speed_est = SpeedEstimator()

        p = node.get_parameter
        self.rate_hz = p('control_rate_hz').value
        self.wheelbase = p('wheelbase').value
        self.max_steer = math.radians(p('max_steer_deg').value)
        self.steer_sign = p('steer_sign').value
        self.default_goal_tol = p('default_goal_tolerance').value
        self.lateral_abort = p('lateral_abort_dist').value
        self.max_timeout = p('max_timeout_sec').value
        self.min_speed = p('min_speed').value

        self.server = ActionServer(
            node, action_type, action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=lambda _:
                CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    # ---- 콜백 ----

    def goal_cb(self, goal):
        if goal.max_linear_speed <= 0.0 or goal.acceleration <= 0.0:
            return GoalResponse.REJECT
        if len(goal.path.poses) < 2:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def lookup_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (t.x, t.y, yaw)
        except Exception:
            return None

    def _result(self, status, dist=0.0, cte=0.0, elapsed=0.0):
        r = self.action_type.Result()
        r.status = status
        r.actual_distance = float(dist)
        r.final_cross_track_error = float(cte)
        r.elapsed_time = float(elapsed)
        return r

    def execute_cb(self, goal_handle):
        goal = goal_handle.request

        # 상호 배제
        with _ACTIVE_LOCK:
            if _ACTIVE_NAME[0] is not None:
                self.node.get_logger().warn(
                    f'{self.name}: {_ACTIVE_NAME[0]} 실행 중 — 거부')
                goal_handle.abort()
                return self._result(self.STATUS_INVALID)
            _ACTIVE_NAME[0] = self.name

        try:
            return self._run(goal_handle, goal)
        finally:
            with _ACTIVE_LOCK:
                _ACTIVE_NAME[0] = None

    def _run(self, goal_handle, goal):
        log = self.node.get_logger()
        path = geo.build_path(
            [(ps.pose.position.x, ps.pose.position.y) for ps in goal.path.poses])
        if path is None:
            goal_handle.abort()
            return self._result(self.STATUS_INVALID)

        goal_tol = goal.goal_tolerance if goal.goal_tolerance > 1e-6 \
            else self.default_goal_tol
        profile = TrapezoidalProfile(
            path.total_length, goal.max_linear_speed, goal.acceleration)
        point_speeds = list(getattr(goal, 'point_speeds', []) or [])
        watchdog = LocalizationWatchdog(
            timeout_sec=self.node.get_parameter('watchdog_timeout_sec').value,
            jump_threshold=self.node.get_parameter('watchdog_jump_threshold').value)
        self.commander.reset()

        start = time.time()
        # 첫 포즈 대기 (5 s)
        pose = None
        while time.time() - start < 5.0:
            pose = self.lookup_pose()
            if pose is not None:
                break
            time.sleep(0.05)
        if pose is None:
            log.error(f'{self.name}: 5 s 내 포즈 없음 — 중단')
            goal_handle.abort()
            return self._result(self.STATUS_TIMEOUT)

        log.info(f'{self.name}: 시작 pose=({pose[0]:.2f},{pose[1]:.2f}) '
                 f'경로 {path.total_length:.1f} m / {len(path.waypoints)} wpts '
                 f'tol={goal_tol} m')

        period = 1.0 / self.rate_hz
        arc = 0.0
        v_est = 0.0
        stall_since = None      # 정체(충돌) 감지 — 명령>0 인데 실속도≈0
        stall_limit = self.node.get_parameter('stall_timeout_sec').value
        while rclpy.ok():
            now = time.time()
            elapsed = now - start

            if goal_handle.is_cancel_requested:
                self.commander.stop_and_release(lambda: self.speed_est.buf[-1]
                                                if self.speed_est.buf else 0.0)
                goal_handle.canceled()
                return self._result(self.STATUS_CANCEL, arc, 0.0, elapsed)

            if elapsed > self.max_timeout:
                self.commander.stop_and_release(lambda: v_est)
                log.error(f'{self.name}: 타임아웃 {elapsed:.0f} s')
                goal_handle.abort()
                return self._result(self.STATUS_TIMEOUT, arc, 0.0, elapsed)

            p = self.lookup_pose()
            if p is not None:
                pose = p
                watchdog.update_pose(pose[0], pose[1], pose[2], now)
                v_est = self.speed_est.update(pose[0], pose[1], now)

            if not watchdog.healthy(now):
                self.commander.stop_and_release(lambda: v_est)
                log.error(f'{self.name}: 측위 워치독 발동(타임아웃/점프) — 중단')
                goal_handle.abort()
                return self._result(self.STATUS_TIMEOUT, arc, 0.0, elapsed)

            closest = geo.closest_point(path, pose[0], pose[1])
            arc = closest.arc_length
            cte = closest.cross_track

            if abs(cte) > self.lateral_abort:
                self.commander.stop_and_release(lambda: v_est)
                log.error(f'{self.name}: 횡이탈 {cte:.2f} m — 중단')
                goal_handle.abort()
                return self._result(self.STATUS_TIMEOUT, arc, cte, elapsed)

            gx, gy = path.waypoints[-1]
            dist_to_goal = math.hypot(gx - pose[0], gy - pose[1])
            if dist_to_goal < goal_tol:
                self.commander.stop_and_release(lambda: v_est)
                final_cte = geo.closest_point(path, pose[0], pose[1]).cross_track
                log.info(f'{self.name}: 도착 (주행 {arc:.1f} m, CTE {final_cte:.2f} m, '
                         f'{elapsed:.1f} s)')
                goal_handle.succeed()
                return self._result(self.STATUS_OK, arc, final_cte, elapsed)

            v_cmd, phase = profile.speed_at(min(arc, path.total_length))
            # waypoint 별 목표 속도(제동거리 선반영) 상한
            if point_speeds:
                v_cmd = min(v_cmd, geo.speed_limit_at(
                    path.cumulative, point_speeds, arc,
                    goal.acceleration, goal.max_linear_speed))
            # 최종 접근은 남은 거리 기반 감속이 지배하도록 프로파일 하한 유지
            if phase != DONE and v_cmd < self.min_speed:
                v_cmd = self.min_speed

            # 정체(충돌) 감지 — 유의미한 속도를 명령 중인데 실속도가 계속 0 이면
            # 장애물에 걸린 것: 계속 미는 대신 중단한다 (라이브 충돌 실측 반영)
            if v_cmd > 0.5 and v_est < 0.15 and elapsed > 3.0:
                if stall_since is None:
                    stall_since = now
                elif now - stall_since > stall_limit:
                    self.commander.stop_and_release(lambda: v_est)
                    log.error(f'{self.name}: 정체 {stall_limit:.0f} s (충돌 의심) — 중단')
                    goal_handle.abort()
                    return self._result(self.STATUS_SAFETY, arc, cte, elapsed)
            else:
                stall_since = None

            delta = self.steer(goal, path, closest, pose, v_est)
            steer_norm = self.steer_sign * (delta / self.max_steer)
            watchdog.set_speed(v_cmd)
            self.commander.command(v_cmd, v_est, steer_norm, period)

            fb = self.action_type.Feedback()
            fb.current_distance = float(arc)
            fb.remaining_distance = float(max(0.0, path.total_length - arc))
            fb.cross_track_error = float(cte)
            fb.heading_error = float(math.degrees(
                geo.normalize_angle(closest.seg_yaw - pose[2])))
            fb.current_speed = float(v_est)
            fb.steer_command = float(steer_norm)
            fb.current_waypoint_index = int(closest.segment_idx)
            fb.total_waypoints = len(path.waypoints)
            goal_handle.publish_feedback(fb)

            time.sleep(period)

        self.commander.stop_and_release(lambda: v_est)
        return self._result(self.STATUS_TIMEOUT, arc, 0.0, time.time() - start)

    def steer(self, goal, path, closest, pose, v_est):
        raise NotImplementedError


class StanleyServer(PathTrackingServer):
    """전륜축 기준 Stanley (ADR D3)."""

    def steer(self, goal, path, closest, pose, v_est):
        k = goal.k_stanley if goal.k_stanley > 1e-6 \
            else self.node.get_parameter('stanley_k').value
        k_soft = goal.k_soft if goal.k_soft > 1e-6 \
            else self.node.get_parameter('stanley_k_soft').value
        fx, fy = geo.front_axle(pose[0], pose[1], pose[2], self.wheelbase)
        front = geo.closest_point(path, fx, fy)
        e_theta = geo.normalize_angle(front.seg_yaw - pose[2])
        return geo.stanley_steer(e_theta, front.cross_track, v_est,
                                 k, k_soft, self.max_steer)


class PurePursuitServer(PathTrackingServer):
    """정식 곡률 Pure Pursuit (ADR D3)."""

    def steer(self, goal, path, closest, pose, v_est):
        ld = goal.lookahead_distance if goal.lookahead_distance > 1e-6 \
            else self.node.get_parameter('pp_lookahead').value
        lx, ly = geo.lookahead_point(path, closest, ld)
        alpha = geo.normalize_angle(
            math.atan2(ly - pose[1], lx - pose[0]) - pose[2])
        return geo.pure_pursuit_steer(alpha, ld, self.wheelbase, self.max_steer)
