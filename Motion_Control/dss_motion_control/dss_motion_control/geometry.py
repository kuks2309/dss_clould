"""경로 기하 순수 함수 — ROS 비의존, 단위시험 대상.

원본 amr_motion_control_2wd 의 검증된 기하(최근접점 투영·부호 CTE·호장 보간)를
계승하되, 조향 수식은 Ackermann 차량용으로 재정식화한다(ADR D3):
 - Stanley: 전륜축 기준 CTE, δ = eθ + atan2(-k·cte, v + k_soft)
 - Pure Pursuit: κ = 2·sin(α)/Ld (원본의 차륜간격 분모 오용 정정), δ = atan(L·κ)
"""

import math
from dataclasses import dataclass


def normalize_angle(angle):
    """±π 정규화."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class PathData:
    waypoints: list          # [(x, y)]
    cumulative: list         # 누적 호장 [m]
    total_length: float


def build_path(points):
    """웨이포인트 목록 → PathData. 2점 미만·총길이 0 이면 None."""
    if len(points) < 2:
        return None
    cumulative = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        cumulative.append(cumulative[-1] + math.hypot(dx, dy))
    if cumulative[-1] < 1e-4:
        return None
    return PathData(list(points), cumulative, cumulative[-1])


@dataclass
class Closest:
    segment_idx: int
    x: float
    y: float
    cross_track: float       # 부호 CTE — 양수 = 경로 진행방향 기준 왼쪽
    arc_length: float
    seg_yaw: float           # 해당 세그먼트 진행 방향


def closest_point(path, rx, ry):
    """전 세그먼트 투영으로 최근접점·부호 CTE·호장을 구한다."""
    best = Closest(0, path.waypoints[0][0], path.waypoints[0][1], 0.0, 0.0, 0.0)
    min_d2 = float('inf')
    for i in range(len(path.waypoints) - 1):
        ax, ay = path.waypoints[i]
        bx, by = path.waypoints[i + 1]
        sdx, sdy = bx - ax, by - ay
        seg_len2 = sdx * sdx + sdy * sdy
        t = 0.0
        if seg_len2 > 1e-12:
            t = ((rx - ax) * sdx + (ry - ay) * sdy) / seg_len2
            t = max(0.0, min(1.0, t))
        cx, cy = ax + t * sdx, ay + t * sdy
        d2 = (rx - cx) ** 2 + (ry - cy) ** 2
        if d2 < min_d2:
            min_d2 = d2
            seg_len = math.sqrt(seg_len2)
            ux = sdx / seg_len if seg_len > 1e-12 else 1.0
            uy = sdy / seg_len if seg_len > 1e-12 else 0.0
            cte = ux * (ry - cy) - uy * (rx - cx)   # 외적 z — 왼쪽 양수
            best = Closest(
                i, cx, cy, cte,
                path.cumulative[i] + t * seg_len,
                math.atan2(sdy, sdx))
    return best


def lookahead_point(path, closest, ld):
    """최근접점 호장 + Ld 지점을 경로 위에서 보간(끝 넘으면 마지막 점)."""
    target = closest.arc_length + ld
    if target >= path.total_length:
        return path.waypoints[-1]
    seg = closest.segment_idx
    for i in range(seg, len(path.waypoints) - 1):
        if path.cumulative[i] <= target <= path.cumulative[i + 1]:
            seg = i
            break
    seg_len = path.cumulative[seg + 1] - path.cumulative[seg]
    t = 0.0
    if seg_len > 1e-12:
        t = max(0.0, min(1.0, (target - path.cumulative[seg]) / seg_len))
    ax, ay = path.waypoints[seg]
    bx, by = path.waypoints[seg + 1]
    return (ax + t * (bx - ax), ay + t * (by - ay))


def straight_path(x0, y0, x1, y1, spacing=2.0):
    """현재 위치→목표 직선 웨이포인트 (양 끝 포함, spacing 간격)."""
    dist = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(dist // spacing))
    pts = [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]
    return pts


def stanley_steer(e_theta, cte, v, k, k_soft, max_steer):
    """Stanley 조향각[rad] — CTE 양수(왼쪽 이탈) → 음수 조향(오른쪽 복귀)."""
    delta = e_theta + math.atan2(-k * cte, abs(v) + k_soft)
    return max(-max_steer, min(max_steer, delta))


def pure_pursuit_steer(alpha, ld, wheelbase, max_steer):
    """Pure Pursuit 조향각[rad] — κ=2 sinα/Ld, δ=atan(L·κ)."""
    if ld < 1e-6:
        return 0.0
    kappa = 2.0 * math.sin(alpha) / ld
    delta = math.atan(wheelbase * kappa)
    return max(-max_steer, min(max_steer, delta))


def front_axle(x, y, yaw, wheelbase):
    """후륜(base) 포즈 → 전륜축 위치 (Stanley 기준점)."""
    return (x + wheelbase * math.cos(yaw), y + wheelbase * math.sin(yaw))


def similarity_2pt(p0, p1, q0, q1):
    """두 대응점 (p: map → q: UTM) 으로 2D 유사변환 (s, θ, tx, ty) 추정.

    q = s·R(θ)·p + t. DSS GPS 는 합성 좌표(축척 ≈104·원점 재시작마다 변동)라
    축척 포함 추정이 필수. 기선(p0-p1)이 짧으면 None.
    """
    dpx, dpy = p1[0] - p0[0], p1[1] - p0[1]
    dqx, dqy = q1[0] - q0[0], q1[1] - q0[1]
    dp = math.hypot(dpx, dpy)
    if dp < 1e-6:
        return None
    s = math.hypot(dqx, dqy) / dp
    th = math.atan2(dqy, dqx) - math.atan2(dpy, dpx)
    c, si = math.cos(th), math.sin(th)
    tx = q0[0] - s * (c * p0[0] - si * p0[1])
    ty = q0[1] - s * (si * p0[0] + c * p0[1])
    return (s, th, tx, ty)


def apply_similarity(t, x, y):
    """similarity_2pt 결과 t 를 점 (x, y) 에 적용."""
    s, th, tx, ty = t
    c, si = math.cos(th), math.sin(th)
    return (s * (c * x - si * y) + tx, s * (si * x + c * y) + ty)


def speed_limit_at(cumulative, point_speeds, arc, accel, vmax):
    """waypoint 별 목표 속도 상한 — 느린 지점을 제동거리만큼 미리 반영.

    point_speeds[i] 는 cumulative[i] 지점의 목표 속도(0 이하 = vmax).
    허용 속도 = min_i sqrt(v_i^2 + 2·a·max(0, s_i - arc)) — 현재 위치에서
    가속도 a 로 감속해 지점 i 도달 시 v_i 이하가 되도록 하는 상한.
    """
    if not point_speeds or len(point_speeds) != len(cumulative):
        return vmax
    allow = vmax
    idx = 0                       # 마지막으로 지난 지점 = 현 구간 속도
    for i, (s_i, v_i) in enumerate(zip(cumulative, point_speeds)):
        v = v_i if v_i > 1e-3 else vmax
        if s_i <= arc:
            idx = i
        else:
            allow = min(allow, math.sqrt(v * v + 2.0 * accel * (s_i - arc)))
    v_here = point_speeds[idx] if point_speeds[idx] > 1e-3 else vmax
    return min(allow, v_here)
