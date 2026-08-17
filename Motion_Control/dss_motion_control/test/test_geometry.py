"""geometry·motion_profile·watchdog 순수 로직 단위시험 (ROS 비의존)."""

import math

from dss_motion_control import geometry as geo
from dss_motion_control.motion_profile import (
    ACCEL, CRUISE, DECEL, DONE, TrapezoidalProfile)
from dss_motion_control.watchdog import LocalizationWatchdog


def test_build_path_rejects_degenerate():
    assert geo.build_path([(0, 0)]) is None
    assert geo.build_path([(0, 0), (0, 0)]) is None
    p = geo.build_path([(0, 0), (10, 0)])
    assert p is not None and abs(p.total_length - 10.0) < 1e-9


def test_closest_point_cte_sign():
    p = geo.build_path([(0, 0), (10, 0)])
    left = geo.closest_point(p, 5.0, 1.0)     # +x 진행 경로의 왼쪽(+y)
    right = geo.closest_point(p, 5.0, -1.0)
    assert left.cross_track > 0.99
    assert right.cross_track < -0.99
    assert abs(left.arc_length - 5.0) < 1e-9
    assert abs(left.seg_yaw) < 1e-9


def test_lookahead_interpolation_and_clamp():
    p = geo.build_path([(0, 0), (10, 0), (10, 10)])
    c = geo.closest_point(p, 0.0, 0.0)
    lx, ly = geo.lookahead_point(p, c, 5.0)
    assert abs(lx - 5.0) < 1e-9 and abs(ly) < 1e-9
    lx, ly = geo.lookahead_point(p, c, 15.0)   # 모서리 넘어 두 번째 세그먼트
    assert abs(lx - 10.0) < 1e-9 and abs(ly - 5.0) < 1e-9
    lx, ly = geo.lookahead_point(p, c, 100.0)  # 경로 끝 클램프
    assert (lx, ly) == (10.0, 10.0)


def test_stanley_steer_direction():
    # 왼쪽 이탈(CTE>0) → 오른쪽 조향(δ<0)
    d = geo.stanley_steer(0.0, 1.0, 2.0, 1.0, 1.0, math.radians(30))
    assert d < 0
    # 오른쪽 이탈 → 왼쪽 조향
    d = geo.stanley_steer(0.0, -1.0, 2.0, 1.0, 1.0, math.radians(30))
    assert d > 0
    # 클램프
    d = geo.stanley_steer(math.pi, 5.0, 0.1, 10.0, 0.1, math.radians(30))
    assert abs(d) <= math.radians(30) + 1e-9


def test_pure_pursuit_steer():
    # 좌측 목표(α>0) → 좌조향(δ>0), κ=2 sinα/Ld
    d = geo.pure_pursuit_steer(math.radians(10), 8.0, 2.7, math.radians(30))
    expected = math.atan(2.7 * 2.0 * math.sin(math.radians(10)) / 8.0)
    assert abs(d - expected) < 1e-9 and d > 0
    assert geo.pure_pursuit_steer(0.0, 8.0, 2.7, 0.5) == 0.0


def test_front_axle_projection():
    fx, fy = geo.front_axle(0.0, 0.0, 0.0, 2.7)
    assert abs(fx - 2.7) < 1e-9 and abs(fy) < 1e-9
    fx, fy = geo.front_axle(0.0, 0.0, math.pi / 2, 2.7)
    assert abs(fx) < 1e-9 and abs(fy - 2.7) < 1e-9


def test_straight_path_spacing():
    pts = geo.straight_path(0, 0, 10, 0, 2.0)
    assert pts[0] == (0, 0) and pts[-1] == (10, 0)
    assert len(pts) == 6                      # 0,2,4,6,8,10


def test_trapezoidal_profile_phases():
    pr = TrapezoidalProfile(100.0, 5.0, 1.0)   # 가속 12.5 m, 감속 12.5 m
    v, ph = pr.speed_at(0.0)
    assert v == 0.0 and ph == ACCEL
    v, ph = pr.speed_at(50.0)
    assert abs(v - 5.0) < 1e-9 and ph == CRUISE
    v, ph = pr.speed_at(95.0)
    assert ph == DECEL and v < 5.0
    v, ph = pr.speed_at(100.0)
    assert v == 0.0 and ph == DONE


def test_trapezoidal_triangular_case():
    pr = TrapezoidalProfile(4.0, 5.0, 1.0)     # 도달 불가 → 삼각
    assert pr.peak < 5.0
    v_mid, _ = pr.speed_at(2.0)
    assert abs(v_mid - pr.peak) < 1e-6


def test_similarity_2pt_scale_rotation():
    # map (0,0)→(1,0) 이 utm (10,10)→(10,12): 축척 2·회전 +90°
    t = geo.similarity_2pt((0, 0), (1, 0), (10, 10), (10, 12))
    assert t is not None
    s, th, _, _ = t
    assert abs(s - 2.0) < 1e-9 and abs(th - math.pi / 2) < 1e-9
    ux, uy = geo.apply_similarity(t, 0.5, 0.0)
    assert abs(ux - 10.0) < 1e-9 and abs(uy - 11.0) < 1e-9
    ux, uy = geo.apply_similarity(t, 0.0, 1.0)   # 왼쪽 1 m → -x 방향 2 m
    assert abs(ux - 8.0) < 1e-9 and abs(uy - 10.0) < 1e-9
    assert geo.similarity_2pt((0, 0), (0, 0), (1, 1), (2, 2)) is None


def test_speed_limit_at_braking_and_segment():
    # 경로 0-10-20 m, 10 m 지점 목표 1 m/s (a=1)
    cum = [0.0, 10.0, 20.0]
    sp = [3.0, 1.0, 0.0]                       # 0 = 제한 없음(vmax)
    # 멀리서(arc=0): sqrt(1+2·1·10)=√21 ≈ 4.58 > 현 구간 3.0 → 3.0
    assert abs(geo.speed_limit_at(cum, sp, 0.0, 1.0, 5.0) - 3.0) < 1e-9
    # 제동 구간 진입(arc=8): sqrt(1+2·2)=√5 ≈ 2.24 (구간 3.0 보다 작음)
    assert abs(geo.speed_limit_at(cum, sp, 8.0, 1.0, 5.0)
               - math.sqrt(5.0)) < 1e-9
    # 지점 통과 후(arc=12): 현 구간 속도 1.0 이 상한
    assert abs(geo.speed_limit_at(cum, sp, 12.0, 1.0, 5.0) - 1.0) < 1e-9
    # point_speeds 미지정·길이 불일치 → vmax
    assert geo.speed_limit_at(cum, [], 0.0, 1.0, 5.0) == 5.0
    assert geo.speed_limit_at(cum, [1.0], 0.0, 1.0, 5.0) == 5.0


def test_watchdog_timeout_and_jump():
    wd = LocalizationWatchdog(timeout_sec=1.0, jump_threshold=0.5,
                              velocity_margin=1.5)
    assert not wd.healthy(0.0)                 # 포즈 없음
    wd.update_pose(0, 0, 0, 0.0)
    assert wd.healthy(0.5)
    assert not wd.healthy(2.0)                 # 타임아웃
    wd.update_pose(0, 0, 0, 2.0)
    wd.set_speed(1.0)
    wd.update_pose(0.1, 0, 0, 2.1)             # 정상 이동
    assert wd.healthy(2.2)
    wd.update_pose(5.0, 0, 0, 2.3)             # 점프 (기대 0.15 m ≪ 4.9 m)
    assert not wd.healthy(2.4)                 # 점프 1회 소비
    wd.update_pose(5.1, 0, 0, 2.5)
    assert wd.healthy(2.6)                     # 회복
