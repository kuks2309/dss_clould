"""사다리꼴 속도 프로파일 — 원본 motion_profile.cpp 의 검증 수식 포팅."""

import math

ACCEL, CRUISE, DECEL, DONE = 1, 2, 3, 4


class TrapezoidalProfile:
    """목표 거리·최대속도·가속도(·탈출속도)로 위치 기반 속도를 준다."""

    def __init__(self, target_distance, max_speed, acceleration, exit_speed=0.0):
        self.target = target_distance
        self.acc = acceleration
        self.exit_speed = exit_speed
        d_accel = (max_speed ** 2) / (2.0 * acceleration)
        d_decel = (max_speed ** 2 - exit_speed ** 2) / (2.0 * acceleration)
        if d_accel + d_decel <= target_distance:
            self.peak = max_speed
            self.accel_dist = d_accel
            self.decel_start = target_distance - d_decel
        else:
            peak = math.sqrt(acceleration * target_distance + exit_speed ** 2 / 2.0)
            self.peak = max(peak, exit_speed)
            self.accel_dist = (self.peak ** 2) / (2.0 * acceleration)
            self.decel_start = self.accel_dist

    def speed_at(self, position):
        """(speed, phase) — 위치는 [0, target] 클램프."""
        pos = max(0.0, min(position, self.target))
        if pos >= self.target:
            return (self.exit_speed, DONE)
        if pos < self.accel_dist:
            v = min(math.sqrt(2.0 * self.acc * pos), self.peak)
            return (v, ACCEL)
        if pos < self.decel_start:
            return (self.peak, CRUISE)
        remaining = self.target - pos
        v = min(math.sqrt(self.exit_speed ** 2 + 2.0 * self.acc * remaining), self.peak)
        return (v, DECEL)
