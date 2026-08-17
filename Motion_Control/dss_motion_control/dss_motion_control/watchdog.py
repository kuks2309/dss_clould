"""측위 워치독 — 원본 localization_watchdog 포팅.

포즈 타임아웃과 '명령 속도 대비 비정상 점프'를 감시한다. DSS 리셋(측위 재초기화)·
측위 이탈 시 제어를 즉시 중단시키는 안전장치 (ADR D2 — 오늘 세션 실측 시나리오 대응).
"""

import math


class LocalizationWatchdog:

    def __init__(self, timeout_sec=2.0, jump_threshold=1.0, velocity_margin=1.5):
        self.timeout = timeout_sec
        self.jump_threshold = jump_threshold
        self.velocity_margin = velocity_margin
        self.last_time = None
        self.prev = None            # (x, y)
        self.pose = None            # (x, y, yaw)
        self.cmd_speed = 0.0
        self.jump_detected = False

    def update_pose(self, x, y, yaw, now):
        """포즈 갱신 + 직전 대비 점프 검출(명령속도×경과시간×여유 초과)."""
        if self.prev is not None and self.last_time is not None:
            dt = max(1e-3, now - self.last_time)
            jump = math.hypot(x - self.prev[0], y - self.prev[1])
            expected = self.cmd_speed * dt
            threshold = max(self.jump_threshold, expected * self.velocity_margin)
            if jump > threshold:
                self.jump_detected = True
        self.prev = (x, y)
        self.pose = (x, y, yaw)
        self.last_time = now

    def set_speed(self, mps):
        self.cmd_speed = abs(mps)

    def healthy(self, now):
        """포즈 부재/노후 또는 점프 발생 시 False (점프 플래그는 1회 소비)."""
        if self.last_time is None:
            return False
        if now - self.last_time > self.timeout:
            return False
        if self.jump_detected:
            self.jump_detected = False
            return False
        return True
