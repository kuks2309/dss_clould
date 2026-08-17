#!/usr/bin/env python3
"""SLAM Manager 3D — DSS 3 스택(dss_lio_sam / HDL / RTAB-Map) 구동 관리 노드.

원본 ros2_3dslam_ws slam_manager_3d 의 프로세스 관리 패턴(setsid 스크립트 +
PID 파일)을 채택하되, 리뷰(docs/code_review/slam_manager_3d/2026-08-17.md)에서
확정한 결함을 회피한다:
 - 정지는 자기 세션(SID) 한정 — 전역 pkill·ros2 daemon 재시작 없음
 - PID 생존 판정은 /proc cmdline 대조 병행 (PID 재사용 오탐 차단)
 - UI 접근은 전부 Qt 시그널 경유
 - Gazebo 자동 감지 없음 (DSS 도 /clock 발행 → /clock 검사는 항상 오판)
"""

import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from ament_index_python.packages import get_package_share_directory

from slam_manager_3d.stacks import STACKS, CLOCK_RESET_THRESHOLD_S


def find_workspace():
    """install/<pkg>/share/<pkg> 에서 워크스페이스 루트를 역산하고 실재 검증.

    원본은 실패 시 Path.home() 으로 조용히 강등돼 source 가 침묵 실패했다 —
    여기서는 setup.bash 부재를 즉시 예외로 올린다.
    """
    pkg_share = Path(get_package_share_directory('slam_manager_3d'))
    ws = pkg_share.parents[3]
    setup = ws / 'install' / 'setup.bash'
    if not setup.is_file():
        raise RuntimeError(f'워크스페이스 검증 실패: {setup} 없음')
    return ws


def quaternion_to_euler(q):
    """쿼터니언 → (roll, pitch, yaw) 라디안. pitch 는 ±90° 클램프."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ProcessHandle:
    """setsid 세션 리더 PID 추적.

    cmdline 대조는 exec 가 이미지(ros2 launch …)로 교체하는 순간 무효가 되므로,
    '세션 리더 속성(getsid(pid) == pid)'으로 생존을 판정한다 — 재사용된 PID 가
    우연히 자기 세션의 리더인 경우는 실질적으로 없어 재사용 오탐을 차단한다.
    """

    def __init__(self, pid, script_path):
        self.pid = pid
        self.script_path = script_path

    def alive(self):
        try:
            return os.getsid(self.pid) == self.pid
        except OSError:
            return False


class StackProcessManager:
    """스택×모드(매핑/측위) 프로세스의 기동·정지·생존 관리."""

    KEYS = [f'{s}_{m}' for s in STACKS for m in ('mapping', 'loc')]

    def __init__(self, ws):
        self.ws = Path(ws)
        self.handles = {k: None for k in self.KEYS}
        self.script_dir = Path(tempfile.mkdtemp(prefix='slam_manager_3d_'))

    def start(self, key, lines, subst=None):
        """셸 라인 목록을 한 세션 스크립트로 기동. 반환 = 성공 여부."""
        if self.handles.get(key) is not None and self.handles[key].alive():
            return False

        subst = dict(subst or {})
        subst.setdefault('ws', str(self.ws))
        body = '\n'.join(line.format(**subst) for line in lines)

        pid_file = self.script_dir / f'{key}.pid'
        script_path = self.script_dir / f'{key}.sh'
        # PID 를 source 보다 먼저 기록 — source 실패 시에도 추적자가 죽음을
        # 감지할 수 있다(원본의 침묵 실패 결함 회피).
        script_path.write_text(
            '#!/bin/bash\n'
            f'echo $$ > {pid_file}\n'
            f'source {self.ws}/install/setup.bash\n'
            f'{body}\n'
        )
        script_path.chmod(0o755)

        subprocess.Popen(
            ['setsid', 'bash', str(script_path)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            cwd=str(Path.home()),
            preexec_fn=os.setpgrp,
        )
        time.sleep(0.7)

        try:
            pid = int(pid_file.read_text().strip())
        except (OSError, ValueError):
            return False

        handle = ProcessHandle(pid, str(script_path))
        if not handle.alive():
            return False
        self.handles[key] = handle
        return True

    @staticmethod
    def _tree_pids(root_pid, sid):
        """정지 대상 = 세션(sid) 구성원 + 그들의 자손 전체.

        rviz2 등 일부 자식은 자체 세션을 새로 만들어 세션 한정 소탕을 벗어난다
        (실측: rviz sid ≠ 리더 sid). 시그널 전에 ppid 트리를 걸어 자손까지
        수집한다 — 자기 트리 한정이므로 전역 소탕 금지 원칙은 유지된다.
        """
        try:
            out = subprocess.run(['ps', '-eo', 'pid,ppid,sid'],
                                 capture_output=True, text=True, timeout=5).stdout
        except Exception:
            return {root_pid}
        children = {}
        session = set()
        for line in out.splitlines()[1:]:
            parts = line.split()
            if len(parts) != 3:
                continue
            pid, ppid, psid = (int(x) for x in parts)
            children.setdefault(ppid, []).append(pid)
            if psid == sid:
                session.add(pid)
        pids = set(session) | {root_pid}
        queue = list(pids)
        while queue:
            p = queue.pop()
            for c in children.get(p, []):
                if c not in pids:
                    pids.add(c)
                    queue.append(c)
        return pids

    def stop(self, key, grace_s=3.0):
        """자기 프로세스 트리 한정 정지 — SIGINT 유예 후 SIGKILL."""
        handle = self.handles.get(key)
        if handle is None:
            return False

        try:
            sid = os.getsid(handle.pid)
        except OSError:
            sid = handle.pid
        targets = self._tree_pids(handle.pid, sid)

        for sig in (signal.SIGINT,):
            for pid in targets:
                try:
                    os.kill(pid, sig)
                except OSError:
                    pass

        deadline = time.time() + grace_s
        while time.time() < deadline:
            if not handle.alive():
                break
            time.sleep(0.3)

        for pid in targets:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass

        self.handles[key] = None
        return True

    def is_running(self, key):
        handle = self.handles.get(key)
        return handle is not None and handle.alive()

    def stop_all(self):
        for key in self.KEYS:
            if self.is_running(key):
                self.stop(key)


class SlamManagerNode(Node):
    """오도메트리 3스택 + /clock 구독 — 표시는 전부 Qt 시그널 브리지로 전달."""

    def __init__(self, bridge):
        super().__init__('slam_manager_3d')
        self._bridge = bridge  # pose_signal(key,6f)·clock_signal(sec,reset) 보유
        self._last_clock = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        topics = [(k, cfg['odom_topic']) for k, cfg in STACKS.items()]
        topics += [(k, cfg['mapping_odom_topic'])
                   for k, cfg in STACKS.items() if 'mapping_odom_topic' in cfg]
        self._subs = [
            self.create_subscription(Odometry, topic, self._make_odom_cb(key), qos)
            for key, topic in topics
        ]
        self._clock_sub = self.create_subscription(
            Clock, '/clock', self._clock_cb, qos)

    def _make_odom_cb(self, key):
        def _cb(msg):
            p = msg.pose.pose.position
            r, pt, yw = quaternion_to_euler(msg.pose.pose.orientation)
            self._bridge.pose_signal.emit(key, p.x, p.y, p.z, r, pt, yw)
        return _cb

    def _clock_cb(self, msg):
        sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        reset = (self._last_clock is not None
                 and sec < self._last_clock - CLOCK_RESET_THRESHOLD_S)
        self._last_clock = sec
        self._bridge.clock_signal.emit(sec, reset)


def main(args=None):
    import sys
    from PyQt5 import QtWidgets
    from PyQt5.QtCore import QTimer
    from slam_manager_3d.slam_manager_3d_ui import SlamManager3DUI

    rclpy.init(args=args)
    ws = find_workspace()
    manager = StackProcessManager(ws)

    app = QtWidgets.QApplication(sys.argv)
    ui = SlamManager3DUI(manager, ws)
    node = SlamManagerNode(ui)
    ui.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    exit_code = app.exec_()

    manager.stop_all()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
