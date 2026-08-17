#!/usr/bin/env python3
"""openpilot 차선 인식 실시간 뷰어 (PyQt5).

DSS 카메라(/dss/sensor/camera/rgb)를 구독해 supercombo 추론 오버레이를 실시간 표시한다.
DSS 없이도 `--frames-dir` 로 캡처 시퀀스를 재생 데모할 수 있다.
FOV·pitch 스핀박스로 캘리브레이션을 라이브 튜닝한다(변경 시 프레임 페어 리셋).

함수표: docs/code_review/openpilot_lane/2026-08-17.md (v3)
"""
import argparse
import glob
import math
import os
import queue
import sys
import threading
import time

import cv2
import numpy as np
import onnxruntime as ort
from PyQt5 import QtCore, QtGui, QtWidgets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_supercombo as rs

CAM_W, CAM_H = 640, 480
LANE_NAMES = ['좌외곽', '좌차선', '우차선', '우외곽']
# LANE_COLORS(BGR) → CSS
LANE_CSS = ['#0080ff', '#00ff00', '#ffff00', '#ff8000']


class LaneModel:
    """supercombo 추론 엔진 — infer 는 InferLoop 스레드에서만 호출(스레드 안전 경계)."""

    def __init__(self, fov_deg: float, pitch: float):
        self.sess = ort.InferenceSession(rs.MODEL_PATH, providers=['CPUExecutionProvider'])
        self.features = np.zeros((1, 24, 512), np.float16)
        self.desire = np.zeros((1, 25, 8), np.float16)
        self.tc = np.array([[1.0, 0.0]], np.float16)   # 한국: 우측통행
        self.at = np.array([[0.3, 0.9]], np.float16)
        # 모델 시간 문맥: 페어 간격 0.2 s (MODEL_CONTEXT_FREQ=5 Hz) — 최근 프레임 이력에서 선택
        self.hist = []  # [(t, med6, big6)]
        self._pending = None
        self._lock = threading.Lock()
        self.cam_wh = (CAM_W, CAM_H)   # 실제 수신 해상도로 첫 프레임에서 갱신
        self._calib = (fov_deg, pitch)
        self._apply_calib(fov_deg, pitch)

    def set_calib(self, fov_deg: float, pitch: float):
        """GUI 스레드에서 호출 — 다음 infer 시점에 적용."""
        with self._lock:
            self._pending = (fov_deg, pitch)

    def _apply_calib(self, fov_deg: float, pitch: float):
        w, h = self.cam_wh
        fl = (w / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
        self.cam_K = np.array([[fl, 0.0, w / 2.0], [0.0, fl, h / 2.0], [0.0, 0.0, 1.0]])
        self.euler = np.array([0.0, pitch, 0.0])
        self.M_med = rs.get_warp_matrix(self.euler, self.cam_K, False)
        self.M_big = rs.get_warp_matrix(self.euler, self.cam_K, True)
        self._calib = (fov_deg, pitch)
        self.hist = []  # 기하 변경 → 시간 문맥 리셋

    def infer(self, rgb):
        with self._lock:
            if self._pending is not None:
                self._apply_calib(*self._pending)
                self._pending = None
        t0 = time.perf_counter()
        h, w = rgb.shape[:2]
        if (w, h) != self.cam_wh:   # 카메라 해상도 변경 감지 → intrinsics 재구성
            self.cam_wh = (w, h)
            self._apply_calib(*self._calib)
        now = time.monotonic()
        cur = (rs.rgb_to_yuv6ch(rs.warp_to_model(rgb, self.M_med)),
               rs.rgb_to_yuv6ch(rs.warp_to_model(rgb, self.M_big)))
        self.hist.append((now, cur[0], cur[1]))
        self.hist = [e for e in self.hist if now - e[0] < 0.6]
        # 0.2 s 전에 가장 가까운 프레임을 페어로 (학습 문맥과 일치)
        older = [e for e in self.hist if now - e[0] >= 0.12]
        if not older:
            return None
        prev = min(older, key=lambda e: abs((now - e[0]) - 0.2))
        img = np.concatenate([prev[1], cur[0]])[None]
        big = np.concatenate([prev[2], cur[1]])[None]
        out = self.sess.run(None, {'img': img, 'big_img': big,
                                   'features_buffer': self.features, 'desire_pulse': self.desire,
                                   'traffic_convention': self.tc, 'action_t': self.at})
        vec = np.asarray(out[0], np.float32)[0]
        self.features = np.concatenate(
            [self.features[:, 1:], vec[rs.SLICES['hidden_state']].astype(np.float16)[None, None]], axis=1)
        lanes, prob, edges = rs.parse_mdn_lanes(vec)
        plan = rs.parse_plan(vec)
        # 원본 카메라 영상 그대로 + 검출 오버레이 (영상처리 확대 없음 — 카메라단 해상도가 정공)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        overlay = rs.draw_overlay(bgr, lanes, prob, edges, self.cam_K, self.euler, plan=plan)
        return overlay, prob, (time.perf_counter() - t0) * 1000.0


class RosGrabber(threading.Thread):
    """rclpy 백그라운드 스핀 — 최신 프레임만 보관."""

    def __init__(self, topic: str):
        super().__init__(daemon=True)
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
        self._rclpy = rclpy
        self._seq = 0
        self._rgb = None
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self.node = rclpy.create_node('lane_viewer_qt')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        def on_image(msg):
            if msg.encoding != 'rgb8':
                return
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
            with self._lock:
                self._rgb = rgb
                self._seq += 1

        self.node.create_subscription(Image, topic, on_image, qos)

    def latest(self):
        with self._lock:
            return self._seq, self._rgb

    def run(self):
        while not self._stop_evt.is_set():
            self._rclpy.spin_once(self.node, timeout_sec=0.2)

    def stop(self):
        self._stop_evt.set()


class PlaybackGrabber(threading.Thread):
    """캡처 PNG 시퀀스 루프 재생 (DSS 미접속 데모)."""

    def __init__(self, frames_dir: str, hz: float = 6.0):
        super().__init__(daemon=True)
        self.paths = sorted(glob.glob(os.path.join(frames_dir, '*.png')))
        if not self.paths:
            raise SystemExit(f'재생할 프레임 없음: {frames_dir}')
        self.dt = 1.0 / hz
        self._seq = 0
        self._rgb = None
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()

    def latest(self):
        with self._lock:
            return self._seq, self._rgb

    def run(self):
        i = 0
        while not self._stop_evt.is_set():
            rgb = cv2.cvtColor(cv2.imread(self.paths[i]), cv2.COLOR_BGR2RGB)
            with self._lock:
                self._rgb = rgb
                self._seq += 1
            i = (i + 1) % len(self.paths)
            time.sleep(self.dt)

    def stop(self):
        self._stop_evt.set()


class InferLoop(threading.Thread):
    """최신 프레임만 소비(백로그 드롭) → result_q. GUI 비블록."""

    def __init__(self, grabber, model: LaneModel):
        super().__init__(daemon=True)
        self.grabber = grabber
        self.model = model
        self.result_q: queue.Queue = queue.Queue(maxsize=2)
        self._stop_evt = threading.Event()

    def run(self):
        last_seq = 0
        while not self._stop_evt.is_set():
            seq, rgb = self.grabber.latest()
            if rgb is None or seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = seq
            res = self.model.infer(rgb)
            if res is None:
                continue
            if self.result_q.full():
                try:
                    self.result_q.get_nowait()
                except queue.Empty:
                    pass
            self.result_q.put(res)

    def stop(self):
        self._stop_evt.set()


class LaneViewerWindow(QtWidgets.QMainWindow):

    def __init__(self, model: LaneModel, infer_loop: InferLoop, mode: str,
                 fov: float, pitch: float):
        super().__init__()
        self.model = model
        self.infer_loop = infer_loop
        self._frame_times = []
        self.setWindowTitle(f'openpilot 차선 인식 — DSS ({mode})')

        self.video = QtWidgets.QLabel()
        self.video.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(CAM_W, CAM_H)
        self.video.setStyleSheet('background:#202020; color:#aaa;')
        self.video.setText('프레임 대기 중…')

        side = QtWidgets.QVBoxLayout()
        side.addWidget(QtWidgets.QLabel('<b>차선 존재 확률</b>'))
        self.bars = []
        for name, css in zip(LANE_NAMES, LANE_CSS):
            side.addWidget(QtWidgets.QLabel(name))
            bar = QtWidgets.QProgressBar()
            bar.setRange(0, 100)
            bar.setTextVisible(True)
            bar.setStyleSheet(f'QProgressBar::chunk {{ background: {css}; }}')
            side.addWidget(bar)
            self.bars.append(bar)
        side.addSpacing(12)

        form = QtWidgets.QFormLayout()
        self.fov_spin = QtWidgets.QDoubleSpinBox()
        self.fov_spin.setRange(40.0, 140.0)
        self.fov_spin.setSingleStep(2.0)
        self.fov_spin.setValue(fov)
        self.fov_spin.setSuffix(' °')
        self.pitch_spin = QtWidgets.QDoubleSpinBox()
        self.pitch_spin.setRange(-0.20, 0.60)
        self.pitch_spin.setSingleStep(0.01)
        self.pitch_spin.setDecimals(2)
        self.pitch_spin.setValue(pitch)
        self.pitch_spin.setSuffix(' rad')
        self.fov_spin.valueChanged.connect(self._on_calib)
        self.pitch_spin.valueChanged.connect(self._on_calib)
        form.addRow('카메라 HFOV', self.fov_spin)
        form.addRow('카메라 pitch', self.pitch_spin)
        side.addLayout(form)
        side.addStretch(1)
        note = QtWidgets.QLabel('FOV ⚠추정값 — DSS CameraInfo 미발행\n'
                                '색: 초록=좌차선 · 노랑=우차선 · 빨강=도로 가장자리')
        note.setWordWrap(True)
        side.addWidget(note)

        central = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(central)
        lay.addWidget(self.video, stretch=1)
        panel = QtWidgets.QWidget()
        panel.setLayout(side)
        panel.setFixedWidth(220)
        lay.addWidget(panel)
        self.setCentralWidget(central)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    def _on_calib(self):
        self.model.set_calib(self.fov_spin.value(), self.pitch_spin.value())

    def _tick(self):
        res = None
        while True:  # 큐 비우고 최신만
            try:
                res = self.infer_loop.result_q.get_nowait()
            except queue.Empty:
                break
        if res is None:
            return
        overlay, prob, dt_ms = res
        rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy()
        self.video.setPixmap(QtGui.QPixmap.fromImage(qimg).scaled(
            self.video.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation))
        for bar, p in zip(self.bars, prob):
            bar.setValue(int(round(float(p) * 100)))
        now = time.monotonic()
        self._frame_times = [t for t in self._frame_times if now - t < 3.0] + [now]
        fps = len(self._frame_times) / 3.0
        self.statusBar().showMessage(f'표시 {fps:.1f} fps · 추론 {dt_ms:.0f} ms/frame')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/dss/sensor/camera/rgb')
    ap.add_argument('--frames-dir', help='지정 시 라이브 대신 PNG 시퀀스 재생')
    ap.add_argument('--fov', type=float, default=rs.DEFAULT_FOV_DEG)
    ap.add_argument('--pitch', type=float, default=0.35)
    args = ap.parse_args()

    rclpy_inited = False
    if args.frames_dir:
        grabber = PlaybackGrabber(args.frames_dir)
        mode = '재생'
    else:
        import rclpy
        rclpy.init()
        rclpy_inited = True
        grabber = RosGrabber(args.topic)
        mode = '라이브'

    model = LaneModel(args.fov, args.pitch)
    infer_loop = InferLoop(grabber, model)
    grabber.start()
    infer_loop.start()

    app = QtWidgets.QApplication(sys.argv)
    win = LaneViewerWindow(model, infer_loop, mode, args.fov, args.pitch)
    win.resize(920, 540)
    win.show()
    code = app.exec_()

    infer_loop.stop()
    grabber.stop()
    grabber.join(timeout=2)
    if rclpy_inited:
        import rclpy
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
