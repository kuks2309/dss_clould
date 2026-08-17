#!/usr/bin/env python3
"""SLAM Manager 3D UI — DSS 3 스택 탭 GUI (프로그램 방식 Qt, 디자이너 .ui 불사용).

스레드 규율: worker 스레드는 위젯을 만지지 않는다 — 로그·상태 전달은 전부
pyqtSignal 경유(원본 리뷰 High 결함 회피). ROS 콜백은 main() 의 QTimer 가
메인스레드에서 spin 하므로 시그널 emit 만 한다.
"""

import math
import os
import shutil
import sqlite3
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from PyQt5 import QtWidgets
from PyQt5.QtCore import QDateTime, QTimer, pyqtSignal
from PyQt5.QtWidgets import QFileDialog, QMessageBox

from slam_manager_3d.stacks import STACKS, LIOSAM_RENAME


class SlamManager3DUI(QtWidgets.QMainWindow):
    """3-스택(LIO-SAM / HDL / RTAB-Map) 구동 관리 메인 윈도우."""

    log_signal = pyqtSignal(str)
    pose_signal = pyqtSignal(str, float, float, float, float, float, float)
    clock_signal = pyqtSignal(float, bool)

    def __init__(self, manager, ws):
        super().__init__()
        self.manager = manager
        self.ws = Path(ws)
        self._pose_labels = {}
        self._map_edits = {}
        self._buttons = {}

        self.setWindowTitle('DSS SLAM Manager 3D')
        self.resize(760, 640)

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)

        # 공통 상단 바 — DSS 클럭·리셋 배너·전체 정지
        top = QtWidgets.QHBoxLayout()
        self.lblClock = QtWidgets.QLabel('sim clock: —')
        self.lblBanner = QtWidgets.QLabel('')
        self.lblBanner.setStyleSheet('color: #B71C1C; font-weight: bold;')
        btn_stop_all = QtWidgets.QPushButton('전체 정지')
        btn_stop_all.clicked.connect(self.on_stop_all)
        top.addWidget(self.lblClock)
        top.addWidget(self.lblBanner, 1)
        top.addWidget(btn_stop_all)
        root.addLayout(top)

        tabs = QtWidgets.QTabWidget()
        for key, cfg in STACKS.items():
            tabs.addTab(self._build_tab(key), cfg['title'])
        root.addWidget(tabs, 2)

        self.txtLog = QtWidgets.QTextEdit()
        self.txtLog.setReadOnly(True)
        root.addWidget(self.txtLog, 1)

        self.setCentralWidget(central)

        self.log_signal.connect(self.log)
        self.pose_signal.connect(self.on_pose)
        self.clock_signal.connect(self.on_clock)

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_button_states)
        self.status_timer.start(500)

        self.log('DSS SLAM Manager 3D 준비 완료 (자동 감지 없음 — DSS 고정 구성)')

    # ---------------- 탭 구성 ----------------

    def _build_tab(self, key):
        cfg = STACKS[key]
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)

        grp_map = QtWidgets.QGroupBox('매핑')
        h1 = QtWidgets.QHBoxLayout(grp_map)
        b_start = QtWidgets.QPushButton('매핑 시작')
        b_stop = QtWidgets.QPushButton('매핑 정지')
        b_save = QtWidgets.QPushButton('지도 저장')
        b_start.clicked.connect(lambda _, k=key: self.on_start(k, 'mapping'))
        b_stop.clicked.connect(lambda _, k=key: self.on_stop(k, 'mapping'))
        b_save.clicked.connect(lambda _, k=key: self.on_save(k))
        for b in (b_start, b_stop, b_save):
            h1.addWidget(b)
        lay.addWidget(grp_map)

        grp_loc = QtWidgets.QGroupBox('측위 (localization)')
        v2 = QtWidgets.QVBoxLayout(grp_loc)
        h2 = QtWidgets.QHBoxLayout()
        edit = QtWidgets.QLineEdit(cfg['map_hint'].format(ws=self.ws))
        edit.setReadOnly(not cfg['map_editable'])
        h2.addWidget(edit, 1)
        if cfg['map_editable']:
            b_browse = QtWidgets.QPushButton('찾기')
            b_browse.clicked.connect(lambda _, k=key: self.on_browse(k))
            h2.addWidget(b_browse)
        v2.addLayout(h2)
        h3 = QtWidgets.QHBoxLayout()
        b_lstart = QtWidgets.QPushButton('측위 시작')
        b_lstop = QtWidgets.QPushButton('측위 정지')
        b_lstart.clicked.connect(lambda _, k=key: self.on_start(k, 'loc'))
        b_lstop.clicked.connect(lambda _, k=key: self.on_stop(k, 'loc'))
        h3.addWidget(b_lstart)
        h3.addWidget(b_lstop)
        if key == 'hdl':
            b_seed = QtWidgets.QPushButton('원점 시드+재위치화')
            b_seed.clicked.connect(self.on_hdl_seed)
            h3.addWidget(b_seed)
        v2.addLayout(h3)
        lay.addWidget(grp_loc)

        grp_pose = QtWidgets.QGroupBox('위치 (6-DOF)')
        h4 = QtWidgets.QHBoxLayout(grp_pose)
        labels = {}
        for name in ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'):
            lbl = QtWidgets.QLabel(f'{name}: —')
            labels[name] = lbl
            h4.addWidget(lbl)
        lay.addWidget(grp_pose)
        lay.addStretch(1)

        self._pose_labels[key] = labels
        self._map_edits[key] = edit
        self._buttons[key] = {
            'mapping_start': b_start, 'mapping_stop': b_stop, 'save': b_save,
            'loc_start': b_lstart, 'loc_stop': b_lstop,
        }
        return w

    # ---------------- 슬롯 ----------------

    def log(self, message):
        stamp = QDateTime.currentDateTime().toString('hh:mm:ss')
        self.txtLog.append(f'[{stamp}] {message}')

    def on_pose(self, key, x, y, z, roll, pitch, yaw):
        labels = self._pose_labels.get(key)
        if not labels:
            return
        labels['X'].setText(f'X: {x:.3f}')
        labels['Y'].setText(f'Y: {y:.3f}')
        labels['Z'].setText(f'Z: {z:.3f}')
        labels['Roll'].setText(f'Roll: {math.degrees(roll):.1f}')
        labels['Pitch'].setText(f'Pitch: {math.degrees(pitch):.1f}')
        labels['Yaw'].setText(f'Yaw: {math.degrees(yaw):.1f}')

    def on_clock(self, sec, reset):
        self.lblClock.setText(f'sim clock: {sec:.0f}s')
        if reset:
            self.lblBanner.setText(
                '⚠ DSS 리셋 감지 — 실행 중 스택은 정지 후 재기동하세요')
            self.log('⚠ DSS 리셋 감지 (sim clock 역행) — 스택 재기동 필요')

    # ---------------- 동작 ----------------

    def _confirm_exclusive(self, key):
        """다른 스택이 살아 있으면 확인 — 3 스택이 같은 map/odom TF 를 다툰다."""
        others = [k for k in self.manager.KEYS
                  if not k.startswith(key) and self.manager.is_running(k)]
        if not others:
            return True
        reply = QMessageBox.question(
            self, '동시 기동 확인',
            f'다른 스택이 실행 중입니다: {", ".join(others)}\n'
            'TF(map/odom)가 충돌할 수 있습니다. 계속할까요?',
            QMessageBox.Yes | QMessageBox.No)
        return reply == QMessageBox.Yes

    def on_start(self, key, mode):
        cfg = STACKS[key]
        proc_key = f'{key}_{mode}'
        other = f'{key}_{"loc" if mode == "mapping" else "mapping"}'
        if self.manager.is_running(other):
            QMessageBox.warning(self, '오류',
                                '같은 스택의 매핑/측위는 동시에 못 돌립니다.')
            return
        if not self._confirm_exclusive(key):
            return

        subst = {}
        if mode == 'loc' and cfg['map_editable']:
            map_path = self._map_edits[key].text().strip()
            if not os.path.exists(map_path):
                QMessageBox.warning(self, '오류', f'지도 파일 없음:\n{map_path}')
                return
            subst['map'] = map_path

        def _worker():
            ok = self.manager.start(proc_key, cfg[mode], subst)
            self.log_signal.emit(
                f'{cfg["title"]} {mode} ' + ('기동' if ok else '기동 실패'))
        threading.Thread(target=_worker, daemon=True).start()

    def on_stop(self, key, mode):
        proc_key = f'{key}_{mode}'

        def _worker():
            ok = self.manager.stop(proc_key)
            self.log_signal.emit(
                f'{STACKS[key]["title"]} {mode} ' + ('정지' if ok else '정지 대상 없음'))
        threading.Thread(target=_worker, daemon=True).start()

    def on_save(self, key):
        cfg = STACKS[key]
        save_dir = Path(cfg['save_dir'].format(ws=self.ws))
        save_dir.mkdir(parents=True, exist_ok=True)
        mode = cfg['save_mode']

        def _worker():
            try:
                if mode == 'hdl_service':
                    dest = save_dir / 'map.pcd'
                    result = subprocess.run(
                        ['ros2', 'service', 'call', '/hdl_graph_slam/save_map',
                         'hdl_graph_slam/srv/SaveMap',
                         f"{{utm: false, resolution: 0.1, destination: '{dest}'}}"],
                        capture_output=True, text=True, timeout=90)
                    ok = 'success=True' in result.stdout
                    self.log_signal.emit(
                        f'HDL 지도 저장 {"성공" if ok else "실패"}: {dest}')
                elif mode == 'rtab_sqlite_backup':
                    src = Path.home() / '.ros' / 'rtabmap.db'
                    if not src.exists():
                        self.log_signal.emit('RTAB 저장 실패: ~/.ros/rtabmap.db 없음')
                        return
                    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                    dest = save_dir / f'rtabmap3d_{ts}.db'
                    # sqlite backup API — 라이브(WAL) DB 도 일관 스냅샷
                    with sqlite3.connect(str(src)) as con_src, \
                         sqlite3.connect(str(dest)) as con_dst:
                        con_src.backup(con_dst)
                    self.log_signal.emit(f'RTAB 지도 저장: {dest}')
                elif mode == 'liosam_stop_rename':
                    self.log_signal.emit(
                        'LIO-SAM 저장 = 매핑 정지(자동 저장) 후 개명 — 정지합니다')
                    self.manager.stop(f'{key}_mapping')
                    time.sleep(2.0)
                    renamed = []
                    for old, new in LIOSAM_RENAME.items():
                        p = save_dir / old
                        if p.exists():
                            p.rename(save_dir / new)
                            renamed.append(new)
                    self.log_signal.emit(
                        f'LIO-SAM 지도 개명 완료: {", ".join(renamed) or "대상 없음"}')
            except Exception as e:  # 저장 실패는 로그로 보고, GUI 는 지속
                self.log_signal.emit(f'지도 저장 오류: {e}')
        threading.Thread(target=_worker, daemon=True).start()

    def on_browse(self, key):
        cfg = STACKS[key]
        start_dir = cfg['save_dir'].format(ws=self.ws)
        path, _ = QFileDialog.getOpenFileName(
            self, '지도 선택', start_dir, cfg['map_filter'])
        if path:
            self._map_edits[key].setText(path)
            self.log(f'{cfg["title"]} 지도 선택: {path}')

    def on_hdl_seed(self):
        """HDL 측위 초기화 — 원점 initialpose 시드 후 BBS 재위치화 호출."""
        def _worker():
            subprocess.run(
                ['ros2', 'topic', 'pub', '--once', '/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'],
                capture_output=True, timeout=15)
            time.sleep(3.0)
            result = subprocess.run(
                ['ros2', 'service', 'call', '/relocalize', 'std_srvs/srv/Empty'],
                capture_output=True, text=True, timeout=30)
            ok = result.returncode == 0
            self.log_signal.emit(
                'HDL 원점 시드 + 재위치화 ' + ('호출 완료' if ok else '호출 실패'))
        threading.Thread(target=_worker, daemon=True).start()
        self.log('HDL 원점 시드 + BBS 재위치화 예약')

    # ---------------- 상태 ----------------

    def update_button_states(self):
        style_run = 'background-color: #2196F3; color: white; font-weight: bold;'
        for key, btns in self._buttons.items():
            mapping = self.manager.is_running(f'{key}_mapping')
            loc = self.manager.is_running(f'{key}_loc')
            btns['mapping_start'].setEnabled(not mapping and not loc)
            btns['mapping_stop'].setEnabled(mapping)
            btns['loc_start'].setEnabled(not mapping and not loc)
            btns['loc_stop'].setEnabled(loc)
            # LIO-SAM 저장은 정지를 동반하므로 매핑 중에만 의미 있음
            btns['save'].setEnabled(
                mapping if STACKS[key]['save_mode'] == 'liosam_stop_rename'
                else True)
            btns['mapping_start'].setStyleSheet(style_run if mapping else '')
            btns['loc_start'].setStyleSheet(style_run if loc else '')

    def on_stop_all(self):
        reply = QMessageBox.question(
            self, '확인', '실행 중인 모든 스택을 정지할까요?',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        def _worker():
            self.manager.stop_all()
            self.log_signal.emit('전체 정지 완료')
        threading.Thread(target=_worker, daemon=True).start()

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self, '종료 확인', '모든 스택을 정지하고 종료할까요?',
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.manager.stop_all()
            event.accept()
        else:
            event.ignore()
