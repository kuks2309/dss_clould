#!/usr/bin/env python3
"""DSS Waypoint Navigator — RViz 지도 클릭으로 waypoint 를 찍고 추종시키는 GUI.

사용 흐름:
 1. RViz 상단 "Publish Point" 도구로 지도 위를 클릭 → waypoint 가 순서대로 쌓이고
    RViz 에 경로(/waypoint_ui/path)가 표시된다.
 2. 창에서 제어기(Stanley / Pure Pursuit)·속도를 고르고 [추종 시작].
 3. [정지] 는 실행 중 goal 을 취소한다(정지 시퀀스는 서버가 수행).

스레드 규율: ROS 콜백은 시그널 emit 만, 위젯 접근은 메인스레드 슬롯에서만.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

from dss_motion_interfaces.action import MotionPurePursuit, MotionStanley

from dss_motion_control import geometry as geo
from dss_motion_control.tracking_server import SpeedEstimator

# 측위 스택 기동은 slam_manager_3d 의 검증된 관리자를 재사용 (세션 트리 한정 정지)
from slam_manager_3d.slam_manager_3d_node import StackProcessManager, find_workspace
from slam_manager_3d.stacks import STACKS as SLAM_STACKS


def densify(pts, spacing=2.0):
    """웨이포인트 열을 세그먼트별 spacing 간격으로 보간 (추종기 입력용)."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(dist // spacing))
        out.extend((a[0] + (b[0] - a[0]) * i / n,
                    a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1))
    return out


class WaypointUI(QtWidgets.QMainWindow):
    """waypoint 목록·추종 제어 창."""

    clicked_signal = pyqtSignal(float, float)
    feedback_signal = pyqtSignal(float, float, float, float)
    result_signal = pyqtSignal(int)

    # slam_manager_3d 와 동일 색 체계
    STYLE_GO = 'background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;'
    STYLE_RUN = 'background-color: #2196F3; color: white; font-weight: bold; padding: 6px;'
    STYLE_STOP = 'background-color: #F44336; color: white; font-weight: bold; padding: 6px;'
    STYLE_EDIT = 'background-color: #FF9800; color: white; font-weight: bold; padding: 6px;'

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.waypoints = []
        self.goal_handle = None

        self.setWindowTitle('DSS Waypoint Navigator')
        self.resize(780, 560)
        central = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(central)

        # ---- 측위 (localization) 선택·구동 ----
        grp_loc = QtWidgets.QGroupBox('측위 (localization)')
        row_loc = QtWidgets.QHBoxLayout(grp_loc)
        self.cmbLocStack = QtWidgets.QComboBox()
        self.cmbLocStack.addItems(['rtab', 'hdl'])
        self.btnLocStart = QtWidgets.QPushButton('측위 시작')
        self.btnLocStop = QtWidgets.QPushButton('측위 정지')
        self.btnLocStart.clicked.connect(self.on_loc_start)
        self.btnLocStop.clicked.connect(self.on_loc_stop)
        self.btnLocStart.setStyleSheet(self.STYLE_GO)
        self.btnLocStop.setStyleSheet(self.STYLE_STOP)
        self.lblLocStatus = QtWidgets.QLabel('TF: —')
        row_loc.addWidget(self.cmbLocStack)
        row_loc.addWidget(self.btnLocStart)
        row_loc.addWidget(self.btnLocStop)
        row_loc.addWidget(self.lblLocStatus, 1)
        lay.addWidget(grp_loc)

        lay.addWidget(QtWidgets.QLabel(
            'RViz "Publish Point" 도구로 지도를 클릭하면 waypoint 가 추가됩니다.\n'
            '(추가 시점의 속도 값이 그 waypoint 목표 속도 — 셀 편집 가능)'))

        self.table = QtWidgets.QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(
            ['X (m)', 'Y (m)', 'Yaw (°)', 'UTM E', 'UTM N', 'UTM Yaw (°)',
             '속도 (m/s)'])
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self._updating = False   # 프로그램적 셀 갱신 중 itemChanged 무시
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self._table_menu)
        self.table.itemChanged.connect(self._on_cell_edited)
        lay.addWidget(self.table, 1)

        # ---- 차량 상태: x, y, yaw, speed | utm_x, utm_y, utm_yaw ----
        grp_state = QtWidgets.QGroupBox('차량 상태')
        col_state = QtWidgets.QVBoxLayout(grp_state)
        self.lblMapPose = QtWidgets.QLabel('x: — | y: — | yaw: — | speed: —')
        self.lblUtmPose = QtWidgets.QLabel('utm_x: — | utm_y: — | utm_yaw: —')
        col_state.addWidget(self.lblMapPose)
        col_state.addWidget(self.lblUtmPose)
        lay.addWidget(grp_state)

        row = QtWidgets.QHBoxLayout()
        self.cmbController = QtWidgets.QComboBox()
        self.cmbController.addItems(['pure_pursuit', 'stanley'])
        self.spinSpeed = QtWidgets.QDoubleSpinBox()
        self.spinSpeed.setRange(0.5, 6.0)
        self.spinSpeed.setSingleStep(0.5)
        self.spinSpeed.setValue(2.5)
        self.spinSpeed.setSuffix(' m/s')
        row.addWidget(QtWidgets.QLabel('제어기'))
        row.addWidget(self.cmbController)
        row.addWidget(QtWidgets.QLabel('속도'))
        row.addWidget(self.spinSpeed)
        lay.addLayout(row)

        btns = QtWidgets.QHBoxLayout()
        self.btnAddHere = QtWidgets.QPushButton('현재 위치 등록')
        self.btnStart = QtWidgets.QPushButton('추종 시작')
        self.btnStop = QtWidgets.QPushButton('정지')
        self.btnUndo = QtWidgets.QPushButton('마지막 삭제')
        self.btnClear = QtWidgets.QPushButton('전체 삭제')
        self.btnAddHere.clicked.connect(self.on_add_here)
        self.btnStart.clicked.connect(self.on_start)
        self.btnStop.clicked.connect(self.on_stop)
        self.btnUndo.clicked.connect(self.on_undo)
        self.btnClear.clicked.connect(self.on_clear)
        self.btnAddHere.setStyleSheet(self.STYLE_RUN)
        self.btnStart.setStyleSheet(self.STYLE_GO)
        self.btnStop.setStyleSheet(self.STYLE_STOP)
        self.btnUndo.setStyleSheet(self.STYLE_EDIT)
        self.btnClear.setStyleSheet(self.STYLE_EDIT)
        for b in (self.btnAddHere, self.btnStart, self.btnStop, self.btnUndo,
                  self.btnClear):
            btns.addWidget(b)
        lay.addLayout(btns)

        self.lblStatus = QtWidgets.QLabel('대기 — waypoint 를 찍어주세요')
        self.lblStatus.setWordWrap(True)
        lay.addWidget(self.lblStatus)
        self.setCentralWidget(central)

        self.btnStop.setEnabled(False)

        # 측위 프로세스 관리자 (slam_manager_3d 재사용)
        try:
            self.loc_manager = StackProcessManager(find_workspace())
        except Exception as e:
            self.loc_manager = None
            self.lblLocStatus.setText(f'관리자 초기화 실패: {e}')

        # ROS 결선
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, node)
        self.path_pub = node.create_publisher(Path, '/waypoint_ui/path', 1)
        node.create_subscription(PointStamped, '/clicked_point',
                                 self.on_clicked, 10)
        # GPS front/rear (UTM·yaw 표시)
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import NavSatFix
        self._gps = {'front': None, 'rear': None}
        node.create_subscription(
            NavSatFix, '/dss/sensor/gps_front/fix',
            lambda m: self._gps.__setitem__('front', (m.latitude, m.longitude)),
            qos_profile_sensor_data)
        node.create_subscription(
            NavSatFix, '/dss/sensor/gps_rear/fix',
            lambda m: self._gps.__setitem__('rear', (m.latitude, m.longitude)),
            qos_profile_sensor_data)
        self._utm = None   # pyproj Transformer (첫 fix 의 경도로 zone 결정)
        self._speed_est = SpeedEstimator(window=3)
        # map→UTM 유사변환 대응점 (mx, my, ux, uy) — 주행하며 수집(기선 확보)
        self._sim_samples = []
        self.stanley_client = ActionClient(node, MotionStanley, 'motion_stanley')
        self.pp_client = ActionClient(node, MotionPurePursuit,
                                      'motion_pure_pursuit')

        self.clicked_signal.connect(self._add_waypoint)
        self.feedback_signal.connect(self.on_feedback)
        self.result_signal.connect(self.on_result)

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(1000)

    # ---- 측위 구동 ----

    def _loc_key(self):
        return f'{self.cmbLocStack.currentText()}_loc'

    def on_loc_start(self):
        if self.loc_manager is None:
            return
        stack = self.cmbLocStack.currentText()
        cfg = SLAM_STACKS[stack]
        map_path = cfg['map_hint'].format(ws=self.loc_manager.ws).split(' ')[0]
        ok = self.loc_manager.start(self._loc_key(), cfg['loc'],
                                    {'map': map_path})
        self.lblLocStatus.setText(
            f'{stack} 측위 {"기동" if ok else "기동 실패(이미 실행?)"}')

    def on_loc_stop(self):
        if self.loc_manager is None:
            return
        self.loc_manager.stop(self._loc_key())
        self.lblLocStatus.setText('측위 정지')

    def _update_status(self):
        tf_ok = self._pose() is not None
        run = ''
        if self.loc_manager is not None:
            running = [k for k in self.loc_manager.KEYS
                       if k.endswith('_loc') and self.loc_manager.is_running(k)]
            run = f' | 실행: {",".join(running)}' if running else ''
        self.lblLocStatus.setText(('TF: 정상' if tf_ok else 'TF: 없음') + run)
        self._update_vehicle_state()

    def _update_vehicle_state(self):
        """1 s 타이머 — map 포즈(x·y·yaw·speed) + GPS UTM(utm_x·utm_y·utm_yaw)."""
        pose = self._pose_full()
        if pose is not None:
            x, y, yaw = pose
            t = self.node.get_clock().now().nanoseconds * 1e-9
            v = self._speed_est.update(x, y, t)
            self.lblMapPose.setText(
                f'x: {x:.2f} | y: {y:.2f} | yaw: {math.degrees(yaw):+.1f}° | '
                f'speed: {v:.2f} m/s')
        front, rear = self._gps['front'], self._gps['rear']
        if front is None:
            return
        try:
            if self._utm is None:
                from pyproj import Transformer
                zone = int((front[1] + 180.0) // 6.0) + 1
                hemi = 'north' if front[0] >= 0 else 'south'
                self._utm = Transformer.from_crs(
                    'EPSG:4326',
                    f'+proj=utm +zone={zone} +{hemi} +datum=WGS84',
                    always_xy=True)
            fe, fn = self._utm.transform(front[1], front[0])
            utm_yaw = '—'
            if rear is not None:
                re_, rn = self._utm.transform(rear[1], rear[0])
                utm_yaw = f'{math.degrees(math.atan2(fn - rn, fe - re_)):+.1f}°'
            self.lblUtmPose.setText(
                f'utm_x: {fe:.2f} | utm_y: {fn:.2f} | utm_yaw: {utm_yaw}')
            # map↔UTM 대응점 수집 — 직전 표본에서 1 m 이상 이동 시
            if pose is not None:
                if (not self._sim_samples or math.hypot(
                        pose[0] - self._sim_samples[-1][0],
                        pose[1] - self._sim_samples[-1][1]) > 1.0):
                    self._sim_samples.append((pose[0], pose[1], fe, fn))
                    if len(self._sim_samples) > 100:
                        del self._sim_samples[1]   # 첫 표본(기선 앵커)은 유지
                    self._refresh_derived_cells()
        except Exception as e:
            self.lblUtmPose.setText(f'UTM 변환 실패: {e}')

    # ---- ROS 콜백 (시그널만) ----

    def on_clicked(self, msg):
        self.clicked_signal.emit(msg.point.x, msg.point.y)

    # ---- 메인스레드 슬롯 ----

    def _add_waypoint(self, x, y):
        v = float(self.spinSpeed.value())
        self.waypoints.append((x, y))
        r = self.table.rowCount()
        self.table.insertRow(r)
        self._updating = True
        self.table.setItem(r, 0, QtWidgets.QTableWidgetItem(f'{x:.2f}'))
        self.table.setItem(r, 1, QtWidgets.QTableWidgetItem(f'{y:.2f}'))
        self.table.setItem(r, 6, QtWidgets.QTableWidgetItem(f'{v:.1f}'))
        self._updating = False
        self._refresh_derived_cells()
        self._publish_path_marker()
        self.lblStatus.setText(f'waypoint {len(self.waypoints)}개')

    def _row_speed(self, row, fallback):
        try:
            return max(0.3, float(self.table.item(row, 6).text()))
        except (TypeError, ValueError):
            return fallback

    def _map_utm_transform(self):
        """수집 대응점의 최장 기선(≥2 m)으로 map→UTM 유사변환 (미확보 None)."""
        if len(self._sim_samples) < 2:
            return None
        p0 = self._sim_samples[0]
        p1 = self._sim_samples[-1]
        if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 2.0:
            return None    # 기선 2 m 미만 — 축척·회전 추정 불가
        return geo.similarity_2pt(p0[:2], p1[:2], p0[2:], p1[2:])

    def _wp_yaw(self, idx):
        """waypoint idx 진행 방향(rad) — 이전 점(첫 점은 차량 위치) 기준."""
        if idx == 0:
            prev = self._pose()
            if prev is None:
                return None
        else:
            prev = self.waypoints[idx - 1]
        wp = self.waypoints[idx]
        dx, dy = wp[0] - prev[0], wp[1] - prev[1]
        if math.hypot(dx, dy) < 1e-6:
            return None
        return math.atan2(dy, dx)

    def _refresh_derived_cells(self):
        """Yaw·UTM E/N·UTM Yaw 파생 컬럼 재계산 — 추가/삭제/변환 개선 시."""
        self._updating = True
        t = self._map_utm_transform()
        for r, (x, y) in enumerate(self.waypoints):
            yaw = self._wp_yaw(r)
            self.table.setItem(r, 2, QtWidgets.QTableWidgetItem(
                f'{math.degrees(yaw):+.1f}' if yaw is not None else '—'))
            if t is None:
                cells = ('—', '—', '—')
            else:
                ux, uy = geo.apply_similarity(t, x, y)
                uyaw = ('—' if yaw is None else
                        f'{math.degrees(geo.normalize_angle(yaw + t[1])):+.1f}')
                cells = (f'{ux:.2f}', f'{uy:.2f}', uyaw)
            for c, text in zip((3, 4, 5), cells):
                self.table.setItem(r, c, QtWidgets.QTableWidgetItem(text))
        self._updating = False

    def _publish_path_marker(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x, y in self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _table_menu(self, pos):
        """테이블 우클릭 메뉴 — 행 추가(복제)·위/아래 이동·행 삭제."""
        r = self.table.rowAt(pos.y())
        valid = 0 <= r < len(self.waypoints)
        menu = QtWidgets.QMenu(self)
        a_add = menu.addAction('행 추가 (이 행 복제)')
        a_up = menu.addAction('위로 이동')
        a_down = menu.addAction('아래로 이동')
        a_del = menu.addAction('행 삭제')
        a_add.setEnabled(valid)
        a_up.setEnabled(valid and r > 0)
        a_down.setEnabled(valid and r < len(self.waypoints) - 1)
        a_del.setEnabled(valid)
        act = menu.exec_(self.table.viewport().mapToGlobal(pos))
        if act is None or not valid:
            return
        if act == a_add:
            self._row_insert(r)
        elif act == a_up:
            self._row_swap(r, r - 1)
        elif act == a_down:
            self._row_swap(r, r + 1)
        elif act == a_del:
            self._row_delete(r)

    def _cell_text(self, row, col):
        it = self.table.item(row, col)
        return it.text() if it else ''

    def _row_insert(self, r):
        self.waypoints.insert(r + 1, self.waypoints[r])
        self._updating = True
        self.table.insertRow(r + 1)
        for c in (0, 1, 6):
            self.table.setItem(
                r + 1, c, QtWidgets.QTableWidgetItem(self._cell_text(r, c)))
        self._updating = False
        self.table.selectRow(r + 1)
        self._after_model_change()

    def _row_swap(self, a, b):
        self.waypoints[a], self.waypoints[b] = self.waypoints[b], self.waypoints[a]
        self._updating = True
        for c in (0, 1, 6):
            ta, tb = self._cell_text(a, c), self._cell_text(b, c)
            self.table.setItem(a, c, QtWidgets.QTableWidgetItem(tb))
            self.table.setItem(b, c, QtWidgets.QTableWidgetItem(ta))
        self._updating = False
        self.table.selectRow(b)
        self._after_model_change()

    def _row_delete(self, r):
        self.waypoints.pop(r)
        self.table.removeRow(r)
        self._after_model_change()

    def _after_model_change(self):
        self._refresh_derived_cells()
        self._publish_path_marker()
        self.lblStatus.setText(f'waypoint {len(self.waypoints)}개')

    def _on_cell_edited(self, item):
        """X/Y 셀 직접 편집 → waypoints 모델 동기화 (숫자 아니면 되돌림)."""
        if self._updating or item.column() not in (0, 1):
            return
        r = item.row()
        if not (0 <= r < len(self.waypoints)):
            return
        try:
            v = float(item.text())
        except ValueError:
            self._updating = True
            item.setText(f'{self.waypoints[r][item.column()]:.2f}')
            self._updating = False
            return
        x, y = self.waypoints[r]
        self.waypoints[r] = (v, y) if item.column() == 0 else (x, v)
        self._after_model_change()

    def on_add_here(self):
        """현재 차량 위치(TF)를 waypoint 로 등록 — 차를 움직여 가며 지점 지정."""
        cur = self._pose()
        if cur is None:
            QtWidgets.QMessageBox.warning(
                self, '오류', '현재 포즈(TF) 없음 — 측위를 확인하세요.')
            return
        self._add_waypoint(cur[0], cur[1])

    def on_undo(self):
        if self.waypoints:
            self.waypoints.pop()
            self.table.removeRow(self.table.rowCount() - 1)
            self._refresh_derived_cells()
            self._publish_path_marker()
            self.lblStatus.setText(f'waypoint {len(self.waypoints)}개')

    def on_clear(self):
        self.waypoints.clear()
        self.table.setRowCount(0)
        self._publish_path_marker()
        self.lblStatus.setText('waypoint 비움')

    def _pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def _pose_full(self):
        """map→base_link 포즈 (x, y, yaw[rad]) — 상태 표시용."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (tf.transform.translation.x, tf.transform.translation.y, yaw)
        except Exception:
            return None

    def on_start(self):
        if not self.waypoints:
            QtWidgets.QMessageBox.warning(self, '오류', 'waypoint 가 없습니다.')
            return
        cur = self._pose()
        if cur is None:
            QtWidgets.QMessageBox.warning(self, '오류',
                                          '현재 포즈(TF) 없음 — 측위를 확인하세요.')
            return

        # 레그별 densify — 보간점은 그 레그의 도착 waypoint 속도를 물려받는다
        default_v = float(self.spinSpeed.value())
        leg_speeds = [self._row_speed(i, default_v)
                      for i in range(len(self.waypoints))]
        pts = [cur]
        speeds = [leg_speeds[0]]
        prev = cur
        for wp, v in zip(self.waypoints, leg_speeds):
            seg = densify([prev, wp], 2.0)[1:]
            pts.extend(seg)
            speeds.extend([v] * len(seg))
            prev = wp
        speeds[-1] = 0.0   # 마지막 지점은 정지

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

        controller = self.cmbController.currentText()
        if controller == 'stanley':
            goal = MotionStanley.Goal()
            client = self.stanley_client
        else:
            goal = MotionPurePursuit.Goal()
            goal.lookahead_distance = 0.0
            client = self.pp_client
        goal.path = path
        goal.point_speeds = speeds
        goal.max_linear_speed = max(leg_speeds)
        goal.acceleration = 1.0
        goal.goal_tolerance = 0.0

        if not client.wait_for_server(timeout_sec=2.0):
            QtWidgets.QMessageBox.warning(self, '오류',
                                          '제어 노드(dss_motion_control) 미기동')
            return

        total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(pts, pts[1:]))
        self.lblStatus.setText(
            f'{controller} 추종 시작 — {len(self.waypoints)} wpt, {total:.1f} m')
        self.btnStart.setEnabled(False)
        self.btnStart.setStyleSheet(self.STYLE_RUN)
        self.btnStop.setEnabled(True)

        def _fb(msg):
            fb = msg.feedback
            self.feedback_signal.emit(fb.remaining_distance,
                                      fb.cross_track_error,
                                      fb.current_speed, fb.steer_command)

        send = client.send_goal_async(goal, feedback_callback=_fb)

        def _on_gh(fut):
            gh = fut.result()
            if gh is None or not gh.accepted:
                self.result_signal.emit(-2)
                return
            self.goal_handle = gh
            gh.get_result_async().add_done_callback(
                lambda rf: self.result_signal.emit(rf.result().result.status))
        send.add_done_callback(_on_gh)

    def on_stop(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.lblStatus.setText('취소 요청 전송')

    def on_feedback(self, remaining, cte, speed, steer):
        self.lblStatus.setText(
            f'잔여 {remaining:.1f} m | CTE {cte:+.2f} m | '
            f'v {speed:.1f} m/s | steer {steer:+.2f}')

    def on_result(self, status):
        names = {0: '도달 성공', -1: '취소됨', -2: '거부/무효',
                 -3: '타임아웃/측위상실', -4: '안전 정지(충돌 의심)'}
        self.lblStatus.setText(f'종료: {names.get(status, status)}')
        self.btnStart.setEnabled(True)
        self.btnStart.setStyleSheet(self.STYLE_GO)
        self.btnStop.setEnabled(False)
        self.goal_handle = None

    def closeEvent(self, event):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.loc_manager is not None:
            self.loc_manager.stop_all()   # 이 UI 가 띄운 측위만 (자기 트리 한정)
        event.accept()


def main(args=None):
    import sys
    rclpy.init(args=args)
    node = Node('waypoint_ui')

    app = QtWidgets.QApplication(sys.argv)
    ui = WaypointUI(node)
    ui.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
