# dss_motion_control 함수표·전역변수표 (as-built 2026-08-17)

> coding.md §2 신규 파일 규정. 근거: docs/adr/2026-08-17-dss-motion-control.md ·
> 참조 리뷰: docs/code_review/amr_motion_control_2wd/2026-08-17.md

## 함수표

### geometry.py — 경로 기하 순수 함수 (단위시험 대상)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| G1 | `normalize_angle` | angle | float | ±π 정규화 | geometry.py:13-19 |
| G10 | `similarity_2pt` | p0,p1,q0,q1 | (s,θ,tx,ty)/None | 두 대응점으로 map→UTM 2D 유사변환 추정(합성 GPS 축척·회전·원점, 기선<1e-6 None) | geometry.py:128-144 |
| G11 | `apply_similarity` | t,x,y | (x,y) | 유사변환 적용 | geometry.py:147-151 |
| G9 | `speed_limit_at` | cumulative,point_speeds,arc,accel,vmax | float | waypoint 별 목표 속도 상한 — 미래 느린 지점을 √(v²+2a·gap) 제동거리로 선반영 + 현 구간 속도 상한 (0 이하 = 제한 없음) | geometry.py:154-172 |
| G2 | `build_path` | poses[(x,y)] | PathData | 웨이포인트·누적 호장·총길이 (2점 미만/0길이 None) | geometry.py:29-41 |
| G3 | `closest_point` | path,x,y | Closest | 전 세그먼트 투영 최근접점·부호 CTE(외적)·호장 | geometry.py:53-82 |
| G4 | `lookahead_point` | path,closest,ld | (x,y) | 호장+Ld 지점 보간(끝 클램프) | geometry.py:85-102 |
| G5 | `straight_path` | x0,y0,x1,y1,spacing | poses | 직선 경로 웨이포인트 생성(D7) | geometry.py:105-110 |
| G6 | `stanley_steer` | e_theta,cte,v,k,k_soft,max_steer | float | δ = eθ + atan2(-k·cte, v+k_soft), 클램프 | geometry.py:113-116 |
| G7 | `pure_pursuit_steer` | alpha,ld,wheelbase,max_steer | float | κ=2sinα/Ld → δ=atan(L·κ), 클램프 | geometry.py:119-125 |
| G8 | `front_axle` | x,y,yaw,wheelbase | (x,y) | 전륜축 투영(Stanley 기준점, D3) | geometry.py:128-130 |

### motion_profile.py — 사다리꼴 프로파일 (원본 #10-12 포팅)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| P1 | `TrapezoidalProfile.__init__` | dist,vmax,acc,vexit | — | 사다리꼴/삼각 판정·피크 산정 | motion_profile.py:11-25 |
| P2 | `TrapezoidalProfile.speed_at` | position | (speed,phase) | 위치 기반 √(2as) 속도·구간 | motion_profile.py:27-39 |

### watchdog.py — 측위 워치독 (원본 #14-18 포팅)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| W1 | `LocalizationWatchdog.__init__` | timeout,jump_thr,vel_margin | — | 설정·기준시각 | watchdog.py:12-20 |
| W2 | `LocalizationWatchdog.update_pose` | x,y,yaw,now | — | 포즈 캐시+점프 검출(명령속도×여유) | watchdog.py:22-33 |
| W3 | `LocalizationWatchdog.set_speed` | mps | — | 점프 임계용 명령 속도 | watchdog.py:35-36 |
| W4 | `LocalizationWatchdog.healthy` | now | bool | age>timeout 또는 점프 → False | watchdog.py:38-47 |

### tracking_server.py — 공통 추종 서버 + 제어기 2종 (D9)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| T1 | `SpeedEstimator.update` | x,y,t | float | 포즈 미분+이동평균 속도 추정 | tracking_server.py:31-46 |
| T2 | `DssCommander.__init__` | node | — | /dss/control 발행자·PI 상태 | tracking_server.py:51-58 |
| T3 | `DssCommander.command` | v_cmd,v_est,steer_norm,dt | — | 데드밴드 FF+PI throttle/brake 산출·전진 기어 명시→DssControl 발행 | tracking_server.py:65-90 |
| T4 | `DssCommander.stop_and_release` | v_est_fn | — | brake 유지→정지 후 0 명령 수 회 | tracking_server.py:81-92 |
| T5 | `PathTrackingServer.__init__` | node,name,action_type | — | 액션 서버·TF·파라미터·상호 배제 등록 | tracking_server.py:101-127 |
| T6 | `PathTrackingServer.lookup_pose` | — | (x,y,yaw)/None | TF map→base_link 조회 | tracking_server.py:136-145 |
| T7 | `PathTrackingServer.execute_cb` | goal_handle | Result | 공통 루프: 배제→경로→프로파일→워치독→가드(취소·타임아웃·횡이탈·워치독·**정체/충돌**)→도착→속도→조향(서브클래스)→명령→피드백 | tracking_server.py:155-300 |
| T8 | `PathTrackingServer.steer` | 상태 | float | 추상 — 서브클래스 구현 | tracking_server.py:285-286 |
| T9 | `StanleyServer.steer` | 상태 | float | 전륜축 CTE→G6 | tracking_server.py:289-301 |
| T10 | `PurePursuitServer.steer` | 상태 | float | G4 lookahead→α→G7 | tracking_server.py:304-315 |

### dss_motion_control_node.py — 노드 결선 + 목표점 UX

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| N1 | `GoalPoseBridge.__init__` | node,servers | — | /goal_pose 구독·기본 제어기 파라미터 | dss_motion_control_node.py:31-41 |
| N2 | `GoalPoseBridge.on_goal` | PoseStamped | — | 현재 위치→목표 직선 경로(G5)→액션 self-goal 발행 | dss_motion_control_node.py:50-87 |
| N3 | `main` | args | — | 노드+서버 2종+브리지, MultiThreadedExecutor spin | dss_motion_control_node.py:90-142 |

### send_goal.py — CLI 테스트 클라이언트

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| C1 | `main` | argv(x y [controller]) | exit | 직선 경로 생성→액션 호출→피드백 출력 | send_goal.py:28-130 |
| C1a | `main._cancel_and_exit` | signum,frame | — | SIGINT/SIGTERM 시 goal 취소 전송 후 종료(좀비 액션 방지) | send_goal.py:104-112 |

## 전역 변수 / 모듈 상수

| # | 변수 | 사용처(함수) | 기능 | 위치 |
| --- | --- | --- | --- | --- |
| V1 | `ACTIVE_LOCK` (가변) | T7 | 서버 간 상호 배제(threading.Lock+보유자명) | tracking_server.py:24-25 |

### waypoint_ui.py — waypoint 지정·추종 GUI (RViz 클릭 + Qt 패널)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| U1 | `densify` | pts,spacing | pts | 웨이포인트 열을 세그먼트별 2 m 간격 보간(순수 함수) | waypoint_ui.py:37-47 |
| U2 | `WaypointUI.__init__` | node | — | Qt 창(측위 그룹·테이블 7열[X·Y·Yaw·UTM E·UTM N·UTM Yaw·속도]·차량 상태 그룹·제어기 콤보·속도 스핀·버튼 5)·시그널·/clicked_point·GPS front/rear 구독·SpeedEstimator·Path 마커 발행 | waypoint_ui.py:63-194 |
| U3 | `WaypointUI.on_clicked` | PointStamped | — | RViz 클릭 → waypoint 추가 시그널 emit (스레드 안전) | waypoint_ui.py:266-267 |
| U4 | `WaypointUI._add_waypoint` | x,y | — | 목록·테이블(속도 컬럼 = 현재 스핀 값) 갱신 + RViz 경로 마커 재발행 | waypoint_ui.py:272-282 |
| U4a | `WaypointUI._row_speed` | row,fallback | float | 테이블 속도 셀 파싱(하한 0.3, 실패 시 fallback) | waypoint_ui.py:284-288 |
| U5 | `WaypointUI._publish_path_marker` | 없음 | — | 현재 waypoint 열을 nav_msgs/Path 로 발행(RViz 표시) | waypoint_ui.py:335-346 |
| U6 | `WaypointUI.on_undo` | 없음 | — | 마지막 waypoint 삭제 | waypoint_ui.py:357-363 |
| U7 | `WaypointUI.on_clear` | 없음 | — | 전체 삭제 | waypoint_ui.py:365-369 |
| U8 | `WaypointUI.on_start` | 없음 | — | 현재 위치+waypoint 열→레그별 densify(보간점이 레그 속도 상속)→point_speeds 정렬 배열→선택 제어기 액션 발행 | waypoint_ui.py:389-467 |
| U9 | `WaypointUI.on_stop` | 없음 | — | 실행 중 goal cancel_goal_async | waypoint_ui.py:469-472 |
| U10 | `WaypointUI.on_feedback` | feedback | — | 잔여·CTE·속도·조향 상태 라벨 갱신 슬롯 | waypoint_ui.py:474-477 |
| U11 | `WaypointUI.on_result` | future | — | 종료 status 라벨·버튼 복구 | waypoint_ui.py:479-486 |
| U12 | `waypoint_ui.main` | args | — | rclpy+Qt+spin QTimer(10 ms) 결선 | waypoint_ui.py:496-512 |
| U13 | `WaypointUI._loc_key` | 없음 | str | 선택 스택의 프로세스 키('rtab_loc'/'hdl_loc') | waypoint_ui.py:193-194 |
| U14 | `WaypointUI.on_loc_start` | 없음 | — | 선택 측위 스택 기동(slam_manager_3d StackProcessManager 재사용, 기본 지도 경로) | waypoint_ui.py:196-205 |
| U15 | `WaypointUI.on_loc_stop` | 없음 | — | 측위 정지(자기 세션 트리 한정) | waypoint_ui.py:207-211 |
| U16 | `WaypointUI._update_status` | 없음 | — | 1 s 타이머 — TF 유무·측위 프로세스 상태 라벨 + 차량 상태 갱신 호출 | waypoint_ui.py:213-221 |
| U17 | `WaypointUI._update_vehicle_state` | `_gps` 캐시,TF | — | map 포즈 x·y·yaw·speed(SpeedEstimator) + GPS UTM utm_x·utm_y·utm_yaw 라벨 + map↔UTM 대응점 수집(1 m 간격, 최대 100·기선 앵커 유지)→UTM 셀 갱신 | waypoint_ui.py:224-265 |
| U18 | `WaypointUI._pose` | 없음 | (x,y)/None | map→base_link 위치 (waypoint 시작점용) | waypoint_ui.py:371-376 |
| U19 | `WaypointUI._pose_full` | 없음 | (x,y,yaw)/None | map→base_link 포즈+quaternion→yaw (상태 표시용) | waypoint_ui.py:378-387 |
| U20 | `WaypointUI._map_utm_transform` | 없음 | (s,θ,tx,ty)/None | 대응점 최장 기선(≥2 m)으로 G10 map→UTM 유사변환 (미확보 None) | waypoint_ui.py:290-298 |
| U21 | `WaypointUI._refresh_derived_cells` | 없음 | — | 테이블 파생 컬럼(Yaw·UTM E/N·UTM Yaw) 일괄 재계산 — 추가/삭제/변환 개선 시 | waypoint_ui.py:314-329 |
| U22 | `WaypointUI._wp_yaw` | idx | rad/None | waypoint 진행 방향 — 이전 점(첫 점은 차량 위치) 기준 atan2 | waypoint_ui.py:300-312 |
| U23 | `WaypointUI.on_add_here` | 없음 | — | [현재 위치 등록] — 차량 현재 TF 위치를 waypoint 로 등록(측위 없으면 경고) | waypoint_ui.py:435-442 |
| U24 | `WaypointUI._table_menu` | pos | — | 테이블 우클릭 메뉴 — 행 추가(복제)·위/아래 이동·행 삭제(경계 행 비활성) | waypoint_ui.py:356-379 |
| U25 | `WaypointUI._cell_text` | row,col | str | 셀 텍스트 안전 조회 | waypoint_ui.py:381-383 |
| U26 | `WaypointUI._row_insert` | r | — | 선택 행 복제 삽입(모델+셀) | waypoint_ui.py:385-394 |
| U27 | `WaypointUI._row_swap` | a,b | — | 행 순서 교환(모델+편집 셀 3종) 후 선택 추적 | waypoint_ui.py:396-405 |
| U28 | `WaypointUI._row_delete` | r | — | 행 삭제(모델+테이블) | waypoint_ui.py:407-410 |
| U29 | `WaypointUI._after_model_change` | 없음 | — | 행 조작/셀 편집 공통 후처리 — 파생 컬럼·경로 마커·상태 라벨 | waypoint_ui.py:412-415 |
| U30 | `WaypointUI._on_cell_edited` | item | — | X/Y 셀 직접 편집 → waypoints 동기화(비숫자 되돌림, `_updating` 가드) | waypoint_ui.py:417-433 |

### test/test_geometry.py — 순수 로직 단위시험 (pytest, ROS 비의존)

| # | 함수 | 대상 | 기능 | 위치 |
| --- | --- | --- | --- | --- |
| X1 | `test_build_path_rejects_degenerate` | G2 | 퇴화 경로 거부·총길이 | test_geometry.py:11-15 |
| X2 | `test_closest_point_cte_sign` | G3 | 부호 CTE·호장·세그 yaw | test_geometry.py:18-25 |
| X3 | `test_lookahead_interpolation_and_clamp` | G4 | 보간·모서리 통과·끝 클램프 | test_geometry.py:28-36 |
| X4 | `test_stanley_steer_direction` | G6 | 이탈 방향별 조향 부호·클램프 | test_geometry.py:39-48 |
| X5 | `test_pure_pursuit_steer` | G7 | κ=2sinα/Ld 수식·0 보호 | test_geometry.py:51-56 |
| X6 | `test_front_axle_projection` | G8 | 전륜축 투영 | test_geometry.py:59-63 |
| X7 | `test_straight_path_spacing` | G5 | 간격·양 끝 포함 | test_geometry.py:66-69 |
| X8 | `test_trapezoidal_profile_phases` | P2 | 4상 판정 | test_geometry.py:72-81 |
| X9 | `test_trapezoidal_triangular_case` | P1 | 삼각 프로파일 | test_geometry.py:84-88 |
| X12 | `test_similarity_2pt_scale_rotation` | G10·G11 | 축척 2·회전 90° 추정·적용·퇴화 None | test_geometry.py:91-101 |
| X10 | `test_speed_limit_at_braking_and_segment` | G9 | 제동거리 선반영·구간 상한·미지정 폴백 | test_geometry.py:104-117 |
| X11 | `test_watchdog_timeout_and_jump` | W2·W4 | 타임아웃·점프·회복 | test_geometry.py:120-134 |
