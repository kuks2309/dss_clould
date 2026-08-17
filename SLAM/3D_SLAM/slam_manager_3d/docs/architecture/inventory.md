# slam_manager_3d 함수표·전역변수표 (as-built 2026-08-17)

> 계획 단계 설계 표 (coding.md §2 신규 파일 규정). 구현 후 줄 앵커를 as-built 로 갱신한다.
> 근거 설계: docs/adr/2026-08-17-slam-manager-3d.md · 참조 리뷰: docs/code_review/slam_manager_3d/2026-08-17.md

## 함수표

### stacks.py — 스택 구성 단일 정의 (함수 없음, 상수 모듈)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| S0 | (함수 없음 — 구성 상수만) | — | — | 실측 검증 launch·인자·토픽 정의 | stacks.py:1-96 |

### slam_manager_3d_node.py — 프로세스 관리 + ROS 노드

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| N1 | `find_workspace` | 없음 | Path | share 경로 parents[3] + install/setup.bash 실재 검증, 실패 시 RuntimeError | slam_manager_3d_node.py:31-42 |
| N2 | `quaternion_to_euler` | q | (r,p,y) | 쿼터니언→오일러 (원본 #2 동일 수식) | slam_manager_3d_node.py:45-57 |
| N3 | `ProcessHandle.__init__` | pid, script_path | — | PID+스크립트 경로 보관 | slam_manager_3d_node.py:68-70 |
| N4 | `ProcessHandle.alive` | 없음 | bool | 세션 리더 속성(getsid==pid) 판정 — exec 후 cmdline 교체·PID 재사용 오탐 모두 회피 | slam_manager_3d_node.py:72-77 |
| N5 | `StackProcessManager.__init__` | ws | — | 프로세스 dict 6키(스택×매핑/측위), 스크립트 디렉토리 준비 | slam_manager_3d_node.py:84-87 |
| N6 | `StackProcessManager.start` | key, lines, subst | bool | 셸 라인 목록→세션 스크립트(PID 선기록→source→exec)→setsid 기동→실PID 추적 | slam_manager_3d_node.py:89-130 |
| N6a | `StackProcessManager._tree_pids` | root_pid,sid | set | 세션 구성원+자손 BFS 수집(정지 대상 확정) | slam_manager_3d_node.py:132-162 |
| N7 | `StackProcessManager.stop` | key,grace_s | bool | 세션+자손 트리(_tree_pids) 수집→SIGINT 유예→SIGKILL (자기 트리 한정, rviz 자체 세션 이탈 포함) | slam_manager_3d_node.py:164-196 |
| N8 | `StackProcessManager.is_running` | key | bool | ProcessHandle.alive 위임 | slam_manager_3d_node.py:198-200 |
| N9 | `StackProcessManager.stop_all` | 없음 | — | 실행 중 전 키 정지 | slam_manager_3d_node.py:202-205 |
| N10 | `SlamManagerNode.__init__` | 없음 | — | 오돔 4구독(lio/rtab/hdl/hdl_map)+/clock 구독, Qt 시그널 브리지 등록 | slam_manager_3d_node.py:211-229 |
| N11 | `SlamManagerNode._make_odom_cb` | stack_key | callable | 스택별 오돔 콜백 생성(6-DOF 시그널 emit) | slam_manager_3d_node.py:231-236 |
| N12 | `SlamManagerNode._clock_cb` | Clock | — | 클럭 역행(>5s) 감지 → 리셋 시그널 emit | slam_manager_3d_node.py:238-243 |
| N13 | `main` | args | int | rclpy.init→Qt app→UI+node 결선→QTimer(10ms) spin_once→종료 정리 | slam_manager_3d_node.py:246-274 |

### slam_manager_3d_ui.py — PyQt5 GUI

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
| --- | --- | --- | --- | --- | --- |
| U1 | `SlamManager3DUI.__init__` | manager, ws | — | 탭 3+공통 바(클럭·StopAll)+로그, 시그널 연결, 상태 QTimer 500ms | slam_manager_3d_ui.py:33-80 |
| U2 | `SlamManager3DUI._build_tab` | stack_key | QWidget | 매핑/측위/위치 GroupBox 를 스택 구성으로 생성 | slam_manager_3d_ui.py:82-142 |
| U3 | `SlamManager3DUI.log` | msg | — | 타임스탬프 로그 append (메인스레드 슬롯) | slam_manager_3d_ui.py:144-146 |
| U4 | `SlamManager3DUI.on_pose` | key,x,y,z,r,p,yw | — | 해당 탭 6-DOF 라벨 갱신 슬롯 | slam_manager_3d_ui.py:148-157 |
| U5 | `SlamManager3DUI.on_clock` | sec, reset | — | 클럭 라벨 갱신 + 리셋 시 배너·로그 경고 슬롯 | slam_manager_3d_ui.py:159-166 |
| U6 | `SlamManager3DUI._confirm_exclusive` | key | bool | 타 스택 실행 중이면 확인 다이얼로그(D8) | slam_manager_3d_ui.py:168-179 |
| U7 | `SlamManager3DUI.on_start` | key, mode | — | 매핑/측위 기동(지도 경로 검증 포함) — worker 스레드 | slam_manager_3d_ui.py:181-204 |
| U8 | `SlamManager3DUI.on_stop` | key, mode | — | 정지 — worker 스레드 | slam_manager_3d_ui.py:206-213 |
| U9 | `SlamManager3DUI.on_save` | key | — | 스택별 저장(D7: lio 정지+개명 / hdl 서비스 / rtab sqlite backup) — worker | slam_manager_3d_ui.py:215-260 |
| U10 | `SlamManager3DUI.on_browse` | key | — | 지도 파일 선택 다이얼로그 | slam_manager_3d_ui.py:262-269 |
| U11 | `SlamManager3DUI.on_hdl_seed` | 없음 | — | HDL initialpose 원점 시드 + /relocalize 호출 — worker | slam_manager_3d_ui.py:271-289 |
| U12 | `SlamManager3DUI.update_button_states` | 없음 | — | 500ms 타이머 — 6 프로세스 상태→버튼 enable/색 | slam_manager_3d_ui.py:291-305 |
| U13 | `SlamManager3DUI.on_stop_all` | 없음 | — | 확인 후 전체 정지 | slam_manager_3d_ui.py:307-317 |
| U14 | `SlamManager3DUI.closeEvent` | event | — | 확인→전체 정지→종료 | slam_manager_3d_ui.py:319-327 |

## 전역 변수 / 모듈 상수

| # | 변수 | 사용처(함수) | 기능 | 위치 |
| --- | --- | --- | --- | --- |
| G1 | `STACKS` (상수) | N6·U1·U2·U7·U9 | 스택별 launch 라인·토픽·지도 경로 정의 | stacks.py:28-86 |
| G2 | `STATIC_TF_LIDAR` (상수) | STACKS hdl | base_link→lidar_link 정적 TF 명령 | stacks.py:13-15 |
| G3 | `HDL_GLOBAL_NODE` (상수) | STACKS hdl loc | BBS 전역 재위치화 노드 명령(실측 슬라이스) | stacks.py:19-25 |
| G4 | `LIOSAM_RENAME` (상수) | U9 | 종료 자동저장 파일 개명 규약(debt-010) | stacks.py:89-93 |
| G5 | `CLOCK_RESET_THRESHOLD_S` (상수) | N12 | DSS 리셋 판정 역행 폭 | stacks.py:96 |
