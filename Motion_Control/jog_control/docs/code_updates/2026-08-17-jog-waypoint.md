# 2026-08-17 — jog GUI waypoint 등록·저장·불러오기 ⟨21:17 전량 철회 — 말미 §철회 참조⟩

- 결정: [ADR 2026-08-17-jog-waypoint](../adr/2026-08-17-jog-waypoint.md) — 사용자 지시(20:58 "현재 위치 waypoint 등록 버튼 + 파일 저장·읽어오기"). 세션 8a436cd4
- 배경: 자율 스택(dss_motion_control)의 경로 소비 형식이 map 프레임 (x, y) m + TF `map→base_link` 측위(tracking_server.py:143·181, send_goal.py:48) — GUI 도 같은 소스·프레임으로 기록해야 파일이 그대로 재사용된다. 기존 waypoint 파일 형식은 부재 → CSV(Comma-Separated Values) 신규 정의.

## 변경

| 대상 | 내용 |
| --- | --- |
| src/Motion_Control/jog_control/src/JogControlNode.cpp | WP 행 신설(등록·저장·불러오기·비우기 + 상태줄) — **controls_ 밖 배치로 jog enable 과 무관하게 상시 활성**. TF Buffer+TransformListener(자체 스레드 spin — GUI 는 executor 미구동). captureWaypoint(TF 부재 시 "측위 없음" 표시만)·saveWaypoints(QFileDialog→CSV, `#` 헤더+`x,y` 3자리)·loadWaypoints(파싱: #·빈 줄 무시, 오류 줄 스킵, 목록 통째 교체)·clearWaypoints·updateWaypointStatus |
| package.xml · CMakeLists.txt | tf2_ros·geometry_msgs 의존 추가 |
| README.md | Waypoint 도구 절 추가 |
| 함수표 | [2026-08-17.md](../code_review/jog_control/2026-08-17.md) 에 #6g~6k·상태 7~9 반영 + 전 함수 줄 번호 재갱신 — 모듈 로컬 + 루트 이중 기록 |

## 검증 (sim, 2026-08-17 21:02~21:04 — 격리 ROS_DOMAIN_ID=42)

| 항목 | 결과 |
| --- | --- |
| colcon build (jog_control) | 성공 6.2 s, 경고 0 · clang-format 통과 |
| TF 부재 등록 | "측위 없음 (TF map→base_link) — 측위 스택 기동 필요" 표시, 목록 불변 ✓ (`experiments/capture/20260817_210249_jog-wp-no-tf.png`) |
| 등록 | 정적 TF(12.3, -4.5) 발행 후 2회 클릭 → "WP 2개 · 마지막 (12.30, -4.50)" ✓ (`20260817_210307_jog-wp-registered.png`) |
| 저장 | CSV 생성 — `#` 헤더 + `12.300,-4.500` 2줄 실측 ✓ |
| 비우기·불러오기 | "WP 0개" → 같은 CSV 로드 → "불러옴 waypoints_test.csv — WP 2개 · 마지막 (12.30, -4.50)" ✓ (`20260817_210335/210342` 캡처) |
| jog 회귀 | enable ON 19.99 Hz · 창 닫기 → 프로세스 정상 종료 ✓ |
| 기동 규율 | 격리 도메인 사용, 시험 후 정적 TF 발행자·GUI 전부 정리, 라이브 그래프 불간섭 |

## 미수행 (정직 선언)

- ~~실측위(SLAM/localization) 스택과의 연동 등록 — 정적 TF 로만 검증(sim)~~ → **완료(sim, 2026-08-17 21:13~21:14, DSS 리셋 후 라이브)**: 라이브 TF 로 WP 등록(229.63, -0.67) → jog ON 전진 4 s 실주행(차량 실이동) → WP 재등록(234.14, -1.11 — 약 4.5 m 전진 반영) → OFF 자율 복귀 → `experiments/waypoints/20260817_jog.csv` 저장 실측(캡처 4장 `20260817_2113*·2114*`). 라이브 수신측은 이번 세션 빌드 mux(jog·enable 구독 확인).
- 저장 waypoint 로 경로 추종 발진하는 기능 — 범위 외(ADR D6), 필요 시 후속 작업(2026-08-17 기준).

## 철회 (2026-08-17 21:17~21:25)

- 사용자 정정: 20:58 waypoint 지시는 **타 세션(waypoint manager 작업)에 보낼 것이 이 세션에 오입력**된 것 — jog GUI 는 대상 아님. waypoint 는 기존 `src/Motion_Control/dss_motion_control/dss_motion_control/waypoint_ui.py` 소관.
- 원위치 내역: JogControlNode.cpp waypoint 코드 전량 제거(mux+종료수정 상태로 복원, waypoint·tf2 잔존 0 확인) · package.xml/CMakeLists 의 tf2_ros·geometry_msgs 제거 · README waypoint 절 제거 · 함수표 waypoint 행 제거 · ADR jog-waypoint → Superseded.
- 검증(sim, 21:24): 재빌드 성공 · 원위치 GUI 기동 캡처(`experiments/capture/20260817_212456_jog-final-clean.png` — waypoint UI 부재, 수신 연결 1) · 구 프로세스 2개(교체 전 바이너리 잔존분 포함) 정리.
- 산출물 잔존: `experiments/waypoints/20260817_jog.csv` (map 프레임 실측 waypoint 2점) — 코드와 무관한 데이터 파일이라 보존. waypoint manager 에서 재사용 가능.
- 실수 기록: docs/claude-mistake/2026-08-17-007.md (기존 waypoint_ui 미조사).
