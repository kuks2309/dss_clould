# 2026-08-17 — jog enable 토글 + DSSControlNode 제어 소스 mux

- 결정: [ADR 2026-08-17-jog-enable-mux](../adr/2026-08-17-jog-enable-mux.md) — 사용자 지시(20:05 "enable 시 GUI 조정, disable 시 다른 노드 조정"). 세션 8a436cd4
- 배경: `/dss/control` 에 자율 발행자(`dss_motion_control`/tracking_server) 실재 — GUI 만 고치면 enable 시 동일 토픽 경합. 중재를 수신측에 배치.

## 변경

| 대상 | 내용 |
| --- | --- |
| src/Motion_Control/jog_control/src/JogControlNode.cpp | **enable 토글**(checkable·기본 OFF·checked 녹색) 신설. 발행 토픽 `/dss/control` → `/dss/control/jog` 이동. `/dss/jog_enabled`(std_msgs/Bool, reliable+transient_local depth 1) 발행 — 기동 시 OFF latch·상태 변화·정상 종료(closeEvent 에서 false). OFF: 발행 전면 중단 + 조작 위젯(controls_) 일괄 비활성 + 키 무시 + 상태줄 "조그 OFF — 제어권 자율". ON→OFF 전환 시 0 명령 1회(안전 벨트). 신규 함수 makeCommand·onJogToggled·publishEnabled·updateEnableUi |
| src/dss_ros2_bridge/src/DSSControlNode.cpp | **제어 소스 mux 개조** — 구독 3개: `/dss/control`(자율)·`/dss/control/jog`(수동)·`/dss/jog_enabled`(latched). enable=true → jog 명령만, false → auto 명령만 UDP 전달. 소스별 last_cmd/last_rx 분리·각각 500 ms dead-man. jog 활성 중 GUI 사망 시 자율 자동 폴백 없음(0 명령 정지 — 오버라이드 중 무단 자율 전환 방지). 소스 전환 INFO 로그 |
| package.xml · CMakeLists.txt (양 패키지) | std_msgs 의존 추가 |
| src/Motion_Control/jog_control/README.md | enable 토글·mux 구조·기본 OFF·비정상 종료 거동 반영 |
| 함수표 | jog_control [2026-08-17.md](../code_review/jog_control/2026-08-17.md) 신규(2026-08-16 표 대체) + 브리지 delta [2026-08-17-jog-enable-mux.md](../code_review/dss_ros2_bridge/2026-08-17-jog-enable-mux.md) — 모듈 로컬 + 루트 이중 기록 |
| 부채 | debt-011 등록 — 창 닫힘 후 프로세스 잔존(기존 결함, 아래 참조) |

## 검증 (2026-08-17, 격리 ROS_DOMAIN_ID=42 — 라이브 그래프 불간섭)

| 항목 | 결과 |
| --- | --- |
| colcon build (dss_ros2_bridge + jog_control) | 성공 14.5 s, 경고 0 |
| clang-format (Microsoft/Allman) | 수정 2파일 통과 |
| 기동 규율(ros2-coding §5) | 사전 `/dss/control` 발행자 1(dss_motion_control)·구독 0 확인 → 시험은 도메인 42 격리로 수행, 종료 후 라이브 그래프 원상 확인 |
| OFF 계약 | `/dss/jog_enabled` latched false ✓ · `/dss/control/jog` 무발행(4 s echo 타임아웃) ✓ |
| ON 계약 | 토글 클릭 → latched true ✓ · `/dss/control/jog` 19.981 Hz ✓ |
| 조작 값 | ▲ 전진 hold → throttle 0.30 ✓ · 뗌 → 전량 0 ✓ (dead-man) |
| OFF 전환 | 0 명령 1회 후 침묵(6 s 캡처 4건뿐, 마지막 전량 0) ✓ · latched false ✓ |
| mux 소스 전환 | DSSControlNode 기동(NATS·UDP Ready) → enable true 발행 → "제어 소스 전환: jog" 로그 ✓ → false → "auto" 로그 ✓ (명령 입력 전량 0 으로 차량 무영향) |
| GUI 시각 검증 | OFF `experiments/capture/20260817_201654_jog-gui-off.png`(조작부 회색) · ON `20260817_201734_jog-toggle-attempt.png`(녹색 토글·조작부 활성) 캡처 확인 |

## 미수행 (정직 선언)

- 자율(tracking_server) ↔ jog 실주행 전환 시나리오의 **실차(시뮬레이터) 구동 검증** — 라이브 시스템(타 세션 활동 중) 간섭을 피해 격리 도메인 검증까지만 수행. 실주행 전환은 사용자 입회 하 확인 권장.
- steer 부호 실측(debt-009) · parkBrake/P/N 인코딩(debt-008) — 기존 잔존 그대로.

## 발견 이슈 (이번 수정 원인 아님 — debt-011)

- 창을 닫아도 GUI 프로세스가 종료되지 않음. **enable/mux 수정 전 원본 코드로 재현 확인(기존 결함)**. exec() 반환(발행 정지로 확인)까지는 정상, 이후 rclcpp 종료/소멸 단계 hang. 잔존 프로세스는 enable=false 를 이미 latch 한 상태라 제어권은 안 잡음. ~~수정안은 debt-011 상환계획 참조 — 승인 시 별도 구현.~~ → 아래 보강에서 해결(2026-08-17 20:50).

## 보강 (2026-08-17 20:45~20:50) — debt-011 종료 hang 수정

사용자 승인("수정 승인") 후 issue_fix SOP(Standard Operating Procedure) 사이클로 처리.

- **변경**: `src/Motion_Control/jog_control/src/JogControlNode.cpp` main — 창(노드·퍼블리셔 보유)을 블록 스코프로 감싸 `QApplication::exec()` 반환 직후 DDS(Data Distribution Service) 엔티티를 먼저 소멸시킨 뒤 `rclcpp::shutdown()` 호출 (5줄).
- **검증** (sim, 2026-08-17 20:50, 격리 ROS_DOMAIN_ID=42): 빌드 성공·clang-format 통과. ① OFF 상태 Alt+F4 → 4 s 내 프로세스 종료 ② enable ON 상태 닫기 → latched false 발행(구독 이력 false→true→false) 후 종료 ③ 회귀: ON 스트림 20.001 Hz.
- **기록**: issues_and_fixes 2026-08-17 [Fix] entry · debt-011 해결 처리.
