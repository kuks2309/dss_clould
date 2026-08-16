# 2026-08-16 — jog_control 신설 + dss_ros2_bridge 제어 수신 노드(DSSControlNode)

- 결정: [ADR 2026-08-16-jog-control](../adr/2026-08-16-jog-control.md) — 사용자 승인(GUI 시안 A '버튼 조그 패드' + ROS2 토픽 경유)

## 변경

| 대상 | 내용 |
| --- | --- |
| src/Motion_Control/jog_control (신규 패키지) | Qt5 jog GUI `JogControlNode` — `/dss/control` (DssControl) 20 Hz reliable 상시 발행, 버튼·키 dead-man(떼면 0), Throttle 상한·Steer 강도 슬라이더. package.xml · CMakeLists.txt · src/JogControlNode.cpp · README.md |
| src/dss_ros2_bridge/src/DSSControlNode.cpp (신규) | `/dss/control` 구독(reliable 10) → `setDriveControl()` → UDP :8886. 50 ms 주기 송신 + 500 ms 수신 두절 시 0 명령(watchdog). main 은 `runBridgeNode` 재사용(중복 main 미신설) |
| src/dss_ros2_bridge/CMakeLists.txt | `DSSControlNode` 타깃(add_dss_node) + install 항목 추가 |
| 함수표 | jog_control 신규 인벤토리 + 브리지 delta(#36~38) — 모듈 로컬 + 루트 이중 기록 |
| 부채 | debt-008(targetGear 인코딩 미확인 — 후진 미구현) · debt-009(steer 부호 미검증) 등록 |

## 검증 (2026-08-16)

| 항목 | 결과 |
| --- | --- |
| colcon build (dss_ros2_bridge + jog_control) | 성공 15.4 s, 경고 0 |
| clang-format (Microsoft/Allman) | 신규 2파일 통과 |
| `/dss/control` 발행 실측 | 20.035 Hz, 대기 상태 값 steer/throttle/brake 전부 0.0 |
| pub↔sub QoS | 양측 RELIABLE·VOLATILE — 1:1 연결 확인 (`ros2 topic info -v`) |
| DSSControlNode 기동 | NATS `100.80.80.15:4222` 연결 · UDP Ready `:8886` · 전달 시작 로그 정상 |
| GUI 시각 검증 | 캡처 `experiments/capture/20260816_104731_jog-control-gui.png` — 승인 시안 A 와 일치 |
| 기동 규율(ros2-coding §5) | 사전 점검: `/dss/control` 기존 발행자 0 · DSSDemoNode 미기동 확인 후 시험 |

## 미수행 (정직 선언)

- ~~실차(시뮬레이터) 주행 jog — 버튼 조작 → 차량 이동의 실측 확인 (2026-08-16 기준)~~ → **완료(sim, 2026-08-16 11:09, 사용자 확인 — issues_and_fixes.md entry 참조)**
- steer 부호 실측 (debt-009) · 후진 기어 (debt-008) — 미수행(2026-08-16 기준)
- 브리지 플로우차트(2026-08-15-flow.drawio)에 DSSControlNode 반영 — drawio 2단 검증 루프 필요로 이번 턴 미수행

## 주의

- **DSSDemoNode 와 동시 기동 금지** — 데모가 0 명령을 상시 UDP 송신해 제어가 경합한다.

---

## 보강 (2026-08-16 10:51~11:04) — 수신 노드 버튼 기동

사용자 지시 변경: launch 일괄 기동안 대신 **GUI 가 수신측 실행 여부를 감지해 미실행이면 버튼으로 기동**.

- `JogControlNode.cpp` 에 추가: `closeEvent`(자식 정리) · `updateBridgeButton`(구독자 수 기반 3상태 버튼) · `startBridgeNode`(ament prefix 경로 조회 → QProcess 기동, 로그 터미널 전달) · `stopBridgeNode`(내가 띄운 것만 terminate→kill). 의존성 `ament_index_cpp` 추가(package.xml·CMakeLists).
- 검증(sim, 2026-08-16 11:04): 빌드 성공 → 버튼 클릭(xdotool)으로 DSSControlNode 실기동 확인 → 버튼 "● 수신 연결됨" 전환·구독 1:1 캡처 확인 (`experiments/capture/20260816_105627_jog-gui-connected.png`).

## 보강 2 (2026-08-16 11:10~11:20) — 후진(target_gear) 지원

사용자 질문 "후진이 왜 없을까" → GPS 이동 벡터 실험(sim, 11:11)으로 인코딩 확정 후 구현.

- 실험 결과: 기어 없음/1/2 = 전진, **targetGear=-1 = 후진**(기준 전진과 정확히 반대 벡터), 음수 스로틀 = 무시. 실험 스크립트는 세션 스크래치(dss_reverse_probe.py). 주의: 실험 중 구 수신 노드가 0 명령을 겹쳐 보냈으나 방향 판정은 대칭·일관(크기만 감소).
- 변경: `msg/DssControl.msg` + `float32 target_gear` · `DSS.VSSClient` setDriveControl 4번째 인자 `targetGear=0.0F`(기본값 — 기존 콜사이트·패킷 무변경) · `DSSControlNode` gear 전달 · GUI ▼브레이크 → **▼후진**(↓/S), ■정지 = 누르는 동안 전 해제+풀 브레이크(Space), 상태줄 기어(D/R) 표시.
- 검증: colcon build 2패키지 성공(18.0 s, 경고 0) · clang-format 통과 · GUI 신규 레이아웃 캡처 확인(`experiments/capture/20260816_111540_jog-gui-reverse.png`). **후진 실주행은 사용자 확인 대기(2026-08-16 기준)**.
- debt-008 부분 상환(targetGear 확정) — 잔존: parkBrake·P/N 인코딩.

## 이슈 진단 기록 (2026-08-16 11:00경) — "버튼 눌러도 차량 안 움직임"

- 판별: 검증 스크립트(전 필드 명시)·C++ 모방 패킷(0 필드 생략) **모두 차량 이동**(GPS 실측) → 인코딩·체인 결함 아님.
- 원인 판정: 증상 당시 GPS 고도 **-99.99 m** — 직전 주행 검증(10:55~58)으로 차량이 맵 밖 추락 상태였고, 그 상태에선 제어 무효. 사용자 위치 리셋 후 직진 정상.
- 상세는 `docs/issues_and_fixes/issues_and_fixes.md` entry 참조(사용자 최종 확인 후 상태 확정).
