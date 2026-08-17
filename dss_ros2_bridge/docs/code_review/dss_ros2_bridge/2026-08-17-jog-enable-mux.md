# 2026-08-17 (KST) — dss_ros2_bridge 함수표 delta: DSSControlNode 제어 소스 mux 개조 (코딩 lane §6 갱신)

> [2026-08-16-jog-control.md](2026-08-16-jog-control.md) 의 DSSControlNode delta 를 대체하는 현행 표.
> 리뷰 산출물 아님, verdict 없음. Core 인벤토리 정본 [2026-08-15.md](2026-08-15.md) 의 번호 체계(#36~)를 잇는다.
> 결정 근거: [ADR 2026-08-17-jog-enable-mux](../../adr/2026-08-17-jog-enable-mux.md)

### 3. 함수 리스트 (delta — src/DSSControlNode.cpp)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
|---|------|------|------|------|------|
| 36 | `DSSControlNode::DSSControlNode` | 없음 | — | VSS `start(게이트웨이, 8886, 4222)` 실패 시 throw → 구독 3개(auto·jog·enable) → 50ms 송신 타이머 | src/DSSControlNode.cpp:19 |
| 36a | `DSSControlNode.auto구독람다` (inner) | `DssControl::SharedPtr` | void | `last_auto_cmd_`·`last_auto_rx_` 갱신 (/dss/control) | src/DSSControlNode.cpp:32 |
| 36b | `DSSControlNode.jog구독람다` (inner) | `DssControl::SharedPtr` | void | `last_jog_cmd_`·`last_jog_rx_` 갱신 (/dss/control/jog) | src/DSSControlNode.cpp:38 |
| 36c | `DSSControlNode.enable구독람다` (inner) | `Bool::SharedPtr` | void | `jog_enabled_` 갱신 + 값 변화 시 "제어 소스 전환" INFO 로그 | src/DSSControlNode.cpp:45 |
| 37 | `DSSControlNode::sendControl` | 없음 | void | 활성 소스(jog_enabled_ ? jog : auto) 명령 선택 → 500ms 내 수신이면 전달, 아니면 0 명령(dead-man). **자율 자동 폴백 없음** | src/DSSControlNode.cpp:67 |
| 38 | `main` | `argc`, `argv` | int | `runBridgeNode<DSSControlNode>` 위임 — 중복 main 미신설 | src/DSSControlNode.cpp:98 |

### 4. 전역 변수 / 모듈 상수 (delta)

| # | 변수 | 사용처(함수) | 기능 | 위치 |
|---|------|--------------|------|------|
| 10 | `kSendPeriodMs` (상수 50) | #36·#37 | UDP 송신 주기 50 ms (20 Hz) | src/DSSControlNode.cpp:84 |
| 11 | `kCommandTimeoutMs` (상수 500) | #37 | dead-man 수신 두절 임계 (소스별 각각 적용) | src/DSSControlNode.cpp:85 |
| 12 | `last_auto_cmd_`·`last_auto_rx_` (인스턴스 가변) | #36a(writer) · #37(reader) | 자율 소스 마지막 명령·수신 시각(steady clock). 단일 스레드 executor 라 동기화 불요 | src/DSSControlNode.cpp:87,89 |
| 12a | `last_jog_cmd_`·`last_jog_rx_` (인스턴스 가변) | #36b(writer) · #37(reader) | 수동(jog) 소스 마지막 명령·수신 시각 | src/DSSControlNode.cpp:88,90 |
| 12b | `jog_enabled_` (인스턴스 가변 bool) | #36c(writer) · #37(reader) | latched /dss/jog_enabled 미러 — 미수신 기본 false(자율) | src/DSSControlNode.cpp:91 |

### A-1. Subscriptions (delta)

| 토픽 | 메시지 타입 | QoS | 콜백 함수 | 위치(file:line) |
| --- | --- | --- | --- | --- |
| /dss/control | dss_ros2_bridge/DssControl | 10 · RELIABLE · VOLATILE | 함수 #36a | src/DSSControlNode.cpp:30 |
| /dss/control/jog | dss_ros2_bridge/DssControl | 10 · RELIABLE · VOLATILE | 함수 #36b | src/DSSControlNode.cpp:36 |
| /dss/jog_enabled | std_msgs/Bool | 1 · RELIABLE · TRANSIENT_LOCAL | 함수 #36c | src/DSSControlNode.cpp:43 |

### 관계

- 2026-08-16 delta 의 단일 구독(`sub_`·`last_cmd_`·`last_rx_`)은 auto/jog 2계열 + enable 로 분화. `msg/DssControl.msg`·`setDriveControl` 시그니처는 무변경.
- 의존성 추가: std_msgs (package.xml · CMakeLists.txt).
