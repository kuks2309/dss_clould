## 2026-08-16 (KST) — dss_ros2_bridge 함수표 delta: DSSControlNode 신설 (코딩 lane 계획)

> coding.md §2 에 따른 **신규 파일 설계 표**다(리뷰 아님, verdict 없음). Core 인벤토리 정본
> [2026-08-15.md](2026-08-15.md) 의 함수 번호를 이어(#36~) 쓴다. 구현 직후 실제 줄 번호로 갱신(§6).
> 결정 근거: [ADR 2026-08-16-jog-control](../../adr/2026-08-16-jog-control.md)

### 3. 함수 리스트 (delta — 신규 파일 src/DSSControlNode.cpp)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
|---|------|------|------|------|------|
| 36 | `DSSControlNode::DSSControlNode` | 없음 | — | VSS `start(게이트웨이, 8886, 4222)` 실패 시 throw → `/dss/control` 구독(reliable 10) → 50ms 송신 타이머 | src/DSSControlNode.cpp:16 |
| 36a | `DSSControlNode.구독람다` (inner) | `DssControl::SharedPtr` | void | `last_cmd_`·`last_rx_` 갱신 | src/DSSControlNode.cpp:29 |
| 37 | `DSSControlNode::sendControl` | 없음 | void | 500ms 내 수신이면 `last_cmd_`, 아니면 0 명령을 `setDriveControl` 로 UDP 전달(dead-man) | src/DSSControlNode.cpp:43 |
| 38 | `main` | `argc`, `argv` | int | `runBridgeNode<DSSControlNode>` 위임 — 중복 main 미신설(2026-08-15 함수 #15 중복 계열 회피) | src/DSSControlNode.cpp:67 |

### 4. 전역 변수 / 모듈 상수 (delta)

| # | 변수 | 사용처(함수) | 기능 | 위치 |
|---|------|--------------|------|------|
| 10 | `kSendPeriodMs` (상수 50) | #36·#37 | UDP 송신 주기 50 ms (20 Hz) | src/DSSControlNode.cpp:58 |
| 11 | `kCommandTimeoutMs` (상수 500) | #37 | dead-man 수신 두절 임계 | src/DSSControlNode.cpp:59 |
| 12 | `last_cmd_`·`last_rx_` (인스턴스 가변) | #36a(writer) · #37(reader) | 마지막 수신 명령·수신 시각(steady clock). 단일 스레드 executor 라 별도 동기화 불요 | src/DSSControlNode.cpp:61-62 |

### A-1. Subscriptions (delta)

| 토픽 | 메시지 타입 | QoS | 콜백 함수 | 위치(file:line) |
| --- | --- | --- | --- | --- |
| /dss/control | dss_ros2_bridge/DssControl | 10 · RELIABLE · VOLATILE · KEEP_LAST | 함수 #36a | src/DSSControlNode.cpp:28 |

### 후진(target_gear) 확장 amendment

- `msg/DssControl.msg` 에 `float32 target_gear` 필드 추가 (0=유지/전진 · -1=후진, GPS 이동 벡터 실측으로 인코딩 확정 — ADR addendum).
- 함수 #22 `DSSVssClient::setDriveControl` 시그니처 확장: `(throttle, steer, brake, targetGear=0.0F)` — 기본값 0 이라 기존 콜사이트(데모 #32d) 무변경, proto3 가 0 필드를 생략해 종전 패킷과 바이트 동일 | src/DSS.VSSClient.h:28 · src/DSS.VSSClient.cpp:149
- 함수 #37 `sendControl` 이 `last_cmd_.target_gear` 를 전달 | src/DSSControlNode.cpp:50
