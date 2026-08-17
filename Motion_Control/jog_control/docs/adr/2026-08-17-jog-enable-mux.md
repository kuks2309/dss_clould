# ADR 2026-08-17 — jog enable 토글 + 수신측 제어 소스 mux (/dss/control/jog · /dss/jog_enabled)

- **Status**: Accepted — 2026-08-17 (사용자 지시 20:05 "조그 GUI 에 enable 을 만들어 enable 시 GUI 로 조정, disable 시 다른 노드가 조정". 세션 8a436cd4)

## Context

- `/dss/control` 에는 이미 자율 발행자가 실재한다 — `dss_motion_control`(tracking_server.py:53, PI 추종 제어). 실측(2026-08-17 20:07) 발행자 1·구독자 0. jog GUI 도 같은 토픽을 발행하므로 동시 기동 시 경합한다 — [ADR 2026-08-16](2026-08-16-jog-control.md) 위험 ③은 기동 규율(동시 기동 금지)로만 통제했고, 이번 요구(GUI 상시 표시 + 토글 전환)는 그 전제를 깬다.
- `DSSControlNode` 가 UDP 전달 단일 관문이다(마지막 수신값 20 Hz 송신 + 500 ms dead-man) — 중재를 넣을 자연스러운 지점.

## Decision

1. **중재는 수신측(DSSControlNode)** 에서 한다 — 발행자들의 규약 준수에 의존하지 않는다. jog GUI 는 전용 토픽 `/dss/control/jog` (DssControl, reliable·depth 10)로 분리 발행하고, 기존 `/dss/control` 은 자율(다른 노드) 제어 표면으로 유지한다(tracking_server 무수정).
2. **`/dss/jog_enabled`** (std_msgs/Bool, reliable + transient_local·depth 1) — GUI 가 상태 변화·기동·정상 종료 시 발행. latched 라 수신측이 늦게 떠도 현재 상태를 받는다.
3. **DSSControlNode**: enable=true → jog 명령만 UDP 전달(자율 무시), false → `/dss/control` 전달. 소스별 `last_cmd`/`last_rx` 분리, 활성 소스 기준 500 ms dead-man(두절 시 0 명령). enable 중 jog 스트림 두절 시 **자율로 자동 폴백하지 않는다** — 수동 오버라이드 중 무단 자율 전환이 정지보다 위험하다. 소스 전환은 INFO 로그.
4. **GUI**: enable 기본 **OFF** — 자율 주행 중 GUI 를 띄워도 제어권을 뺏지 않는다. OFF 상태에서는 발행 전면 중단 + 조작 위젯 비활성 + 키 무시. ON→OFF 전환 시 0 명령 1회 발행 후 침묵(안전 벨트). 정상 종료(closeEvent) 시 enable=false 발행.

## Alternatives

- **A. 단일 토픽 유지 + 발행자 자율 양보** (다른 노드들이 `/dss/jog_enabled` 를 구독해 스스로 발행 중지): 발행자 전원이 규약을 알아야 안전 — 모르는 발행자 하나로 무너지고, tracking_server 수정 필요(타 작업 영역 간섭) → 기각.
- **B. GUI 발행 게이트만** (중재 없음): disable 은 해결되나 enable ON 동안 자율 발행자와 20 Hz last-write-wins 경합 지속(지터) → 기각.

## Consequences

- 이득: 수동 오버라이드가 **구조적으로** 보장된다(다른 발행자가 규약을 몰라도 안전). `/dss/control` 이 자율 표면으로 온전히 남아 tracking_server 등 기존 발행자 무수정.
- 비용: 브리지에 구독 2개 추가, jog GUI 발행 토픽 이동(`/dss/control` → `/dss/control/jog`), 양 패키지에 std_msgs 의존 추가.
- 위험: enable ON 채 GUI 비정상 종료 → 수신측은 마지막 true 를 유지하고 jog 스트림 dead-man 으로 0 명령(차량 정지). 자율 복귀는 수동(GUI 재실행 후 OFF 또는 수신측 재기동) — 정지가 기본값인 안전 우선 설계.

## Rollback

N/A (가역) — GUI 발행 토픽을 `/dss/control` 로 되돌리고 enable 토글·Bool 발행을 제거, DSSControlNode 를 단일 구독으로 원복하면 2026-08-16 상태로 복귀.
