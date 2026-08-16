# ADR 2026-08-16 — 차량 jog 제어: jog_control GUI + /dss/control 토픽 + DSSControlNode UDP 전달

- **Status**: Accepted — 2026-08-16 (사용자 승인: GUI 시안 A '버튼 조그 패드' + 'ROS2 토픽 경유' 채택. 실기 검증 미수행)

## Context

- 차량 제어의 유일한 실경로는 `DSSVssClient::setDriveControl(throttle, steer, brake)` → protobuf `dss::DssSetControl` → UDP :8886 이다 (src/DSS.VSSClient.cpp:149). ROS2 제어 토픽은 존재하지 않았다.
- `dss_ros2_bridge/msg/DssControl` (steer·throttle·brake, float32) 이 정의·빌드되지만 발행자·구독자 모두 0 인 미사용 상태였다.
- `DSSVssClient::start()` 는 NATS(:4222) 연결 성공 후에만 UDP 소켓을 준비한다 (src/DSS.VSSClient.cpp:54-71) — 제어 전달 노드도 NATS 도달이 전제다.
- 값 범위(proto 주석 근거): steer -1.0~+1.0, throttle 0.0~1.0, brake 0.0~1.0. `targetGear`(후진)·`parkBrake` 의 값 인코딩은 미확인.
- DSS 호스트는 `getDefaultGateway()` = `100.80.80.15` 하드코딩 (src/defaultGateway.cpp:15).

## Decision

1. 신규 패키지 `src/Motion_Control/jog_control` — Qt5 Widgets GUI(`JogControlNode`). `/dss/control` 을 reliable·depth 10 으로 **20 Hz 상시 발행**(놓은 상태에서도 0 명령 스트림 유지). 버튼·키를 누르는 동안만 값이 실리고 떼면 0 (dead-man). 스로틀 상한·조향 강도 슬라이더 제공.
2. `dss_ros2_bridge` 에 `DSSControlNode` 신설 — `/dss/control` 구독(reliable 10) → `setDriveControl()` → UDP :8886. 50 ms 주기 송신 + **500 ms 수신 두절 시 0 명령**(watchdog dead-man).
3. 후진(`targetGear`)·`parkBrake`·라이트류는 1차 범위 제외 — 인코딩 미확인 상태의 추정 구현 금지(reverse_engineering §6 원칙). 필요 시 DSS 매뉴얼 대조 후 확장.
4. steer 부호(◀=음수 가정)는 실기 미검증 — 코드 주석·README 에 뒤집는 지점을 명시한다.

## Alternatives

- GUI 직접 UDP 송신(브리지 무수정): proto·UDP 로직 중복 + 제어 토픽 부재로 다른 노드가 제어 경로를 재사용 불가 → 기각(사용자 선택 2026-08-16).

## Consequences

- 이득: GUI 수동 jog 조작 가능. `/dss/control` 이 공개 제어 표면이 되어 향후 자율주행 스택이 같은 경로를 재사용한다.
- 비용: 브리지에 노드 1개 추가, 신규 패키지 1개.
- 위험: ① steer 부호 미검증 ② UDP 제어는 유실이 보고되지 않음(기존 특성 — 2026-08-15 리뷰 의존성 표) ③ 제어 발행자가 2개 이상이면 경합 — 기동 규율(ros2-coding §5)로 통제하며, 특히 `DSSDemoNode`(0 명령 상시 송신)와 동시 기동 금지.

## Rollback

N/A (가역) — `jog_control` 패키지 삭제 + `dss_ros2_bridge/CMakeLists.txt` 의 `DSSControlNode` 타깃·install 항목 제거로 원상복구.

## Addendum (2026-08-16) — 수신 노드 버튼 기동 + 후진 지원

- **수신 노드 버튼 기동** (사용자 지시 10:51): launch 일괄 기동안 대신 GUI 가 `/dss/control` 구독자 수로 수신측 실행 여부를 감지, 미연결이면 [수신 노드 실행] 버튼으로 `DSSControlNode` 를 자식 프로세스(QProcess)로 기동. GUI 종료 시 자기가 띄운 것만 정리. 의존성 `ament_index_cpp` 추가.
- **후진 인코딩 실측 확정** (사용자 질문 11:10 "후진이 왜 없을까"): GPS 이동 벡터 대조 실험(sim, 2026-08-16 11:11) — 기어 없음/1/2 = 전진, **targetGear=-1 = 후진**(기준 전진과 정확히 반대 벡터), 음수 스로틀 = 무시. Decision 3 항(후진 제외)을 이 실측으로 supersede 한다.
- **구현**: `DssControl.msg` 에 `float32 target_gear`(0=유지 · -1=후진) 추가, `setDriveControl` 4번째 인자 `targetGear=0.0F`(기본값 — 기존 콜사이트·패킷 무변경, proto3 0 생략), GUI ▼ 브레이크 → ▼ 후진 교체(↓/S), ■ 정지 = 누르는 동안 전 해제+풀 브레이크(Space).
- **Rollback**: N/A (가역) — msg 필드·인자 기본값이라 제거 시 컴파일 오류 지점만 원복.
