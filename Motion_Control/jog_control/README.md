# jog_control — DSS 차량 수동 조작(jog) Qt5 GUI

`/dss/control` (dss_ros2_bridge/DssControl) 을 20 Hz 로 상시 발행하는 조그 패드.
버튼·키를 누르는 동안만 명령이 실리고 떼면 0 (dead-man). 토픽 수신·UDP 전달은
dss_ros2_bridge 의 `DSSControlNode` 가 담당한다.

```
[JogControlNode(GUI)] --/dss/control (20 Hz, reliable)--> [DSSControlNode] --UDP :8886--> DSS
```

## 실행

```bash
# 워크스페이스 루트에서
colcon build --symlink-install --packages-select dss_ros2_bridge jog_control
source install/setup.bash

ros2 run jog_control JogControlNode
```

GUI 가 수신측(DSSControlNode) 실행 여부를 감지한다 — 미연결이면 우상단 **[수신 노드 실행]**
버튼이 활성화되고, 누르면 GUI 가 자식 프로세스로 기동한다(연결되면 "● 수신 연결됨").
GUI 를 닫으면 버튼으로 띄운 수신 노드도 함께 정리된다(외부에서 따로 띄운 것은 건드리지 않음).
수신측을 따로 띄우려면: `ros2 run dss_ros2_bridge DSSControlNode` (NATS :4222 도달 필요).

## 조작

| 조작 | 버튼 | 키 |
| --- | --- | --- |
| 전진 | ▲ | ↑ / W |
| 후진 (target_gear=-1) | ▼ | ↓ / S |
| 좌·우 조향 | ◀ / ▶ | ←/A · →/D |
| 정지 (누르는 동안 전 해제 + 풀 브레이크) | ■ 정지 | Space |

- Throttle 상한·Steer 강도는 슬라이더로 조절 (기본 0.30 / 0.50).
- 상태줄의 "수신 노드 N" 이 1 이상이어야 명령이 차량까지 전달된다.

## 주의

- **DSSDemoNode 와 동시 기동 금지** — 데모가 0 명령을 상시 송신해 제어가 경합한다 (ros2-coding §5 기동 규율).
- steer 부호(◀=음수)는 실기 미검증 — 반대로 돌면 `src/JogControlNode.cpp` 의 buildUi·applyKey 의 ±1 을 뒤집는다.
- 후진은 `target_gear=-1` (GPS 실측 확정 — [ADR addendum](docs/adr/2026-08-16-jog-control.md)). parkBrake·P/N 인코딩은 미확인(debt-008 잔존).

## 문서

| 경로 | 내용 |
| --- | --- |
| [docs/adr/](docs/adr/) | 설계 결정 기록 |
| [docs/code_review/jog_control/](docs/code_review/jog_control/) | 함수표·전역변수표 인벤토리 |
