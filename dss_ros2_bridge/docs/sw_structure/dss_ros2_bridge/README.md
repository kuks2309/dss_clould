# dss_ros2_bridge — SW(Software) 구조 분석 타임라인

대상: `src/dss_ros2_bridge` (ROS 2 Humble, ament_cmake, 실행 타깃 10개)

| 날짜 | 코드 버전 | 산출물 분기 | 핵심 |
| --- | --- | --- | --- |
| [2026-08-16](2026-08-16.md) | 비-git · 파일 SHA256 (`DssBridgeNode.h` `99c75bfa…` 외 19개) | 파일 그래프 + 클래스 + 시퀀스 | **기반 클래스 도입 반영.** 클래스 계층 2단(`rclcpp::Node` → `DssBridgeNode` → 브리지 9) · 번역단위 20 · 골격 중복 9벌 → 1벌 · ② 엣지 39 → 23 |
| [2026-08-15](2026-08-15.md) | 비-git · 파일 SHA256 (`DSSToROSImage.cpp` `7486c1cb…` 외 20개) | 파일 그래프 + 클래스 + 시퀀스 | 노드 10개 평면 구조 · 브리지 9개 골격 완전 반복 · 순환 없음 · 공유 헤더 `dss.pb.h`(fan-in 11) `defaultGateway.h`(10) |

> 2026-08-15 스냅샷은 **그 시점 기록으로 보존**한다. 현재 코드 기준 구조는 2026-08-16 을 볼 것 — 클래스 계층·파일 구성·중복 상태가 모두 다르다. 경위는 [ADR 기반 클래스 추출](../../adr/2026-08-16-bridge-base-class.md).

## 동반 다이어그램

| 날짜 | ① 파일 그래프 | ② 클래스 | ③ 시퀀스 |
| --- | --- | --- | --- |
| 2026-08-16 | [2026-08-16-file-graph.drawio](2026-08-16-file-graph.drawio) | [2026-08-16-class.drawio](2026-08-16-class.drawio) | [2026-08-16-sequence.drawio](2026-08-16-sequence.drawio) |
| 2026-08-15 | [2026-08-15-file-graph.drawio](2026-08-15-file-graph.drawio) | [2026-08-15-class.drawio](2026-08-15-class.drawio) | [2026-08-15-sequence.drawio](2026-08-15-sequence.drawio) |

## 기록 위치

- 루트 정본 (canonical): `docs/sw_structure/dss_ros2_bridge/`
- 패키지 병기 (mirror): `src/dss_ros2_bridge/docs/sw_structure/dss_ros2_bridge/`

두 사본은 동일 내용을 유지한다. 결함·품질 판정은 본 번들 범위 밖이며 `docs/code_review/dss_ros2_bridge/` 가 담당한다.
