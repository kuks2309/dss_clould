# dss_ros2_bridge — SW 구조 분석 타임라인

대상: `src/dss_ros2_bridge` (ROS 2 Humble, ament_cmake, 실행 타깃 10개)

| 날짜 | 코드 버전 | 산출물 분기 | 핵심 |
| --- | --- | --- | --- |
| [2026-08-15](2026-08-15.md) | 비-git · 파일 SHA256 고정 (`DSSToROSImage.cpp` `7486c1cb…` 외 20개) | 파일 그래프 + 클래스 + 시퀀스 | 노드 10개 평면 구조 · 브리지 9개 골격 완전 반복 · 순환 없음 · 공유 헤더 `dss.pb.h`(fan-in 11) `defaultGateway.h`(10) |

## 동반 다이어그램

| 날짜 | ① 파일 그래프 | ② 클래스 | ③ 시퀀스 |
| --- | --- | --- | --- |
| 2026-08-15 | [2026-08-15-file-graph.drawio](2026-08-15-file-graph.drawio) | [2026-08-15-class.drawio](2026-08-15-class.drawio) | [2026-08-15-sequence.drawio](2026-08-15-sequence.drawio) |

## 기록 위치

- 루트 정본 (canonical): `docs/sw_structure/dss_ros2_bridge/`
- 패키지 병기 (mirror): `src/dss_ros2_bridge/docs/sw_structure/dss_ros2_bridge/`

두 사본은 동일 내용을 유지한다. 결함·품질 판정은 본 번들 범위 밖이며 `docs/code_review/dss_ros2_bridge/` 가 담당한다.
