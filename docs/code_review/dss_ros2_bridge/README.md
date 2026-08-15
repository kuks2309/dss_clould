# dss_ros2_bridge — 코드 리뷰 타임라인

대상: `src/dss_ros2_bridge` (ROS 2 Humble, ament_cmake, 실행 타깃 10개)

| 날짜 | 코드 버전 | Verdict | 핵심 |
| --- | --- | --- | --- |
| [2026-08-16 08:30](2026-08-16-0830.md) (delta) | `DssBridgeNode.h` `99c75bfa…` 신규 4파일 + 노드 9개 재작성 | COMMENT | **High #8 해소.** 골격을 `DssBridgeNode` 로 추출 — 노드 9개 1,755줄 → 565줄(+공통 272줄, 52% 감소). 동작 보존 11항목 대조 통과. **미해결 High 0** |
| [2026-08-16 08:15](2026-08-16-0815.md) (delta) | `DSSToROSImage.cpp` `1a1acff8…` 외 14개 변경 | REQUEST CHANGES | **리뷰 지적 반영.** High 7 + 신규 High 1 + Medium 5 `[해결]` — 퍼블리셔 순서 race · sprintf · closure UB · start() 무시 · 좀비 노드 · package.xml · unsubscribe · 쿼터니언 정규화. High 1(9벌 골격 중복) `[잔존]` → 08:30 에 해소 |
| [2026-08-16](2026-08-16.md) (delta) | `DSSToROSImage.cpp` `16ab80eb…` 외 9개 변경 | REQUEST CHANGES | **실기 구동 delta.** 신규 High 2 (IMU 쿼터니언 norm 0.707 비정규화 발행, 스테레오 4개 노드의 원천 subject 부재) · QoS `[해결]` · Critical 1 → Medium 재평가 · Low 1 철회 |
| [2026-08-15](2026-08-15.md) | 비-git · 파일 내용 해시 (`DSSToROSImage.cpp` `7486c1cb…` 외 18개) | REQUEST CHANGES | Critical 1 (포인트클라우드 경계 검증 부재) · High 8 (퍼블리셔 순서 race, sprintf 오버플로, closure 타입 UB, start() 무시, 좀비 노드, package.xml 의존성 누락, unsubscribe 이중 해제, 9벌 골격 중복) |

> **2026-08-15 스냅샷의 정정 사항** — 실기 측정으로 세 항목이 바뀌었다: Critical 1(포인트클라우드 경계) → Medium 재평가(현재 데이터는 정합해 미발동), Medium `[QoS]` 의 성능 서술 반증, Low(IMU covariance) 철회. 상세는 2026-08-16 delta 를 볼 것. 스냅샷 본문은 그 시점 기록으로 보존한다.

- 분석 분기: 전체 구조 분석 (다중 진입점 10개 → 흐름 A/B 분리)
- 감지된 도메인: ros2-review · concurrency
- 동반 플로우차트: [2026-08-15-flow.drawio](2026-08-15-flow.drawio)
- staleness: 코드가 바뀌면 파일 내용 해시가 달라진다. 재리뷰 시 위 표의 해시와 대조해 delta 범위를 정할 것.

## 기록 위치

- 루트 정본 (canonical): `docs/code_review/dss_ros2_bridge/`
- 패키지 병기 (mirror): `src/dss_ros2_bridge/docs/code_review/dss_ros2_bridge/`

## 관련 산출물

- 구조 지도(연결 관계·클래스·시퀀스): `docs/sw_structure/dss_ros2_bridge/2026-08-15.md`
  본 리뷰의 인벤토리와 같은 코드 상태를 공유한다. 연결 관계 질문은 그쪽, 결함 판정은 이쪽이 담당한다.

## 미해결

본 리뷰는 작성 lane 에서 `APPROVE` 할 수 없다(SOP rule 11). Critical 1 · High 8 조치 후 별도 lane 재리뷰 필요.
