# jog_control — 인벤토리 타임라인

대상: `src/Motion_Control/jog_control` (ROS 2 Humble, ament_cmake, Qt5 Widgets, 실행 타깃 1개)

| 날짜 | 성격 | 핵심 |
| --- | --- | --- |
| [2026-08-16](2026-08-16.md) | coding 계획 표 (리뷰 아님) | 신규 패키지 함수표·전역표·의존성 — jog GUI `/dss/control` 20 Hz 발행, dead-man. 검증·이력: `docs/code_updates/2026-08-16-jog-control.md` |

## 기록 위치

- 루트 정본 (canonical): `docs/code_review/jog_control/`
- 패키지 병기 (mirror): `src/Motion_Control/jog_control/docs/code_review/jog_control/`

## 미해결

- 정식 코드 리뷰(별도 lane) 미수행 — 본 표는 coding SOP §2 계획 인벤토리다.
- steer 부호 실측(debt-009) · 후진 기어(debt-008) → `docs/debt/registry.md`
