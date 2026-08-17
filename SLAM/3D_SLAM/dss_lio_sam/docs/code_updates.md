# dss_lio_sam 수정 이력 (패키지 병기본)

> 정본은 Divine 루트 [docs/code_updates/2026-08-16-3d-slam-port.md](../../../../docs/code_updates/2026-08-16-3d-slam-port.md).
> 본 파일은 패키지 곁 병기 — 항목 제목과 요지만 미러한다.

## 2026-08-16

- **이식·개조** — livox_lio_sam → DSS 전용 이식 (params_dss.yaml 실측 기반 ·
  dss_imu_gravity.py 중력 릴레이 · run_slam_dss.launch.py · launch_utils 제거 ·
  저장 경로 HOME-상대 규약 · SIGINT 처리)
- **패키지명 rename** — livox_lio_sam → dss_lio_sam (msg/srv 네임스페이스·실행 파일 동반)
- **Localization 구성** (11:07) — params_dss_localization.yaml + 
  launch/localization/run_localization_dss.launch.py 신설. 실사격 검증:
  지도 로드 471k pt → 재인식 (0.001, 0.252, -0.102) · 드리프트 <1 mm/10 s.
  원본 결함 debt-010 발견(종료 자동저장이 지도 폴더를 rm -r 후 다른 파일명으로 저장
  → 로더와 불일치) — 상세는 루트 정본·debt registry 참조.
