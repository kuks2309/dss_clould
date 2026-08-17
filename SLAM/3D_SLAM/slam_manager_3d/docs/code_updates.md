# slam_manager_3d 수정 이력 (패키지 병기본)

> 정본은 Divine 루트 docs/code_updates/2026-08-16-3d-slam-port.md 추기.

## 2026-08-17 — 패키지 신설 (v0.1.0)

원본 ~/Study/ros2_3dslam_ws slam_manager_3d 분석(리뷰:
docs/code_review/slam_manager_3d/2026-08-17.md, 함수 77 전수) 기반으로 DSS 전용
3-스택(dss_lio_sam / HDL / RTAB-Map) 구동 GUI 를 신설. 설계 결정은
docs/adr/2026-08-17-slam-manager-3d.md (D1~D9).

원본 결함 회피 실측 반영:
- Gazebo 자동 감지 제거(High — DSS 도 /clock 발행이라 항상 오판)
- 정지 = 자기 프로세스 트리 한정(전역 pkill·ros2 daemon 재시작 제거)
- UI 접근 전부 시그널 경유(High — cross-thread 위젯 접근)
- PID 생존 = 세션 리더 속성(getsid==pid) — exec 의 cmdline 교체·PID 재사용 오탐 회피
- DSS 클럭 리셋 감지 배너 신설(sim clock 역행 5 s)

구현 중 실측 수정 2건:
1. ProcessHandle.alive cmdline 대조 → 세션 리더 판정 교체 (exec 가 이미지 교체하여
   스크립트 경로 대조가 기동 직후 무효 — 단위시험 재현)
2. stop 을 세션 한정 pkill → 세션+자손 트리 BFS 로 보강 (E2E 실측: launch 산하
   rviz2 가 자체 세션을 새로 만들어 세션 소탕을 벗어남)

검증: StackProcessManager 단위 4/4 (기동·세션 자식·정지 소탕·이중 기동 거부),
GUI E2E — computer-use 클릭으로 RTAB 매핑 기동(icp_odometry+rtabmap+rviz 생성,
6-DOF 라벨 라이브, 버튼 배타 로직)→정지(전 프로세스 0) 통과.
