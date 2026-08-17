# hdl_global_localization 수정 이력 (패키지 병기본)

> 정본은 Divine 루트 docs/code_updates/2026-08-16-3d-slam-port.md. vendored ROS2 포트.

## 2026-08-17 — "no map received" 진단·수정 3건 (전부 실측 재현 후 수정)

1. **다운샘플 해상도 0 버그** — `src/hdl_global_localization_node.cpp:24-25`:
   `declare_parameter` 반환값을 멤버에 대입하지 않아 해상도가 0 으로 남았고, leaf-0
   ApproximateVoxelGrid 가 63만 점 지도를 **1점으로 붕괴**시켜 BBS 격자가 비었다.
   진단 로그 삽입으로 실측 확인(`recv 634597 → after downsample(0): 1`) 후 대입 수정.
   수정 후: downsample(0.5) → 152,870 pt → Set Map 12,897 pt.
2. **BBS 토픽 프리픽스 누락** — `engines/global_localization_bbs.cpp:12-14`:
   포트가 원본의 `bbs/` 프리픽스를 빼고 `gridmap` 등으로 발행해 RViz 설정
   (`bbs/gridmap`)과 어긋났다. `bbs/gridmap`·`bbs/map_slice`·`bbs/scan_slice` 로 정정.
3. **1회성 발행 → 재발행 타이머** — 격자맵이 지도 설정 시 1회만 발행돼 늦게 붙은
   구독자(RViz)가 놓쳤다. 최근 격자·슬라이스를 캐시하고 1 Hz 재발행 타이머 추가
   (hpp 멤버 3 + cpp 타이머). `to_rosmsg()` 반환형은 ConstSharedPtr.

검증: RViz Map 디스플레이 Status Ok·"Map received"(1024×1024@0.5 m 렌더 확인 캡처),
/hdl_global_localization/bbs/gridmap Publisher 1.

운영 파라미터 (DSS·현 지도 기준): `global_localization_engine:=BBS`,
`bbs/map_min_z:=-0.5`, `bbs/map_max_z:=1.5` (지도 z 히스토그램 실측 기반 — 지면 링 -3~-1,
벽 0~20). 노드는 `--ros-args -r __ns:=/hdl_global_localization` 로 기동해야 서비스명이
클라이언트 기대(`/hdl_global_localization/set_global_map`)와 일치.

추가 실측 (2026-08-17 10:17): **스캔 슬라이스도 필수** — `bbs/scan_min_z:=0.5`,
`bbs/scan_max_z:=5.0`. DSS 라이다는 센서 좌표 z -0.5~+0.5 대역이 **완전히 비어**(빔 패턴
사각, 실측 0점) 기본값(-0.2~0.2)으로는 `Query 0 points` 로 BBS 가 항상 실패한다.
벽 점은 +0.5~+5.0 에 ~680점. 이 설정으로 /relocalize 실증 성공: 회전 오정합(yaw 106.7°)
상태에서 호출 → (0.04, 0.00, yaw -0.4°) 로 전역 스냅, RViz 벽면 정합 캡처 확인.

콜드스타트 주의: hdl_localization 포트는 `last_scan` 저장이 pose_estimator 생성 이후라
(points_callback 선두의 waiting-for-initial-pose 반환), **/relocalize 는 initialpose 를
한 번 넣은 뒤에만 동작**한다(시드는 틀려도 무방 — BBS 가 전 회전 범위를 탐색).
