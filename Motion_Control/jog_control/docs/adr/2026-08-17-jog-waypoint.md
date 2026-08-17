# ADR 2026-08-17 — jog GUI waypoint 등록·저장·불러오기

- **Status**: Superseded — 2026-08-17 21:17 (사용자 정정: 해당 지시는 **타 세션에 보낼 것이 이 세션에 오입력**된 것으로, jog GUI 는 대상이 아님. waypoint 는 기존 `dss_motion_control/waypoint_ui.py` 소관. 구현 전량 원위치 — 코드·의존성(tf2_ros·geometry_msgs)·README 제거, 기록: docs/code_updates/2026-08-17-jog-waypoint.md §철회, mistake 2026-08-17-007)
- ~~**Status**: Accepted — 2026-08-17 (사용자 지시 20:58 "현재 위치 waypoint 등록 버튼 + 파일 저장·읽어오기". 세션 8a436cd4)~~

## Context

- 자율 스택(dss_motion_control)의 경로 소비 형식은 **map 프레임 (x, y) 미터**다 — tracking_server 는 `goal.path.poses` 의 x/y 만 사용(tracking_server.py:181)하고, 현재 위치는 TF(Transform) `map→base_link` lookup 으로 취득한다(tracking_server.py:143, send_goal.py:48).
- 프로젝트에 기존 waypoint **파일 형식이 없다** — send_goal.py 는 CLI 인자 (x, y) 만 받는다. 형식 신규 정의 필요.
- jog GUI(JogControlNode)는 executor 를 spin 하지 않는다 — TF 구독은 자체 스레드를 가진 리스너가 필요.

## Decision

1. **위치 소스 = TF `map→base_link`** — 소비처(tracking_server)와 동일 소스·동일 프레임. 등록한 waypoint 를 자율 스택이 변환 없이 그대로 쓴다. GPS(위경도) 대안은 프레임 변환이 한 겹 더 필요해 기각. TF 부재 시(측위 스택 미기동) 등록 버튼은 오류 표시("측위 없음")만 하고 무동작.
2. **TF 취득**: `tf2_ros::Buffer` + `TransformListener(spin_thread=true, 내부 전용 노드)` — GUI 가 spin 하지 않아도 자체 스레드로 수신.
3. **파일 형식 = CSV(Comma-Separated Values)**: 줄당 `x,y` (m, map 프레임), `#` 주석 헤더 허용. 확장자 `.csv`. 사람이 읽고 손편집 가능 + 파이썬(자율 측)에서 한 줄 파싱.
4. **UI**: [현위치 WP 등록]·[저장]·[불러오기]·[비우기] 버튼 + 상태줄("WP N개 · 마지막 (x, y)"). **jog enable 과 무관하게 상시 활성**(controls_ 묶음 밖) — 자율 주행을 지켜보며 등록하는 용례 지원. 저장·불러오기는 QFileDialog.
5. **비우기 포함**(요청 밖 최소 보강): 목록 교체 수단이 불러오기뿐이면 오등록 정정이 파일 손편집을 강제한다 — 버튼 1개로 해결.
6. GUI 는 등록·보관·파일 IO(Input/Output)까지만 담당 — **goal 전송(자율 발진)은 범위 외**(기존 send_goal/액션 경로 소관).

## Alternatives

- GPS 위경도 기록: 소비처가 map x/y 라 사용 시마다 변환 필요 + 측위-GPS 정합 의존 → 기각.
- YAML 형식: 구조 확장성은 있으나 현 소비 형식이 (x,y) 리스트뿐 — CSV 로 충분, 필요 시 후속 ADR 로 확장.

## Consequences

- 이득: 저장 파일이 자율 스택 좌표계와 1:1 — 후속으로 "waypoint 경로 발진" 기능을 붙일 토대.
- 비용: jog_control 에 tf2_ros·geometry_msgs 의존 추가. TF 리스너 내부 노드 1개 증가.
- 위험: 측위(SLAM/localization) 미기동 상태에선 등록 불가(의도된 제약 — 오류 표시). map 원점이 세션마다 다르면 저장 파일의 재사용성이 측위 스택의 원점 고정성에 종속(측위 소관, 본 ADR 범위 외).

## Rollback

N/A (가역) — waypoint UI 블록·TF 멤버·의존성(tf2_ros·geometry_msgs) 제거로 mux ADR 직후 상태로 복귀. 저장된 CSV 파일은 독립 산출물이라 코드 롤백과 무관하게 잔존(무해).
