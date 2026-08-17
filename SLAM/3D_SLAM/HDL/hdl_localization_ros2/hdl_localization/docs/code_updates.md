# hdl_localization 수정 이력 (패키지 병기본)

> 정본은 Divine 루트 docs/code_updates/2026-08-16-3d-slam-port.md (2026-08-17 09:40 추기).
> 본 패키지는 vendored(원본: koide3 hdl_localization 의 ROS2 포트, parking_robot_ros2_ws 경유).

## 2026-08-17

- **DSS QoS 정합 패치** — `apps/hdl_localization_nodelet.cpp:61·65`:
  points·imu 구독을 기본 RELIABLE → `rclcpp::SensorDataQoS()` 로 변경.
  DSS 브리지가 BEST_EFFORT 로 발행하므로 기본값으로는 RxO 위반(offered<requested)이라
  메시지가 0건 수신되던 실측 결함의 수정. 그 외 소스 무수정.
- 운영 규약(코드 아님): `/odom`→`/hdl/odom` 리맵 주의 · `/initialpose` 주입 필요 ·
  `odom_child_frame_id` 는 `base_link` 사용(lidar_link 지정 시 TF 이중 부모 충돌).
