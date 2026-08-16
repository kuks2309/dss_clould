# dss_clould — DSS 시뮬레이터 ROS2 워크스페이스 (src)

git 협업 모드: solo

DSS(Digital twin Simulation System) 시뮬레이터를 ROS2 Humble 에 잇는 패키지 모음.

| 패키지 | 내용 |
| --- | --- |
| [dss_ros2_bridge/](dss_ros2_bridge/) | DSS NATS 스트림 → ROS2 토픽 브리지 (센서 9종 + 제어 데모 + `/dss/control` UDP 전달 노드) |
| [Motion_Control/jog_control/](Motion_Control/jog_control/) | DSS 차량 수동 조작(jog) Qt5 GUI — `/dss/control` 20 Hz 발행 |
| [SLAM/3D_SLAM/dss_lio_sam/](SLAM/3D_SLAM/dss_lio_sam/) | DSS 전용 LIO-SAM 포크 (라이다-관성 SLAM) |
| [SLAM/3D_SLAM/rtab_map_3d/](SLAM/3D_SLAM/rtab_map_3d/) | RTAB-Map 3D 라이다 SLAM/Localization 설정 |

빌드: 워크스페이스 루트에서 `colcon build --symlink-install`.
각 패키지 설치·실행 절차는 패키지별 README/docs 참조.
