"""DSS 3D SLAM 스택 구성 — 실측 검증된 launch·인자·토픽의 단일 정의.

각 항목은 라이브 DSS 검증에서 확정된 값이다. 자동 감지는 두지 않는다 —
DSS 도 /clock 을 발행하므로 원본 slam_manager_3d 의 Gazebo 판정(/clock 존재
검사)은 이 환경에서 항상 오판한다.

명령은 '한 줄 = 한 프로세스'인 셸 라인 목록으로 정의한다. 마지막 줄이 exec 로
전면 실행되고 앞 줄들은 & 로 같은 세션에 배경 기동된다(정지 시 세션 단위 소탕).
{ws}: 워크스페이스 루트, {map}: 지도 경로 치환자.
"""

# 브리지는 TF 를 발행하지 않으므로 라이다 프레임 정적 TF 를 세션 안에서 채운다.
STATIC_TF_LIDAR = (
    'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar_link'
)

# BBS(Branch-and-Bound Search) 전역 재위치화 노드 — 슬라이스 대역은 실측값
# (지도: 벽 0.5~5.0 m, 스캔: 센서 z -0.5~+0.5 빔 사각을 피한 0.5~5.0).
HDL_GLOBAL_NODE = (
    'ros2 run hdl_global_localization hdl_global_localization_node --ros-args'
    ' -r __ns:=/hdl_global_localization -p use_sim_time:=true'
    ' -p global_localization_engine:=BBS'
    ' -p bbs/map_min_z:=0.5 -p bbs/map_max_z:=5.0'
    ' -p bbs/scan_min_z:=0.5 -p bbs/scan_max_z:=5.0'
)

STACKS = {
    'lio': {
        'title': 'LIO-SAM (dss_lio_sam)',
        'odom_topic': '/lio_sam/mapping/odometry',
        'mapping': ['exec ros2 launch dss_lio_sam run_slam_dss.launch.py'],
        # 측위 지도 경로는 params_dss_localization.yaml 에 고정(절대경로) —
        # GUI 에서 바꾸려면 params 수정이 필요하므로 읽기 전용 안내만 한다.
        'loc': ['exec ros2 launch dss_lio_sam run_localization_dss.launch.py'],
        'map_hint': '{ws}/maps/lio_sam/GlobalMap.pcd (params 고정)',
        'map_editable': False,
        'map_filter': 'PCD Files (*.pcd);;All Files (*)',
        # 저장 = 매핑 정지(SIGINT 자동 저장) 후 개명 규약(debt-010) 적용
        'save_mode': 'liosam_stop_rename',
        'save_dir': '{ws}/maps/lio_sam',
    },
    'hdl': {
        'title': 'HDL (graph SLAM + NDT/BBS)',
        'odom_topic': '/hdl/odom',
        'mapping_odom_topic': '/odom',
        'mapping': [
            STATIC_TF_LIDAR + ' &',
            'exec ros2 launch hdl_graph_slam hdl_graph_slam.launch.py'
            ' points_topic:=/dss/sensor/lidar3d use_sim_time:=true',
        ],
        'loc': [
            STATIC_TF_LIDAR + ' &',
            HDL_GLOBAL_NODE + ' &',
            'sleep 3',
            'exec ros2 launch hdl_localization hdl_localization.launch.py'
            ' points_topic:=/dss/sensor/lidar3d use_imu:=false'
            ' odom_child_frame_id:=base_link globalmap_pcd:={map}'
            ' use_global_localization:=true use_sim_time:=true use_rviz:=true'
            ' downsample_resolution:=0.1',
        ],
        'map_hint': '{ws}/maps/hdl_slam/map.pcd',
        'map_editable': True,
        'map_filter': 'PCD Files (*.pcd);;All Files (*)',
        'save_mode': 'hdl_service',
        'save_dir': '{ws}/maps/hdl_slam',
    },
    'rtab': {
        'title': 'RTAB-Map (3D LiDAR)',
        'odom_topic': '/rtabmap/odom',
        'mapping': [
            'exec ros2 launch rtab_map_3d_config rtabmap_dss_3dlidar_slam.launch.py',
        ],
        'loc': [
            'exec ros2 launch rtab_map_3d_config'
            ' rtabmap_dss_3dlidar_localization.launch.py database_path:={map}',
        ],
        'map_hint': '{ws}/maps/rtab_map/rtabmap.db',
        'map_editable': True,
        'map_filter': 'RTAB-Map DB (*.db);;All Files (*)',
        'save_mode': 'rtab_sqlite_backup',
        'save_dir': '{ws}/maps/rtab_map',
    },
}

# LIO-SAM 종료 자동 저장 파일 → 측위 로더 기대 이름 (debt-010 개명 규약)
LIOSAM_RENAME = {
    'cloudGlobal.pcd': 'GlobalMap.pcd',
    'cloudCorner.pcd': 'CornerMap.pcd',
    'cloudSurf.pcd': 'SurfMap.pcd',
}

# 클럭이 이 폭(초) 이상 되감기면 DSS 리셋으로 판정 — 실행 중 스택 재기동 필요
CLOCK_RESET_THRESHOLD_S = 5.0
