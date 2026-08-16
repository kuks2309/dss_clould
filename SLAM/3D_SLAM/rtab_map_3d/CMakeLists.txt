cmake_minimum_required(VERSION 3.8)
project(rtab_map_3d_config)

find_package(ament_cmake REQUIRED)

# launch 디렉토리 구조를 보존해 설치한다 (trailing slash 없음) —
# launch 파일이 share/<pkg>/launch/dss_static_transforms.launch.py 경로로 include 한다.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
