cmake_minimum_required(VERSION 3.5)
project(dss_ros2_bridge)

# 사용 언어
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# protobuf 3.15 는 README 절차상 /usr/local 에 소스 빌드로 들어간다.
# 배포판 패키지나 다른 prefix 를 쓰면 -DProtobuf_DIR=... 로 덮어쓴다.
set(Protobuf_DIR "/usr/local/lib/cmake/protobuf" CACHE PATH "protobuf CMake 패키지 경로")

# 공통 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Protobuf 3.15 REQUIRED)
find_package(Threads REQUIRED)

# 메시지 생성기
find_package(rosidl_default_generators REQUIRED)


find_library(NATS_LIB nats PATHS /usr/local/lib)
if(NOT NATS_LIB)
  message(FATAL_ERROR "libnats 를 찾지 못했습니다. README '2.1 필수 라이브러리 설치' 절차로 설치하십시오 (apt install libnats-dev).")
endif()

# ===============================
# 메시지 인터페이스 생성
# ===============================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DssControl.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===============================
# 실행 파일 생성
# ===============================

# 노드 실행 타깃 하나를 만든다. 모든 노드가 공유하는 것(protobuf 생성물·게이트웨이
# 조회·링크 라이브러리)을 한곳에 모아, 노드가 늘어도 블록을 복제하지 않게 한다.
#   SOURCES  노드 고유 소스
#   DEPS     ament_target_dependencies 인자
function(add_dss_node target)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
  add_executable(${target} ${ARG_SOURCES} src/dss.pb.cc src/defaultGateway.cpp)
  ament_target_dependencies(${target} ${ARG_DEPS})
  target_link_libraries(${target}
    "${cpp_typesupport_target}"
    ${NATS_LIB}
    Threads::Threads
    protobuf::libprotobuf
  )
endfunction()

# 브리지 노드는 공통 골격(DssBridgeNode)을 함께 빌드한다.
function(add_bridge_node target)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
  add_dss_node(${target} SOURCES ${ARG_SOURCES} src/DssBridgeNode.cpp DEPS ${ARG_DEPS})
endfunction()

# 이미지 계열 5개 — JPEG 디코드 변환을 공유한다.
foreach(spec
    "DSSToROSImageNode|src/DSSToROSImage.cpp"
    "DSSToROSStereoCameraColorNode|src/DSSToROSStereoCameraColor.cpp"
    "DSSToROSStereoCameraDepthNode|src/DSSToROSStereoCameraDepth.cpp"
    "DSSToROSStereoCameraInfra1Node|src/DSSToROSStereoCameraInfra1.cpp"
    "DSSToROSStereoCameraInfra2Node|src/DSSToROSStereoCameraInfra2.cpp")
  string(REPLACE "|" ";" parts "${spec}")
  list(GET parts 0 node_target)
  list(GET parts 1 node_source)
  add_bridge_node(${node_target}
    SOURCES ${node_source} src/DssImageConvert.cpp
    DEPS rclcpp sensor_msgs OpenCV)
endforeach()

add_bridge_node(DSSToROSPointCloudNode SOURCES src/DSSToROSPointCloud.cpp DEPS rclcpp sensor_msgs)
add_bridge_node(DSSToROSIMUNode        SOURCES src/DSSToROSIMU.cpp        DEPS rclcpp sensor_msgs)
add_bridge_node(DSSToROSGpsNode        SOURCES src/DSSToROSGps.cpp        DEPS rclcpp sensor_msgs)
add_bridge_node(DSSToROSClockNode      SOURCES src/DSSToROSClock.cpp      DEPS rclcpp rosgraph_msgs)

# 데모는 브리지가 아니라 제어 송신 노드라 공통 골격을 쓰지 않는다.
add_dss_node(DSSDemoNode
  SOURCES src/DSSDemo.cpp src/DSS.VSSClient.cpp
  DEPS rclcpp sensor_msgs)

# /dss/control 토픽을 DSS UDP 제어(:8886)로 전달하는 수신 노드 (jog GUI 등 발행자의 수신측).
add_dss_node(DSSControlNode
  SOURCES src/DSSControlNode.cpp src/DSS.VSSClient.cpp
  DEPS rclcpp)


# ===============================
# 설치
# ===============================

install(
  TARGETS
    DSSToROSImageNode
    DSSToROSClockNode
    DSSToROSPointCloudNode
    DSSToROSIMUNode
    DSSToROSGpsNode
    DSSToROSStereoCameraColorNode
    DSSToROSStereoCameraDepthNode
    DSSToROSStereoCameraInfra1Node
    DSSToROSStereoCameraInfra2Node
    DSSDemoNode
    DSSControlNode
    ${cpp_typesupport_target}        # ← ★ 추가! typesupport .so 설치
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# 메시지 런타임 export
ament_export_dependencies(rosidl_default_runtime)

ament_package()
