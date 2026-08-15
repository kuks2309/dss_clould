# 2026-08-16 — 코드 리뷰 지적 반영 (High 7건 + Medium 5건)

- 결정 근거: [ADR 2026-08-16 — 실패를 감추지 않는 방향으로 동작 변경](../adr/2026-08-16-review-fixes.md)
- 대상: `src/dss_ros2_bridge` 소스 13개 + `CMakeLists.txt` + `package.xml`
- 검증: 빌드 성공(13.0 s) + 실기 재구동(NATS `100.80.80.15:4222`, DSS 시뮬레이터 가동) + 실패 경로 격리 시험

## 반영한 지적

| 리뷰 항목 | 무엇을 고쳤나 | 파일 |
| --- | --- | --- |
| High #1 퍼블리셔 순서 race | `create_publisher` 를 `subscribeTopicRaw` **앞으로** 이동. 구독이 활성화된 시점에 `pub_` 이 이미 유효하다 | 브리지 9개 |
| High #2 `sprintf` 스택 오버플로 | `snprintf` + 잘림 검사. 넘치면 `NATS_INVALID_ARG` 로 콜백하고 요청을 보내지 않는다 | DSS.VSSClient.cpp |
| High #3 closure 캐스팅 UB | `std::function<void(std::string, std::vector<uint8_t>)>*` → `OnRawMessage*` (실제 생성 타입) | DSS.VSSClient.cpp |
| High #4 `start()` 무시 | 싱글톤 생성자에서 자동 연결 제거 → `start()` 가 실제로 연결한다. 데모는 반환값을 검사하고 실패 시 종료 | DSS.VSSClient.cpp · DSSDemo.cpp |
| High #5 좀비 노드 | 연결 실패 시 `RCLCPP_FATAL` + `throw` → `main` 이 잡아 **종료 코드 1** | 브리지 9개 + DSSDemo.cpp |
| High #6 빌드 의존성 누락 | `package.xml` 에 sensor_msgs·rosgraph_msgs·OpenCV·protobuf·nlohmann-json 선언. CMake 에 `rosgraph_msgs`·`Threads` `find_package`, `NATS_LIB` 미발견 시 `FATAL_ERROR`, 미사용 `find_package` 6개 제거, `Protobuf_DIR` 을 CACHE 로 | package.xml · CMakeLists.txt |
| High #7 `unsubscribe` 이중 해제 | 헤더 inline → `.cpp` 정의로 옮기고 전역 클로저 맵에서 항목을 지운 뒤 파괴 | DSS.VSSClient.{h,cpp} |
| 신규 High 쿼터니언 비정규화 | `normalizeOrientation()` 신설 — 노름 0 이면 단위 쿼터니언, `1e-3` 이상 벗어나면 경고(throttle) 후 정규화 | DSSToROSIMU.cpp |
| Critical→Medium 재평가 경계 검증 | `createPointCloud2` 진입부에 `point_step >= 14` · `data.size()/step >= width` 가드. 위반 시 예외 → 프레임 폐기 | DSSToROSPointCloud.cpp |
| Medium #1 고아 구독 | 데모의 라이다 구독 토픽 `/dss/sensor/lidar` → `/dss/sensor/lidar3d` | DSSDemo.cpp |
| Medium #3 비상수 포맷 문자열 | `RCLCPP_INFO(logger, str.c_str())` → `RCLCPP_INFO(logger, "%s", str.c_str())` (11곳) | 브리지 9개 |
| Medium #5 중복 함수 | `subscribeSensor` 가 `subscribe` 로 위임 (본문 중복 제거, 공개 API 유지) | DSS.VSSClient.cpp |
| Medium #6 dead code | `createPointCloud` 121줄 삭제(호출부 0). 미사용 지역변수 `r` 삭제 | DSSToROSPointCloud.cpp |
| Medium #7 하드코딩 주소 | 데모가 `"100.80.80.15"` 리터럴 대신 `getDefaultGateway()` 사용 — 브리지와 같은 출처 | DSSDemo.cpp |
| (부수) package.xml 스키마 위반 | `test_depend` 가 `member_of_group` 뒤에 있어 format 3 스키마 오류 → 순서 교정 | package.xml |

## 함수·전역 인벤토리 변동

| 구분 | 대상 | 위치 |
| --- | --- | --- |
| 삭제 | `DSSToROSPointCloudNode::createPointCloud` (리뷰 함수 #10) | — |
| 신규 | `DSSToROSIMUNode::normalizeOrientation` | src/DSSToROSIMU.cpp:60 |
| 신규 | `buildVssJson` (익명 네임스페이스) | src/DSS.VSSClient.cpp:178 |
| 이동 | `DSSVssClient::unsubscribe` 헤더 inline → `.cpp` 정의 | src/DSS.VSSClient.cpp:365 |
| 신규 상수 | `kVssJsonCap`(256) · `kQuatNormTolerance`(1e-3) · `kBytesReadPerPoint`(14) | 각 파일 |
| 신규 멤버 | `DSSToROSIMUNode::throttle_clock_` (RCL_STEADY_TIME) | src/DSSToROSIMU.cpp:44 |

갱신된 표는 [코드 리뷰 2026-08-16-0815 delta](../code_review/dss_ros2_bridge/2026-08-16-0815.md) 참조.

## 검증 결과

| 항목 | 결과 |
| --- | --- |
| 빌드 | `colcon build --symlink-install` 성공 (13.0 s) |
| 노드 5개 기동 | 전부 생존 · lidar3d 7.1 Hz · camera/rgb 5.3 Hz · imu 141 Hz · /clock 141 Hz |
| 쿼터니언 정규화 | 200 샘플 노름 min/max/avg **모두 1.000000** (수정 전 0.707131) |
| 데모 접속 대상 | `[VssClient] connected: nats://100.80.80.15:4222` (수정 전에는 기본 인자 `172.25.96.1` 로 붙어 도달 불가였음) |
| 실패 경로 | 네트워크 네임스페이스로 격리해 NATS 도달 불가 상태 재현 → `[FATAL] NATS 연결 실패 … No server available` → `[FATAL] 노드 기동 실패` → **종료 코드 1** (수정 전: 종료 코드 0 + 무동작 생존) |

## 남은 것

- **High #8 9벌 골격 중복** — 미반영. 기반 클래스로 묶는 리팩터링이라 이번 수정들과 섞으면 회귀 원인 분리가 어렵다. 별도 작업 단위로 둔다.
- **쿼터니언 원천** — 브리지는 정규화해 발행하지만 노름 0.707 의 원인(성분 누락 의심)은 DSS 측 확인이 남아 있다. 정규화된 값도 원천이 틀리면 여전히 틀린 자세다.
- **스테레오 4개 노드** — 원천 subject 가 없어 무동작. `launch.py` 조정 또는 미수신 경고는 미반영.
- **`launch.py` `respawn`** — 연결 실패가 이제 프로세스 종료이므로, NATS 가 늦게 뜨는 환경에서는 재기동 정책이 필요하다.
