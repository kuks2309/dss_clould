# 2026-08-16 — 브리지 골격 기반 클래스 추출 (리뷰 High #8 해소)

- 결정 근거: [ADR 2026-08-16 — 브리지 노드 공통 골격을 기반 클래스로 추출](../adr/2026-08-16-bridge-base-class.md)
- 인벤토리: [코드 리뷰 2026-08-16 08:30 delta](../code_review/dss_ros2_bridge/2026-08-16-0830.md)
- 검증: 빌드 성공(10.9 s) + 브리지 9개 전부 기동 + 동작 보존 계약 11항목 대조

## 무엇을 바꿨나

브리지 노드 9개가 각자 복제해 갖던 골격을 기반 클래스 하나로 모았다.

| 신규 파일 | 내용 |
| --- | --- |
| `src/DssBridgeNode.h` | `DssBridgeNode` 선언 + `runBridgeNode<NodeT>()` main 골격 템플릿 |
| `src/DssBridgeNode.cpp` | NATS 연결·구독 등록·핸들러 수명·heartbeat 구현 |
| `src/DssImageConvert.h` | `dssImageToRosImage()` 선언 |
| `src/DssImageConvert.cpp` | JPEG → rgb8 변환 (이미지 5개 노드 공유) |

파생 노드에 남은 것은 **퍼블리셔·구독 subject·변환 함수**뿐이다.

| 파일 | 이전 | 이후 |
| --- | --- | --- |
| DSSToROSImage.cpp | 199줄 | 41줄 |
| DSSToROSStereoCameraColor.cpp | 198줄 | 41줄 |
| DSSToROSStereoCameraDepth.cpp | 199줄 | 41줄 |
| DSSToROSStereoCameraInfra1.cpp | 198줄 | 41줄 |
| DSSToROSStereoCameraInfra2.cpp | 198줄 | 41줄 |
| DSSToROSIMU.cpp | 202줄 | 114줄 |
| DSSToROSPointCloud.cpp | 219줄 | 128줄 |
| DSSToROSGps.cpp | 189줄 | 65줄 |
| DSSToROSClock.cpp | 183줄 | 53줄 |
| **노드 9개 합계** | **1,755줄** | **565줄** |
| 신규 공통 4파일 | — | 272줄 |
| **총계** | **1,755줄** | **837줄 (52% 감소)** |

`CMakeLists.txt` 의 실행 타깃 블록 10벌도 `add_dss_node()`·`add_bridge_node()` 함수와 `foreach` 로 정리했다(148줄 → 46줄).

## 곁들여 해결된 지적

- **`nats_Close()` 위치** — 노드 소멸자가 부르던 라이브러리 전역 종료를 `runBridgeNode()` 끝으로 이동. 한 프로세스에 노드가 둘 이상 올라가는 구성에서 먼저 소멸하는 노드가 나머지 연결을 끊던 문제가 구조적으로 사라졌다.
- **`MAX_SUBS` 매크로 9벌** → `DssBridgeNode::kMaxSubs` 상수 1개.
- **매직 넘버** — 역양자화 기준(100 m·1.0), 프레임 델타(5 ms), covariance 크기(9)를 명명 상수로.
- **파싱 실패 로그** — `std::cerr` → `RCLCPP_WARN`. ROS 노드의 진단은 `/rosout` 으로 나가야 감시 측이 본다. **동작 보존 원칙에서 의도적으로 벗어난 유일한 항목**이며, 발행 데이터에는 영향이 없다.

## 검증 결과 (동작 보존 계약)

| 항목 | 기준 | 결과 |
| --- | --- | --- |
| 노드 생존 | — | 9/9 |
| 발행자 수 | 토픽당 1 | 9개 토픽 전부 1 |
| 발행 QoS | 이미지 RELIABLE · 나머지 BEST_EFFORT | 동일 |
| lidar3d / rgb / imu / clock | 7.1 / 5.3 / 141 / 141 Hz | 7.095 / 5.202 / 141.4 / 141.2 Hz |
| IMU orientation 노름 | 1.000000 | 200 샘플 전부 1.000000 |
| heartbeat subject 9종 | — | NATS 관측으로 전부 보존 확인 |
| NATS 연결 실패 | FATAL 2줄 + 종료 코드 1 | 동일 |

## 남은 것

- **heartbeat subject 명명 불일치** — Infra1·Infra2 만 `dss.DSSToROS…Node.heartBeat` 형식이다. 이번에 값을 바꾸지 않았다(발행 subject 변경은 감시 측 영향 + 동작 보존 원칙). 이제 생성자 인자 한 줄이라 바꾸기는 쉽다.
- **이미지 `frame_id`·depth/infra 인코딩** — `dssImageToRosImage(msg, frame_id)` 로 인자화해 두었으나 값은 그대로 `"camera"`·`rgb8` 이다.
- **단위 테스트 0건** — 변환 함수가 공용 파일로 나와 테스트하기 쉬워졌다(`dssImageToRosImage`·`createPointCloud2` 는 노드 없이 호출 가능).
