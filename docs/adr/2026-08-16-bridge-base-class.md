# ADR 2026-08-16 — 브리지 노드 공통 골격을 기반 클래스로 추출

- **Status**: Accepted — 2026-08-16 (실기 재구동 + 발행률 대조로 검증)

## Context

브리지 노드 9개(`DSSToROS*Node`)가 **같은 골격을 각 파일에 복제**해 갖고 있다. 리뷰 High #8 이 지적한 항목이며, 복제 대상은 다음과 같다.

| 복제된 것 | 벌 수 |
| --- | --- |
| `struct NatsClient` (파일 스코프) · `#define MAX_SUBS (64)` · `using json = nlohmann::json;` | 9 |
| `struct TopicCtx` · `using TopicHandler` · 멤버 `nats_`·`topicHandlers_`·`rawCtx_`·`timer_` | 9 |
| `sOnTopicRaw` · `subscribeTopicRaw` · `publishHeartBeat` · `getCurrentTimeISO8601` · `onTick` | 9 |
| 생성자 골격(게이트웨이 조회 → NATS 연결 → 타이머 → 퍼블리셔 → 구독) · 소멸자 · `main` | 9 |
| `createImage` (JPEG 디코드 → rgb8) | 5 |

**대가는 추정이 아니라 이미 관측되었다.**

- 직전 수정([code_updates 2026-08-16-review-fixes](../code_updates/2026-08-16-review-fixes.md))에서 High #1(퍼블리셔 순서)·High #5(좀비 노드)·Medium #3(포맷 문자열)을 **9개 파일에 같은 방식으로 반복 적용**해야 했고, 손으로 하면 어긋날 것이 확실해 변환 스크립트를 따로 작성했다.
- 그 스크립트조차 `DSSToROSPointCloud.cpp` 에서 빗나갔다 — 주석 처리된 `//pub_ = ...` 줄을 실제 퍼블리셔로 오인해 그 파일만 High #1 이 안 고쳐진 채 남았고, 사후 확인에서 잡아 수동으로 고쳤다.
- 복제가 이미 어긋난 증거도 있다: heartbeat subject 명명이 7개는 `dss.dssToROSXxx.heartBeat` 인데 Infra1·Infra2 만 `dss.DSSToROSXxxNode.heartBeat` 다(리뷰 Info #1).

즉 이 중복은 **수정 1건이 9곳에 흩어지고, 그 과정에서 실제로 누락이 발생하는** 상태다.

## Decision

공통 골격을 기반 클래스와 공용 헤더로 추출한다. 실행 타깃 10개 구성은 그대로 둔다(런치·배포 영향 없음).

| 신설 | 내용 |
| --- | --- |
| `src/DssBridgeNode.h` / `.cpp` | `DssBridgeNode : public rclcpp::Node` — NATS 연결·구독 등록·핸들러 수명·heartbeat 타이머·연결 실패 시 예외. 파생 클래스는 **퍼블리셔 생성 + subject 구독 + 변환 함수**만 갖는다 |
| 〃 `runBridgeNode<NodeT>()` | `main` 골격(init → try spin → catch FATAL → shutdown → `nats_Close()`) |
| `src/DssImageConvert.h` / `.cpp` | `dssImageToRosImage()` — 5개 이미지 노드가 쓰던 JPEG→rgb8 변환 |

`nats_Close()` 는 노드 소멸자에서 `runBridgeNode()` 끝으로 옮긴다. 라이브러리 전역 종료를 노드 소멸자가 부르면, 한 프로세스에 노드가 둘 이상 올라가는 구성에서 먼저 소멸하는 노드가 나머지의 NATS 를 끊는다(리뷰 Medium).

**본 변경은 동작 보존(behavior-preserving)을 목표로 한다.** 발행 토픽·QoS·subject·frame_id·heartbeat subject·메시지 내용은 그대로 둔다. 검증은 수정 전후 발행률·메시지 내용 대조로 한다.

## Alternatives

- **현행 유지** — 기각. 위 Context 의 관측(수정 누락 실발생).
- **이미지 5개를 노드 하나 + 파라미터로 통합** — 기각(이번 범위 밖). 실행 타깃이 5개에서 1개로 줄어 런치·배포 절차가 바뀐다. 기반 클래스 추출 후에 별도로 판단할 일이다.
- **템플릿 메타프로그래밍으로 노드 전체 생성** — 기각. 변환 함수 시그니처가 노드마다 달라 얻는 것보다 읽기 어려움이 크다.
- **골격 수정 시 스크립트로 9곳 일괄 적용(현행 방식 유지)** — 기각. 이번에 그 방식이 실제로 한 파일을 빠뜨렸다.

## Consequences

### 이득

- 골격 수정이 **1곳**이 된다. 리뷰 High #1·#5, Medium #3 같은 항목이 다시 나와도 9곳 반복이 없다.
- 노드 파일이 평균 190줄 → 약 70줄. 각 파일에 남는 것은 그 노드 고유의 것(토픽·subject·변환)뿐이라 차이가 눈에 보인다.
- `nats_Close()` 위치가 바로잡혀 컴포지션 구성으로 갈 때의 잠재 결함이 사라진다.
- heartbeat subject 같은 값이 **생성자 인자로 한 줄에 모여** 명명 불일치가 드러난다.

### 비용 / 남는 위험

- **공개 표면 신설**: `DssBridgeNode`·`runBridgeNode`·`dssImageToRosImage` 가 패키지 헤더로 노출된다. 패키지 밖에서 include 하는 사용자는 아직 없다.
- **회귀 위험**: 9개 노드를 동시에 고쳐 쓴다. 검증은 재구동 후 토픽별 발행률·메시지 필드를 수정 전 실측값과 대조해서 한다.
- 상속 계층이 1단 늘어난다(`rclcpp::Node` → `DssBridgeNode` → 각 노드). SW 구조 문서의 클래스 관계도 갱신 필요.
- **명명 불일치(Info #1)는 이번에 고치지 않는다** — 발행되는 subject 이름이 바뀌면 감시 측이 영향을 받고, 그것은 동작 보존 원칙에 어긋난다. 값이 한 줄로 모였으니 별도 작업으로 판단한다.
- `frame_id`·depth/infra 인코딩(리뷰 Medium #4)도 같은 이유로 이번 범위 밖 — 다만 `dssImageToRosImage(msg, frame_id)` 로 인자화해 두어 바꾸기 쉬워졌다.

## Rollback

가역이다.

1. `git revert <커밋>` — 또는 신설 4개 파일 삭제 후 노드 9개를 이전 판으로 되돌린다.
2. `CMakeLists.txt` 의 `src/DssBridgeNode.cpp`·`src/DssImageConvert.cpp` 항목 제거.
3. `colcon build --symlink-install` 후 재기동.

영속 상태·스키마·펌웨어를 건드리지 않는다.
