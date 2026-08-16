#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nats/nats.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * DSS 가 NATS 로 내보내는 protobuf 스트림을 ROS2 토픽으로 옮기는 브리지 노드들의 공통 골격.
 *
 * 파생 클래스가 갖는 것은 그 노드 고유의 것뿐이다 — 퍼블리셔, 구독할 subject, 변환 함수.
 * NATS 연결·구독 등록·콜백 수명 관리·heartbeat 발행은 이 클래스가 맡는다.
 */
class DssBridgeNode : public rclcpp::Node
{
public:
    /// subject, 페이로드 시작 주소, 바이트 길이. 페이로드는 콜백 반환 후 무효가 된다.
    using TopicHandler = std::function<void(const std::string& subject, const char* data, int len)>;

protected:
    /**
     * NATS 에 연결하고 heartbeat 타이머를 건다.
     *
     * @param node_name          ROS2 노드 이름
     * @param heartbeat_subject  생존 신호를 발행할 NATS subject
     * @throws std::runtime_error 연결 실패 시. 연결 없이 살아 있는 노드는 감시 측에서
     *         정상 동작과 구별되지 않으므로 기동 자체를 실패시킨다.
     */
    DssBridgeNode(const std::string& node_name, const std::string& heartbeat_subject);

    ~DssBridgeNode() override;

    /**
     * NATS subject 를 구독한다. handler 와 그 컨텍스트의 수명은 이 객체가 보장한다.
     *
     * 파생 클래스는 퍼블리셔를 만든 **뒤에** 호출해야 한다 — 구독이 활성화된 순간부터
     * NATS 스레드가 handler 를 부를 수 있어서, 그 전에 퍼블리셔가 없으면 역참조가 난다.
     *
     * @param queue  비어 있지 않으면 queue group 구독
     * @return 구독 등록 성공 여부
     */
    bool subscribeTopicRaw(const std::string& subject, TopicHandler handler, const char* queue = nullptr);

private:
    struct TopicCtx { TopicHandler* fn; };

    static void sOnTopicRaw(natsConnection* nc, natsSubscription* sub, natsMsg* msg, void* closure);
    static void sOnDisconnected(natsConnection* nc, void* closure);
    static void sOnReconnected(natsConnection* nc, void* closure);
    static std::string currentTimeIso8601();

    void onTick();
    void publishHeartBeat();

    /// 노드 하나가 보유할 수 있는 최대 구독 수.
    static constexpr int kMaxSubs = 64;
    /// heartbeat 발행 주기.
    static constexpr std::chrono::seconds kHeartbeatPeriod{3};

    natsConnection*   conn_ = nullptr;
    natsSubscription* subs_[kMaxSubs]{};
    int               sub_count_ = 0;

    std::vector<std::unique_ptr<TopicHandler>> handlers_;
    std::vector<std::unique_ptr<TopicCtx>>     contexts_;
    rclcpp::TimerBase::SharedPtr               heartbeat_timer_;
    std::string                                heartbeat_subject_;
};

/**
 * 브리지 노드용 main 골격.
 *
 * 노드 생성 실패(대개 NATS 연결 실패)를 종료 코드 1 로 드러낸다 — 실패한 노드가
 * 조용히 살아 있으면 launch·감독자·CI 어디에서도 알 수 없다.
 * nats_Close() 는 라이브러리 전역 종료라 프로세스 끝에서 한 번만 부른다.
 */
template <typename NodeT>
int runBridgeNode(int argc, char** argv, const char* logger_name)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<NodeT>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger(logger_name), "노드 기동 실패: %s", e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    nats_Close();
    return exit_code;
}
