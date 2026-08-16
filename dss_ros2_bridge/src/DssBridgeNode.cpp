#include "DssBridgeNode.h"
#include "defaultGateway.h"

#include <nlohmann/json.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
    /// DSS 가 NATS 를 여는 포트.
    constexpr int kNatsPort = 4222;
    /// 연결 상실 후 재연결 시도 간격.
    constexpr int kReconnectWaitMs = 2000;
}

DssBridgeNode::DssBridgeNode(const std::string& node_name, const std::string& heartbeat_subject)
    : rclcpp::Node(node_name)
{
    // 파라미터가 생성자 인자를 덮을 수 있다 — 같은 실행파일을 launch 에서
    // 여러 인스턴스로 띄울 때 인스턴스별 heartbeat 를 구분하기 위함.
    heartbeat_subject_ = declare_parameter("heartbeat_subject", heartbeat_subject);

    const std::string nats_url = "nats://" + getDefaultGateway() + ":" + std::to_string(kNatsPort);
    RCLCPP_INFO(get_logger(), "%s", nats_url.c_str());

    // 최초 연결은 fail-fast(연결 없이 살아 있는 노드 금지 계약). 연결 수립 후
    // 서버가 내려가면 무제한 재시도 — 시뮬레이터 재시작(수 분~수 시간)을 넘겨야 한다.
    // nats.c 기본값(60회 후 포기)은 밤사이 재시작에서 브리지 전멸을 실측시켰다.
    natsOptions* opts = nullptr;
    natsStatus s = natsOptions_Create(&opts);
    if (s == NATS_OK) s = natsOptions_SetURL(opts, nats_url.c_str());
    if (s == NATS_OK) s = natsOptions_SetMaxReconnect(opts, -1);
    if (s == NATS_OK) s = natsOptions_SetReconnectWait(opts, kReconnectWaitMs);
    if (s == NATS_OK) s = natsOptions_SetDisconnectedCB(opts, sOnDisconnected, this);
    if (s == NATS_OK) s = natsOptions_SetReconnectedCB(opts, sOnReconnected, this);
    if (s == NATS_OK) s = natsConnection_Connect(&conn_, opts);
    natsOptions_Destroy(opts);

    if (s != NATS_OK) {
        RCLCPP_FATAL(get_logger(), "NATS 연결 실패 (%s): %s",
                     nats_url.c_str(), natsStatus_GetText(s));
        throw std::runtime_error("NATS 연결 실패");
    }

    heartbeat_timer_ = create_wall_timer(kHeartbeatPeriod,
                                         std::bind(&DssBridgeNode::onTick, this));
}

void DssBridgeNode::sOnDisconnected(natsConnection*, void* closure)
{
    auto* self = static_cast<DssBridgeNode*>(closure);
    RCLCPP_WARN(self->get_logger(), "NATS 연결 끊김 — %d ms 간격 무제한 재연결 대기", kReconnectWaitMs);
}

void DssBridgeNode::sOnReconnected(natsConnection*, void* closure)
{
    auto* self = static_cast<DssBridgeNode*>(closure);
    RCLCPP_INFO(self->get_logger(), "NATS 재연결 완료 — 구독은 라이브러리가 자동 복구");
}

DssBridgeNode::~DssBridgeNode()
{
    for (int i = 0; i < sub_count_; ++i) {
        natsSubscription_Destroy(subs_[i]);
    }
    natsConnection_Destroy(conn_);
    // nats_Close() 는 부르지 않는다 — 라이브러리 전역 종료라 한 프로세스에 노드가
    // 둘 이상 올라간 구성에서 먼저 소멸하는 노드가 나머지의 연결을 끊는다.
}

bool DssBridgeNode::subscribeTopicRaw(const std::string& subject, TopicHandler handler, const char* queue)
{
    if (sub_count_ >= kMaxSubs) {
        RCLCPP_ERROR(get_logger(), "구독 한도(%d) 초과 — '%s' 등록 실패", kMaxSubs, subject.c_str());
        return false;
    }

    handlers_.emplace_back(std::make_unique<TopicHandler>(std::move(handler)));
    contexts_.emplace_back(std::make_unique<TopicCtx>(TopicCtx{handlers_.back().get()}));
    auto* ctx = contexts_.back().get();

    natsSubscription* sub = nullptr;
    const natsStatus s = (queue && *queue)
        ? natsConnection_QueueSubscribe(&sub, conn_, subject.c_str(), queue, sOnTopicRaw, ctx)
        : natsConnection_Subscribe(&sub, conn_, subject.c_str(), sOnTopicRaw, ctx);

    if (s != NATS_OK) {
        RCLCPP_ERROR(get_logger(), "구독 실패 '%s': %s", subject.c_str(), natsStatus_GetText(s));
        contexts_.pop_back();
        handlers_.pop_back();
        return false;
    }

    subs_[sub_count_++] = sub;
    return true;
}

void DssBridgeNode::sOnTopicRaw(natsConnection*, natsSubscription*, natsMsg* msg, void* closure)
{
    auto* ctx = static_cast<TopicCtx*>(closure);
    const std::string subject = natsMsg_GetSubject(msg);
    const char* data = natsMsg_GetData(msg);
    const int len = natsMsg_GetDataLength(msg);

    // 이 콜백은 NATS 라이브러리 스레드에서 실행된다. 예외가 새어 나가면 그 스레드가
    // 죽어 이후 모든 구독이 조용히 멎으므로 여기서 막는다.
    try {
        if (data && len > 0) (*ctx->fn)(subject, data, len);
    } catch (const std::exception& e) {
        std::cerr << "sOnTopicRaw error: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "sOnTopicRaw unknown error\n";
    }

    natsMsg_Destroy(msg);
}

void DssBridgeNode::publishHeartBeat()
{
    if (!conn_) return;

    nlohmann::json message;
    message["timeStamp"] = currentTimeIso8601();
    message["status"]    = "alive";

    const natsStatus s = natsConnection_PublishString(conn_, heartbeat_subject_.c_str(),
                                                      message.dump().c_str());
    if (s != NATS_OK) {
        std::cerr << "Heartbeat publish error: " << natsStatus_GetText(s) << std::endl;
    }
}

std::string DssBridgeNode::currentTimeIso8601()
{
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto t = system_clock::to_time_t(now);
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%S");
    oss << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";
    return oss.str();
}

void DssBridgeNode::onTick()
{
    publishHeartBeat();
}
