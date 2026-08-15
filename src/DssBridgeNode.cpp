#include "DssBridgeNode.h"
#include "defaultGateway.h"

#include <nlohmann/json.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
    /// DSS 가 NATS 를 여는 포트.
    constexpr int kNatsPort = 4222;
}

DssBridgeNode::DssBridgeNode(const std::string& node_name, const std::string& heartbeat_subject)
    : rclcpp::Node(node_name), heartbeat_subject_(heartbeat_subject)
{
    const std::string nats_url = "nats://" + getDefaultGateway() + ":" + std::to_string(kNatsPort);
    RCLCPP_INFO(get_logger(), "%s", nats_url.c_str());

    const natsStatus s = natsConnection_ConnectTo(&conn_, nats_url.c_str());
    if (s != NATS_OK) {
        RCLCPP_FATAL(get_logger(), "NATS 연결 실패 (%s): %s",
                     nats_url.c_str(), natsStatus_GetText(s));
        throw std::runtime_error("NATS 연결 실패");
    }

    heartbeat_timer_ = create_wall_timer(kHeartbeatPeriod,
                                         std::bind(&DssBridgeNode::onTick, this));
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
