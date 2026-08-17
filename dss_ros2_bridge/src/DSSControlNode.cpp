#include "DSS.VSSClient.h"
#include "DssBridgeNode.h"
#include "defaultGateway.h"
#include "dss_ros2_bridge/msg/dss_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

// 명령 토픽을 DSS 제어 UDP(DssSetControl, :8886)로 전달하는 수신 노드이자 제어 소스 mux.
// /dss/jog_enabled (latched Bool) 가 true 면 /dss/control/jog(수동)을, false 면 /dss/control(자율)을
// 전달한다 — 중재를 수신측에 두어 발행자들의 규약 준수에 의존하지 않는다 (설계 근거: docs/adr/ jog-enable-mux).
// DSS 는 주기 제어 스트림을 기대하므로 활성 소스의 마지막 수신값을 타이머로 계속 내보낸다.
class DSSControlNode : public rclcpp::Node
{
  public:
    DSSControlNode() : Node("DSSControlNode")
    {
        auto &vss = DSSVssClient::singleton();
        const std::string dss_host = getDefaultGateway();
        const natsStatus vss_status = vss.start(dss_host, 8886, 4222);
        if (vss_status != NATS_OK)
        {
            throw std::runtime_error("DSS VSS 연결 실패 (" + dss_host + "): " + natsStatus_GetText(vss_status));
        }

        // 명령 토픽은 reliable — 제어 유실은 조용한 오동작이 된다. 발행측과 일치해야 연결된다.
        auto_sub_ = create_subscription<dss_ros2_bridge::msg::DssControl>(
            "/dss/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](dss_ros2_bridge::msg::DssControl::SharedPtr msg) {
                last_auto_cmd_ = *msg;
                last_auto_rx_ = std::chrono::steady_clock::now();
            });
        jog_sub_ = create_subscription<dss_ros2_bridge::msg::DssControl>(
            "/dss/control/jog", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](dss_ros2_bridge::msg::DssControl::SharedPtr msg) {
                last_jog_cmd_ = *msg;
                last_jog_rx_ = std::chrono::steady_clock::now();
            });
        // enable 은 latched(transient_local) — 이 노드가 GUI 보다 늦게 떠도 현재 상태를 받는다.
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/dss/jog_enabled", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                if (jog_enabled_ != msg->data)
                {
                    jog_enabled_ = msg->data;
                    RCLCPP_INFO(get_logger(), "제어 소스 전환: %s",
                                jog_enabled_ ? "jog (/dss/control/jog)" : "auto (/dss/control)");
                }
            });

        timer_ = create_wall_timer(std::chrono::milliseconds(kSendPeriodMs), [this]() { sendControl(); });

        RCLCPP_INFO(get_logger(),
                    "/dss/control(자율)·/dss/control/jog(수동) -> UDP %s:8886 전달 시작 "
                    "(%d ms 주기, dead-man %d ms, 소스 선택: /dss/jog_enabled)",
                    dss_host.c_str(), kSendPeriodMs, kCommandTimeoutMs);
    }

  private:
    // 활성 소스(jog_enabled_)의 마지막 수신 명령을 UDP 로 전달한다. 그 소스의 수신이
    // kCommandTimeoutMs 를 넘겨 끊기면 0 명령(dead-man) — 발행자가 죽었을 때 차량이 마지막
    // 명령으로 계속 움직이는 것을 막는다. jog 활성 중 GUI 가 죽어도 자율로 자동 폴백하지
    // 않는다 — 수동 오버라이드 중 무단 자율 전환이 정지보다 위험하다.
    void sendControl()
    {
        // steady clock 기준 — sim time 정지·점프에 dead-man 판정이 흔들리면 안 된다.
        const auto now = std::chrono::steady_clock::now();
        const dss_ros2_bridge::msg::DssControl &cmd = jog_enabled_ ? last_jog_cmd_ : last_auto_cmd_;
        const std::chrono::steady_clock::time_point last_rx = jog_enabled_ ? last_jog_rx_ : last_auto_rx_;
        auto &vss = DSSVssClient::singleton();
        if (now - last_rx < std::chrono::milliseconds(kCommandTimeoutMs))
        {
            vss.setDriveControl(cmd.throttle, cmd.steer, cmd.brake, cmd.target_gear);
        }
        else
        {
            vss.setDriveControl(0.0F, 0.0F, 0.0F);
        }
    }

    static constexpr int kSendPeriodMs = 50;      // UDP 송신 주기 (20 Hz)
    static constexpr int kCommandTimeoutMs = 500; // 이보다 오래 수신이 없으면 0 명령

    dss_ros2_bridge::msg::DssControl last_auto_cmd_;       // /dss/control (자율)
    dss_ros2_bridge::msg::DssControl last_jog_cmd_;        // /dss/control/jog (수동)
    std::chrono::steady_clock::time_point last_auto_rx_{}; // epoch 초기값 — 기동 직후엔 0 명령
    std::chrono::steady_clock::time_point last_jog_rx_{};
    bool jog_enabled_ = false; // latched /dss/jog_enabled — 미수신이면 자율 소스가 기본
    rclcpp::Subscription<dss_ros2_bridge::msg::DssControl>::SharedPtr auto_sub_;
    rclcpp::Subscription<dss_ros2_bridge::msg::DssControl>::SharedPtr jog_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    return runBridgeNode<DSSControlNode>(argc, argv, "DSSControlNode");
}
