#include "DSS.VSSClient.h"
#include "DssBridgeNode.h"
#include "defaultGateway.h"
#include "dss_ros2_bridge/msg/dss_control.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

// /dss/control 명령 토픽을 DSS 제어 UDP(DssSetControl, :8886)로 전달하는 수신 노드.
// DSS 는 주기 제어 스트림을 기대하므로 마지막 수신값을 타이머로 계속 내보낸다.
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

        // 명령 토픽은 reliable — 제어 유실은 조용한 오동작이 된다. 발행측(jog GUI)과 일치해야 연결된다.
        sub_ = create_subscription<dss_ros2_bridge::msg::DssControl>(
            "/dss/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](dss_ros2_bridge::msg::DssControl::SharedPtr msg) {
                last_cmd_ = *msg;
                last_rx_ = std::chrono::steady_clock::now();
            });

        timer_ = create_wall_timer(std::chrono::milliseconds(kSendPeriodMs), [this]() { sendControl(); });

        RCLCPP_INFO(get_logger(), "/dss/control -> UDP %s:8886 전달 시작 (%d ms 주기, dead-man %d ms)",
                    dss_host.c_str(), kSendPeriodMs, kCommandTimeoutMs);
    }

  private:
    // 마지막 수신 명령을 UDP 로 전달한다. 수신이 kCommandTimeoutMs 를 넘겨 끊기면 0 명령을
    // 보낸다(dead-man) — 발행자가 죽었을 때 차량이 마지막 명령으로 계속 움직이는 것을 막는다.
    void sendControl()
    {
        // steady clock 기준 — sim time 정지·점프에 dead-man 판정이 흔들리면 안 된다.
        const auto age = std::chrono::steady_clock::now() - last_rx_;
        auto &vss = DSSVssClient::singleton();
        if (age < std::chrono::milliseconds(kCommandTimeoutMs))
        {
            vss.setDriveControl(last_cmd_.throttle, last_cmd_.steer, last_cmd_.brake, last_cmd_.target_gear);
        }
        else
        {
            vss.setDriveControl(0.0F, 0.0F, 0.0F);
        }
    }

    static constexpr int kSendPeriodMs = 50;      // UDP 송신 주기 (20 Hz)
    static constexpr int kCommandTimeoutMs = 500; // 이보다 오래 수신이 없으면 0 명령

    dss_ros2_bridge::msg::DssControl last_cmd_;
    std::chrono::steady_clock::time_point last_rx_{}; // epoch 초기값 — 기동 직후엔 0 명령
    rclcpp::Subscription<dss_ros2_bridge::msg::DssControl>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    return runBridgeNode<DSSControlNode>(argc, argv, "DSSControlNode");
}
