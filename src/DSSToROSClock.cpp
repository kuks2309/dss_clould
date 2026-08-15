#include "DssBridgeNode.h"

#include <rosgraph_msgs/msg/clock.hpp>

#include "dss.pb.h"

class DSSToROSClockNode : public DssBridgeNode
{
public:
    DSSToROSClockNode() : DssBridgeNode("DSSToROSClockNode", "dss.dssToROSClock.heartBeat")
    {
        // rclcpp TimeSource 가 /clock 을 구독할 때 쓰는 기본 프로파일과 동일해야 짝이 맞는다
        // (KeepLast(1) · BEST_EFFORT · Volatile — rclcpp/time_source.hpp, rclcpp/qos.hpp)
        pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

        subscribeTopicRaw("dss.simTime.clock",
            [this](const std::string&, const char* bytes, int len)
            {
                dss::DssOneFrameFixedRateResult msg;
                if (!msg.ParseFromArray(bytes, len)) {
                    RCLCPP_WARN(get_logger(), "DssOneFrameFixedRateResult protobuf 파싱 실패 — 프레임 폐기");
                    return;
                }
                if (get_parameter("use_sim_time").as_bool()) {
                    pub_->publish(createClock(msg));
                }
            });

        RCLCPP_INFO(get_logger(), "[NATS]dss.simTime.clock → [ROS2]/clock");
    }

private:
    /// DSS 고정 프레임 델타(ns). 시뮬레이터의 custom_delta_time 과 일치해야 한다.
    static constexpr uint64_t kFrameDeltaNs = 5'000'000ULL;
    static constexpr uint64_t kNsPerSec     = 1'000'000'000ULL;

    static rosgraph_msgs::msg::Clock createClock(const dss::DssOneFrameFixedRateResult& msg)
    {
        const uint64_t sim_time_ns = static_cast<uint64_t>(msg.frame_count()) * kFrameDeltaNs;

        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec     = static_cast<int32_t>(sim_time_ns / kNsPerSec);
        clock_msg.clock.nanosec = static_cast<uint32_t>(sim_time_ns % kNsPerSec);
        return clock_msg;
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    return runBridgeNode<DSSToROSClockNode>(argc, argv, "DSSToROSClockNode");
}
