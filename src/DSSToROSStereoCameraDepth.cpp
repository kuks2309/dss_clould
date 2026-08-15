#include "DssBridgeNode.h"
#include "DssImageConvert.h"

#include <sensor_msgs/msg/image.hpp>

class DSSToROSStereoCameraDepthNode : public DssBridgeNode
{
public:
    DSSToROSStereoCameraDepthNode() : DssBridgeNode("DSSToROSStereoCameraDepthNode", "dss.dssToROSStereoCameraDepth.heartBeat")
    {
        // 비압축 rgb8 프레임은 640x480x3 = 921,600 B 라 RTPS 조각화 전송을 탄다.
        // best-effort 는 조각 1 개 손실이 샘플 전체 손실이 되어 프레임 유실이 크다 —
        // 이미지 토픽만 재전송이 있는 reliable 을 쓴다.
        pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/dss/sensor/camera/depth/image_raw", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        subscribeTopicRaw("dss.sensor.camera.stereo.depth",
            [this](const std::string&, const char* bytes, int len)
            {
                dss::DSSImage img_msg;
                if (!img_msg.ParseFromArray(bytes, len)) {
                    RCLCPP_WARN(get_logger(), "DSSImage protobuf 파싱 실패 — 프레임 폐기");
                    return;
                }
                pub_->publish(dssImageToRosImage(img_msg, kFrameId));
            });

        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.camera.stereo.depth → [ROS2]/dss/sensor/camera/depth/image_raw");
    }

private:
    /// DSS 카메라 좌표계 이름.
    static constexpr const char* kFrameId = "camera";

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    return runBridgeNode<DSSToROSStereoCameraDepthNode>(argc, argv, "DSSToROSStereoCameraDepthNode");
}
