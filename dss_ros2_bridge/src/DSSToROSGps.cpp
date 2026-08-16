#include "DssBridgeNode.h"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "dss.pb.h"

class DSSToROSGpsNode : public DssBridgeNode
{
public:
    DSSToROSGpsNode() : DssBridgeNode("DSSToROSGpsNode", "dss.dssToROSGps.heartBeat")
    {
        // 같은 실행파일로 GPS·GPSFront·GPSRear 3 인스턴스를 띄운다(launch 파라미터로 구분).
        // 기본값 = 종전 단일 GPS 동작.
        const auto ros_topic    = declare_parameter("ros_topic", std::string("/dss/sensor/gps/fix"));
        const auto nats_subject = declare_parameter("nats_subject", std::string("dss.sensor.gps"));

        pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(ros_topic, rclcpp::SensorDataQoS());

        subscribeTopicRaw(nats_subject,
            [this](const std::string&, const char* bytes, int len)
            {
                dss::DSSGPS gps_msg;
                if (!gps_msg.ParseFromArray(bytes, len)) {
                    RCLCPP_WARN(get_logger(), "DSSGPS protobuf 파싱 실패 — 프레임 폐기");
                    return;
                }
                pub_->publish(createNavSatFix(gps_msg));
            });

        RCLCPP_INFO(get_logger(), "[NATS]%s → [ROS2]%s", nats_subject.c_str(), ros_topic.c_str());
    }

private:
    /// NavSatFix.position_covariance 의 원소 수(3x3 row-major).
    static constexpr int kCovarianceSize = 9;

    sensor_msgs::msg::NavSatFix createNavSatFix(const dss::DSSGPS& dss_gps)
    {
        sensor_msgs::msg::NavSatFix ros_gps;

        const double stamp_sec = dss_gps.header().stamp();
        ros_gps.header.stamp    = rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9), RCL_ROS_TIME);
        ros_gps.header.frame_id = dss_gps.header().frame_id();

        ros_gps.status.status  = dss_gps.status().status();
        ros_gps.status.service = dss_gps.status().service();

        ros_gps.latitude  = dss_gps.latitude();
        ros_gps.longitude = dss_gps.longitude();
        ros_gps.altitude  = dss_gps.altitude();

        if (dss_gps.position_covariance_size() == kCovarianceSize) {
            for (int i = 0; i < kCovarianceSize; ++i) {
                ros_gps.position_covariance[i] = dss_gps.position_covariance(i);
            }
        }
        ros_gps.position_covariance_type = dss_gps.position_covariance_type();

        return ros_gps;
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    return runBridgeNode<DSSToROSGpsNode>(argc, argv, "DSSToROSGpsNode");
}
