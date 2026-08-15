#include "DssBridgeNode.h"

#include <sensor_msgs/msg/imu.hpp>

#include <cmath>

#include "dss.pb.h"

class DSSToROSIMUNode : public DssBridgeNode
{
public:
    DSSToROSIMUNode() : DssBridgeNode("DSSToROSIMUNode", "dss.dssToROSImu.heartBeat")
    {
        pub_ = create_publisher<sensor_msgs::msg::Imu>("/dss/sensor/imu", rclcpp::SensorDataQoS());

        subscribeTopicRaw("dss.sensor.imu",
            [this](const std::string&, const char* bytes, int len)
            {
                dss::DSSIMU imu_msg;
                if (!imu_msg.ParseFromArray(bytes, len)) {
                    RCLCPP_WARN(get_logger(), "DSSIMU protobuf 파싱 실패 — 프레임 폐기");
                    return;
                }
                pub_->publish(createImu(imu_msg));
            });

        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.imu → [ROS2]/dss/sensor/imu");
    }

private:
    /// DSS IMU 좌표계 이름.
    static constexpr const char* kFrameId = "imu_link";
    /// 단위 쿼터니언 판정 허용 오차. 부동소수 누적 오차는 통과시키고
    /// 성분 누락 같은 구조적 이상만 걸러내는 크기.
    static constexpr double kQuatNormTolerance = 1e-3;

    sensor_msgs::msg::Imu createImu(const dss::DSSIMU& imu_msg)
    {
        sensor_msgs::msg::Imu imu;

        const double stamp_sec = imu_msg.header().stamp();
        imu.header.stamp    = rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9), RCL_ROS_TIME);
        imu.header.frame_id = kFrameId;

        if (imu_msg.has_orientation()) {
            imu.orientation.x = imu_msg.orientation().x();
            imu.orientation.y = imu_msg.orientation().y();
            imu.orientation.z = imu_msg.orientation().z();
            imu.orientation.w = imu_msg.orientation().w();
        } else {
            imu.orientation.x = 0.0;
            imu.orientation.y = 0.0;
            imu.orientation.z = 0.0;
            imu.orientation.w = 1.0;
        }
        normalizeOrientation(imu);

        imu.angular_velocity.x = imu_msg.angular_velocity().x();
        imu.angular_velocity.y = imu_msg.angular_velocity().y();
        imu.angular_velocity.z = imu_msg.angular_velocity().z();

        imu.linear_acceleration.x = imu_msg.linear_acceleration().x();
        imu.linear_acceleration.y = imu_msg.linear_acceleration().y();
        imu.linear_acceleration.z = imu_msg.linear_acceleration().z();

        // DSS 가 보내는 covariance 배열은 전 원소가 0 이라 유효 정보가 없다.
        // 0 은 "분산 0 = 완전 정확"으로 해석되므로 unknown 규약인 -1 을 쓴다.
        imu.orientation_covariance[0]         = -1.0;
        imu.angular_velocity_covariance[0]    = -1.0;
        imu.linear_acceleration_covariance[0] = -1.0;

        return imu;
    }

    /**
     * ROS/TF2 는 orientation 이 단위 쿼터니언인 것을 전제한다. 이 전제가 깨진 값을
     * 그대로 내보내면 소비자(TF2·로컬라이저)에서 자세가 틀어지거나 거부된다.
     * 노름을 맞춰 쓸 수 있는 값으로 만들되 원천 이상은 경고로 계속 드러낸다 —
     * 성분이 빠진 종류의 이상이면 정규화한 자세도 여전히 틀리기 때문이다.
     */
    void normalizeOrientation(sensor_msgs::msg::Imu& imu)
    {
        auto& q = imu.orientation;
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

        if (norm < 1e-6) {
            RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 5000,
                "orientation 노름이 0 입니다. 단위 쿼터니언으로 대체합니다 — 원천 확인 필요");
            q.x = q.y = q.z = 0.0;
            q.w = 1.0;
            return;
        }

        if (std::abs(norm - 1.0) > kQuatNormTolerance) {
            RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 5000,
                "orientation 이 단위 쿼터니언이 아닙니다 (노름 %.6f). 정규화해 발행합니다 — 원천 확인 필요",
                norm);
            q.x /= norm;
            q.y /= norm;
            q.z /= norm;
            q.w /= norm;
        }
    }

    /// 경고 throttle 용. sim time 이 멈춰도 경고가 계속 나오도록 시스템 단조 시계를 쓴다.
    rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    return runBridgeNode<DSSToROSIMUNode>(argc, argv, "DSSToROSIMUNode");
}
