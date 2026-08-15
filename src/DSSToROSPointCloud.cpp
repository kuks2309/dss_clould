#include "DssBridgeNode.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sstream>
#include <stdexcept>

#include "dss.pb.h"

class DSSToROSPointCloudNode : public DssBridgeNode
{
public:
    DSSToROSPointCloudNode() : DssBridgeNode("DSSToROSPointCloudNode", "dss.dssToROSPointCloud.heartBeat")
    {
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dss/sensor/lidar3d",
                                                               rclcpp::SensorDataQoS());

        subscribeTopicRaw("dss.sensor.lidar",
            [this](const std::string&, const char* bytes, int len)
            {
                dss::DssLidarPointCloud pcd_msg;
                if (!pcd_msg.ParseFromArray(bytes, len)) {
                    RCLCPP_WARN(get_logger(), "DssLidarPointCloud protobuf 파싱 실패 — 프레임 폐기");
                    return;
                }
                pub_->publish(createPointCloud2(pcd_msg));
            });

        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.lidar → [ROS2]/dss/sensor/lidar3d");
    }

private:
    /// DSS 라이다 좌표계 이름.
    static constexpr const char* kFrameId = "lidar_link";
    /// 디코딩이 포인트마다 읽는 바이트 수 — int16 5개(x,y,z,intensity,ring) + float(time).
    static constexpr uint32_t kBytesReadPerPoint = 14;
    /// 위치 성분 역양자화 기준 거리(m).
    static constexpr float kPositionMaxAbs = 100.0f;
    /// intensity 역양자화 기준값(정규화 범위).
    static constexpr float kIntensityMaxAbs = 1.0f;

    /// int16 양자화 값을 실수 범위로 되돌린다.
    static float dequantize(int16_t quantized, float max_abs)
    {
        return (static_cast<float>(quantized) / 32767.0f) * max_abs;
    }

    sensor_msgs::msg::PointCloud2 createPointCloud2(const dss::DssLidarPointCloud& pcd_msg)
    {
        const uint32_t num_points = pcd_msg.width();
        const auto* raw = reinterpret_cast<const uint8_t*>(pcd_msg.data().data());
        const uint32_t step = pcd_msg.point_step();

        // width·point_step·data 는 원격 발행자가 정하는 값이라 정합이 보장되지 않는다.
        // 검증 없이 포인터 산술에 쓰면 버퍼 밖을 읽는다. 나눗셈으로 비교해
        // num_points * step 의 32비트 오버플로를 피한다.
        const size_t data_size = pcd_msg.data().size();
        if (step < kBytesReadPerPoint || data_size / step < num_points) {
            std::ostringstream why;
            why << "DssLidarPointCloud 필드 불일치 — width=" << num_points
                << " point_step=" << step << " data=" << data_size
                << "B (포인트당 최소 " << kBytesReadPerPoint << "B 필요)";
            throw std::runtime_error(why.str());
        }

        sensor_msgs::msg::PointCloud2 msg;

        const double stamp_sec = pcd_msg.header().stamp();
        msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9), RCL_ROS_TIME);
        msg.header.frame_id = kFrameId;

        msg.height       = 1;
        msg.width        = num_points;
        msg.is_bigendian = false;
        msg.is_dense     = true;

        // 필드 구성은 LIO-SAM 이 기대하는 형태(x,y,z,intensity,ring,time)다.
        // point_step·row_step 은 아래 modifier 가 이 구성에 맞춰 채운다.
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(
            6,
            "x",         1, sensor_msgs::msg::PointField::FLOAT32,
            "y",         1, sensor_msgs::msg::PointField::FLOAT32,
            "z",         1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "ring",      1, sensor_msgs::msg::PointField::UINT16,
            "time",      1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(num_points);

        sensor_msgs::PointCloud2Iterator<float>    iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float>    iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float>    iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float>    iter_i(msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_r(msg, "ring");
        sensor_msgs::PointCloud2Iterator<float>    iter_t(msg, "time");

        for (uint32_t i = 0; i < num_points; ++i) {
            const uint8_t* ptr = raw + static_cast<size_t>(i) * step;

            const auto xi = *reinterpret_cast<const int16_t*>(ptr + 0);
            const auto yi = *reinterpret_cast<const int16_t*>(ptr + 2);
            const auto zi = *reinterpret_cast<const int16_t*>(ptr + 4);
            const auto ii = *reinterpret_cast<const int16_t*>(ptr + 6);
            const auto ri = *reinterpret_cast<const int16_t*>(ptr + 8);
            const auto ti = *reinterpret_cast<const float*>(ptr + 10);

            *iter_x = dequantize(xi, kPositionMaxAbs);
            *iter_y = dequantize(yi, kPositionMaxAbs);
            *iter_z = dequantize(zi, kPositionMaxAbs);
            *iter_i = dequantize(ii, kIntensityMaxAbs);
            *iter_r = static_cast<uint16_t>(ri);
            *iter_t = ti;

            ++iter_x; ++iter_y; ++iter_z;
            ++iter_i; ++iter_r; ++iter_t;
        }

        return msg;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    return runBridgeNode<DSSToROSPointCloudNode>(argc, argv, "DSSToROSPointCloudNode");
}
