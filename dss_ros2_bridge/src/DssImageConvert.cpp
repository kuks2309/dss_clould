#include "DssImageConvert.h"

#include <opencv2/opencv.hpp>
#include <rclcpp/time.hpp>

#include <cstring>
#include <stdexcept>
#include <vector>

sensor_msgs::msg::Image dssImageToRosImage(const dss::DSSImage& nats_msg, const std::string& frame_id)
{
    const double stamp_sec = nats_msg.header().stamp();

    const auto* jpeg_data = reinterpret_cast<const uint8_t*>(nats_msg.data().data());
    const size_t jpeg_size = nats_msg.data().size();

    const std::vector<uint8_t> jpeg_buffer(jpeg_data, jpeg_data + jpeg_size);
    const cv::Mat bgr = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        throw std::runtime_error("JPEG decode failed!");
    }

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

    sensor_msgs::msg::Image msg;
    msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9), RCL_ROS_TIME);
    msg.header.frame_id = frame_id;

    msg.height       = rgb.rows;
    msg.width        = rgb.cols;
    msg.encoding     = "rgb8";
    msg.is_bigendian = false;
    msg.step         = rgb.cols * 3;

    msg.data.resize(static_cast<size_t>(rgb.rows) * rgb.cols * 3);
    std::memcpy(msg.data.data(), rgb.data, msg.data.size());

    return msg;
}
