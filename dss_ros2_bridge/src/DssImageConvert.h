#pragma once

#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "dss.pb.h"

/**
 * DSS 가 JPEG 로 보내는 프레임을 디코드해 rgb8 Image 로 만든다.
 *
 * stamp 는 DSS 의 초 단위 sim time 을 ns 로 환산한 값이다(`RCL_ROS_TIME`).
 *
 * @param frame_id  발행 노드가 정하는 좌표계 이름
 * @throws std::runtime_error JPEG 디코드 실패 시. 호출자(NATS 콜백)가 잡아 프레임을 버린다.
 */
sensor_msgs::msg::Image dssImageToRosImage(const dss::DSSImage& nats_msg, const std::string& frame_id);
