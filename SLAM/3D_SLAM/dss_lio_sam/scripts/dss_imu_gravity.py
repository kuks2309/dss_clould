#!/usr/bin/env python3
"""DSS IMU 중력 합성 릴레이 — /dss/sensor/imu → /dss/sensor/imu_gravity.

왜 있나 (ADR 2026-08-16 D5):
    DSS IMU 는 실측상 linear_acceleration ≈ 0 (중력 미포함), angular_velocity = 0 이다.
    LIO-SAM `imuPreintegration` 은 gtsam 에 가속도를 그대로 넣으므로, 가속도 0 은 중력
    모델 (0,0,-9.80511) 과 결합해 자유낙하로 해석된다 → 추정 bias 가 발산해
    `failureDetection()` 이 최적화를 반복 리셋한다.

무엇을 하나 (이것 하나뿐):
    orientation 쿼터니언으로 월드 중력 g_w=(0,0,+g) 를 바디 좌표로 돌려
    linear_acceleration 에 채운다. 정지 상태의 가속도계는 중력 반작용 +g 를 읽는
    것이 물리적으로 옳은 값이다. angular_velocity·orientation·covariance 는
    원본 그대로 통과시킨다.

무엇이 아닌가:
    DSS IMU 결함의 수정이 아니라 우회다. 각속도 0 은 그대로 남는다 — 플랫폼이
    실제로 회전해도 LIO-SAM 은 그 회전을 보지 못한다. 근본 수정은 DSS 측 발행이다.
    (debt: docs/debt/registry.md)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class DssImuGravity(Node):
    def __init__(self):
        super().__init__('dss_imu_gravity')
        # params_dss.yaml 의 imuGravity 와 같은 값이어야 preintegration 중력 모델과 상쇄된다.
        self.declare_parameter('gravity', 9.80511)          # m/s^2
        self.declare_parameter('input_topic', '/dss/sensor/imu')
        self.declare_parameter('output_topic', '/dss/sensor/imu_gravity')

        self.g = self.get_parameter('gravity').get_parameter_value().double_value
        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # 브리지가 SensorDataQoS(BEST_EFFORT) 로 발행 — 같은 프로파일로 받고 같은 프로파일로 낸다.
        self.pub = self.create_publisher(Imu, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(Imu, in_topic, self.on_imu,
                                            qos_profile_sensor_data)
        self.get_logger().info(
            f'{in_topic} → {out_topic} · 중력 {self.g} m/s² 합성 (각속도·자세는 무가공 통과)')

    def on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        # R(q)^T · (0,0,g) — 월드 중력 반작용을 바디 좌표로. R^T 의 3열만 계산한다.
        #   R^T(0,0,g) = g · (R31, R32, R33)
        r31 = 2.0 * (q.x * q.z - q.w * q.y)
        r32 = 2.0 * (q.y * q.z + q.w * q.x)
        r33 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        # 원본 가속도(≈0)에 중력 반작용을 더한다 — DSS 가 훗날 실측치를 채우면 그대로 합산된다.
        out.linear_acceleration.x = msg.linear_acceleration.x + self.g * r31
        out.linear_acceleration.y = msg.linear_acceleration.y + self.g * r32
        out.linear_acceleration.z = msg.linear_acceleration.z + self.g * r33
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DssImuGravity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch 의 SIGINT 종료 경로 — 예외가 새면 종료 코드 1 로 죽어 launch 가 ERROR 를 찍는다
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
