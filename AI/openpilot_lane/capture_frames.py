#!/usr/bin/env python3
"""DSS 카메라 프레임 캡처 (openpilot 차선 인식 PoC 입력 데이터 수집).

/dss/sensor/camera/rgb (640x480 rgb8, 실측 5.2~7.2 Hz) 를 구독해
frames/%06d.png + stamps.csv 로 저장한다. 함수표: docs/code_review/openpilot_lane/2026-08-17.md
"""
import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class CaptureNode(Node):

    def __init__(self, out_dir: str):
        super().__init__('openpilot_lane_capture')
        self.frames_dir = os.path.join(out_dir, 'frames')
        os.makedirs(self.frames_dir, exist_ok=True)
        self.csv_file = open(os.path.join(out_dir, 'stamps.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['idx', 'stamp_sec'])
        self.idx = 0
        # 브리지 발행 QoS(reliable, KeepLast10)와 일치 — docs/adr/2026-08-16-topic-qos.md
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub = self.create_subscription(Image, '/dss/sensor/camera/rgb',
                                            self.on_image, qos)

    def on_image(self, msg: Image):
        if msg.encoding != 'rgb8':
            self.get_logger().warn(f'unexpected encoding {msg.encoding}, skip')
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # PNG 는 BGR 순서로 기록
        cv2.imwrite(os.path.join(self.frames_dir, f'{self.idx:06d}.png'),
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv.writerow([self.idx, f'{stamp:.6f}'])
        self.idx += 1

    def close(self):
        self.csv_file.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=45.0)
    args = ap.parse_args()

    rclpy.init()
    node = CaptureNode(args.out)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.close()
        n = node.idx
        node.destroy_node()
        rclpy.shutdown()
        print(f'captured {n} frames -> {args.out}')
        sys.exit(0 if n > 0 else 1)


if __name__ == '__main__':
    main()
