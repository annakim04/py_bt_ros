#!/usr/bin/env python3
# qr_detector_node.py
#
# LIMO RGB 카메라 이미지에서 QR 코드를 인식하고,
# - /qr_text 토픽(std_msgs/msg/String)으로 publish
# - 홈 디렉터리의 ~/qr_logs_limo/qr_codes.txt 파일에 기록

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
from datetime import datetime
from pathlib import Path


class LimoQrDetector(Node):
    def __init__(self):
        super().__init__('limo_qr_detector')

        # 파라미터: 카메라 이미지 토픽 이름
        # 기본값은 LIMO에서 많이 쓰는 /camera/color/image_raw 로 가정
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # 파라미터: OpenCV 창으로 디스플레이 할지 여부 (limo 환경상 현재 False)
        self.declare_parameter('enable_view', False)
        self.enable_view = self.get_parameter('enable_view').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # 마지막으로 인식한 QR 텍스트 (같은 값이 반복될 때 너무 많이 저장하는 것 방지)
        self.last_qr_text = None

        # QR 텍스트 publish
        self.qr_pub = self.create_publisher(String, 'qr_list', 10)

        # 카메라 subscribe
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        # 로그 파일 경로 준비
        log_dir = Path.home() / 'qr_logs_limo'
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = log_dir / 'qr_codes.txt'

        self.get_logger().info(f'[LIMO QR] Subscribed to image topic: {image_topic}')
        self.get_logger().info(f'[LIMO QR] Logging to: {self.log_file}')
        if self.enable_view:
            self.get_logger().info('[LIMO QR] OpenCV view enabled')

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV 이미지
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        #### QR 코드  detect & 디코드 ####
        data, points, _ = self.qr_detector.detectAndDecode(cv_image)

        # Debug log
        if points is not None and len(points) > 0:
            self.get_logger().info('[LIMO QR] QR pattern detected in image.')

            if data:
                self.get_logger().info(f'[LIMO QR] Decoded data: {repr(data)}')
            else:
                self.get_logger().warn('[LIMO QR] QR detected but decode failed (empty data).')
        else:
            # QR 패턴 자체를 찾지 못한 프레임 (필요하면 사용)
            # self.get_logger().debug('[LIMO QR] No QR detected in this frame.')
            pass

        # data: 인식된 문자열 (없으면 빈 문자열)
        if data:
            # 새로운 문자열일 때만 처리 (QR 화면에 계속 떠 있어도 1번만 찍히게 하는 구조)
            if data != self.last_qr_text:
                self.last_qr_text = data

                # 로그 출력
                self.get_logger().info(f'[LIMO QR] Detected: {data}')

                # 토픽 publish
                out_msg = String()
                out_msg.data = data
                self.qr_pub.publish(out_msg)

                # 로그에 QR text 저장
                now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                with self.log_file.open('a', encoding='utf-8') as f:
                    f.write(f'[{now}] {data}\n')

        # enable_view : True시 실행
        # 선택) OpenCV 창에 표시
        if self.enable_view:
            # QR 코드 위치 사각형으로 표시
            if points is not None and len(points) > 0:
                pts = points.astype(int).reshape(-1, 2)
                for i in range(4):
                    pt1 = tuple(pts[i])
                    pt2 = tuple(pts[(i + 1) % 4])
                    cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)

            cv2.imshow('LIMO QR View', cv_image)
            cv2.waitKey(1)

    # enable_view : True시 실행
    # 노드 종료 시 OpenCV 창 종료
    def destroy_node(self):
        if self.enable_view:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LimoQrDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
