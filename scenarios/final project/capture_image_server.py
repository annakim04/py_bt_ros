#!/usr/bin/env python3
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2


class CaptureImageServer(Node):
    def __init__(self):
        super().__init__('capture_image_server')

        self.bridge = CvBridge()
        self.last_image = None

        camera_topic = '/camera/color/image_raw'
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.srv = self.create_service(
            Trigger,
            'capture_image',
            self.capture_callback
        )

        self.save_dir = os.path.expanduser('~/bt_captured_images')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(
            f'CaptureImageServer started. '
            f'Subscribe: {camera_topic}, Service: /capture_image, Save dir: {self.save_dir}'
        )

    def image_callback(self, msg: Image):
        self.last_image = msg

    def capture_callback(self, request, response):
        if self.last_image is None:
            response.success = False
            response.message = 'No image received yet.'
            return response

        cv_img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding='bgr8')

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.save_dir, f'image_{stamp}.png')

        cv2.imwrite(filename, cv_img)
        self.get_logger().info(f'Saved image: {filename}')

        response.success = True
        response.message = f'Saved to {filename}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CaptureImageServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
