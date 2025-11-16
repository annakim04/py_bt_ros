import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import os
import time


class CaptureImageServer(Node):
    def __init__(self):
        super().__init__("capture_image_server")

        # ---- Configurable camera topic ----
        self.camera_topic = "/camera/image_raw"   # 필요하면 수정 (/TurtleBot3Burger/front_camera/image_color 등)

        # ---- Subscribers ----
        self.bridge = CvBridge()
        self.last_image = None

        self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # ---- Service ----
        self.create_service(
            Trigger,
            "/capture_image",
            self.capture_callback
        )

        # ---- Save Directory ----
        self.save_dir = "/home/kimwonkyung/captured_images"
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(f"[CaptureImageServer] Started. Listening on {self.camera_topic}")

    # --------- 이미지 콜백 ----------
    def image_callback(self, msg):
        self.last_image = msg

    # --------- 서비스 처리 ----------
    def capture_callback(self, request, response):
        # 이미지 수신 여부 체크
        if self.last_image is None:
            response.success = False
            response.message = "No image received yet. Try again in a moment."
            self.get_logger().warn("[CaptureImageServer] No image available yet.")
            return response

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.last_image, "bgr8")
        except Exception as e:
            response.success = False
            response.message = f"cv_bridge conversion failed: {e}"
            self.get_logger().error(f"[CaptureImageServer] cv_bridge error: {e}")
            return response

        # 파일 이름 생성
        timestamp = str(int(time.time()))
        filename = os.path.join(self.save_dir, f"photo_{timestamp}.jpg")

        try:
            cv2.imwrite(filename, cv_img)
        except Exception as e:
            response.success = False
            response.message = f"Saving failed: {e}"
            self.get_logger().error(f"[CaptureImageServer] Save error: {e}")
            return response

        response.success = True
        response.message = f"Saved: {filename}"
        self.get_logger().info(f"[CaptureImageServer] Saved image → {filename}")
        return response


def main():
    rclpy.init()
    node = CaptureImageServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

