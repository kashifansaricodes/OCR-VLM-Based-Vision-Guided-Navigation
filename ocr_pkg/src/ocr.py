#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import easyocr
import matplotlib.pyplot as plt
import time


class OcrNode(Node):
    def __init__(self):
        super().__init__('ocr_node')
        self.subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_raw',
            self.image_callback,
            10  # 10 Hz subscription rate, but we filter ourselves
        )
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(['en'], gpu=True)
        self.last_processed_time = 0.0
        self.process_interval = 10.0  # seconds

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_processed_time < self.process_interval:
            return
        self.last_processed_time = current_time

        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("Processing frame for OCR...")

        # OCR processing
        result = self.reader.readtext(frame, detail=1)
        for (bbox, text, prob) in result:
            (tl, tr, br, bl) = bbox
            tl, br = tuple(map(int, tl)), tuple(map(int, br))
            cv2.rectangle(frame, tl, br, (0, 255, 0), 2)
            cv2.putText(frame, text, (tl[0], tl[1] + 50), cv2.FONT_HERSHEY_SIMPLEX,
                        2.0, (0, 0, 255), 5)
            self.get_logger().info(f"Detected: '{text}' with confidence {prob:.2f}")

        plt.figure(figsize=(10, 8))
        plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        plt.title("OCR Result")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = OcrNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
