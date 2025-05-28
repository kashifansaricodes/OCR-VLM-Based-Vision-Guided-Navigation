#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import easyocr
import time
from ocr_pkg.srv import OCRInfer  

class OcrNode(Node):
    def __init__(self):
        super().__init__('ocr_node')
        
        # Create the OCR service server
        self.srv = self.create_service(
            OCRInfer,
            'ocr_infer',
            self.ocr_service_callback
        )
        
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(['en'], gpu=False)
        self.get_logger().info('OCR Service Node has been initialized')

    def ocr_service_callback(self, request, response):
        """
        Process the image and return detected text
        """
        self.get_logger().info("Received OCR service request")
        
        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
        self.get_logger().info("Processing frame for OCR...")

        # OCR processing
        result = self.reader.readtext(frame, detail=1)
        
        # Concatenate all detected text
        detected_text = ""
        for (bbox, text, prob) in result:
            (tl, tr, br, bl) = bbox
            tl, br = tuple(map(int, tl)), tuple(map(int, br))
            self.get_logger().info(f"Detected: '{text}' with confidence {prob:.2f}")
            
            # Add the detected text to the output string
            detected_text += text + " "
        
        # Set the response
        response.text = detected_text.strip()
        self.get_logger().info(f"Returning OCR text: '{response.text}'")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OcrNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()