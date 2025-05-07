#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys
import tempfile

# Add the MobileVLM directory to the Python path
# You'll need to update this path to match your setup
MOBILEVLM_PATH = os.path.expanduser("~/VGN/MobileVLM")
sys.path.append(MOBILEVLM_PATH)

# Import the inference module from MobileVLM
from scripts.inference import inference_once


class VlmNode(Node):
    def __init__(self):
        super().__init__('vlm_node')
        
        # Model configuration
        self.model_path = "mtgv/MobileVLM_V2-1.7B"  # MobileVLM V2
        
        # Create a publisher for VLM inference results
        self.publisher = self.create_publisher(
            String,
            '/vlm_inference_output',
            10
        )
        
        # Subscribe to camera image
        self.image_subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to OCR text
        self.text_subscription = self.create_subscription(
            String,
            '/ocr_text',
            self.text_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_ocr_text = None
        self.get_logger().info('VLM Node has been initialized')
    
    def image_callback(self, msg):
        # Store the latest image
        self.latest_image = msg
        self.get_logger().debug("Received new image")
        self.process_if_ready()
    
    def text_callback(self, msg):
        # Store the latest OCR text
        self.latest_ocr_text = msg.data
        self.get_logger().info(f"Received OCR text: '{self.latest_ocr_text}'")
        self.process_if_ready()
    
    def process_if_ready(self):
        # Check if we have both image and OCR text
        if self.latest_image is not None and self.latest_ocr_text is not None:
            self.get_logger().info("Processing with VLM...")
            
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            # Save the image temporarily
            with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as temp_file:
                temp_image_path = temp_file.name
                cv2.imwrite(temp_image_path, cv_image)
            
            # Create args object for inference
            args = type('Args', (), {
                "model_path": self.model_path,
                "image_file": temp_image_path,
                "prompt": self.latest_ocr_text,
                "conv_mode": "v1",
                "temperature": 0,
                "top_p": None,
                "num_beams": 1,
                "max_new_tokens": 512,
                "load_8bit": False,
                "load_4bit": False,
            })()
            
            try:
                # Run inference
                result = inference_once(args)
                
                # Publish result
                output_msg = String()
                output_msg.data = result
                self.publisher.publish(output_msg)
                self.get_logger().info(f"Published VLM result: '{result}'")
            except Exception as e:
                self.get_logger().error(f"Error during VLM inference: {str(e)}")
            finally:
                # Remove temporary file
                try:
                    os.unlink(temp_image_path)
                except:
                    pass
            
            # Reset to prevent re-processing the same data
            # Keep the image but clear the OCR text
            self.latest_ocr_text = None


def main(args=None):
    rclpy.init(args=args)
    node = VlmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()