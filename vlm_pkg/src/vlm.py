#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import tempfile
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ocr_pkg.srv import OCRInfer
from con_pkg.action import NavCommand

# Add the MobileVLM directory to the Python path
# You'll need to update this path to match your setup
MOBILEVLM_PATH = os.path.expanduser("~/VGN/MobileVLM")
sys.path.append(MOBILEVLM_PATH)

# Import the inference module from MobileVLM
from scripts.inference import inference_once


class VlmOrchestratorNode(Node):
    def __init__(self):
        super().__init__('vlm_orchestrator_node')
        
        # Model configuration
        self.model_path = "mtgv/MobileVLM_V2-1.7B"  # MobileVLM V2
        
        # Use a callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to camera image
        self.image_subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create OCR service client
        self.ocr_client = self.create_client(
            OCRInfer,
            'ocr_infer',
            callback_group=self.callback_group
        )
        
        # Create action client for controller
        self.nav_client = ActionClient(
            self,
            NavCommand,
            'nav_command',
            callback_group=self.callback_group
        )
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.processing = False  # Flag to prevent parallel processing
        self.should_exit = False  # Flag to exit the loop
        
        self.get_logger().info('VLM Orchestrator Node has been initialized')
        
        # Wait for services and action servers
        while not self.ocr_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('OCR service not available, waiting...')
            
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigation action server not available, waiting...')
            
        self.get_logger().info('All services and action servers are available')
    
    def image_callback(self, msg):
        # Store the latest image
        self.latest_image = msg
        self.get_logger().debug("Received new image")
        
        # Start processing if not already in progress
        if not self.processing and not self.should_exit:
            self.processing = True
            self.process_iteration()
    
    async def process_iteration(self):
        """Main processing cycle"""
        try:
            if self.latest_image is None:
                self.processing = False
                return
                
            # 1. Call OCR service with the image
            ocr_text = await self.call_ocr_service(self.latest_image)
            if not ocr_text:
                self.get_logger().info("No text detected, waiting for next image")
                self.processing = False
                return
                
            # 2. Check if we should stop based on OCR text
            if "stop" in ocr_text.lower():
                self.get_logger().info("Stop sign detected, ending the process")
                self.should_exit = True
                self.processing = False
                return
                
            # 3. Run VLM inference
            vlm_result = await self.run_vlm_inference(self.latest_image, ocr_text)
            
            # 4. Check if VLM result indicates stop
            if "stop" in vlm_result.lower():
                self.get_logger().info("VLM indicated stop, ending the process")
                self.should_exit = True
                self.processing = False
                return
                
            # 5. Send navigation command
            await self.send_nav_command(vlm_result)
            
            # 6. Reset for next iteration
            self.processing = False
            
        except Exception as e:
            self.get_logger().error(f"Error in processing iteration: {str(e)}")
            self.processing = False
    
    async def call_ocr_service(self, image_msg):
        """Call the OCR service and return the detected text"""
        request = OCRInfer.Request()
        request.image = image_msg
        
        self.get_logger().info("Calling OCR service...")
        response = await self.ocr_client.call_async(request)
        self.get_logger().info(f"OCR service returned: '{response.text}'")
        return response.text
    
    async def run_vlm_inference(self, image_msg, ocr_text):
        """Run VLM inference on the image and OCR text"""
        self.get_logger().info("Processing with VLM...")
        
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        # Save the image temporarily
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as temp_file:
            temp_image_path = temp_file.name
            cv2.imwrite(temp_image_path, cv_image)
        
        try:
            # Create prompt with OCR text
            prompt = f"I see the text: '{ocr_text}'. Based on this and the image, what navigation command should I execute? Reply with a simple command like 'go forward', 'turn right', 'turn left', 'stop', etc."
            
            # Create args object for inference
            args = type('Args', (), {
                "model_path": self.model_path,
                "image_file": temp_image_path,
                "prompt": prompt,
                "conv_mode": "v1",
                "temperature": 0,
                "top_p": None,
                "num_beams": 1,
                "max_new_tokens": 512,
                "load_8bit": False,
                "load_4bit": False,
            })()
            
            # Run inference
            result = inference_once(args)
            self.get_logger().info(f"VLM result: '{result}'")
            return result
        
        except Exception as e:
            self.get_logger().error(f"Error during VLM inference: {str(e)}")
            return "error"
        finally:
            # Remove temporary file
            try:
                os.unlink(temp_image_path)
            except:
                pass
    
    async def send_nav_command(self, command):
        """Send the navigation command to the controller and wait for completion"""
        goal_msg = NavCommand.Goal()
        goal_msg.command = command
        
        self.get_logger().info(f"Sending navigation command: '{command}'")
        
        # Send the goal and wait for the result
        send_goal_future = await self.nav_client.send_goal_async(goal_msg)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected")
            return False
            
        self.get_logger().info("Navigation goal accepted, waiting for result...")
        result_future = await goal_handle.get_result_async()
        result = result_future.result().result
        
        if result.success:
            self.get_logger().info(f"Navigation action completed: {result.message}")
        else:
            self.get_logger().error(f"Navigation action failed: {result.message}")
            
        return result.success


def main(args=None):
    rclpy.init(args=args)
    node = VlmOrchestratorNode()
    
    # Use a multithreaded executor for async operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()