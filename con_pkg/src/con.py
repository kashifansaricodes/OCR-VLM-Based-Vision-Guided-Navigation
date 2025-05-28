#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading
import asyncio
from con_pkg.action import NavCommand
from geometry_msgs.msg import Twist

class ActionControllerNode(Node):
    def __init__(self):
        super().__init__('action_controller_node')
        
        # Use callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            NavCommand,
            'nav_command',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        # Create publisher for robot velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Define word banks for different motion commands
        self.forward_words = ['forward', 'ahead', 'upward', 'up', 'front', 'straight', 'go', 'north']
        self.left_words = ['left', 'turn left', 'leftward', 'port', 'anticlockwise', 'west']
        self.right_words = ['right', 'turn right', 'rightward', 'starboard', 'clockwise', 'east']
        self.down_words = ['backward', 'downward', 'down', 'reverse', 'back', 'retreat']
        self.stop_words = ['stop', 'halt', 'brake', 'pause', 'hold', 'wait', 'no']
        
        # Set movement parameters
        self.linear_speed = 0.5  # meters per second
        self.angular_speed = 0.8  # radians per second
        self.movement_duration = 3.0  # seconds to move forward
        self.turn_duration = 2.0  # seconds to turn
        
        self.get_logger().info('Action Controller Node initialized')
    
    async def execute_callback(self, goal_handle):
        """Execute the goal command based on natural language input"""
        command = goal_handle.request.command
        self.get_logger().info(f'Received command: {command}')
        
        # Create a result message
        result = NavCommand.Result()
        
        # Parse the command and execute appropriate action
        command_lower = command.lower()
        
        # Check for forward movement keywords
        if any(word in command_lower for word in self.forward_words):
            success = await self.execute_up_movement()
            action_type = "forward movement"
        # Check for left turn keywords
        elif any(word in command_lower for word in self.left_words):
            success = await self.execute_left_movement()
            action_type = "left turn"
        # Check for right turn keywords
        elif any(word in command_lower for word in self.right_words):
            success = await self.execute_right_movement()
            action_type = "right turn"
        # Check for stop keywords
        elif any(word in command_lower for word in self.stop_words):
            success = await self.execute_stop()
            action_type = "stop"
        else:
            self.get_logger().info(f"Unknown command: {command}")
            success = False
            action_type = "unknown action"
            
        # Set result
        if success:
            result.success = True
            result.message = f"Successfully executed {action_type} for '{command}'"
        else:
            result.success = False
            result.message = f"Failed to execute {action_type} for '{command}'"
            
        goal_handle.succeed()
        return result
    
    async def execute_up_movement(self):
        """
        Execute forward movement for the robot
        """
        self.get_logger().info("Executing forward movement")
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        
        # Move for specified duration
        return await self.publish_velocity_for_duration(twist, self.movement_duration)
    
    async def execute_down_movement(self):
        """
        Execute backward movement for the robot
        """
        self.get_logger().info("Executing backward movement")
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = -self.linear_speed
        twist.angular.z = 0.0
        
        # Move for specified duration
        return await self.publish_velocity_for_duration(twist, self.movement_duration)
    
    async def execute_left_movement(self):
        """
        Execute left turn for the robot
        """
        self.get_logger().info("Executing left turn")
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        
        # Move for specified duration
        return await self.publish_velocity_for_duration(twist, self.turn_duration)
    
    async def execute_right_movement(self):
        """
        Execute right turn for the robot
        """
        self.get_logger().info("Executing right turn")
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.angular_speed
        
        # Move for specified duration
        return await self.publish_velocity_for_duration(twist, self.turn_duration)
    
    async def execute_stop(self):
        """
        Execute stop action for the robot
        """
        self.get_logger().info("Executing stop command")
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        # Just publish stop command once
        self.velocity_publisher.publish(twist)
        await asyncio.sleep(0.5)  # Short pause
        return True
    
    async def publish_velocity_for_duration(self, twist_msg, duration):
        """
        Publish velocity commands for a specified duration
        """
        self.get_logger().info(f"Moving with linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z} for {duration} seconds")
        
        # Publish velocity at 10Hz for the specified duration
        start_time = self.get_clock().now()
        end_time = start_time + rclpy.duration.Duration(seconds=duration)
        
        # Publish the velocity command repeatedly
        while self.get_clock().now() < end_time:
            self.velocity_publisher.publish(twist_msg)
            await asyncio.sleep(0.1)  # 10Hz control loop
        
        # Send stop command after duration
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.velocity_publisher.publish(stop_twist)
        
        self.get_logger().info("Movement completed")
        return True
    
    async def simulate_movement(self, duration):
        """
        Deprecated: Use publish_velocity_for_duration instead
        """
        self.get_logger().info(f"Starting movement simulation for {duration} seconds")
        await asyncio.sleep(duration)
        self.get_logger().info("Movement simulation completed")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ActionControllerNode()
    
    # Use multithreaded executor for concurrent operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure robot stops when node is terminated
        stop_twist = Twist()
        node.velocity_publisher.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()