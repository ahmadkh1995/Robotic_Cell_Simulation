#!/usr/bin/env python3

"""
stack_light_handler.py

This file implements a robotic cell's stack light handler. It publishes stack light status messages (color codes) every second.
Also it has service server to control the stack light (set color) on request.

Author: [Ahmad Kheirandish]
Date: [27/07/2025]

"""


import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

class StackLightHandler(Node):
    def __init__(self):
        super().__init__('stack_light_handler')
        # Create service client for robot cell status
        self.status_client = self.create_client(Trigger, 'robotic_cell/get_status')
        # Create publisher for stack light status
        self.status_publisher = self.create_publisher(Int32, 'stack_light/status', 10)
        # Timer to periodically check robot cell status
        self.timer = self.create_timer(1.0, self.check_robot_status)  # Check every 1 second
        self.get_logger().info('Stack Light Handler initialized')
        self.get_logger().info('Service client created for: robotic_cell/get_status')
        self.get_logger().info('Publisher created for: stack_light/status')
        
    def check_robot_status(self):
        """Periodically check robot cell status and update stack light accordingly"""
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug('Robot cell status service not available')
            return
            
        # Create and send request
        request = Trigger.Request()
        future = self.status_client.call_async(request)
        # Add callback to handle response
        future.add_done_callback(self.handle_status_response)
    
    def handle_status_response(self, future):
        """Handle the status response from robot cell simulator"""
        try:
            response = future.result()           
            if response.success:
                # Parse the status code from the response message
                try:
                    status_code = int(response.message)
                    self.update_stack_light(status_code)
                except ValueError:
                    self.get_logger().warn(f"Invalid status code received: {response.message}")
            else:
                self.get_logger().warn("Failed to get robot cell status")
                
        except Exception as e:
            self.get_logger().error(f"Error handling status response: {str(e)}")
    
    def update_stack_light(self, status_code: int):
        """Update stack light based on robot cell status code"""
        # Publish the status code
        msg = Int32()
        msg.data = status_code
        self.status_publisher.publish(msg)
        self.get_logger().debug(f"Published status code: {status_code}")       
        if status_code == -1:
            # Emergency button active - RED light
            self.set_stack_light_red()
            self.get_logger().debug("Stack light: RED (Emergency active)")
            
        elif status_code == 1:
            # Door open - YELLOW light
            self.set_stack_light_yellow()
            self.get_logger().debug("Stack light: YELLOW (Door open)")
            
        elif status_code == 0:
            # Normal operation (door closed, emergency inactive) - GREEN light
            self.set_stack_light_green()
            self.get_logger().debug("Stack light: GREEN (Normal operation)")
            
        else:
            self.get_logger().warn(f"Unknown status code: {status_code}")
    
    def set_stack_light_red(self):
        """Set stack light to RED (Emergency state)"""
        self.get_logger().info("🔴 STACK LIGHT: RED - EMERGENCY ACTIVE")
    
    def set_stack_light_yellow(self):
        """Set stack light to YELLOW (Warning state)"""
        self.get_logger().info("🟡 STACK LIGHT: YELLOW - DOOR OPEN")
    
    def set_stack_light_green(self):
        """Set stack light to GREEN (Normal operation)"""
        self.get_logger().info("🟢 STACK LIGHT: GREEN - NORMAL OPERATION")

def run(args=None):
    rclpy.init(args=args)
    try:
        stack_light_handler = StackLightHandler()
        stack_light_handler.get_logger().info("Stack Light Handler node started")
        rclpy.spin(stack_light_handler)
        
    except KeyboardInterrupt:
        stack_light_handler.get_logger().info('Node interrupted by user')
        
    finally:
        if 'stack_light_handler' in locals():
            stack_light_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run()

