#!/usr/bin/env python3

"""
emergency_button_handler.py

This file implements a robotic cell's emergency button handler. It publishes emergency button state messages (activated/deactivated) every second.
Also it has service server to control the emergency button (activate/deactivate) on request. (True = activated, False = deactivated)
By default, the emergency button is deactivated for normal operation.

Author: [Ahmad Kheirandish]
Date: [27/07/2025]

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import time


class EmergencyButtonHandlerNode(Node):

    def __init__(self):
        super().__init__('emergency_button_handler')
        # Create publisher for emergency button state
        self.publisher_ = self.create_publisher(
            Bool, 
            'robotic_cell/emergency_state', 
            10
        )
        
        # Create service server for emergency button control
        self.service = self.create_service(
            SetBool,
            'robotic_cell/emergency_control',
            self.control_emergency_callback
        )
        
        # Create timer for continuous publishing (every 1 second)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize emergency button state (False = deactivated, True = activated)
        # Default: emergency is DEACTIVATED for normal operation
        self.emergency_activated = False
        self.get_logger().info('Emergency Button Handler Node started')
        self.get_logger().info('Publishing on topic: robotic_cell/emergency_state')
        self.get_logger().info('Service available at: robotic_cell/emergency_control')
        self.get_logger().info(f'Initial emergency state: {"ACTIVATED" if self.emergency_activated else "DEACTIVATED"}')
        self.get_logger().info('Emergency Button Handler is running...')

    def control_emergency_callback(self, request, response):
        """Service callback to control emergency button activate/deactivate"""
        try:
            # Update emergency button state
            old_state = self.emergency_activated
            self.emergency_activated = request.data  # True = activate, False = deactivate
            # Log the state change
            new_state_text = "ACTIVATED" if self.emergency_activated else "DEACTIVATED"
            old_state_text = "ACTIVATED" if old_state else "DEACTIVATED"
            if old_state != self.emergency_activated:
                self.get_logger().info(f'Emergency button state changed: {old_state_text} -> {new_state_text}')
            else:
                self.get_logger().info(f'Emergency button state confirmed: {new_state_text}')
            
            # Publish the new emergency button state immediately
            self.publish_emergency_state()
            # Set response
            response.success = True
            response.message = f"Emergency button is now {new_state_text.lower()}"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error in emergency button control: {str(e)}')
            response.success = False
            response.message = f"Emergency button control failed: {str(e)}"
            return response

    def publish_emergency_state(self):
        """Publish current emergency button state"""
        msg = Bool()
        msg.data = self.emergency_activated
        self.publisher_.publish(msg)
        state_text = "ACTIVATED" if self.emergency_activated else "DEACTIVATED"
        self.get_logger().debug(f'Published emergency state: {state_text} (data={self.emergency_activated})')

    def timer_callback(self):
        """Timer callback to publish emergency state periodically"""
        self.publish_emergency_state()

def run(args=None):    
    rclpy.init(args=args)
    emergency_handler = EmergencyButtonHandlerNode()
    try:
        rclpy.spin(emergency_handler)
        
    except KeyboardInterrupt:
        emergency_handler.get_logger().info('Node interrupted by user')
        
    finally:
        emergency_handler.destroy_node()
        rclpy.shutdown()
        emergency_handler.get_logger().info('Node shutdown complete')

if __name__ == '__main__':
    run()

