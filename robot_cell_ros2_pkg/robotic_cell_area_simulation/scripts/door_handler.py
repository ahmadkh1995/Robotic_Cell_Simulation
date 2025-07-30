#!/usr/bin/env python3

"""
door_handler.py

This file implements a robotic cell's door handler. It publishes door state messages (open/closed) every second.
Also it has service server to control the door (open/close) on request. (True = door closed, False = door open)

Author: [Ahmad Kheirandish]
Date: [26/07/2025]

"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class DoorHandlerNode(Node):

    def __init__(self):
        super().__init__('door_handler')
        
        # Create publisher for door state
        self.publisher_ = self.create_publisher(
            Bool, 
            'robotic_cell/door_state', 
            10
        )
        # Create service server for door control
        self.service = self.create_service(
            SetBool,
            'robotic_cell/door_control',
            self.control_door_callback
        )
        # Create timer for continuous publishing (every 1 second)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize door state (False = open, True = closed)
        # Default: door is OPEN for safety
        self.door_closed = False
        self.get_logger().info('Door Handler Node started')
        self.get_logger().info('Publishing on topic: robotic_cell/door_state')
        self.get_logger().info('Service available at: robotic_cell/door_control')
        self.get_logger().info(f'Initial door state: {"CLOSED" if self.door_closed else "OPEN"}')
        self.get_logger().info('Door Handler is running...')

    def control_door_callback(self, request, response):
        """Service callback to control door open/close"""
        try:
            # Update door state
            old_state = self.door_closed
            self.door_closed = request.data  # True = close, False = open
            # Log the state change
            new_state_text = "CLOSED" if self.door_closed else "OPEN"
            old_state_text = "CLOSED" if old_state else "OPEN"
            if old_state != self.door_closed:
                self.get_logger().info(f'Door state changed: {old_state_text} -> {new_state_text}')
            else:
                self.get_logger().info(f'Door state confirmed: {new_state_text}')
            
            # Publish the new door state immediately
            self.publish_door_state()
            # Set response
            response.success = True
            response.message = f"Door is now {new_state_text.lower()}"
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error in door control: {str(e)}')
            response.success = False
            response.message = f"Door control failed: {str(e)}"
            return response

    def publish_door_state(self):
        """Publish current door state"""
        msg = Bool()
        msg.data = self.door_closed
        self.publisher_.publish(msg)
        
        state_text = "CLOSED" if self.door_closed else "OPEN"
        self.get_logger().debug(f'Published door state: {state_text} (data={self.door_closed})')

    def timer_callback(self):
        """Timer callback to publish door state periodically"""
        self.publish_door_state()

def run(args=None):    
    rclpy.init(args=args)
    door_handler = DoorHandlerNode()
    try:
        rclpy.spin(door_handler)
        
    except KeyboardInterrupt:
        door_handler.get_logger().info('Node interrupted by user')

    finally:
        door_handler.destroy_node()
        rclpy.shutdown()
        door_handler.get_logger().info('Node shutdown complete')


if __name__ == '__main__':
    run()

