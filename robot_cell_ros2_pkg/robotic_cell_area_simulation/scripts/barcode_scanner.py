#!/usr/bin/env python3

"""
barcode_scanner.py

This file implements a barcode scanner simulation. It publishes barcode messages with 5 random digits every second.
Also it changes barcode number every 10 seconds. Also it has service server to provide latest barcode on request.

Author: [Ahmad Kheirandish]
Date: [26/07/2025]

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import random


class BarcodeScannerNode(Node):

    def __init__(self):
        super().__init__('barcode_scanner')
        # Create publisher for barcode messages
        self.publisher_ = self.create_publisher(
            String, 
            'barcode_scanner/barcode', 
            10
        )
        # Create service server for barcode requests
        self.service = self.create_service(
            Trigger,
            'barcode_scanner/get_barcode',
            self.get_barcode_callback
        )
        # Create timer for continuous publishing (every 1 second)
        publish_timer_period = 1.0  # seconds
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_callback)
        # Create timer for barcode generation (every 10 seconds)
        generate_timer_period = 10.0  # seconds
        self.generate_timer = self.create_timer(generate_timer_period, self.generate_new_barcode)
        # Generate initial barcode
        self.current_barcode = self.generate_barcode()
        self.get_logger().info('Barcode Scanner Node started')
        self.get_logger().info('Publishing on topic: barcode_scanner/barcode')
        self.get_logger().info('Service available at: barcode_scanner/get_barcode')
        self.get_logger().info(f'Initial barcode: {self.current_barcode}')
        self.get_logger().info('Publishing every 1 second, changing barcode every 10 seconds')

    def generate_barcode(self) -> str:
        """Generate a 5-digit random barcode"""
        # Generate 5 random digits (0-9)
        barcode = ''.join([str(random.randint(0, 9)) for _ in range(5)])
        return barcode

    def publish_callback(self):
        """Timer callback to publish current barcode every second"""
        # Create and publish message
        msg = String()
        msg.data = self.current_barcode
        self.publisher_.publish(msg)
        # Log the published barcode (less frequently to avoid spam)
        self.get_logger().debug(f'Published barcode: {self.current_barcode}')

    def generate_new_barcode(self):
        """Timer callback to generate new barcode every 10 seconds"""
        # Generate new barcode
        old_barcode = self.current_barcode
        self.current_barcode = self.generate_barcode()
        # Log the barcode change
        self.get_logger().info(f'Barcode changed: {old_barcode} -> {self.current_barcode}')

    def get_barcode_callback(self, request, response):
        """Service callback to provide latest barcode"""
        response.success = True
        response.message = self.current_barcode
        self.get_logger().info(f'Service request received, returning barcode: {self.current_barcode}')
        return response


def run(args=None):
    rclpy.init(args=args)
    barcode_scanner = BarcodeScannerNode()
    try:
        barcode_scanner.get_logger().info('Barcode Scanner is running...')
        rclpy.spin(barcode_scanner)
    except KeyboardInterrupt:
        barcode_scanner.get_logger().info('Node interrupted by user')
    finally:
        barcode_scanner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run()



