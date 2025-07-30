#!/usr/bin/env python3

"""
robot_cell_simulator.py

This file implements a robotic cell simulator that interacts with a WMS server and a GUI.
It simulates robotic operations such as picking items, handling emergency states, and controlling
the robotic cell's door. The simulator can process pick requests, manage the robotic cell's state, and 
communicate with the WMS server and GUI via HTTP requests.


Author: [Ahmad Kheirandish]
Date: [27/07/2025]

"""

from flask import Flask, request, jsonify
import requests
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import time
import random
import threading
from datetime import datetime
from typing import Dict
from std_msgs.msg import Bool

class RoboticCellSimulator(Node):
    def __init__(self, cell_id: str):
        super().__init__('robotic_cell_simulator')
        self.cell_id = cell_id
        self.is_busy = False
        self.pick_success_rate = 0.9  # 90% success rate
        self.door_closed = False  # Track door state (False = open, True = closed)
        self.door_state_received = False  # Track if we've received any door state messages
        self.emergency_activated = False  # Track emergency button state (False = deactivated, True = activated)
        self.emergency_state_received = False  # Track if we've received any emergency state messages
        # Add stack light status tracking
        self.stack_light_status = {
            "red": False,
            "yellow": False, 
            "green": True,  # Default to green when ready
            "buzzer": False
        }
        
        # Declare ROS2 parameters
        self.declare_parameter('wms_server_ip', 'localhost')
        self.declare_parameter('wms_server_port', 5000)
        self.declare_parameter('robot_cell_port', 8080)
        self.declare_parameter('gui_port', 8081)
        self.declare_parameter('success_rate', 0.9)
        # Get parameter values
        self.wms_server_ip = self.get_parameter('wms_server_ip').get_parameter_value().string_value
        self.wms_server_port = self.get_parameter('wms_server_port').get_parameter_value().integer_value
        self.robot_cell_port = self.get_parameter('robot_cell_port').get_parameter_value().integer_value
        self.gui_port = self.get_parameter('gui_port').get_parameter_value().integer_value
        self.pick_success_rate = self.get_parameter('success_rate').get_parameter_value().double_value
        # Create service client for barcode scanner
        self.barcode_client = self.create_client(Trigger, 'barcode_scanner/get_barcode')
        # Create service client for door control
        self.door_client = self.create_client(SetBool, 'robotic_cell/door_control')
        # Create service client for emergency button control
        self.emergency_client = self.create_client(SetBool, 'robotic_cell/emergency_control')
        # Subscribe to door state topic
        self.door_state_subscription = self.create_subscription(
            Bool,
            'robotic_cell/door_state',
            self.door_state_callback,
            10
        )
        
        # Subscribe to emergency button state topic
        self.emergency_state_subscription = self.create_subscription(
            Bool,
            'robotic_cell/emergency_state',
            self.emergency_state_callback,
            10
        )
        
        # Create service server for robot cell status
        self.status_service = self.create_service(
            Trigger, 
            'robotic_cell/get_status', 
            self.get_status_callback
        )
        
        self.get_logger().info(f'Robotic Cell Simulator - {self.cell_id} initialized')
        self.get_logger().info(f'WMS Server: {self.wms_server_ip}:{self.wms_server_port}')
        self.get_logger().info(f'GUI Port: {self.gui_port}')
        self.get_logger().info(f'Success Rate: {self.pick_success_rate * 100}%')
        self.get_logger().info('Barcode service client created for: barcode_scanner/get_barcode')
        self.get_logger().info('Door control service client created for: robotic_cell/door_control')
        self.get_logger().info('Emergency control service client created for: robotic_cell/emergency_control')
        self.get_logger().info('Door state subscriber created for: robotic_cell/door_state')
        self.get_logger().info('Emergency state subscriber created for: robotic_cell/emergency_state')
        self.get_logger().info('Status service server created for: robotic_cell/get_status')
        self.get_logger().info(f'Initial door state: CLOSED={self.door_closed} (waiting for topic messages)')
        self.get_logger().info(f'Initial emergency state: ACTIVATED={self.emergency_activated} (waiting for topic messages)')

    def door_state_callback(self, msg):
        """Callback to receive door state updates"""
        old_state = self.door_closed
        self.door_closed = msg.data
        self.door_state_received = True
        
        if old_state != self.door_closed or not self.door_state_received:
            state_text = "CLOSED" if self.door_closed else "OPEN"
            self.get_logger().info(f'Door state updated: {state_text} (door_closed={self.door_closed})')
            
            # If door opened while robot is busy, we should handle this
            if not self.door_closed and self.is_busy:
                self.get_logger().warn('Door opened while robot was busy - operations may be affected')
        
        # Update stack light when door state changes
        self.update_stack_light_status()

    def emergency_state_callback(self, msg):
        """Callback to receive emergency button state updates"""
        old_state = self.emergency_activated
        self.emergency_activated = msg.data
        self.emergency_state_received = True
        
        if old_state != self.emergency_activated or not self.emergency_state_received:
            state_text = "ACTIVATED" if self.emergency_activated else "DEACTIVATED"
            self.get_logger().info(f'Emergency state updated: {state_text} (emergency_activated={self.emergency_activated})')
            
            # If emergency activated while robot is busy, we should handle this
            if self.emergency_activated and self.is_busy:
                self.get_logger().warn('Emergency activated while robot was busy - operations may be affected')
        
        # Update stack light when emergency state changes
        self.update_stack_light_status()

    def get_barcode_from_scanner(self) -> int:
        """Request latest barcode from barcode scanner service"""
        try:
            if not self.barcode_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('Barcode scanner service not available, using fallback barcode')
                return random.randint(100000, 999999)
            
            request = Trigger.Request()
            future = self.barcode_client.call_async(request)
            
            # Wait for response with timeout
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Retrieved barcode from scanner: {response.message}')
                    # Convert string barcode to integer
                    return int(response.message)
                else:
                    self.get_logger().warn('Barcode service returned failure, using fallback')
                    return random.randint(100000, 999999)
            else:
                self.get_logger().warn('Barcode service call timed out, using fallback')
                return random.randint(100000, 999999)
                
        except Exception as e:
            self.get_logger().warn(f'Error calling barcode service: {str(e)}, using fallback')
            return random.randint(100000, 999999)
        
    def update_stack_light_status(self):
        """Update stack light status based on current robot state"""
        if self.emergency_activated:
            # Emergency: Red light + buzzer
            self.stack_light_status = {
                "red": True,
                "yellow": False,
                "green": False,
                "buzzer": True
            }
        elif not self.door_closed:
            # Door open: Yellow light (warning)
            self.stack_light_status = {
                "red": False,
                "yellow": True,
                "green": False,
                "buzzer": False
            }
        elif self.is_busy:
            # Robot busy: Yellow light (working)
            self.stack_light_status = {
                "red": False,
                "yellow": True,
                "green": False,
                "buzzer": False
            }
        else:
            # Normal operation: Green light
            self.stack_light_status = {
                "red": False,
                "yellow": False,
                "green": True,
                "buzzer": False
            }

    def simulate_pick_operation(self, pick_id: int, quantity: int) -> Dict:
        """Simulate the picking operation and update GUI"""
        self.get_logger().info(f"Robot {self.cell_id} starting pick operation for pickId={pick_id}, quantity={quantity}")
        self.get_logger().info(f"Current door state: CLOSED={self.door_closed}, state_received={self.door_state_received}")
        self.get_logger().info(f"Current emergency state: ACTIVATED={self.emergency_activated}, state_received={self.emergency_state_received}")
        self.is_busy = True
        self.update_stack_light_status()  # Update stack light when becoming busy
        
        # CRITICAL: Check safety conditions first - if door is open OR emergency is activated, fail immediately
        if not self.door_closed:
            self.get_logger().error(f"Pick {pick_id} FAILED: Door is OPEN (door_closed={self.door_closed}), robot movement not allowed")
            
            result = {
                "pickId": pick_id,
                "pickSuccessful": False,
                "errorMessage": "Robotic cell door is open - robot movement not allowed for safety",
                "itemBarcode": None
            }
            
            self.is_busy = False
            self.update_stack_light_status()  # Update stack light when finishing
            return result
        
        if self.emergency_activated:
            self.get_logger().error(f"Pick {pick_id} FAILED: Emergency button is ACTIVATED (emergency_activated={self.emergency_activated}), robot movement not allowed")
            result = {
                "pickId": pick_id,
                "pickSuccessful": False,
                "errorMessage": "Emergency button is activated - robot movement not allowed for safety",
                "itemBarcode": None
            }
            
            self.is_busy = False
            self.update_stack_light_status()  # Update stack light when finishing
            return result
        
        self.get_logger().info(f"Safety checks passed - door CLOSED and emergency DEACTIVATED, proceeding with pick operation")
        # Send command to GUI to start arm movement and wait for completion
        gui_success = self.send_command_to_gui_and_wait(pick_id, quantity)
        
        if not gui_success:
            self.get_logger().warn(f"GUI movement failed or unavailable for pick {pick_id}")
        
        # Check safety conditions again during operation
        if not self.door_closed:
            self.get_logger().error(f"Pick {pick_id} FAILED: Door OPENED during operation (door_closed={self.door_closed})")
            result = {
                "pickId": pick_id,
                "pickSuccessful": False,
                "errorMessage": "Robotic cell door opened during operation - safety abort",
                "itemBarcode": None
            }
            
            self.is_busy = False
            return result
        
        if self.emergency_activated:
            self.get_logger().error(f"Pick {pick_id} FAILED: Emergency ACTIVATED during operation (emergency_activated={self.emergency_activated})")
            result = {
                "pickId": pick_id,
                "pickSuccessful": False,
                "errorMessage": "Emergency button activated during operation - safety abort",
                "itemBarcode": None
            }
            self.is_busy = False
            return result
        
        # Additional processing time after GUI movement
        additional_time = random.uniform(0.5, 1.5)
        self.get_logger().info(f"Processing pick operation for {additional_time:.2f} additional seconds")
        time.sleep(additional_time)
        # Final safety checks before completing operation
        if not self.door_closed:
            self.get_logger().error(f"Pick {pick_id} FAILED: Door OPENED during final processing (door_closed={self.door_closed})")
            result = {
                "pickId": pick_id,
                "pickSuccessful": False,
                "errorMessage": "Robotic cell door opened during final processing - safety abort",
                "itemBarcode": None
            }
            self.is_busy = False
            return result
        
        if self.emergency_activated:
            self.get_logger().error(f"Pick {pick_id} FAILED: Emergency ACTIVATED during final processing (emergency_activated={self.emergency_activated})")
            
            result = {
                "pickId": pick_id,
                "pickSuccessful": False,
                "errorMessage": "Emergency button activated during final processing - safety abort",
                "itemBarcode": None
            }
            self.is_busy = False
            return result
        
        # Only proceed with success/failure simulation if all safety conditions are met throughout
        pick_successful = random.random() < self.pick_success_rate
        # Get barcode from scanner if successful, otherwise null
        item_barcode = None
        if pick_successful:
            item_barcode = self.get_barcode_from_scanner()
        
        # Generate error message if failed, otherwise null
        error_message = None
        if not pick_successful:
            error_messages = [
                "Item not found at specified location",
                "Gripper failed to grasp item", 
                "Obstacle detected in robotic path",
                "Item too heavy for current gripper configuration",
                "Barcode scanner read error",
                "Positioning sensor malfunction",
                "Item damaged during pick attempt",
                "Vacuum gripper lost suction",
                "Item slipped from gripper during transport",
                "Target location occupied by another item"
            ]
            error_message = random.choice(error_messages)
        
        # Ensure exact structure as required
        result = {
            "pickId": pick_id,
            "pickSuccessful": pick_successful,
            "errorMessage": error_message,
            "itemBarcode": item_barcode
        }
        
        self.is_busy = False
        self.update_stack_light_status()  # Update stack light when finishing
        self.get_logger().info(f"Pick operation completed: success={pick_successful}, barcode={item_barcode}")
        if error_message:
            self.get_logger().warn(f"Pick {pick_id} failed: {error_message}")
        
        return result
    
    def send_command_to_gui_and_wait(self, pick_id: int, quantity: int) -> bool:
        """Send pick command to GUI and wait for movement completion"""
        try:
            url = f"http://localhost:{self.gui_port}/pick"
            
            payload = {
                "pickId": pick_id,
                "quantity": quantity
            }
            
            self.get_logger().info(f"Sending pick command to GUI at {url}")
            
            response = requests.post(
                url,
                json=payload,
                timeout=10,
                headers={'Content-Type': 'application/json'}
            )
            
            if response.status_code == 200:
                self.get_logger().info(f"Successfully sent pick command to GUI for pickId={pick_id}")
                
                # Wait for GUI to complete movement (poll status)
                return self.wait_for_gui_completion(pick_id)
            else:
                self.get_logger().warn(f"Failed to send command to GUI: {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Could not communicate with GUI: {str(e)}")
            return False
        except Exception as e:
            self.get_logger().warn(f"Unexpected error communicating with GUI: {str(e)}")
            return False
    
    def wait_for_gui_completion(self, pick_id: int, max_wait_time: float = 15.0) -> bool:
        """Wait for GUI to complete the arm movement"""
        start_time = time.time()
        
        while time.time() - start_time < max_wait_time:
            try:
                # Check GUI status
                status_url = f"http://localhost:{self.gui_port}/status"
                response = requests.get(status_url, timeout=5)
                
                if response.status_code == 200:
                    status_data = response.json()
                    is_moving = status_data.get('is_moving', False)
                    
                    if not is_moving:
                        self.get_logger().info(f"GUI completed movement for pick {pick_id}")
                        return True
                        
                # Wait a bit before checking again
                time.sleep(0.5)
                
            except requests.exceptions.RequestException:
                # GUI might not be available, continue without waiting
                self.get_logger().warn("Cannot check GUI status, proceeding without waiting")
                time.sleep(3)  # Default wait time
                return False
                
        self.get_logger().warn(f"GUI movement timeout for pick {pick_id}")
        return False
    
    def send_pick_confirmation(self, confirmation_data: Dict) -> bool:
        """Send pick confirmation to WMS server"""
        try:
            url = f"http://{self.wms_server_ip}:{self.wms_server_port}/confirmPick"
            
            self.get_logger().info(f"Sending pick confirmation to {url}")
            self.get_logger().info(f"Confirmation data: {confirmation_data}")
            
            response = requests.post(
                url,
                json=confirmation_data,
                timeout=10,
                headers={'Content-Type': 'application/json'}
            )
            
            if response.status_code == 200:
                self.get_logger().info(f"Successfully sent pick confirmation for pickId={confirmation_data['pickId']}")
                return True
            else:
                self.get_logger().error(f"Failed to send pick confirmation: HTTP {response.status_code}")
                self.get_logger().error(f"Response: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Network error sending pick confirmation: {str(e)}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error sending pick confirmation: {str(e)}")
            return False

    def control_door(self, close_door: bool) -> bool:
        """Control door via ROS2 service"""
        try:
            if not self.door_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('Door control service not available')
                return False
            
            request_msg = SetBool.Request()
            request_msg.data = close_door  # True = close, False = open
            
            future = self.door_client.call_async(request_msg)
            
            # Wait for response with timeout
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    action = "closed" if close_door else "opened"
                    self.get_logger().info(f'Door successfully {action}: {response.message}')
                    return True
                else:
                    self.get_logger().warn(f'Door control failed: {response.message}')
                    return False
            else:
                self.get_logger().warn('Door control service call timed out')
                return False
                
        except Exception as e:
            self.get_logger().warn(f'Error calling door control service: {str(e)}')
            return False

    def control_emergency(self, activate: bool) -> bool:
        """Control emergency button activation/deactivation"""
        try:
            self.get_logger().info(f"Sending emergency control request: {'activate' if activate else 'deactivate'}")
            
            request = SetBool.Request()
            request.data = activate
            
            future = self.emergency_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Emergency control successful: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"Emergency control failed: {response.message}")
                    return False
            else:
                self.get_logger().error("Emergency control service call failed - no response")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Emergency control service call failed: {str(e)}")
            return False

    def get_status_callback(self, request, response):
        """Service callback to provide robot cell status"""
        try:
            # Determine status code based on priority:
            # Emergency has highest priority (-1)
            # Door open has medium priority (1) 
            # Normal operation has lowest priority (0)
            
            if self.emergency_activated:
                status_code = -1
                status_text = "Emergency button is active"
                self.get_logger().debug("Status request: Emergency active (-1)")
            elif not self.door_closed:
                status_code = 1
                status_text = "Door is open"
                self.get_logger().debug("Status request: Door open (1)")
            else:
                status_code = 0
                status_text = "Normal operation - door closed, emergency inactive"
                self.get_logger().debug("Status request: Normal operation (0)")
            
            response.success = True
            response.message = str(status_code)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in status service callback: {str(e)}")
            response.success = False
            response.message = "Error getting status"
            return response

# Global variables for Flask integration
robot_node = None
app = Flask(__name__)

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    global robot_node
    if robot_node is None:
        return jsonify({"error": "Robot node not initialized"}), 500
    
    return jsonify({
        "status": "healthy",
        "robot_id": robot_node.cell_id,
        "is_busy": robot_node.is_busy,
        "timestamp": datetime.now().isoformat()
    })

@app.route('/door', methods=['POST'])
def control_door():
    """Control door via HTTP request from GUI"""
    global robot_node
    if robot_node is None:
        return jsonify({"error": "Robot node not initialized"}), 500
        
    try:
        data = request.get_json()
        
        robot_node.get_logger().info(f"Door endpoint called with data: {data}")
        
        # Validate request
        if not data or 'close' not in data:
            robot_node.get_logger().error("Missing required field: close")
            return jsonify({"error": "Missing required field: close"}), 400
        
        close_door = data['close']
        
        # Validate data type
        if not isinstance(close_door, bool):
            robot_node.get_logger().error("close must be boolean")
            return jsonify({"error": "close must be boolean"}), 400
        
        robot_node.get_logger().info(f"Processing door control request: {'close' if close_door else 'open'}")
        
        # Control door via ROS2 service
        success = robot_node.control_door(close_door)
        
        if success:
            robot_node.get_logger().info(f"Door control successful: {'closed' if close_door else 'opened'}")
            return jsonify({
                "message": f"Door {'closed' if close_door else 'opened'} successfully",
                "door_closed": close_door
            }), 200
        else:
            robot_node.get_logger().error("Door control failed")
            return jsonify({
                "error": "Failed to control door",
                "door_closed": None
            }), 500
        
    except Exception as e:
        robot_node.get_logger().error(f"Error controlling door: {str(e)}")
        return jsonify({"error": "Internal server error"}), 500

@app.route('/emergency', methods=['POST'])
def control_emergency():
    """Control emergency button activation/deactivation"""
    global robot_node
    if robot_node is None:
        return jsonify({"error": "Robot node not initialized"}), 500
        
    try:
        data = request.get_json()
        
        robot_node.get_logger().info(f"Emergency endpoint called with data: {data}")
        
        # Validate request
        if not data or 'activate' not in data:
            robot_node.get_logger().error("Missing required field: activate")
            return jsonify({"error": "Missing required field: activate"}), 400
        
        activate = data['activate']
        
        # Validate data type
        if not isinstance(activate, bool):
            robot_node.get_logger().error("activate must be boolean")
            return jsonify({"error": "activate must be boolean"}), 400
        
        robot_node.get_logger().info(f"Processing emergency control request: {'activate' if activate else 'deactivate'}")
        
        # Control emergency via ROS2 service
        success = robot_node.control_emergency(activate)
        
        if success:
            robot_node.get_logger().info(f"Emergency control successful: {'activated' if activate else 'deactivated'}")
            return jsonify({
                "message": f"Emergency button {'activated' if activate else 'deactivated'} successfully",
                "emergency_activated": activate
            }), 200
        else:
            robot_node.get_logger().error("Emergency control failed")
            return jsonify({
                "error": "Failed to control emergency button",
                "emergency_activated": None
            }), 500
        
    except Exception as e:
        robot_node.get_logger().error(f"Error controlling emergency: {str(e)}")
        return jsonify({"error": "Internal server error"}), 500

@app.route('/pick', methods=['POST'])
def receive_pick_request():
    """Receive pick request from WMS and process it"""
    global robot_node
    if robot_node is None:
        return jsonify({"error": "Robot node not initialized"}), 500
        
    try:
        data = request.get_json()
        
        # Validate request
        if not data or 'pickId' not in data or 'quantity' not in data:
            return jsonify({"error": "Missing required fields: pickId, quantity"}), 400
        
        pick_id = data['pickId']
        quantity = data['quantity']
        
        # Validate data types
        if not isinstance(pick_id, int) or not isinstance(quantity, int):
            return jsonify({"error": "pickId and quantity must be integers"}), 400
        
        if quantity <= 0:
            return jsonify({"error": "quantity must be positive"}), 400
        
        # Check if robot is busy
        if robot_node.is_busy:
            robot_node.get_logger().warn(f"Robot is busy, rejecting pick request {pick_id}")
            return jsonify({"error": "Robot is currently busy"}), 409
        
        robot_node.get_logger().info(f"Received pick request: pickId={pick_id}, quantity={quantity}")
        
        # Process pick request asynchronously
        def process_pick():
            try:
                # Perform the pick operation (door state will be checked inside)
                result = robot_node.simulate_pick_operation(pick_id, quantity)
                
                # Send confirmation back to WMS
                success = robot_node.send_pick_confirmation(result)
                
                if not success:
                    robot_node.get_logger().error(f"Failed to send confirmation for pick {pick_id}")
                    
            except Exception as e:
                robot_node.get_logger().error(f"Error processing pick request {pick_id}: {str(e)}")
        
        # Start processing in background thread
        threading.Thread(target=process_pick, daemon=True).start()
        
        return jsonify({
            "message": "Pick request accepted",
            "pickId": pick_id,
            "robot_id": robot_node.cell_id
        }), 200
        
    except Exception as e:
        robot_node.get_logger().error(f"Error receiving pick request: {str(e)}")
        return jsonify({"error": "Internal server error"}), 500

@app.route('/status', methods=['GET'])
def get_robot_status():
    """Get current robot status"""
    global robot_node
    if robot_node is None:
        return jsonify({"error": "Robot node not initialized"}), 500
        
    return jsonify({
        "robot_id": robot_node.cell_id,
        "is_busy": robot_node.is_busy,
        "door_closed": robot_node.door_closed,
        "door_state_received": robot_node.door_state_received,
        "emergency_activated": robot_node.emergency_activated,
        "emergency_state_received": robot_node.emergency_state_received,
        "success_rate": robot_node.pick_success_rate,
        "timestamp": datetime.now().isoformat()
    })

@app.route('/config', methods=['POST'])
def update_robot_config():
    """Update robot configuration"""
    global robot_node
    if robot_node is None:
        return jsonify({"error": "Robot node not initialized"}), 500
        
    try:
        data = request.get_json()
        
        if 'success_rate' in data:
            success_rate = data['success_rate']
            if 0.0 <= success_rate <= 1.0:
                robot_node.pick_success_rate = success_rate
                robot_node.get_logger().info(f"Updated success rate to {success_rate}")
            else:
                return jsonify({"error": "success_rate must be between 0.0 and 1.0"}), 400
        
        return jsonify({
            "message": "Configuration updated",
            "success_rate": robot_node.pick_success_rate
        })
        
    except Exception as e:
        robot_node.get_logger().error(f"Error updating robot config: {str(e)}")
        return jsonify({"error": "Internal server error"}), 500

@app.route('/stack_light/status', methods=['GET'])
def get_stack_light_status():
    """Get current stack light status"""
    global robot_node
    if robot_node is None:
        return jsonify({"error": "Robot node not initialized"}), 500
    
    # Update status before returning and log for debugging
    robot_node.update_stack_light_status()
    robot_node.get_logger().debug(f"Stack light status request: {robot_node.stack_light_status}")
    
    return jsonify({
        "robot_id": robot_node.cell_id,
        "stack_light": robot_node.stack_light_status,
        "robot_state": {
            "is_busy": robot_node.is_busy,
            "door_closed": robot_node.door_closed,
            "emergency_activated": robot_node.emergency_activated
        },
        "timestamp": datetime.now().isoformat()
    })

def run(args=None):
    global robot_node
    rclpy.init(args=args)
    robot_node = RoboticCellSimulator("ROBOT-001")
    
    try:
        robot_node.get_logger().info(f"Starting Robotic Cell Simulator - {robot_node.cell_id}")
        # Debug: List all registered routes
        robot_node.get_logger().info("Flask routes registered:")
        for rule in app.url_map.iter_rules():
            robot_node.get_logger().info(f"  {rule.rule} [{', '.join(rule.methods)}]")
        
        # Start Flask server in a separate thread
        def run_flask():
            robot_node.get_logger().info(f"Starting Flask server on 0.0.0.0:{robot_node.robot_cell_port}")
            app.run(host='0.0.0.0', port=robot_node.robot_cell_port, debug=False, use_reloader=False, threaded=True)
        
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()
        robot_node.get_logger().info(f"Flask server thread started")
        # Give Flask server time to start
        time.sleep(5)
        # Test the server is responding
        try:
            test_url = f"http://localhost:{robot_node.robot_cell_port}/health"
            response = requests.get(test_url, timeout=2)
            robot_node.get_logger().info(f"Health check response: {response.status_code}")
        except Exception as e:
            robot_node.get_logger().error(f"Health check failed: {e}")
        
        rclpy.spin(robot_node)
        
    except KeyboardInterrupt:
        robot_node.get_logger().info('Node interrupted by user')
        
    finally:
        # Cleanup
        robot_node.destroy_node()
        rclpy.shutdown()
        robot_node.get_logger().info('Node shutdown complete')

if __name__ == '__main__':
    run()

