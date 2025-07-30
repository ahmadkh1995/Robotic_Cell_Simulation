"""
robot_cell_monitor.py

This file implements a monitoring system for robotic cells that interact with a WMS server and robot cell simulator.
It handles the monitoring of pick requests, manages robotic cell states, and communicates with the
robotic cells and GUI via HTTP requests. It also includes robot kinematics calculations for robotic arm movements.


Author: [Ahmad Kheirandish]
Date: [27/07/2025]

"""

import sys
import threading
import time
import random
import numpy as np
import requests
import logging
from datetime import datetime
from flask import Flask, request, jsonify
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QTextEdit, QGroupBox, QFrame)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class Kinematics:
    @staticmethod
    def inverse_kinematics(x, y, l1, l2, l3):
        """Calculate joint angles for 3-DOF robotic arm using inverse kinematics"""
        distance = np.hypot(x, y)
        if distance > (l1 + l2 + l3):
            return None
        
        angle_to_target = np.arctan2(y, x)
        x_wrist = x - l3 * np.cos(angle_to_target)
        y_wrist = y - l3 * np.sin(angle_to_target)
        dx, dy = x_wrist, y_wrist
        D = (dx**2 + dy**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if abs(D) > 1:
            return None
            
        theta2 = np.arccos(D)
        theta1 = np.arctan2(dy, dx) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
        theta12 = theta1 + theta2
        theta3 = angle_to_target - theta12
        return np.degrees([theta1, theta2, theta3])

    @staticmethod
    def forward_kinematics(theta1, theta2, theta3, l1, l2, l3):
        """Calculate end effector position from joint angles"""
        theta1_rad = np.radians(theta1)
        theta2_rad = np.radians(theta2)
        theta3_rad = np.radians(theta3)
        x = l1 * np.cos(theta1_rad) + l2 * np.cos(theta1_rad + theta2_rad) + l3 * np.cos(theta1_rad + theta2_rad + theta3_rad)
        y = l1 * np.sin(theta1_rad) + l2 * np.sin(theta1_rad + theta2_rad) + l3 * np.sin(theta1_rad + theta2_rad + theta3_rad)
        return x, y

class RoboticCellInterface:
    def __init__(self):
        self.wms_server_url = "http://localhost:5000"
        self.current_angles = [0, 0, 0]
        self.target_angles = [0, 0, 0]
        self.start_angles = [0, 0, 0]
        self.target_position = (100, 50)  # Default position
        self.lengths = [100, 75, 50]  # Link lengths
        self.is_moving = False
        self.animation_progress = 0.0
        self.animation_duration = 3.0  # 3 seconds for smooth movement
        self.animation_start_time = 0
        self.current_pick_id = None  # Track current pick operation
        self.show_target = False  # Flag to control target visibility
        self.is_returning_home = False  # Flag to track return-to-home movement
        self.home_position = [0, 0, 0]  # Home joint angles
        
    def animate_to_position(self, target_x, target_y, pick_id=None):
        """Start smooth animation to target position"""
        # Calculate target angles
        result = Kinematics.inverse_kinematics(target_x, target_y, *self.lengths)
        if result is None:
            logger.warning("Target position unreachable, using default")
            result = [30, 45, -30]  # Default safe position
            target_x, target_y = Kinematics.forward_kinematics(result[0], result[1], result[2], *self.lengths)
        
        # Set up animation parameters
        self.start_angles = self.current_angles.copy()
        self.target_angles = result
        self.target_position = (target_x, target_y)
        self.is_moving = True
        self.show_target = True  # Show target when animation starts
        self.is_returning_home = False  # Not returning home during pick
        self.animation_progress = 0.0
        self.animation_start_time = time.time()
        self.current_pick_id = pick_id
        logger.info(f"Starting smooth animation to position ({target_x:.1f}, {target_y:.1f})")
        logger.info(f"Target angles: θ1={result[0]:.1f}°, θ2={result[1]:.1f}°, θ3={result[2]:.1f}°")
    
    def animate_to_home(self):
        """Start smooth animation back to home position (0,0,0)"""
        # Set up animation parameters for return to home
        self.start_angles = self.current_angles.copy()
        self.target_angles = self.home_position.copy()
        # Calculate home position coordinates for display
        home_x, home_y = Kinematics.forward_kinematics(0, 0, 0, *self.lengths)
        self.target_position = (home_x, home_y)
        self.is_moving = True
        self.show_target = True  # Show home target during return
        self.is_returning_home = True  # Flag to indicate return-to-home movement
        self.animation_progress = 0.0
        self.animation_start_time = time.time()
        self.current_pick_id = None
        logger.info(f"Starting smooth return to home position (0°, 0°, 0°)")
        logger.info(f"Home position coordinates: ({home_x:.1f}, {home_y:.1f})")
    
    def update_animation(self):
        """Update animation progress and current angles"""
        if not self.is_moving:
            return False
            
        elapsed_time = time.time() - self.animation_start_time
        self.animation_progress = min(elapsed_time / self.animation_duration, 1.0)
        # Use smooth easing function (ease-in-out)
        t = self.animation_progress
        smoothed_progress = t * t * (3.0 - 2.0 * t)  # Smooth step function
        # Interpolate between start and target angles
        for i in range(3):
            angle_diff = self.target_angles[i] - self.start_angles[i]
            self.current_angles[i] = self.start_angles[i] + angle_diff * smoothed_progress
        
        # Check if animation is complete
        if self.animation_progress >= 1.0:
            self.current_angles = self.target_angles.copy()
            self.is_moving = False
            self.show_target = False  # Hide target when animation completes
            
            if self.is_returning_home:
                self.is_returning_home = False
                logger.info("Return to home completed")
            else:
                logger.info("Pick animation completed - starting return to home")
                # Start return to home after a brief pause
                threading.Timer(1.0, self.animate_to_home).start()
            
            self.current_pick_id = None
            return True
            
        return False
        
    def simulate_pick_operation(self, pick_id, quantity):
        """Simulate a pick operation by moving to a random position"""
        try:
            logger.info(f"Proceeding with pick {pick_id}")
            # Generate random target position within workspace
            max_reach = sum(self.lengths) * 0.8  # 80% of max reach for safety
            angle = random.uniform(0, 2 * np.pi)
            distance = random.uniform(50, max_reach)
            target_x = distance * np.cos(angle)
            target_y = distance * np.sin(angle)
            logger.info(f"Pick {pick_id}: Moving to random position ({target_x:.1f}, {target_y:.1f})")
            # Start smooth animation
            self.animate_to_position(target_x, target_y, pick_id)
            return True
            
        except Exception as e:
            logger.error(f"Error in pick operation {pick_id}: {str(e)}")
            return False
    
class WMSInfoMonitor(QThread):
    """Monitor WMS server for request/response information"""
    wms_request_received = pyqtSignal(dict)
    robot_response_received = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.app = Flask(__name__)
        self.setup_wms_routes()
        
    def setup_wms_routes(self):
        """Setup routes to monitor WMS communication"""
        
        @self.app.route('/wms_monitor/request', methods=['POST'])
        def log_wms_request():
            """Receive WMS request information for display"""
            try:
                data = request.get_json()
                self.wms_request_received.emit(data)
                return jsonify({"status": "logged"}), 200
            except Exception as e:
                logger.error(f"Error logging WMS request: {e}")
                return jsonify({"error": str(e)}), 500
        
        @self.app.route('/wms_monitor/response', methods=['POST'])
        def log_robot_response():
            """Receive robot response information for display"""
            try:
                data = request.get_json()
                self.robot_response_received.emit(data)
                return jsonify({"status": "logged"}), 200
            except Exception as e:
                logger.error(f"Error logging robot response: {e}")
                return jsonify({"error": str(e)}), 500
    
    def run(self):
        self.app.run(host='0.0.0.0', port=8082, debug=False, use_reloader=False)

class PickRequestHandler(QThread):
    pick_received = pyqtSignal(int, int)  # pick_id, quantity
    
    def __init__(self, robotic_interface):
        super().__init__()
        self.robotic_interface = robotic_interface
        self.app = Flask(__name__)
        self.setup_routes()
        
    def setup_routes(self):
        @self.app.route('/health', methods=['GET'])
        def health_check():
            return jsonify({
                "status": "healthy",
                "robot_id": "WMS-ROBOT-ARM-GUI",
                "is_busy": self.robotic_interface.is_moving,
                "timestamp": datetime.now().isoformat()
            })
        
        @self.app.route('/status', methods=['GET'])
        def get_status():
            """Get current GUI status including movement state"""
            return jsonify({
                "robot_id": "WMS-ROBOT-ARM-GUI",
                "is_moving": self.robotic_interface.is_moving,
                "animation_progress": self.robotic_interface.animation_progress,
                "current_pick_id": self.robotic_interface.current_pick_id,
                "current_angles": [float(angle) for angle in self.robotic_interface.current_angles],
                "target_angles": [float(angle) for angle in self.robotic_interface.target_angles],
                "timestamp": datetime.now().isoformat()
            })
        
        @self.app.route('/pick', methods=['POST'])
        def receive_pick_request():
            try:
                data = request.get_json()
                if not data or 'pickId' not in data or 'quantity' not in data:
                    return jsonify({"error": "Missing required fields"}), 400
                
                pick_id = data['pickId']
                quantity = data['quantity']
                # Check if GUI is already moving
                if self.robotic_interface.is_moving:
                    logger.warning(f"GUI is busy, rejecting pick request {pick_id}")
                    return jsonify({"error": "GUI is currently moving"}), 409
                
                logger.info(f"GUI received pick request: pickId={pick_id}, quantity={quantity}")
                # Emit signal to update GUI
                self.pick_received.emit(pick_id, quantity)
                # Process pick in background thread
                threading.Thread(
                    target=self.robotic_interface.simulate_pick_operation,
                    args=(pick_id, quantity),
                    daemon=True
                ).start()
                
                return jsonify({
                    "message": "Pick request accepted by GUI",
                    "pickId": pick_id,
                    "robot_id": "WMS-ROBOT-ARM-GUI"
                }), 200
                
            except Exception as e:
                logger.error(f"Error receiving pick request in GUI: {str(e)}")
                return jsonify({"error": "Internal server error"}), 500
    
    def run(self):
        try:
            logger.info("Starting GUI Flask server on port 8081")
            self.app.run(host='0.0.0.0', port=8081, debug=False, use_reloader=False, threaded=True)
        except Exception as e:
            logger.error(f"Error starting GUI Flask server: {e}")

class WMSRoboticArmGUI(QWidget):
    # Add custom signal for thread-safe log updates
    log_update_signal = pyqtSignal(str)
    stack_light_update_signal = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WMS 3-DOF Robotic Arm HMI")
        self.resize(1400, 800)  # Increased width for side panel
        
        # Initialize robotic interface
        self.robotic_interface = RoboticCellInterface()
        
        # Connect custom signals
        self.log_update_signal.connect(self.add_log_message)
        
        # Initialize UI
        self.init_ui()
        
        self.status_label.setText("Status: Ready - Waiting for pick requests...")
        
        # Start Flask server for receiving pick requests
        self.pick_handler = PickRequestHandler(self.robotic_interface)
        self.pick_handler.pick_received.connect(self.on_pick_received)
        self.pick_handler.start()
        
        # Start WMS monitoring server
        self.wms_monitor = WMSInfoMonitor()
        self.wms_monitor.wms_request_received.connect(self.on_wms_request_received)
        self.wms_monitor.robot_response_received.connect(self.on_robot_response_received)
        self.wms_monitor.start()
        
        # Timer for updating arm visualization
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_arm_visualization)
        self.timer.start(50)  # Update every 50ms for smooth animation
        
        # Timer for polling stack light status
        self.stack_light_timer = QTimer()
        self.stack_light_timer.timeout.connect(self.poll_stack_light_status)
        self.stack_light_timer.start(1000)  # Poll every 1 second
        
        # Connect stack light signal
        self.stack_light_update_signal.connect(self.update_stack_light_display)
        
        # Track last stack light status to avoid duplicate messages
        self.last_stack_light_status = None
        
        logger.info("WMS Robotic Arm GUI started")
        logger.info("Listening for pick requests on port 8081")
        logger.info("Listening for WMS monitoring on port 8082")
        logger.info("Polling stack light status every 1 second from robot simulator")

    def init_ui(self):
        # Main horizontal layout
        main_layout = QHBoxLayout()
        # Left side - Arm visualization and controls
        left_panel = QVBoxLayout()
        # Title
        title = QLabel("WMS 3-DOF Robotic Arm HMI")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        left_panel.addWidget(title)
        # Status display
        self.status_label = QLabel("Status: Ready - Waiting for pick requests...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(QFont("Arial", 12))
        left_panel.addWidget(self.status_label)
        # Arm visualization
        self.figure = Figure(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)
        left_panel.addWidget(self.canvas)
        # Joint angles display
        self.angles_label = QLabel("Joint Angles: θ1=0°, θ2=0°, θ3=0°")
        self.angles_label.setAlignment(Qt.AlignCenter)
        self.angles_label.setFont(QFont("Arial", 10))
        left_panel.addWidget(self.angles_label)
        # Position display
        self.position_label = QLabel("End Effector Position: (100.0, 50.0)")
        self.position_label.setAlignment(Qt.AlignCenter)
        self.position_label.setFont(QFont("Arial", 10))
        left_panel.addWidget(self.position_label)
        # Stack Light Status Panel
        stack_light_panel = self.create_stack_light_panel()
        left_panel.addWidget(stack_light_panel)
        # Log display
        log_label = QLabel("Activity Log:")
        log_label.setFont(QFont("Arial", 10, QFont.Bold))
        left_panel.addWidget(log_label)
        self.log_display = QTextEdit()
        self.log_display.setMaximumHeight(120)
        self.log_display.setReadOnly(True)
        left_panel.addWidget(self.log_display)
        # Right side - WMS Information Panel
        right_panel = self.create_wms_info_panel()
        # Add panels to main layout
        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        main_layout.addWidget(left_widget, 2)  # 2/3 of space
        main_layout.addWidget(right_panel, 1)  # 1/3 of space
        self.setLayout(main_layout)
        # Initial arm drawing
        self.draw_arm()
    
    def create_wms_info_panel(self):
        """Create the WMS information side panel"""
        # Main container
        panel = QFrame()
        panel.setFrameStyle(QFrame.StyledPanel)
        panel.setLineWidth(2)
        panel.setMaximumWidth(400)
        layout = QVBoxLayout()
        # Panel title
        panel_title = QLabel("WMS Communication Monitor")
        panel_title.setAlignment(Qt.AlignCenter)
        panel_title.setFont(QFont("Arial", 14, QFont.Bold))
        panel_title.setStyleSheet("QLabel { background-color: #2E8B57; color: white; padding: 8px; }")
        layout.addWidget(panel_title)
        # WMS Request Information Group
        wms_request_group = QGroupBox("WMS Request Information")
        wms_request_group.setFont(QFont("Arial", 11, QFont.Bold))
        wms_request_layout = QVBoxLayout()
        self.wms_request_display = QTextEdit()
        self.wms_request_display.setMaximumHeight(200)
        self.wms_request_display.setReadOnly(True)
        self.wms_request_display.setStyleSheet("QTextEdit { background-color: #F0F8FF; }")
        self.wms_request_display.setPlainText("Waiting for WMS requests...")
        wms_request_layout.addWidget(self.wms_request_display)
        wms_request_group.setLayout(wms_request_layout)
        layout.addWidget(wms_request_group)
        # Robot Response Information Group
        robot_response_group = QGroupBox("Robot Response Information")
        robot_response_group.setFont(QFont("Arial", 11, QFont.Bold))
        robot_response_layout = QVBoxLayout()
        self.robot_response_display = QTextEdit()
        self.robot_response_display.setMaximumHeight(200)
        self.robot_response_display.setReadOnly(True)
        self.robot_response_display.setStyleSheet("QTextEdit { background-color: #FFF8DC; }")
        self.robot_response_display.setPlainText("Waiting for robot responses...")
        robot_response_layout.addWidget(self.robot_response_display)
        robot_response_group.setLayout(robot_response_layout)
        layout.addWidget(robot_response_group)
        # Communication Statistics
        stats_group = QGroupBox("Communication Statistics")
        stats_group.setFont(QFont("Arial", 11, QFont.Bold))
        stats_layout = QVBoxLayout()
        self.stats_label = QLabel(
            "Requests Received: 0\n"
            "Responses Sent: 0\n"
            "Success Rate: N/A\n"
            "Last Activity: N/A"
        )
        self.stats_label.setFont(QFont("Arial", 9))
        self.stats_label.setStyleSheet("QLabel { background-color: #F5F5F5; padding: 10px; }")
        stats_layout.addWidget(self.stats_label)
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        # Status indicator
        self.connection_status = QLabel("🔴 Monitoring: Inactive")
        self.connection_status.setAlignment(Qt.AlignCenter)
        self.connection_status.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(self.connection_status)
        panel.setLayout(layout)
        # Initialize statistics
        self.request_count = 0
        self.response_count = 0
        self.successful_responses = 0
        return panel
    
    def create_stack_light_panel(self):
        """Create the stack light status panel"""
        # Main container frame
        panel = QFrame()
        panel.setFrameStyle(QFrame.StyledPanel)
        panel.setLineWidth(1)
        panel.setFixedHeight(80)
        panel.setStyleSheet("QFrame { background-color: #F0F0F0; border: 1px solid #CCCCCC; }")
        layout = QHBoxLayout()
        layout.setContentsMargins(10, 5, 10, 5)
        # Stack Light Title
        title_label = QLabel("Stack Light Status:")
        title_label.setFont(QFont("Arial", 10, QFont.Bold))
        title_label.setFixedWidth(120)
        layout.addWidget(title_label)
        # Gray rectangle container for stack lights
        self.stack_light_container = QFrame()
        self.stack_light_container.setFrameStyle(QFrame.Box)
        self.stack_light_container.setLineWidth(2)
        self.stack_light_container.setFixedSize(200, 60)
        self.stack_light_container.setStyleSheet("""
            QFrame {
                background-color: #808080;
                border: 2px solid #404040;
                border-radius: 5px;
            }
        """)
        
        # Layout for lights inside the gray rectangle
        lights_layout = QHBoxLayout()
        lights_layout.setContentsMargins(10, 10, 10, 10)
        lights_layout.setSpacing(15)
        # Create individual light indicators
        self.red_light = QLabel("●")
        self.red_light.setAlignment(Qt.AlignCenter)
        self.red_light.setFont(QFont("Arial", 20))
        self.red_light.setStyleSheet("QLabel { color: #400000; }")  # Dark red (off)
        
        self.yellow_light = QLabel("●")
        self.yellow_light.setAlignment(Qt.AlignCenter)
        self.yellow_light.setFont(QFont("Arial", 20))
        self.yellow_light.setStyleSheet("QLabel { color: #404000; }")  # Dark yellow (off)
        
        self.green_light = QLabel("●")
        self.green_light.setAlignment(Qt.AlignCenter)
        self.green_light.setFont(QFont("Arial", 20))
        self.green_light.setStyleSheet("QLabel { color: #00FF00; }")  # Bright green (on by default)
        
        lights_layout.addWidget(self.red_light)
        lights_layout.addWidget(self.yellow_light)
        lights_layout.addWidget(self.green_light)
        
        self.stack_light_container.setLayout(lights_layout)
        layout.addWidget(self.stack_light_container)
        
        # Status text
        self.stack_light_status_label = QLabel("Ready")
        self.stack_light_status_label.setFont(QFont("Arial", 10))
        self.stack_light_status_label.setStyleSheet("QLabel { color: #006600; font-weight: bold; }")
        layout.addWidget(self.stack_light_status_label)
        
        # Buzzer indicator
        self.buzzer_label = QLabel("🔇")
        self.buzzer_label.setFont(QFont("Arial", 16))
        self.buzzer_label.setFixedWidth(30)
        layout.addWidget(self.buzzer_label)
        
        layout.addStretch()  # Push everything to the left
        
        panel.setLayout(layout)
        return panel

    def on_wms_request_received(self, request_data):
        """Handle WMS request information"""
        try:
            self.request_count += 1
            timestamp = datetime.now().strftime('%H:%M:%S')
            # Format request information
            request_info = f"[{timestamp}] WMS REQUEST\n"
            request_info += f"Pick ID: {request_data.get('pickId', 'N/A')}\n"
            request_info += f"Quantity: {request_data.get('quantity', 'N/A')}\n"
            request_info += "-" * 30 + "\n"
            # Update display (prepend new requests)
            current_text = self.wms_request_display.toPlainText()
            if "Waiting for WMS requests..." in current_text:
                self.wms_request_display.setPlainText(request_info)
            else:
                self.wms_request_display.setPlainText(request_info + current_text)
            
            # Update statistics
            self.update_statistics()
            self.connection_status.setText("🟢 Monitoring: Active")
            logger.info(f"GUI received WMS request info: Pick ID {request_data.get('pickId')}")
            
        except Exception as e:
            logger.error(f"Error processing WMS request info: {e}")
    
    def on_robot_response_received(self, response_data):
        """Handle robot response information"""
        try:
            self.response_count += 1
            if response_data.get('pickSuccessful', False):
                self.successful_responses += 1
            
            timestamp = datetime.now().strftime('%H:%M:%S')
            
            # Format response information
            response_info = f"[{timestamp}] ROBOT RESPONSE\n"
            response_info += f"Pick ID: {response_data.get('pickId', 'N/A')}\n"
            response_info += f"Success: {'✅ Yes' if response_data.get('pickSuccessful') else '❌ No'}\n"
            response_info += f"Barcode: {response_data.get('itemBarcode', 'N/A')}\n"
            if response_data.get('errorMessage'):
                response_info += f"Error: {response_data.get('errorMessage')}\n"
            
            response_info += "-" * 30 + "\n"
            # Update display (prepend new responses)
            current_text = self.robot_response_display.toPlainText()
            if "Waiting for robot responses..." in current_text:
                self.robot_response_display.setPlainText(response_info)
            else:
                self.robot_response_display.setPlainText(response_info + current_text)
            
            # Update statistics
            self.update_statistics()
            logger.info(f"GUI received robot response info: Pick ID {response_data.get('pickId')}")
            
        except Exception as e:
            logger.error(f"Error processing robot response info: {e}")
    
    def update_statistics(self):
        """Update communication statistics display"""
        success_rate = "N/A"
        if self.response_count > 0:
            success_rate = f"{(self.successful_responses / self.response_count) * 100:.1f}%"
        
        last_activity = datetime.now().strftime('%H:%M:%S')
        
        stats_text = (
            f"Requests Received: {self.request_count}\n"
            f"Responses Sent: {self.response_count}\n"
            f"Success Rate: {success_rate}\n"
            f"Last Activity: {last_activity}"
        )
        
        self.stats_label.setText(stats_text)
    
    def on_pick_received(self, pick_id, quantity):
        """Handle received pick request"""
        message = f"[{datetime.now().strftime('%H:%M:%S')}] Pick request received: ID={pick_id}, Quantity={quantity}"
        self.log_display.append(message)
        self.status_label.setText(f"Status: Processing pick {pick_id}...")
        logger.info(message)
    
    def add_log_message(self, message):
        """Thread-safe method to add log messages"""
        self.log_display.append(message)

    def update_arm_visualization(self):
        """Update the arm visualization with current angles"""
        # Update animation if moving
        animation_completed = self.robotic_interface.update_animation()
        # Draw the arm
        self.draw_arm()
        
        # Update labels
        angles = self.robotic_interface.current_angles
        self.angles_label.setText(f"Joint Angles: θ1={angles[0]:.1f}°, θ2={angles[1]:.1f}°, θ3={angles[2]:.1f}°")
        
        # Calculate and display end effector position
        x, y = Kinematics.forward_kinematics(angles[0], angles[1], angles[2], *self.robotic_interface.lengths)
        self.position_label.setText(f"End Effector Position: ({x:.1f}, {y:.1f})")
        
        # Update status based on movement
        if self.robotic_interface.is_moving:
            progress = self.robotic_interface.animation_progress * 100
            if self.robotic_interface.is_returning_home:
                self.status_label.setText(f"Status: Returning to home position... {progress:.0f}%")
            else:
                pick_info = f" (Pick {self.robotic_interface.current_pick_id})" if self.robotic_interface.current_pick_id else ""
                self.status_label.setText(f"Status: Moving to target position{pick_info}... {progress:.0f}%")
        elif not "Processing pick" in self.status_label.text():
            self.status_label.setText("Status: Ready - Waiting for pick requests...")
    
    def draw_arm(self):
        """Draw the robotic arm using current joint angles"""
        angles = self.robotic_interface.current_angles
        theta1, theta2, theta3 = [np.radians(a) for a in angles]
        l1, l2, l3 = self.robotic_interface.lengths
        
        # Calculate joint positions
        x0, y0 = 0, 0  # Base
        x1 = l1 * np.cos(theta1)
        y1 = l1 * np.sin(theta1)
        x2 = x1 + l2 * np.cos(theta1 + theta2)
        y2 = y1 + l2 * np.sin(theta1 + theta2)
        x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
        
        x_coords = [x0, x1, x2, x3]
        y_coords = [y0, y1, y2, y3]
        
        # Clear and redraw
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        
        # Draw arm links with same blue color for all segments
        # Base to joint 1
        ax.plot([x0, x1], [y0, y1], '-', linewidth=4, color='blue', alpha=0.8)
        # Joint 1 to joint 2  
        ax.plot([x1, x2], [y1, y2], '-', linewidth=4, color='blue', alpha=0.8)
        # Joint 2 to end effector
        ax.plot([x2, x3], [y2, y3], '-', linewidth=4, color='blue', alpha=0.8)
        
        # Draw joints
        ax.plot([x0, x1, x2], [y0, y1, y2], 'o', markersize=10, color='black', label='Joints')
        
        # Highlight end effector
        ax.plot(x3, y3, 'o', markersize=15, label='End Effector', color='red')
        
        # Draw workspace circle
        max_reach = sum(self.robotic_interface.lengths)
        circle = Circle((0, 0), max_reach, fill=False, linestyle='--', alpha=0.3, color='gray')
        ax.add_patch(circle)
        
        # Always show home position as a small marker
        home_x, home_y = Kinematics.forward_kinematics(0, 0, 0, *self.robotic_interface.lengths)
        ax.plot(home_x, home_y, 'h', markersize=8, label='Home Position', color='green', alpha=0.6)
        
        # Only draw target position if show_target is True and target is different from current
        if self.robotic_interface.show_target:
            target_x, target_y = self.robotic_interface.target_position
            if abs(target_x - x3) > 1 or abs(target_y - y3) > 1:
                if self.robotic_interface.is_returning_home:
                    # Use different color/style for home target
                    ax.plot(target_x, target_y, 'h', markersize=12, label='Returning to Home', color='green', alpha=0.8)
                    ax.plot([x3, target_x], [y3, target_y], '--', alpha=0.6, color='green')
                else:
                    # Regular pick target
                    ax.plot(target_x, target_y, 's', markersize=12, label='Target Position', color='purple', alpha=0.7)
                    ax.plot([x3, target_x], [y3, target_y], '--', alpha=0.5, color='purple')
        
        # Add movement trail effect if moving
        if self.robotic_interface.is_moving:
            # Draw a faded trail of previous positions
            progress = self.robotic_interface.animation_progress
            trail_color = 'green' if self.robotic_interface.is_returning_home else 'red'
            
            for i in range(5):
                alpha_trail = 0.1 + (i / 5) * 0.3 * progress
                trail_progress = max(0, progress - (5-i) * 0.05)
                if trail_progress > 0:
                    # Calculate trail position
                    trail_angles = []
                    for j in range(3):
                        angle_diff = self.robotic_interface.target_angles[j] - self.robotic_interface.start_angles[j]
                        trail_angle = self.robotic_interface.start_angles[j] + angle_diff * trail_progress
                        trail_angles.append(trail_angle)
                    
                    # Calculate trail end effector position
                    trail_x, trail_y = Kinematics.forward_kinematics(
                        trail_angles[0], trail_angles[1], trail_angles[2], *self.robotic_interface.lengths
                    )
                    ax.plot(trail_x, trail_y, 'o', markersize=3, color=trail_color, alpha=alpha_trail)
        
        # Set equal aspect ratio and limits
        ax.set_xlim(-250, 250)
        ax.set_ylim(-250, 250)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Update title based on movement status
        if self.robotic_interface.is_moving:
            progress = self.robotic_interface.animation_progress * 100
            if self.robotic_interface.is_returning_home:
                ax.set_title(f'3-DOF Robotic Arm - Returning Home... {progress:.0f}%')
            else:
                ax.set_title(f'3-DOF Robotic Arm - Moving... {progress:.0f}%')
        else:
            ax.set_title('3-DOF Robotic Arm - Ready')
        
        self.canvas.draw()

    def poll_stack_light_status(self):
        """Poll stack light status from robot simulator"""
        try:
            # Make request to robot simulator
            response = requests.get(
                "http://localhost:8080/stack_light/status",
                timeout=2
            )
            
            if response.status_code == 200:
                data = response.json()
                logger.debug(f"Stack light data received: {data}")
                self.stack_light_update_signal.emit(data)
            else:
                logger.warning(f"Failed to get stack light status: HTTP {response.status_code}")
                
        except requests.exceptions.ConnectError:
            # Robot simulator not available - only log once to avoid spam
            if self.last_stack_light_status is not None:
                self.log_update_signal.emit(f"[{datetime.now().strftime('%H:%M:%S')}] Stack Light: Robot simulator not available")
                self.last_stack_light_status = None
                logger.info("Robot simulator connection lost")
        except requests.exceptions.Timeout:
            logger.warning("Stack light status request timed out")
        except Exception as e:
            logger.warning(f"Error polling stack light status: {e}")

    def update_stack_light_display(self, data):
        """Update stack light status display in activity log"""
        try:
            stack_light = data.get('stack_light', {})
            robot_state = data.get('robot_state', {})
            timestamp = datetime.now().strftime('%H:%M:%S')
            
            logger.debug(f"Updating stack light display with: {stack_light}, robot_state: {robot_state}")
            
            # Update visual indicators in the panel
            self.update_stack_light_panel(stack_light, robot_state)
            
            # Create status string
            lights = []
            if stack_light.get('red', False):
                lights.append("🔴 RED")
            if stack_light.get('yellow', False):
                lights.append("🟡 YELLOW")
            if stack_light.get('green', False):
                lights.append("🟢 GREEN")
            
            buzzer_status = "🔊 BUZZER ON" if stack_light.get('buzzer', False) else ""
            
            # Determine reason for current state
            if robot_state.get('emergency_activated', False):
                reason = "Emergency activated"
            elif not robot_state.get('door_closed', True):
                reason = "Door open"
            elif robot_state.get('is_busy', False):
                reason = "Robot working"
            else:
                reason = "Ready"
            
            # Create status message
            light_str = " + ".join(lights) if lights else "OFF"
            status_msg = f"Stack Light: {light_str}"
            if buzzer_status:
                status_msg += f" + {buzzer_status}"
            status_msg += f" ({reason})"
            
            # Log every update for debugging (remove duplicate check temporarily)
            logger.debug(f"Stack light status: {status_msg}")
            
            # Only log if status changed to avoid spam in production
            if self.last_stack_light_status != status_msg:
                self.log_update_signal.emit(f"[{timestamp}] {status_msg}")
                self.last_stack_light_status = status_msg
                logger.info(f"Stack light status changed: {status_msg}")
            
        except Exception as e:
            logger.error(f"Error updating stack light display: {e}")

    def update_stack_light_panel(self, stack_light, robot_state):
        """Update the visual stack light panel"""
        try:
            # Update red light
            if stack_light.get('red', False):
                self.red_light.setStyleSheet("QLabel { color: #FF0000; }")  # Bright red
            else:
                self.red_light.setStyleSheet("QLabel { color: #400000; }")  # Dark red
            
            # Update yellow light
            if stack_light.get('yellow', False):
                self.yellow_light.setStyleSheet("QLabel { color: #FFFF00; }")  # Bright yellow
            else:
                self.yellow_light.setStyleSheet("QLabel { color: #404000; }")  # Dark yellow
            
            # Update green light
            if stack_light.get('green', False):
                self.green_light.setStyleSheet("QLabel { color: #00FF00; }")  # Bright green
            else:
                self.green_light.setStyleSheet("QLabel { color: #004000; }")  # Dark green
            
            # Update buzzer indicator
            if stack_light.get('buzzer', False):
                self.buzzer_label.setText("🔊")
                self.buzzer_label.setStyleSheet("QLabel { color: #FF0000; }")
            else:
                self.buzzer_label.setText("🔇")
                self.buzzer_label.setStyleSheet("QLabel { color: #808080; }")
            
            # Update status text and color
            if robot_state.get('emergency_activated', False):
                self.stack_light_status_label.setText("EMERGENCY")
                self.stack_light_status_label.setStyleSheet("QLabel { color: #FF0000; font-weight: bold; }")
            elif not robot_state.get('door_closed', True):
                self.stack_light_status_label.setText("Door Open")
                self.stack_light_status_label.setStyleSheet("QLabel { color: #FFA500; font-weight: bold; }")
            elif robot_state.get('is_busy', False):
                self.stack_light_status_label.setText("Working")
                self.stack_light_status_label.setStyleSheet("QLabel { color: #FFA500; font-weight: bold; }")
            else:
                self.stack_light_status_label.setText("Ready")
                self.stack_light_status_label.setStyleSheet("QLabel { color: #006600; font-weight: bold; }")
                
        except Exception as e:
            logger.error(f"Error updating stack light panel: {e}")



def run():
    app = QApplication(sys.argv)
    gui = WMSRoboticArmGUI()
    gui.show()
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        logger.info("Application terminated by user")


if __name__ == "__main__":
    run()

