"""
robot_cell_monitor.py

This file implements a control panel for controlling robotic cells.
It provides options to control the door and emergency button of the robotic cell.
It uses a GUI built with PyQt5 and communicates with a REST API to perform actions.


Author: [Ahmad Kheirandish]
Date: [27/07/2025]

"""


import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QPushButton, QDialog, QRadioButton, 
                           QButtonGroup, QTextEdit, QGroupBox, QMessageBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
import requests
import datetime
from typing import Optional

class DoorControlThread(QThread):
    """Thread for handling door control API requests"""
    finished = pyqtSignal(bool, str)
    
    def __init__(self, url: str, close_door: bool):
        super().__init__()
        self.url = url
        self.close_door = close_door
        
    def run(self):
        """Execute the door control request"""
        try:
            payload = {"close": self.close_door}
            
            response = requests.post(
                f"{self.url}/door",
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=10
            )
            
            if response.status_code == 200:
                response_data = response.json()
                message = response_data.get('message', 'Success')
                self.finished.emit(True, message)
            else:
                error_msg = f"HTTP {response.status_code}: {response.text}"
                self.finished.emit(False, error_msg)
                
        except requests.exceptions.ConnectionError:
            self.finished.emit(False, "Cannot connect to robot cell API")
        except requests.exceptions.Timeout:
            self.finished.emit(False, "Request timeout")
        except Exception as e:
            self.finished.emit(False, f"Unexpected error: {str(e)}")

class DoorControlDialog(QDialog):
    """Dialog for door control options"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.result = None
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the dialog UI"""
        self.setWindowTitle("Door Control")
        self.setFixedSize(300, 200)
        self.setModal(True)
        
        # Main layout
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title label
        title_label = QLabel("Select Door Action:")
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Radio button group
        radio_group = QGroupBox()
        radio_layout = QVBoxLayout()
        radio_group.setLayout(radio_layout)
        
        self.button_group = QButtonGroup()
        
        self.close_radio = QRadioButton("Close Door")
        self.close_radio.setChecked(True)
        self.open_radio = QRadioButton("Open Door")
        
        self.button_group.addButton(self.close_radio, 0)
        self.button_group.addButton(self.open_radio, 1)
        
        radio_layout.addWidget(self.close_radio)
        radio_layout.addWidget(self.open_radio)
        layout.addWidget(radio_group)
        
        # Button layout
        button_layout = QHBoxLayout()
        
        self.confirm_btn = QPushButton("Confirm")
        self.confirm_btn.clicked.connect(self.confirm_action)
        
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.cancel_action)
        
        button_layout.addWidget(self.confirm_btn)
        button_layout.addWidget(self.cancel_btn)
        
        layout.addLayout(button_layout)
        
    def confirm_action(self):
        """Handle confirm button click"""
        self.result = self.close_radio.isChecked()
        self.accept()
        
    def cancel_action(self):
        """Handle cancel button click"""
        self.result = None
        self.reject()
        
    def get_result(self) -> Optional[bool]:
        """Get the dialog result"""
        if self.exec_() == QDialog.Accepted:
            return self.result
        return None

class EmergencyControlThread(QThread):
    """Thread for handling emergency control API requests"""
    finished = pyqtSignal(bool, str)
    
    def __init__(self, url: str, activate: bool):
        super().__init__()
        self.url = url
        self.activate = activate
        
    def run(self):
        """Execute the emergency control request"""
        try:
            payload = {"activate": self.activate}
            
            response = requests.post(
                f"{self.url}/emergency",
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=10
            )
            
            if response.status_code == 200:
                response_data = response.json()
                message = response_data.get('message', 'Success')
                self.finished.emit(True, message)
            else:
                error_msg = f"HTTP {response.status_code}: {response.text}"
                self.finished.emit(False, error_msg)
                
        except requests.exceptions.ConnectionError:
            self.finished.emit(False, "Cannot connect to robot cell API")
        except requests.exceptions.Timeout:
            self.finished.emit(False, "Request timeout")
        except Exception as e:
            self.finished.emit(False, f"Unexpected error: {str(e)}")

class EmergencyControlDialog(QDialog):
    """Dialog for emergency control options"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.result = None
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the dialog UI"""
        self.setWindowTitle("Emergency Control")
        self.setFixedSize(300, 200)
        self.setModal(True)
        
        # Main layout
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title label
        title_label = QLabel("Select Emergency Action:")
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Radio button group
        radio_group = QGroupBox()
        radio_layout = QVBoxLayout()
        radio_group.setLayout(radio_layout)
        
        self.button_group = QButtonGroup()
        
        self.activate_radio = QRadioButton("Activate Emergency")
        self.activate_radio.setChecked(True)
        self.release_radio = QRadioButton("Release Emergency")
        
        self.button_group.addButton(self.activate_radio, 0)
        self.button_group.addButton(self.release_radio, 1)
        
        radio_layout.addWidget(self.activate_radio)
        radio_layout.addWidget(self.release_radio)
        layout.addWidget(radio_group)
        
        # Button layout
        button_layout = QHBoxLayout()
        
        self.confirm_btn = QPushButton("Confirm")
        self.confirm_btn.clicked.connect(self.confirm_action)
        
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.cancel_action)
        
        button_layout.addWidget(self.confirm_btn)
        button_layout.addWidget(self.cancel_btn)
        
        layout.addLayout(button_layout)
        
    def confirm_action(self):
        """Handle confirm button click"""
        self.result = self.activate_radio.isChecked()
        self.accept()
        
    def cancel_action(self):
        """Handle cancel button click"""
        self.result = None
        self.reject()
        
    def get_result(self) -> Optional[bool]:
        """Get the dialog result"""
        if self.exec_() == QDialog.Accepted:
            return self.result
        return None

class RobotCellControlPanel(QMainWindow):
    """Main GUI panel for robot cell control"""
    
    def __init__(self):
        super().__init__()
        self.robot_cell_url = "http://localhost:8080"
        self.door_thread = None
        self.emergency_thread = None
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the main GUI components"""
        self.setWindowTitle("Robot Cell Control Panel")
        self.setGeometry(100, 100, 400, 450)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Title
        title_label = QLabel("Robot Cell Control Panel")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # Controls group
        controls_group = QGroupBox("Controls")
        controls_layout = QVBoxLayout()
        controls_group.setLayout(controls_layout)
        
        self.door_control_btn = QPushButton("Control Cell Door")
        self.door_control_btn.setFont(QFont("Arial", 12))
        self.door_control_btn.setMinimumHeight(40)
        self.door_control_btn.clicked.connect(self.show_door_control)
        controls_layout.addWidget(self.door_control_btn)
        
        self.emergency_control_btn = QPushButton("Emergency Button")
        self.emergency_control_btn.setFont(QFont("Arial", 12))
        self.emergency_control_btn.setMinimumHeight(40)
        self.emergency_control_btn.clicked.connect(self.show_emergency_control)
        controls_layout.addWidget(self.emergency_control_btn)
        
        main_layout.addWidget(controls_group)
        
        # Activity Log group
        activity_group = QGroupBox("Activity Log")
        activity_layout = QVBoxLayout()
        activity_group.setLayout(activity_layout)
        
        self.activity_log = QTextEdit()
        self.activity_log.setReadOnly(True)
        self.activity_log.setMaximumHeight(150)
        self.activity_log.setFont(QFont("Courier", 9))
        activity_layout.addWidget(self.activity_log)
        
        main_layout.addWidget(activity_group)
        
        # Initial log message
        self.add_status_message("Robot Cell Control Panel initialized")
        
    def add_status_message(self, message: str):
        """Add a message to the activity log"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        
        # Add to GUI log
        self.activity_log.append(formatted_message)
        
        # Also print to console
        print(formatted_message)
        
    def show_door_control(self):
        """Show door control dialog and handle the result"""
        self.add_status_message("Opening door control dialog...")
        
        dialog = DoorControlDialog(self)
        result = dialog.get_result()
        
        if result is not None:
            action = "close" if result else "open"
            self.add_status_message(f"User selected: {action} door")
            self.send_door_request(result)
        else:
            self.add_status_message("Door control operation canceled")
            
    def send_door_request(self, close_door: bool):
        """Send door control request to robot cell API"""
        # Disable button during request
        self.door_control_btn.setEnabled(False)
        
        url = self.robot_cell_url
        action = "close" if close_door else "open"
        self.add_status_message(f"Sending request to {action} door...")
        
        # Create and start thread
        self.door_thread = DoorControlThread(url, close_door)
        self.door_thread.finished.connect(self.handle_door_response)
        self.door_thread.start()
        
    def handle_door_response(self, success: bool, message: str):
        """Handle door control API response"""
        # Re-enable button
        self.door_control_btn.setEnabled(True)
        
        if success:
            self.add_status_message(f"✓ {message}")
            QMessageBox.information(self, "Success", message)
        else:
            self.add_status_message(f"✗ Error - {message}")
            QMessageBox.critical(self, "Error", f"Failed to control door:\n{message}")
    
    def show_emergency_control(self):
        """Show emergency control dialog and handle the result"""
        self.add_status_message("Opening emergency control dialog...")
        
        dialog = EmergencyControlDialog(self)
        result = dialog.get_result()
        
        if result is not None:
            action = "activate" if result else "release"
            self.add_status_message(f"User selected: {action} emergency")
            self.send_emergency_request(result)
        else:
            self.add_status_message("Emergency control operation canceled")
            
    def send_emergency_request(self, activate: bool):
        """Send emergency control request to robot cell API"""
        # Disable button during request
        self.emergency_control_btn.setEnabled(False)
        
        url = self.robot_cell_url
        action = "activate" if activate else "release"
        self.add_status_message(f"Sending request to {action} emergency...")
        
        # Create and start thread
        self.emergency_thread = EmergencyControlThread(url, activate)
        self.emergency_thread.finished.connect(self.handle_emergency_response)
        self.emergency_thread.start()
        
    def handle_emergency_response(self, success: bool, message: str):
        """Handle emergency control API response"""
        # Re-enable button
        self.emergency_control_btn.setEnabled(True)
        
        if success:
            self.add_status_message(f"✓ {message}")
            QMessageBox.information(self, "Success", message)
        else:
            self.add_status_message(f"✗ Error - {message}")
            QMessageBox.critical(self, "Error", f"Failed to control emergency:\n{message}")

def run():
    app = QApplication(sys.argv)
    try:
        window = RobotCellControlPanel()
        window.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Application interrupted by user")
    except Exception as e:
        print(f"Error running application: {e}")

if __name__ == "__main__":
    run()


