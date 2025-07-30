#!/bin/bash


echo "🧹 Cleaning up any existing processes..."

# Kill any remaining processes on our ports
fuser -k 5000/tcp 2>/dev/null
fuser -k 8080/tcp 2>/dev/null
fuser -k 8081/tcp 2>/dev/null
fuser -k 8082/tcp 2>/dev/null

# Kill any remaining ROS2 processes and Python scripts related to the system
pkill -f "barcode_scanner.py" 2>/dev/null
pkill -f "door_handler.py" 2>/dev/null
pkill -f "emergency_button_handler.py" 2>/dev/null
pkill -f "stack_light_handler.py" 2>/dev/null
pkill -f "robotic_cell_simulator.py" 2>/dev/null
pkill -f "wms_server.py" 2>/dev/null
pkill -f "robot_cell_monitor.py" 2>/dev/null
pkill -f "robot_cell_control.py" 2>/dev/null

# Give processes time to clean up
sleep 2

echo "✅ Cleanup completed. Starting fresh instances..."


# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Stopping all servers and ROS2 nodes..."
    kill $WMS_PID 2>/dev/null
    kill $LAUNCH_PID 2>/dev/null
    kill $GUI_PID 2>/dev/null
    kill $CONTROL_PID 2>/dev/null

    # Kill any remaining processes on our ports
    fuser -k 5000/tcp 2>/dev/null
    fuser -k 8080/tcp 2>/dev/null
    fuser -k 8081/tcp 2>/dev/null
    fuser -k 8082/tcp 2>/dev/null

    # Kill any remaining ROS2 processes and Python scripts related to the system
    pkill -f "barcode_scanner.py" 2>/dev/null
    pkill -f "door_handler.py" 2>/dev/null
    pkill -f "emergency_button_handler.py" 2>/dev/null
    pkill -f "stack_light_handler.py" 2>/dev/null
    pkill -f "robotic_cell_simulator.py" 2>/dev/null
    pkill -f "wms_server.py" 2>/dev/null
    pkill -f "robot_cell_monitor.py" 2>/dev/null
    pkill -f "robot_cell_control.py" 2>/dev/null

    echo "All servers and ROS2 nodes stopped."
    exit
}

# Start WMS Server
echo "Starting WMS Server..."
python3 wms_server.py &
WMS_PID=$!
sleep 2

# Start ROS2 Cell Simulation Launch File
echo "Starting ROS2 Cell Simulation (all nodes via launch file)..."
ros2 launch robotic_cell_area_simulation cell_simulation.launch.py &
LAUNCH_PID=$!
sleep 5

# Start wms robotic cell HMI for monitoring
echo "Starting WMS Robotic Cell HMI for monitoring..."
python3 robot_cell_monitor.py &
GUI_PID=$!
sleep 3

# Start Robotic Arm GUI with control panel
echo "Starting Robotic Arm GUI with control panel..."
python3 robot_cell_control.py &
CONTROL_PID=$!
sleep 3