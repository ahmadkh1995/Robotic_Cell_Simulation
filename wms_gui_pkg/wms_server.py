"""
wms_server.py

This file implements a WMS server that interacts with robotic cells and a GUI.
It handles pick requests, manages robotic cell states, and communicates with the
robotic cells and GUI via HTTP requests.


Author: [Ahmad Kheirandish]
Date: [27/07/2025]

"""

from flask import Flask, request, jsonify
import requests
import logging
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional
import random

app = Flask(__name__)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# In-memory storage for pick requests and robotic cells
pick_requests: Dict[int, Dict] = {}
robotic_cells: List[str] = []  # List of robotic cell IP addresses
pick_confirmations: Dict[int, Dict] = {}  # Store pick confirmations
# Auto-incrementing pick ID counter starting from 100
_next_pick_id = 100

def get_next_pick_id() -> int:
    """Generate the next pick ID"""
    global _next_pick_id
    pick_id = _next_pick_id
    _next_pick_id += 1
    return pick_id

class PickStatus:
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

# Add GUI monitoring URL
GUI_MONITOR_URL = "http://localhost:8082"

def send_request_to_gui_monitor(request_data):
    """Send WMS request information to GUI monitor"""
    try:
        url = f"{GUI_MONITOR_URL}/wms_monitor/request"
        requests.post(url, json=request_data, timeout=2)
    except Exception as e:
        logger.debug(f"Could not send request info to GUI monitor: {e}")

def send_response_to_gui_monitor(response_data):
    """Send robot response information to GUI monitor"""
    try:
        url = f"{GUI_MONITOR_URL}/wms_monitor/response"
        requests.post(url, json=response_data, timeout=2)
    except Exception as e:
        logger.debug(f"Could not send response info to GUI monitor: {e}")

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({"status": "healthy", "timestamp": datetime.now().isoformat()})

@app.route('/pick', methods=['POST'])
def receive_pick_request():
    """Receive pick request from WMS"""
    try:
        data = request.get_json()
        
        # Validate request - pickId is now optional and auto-generated
        if not data or 'quantity' not in data:
            return jsonify({"error": "Missing required field: quantity"}), 400
        
        # Generate pickId automatically if not provided
        if 'pickId' in data:
            pick_id = data['pickId']
            # Validate provided pickId
            if not isinstance(pick_id, int):
                return jsonify({"error": "pickId must be an integer"}), 400
            # Check for duplicate pickId
            if pick_id in pick_requests:
                return jsonify({"error": f"pickId {pick_id} already exists"}), 409
        else:
            # Auto-generate pickId
            pick_id = get_next_pick_id()
        
        quantity = data['quantity']
        
        # Validate data types and values
        if not isinstance(quantity, int):
            return jsonify({"error": "quantity must be an integer"}), 400
        
        if quantity <= 0:
            return jsonify({"error": "quantity must be positive"}), 400
        
        # Store pick request
        pick_requests[pick_id] = {
            "pickId": pick_id,
            "quantity": quantity,
            "status": PickStatus.PENDING,
            "timestamp": datetime.now().isoformat(),
            "robotic_cell": None
        }
        
        # Send request info to GUI monitor immediately
        monitor_request_data = {
            "pickId": pick_id,
            "quantity": quantity,
            "timestamp": datetime.now().isoformat()
        }
        send_request_to_gui_monitor(monitor_request_data)
        
        logger.info(f"Received pick request: pickId={pick_id}, quantity={quantity}")
        
        # Process the pick request (async)
        threading.Thread(target=process_pick_request, args=(pick_id,)).start()
        
        return jsonify({
            "message": "Pick request received",
            "pickId": pick_id,
            "status": PickStatus.PENDING
        }), 202
        
    except Exception as e:
        logger.error(f"Error processing pick request: {str(e)}")
        return jsonify({"error": "Internal server error"}), 500

@app.route('/pick/<int:pick_id>', methods=['GET'])
def get_pick_status(pick_id: int):
    """Get status of a specific pick request"""
    if pick_id not in pick_requests:
        return jsonify({"error": "Pick request not found"}), 404
    
    return jsonify(pick_requests[pick_id])

@app.route('/picks', methods=['GET'])
def get_all_picks():
    """Get all pick requests"""
    return jsonify(list(pick_requests.values()))

@app.route('/robotic-cells', methods=['POST'])
def register_robotic_cell():
    """Register a new robotic cell"""
    try:
        data = request.get_json()
        if not data or 'ip_address' not in data:
            return jsonify({"error": "Missing ip_address field"}), 400
        
        ip_address = data['ip_address']
        if ip_address not in robotic_cells:
            robotic_cells.append(ip_address)
            logger.info(f"Registered robotic cell: {ip_address}")
        
        return jsonify({"message": "Robotic cell registered", "ip_address": ip_address})
        
    except Exception as e:
        logger.error(f"Error registering robotic cell: {str(e)}")
        return jsonify({"error": "Internal server error"}), 500

@app.route('/robotic-cells', methods=['GET'])
def get_robotic_cells():
    """Get all registered robotic cells"""
    return jsonify({"robotic_cells": robotic_cells})

@app.route('/confirmPick', methods=['POST'])
def receive_pick_confirmation():
    """Receive pick confirmation from robotic cell"""
    try:
        data = request.get_json()
        
        # Validate request - all fields are required
        required_fields = ['pickId', 'pickSuccessful', 'errorMessage', 'itemBarcode']
        if not data or not all(field in data for field in required_fields):
            return jsonify({"error": f"Missing required fields: {required_fields}"}), 400
        
        pick_id = data['pickId']
        pick_successful = data['pickSuccessful']
        item_barcode = data['itemBarcode']
        error_message = data['errorMessage']
        
        # Validate data types
        if not isinstance(pick_id, int) or not isinstance(pick_successful, bool):
            return jsonify({"error": "Invalid data types for pickId or pickSuccessful"}), 400
        
        # itemBarcode should be integer or null
        if item_barcode is not None and not isinstance(item_barcode, int):
            return jsonify({"error": "itemBarcode must be integer or null"}), 400
        
        # errorMessage should be string or null
        if error_message is not None and not isinstance(error_message, str):
            return jsonify({"error": "errorMessage must be string or null"}), 400
        
        # Store confirmation
        pick_confirmations[pick_id] = {
            "pickId": pick_id,
            "pickSuccessful": pick_successful,
            "itemBarcode": item_barcode,
            "errorMessage": error_message,
            "confirmed_at": datetime.now().isoformat()
        }
        
        # Send response info to GUI monitor (exact structure only)
        monitor_response_data = {
            "pickId": pick_id,
            "pickSuccessful": pick_successful,
            "errorMessage": error_message,
            "itemBarcode": item_barcode
        }
        send_response_to_gui_monitor(monitor_response_data)
        
        # Update original pick request if it exists
        if pick_id in pick_requests:
            pick_requests[pick_id].update({
                "status": PickStatus.COMPLETED if pick_successful else PickStatus.FAILED,
                "pickSuccessful": pick_successful,
                "itemBarcode": item_barcode,
                "errorMessage": error_message,
                "confirmed_at": datetime.now().isoformat()
            })
        
        logger.info(f"Received pick confirmation: pickId={pick_id}, successful={pick_successful}, barcode={item_barcode}")
        
        return jsonify({
            "message": "Pick confirmation received",
            "pickId": pick_id
        }), 200
        
    except Exception as e:
        logger.error(f"Error processing pick confirmation: {str(e)}")
        return jsonify({"error": "Internal server error"}), 500

def process_pick_request(pick_id: int):
    """Process pick request asynchronously"""
    try:
        if pick_id not in pick_requests:
            logger.error(f"Pick request {pick_id} not found")
            return
        
        pick_request = pick_requests[pick_id]
        pick_request["status"] = PickStatus.IN_PROGRESS
        
        # Remove this duplicate monitoring call - it's already sent in the main endpoint
        # monitor_request_data = {
        #     "pickId": pick_id,
        #     "quantity": pick_request["quantity"],
        #     "timestamp": datetime.now().isoformat()
        # }
        # send_request_to_gui_monitor(monitor_request_data)
        
        # Find available robotic cell
        robotic_cell = find_available_robotic_cell()
        if not robotic_cell:
            logger.warning(f"No available robotic cells for pick {pick_id}")
            pick_request["status"] = PickStatus.FAILED
            pick_request["error"] = "No available robotic cells"
            return
        
        pick_request["robotic_cell"] = robotic_cell
        logger.info(f"Assigned pick {pick_id} to robotic cell {robotic_cell}")
        
        # Send request to robotic cell
        success = send_pick_to_robotic_cell(robotic_cell, pick_request)
        
        if success:
            # Keep status as IN_PROGRESS - will be updated when confirmation is received
            logger.info(f"Pick {pick_id} sent to robotic cell successfully, waiting for confirmation")
        else:
            pick_request["status"] = PickStatus.FAILED
            pick_request["error"] = "Failed to communicate with robotic cell"
            logger.error(f"Pick {pick_id} failed to send to robotic cell")
            
    except Exception as e:
        logger.error(f"Error processing pick request {pick_id}: {str(e)}")
        if pick_id in pick_requests:
            pick_requests[pick_id]["status"] = PickStatus.FAILED
            pick_requests[pick_id]["error"] = str(e)

def find_available_robotic_cell() -> Optional[str]:
    """Find an available robotic cell"""
    # Simple round-robin selection
    # In a real implementation, you might check cell status/availability
    if not robotic_cells:
        return None
    
    # For demo purposes, return the first available cell
    return robotic_cells[0] if robotic_cells else None

def send_pick_to_robotic_cell(robotic_cell_ip: str, pick_request: Dict) -> bool:
    """Send pick request to robotic cell"""
    try:
        url = f"http://{robotic_cell_ip}:8080/pick"
        
        payload = {
            "pickId": pick_request["pickId"],
            "quantity": pick_request["quantity"]
        }
        
        logger.info(f"Sending pick request to {url}: {payload}")
        
        response = requests.post(
            url,
            json=payload,
            timeout=30,
            headers={'Content-Type': 'application/json'}
        )
        
        if response.status_code == 200:
            logger.info(f"Successfully sent pick request to {robotic_cell_ip}")
            return True
        else:
            logger.error(f"Failed to send pick request to {robotic_cell_ip}: {response.status_code}")
            return False
            
    except requests.exceptions.RequestException as e:
        logger.error(f"Network error sending pick request to {robotic_cell_ip}: {str(e)}")
        return False
    except Exception as e:
        logger.error(f"Unexpected error sending pick request to {robotic_cell_ip}: {str(e)}")
        return False



def run():
    # Add some default robotic cells for testing
    robotic_cells.extend([
        "127.0.0.1",  # localhost where the simulator runs
        "localhost"   # alternative localhost reference
    ])
    
    logger.info("Starting WMS Server...")
    logger.info(f"Registered robotic cells: {robotic_cells}")
    logger.info("WMS Server listening on port 5000")
    logger.info("Pick confirmation endpoint: http://localhost:5000/confirmPick")
    app.run(host='0.0.0.0', port=5000, debug=False)


if __name__ == "__main__":
    run()

