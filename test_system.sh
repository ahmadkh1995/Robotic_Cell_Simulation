#!/bin/bash

echo "Testing System Health and Pick Operation..."
echo "=========================================="

# Test WMS server health
echo "1. WMS Server Health:"
if curl -s http://localhost:5000/health > /dev/null; then
    curl -s http://localhost:5000/health | python3 -m json.tool
else
    echo "❌ WMS Server not responding. Make sure ./run_all.sh is running."
    exit 1
fi

echo -e "\n2. Robotic Cell Simulator Health:"
if curl -s http://localhost:8080/health > /dev/null; then
    curl -s http://localhost:8080/health | python3 -m json.tool
else
    echo "❌ Robotic Cell Simulator not responding. Make sure ./run_all.sh is running."
    exit 1
fi

echo -e "\n3. Robotic Arm GUI Health:"
if curl -s http://localhost:8081/health > /dev/null; then
    curl -s http://localhost:8081/health | python3 -m json.tool
else
    echo "❌ Robotic Arm GUI not responding. Make sure ./run_all.sh is running."
    exit 1
fi

echo -e "\n4. Sending Pick Request:"
RESPONSE=$(curl -s -X POST http://localhost:5000/pick \
  -H "Content-Type: application/json" \
  -d '{"quantity": 3}')
echo $RESPONSE | python3 -m json.tool

# Extract pickId from response
PICK_ID=$(echo $RESPONSE | python3 -c "import sys, json; print(json.load(sys.stdin)['pickId'])" 2>/dev/null || echo "100")

echo -e "\n5. Processing workflow (waiting 10 seconds)..."
for i in {10..1}; do
    echo -n "$i... "
    sleep 1
done
echo "Done!"

echo -e "\n6. Pick Status:"
curl -s http://localhost:5000/pick/$PICK_ID | python3 -m json.tool

echo -e "\n7. System Status:"
echo "Robotic Cell:"
curl -s http://localhost:8080/status | python3 -m json.tool

echo -e "\nGUI Status:"
curl -s http://localhost:8081/status | python3 -m json.tool

echo -e "\n=========================================="
echo "✅ Test completed!"


