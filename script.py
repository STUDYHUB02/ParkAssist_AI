import cv2
import numpy as np
import os
import serial
import time
import subprocess
import serial.tools.list_ports

# Automatically detect the Arduino COM port
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'CH340' in port.description:
            return port.device
    return None

# Release and reset the COM port using a system command
def reset_serial_port(port):
    subprocess.run(f"mode {port}", shell=True)
    time.sleep(1)

# Find and connect to Arduino
arduino_port = find_arduino_port()
if arduino_port:
    reset_serial_port(arduino_port)
    try:
        arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=0.1)
        print(f"Connected to Arduino on {arduino_port}")
    except Exception as e:
        print(f"Failed to connect: {e}")
        exit(1)
else:
    print("No Arduino device found. Please check the connection.")
    exit(1)

# Load YOLO model
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Load COCO names
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# IP Webcam URL
url = "https://10.0.1.94:8080/video"
cap = cv2.VideoCapture(url)

# Alarm variables
alarm_active = False
blink_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from camera.")
        break

    height, width, channels = frame.shape

    # Check for alert signal from Arduino
    if arduino.in_waiting > 0:
        alert_signal = arduino.readline().decode('utf-8').strip()
        if alert_signal == "ALERT":
            alarm_active = True

    # YOLO object detection
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (256, 256), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    boxes, confidences, class_ids = [], [], []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.3:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.4)

    # Display detected objects
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Red blinking alarm overlay if alert is active
    if alarm_active:
        if blink_count % 20 < 10:  # Blinking effect
            cv2.rectangle(frame, (0, 0), (width, height), (0, 0, 255), thickness=-1)
            cv2.putText(frame, "ALARM: Object Detected!", (50, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        blink_count += 1
        if blink_count > 60:  # Stop alarm after 3 blinks
            alarm_active = False
            blink_count = 0

    # Display the frame
    cv2.imshow('ParkAssist AI', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()