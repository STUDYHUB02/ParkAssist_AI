**ParkAssist AI**

**Overview**
ParkAssist AI is a smart parking alert system that integrates an Arduino-based ultrasonic sensor with a computer vision-based object detection system. The system detects obstacles using an HC-SR04 ultrasonic sensor and provides a real-time alert via a buzzer. Simultaneously, a Python script utilizes YOLO object detection to identify objects through an IP camera feed, providing a visual alarm when obstacles are detected.

**Features**
- Ultrasonic distance measurement using an HC-SR04 sensor
- Audible alert using a buzzer if an obstacle is within 30 cm
- Serial communication between Arduino and Python script
- Object detection using YOLOv3
- Real-time video processing from an IP camera
- Visual alarm overlay when an obstacle is detected

**Project Components**
1. **Arduino-based sensor system** (arduino.cpp)
   - Measures distance using HC-SR04
   - Sends an alert signal via serial communication
   - Activates a buzzer if an obstacle is detected
2. **Python-based object detection** (script.py)
   - Reads Arduino alerts
   - Captures video from an IP camera
   - Processes frames using YOLOv3
   - Displays detected objects and triggers a red blinking alarm overlay

**Hardware Requirements**
- Arduino Uno (or compatible board)
- HC-SR04 Ultrasonic Sensor
- Buzzer module
- Jumper wires
- Computer with Python and OpenCV installed
- IP Camera or mobile device running an IP webcam app

**Software Requirements**
- **Arduino IDE** (to upload `arduino.cpp` to Arduino)
- **Python 3.x**
- **Required Python Libraries:**
  - OpenCV (`cv2`)
  - NumPy (`numpy`)
  - PySerial (`serial` and `serial.tools.list_ports`)
  - YOLOv3 model files (`yolov3.weights`, `yolov3.cfg`, `coco.names`)
- **IP Webcam App** (if using a smartphone as a camera)

**Installation & Setup**
**Step 1: Upload Arduino Code**
1. Connect the Arduino to your PC via USB.
2. Open `arduino.cpp` in the Arduino IDE.
3. Select the correct board and port from **Tools > Board** and **Tools > Port**.
4. Click **Upload** to flash the code onto the Arduino.

**Step 2: Install Python Dependencies**
Ensure you have Python installed and then run:
```sh
pip install opencv-python numpy pyserial
```

**Step 3: Set Up YOLOv3**
1. Download YOLOv3 model files:
   - `yolov3.weights`
   - `yolov3.cfg`
   - `coco.names`
2. Place them in the same directory as `script.py`.

**Step 4: Run the Python Script**
1. Connect your IP camera or mobile device running an IP webcam app.
2. Update the `url` variable in `script.py` with your camera's stream URL.
3. Run the script using:
```sh
python script.py
```
4. The program will attempt to connect to the Arduino automatically.
5. If an obstacle is detected, a red blinking alarm overlay will appear on the video feed.

**Usage**
- Ensure the Arduino is powered and connected before running `script.py`.
- Press `q` to exit the Python program.
- Adjust the object detection confidence threshold in `script.py` if needed.

**Troubleshooting**
### No Arduino Device Found
- Ensure the Arduino is connected via USB.
- Check the serial port using the Arduino IDE (**Tools > Port**).
- Restart the Python script.

**Camera Feed Not Working**
- Ensure the correct IP webcam URL is set in `script.py`.
- Check the IP camera connection by opening the URL in a browser.

**YOLO Not Detecting Objects**
- Ensure YOLO model files are correctly placed.
- Increase confidence threshold if false positives occur.

**Author**
Developed by Saahil A Vishwakarma and Vrashabha Nilajagi.
