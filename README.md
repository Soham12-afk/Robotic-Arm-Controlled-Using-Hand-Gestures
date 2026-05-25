🦾 Gesture-Controlled Robotic Arm
Control a 6-DOF robotic arm in real-time using hand gestures via webcam. No buttons, no joystick — just your hand.
 
<!-- Suggested: a GIF or photo of your arm moving while you gesture --> 
________________________________________
How It Works
A Python script (MediaPipe) detects your hand through a webcam, maps its position to a 3×3 grid zone, and sends a single-character UDP command to an ESP32 over WiFi. The ESP32 drives servos (via PCA9685) and a stepper motor in real time.
Webcam → MediaPipe → Zone Detection → UDP → ESP32 → PCA9685 + Stepper
________________________________________
Hardware
Component	Details
Microcontroller	ESP32
Servo Driver	PCA9685 (I2C, address 0x40)
Servos	5× (shoulder, elbow ×2 mirrored, wrist, gripper)
Base Rotation	Stepper motor (DIR=GPIO4, STEP=GPIO5)
Wiring	SDA → GPIO21, SCL → GPIO22
 
<!-- Suggested: your actual wiring photo or a Fritzing diagram --> 
________________________________________
Software
•	Python 3.x
•	opencv-python
•	mediapipe
•	numpy
•	Arduino IDE with: 
o	WiFi.h, WiFiUdp.h (built-in ESP32)
o	HCPCA9685 library
________________________________________
Setup
ESP32
1.	Open PRACTICE_ARM.ino in Arduino IDE
2.	Set your WiFi credentials: 
3.	const char* ssid     = "YOUR_SSID";const char* password = "YOUR_PASSWORD";
4.	Flash to ESP32
5.	Open Serial Monitor (115200 baud) → note the IP printed: IP: 192.168.x.x
Python
1.	Install dependencies: 
2.	pip install opencv-python mediapipe numpy
3.	Set the ESP32 IP in PRATICEEE.py: 
4.	ESP32_IP = "192.168.x.x"   # paste IP from Serial Monitor
5.	Run: 
6.	python PRATICEEE.py
________________________________________
Control Zones
Hold your hand in a grid zone to send commands. Open hand vs. fist changes the action.
┌─────────────┬──────────────┬──────────────┐
│  ELBOW UP   │ SHOULDER UP  │  ELBOW DOWN  │
│  open → P   │  open → C   │   open → p   │
├─────────────┼──────────────┼──────────────┤
│  BASE LEFT  │  CLAW ZONE  │  BASE RIGHT  │
│  open → S   │  open → F   │   open → O   │
│  fist → L   │  fist → f   │   fist → R   │
├─────────────┼──────────────┼──────────────┤
│  WRIST UP   │ SHOULDER DN  │  WRIST DOWN  │
│  open → U   │  open → c   │   open → G   │
└─────────────┴──────────────┴──────────────┘
Command	Action
P / p	Elbow up / down
C / c	Shoulder up / down
U / G	Wrist up / down
S / O	Base rotate left / right
F / f	Gripper open / close
L / R	Wrist2 CCW / CW
 
<!-- Suggested: screenshot of the Python HUD showing the 3x3 grid --> 
________________________________________
Timing
Parameter	Value	Reason
Python send delay	350ms	> ESP32 debounce
ESP32 debounce	250ms	Prevents duplicate execution
Servo settle	60ms	Servo reaches position before next command
________________________________________
Demo
 
<!-- Suggested: short GIF of the arm responding to your gestures --> 
________________________________________
File Structure
├── PRACTICE_ARM.ino   # ESP32 firmware
├── PRATICEEE.py       # Python gesture controller
└── README.md
________________________________________
Tuning
•	Speed: Adjust inc_1 through inc_4 in the .ino for step size per command
•	Smoothing: Change SMOOTH_N in Python (frames averaged for hand position)
•	Fist sensitivity: Change FIST_THRESHOLD (default: 4 fingers closed = fist)
•	Servo limits: MIN_* / MAX_* constants in .ino to prevent over-rotation

