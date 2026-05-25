**🦾 Gesture-Controlled Robotic Arm**

Control a 6-DOF robotic arm in real-time using hand gestures via webcam. No buttons, no joystick — just your hand.

\<\!-- Suggested: a GIF or photo of your arm moving while you gesture \--\>

---

**How It Works**

A Python script (MediaPipe) detects your hand through a webcam, maps its position to a 3×3 grid zone, and sends a single-character UDP command to an ESP32 over WiFi. The ESP32 drives servos (via PCA9685) and a stepper motor in real time.

Webcam → MediaPipe → Zone Detection → UDP → ESP32 → PCA9685 \+ Stepper

---

**Hardware**

| Component | Details |
| :---- | :---- |
| Microcontroller | ESP32 |
| Servo Driver | PCA9685 (I2C, address 0x40) |
| Servos | 5× (shoulder, elbow ×2 mirrored, wrist, gripper) |
| Base Rotation | Stepper motor (DIR=GPIO4, STEP=GPIO5) |
| Wiring | SDA → GPIO21, SCL → GPIO22 |

 

---

**Software**

* Python 3.x  
* opencv-python  
* mediapipe  
* numpy  
* Arduino IDE with:  
  * WiFi.h, WiFiUdp.h (built-in ESP32)  
  * [HCPCA9685](https://github.com/HobbyComponents/HCPCA9685) library

---

**Setup**

**ESP32**

1. Open PRACTICE\_ARM.ino in Arduino IDE  
2. Set your WiFi credentials:  
3. const char\* ssid 	\= "YOUR\_SSID";const char\* password \= "YOUR\_PASSWORD";  
4. Flash to ESP32  
5. Open Serial Monitor (115200 baud) → note the IP printed: IP: 192.168.x.x

**Python**

1. Install dependencies:  
2. pip install opencv-python mediapipe numpy  
3. Set the ESP32 IP in PRATICEEE.py:  
4. ESP32\_IP \= "192.168.x.x"   \# paste IP from Serial Monitor  
5. Run:  
6. python PRATICEEE.py

---

**Control Zones**

Hold your hand in a grid zone to send commands. Open hand vs. fist changes the action.

| Command | Action |
| :---- | :---- |
| P / p | Elbow up / down |
| C / c | Shoulder up / down |
| U / G | Wrist up / down |
| S / O | Base rotate left / right |
| F / f | Gripper open / close |
| L / R | Wrist2 CCW / CW |

---

**Timing**

| Parameter | Value | Reason |
| :---- | :---- | :---- |
| Python send delay | 350ms | \> ESP32 debounce |
| ESP32 debounce | 250ms | Prevents duplicate execution |
| Servo settle | 60ms | Servo reaches position before next command |

---

**Demo**

\<\!-- Suggested: short GIF of the arm responding to your gestures \--\>

---

**File Structure**

├── PRACTICE\_ARM.ino   \# ESP32 firmware

├── PRATICEEE.py   	\# Python gesture controller

└── README.md

---

**Tuning**

* **Speed**: Adjust inc\_1 through inc\_4 in the .ino for step size per command  
* **Smoothing**: Change SMOOTH\_N in Python (frames averaged for hand position)  
* **Fist sensitivity**: Change FIST\_THRESHOLD (default: 4 fingers closed \= fist)  
* **Servo limits**: MIN\_\* / MAX\_\* constants in .ino to prevent over-rotation

 

