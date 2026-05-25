# Robotic-Arm-Controlled-Using-Hand-Gestures
Control a robotic arm with hand gestures. Webcam + MediaPipe detects hand zone (3x3 grid). Python sends UDP commands to ESP32, which drives 5 servos via PCA9685 and a stepper motor. Open hand vs fist = 12 unique commands covering all arm joints.
