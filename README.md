# Educational Robotic Arm (3-DOF Arduino-Based)

This project presents a low-cost, 3-DOF robotic arm designed for educational purposes in schools, colleges, and universities. Built with Arduino Uno R3, SG90 and MG996R servo motors, and 3D-printed components, the arm enables students to explore core robotics concepts such as kinematics, motion control, and embedded programming.

## Features
- **Manual Control** via Serial Monitor (WASD-style commands)
- **Inverse & Forward Kinematics** for 3D movement
- **Linear Interpolation** for smooth servo motion
- **Detachable End Effector** for gripping and handling objects
- **Fully 3D Printed** with open-source designs
- **Budget-friendly** (~£77 total cost)

## Technologies & Components
- Arduino Uno R3
- Servo Motors (MG996R x3, SG90 x3)
- Custom PCB (optional, breadboard-friendly)
- 3D Printed Parts
- Arduino IDE

## Getting Started
1. Upload `RoboticArm.ino` to your Arduino.
2. Connect servos to the correct PWM pins (2–7).
3. Open Serial Monitor (baud rate: 115200).
4. Use keys like `w`, `a`, `s`, `d` to control the arm manually.
5. Use `moveTo(x, y, z)` in code for coordinate-based movement.

## Educational Scope
- **School:** Hands-on intro to robotics
- **College:** Programming & motion logic
- **University:** Kinematic modeling & control theory

## Demo
[Watch on YouTube](https://youtu.be/6I-tcn5AtSY)

## License
This project uses open-source hardware and software principles for non-commercial, educational use.
