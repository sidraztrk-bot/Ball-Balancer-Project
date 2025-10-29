# 🎯 STM32 Ball Tracker System

A real-time **white ball tracking and balancing system** combining **Python (OpenCV)** for vision and **STM32** for control.  
The camera tracks the ball, sends coordinates to STM32, which uses **PID** to control two servos that tilt a platform to center the ball.

---

## ⚙️ System Overview

| Component | Function |
|------------|-----------|
| **Camera** | Captures video feed |
| **Python + OpenCV** | Detects ball, converts pixels → cm, sends to STM32 |
| **STM32F103C8T6** | Runs PID on X/Y axes |
| **2 Servos** | Tilt the plate to balance the ball |
| **UART (38400 baud)** | Communication between PC and STM32 |

---

## 🧰 Requirements

### Python Side
```bash
pip install numpy opencv-python pyserial
STM32 Side

STM32CubeIDE

Servo connections:

X servo → PA8 (TIM1_CH1)

Y servo → PA1 (TIM2_CH2)

UART2: PA9 (TX), PA10 (RX)

Common GND with servos

⚙️ Python Script

File: STM32BallTracker.py

Tracks a white ball in real-time

Sends coordinates as:

700,810\n  →  (7.00 cm, 8.10 cm)


ROI-based scaling

Adjustable parameters:

REAL_WIDTH_CM = 16
REAL_HEIGHT_CM = 16
STM32_PORT = 'COM9'
BAUDRATE = 38400
CAMERA_SOURCE = "http://172.20.10.4:8080/video"


Keys

Key	Action
Q	Quit
R	Reselect ROI
S	Toggle STM32 comm
⚙️ STM32 Firmware

File: main.c

Receives UART data ("x,y")

Parses values, applies PID control

Controls two servos (PWM 500–2500 µs)

Neutral angle = 90°

Timeout → returns platform to center

PID Constants Example

// X axis
Kp = 1.6; Ki = 0.003; Kd = 145;
// Y axis
Kp = 1.2; Ki = 0.003; Kd = 175;

🧩 How It Works

Python tracks white ball and sends (x, y) in cm

STM32 reads coordinates via UART

Two PID controllers adjust servo angles

Platform tilts to keep ball centered

🧠 Tips

Ensure good lighting for reliable white detection

Adjust HSV thresholds if needed:

lower_white = np.array([0, 0, 120])
upper_white = np.array([180, 80, 255])


Recalibrate ROI if camera or setup changes

🧾 License

Open for educational and research use.
Developed by Sıdra Öztürk 🧠
Python + STM32 Embedded Vision Project


---

Would you like me to make a **simple diagram (text-based or image)** showing the system flow — e.g. “Camera → Python → UART → STM32 → Servos”? It would look great at the top of the README.
