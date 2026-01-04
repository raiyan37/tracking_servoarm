# Hand-Tracked Servo Arm Control

<video src="./assets/IMG_6249.mp4" autoplay loop muted playsinline>
  <img src="./assets/IMG_6249.gif" alt="Hand tracking driving a physical servo arm" />
</video>


This project connects real-time hand tracking on a computer to a physical servo-driven arm using a wired Arduino connection.

A Python process captures video from a webcam, detects a single hand, derives finger bend angles, and streams those values over serial. An Arduino sketch receives the data and drives five servos in near real time.

The Python side is implemented in [`hand_control.py`](./hand_control.py).  
The Arduino firmware lives in [`servoarm_wired.ino`](./servoarm_wired.ino).

---

## Overview

- **Input:** Live webcam feed  
- **Processing:** Hand landmark detection and angle mapping  
- **Transport:** USB serial (comma-separated integers)  
- **Output:** Five servo motors driven directly by an Arduino  

The GIF above shows the full loop in action: live tracking, angle extraction, and physical actuation.

---

## Notable Techniques

- **Real-time hand landmark tracking** using MediaPipe Hands, which provides stable 21-point hand skeletons suitable for continuous control  
  https://developers.google.com/mediapipe/solutions/vision/hand_landmarker

- **Geometric angle calculation from 2D landmarks** to infer finger flexion instead of relying on model-specific gesture labels

- **Frame-synchronous processing loop** with OpenCV’s camera capture and display APIs  
  https://docs.opencv.org/

- **Serial command streaming with explicit framing** (newline-terminated, comma-separated values) to keep parsing predictable on constrained hardware

- **Heap-safe serial parsing on Arduino** using fixed-size `char` buffers and `strtok` instead of dynamic `String` objects

- **Servo write clamping** to ensure invalid or noisy input never drives motors beyond physical limits

---

## Technologies and Libraries Used

These are easy to miss if you only skim the code, but they matter for maintainability and performance:

- **MediaPipe (Python)** – High-performance, cross-platform hand tracking  
  https://developers.google.com/mediapipe

- **OpenCV (cv2)** – Camera access, image conversion, and on-screen diagnostics  
  https://opencv.org/

- **NumPy** – Vector math for landmark and angle calculations  
  https://numpy.org/

- **pySerial** – Low-level serial communication between Python and Arduino  
  https://pyserial.readthedocs.io/

- **Arduino Servo Library** – Hardware-timed PWM control for standard hobby servos  
  https://www.arduino.cc/reference/en/libraries/servo/

---

## Data Flow

1. Webcam frames are captured and converted to RGB.  
2. MediaPipe detects and tracks a single hand.  
3. Finger joint angles are computed from landmark positions.  
4. Angles are serialized as comma-separated integers.  
5. Arduino parses the line and updates five servos in one pass.  

The visual feedback loop shown in the GIF is intentional: it makes debugging latency, jitter, and mapping errors straightforward.

---

## Project Structure

```text
.
├── assets/
├── hand_control.py
└── servoarm_wired.ino
