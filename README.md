# Exocentric Navigation System

A real-time **distributed robotic navigation system** designed for resource-constrained embedded platforms, where perception, state estimation, and planning are decoupled from onboard computation and executed on an external host to achieve low-latency closed-loop control.

---
# Problem Statement & Motivation

Autonomous navigation on low-cost robotic platforms is fundamentally constrained by limited onboard compute. Intelligent algorithms are compute-heavy, but embedded microcontrollers are compute-limited. 

Instead of scaling onboard hardware, this system explores a **compute-offloaded navigation paradigm**. By utilizing an exocentric (overhead) camera, the system achieves global observability of the environment, eliminating the need for complex onboard SLAM, IMU-based drift correction, or edge GPUs. 

The system demonstrates how robotics can be restructured as a **distributed real-time control problem rather than a monolithic onboard computation system**.

---

# System Overview 

This project presents a **real-time distributed control pipeline** that integrates computer vision, probabilistic state estimation, and graph-based path planning under a constrained wireless control loop.

A centralized host machine and an overhead camera (e.g., C270 HD Webcam @ 720p/30fps) provide a global environmental view. This enables exocentric robot localization through motion tracking, stabilized by a Kalman filter to mitigate visual noise. Obstacle perception is performed using a host-side YOLOv8 inference pipeline (running on Python 3.8+), which converts semantic detections into a structured grid-based cost map.

Navigation is executed using a weighted A* planner operating over this cost map. A pulse-based control strategy then converts continuous motion requirements into discrete actuation commands, mitigating overshoot introduced by network latency. These commands are dispatched via an ESP8266 Wi-Fi UDP bridge to a lightweight Arduino agent, which acts strictly as a deterministic motor actuator.

### Core Pipeline

Camera → External Vision Compute → State Estimation → Cost Map Generation → Path Planning → Control Signal Generation → Wireless Actuation → Robot Motion

This project was developed as a group effort; however, the end-to-end perception–planning–control architecture, system integration, and core implementation were primarily designed and implemented by the author.

---

# System Boundaries & Runtime Characteristics

The architecture enforces strict operational boundaries between non-deterministic intelligence and deterministic actuation.

* **Host System (Perception & Planning):** Operates in soft real-time. The perception loop runs at approximately **15–30 Hz** (bottlenecked by native **720p** camera capture rate and YOLOv8 host inference). Pathfinding executes virtually instantaneously over a **20x20 quantized spatial grid mapped from the native 1280 X 720 frame canvas**, utilizing an **80-pixel dynamic lookahead radius** to smooth trajectory execution and prevent control oscillation near waypoints.
* **Network Layer (Communication):** Operates on a best-effort basis. UDP prioritizes the most recent command over reliable delivery. Navigation commands are dispatched at a strictly bounded **400ms control interval**, dynamically compensating for heading errors using a **±30° tolerance deadband** without overloading the UDP network buffer.
* **Embedded Layer (Actuation):** Operates as a hard real-time execution agent. The Arduino maintains no state awareness; it executes calibrated, hardware-tuned actuation pulses (**200ms for translation, 160ms for rotation**) per received command before auto-halting to prevent network-induced overshoot.

---
 
# Key Features

- Exocentric distributed navigation architecture
- Real-time robot detection and tracking
- Kalman filter-based state estimation
- YOLOv8-based obstacle detection
- Grid-based environment representation
- Weighted A* path planning with cost penalties
- Obstacle inflation and path memory for smoother trajectories
- Pulse-based motion control for stability
- UDP-based wireless communication pipeline
- ESP8266 → Arduino motor control bridge

---

# System Architecture

```
                +----------------------+
                |  Overhead Camera     |
                +----------+-----------+
                           |
                           v
              Vision Processing Pipeline
        (Detection + Tracking + Kalman Filter)
                           |
                           v
                Environment Representation
                 (Grid + Obstacles + Costs)
                           |
                           v
                   Path Planning Engine
                     (Weighted A*)
                           |
                           v
                    Motion Controller
            (Pulse-based command generation)
                           |
                           v
                  UDP Communication Layer
                           |
                     ESP8266 Bridge
                           |
                       UART Serial
                           |
                           v
                   Stacked Arduino UNO + L293D Shield
                           |
                           v
                      Differential Robot

```

### System Architecture (Visual)

The following diagram presents the complete closed-loop exocentric navigation pipeline implemented in this system, highlighting the stacked embedded control configuration:

![System Architecture](docs/architecture.png)

---

# Design Philosophy

The system is intentionally designed around a **compute–control separation principle**, where all non-deterministic and compute-heavy tasks are externalized, while the embedded system is restricted to deterministic execution.

This separation improves:
- Real-time performance
- System modularity
- Fault isolation
- Scalability to multi-robot extensions

Key design principles:
- Prioritize real-time responsiveness over communication reliability (UDP choice)
- Reduce onboard complexity to improve embedded robustness
- Stabilize noisy perception using probabilistic filtering (Kalman filter)
- Convert continuous control into discrete actuation to mitigate network jitter
- Use grid abstraction to unify perception and planning layers

---

# Key Engineering Decisions

- Deep learning inference is executed on a host machine rather than onboard the robot, allowing computationally intensive perception without embedded constraints.

- UDP communication is used to minimize control latency, prioritizing real-time responsiveness over guaranteed delivery.

- A Kalman filter is used to improve robustness against noisy visual measurements and transient tracking errors.

- Pulse-based motion control is adopted to reduce overshoot caused by discrete wireless command updates.

- Weighted A* planning is chosen as a deterministic, computationally efficient path planning algorithm with safety-aware cost penalties.

### Architectural Tradeoff Analysis

| Component | Selected Approach | Rejected Alternative | Primary Systems Tradeoff Justification |
| :--- | :--- | :--- | :--- |
| **Communication Layer** | **UDP** | TCP / WebSockets | **Latency over Reliability.** TCP's packet retry mechanism causes head-of-line blocking. Dropping a stale packet is safer than executing delayed movement commands. |
| **Perception Pipeline** | **Exocentric Vision (Host)** | Onboard SLAM (e.g., Jetson Nano) | **Compute over Autonomy.** Offloading to a host allows for heavy YOLOv8 inference without the SWaP-C (Size, Weight, Power, Cost) penalty of onboard GPUs. |
| **Control Strategy** | **Calibrated Actuation Pulses** | Continuous PID Control | **Jitter Tolerance over Smoothness.** Continuous control over wireless introduces network-induced overshoot. Bounded hardware pulses auto-halt the robot if the network drops. |

---

# Engineering Highlights

- Distributed real-time robotics architecture under embedded compute constraints
- Externalized perception pipeline replacing onboard SLAM dependency
- Probabilistic state estimation under noisy visual input
- Real-time cost-map based navigation with grid abstraction
- Latency-aware control design over unreliable wireless communication
- Hybrid classical + deep learning robotics pipeline
- Closed-loop execution under non-deterministic network delays

---

# Repository Structure

```
vision/        → Homography, detection pipeline
tracking/      → Kalman filter, object tracking
planning/      → A* path planning, cost maps
control/       → State machine, motion control
comm/          → UDP communication and protocol handling
utils/         → Geometry and visualization tools
config/        → System configuration files
robot/         → ESP8266 and Arduino firmware
models/        → Trained YOLO weights

assets/        → Demo media (images, videos, GIFs)
docs/          → Technical report and architecture diagrams

main.py        → Application entry point
requirements.txt → Python dependencies
.gitignore
LICENSE
README.md
```

---

# Hardware Architecture

- **Robot Platform:** Stacked Arduino UNO + L293D Shield
- **Communication:** ESP8266 (Wi-Fi UDP → Serial bridge)
- **Compute:** Host PC (Python-based vision + planning)
- **Sensor:** Logitech C270 HD Webcam (720p/30fps, parallel overhead mount)

**Hardware Iteration Note:**  
During early prototyping, a LILYGO T-A7672S board (ESP32 + SIMCom A7672S LTE module) was used for testing communication and system integration.  

The final implementation uses an ESP8266 module as a lightweight Wi-Fi UDP bridge between the host system and Arduino, chosen for its simplicity, sufficient performance, and lower system overhead.

**System Consistency Note:**  
The repository codebase corresponds to the final implementation using ESP8266 as the Wi-Fi communication bridge. Earlier prototyping used a different hardware setup, but all current scripts, firmware, and communication logic are aligned with the ESP8266-based system.

---

# Software Stack

- Python (core system)
- OpenCV (vision pipeline)
- NumPy (numerical computation)
- Ultralytics YOLOv8 (object detection)
- Arduino IDE (motor control firmware)
- ESP8266 Wi-Fi firmware

**Embedded Firmware:**
- C++/Arduino Core
- SoftwareSerial Library (Inter-board UART bridge)
- Motor_Shield Library (Provides the DCMotor abstraction for controlling the L293D motor driver)

---

# Installation & Setup

### 1. Embedded Hardware Setup
1. Open the firmware files in the `/robot` directory using the Arduino IDE.
2. Open the ESP8266 firmware code and locate the Wi-Fi configuration variables. 
3. **Change the Wi-Fi name (SSID) and password** to match your local Wi-Fi network (ensure this is the exact same network your host laptop is connected to).
4. Flash the ESP8266 module, then open the Serial Monitor to verify successful network connection and confirm the active IP address.
5. Flash the Arduino UNO motor control firmware.

### 2. Host System Setup
Clone the repository and install the required Python dependencies:

```bash
git clone https://github.com/<your-username>/exocentric-navigation-system.git
cd exocentric-navigation-system
python -m pip install -r requirements.txt
```

---

# Execution

1. Verify that both your host machine (laptop) and the ESP8266 are connected to your configured local Wi-Fi network.
2. Update the ESP8266 IP address in `config/settings.py` with the one obtained from the Serial Monitor.
3. Launch the host pipeline:

```bash
python main.py
```

4. **Calibrate:** Click 4 points in the video feed to define the homography workspace.
5. **Initialize:** Press **SPACE** to initialize robot orientation calibration.
6. **Navigate:** Click a target location on the interface for autonomous pathfinding and execution.

---

# Core Algorithms

### 1. Homography Transformation
Maps perspective view into a top-down coordinate system.

### 2. Kalman Filtering
Reduces noise in visual tracking and stabilizes state estimation.

### 3. YOLOv8 Detection
Detects obstacles and converts them into spatial grid constraints.

### 4. Weighted A*
Computes optimal path considering obstacle density and safety margins.

### 5. Pulse-Based Control
Transforms continuous motion into discrete stable movement commands using hardware-calibrated duty limits and windowed delays.

---

# Limitations

Like most vision-based autonomous navigation systems, the current implementation makes several design assumptions:

- **Overhead Camera Requirement:** The system relies on a fixed overhead camera to obtain a global view of the environment. Navigation accuracy depends on maintaining sufficient camera headroom and an unobstructed field of view.

- **Homography-Based Workspace:** Robot localization is performed using a manually calibrated homography transformation. Significant camera movement or changes in the workspace require recalibration.

- **Known Obstacle Classes:** Obstacle avoidance is limited to object categories that are recognized by the trained YOLOv8 detection model. Previously unseen obstacle classes may not be incorporated into the navigation map.

- **Single-Robot Operation:** The current architecture is designed for a single mobile robot and does not include multi-robot coordination or cooperative path planning.

- **Static Planning Environment:** Path planning assumes that obstacle locations remain relatively stable during navigation. Highly dynamic environments may require continuous replanning and more advanced prediction techniques.
 
---

# Edge Behavior & System Degradation

The system exhibits specific deterministic behaviors under edge-case conditions and sensor failure:

* **Out-of-Bounds State Loss:** The operational workspace is strictly bounded by the calibrated homography matrix. If the robot travels outside this mapped coordinate space (e.g., into the unmapped camera periphery), visual tracking immediately fails. The system is designed to fail safely: it halts state estimation and broadcasts a continuous `X` (STOP) pulse over UDP to prevent runaway actuation.
* **Scale-Dependent Padding Variance:** Obstacle avoidance relies on static spatial padding added to YOLOv8 bounding boxes. Because the homography is calibrated using a flat physical reference, any significant change in the camera's height or the height of the physical obstacles alters the perspective scale. In edge cases, this perspective distortion results in insufficient obstacle inflation, occasionally allowing the robot to clip the edges of taller objects.
* **Dynamic Obstacle Occlusion:** Since the system relies entirely on exocentric overhead vision, if an obstacle physically occludes the robot from the camera's view, the Kalman filter will attempt to predict the robot's trajectory. However, prolonged occlusion will cause state drift, eventually requiring a system reset.

---

# Future Improvements

- ROS2 migration
- Multi-robot coordination
- Multi-camera fusion
- SLAM-based mapping
- GPU-accelerated inference
- Reinforcement learning-based navigation

---

# Results

The implemented system successfully demonstrates:

- Real-time robot localization using overhead vision
- Robust visual tracking with Kalman filter state estimation
- Real-time obstacle detection using a custom YOLOv8 model
- Weighted A* path planning with obstacle inflation and path smoothing
- Autonomous navigation to user-selected destinations
- Low-latency wireless control using an ESP8266 UDP communication bridge

### Autonomous Navigation

The figure below shows the complete navigation pipeline during runtime. The system simultaneously performs robot localization, obstacle detection, path planning, and closed-loop control while navigating toward the selected destination.

![Autonomous Navigation](assets/screenshots/autonomous_navigation.jpeg)

---

# Demonstration

### Autonomous Navigation Demo

![Robot Demo](assets/robot_demo.gif)

The animation demonstrates the complete perception-to-action pipeline, including robot tracking, obstacle detection, weighted A* path planning, and autonomous target reaching.

### Hardware Platform

![Robot Hardware](assets/robot.jpg)

### Technical Report

A detailed implementation report is available here:

📄 [Technical Report (PDF)](docs/Final_ENDTERM_REPORT.pdf)

### Full Demonstration Video

A complete demonstration, including multiple autonomous navigation runs, is available here:

▶️ [Full Demonstration Video (MP4)](assets/robot_demo.mp4)

---

# License

MIT License
