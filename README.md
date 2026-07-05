# Exocentric Navigation System

A real-time **distributed robotic navigation system** designed for resource-constrained embedded platforms, where perception, state estimation, and planning are decoupled from onboard computation and executed on an external host to achieve low-latency closed-loop control.

---

# Problem Statement

Autonomous navigation on low-cost robotic platforms is fundamentally constrained by limited onboard compute, unreliable localization, and the inability to execute modern perception models in real time.

This system addresses these constraints through a **distributed exocentric architecture**, where an external vision system provides global observability of the environment, eliminating reliance on onboard SLAM, IMU-based drift correction, or heavy onboard inference pipelines.

The core challenge addressed is real-time closed-loop navigation under externalized perception with strict communication latency constraints and noisy visual state estimation.

A centralized compute node performs:
- real-time visual localization
- deep learning-based obstacle inference (YOLOv8)
- state stabilization via Kalman filtering
- cost-aware path planning using weighted A*

while a lightweight embedded agent executes deterministic motion commands over a wireless control channel.

---

# Project Motivation

This work is motivated by a fundamental constraint in embedded robotics systems: intelligent algorithms are compute-heavy, but embedded platforms are compute-limited.

Instead of scaling onboard hardware, this system explores a **compute-offloaded navigation paradigm**, where the robot is reduced to a minimal actuation unit and all perception and decision-making is shifted to an external processing node.

This enables:
- deployment of deep learning models without onboard GPU constraints  
- stable global localization without SLAM complexity  
- deterministic control over low-cost embedded hardware  

The system demonstrates how robotics can be restructured as a **distributed real-time control problem rather than a monolithic onboard computation system**.

---

# Abstract

This project presents a **real-time distributed robotic navigation system** integrating computer vision, probabilistic state estimation, and graph-based path planning under a constrained wireless control loop.

An overhead camera provides a global environmental view, enabling exocentric robot localization through motion tracking and filtering techniques. A Kalman filter is applied to stabilize state estimation under visual noise and temporary tracking degradation.

Obstacle perception is performed using a YOLOv8-based inference pipeline running on a host system, converting semantic detections into a structured grid-based cost map.

Navigation is performed using a weighted A* planner operating over this cost map, incorporating obstacle inflation and soft constraints to ensure safety-aware trajectory generation.

A pulse-based control strategy converts continuous motion requirements into discrete actuation commands, mitigating overshoot introduced by network latency and non-deterministic wireless transmission.

Wireless communication is implemented using an ESP8266 module over UDP, which forwards commands to an Arduino microcontroller responsible for motor actuation.

The system demonstrates real-time performance in closed-loop perception–planning–control execution under externalized computation constraints.

---

# System Overview

The system is designed as a **distributed real-time control pipeline**, where computational intelligence is moved outside the embedded boundary.

### Core Pipeline

Camera → External Vision Compute → State Estimation → Cost Map Generation → Path Planning → Control Signal Generation → Wireless Actuation → Robot Motion

This separation enables:
- deterministic embedded execution
- compute-heavy perception on host system
- real-time closed-loop feedback under network latency constraints

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
- real-time performance
- system modularity
- fault isolation
- scalability to multi-robot extensions

Key design principles:
- prioritize real-time responsiveness over communication reliability (UDP choice)
- reduce onboard complexity to improve embedded robustness
- stabilize noisy perception using probabilistic filtering (Kalman filter)
- convert continuous control into discrete actuation to mitigate network jitter
- use grid abstraction to unify perception and planning layers

---

# Key Engineering Decisions

- Deep learning inference is executed on a host machine rather than onboard the robot, allowing computationally intensive perception without embedded constraints.

- UDP communication is used to minimize control latency, prioritizing real-time responsiveness over guaranteed delivery.

- A Kalman filter is used to improve robustness against noisy visual measurements and transient tracking errors.

- Pulse-based motion control is adopted to reduce overshoot caused by discrete wireless command updates.

- Weighted A* planning is chosen as a deterministic, computationally efficient path planning algorithm with safety-aware cost penalties.

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
- **Sensor:** Overhead monocular camera

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

# Installation

```bash
git clone https://github.com/<your-username>/exocentric-navigation-systems.git
cd exocentric-navigation-systems
python -m pip install -r requirements.txt
```

---

# Execution

1. Flash ESP8266 and Arduino firmware from `/robot`
2. Connect system to Wi-Fi network
3. Update ESP8266 IP in `config/settings.py`
4. Run:

```bash
python main.py
```

5. Calibrate using four-point homography selection
6. Press **SPACE** to initialize robot orientation calibration
7. Click target location for autonomous navigation

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
Transforms continuous motion into discrete stable movement commands.

---

# Engineering Highlights

- Modular multi-layer architecture (vision → planning → control)
- Distributed computation between PC and embedded systems
- Real-time perception-to-action loop
- Robust tracking with sensor fusion principles
- Hybrid classical + deep learning robotics pipeline
- Networked robot control using UDP protocol

---

# Limitations

Like most vision-based autonomous navigation systems, the current implementation makes several design assumptions:

- **Overhead Camera Requirement:** The system relies on a fixed overhead camera to obtain a global view of the environment. Navigation accuracy depends on maintaining sufficient camera headroom and an unobstructed field of view.

- **Homography-Based Workspace:** Robot localization is performed using a manually calibrated homography transformation. Significant camera movement or changes in the workspace require recalibration.

- **Known Obstacle Classes:** Obstacle avoidance is limited to object categories that are recognized by the trained YOLOv8 detection model. Previously unseen obstacle classes may not be incorporated into the navigation map.

- **Single-Robot Operation:** The current architecture is designed for a single mobile robot and does not include multi-robot coordination or cooperative path planning.

- **Static Planning Environment:** Path planning assumes that obstacle locations remain relatively stable during navigation. Highly dynamic environments may require continuous replanning and more advanced prediction techniques.
 
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
