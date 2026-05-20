# Vehicle CAN Cybersecurity Testbed

A CARLA–ROS virtual testbed for demonstrating CAN-based cyber attacks against an autonomous vehicle, including spoofing, replay, denial-of-service (DoS), a LiDAR false-obstacle sensor attack, and rule-based IDS monitoring.

## Overview

This project was developed as a final year cybersecurity project to explore how insecure in-vehicle communication can affect autonomous driving behaviour.

The system combines:

- **CARLA** for autonomous vehicle simulation
- **ROS Noetic** for middleware and node communication
- **CAN-style control frames** for throttle, steering, and brake
- **A custom dashboard** for attack triggering and monitoring
- **A rule-based IDS** for live attack detection
- **A LiDAR-based sensor attack** to demonstrate perception-layer interference

The project focuses on how malicious CAN traffic and manipulated sensor data can influence autonomous vehicle control in a safe virtual environment.

---

## Key Features

- Autonomous driving in CARLA using `BehaviorAgent`
- CAN control mapping for:
  - `0x100` Throttle
  - `0x101` Steering
  - `0x102` Brake
- Direct spoofing attacks:
  - Throttle
  - Steering
  - Brake
- Replay attack with **record now / replay later**
- Denial-of-Service (DoS) attack
- LiDAR false-obstacle sensor attack
- Real-time dashboard for:
  - CAN traffic
  - attack controls
  - IDS alerts
- Rule-based IDS for suspicious control traffic and flooding
- Logging support for attack and CAN activity

---

## System Architecture

The project is split across two environments.

### Windows host
Runs:

- CARLA simulator
- autonomous driving script
- traffic generation script
- dashboard interface
- LiDAR safety / sensor attack script

### Ubuntu / ROS environment
Runs:

- ROS core
- rosbridge
- CAN bridge
- attack injection node
- IDS node
- logger
- supporting ROS package and launch files

### Data flow

```text
CARLA / BehaviorAgent -> Control -> CAN-style frames -> Vehicle
                ^                           ^
                |                           |
         Sensor Safety Layer          Attack Node
                ^                           |
              LiDAR                      IDS / Logger
```

Under normal conditions, the CARLA autonomous controller drives the vehicle and mirrors its control values into CAN-style frames for monitoring and replay capture.

During attacks, malicious CAN frames can interfere with throttle, steering, or braking behaviour. Because the autonomous controller remains active, the final behaviour is a hybrid control conflict rather than a complete controller takeover. This means the vehicle may visibly resist some attacks while still being affected by them.

### Attack Scenarios
### 1. Throttle attack

Injects malicious throttle values to cause unintended acceleration or conflict with normal autonomous control.

### 2. Steering attack

Injects malicious steering values to disturb lane following and route tracking.

### 3. Brake attack

Injects malicious brake values to force unintended deceleration.

### 4. Replay attack

Records legitimate CAN control frames and replays them later in a different driving context.

### 5. DoS attack

Floods critical control IDs at a high rate to disrupt normal control communication.

### 6. Sensor attack

Injects a false obstacle into LiDAR-derived perception data so the safety logic brakes even when the road ahead is clear.

### IDS

The project includes a lightweight rule-based Intrusion Detection System that monitors control traffic and raises alerts for:

suspicious transmitted control frames
extreme throttle values
extreme brake values
abnormal steering deviation
high-rate flooding consistent with DoS behaviour

IDS alerts are displayed live in the dashboard.

Repository Structure
vehicle-can-cybersecurity-testbed-Project/
├── dashboard/          # Dashboard code and related files
├── docs/               # Project documentation
├── launch/             # ROS launch files
├── ros_nodes/          # ROS nodes (attack node, IDS, bridge, logger, etc.)
├── README.md
└── requirements.txt
Documentation

The docs/ folder contains supporting material such as:

architecture notes
setup guidance
attack explanations
troubleshooting information

Suggested docs to include:

architecture.md
setup-guide.md
attack-scenarios.md
troubleshooting.md
Example Demo Flow

A typical demonstration follows this order:

Start CARLA
Start ROS and rosbridge
Launch the ROS package / required nodes
Start traffic generation
Start autonomous driving
Open the dashboard
Show normal driving baseline
Trigger CAN attacks
Show IDS alerts
Trigger the LiDAR false-obstacle attack

This creates a clear comparison between:

normal operation
direct control-plane attacks
indirect perception-layer attack effects
Technologies Used
CARLA 0.9.13
ROS Noetic
Python 3.7
SocketCAN / python-can
pygame
roslibpy
PCAN-USB (used during development/testing)
HTML/CSS/JavaScript (for project showcase material)
Project Outcome

This project demonstrates that insecure CAN-style control paths can be manipulated to affect autonomous vehicle behaviour, and that sensor/perception-level interference can also indirectly influence vehicle decisions.

The testbed provides a safe and reproducible platform for studying:

CAN-based attack effects
control-loop interference
replay and flooding behaviour
simple IDS-based monitoring
interaction between autonomous driving and malicious input
Future Improvements

Possible future extensions include:

stronger sensor/perception attack models
authenticated CAN or gateway protections
more advanced IDS techniques
cleaner full-system launch automation
richer logging and replay analysis
expanded visualisation and metrics
SETU Carlow
