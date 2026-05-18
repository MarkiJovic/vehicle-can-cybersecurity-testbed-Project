# System Architecture

This project connects a simulated vehicle environment to a ROS-based monitoring and CAN security testbed.

---

## Main Components

| Component | Description |
|---|---|
| CARLA | Vehicle simulator used to visualise behaviour |
| ROS Noetic | Middleware connecting nodes and topics |
| CARLA ROS Bridge | Connects CARLA to ROS |
| CAN Bridge | Sends and receives CAN-style messages |
| PCAN-USB | Physical CAN interface |
| Dashboard | Displays vehicle state, CAN traffic, and attacks |
| IDS Node | Detects suspicious CAN behaviour |
| Attack Node | Generates controlled attack scenarios |

---

## Architecture Diagram

Add a proper image later:

```markdown
![System Architecture](../images/system-architecture.png)
```

Temporary text diagram:

```text
CARLA Simulator <-> ROS Noetic <-> CAN Bridge <-> PCAN-USB / CAN Traffic
                         |
                         v
                    Dashboard / IDS
```
