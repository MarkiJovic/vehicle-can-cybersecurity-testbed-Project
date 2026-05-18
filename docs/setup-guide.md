# Setup Guide

This guide explains the main setup steps for the Vehicle CAN Cybersecurity Testbed.

> Update this file with the exact commands and paths from your final implementation before publishing.

---

## 1. Environment Requirements

### Windows

- Windows 11
- CARLA 0.9.13
- Python 3.7 for CARLA API compatibility
- PCAN-USB driver / PCAN-View if using physical CAN hardware

### Ubuntu

- Ubuntu 20.04
- ROS Noetic
- SocketCAN tools
- CARLA ROS bridge
- rosbridge server

---

## 2. Start CARLA

Example Windows command:

```powershell
CarlaUE4.exe /Game/Carla/Maps/Town01 -windowed -ResX=800 -ResY=600 -carla-server
```

If port 2000 is unavailable, use a different RPC port:

```powershell
CarlaUE4.exe /Game/Carla/Maps/Town01 -windowed -ResX=800 -ResY=600 -carla-server -carla-rpc-port=3000 -carla-streaming-port=3002
```

---

## 3. Start ROS

```bash
roscore
```

Launch the CARLA ROS bridge:

```bash
roslaunch carla_ros_bridge carla_ros_bridge.launch host:=<WINDOWS_IP> port:=2000 timeout:=10000
```

---

## 4. Start Project Nodes

```bash
roslaunch car_nodes complete_car_simulation.launch debug:=true
```

Launch rosbridge for the dashboard:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

---

## 5. Start Dashboard

```bash
python dashboard.py
```

---

## 6. CAN Hardware Checks

Useful checks:

```bash
candump can0
```

```bash
cansend can0 100#FF00000000000000
```

Make sure:

- both CAN devices use the same bitrate
- CAN_H and CAN_L are wired correctly
- ground is connected if required
- the bus has correct termination

---

## 7. Notes

Before publishing the repository, remove private IP addresses, usernames, tokens, and any machine-specific paths that are not needed.
