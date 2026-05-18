# Troubleshooting

Common issues and checks used during the project.

---

## CARLA Bridge Timeout

If ROS cannot connect to CARLA:

- check CARLA is running
- check the correct Windows IP address is used
- check port 2000 is not blocked or already in use
- check firewall rules
- increase bridge timeout

Example:

```bash
roslaunch carla_ros_bridge carla_ros_bridge.launch host:=<WINDOWS_IP> port:=2000 timeout:=10000
```

---

## Port 2000 Conflict on Windows

PowerShell check:

```powershell
netstat -ano | findstr :2000
```

If another process is using the port, either stop the process or launch CARLA on another port.

---

## rosbridge Not Running

Check port 9090 on Ubuntu:

```bash
ss -ltnp | grep 9090
```

Launch rosbridge:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

---

## CAN Traffic Not Visible

Check:

- same bitrate on both adapters
- correct CAN_H/CAN_L wiring
- proper termination
- interface is up
- PCAN-View / candump is listening on the correct device

Example:

```bash
candump can0
```
