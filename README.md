# Leak Safety Node

This ROS 2 Python package monitors leak sensors and triggers emergency shutdown procedures by:
- Disarming the manipulator and vehicle via ROS2 service
- Shutting down the Raspberry Pi

---

## ✅ Prerequisites

Make sure the following are installed on your Raspberry Pi:
  
```bash
sudo apt install python3-rpi.gpio
``` 

## Run sos_leak publisher / subscriber option

On the top tube (main) run:

```bash
ros2 launch sos_leak sos_leak.launch.py vehicle_name:=klopsi00 tube_name:=main
``` 

On the bottom tube (buddy) run:

```bash
ros2 launch sos_leak sos_leak.launch.py vehicle_name:=klopsi00 tube_name:=buddy
``` 


## Run sos_leak server option

On the top tube (main) run:

```bash
ros2 launch sos_leak sos_leak_service.launch.py vehicle_name:=klopsi00 tube_name:=main
``` 

On the bottom tube (buddy) run:

```bash
ros2 launch sos_leak sos_leak_service.launch.py vehicle_name:=klopsi00 tube_name:=buddy
``` 