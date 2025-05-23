# WICDPR: Winch-Integrated Cable-Driven Parallel Robots for Agriculture

## Project Overview

This repository contains the full open-source hardware, software, and documentation for both 2D and 3D Winch-Integrated Cable-Driven Parallel Robots (WICDPRs).  
These robots are designed for rapid deployment and high-precision motion in agricultural environments (e.g., greenhouses, field plots), minimizing soil compaction and maximizing flexibility by keeping all winches on the mobile platform.
## System Features

- **2D and 3D platforms:** Planar and spatial robots for different workspace needs.
- **Real-time feedback:** Rotary encoders and MPU6050 IMU for position and tilt.
- **Double-loop PID control:** Ziegler–Nichols tuned, with auto-leveling.
- **Teleoperation:** Manual control using ROS teleop nodes.
- **Safety:** E-stop support, tension checks, WiFi router for secure SSH.
- **Comprehensive docs:** Bills of Materials (BOM), STL files, wiring diagrams, and demo videos.


# Getting Started: Step-by-Step Setup for WICDPR

## 1. Hardware Assembly

### 2D System

1. **Print all required STL parts** (see [`docs/stl/`](docs/stl/)).
2. **Assemble the frame** with PETG parts, plywood sheets, M3 and M4 bolts, and ball bearings.
3. **Install NEMA-17 stepper motors and planetary gearboxes** as per mechanical drawings.
4. **Mount both rotary encoders** to the winch spools; ensure pull-up resistor adapters are installed.
5. **Attach the IMU (MPU6050)** securely on the platform for accurate roll/pitch sensing.
6. **String the braided fishing line** through all pulleys and attach to the anchors.
7. **Double-check all power and signal connections** using the wiring diagram.
8. **[Optional]** Place test load (marbles) on platform for load/stability testing.

> ![2D Platform Render](docs/stl/2d_platform_render.png)
> *Figure: 2D Platform CAD Render*

---

### 3D System

1. **Print and assemble all 3D parts** for platform, winch modules, and mounting brackets.
2. **Mount all four DC geared motors** and rotary encoders (DFRobot SEN0230) to winch modules.
3. **Install the IMU (MPU6050)** on the platform.
4. **Set up the winch pods** at each platform corner with swivel/lazy-susan bearings.
5. **String 20m of braided fishing line** for each cable, tension and secure at anchor points.
6. **Connect all electronics:**  
   - Arduino Mega (motor board), Arduino Uno (sensor board)
   - Dual 7A DC motor drivers (DFRobot DRI0041)
   - Pull-up adapters on each encoder
7. **Wire up power supplies, Pi 4, and WiFi router.**

> ![3D Platform Assembly](diagrams/3d_platform_assembly.png)
> *Figure: 3D Platform with Electronics*

---

## 2. Electronics and Firmware

1. **Flash the sensor node code** (`arduino/arduino_encoder_revised.ino`) to the Arduino Uno.
2. **Flash the motor node code** (e.g., `arduino_motor_3d.ino` for 3D, or the correct 2D sketch) to the Arduino Mega (3D) or second Uno (2D).
3. **Insert formatted 64GB microSD cards** into the Pi(s).
4. **Power on all electronics** and connect Arduinos and Pi via USB cables.

---

## 3. Raspberry Pi and ROS Setup

1. **Connect the Pi to your WiFi router** (built-in or TP-Link USB adapter).
2. **SSH into the Pi** from your PC using its IP address:
   ```bash
   ssh pi@<your_pi_ip_address>
   

```
source /opt/ros/noetic/setup.bash
```
pip3 install -r requirements.txt
```
roscore
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 _baud:=57600
```
python3 scripts/cable_platform_2d_controller.py
```
python3 scripts/cable_platform_3d_controller.py
```
python3 scripts/teleop_controller_2D.py
```
python3 scripts/teleop_controller_3D.py

## Bill of Materials (BOM)

- [2D_WICDPR_BOM.xlsx](docs/2D_WICDPR_BOM.xlsx)
- [3D_WICDPR_BOM.xlsx](docs/3D_WICDPR_BOM.xlsx)

*All components, suppliers, and links are included. Please check the BOM before assembly.*
