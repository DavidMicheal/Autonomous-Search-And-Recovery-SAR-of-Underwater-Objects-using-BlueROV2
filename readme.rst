======================================================
Autonomous Underwater Search & Recovery Using BlueROV2
======================================================

.. contents::
   :local:
   :depth: 2

Overview
========
This repository contains the code and documentation for our **Autonomous Underwater Search & Recovery** project, conducted using a **BlueROV2** platform. The main objective is to detect and retrieve a sunken black box using computer vision (YOLO-based object detection), a custom robotic gripper, and robust control (PID controllers for yaw and depth).

Key Features
------------
- **Object Detection**: YOLOv5-based model to detect both the black box and its handle.
- **Custom Gripper**: 3D-printed end-effector that attaches to or replaces the Newton Sub-Sea Gripper.
- **PID Control**: Independent depth-hold and yaw-hold controllers for stable positioning.
- **ROS2 Integration**: Nodes for detection, control, gripper actuation, etc.
- **Semi-Autonomous Operation**: Operator provides short forward pulses, while the system handles depth/yaw stabilization.

Hardware Requirements
=====================
1. **BlueROV2** from Blue Robotics:
   - 8 Thrusters
   - Bar30 Pressure/Depth Sensor
   - IMU (accelerometer / gyroscope / compass)
   - Raspberry Pi or companion computer
2. **Gripper** (Newton Sub-Sea or custom design)
3. **Tether** for communication
4. **Laptop** (GCS) with Ethernet port

Software Requirements
=====================
- **Ubuntu 22.x** or similar Linux distribution
- **ROS2** (Humble or Foxy recommended)
- **Python 3**
- **QGroundControl** (for initial calibration/checks)
- **BlueOS** (for thruster calibration, firmware)
- **MAVLink / mavros** for GCS-ROV communication
- **YOLOv5** (object detection)
- **PyQt5** or **Unity** with **ROS-TCP Connector** (for GUIs)


Results
=======
- **PID Stability**: Depth and yaw controllers performed reliably.
- **Robust Detection**: YOLOv5 detects the black box and handle under varied lighting.
- **Successful Retrieval**: Multiple test runs showed consistent grasping and retrieval.

Refer to the :download:`Final Report <docs/Final_Report.pdf>` for detailed analyses, charts, and technical breakdowns.

Future Work
===========
1. **Improved Gripper**: A more enclosed design could further simplify handle alignment.
2. **Lateral Controller**: An automatic lateral motion controller would reduce manual repositioning.
3. **Sensor Expansion**: Consider SONAR or other advanced sensing for turbid water.

Authors & Contact
=================
- David Eskoundos: https://github.com/DavidMicheal
- Shubh Singhal

Institution: CIRTESU / Universitat Jaume I (UJI), Castellón, Spain

For inquiries or collaboration proposals, please open an issue in this repository or refer to the contact details in the `Final Report (Bluerov_Cirtesu_Report.pdf)`.
