Autonomous Underwater Search & Recovery Using BlueROV2
This repository contains the code and documentation for our Autonomous Underwater Search & Recovery project, conducted using a BlueROV2 platform. The main objective is to detect and retrieve a sunken black box using computer vision (YOLO-based object detection), a custom robotic gripper, and robust control (PID controllers for yaw and depth).

Table of Contents
Overview
Hardware Requirements
Software Requirements
Project Structure
Setup Instructions
Usage
Simulation
GUI Implementations
Results
Future Work
License
Overview
In underwater environments, accurate navigation, robust object detection, and manipulation are essential to conduct safe and efficient search and rescue or recovery missions.

Key Features
Object Detection: Uses a YOLOv5-based model to detect both the black box and its handle in real time.
Custom Gripper: Designed and 3D-printed to attach onto the Newton Sub-Sea Gripper, allowing better handle grasping.
PID Control: Independent depth-hold and yaw-hold controllers ensure stable positioning.
Semi-Autonomous Operation: Once aligned, the operator provides short forward pulses for approach; the system handles depth/yaw stabilization.
ROS2 Integration: All components (detection, control, gripper actuation) communicate via ROS2 topics.
A detailed PDF Project Report can be found in the docs/ folder (or in this repository’s Releases section) for more in-depth technical details.

Hardware Requirements
BlueROV2 from Blue Robotics, equipped with:
8 Thrusters
Bar30 Pressure/Depth Sensor
IMU (accelerometer/gyroscope/compass)
Raspberry Pi or companion computer
Custom or Newton Sub-Sea Gripper for the BlueROV2
Tether for communication
Laptop (GCS) with at least 1 free Ethernet port
Software Requirements
Ubuntu 22.x (recommended) or similar Linux distribution
ROS2 (Humble or Foxy recommended)
Python 3
QGroundControl for initial calibration and manual control checks
BlueOS for thruster calibration, firmware updates, etc.
MAVLink / mavros for communication between the ROV and the GCS
YOLOv5 model for object detection
PyQt5 or Unity with ROS-TCP Connector (for GUI)
Project Structure
lua
Copy
.
├── README.md                    <-- This file
├── docs/
│   └── Final_Report.pdf         <-- Detailed project report (LaTeX compiled)
├── hardware/
│   ├── gripper_design/          <-- 3D design files for the custom gripper
│   └── ...
├── scripts/
│   ├── manual_controller.py     <-- RC input node for manual vs. auto mode
│   ├── pid_controllers.py       <-- PID controllers for depth & yaw
│   ├── yolo_detection_node.py   <-- YOLOv5 detection node
│   ├── robot_grasper.py         <-- Center publishing, low-pass filtering, handle detection
│   └── ...
├── gui/
│   ├── qt_gui.py                <-- PyQt5-based GUI
│   ├── unity_gui/               <-- Unity project for GUI with ROS-TCP Connector
│   └── ...
├── simulation/
│   ├── BLUE_sim/                <-- Blue simulator assets and configs
│   ├── ...
└── models/
    ├── yolo_weights.pt          <-- Trained YOLOv5 weights
    └── ...
Setup Instructions
Clone the Repository

bash
Copy
git clone https://github.com/YourUsername/Autonomous-Search-And-Recovery-SAR-of-Underwater-Objects-using-BlueROV2.git
cd Autonomous-Search-And-Recovery-SAR-of-Underwater-Objects-using-BlueROV2
Install ROS2 Dependencies
Make sure to install ROS2 packages, such as mavros, vision_opencv, etc.

bash
Copy
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras ...
Install Python Dependencies

bash
Copy
pip install -r requirements.txt
This installs YOLOv5, PyQt5, and other Python libraries.

Setup YOLOv5 Weights

Place your trained model weights (.pt file) under models/.
Modify paths in scripts/yolo_detection_node.py if needed.
Build and Source
If you are using a ROS2 workspace structure:

bash
Copy
colcon build
source install/setup.bash
Adjust the above commands if using a different build system.

Usage
Establish Connection to ROV

Ensure your BlueROV2 is powered and connected via Ethernet (tether).
Open BlueOS (typical IP) in a browser to confirm connection.
Launch QGroundControl to verify thruster and sensor calibrations if needed.
Launch Nodes

In a terminal, start the core ROS2 nodes for manual vs. autonomous control, PID, and detection:
bash
Copy
ros2 launch scripts system_launch.launch.py
(Adjust the launch file names as per your setup.)
Run the GUI

PyQt5 GUI:
bash
Copy
python3 gui/qt_gui.py
Unity GUI:
Open gui/unity_gui/ in Unity.
Press Play; ensure the ROS connection is set in the ROS-TCP Connector settings.
Switch to Autonomous Mode

In the GUI, toggle the Manual/Autonomous switch.
Check that depth-hold and yaw-hold PID controllers are engaged.
Watch the Detection

You should see bounding boxes on the camera feed for both the black box and the handle.
Align and approach the handle as described in the Final Report.
Grasp the Box

Use short forward pulses to move toward the box.
Close the gripper when the handle is within the jaws.
Simulation
A simulation environment is provided using the Blue simulator.

Install Blue following its official instructions.
Place the tank model and black box models from simulation/BLUE_sim/ into your Blue environment.
Run the simulation and launch your ROS2 nodes similarly.
The ROV model will respond to your controllers, and YOLO detection can be tested with the simulator camera feed.
GUI Implementations
Two GUIs are available:

PyQt5-based GUI (in gui/qt_gui.py):

Real-time camera feed
Sliders for depth, gain, and alpha values
Buttons for manual movement and gripper actuation
Yaw hold and autonomous mode toggles
Unity-based GUI (in gui/unity_gui/):

Real-time camera feed (via ROS-TCP Connector)
Similar controls as the PyQt5 GUI, but in a 3D environment
Choose the GUI that best fits your workflow or system requirements.

Results
Stable Depth and Yaw: The independent PID controllers achieved reliable stabilization, making object approach more consistent.
Robust Detection: YOLOv5-based detection successfully identifies both the black box and its handle, even under varying lighting.
Successful Retrieval: Multiple trials showed consistent retrieval of the target, demonstrating the viability of this semi-autonomous approach.
For a full analysis, see the Final Report.

Future Work
Improved Gripper Design

A fully enclosed bottom arm could simplify handle alignment and increase success rate.
Lateral Controller

Adding an automatic lateral (strafe) control loop would reduce the need for manual repositioning during approach.
Additional Sensors

Incorporating acoustic sensors (SONAR) or more advanced vision modalities for turbid or low-visibility environments.
License
This project is licensed under the MIT License (or whichever license you choose). See the LICENSE file for details.

Authors:

David Eskoundos
Shubh Singhal
Institution: CIRTESU / Universitat Jaume I (UJI), Castellón, Spain

For inquiries or collaborations, please contact us via GitHub Issues or through the contact details provided in the Final Report. Happy Diving!
