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

// Project Structure
// =================
// .. code-block:: text

//     .
//     ├── README.rst              <-- This file
//     ├── docs/
//     │   └── Final_Report.pdf     <-- Detailed project report (LaTeX-compiled)
//     ├── hardware/
//     │   ├── gripper_design/      <-- 3D design files
//     │   └── ...
//     ├── scripts/
//     │   ├── manual_controller.py <-- RC input node
//     │   ├── pid_controllers.py   <-- PID controllers (depth, yaw)
//     │   ├── yolo_detection_node.py <-- YOLOv5 detection
//     │   ├── robot_grasper.py     <-- Center publishing, low-pass filtering
//     │   └── ...
//     ├── gui/
//     │   ├── qt_gui.py            <-- PyQt5-based GUI
//     │   ├── unity_gui/           <-- Unity GUI project (ROS-TCP)
//     │   └── ...
//     ├── simulation/
//     │   ├── BLUE_sim/            <-- Files for Blue simulator
//     │   └── ...
//     └── models/
//         ├── yolo_weights.pt      <-- YOLOv5 trained weights
//         └── ...

// Setup Instructions
// =================
// 1. **Clone the Repository**:

//    .. code-block:: bash

//        git clone https://github.com/YourUsername/Autonomous-Search-And-Recovery-SAR-of-Underwater-Objects-using-BlueROV2.git
//        cd Autonomous-Search-And-Recovery-SAR-of-Underwater-Objects-using-BlueROV2

// 2. **Install ROS2 Dependencies**:

//    .. code-block:: bash

//        sudo apt-get update
//        sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras ...

// 3. **Install Python Dependencies**:

//    .. code-block:: bash

//        pip install -r requirements.txt

// 4. **Configure YOLOv5 Weights**:
//    - Place your ``.pt`` file in the ``models/`` folder.
//    - Adjust paths in ``scripts/yolo_detection_node.py`` if required.

// 5. **Build and Source** (Typical ROS2 workflow):

//    .. code-block:: bash

//        colcon build
//        source install/setup.bash

// Usage
// =====
// 1. **Establish Connection to ROV**:
//    - Power on BlueROV2, connect via Ethernet (tether).
//    - Access `BlueOS` (often at http://192.168.2.2/).
//    - Optionally use `QGroundControl` for final checks and calibrations.

// 2. **Launch Nodes**:

//    .. code-block:: bash

//        ros2 launch scripts system_launch.launch.py

//    (Example command; adapt to your own launch files and node setup.)

// 3. **Run the GUI**:
//    - **PyQt5**:

//      .. code-block:: bash

//          python3 gui/qt_gui.py

//    - **Unity**:
//      - Open the Unity project in ``gui/unity_gui/``.
//      - Press **Play**; ensure the ROS-TCP Connector is set.

// 4. **Autonomous Mode**:
//    - Use the GUI toggle to switch from Manual to Autonomous.
//    - Depth-hold and yaw-hold PIDs engage to stabilize the ROV.

// 5. **Detect & Grasp**:
//    - YOLOv5 detects the black box and handle (bounding boxes in camera feed).
//    - Approach and close the gripper once aligned.

// Simulation
// ==========
// A simulation environment with **Blue** (a ROS2-based underwater simulator) is also provided:

// 1. **Install Blue** following its GitHub instructions.
// 2. Place the models from ``simulation/BLUE_sim/`` into your Blue environment.
// 3. Start the simulator and launch the same ROS2 nodes.
// 4. The ROV should respond to the same control and detection logic.

// GUI Implementations
// ===================
// Two GUI options are available:

// - **PyQt5-based GUI** (in ``gui/qt_gui.py``)
//   - Real-time camera feed
//   - Sliders for depth, gain, alpha
//   - Buttons for manual motion, gripper commands
//   - Mode toggles for yaw hold and autonomous mode

// - **Unity-based GUI** (in ``gui/unity_gui/``)
//   - Real-time camera feed via ROS-TCP
//   - Similar controls in a 3D environment

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

// License
// =======
// This project is distributed under the MIT License. See ``LICENSE`` for details.

Authors & Contact
=================
- David Eskoundos: https://github.com/DavidMicheal
- Shubh Singhal

Institution: CIRTESU / Universitat Jaume I (UJI), Castellón, Spain

For inquiries or collaboration proposals, please open an issue in this repository or refer to the contact details in the `Final Report (docs/Final_Report.pdf)`.
