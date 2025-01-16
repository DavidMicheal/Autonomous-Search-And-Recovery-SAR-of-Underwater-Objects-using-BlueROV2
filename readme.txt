Dear all,

The new Bluerovs now embed a Navigator board as a hat board directly connected to a RPI 4, instead of a Pixhawk board linked to a RPI3 through USB cable. In addition, the RPI4 is now running a linux OS with the BlueOS layer from BlueRobotics that allows configuring the system more easily. On the old bluerovs with RPI3/Pixhawk, a linux/ros1 system was embedded, with the mavros node being run onboard to deal with MAVlink communications with the hardware.
Now, there is no more ros onboard (it could be possible to embed one, but it is cumbersome due to blueOS running at the same time, and it is no more necessary).
As a consequence, the mavros node (ros2) will have to be run on your laptop (I built a launch file, that starts the mavros node, joy node and listenMIR node with ros2). BlueOS is configured to run a MAVlink endpoint client onboard, where the mavros node and the QGC on your laptop can connect to, the MAVlink endpoint acting as a relay of sensor readings/commands, etc.

Here are the instructions
LAPTOP:
1) install Ubuntu 22.04 LTS
2) install ros humble or ros iron (I installed ros iron, both are ros2 versions)
3) install mavros and mavlink (use sudo apt-get install ros-iron-mavros ros-iron-mavlink ros-iron-mavros-extras ros-iron-mavros-msgs)
4) install python3 stuff if not already done:
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon
5) Install GeographicLib datasets:
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod u+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
6) install additional stuff for 3D transformations:
sudo apt-get install ros-iron-tf-transformations
sudo pip3 install transform3d
7) Create workspace if not already done:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
8) install ros2 package autonomous_rov in src folder (file attached):
tar xzvf pack-autonomous-rov.tgz
9) Compile:
colcon build
10) install:
source install/setup.bash
11) create a network connection (USB/Ethernet) with the blue box (you need to connect the USB cable of the blue box to your laptop), using IP 192.168.2.1, and netmask 255.255.255.0

EMBEDDED BOARD (we need to update the firmware for the hardware part of the navigator board):
1) download the updated ardusub firmware from the link below:
https://filesender.renater.fr/?s=download&token=6e23a41e-86a7-448d-9902-3243915695ab
2) connect to blueOS by entering the IP address of the RPI in your browser : 192.168.2.2
Follow the instructions to upload the firmware
3) Reboot
4) connect to BlueOS and change the frame configuration into the custom one (instead of vectored). This is to address motors individually using the gamepad, or ros2 line command with ros2 topic pub ..., or using python code.
Restart the ardusub ardupilot.

LAPTOP:
1) connect the gamepad
2) run the launch file (if gamepad not detected, change the device path "joy_dev" inside the launch file):
ros2 launch autonomous_rov run_all.launch
3) modify the listenerMIR.py file to implement the algorithm designed with Prof. Omerdic.
4) do not forget to run the commands: colcon build, and source intall/setup.bash from the ros2_ws each time you modify a launch or python file.
5) camera video can be displayed with: ros2 launch autonomous_rov video.launch.py

N.B. : it is now possible to run the QGControl interface to do the calibration steps, without any change inside the RPI.
I have noticed that QGControl is however not very stable and may crash some time after the calibration. In addition, I am not sure whether it is fully compatible w.r.t. some commands. Never mind.

