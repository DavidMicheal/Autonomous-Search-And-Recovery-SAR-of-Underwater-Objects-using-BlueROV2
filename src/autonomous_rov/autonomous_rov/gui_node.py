#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from threading import Thread
from PyQt5 import QtWidgets, QtGui, QtCore
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from custom_msgs.msg import OverrideRCInput
from sensor_msgs.msg import Joy
import signal
from geometry_msgs.msg import Point

class GUINode(Node):
    def __init__(self):
        super().__init__("gui_node")
        self.gain = 0.3  # Default gain value

        # Publishers
        self.depth_pub = self.create_publisher(Float64, "/bluerov2/depth_desired", 10)
        self.joy_pub = self.create_publisher(Joy, "/joyy", 10)
        self.rc_pub = self.create_publisher(OverrideRCInput, "/rc_input", 10)
        self.gain_pub = self.create_publisher(Float64, "/bluerov2/gain", 10)
        self.yaw_hold_pub = self.create_publisher(Bool, "/bluerov2/yaw_hold", 10)
        self.alpha_pub = self.create_publisher(Float64, "/alpha", 10)
        self.alpha_toggle_pub = self.create_publisher(Bool, "/automatic_alpha", 10)



        # Subscribers   
        self.create_subscription(Image, "/bluerov2/camera/image_raw", self.image_callback, 10)
        self.create_subscription(Image, "/Bluerov2/detected_objects/image" , self.image_detected_callback, 10)
        self.create_subscription(Float64, "/depth", self.depth_callback, 10)
        self.create_subscription(OverrideRCInput, "/rc_input", self.rc_input_callback, 10)
        self.create_subscription(Point, "/redbox_center", self.redbox_callback, 10)
        self.create_subscription(Point, "/handle_center", self.handle_callback, 10)

        
        
        self.bridge = CvBridge()    

        self.detected= None
        self.redbox_detected = False
        self.handle_detected = False
        # Timers for detection timeouts
        self.redbox_timer = self.create_timer(0.1, self.reset_redbox_detection)
        self.handle_timer = self.create_timer(0.1, self.reset_handle_detection)

        # Current data
        self.current_image = None
        self.current_depth = 0.0
        self.current_rc = OverrideRCInput()

        self.manual_mode = True  # keep track of current mode
        self.yaw_hold = False  # keep track of current yaw hold status


    def set_alpha_toggle(self, enable: bool):
        """Publish a message to toggle automatic alpha."""
        msg = Bool()
        msg.data = enable
        self.alpha_toggle_pub.publish(msg)

    def set_yaw_hold(self, enable: bool):
        """Publish a message to toggle yaw hold."""
        self.yaw_hold = enable
        msg = Bool()
        msg.data = enable
        self.yaw_hold_pub.publish(msg)
    
    def redbox_callback(self, msg):
        self.redbox_detected = True

    def handle_callback(self, msg):
        self.handle_detected = True

    def reset_redbox_detection(self):
        self.redbox_detected = False

    def reset_handle_detection(self):
        self.handle_detected = False


    def image_callback(self, msg):
        if self.detected is None:        
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image

    def image_detected_callback(self, msg):
        # print("Image Detected")
        self.detected= True
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.current_image = cv_image

    def depth_callback(self, msg):
        self.current_depth = msg.data

    def rc_input_callback(self, msg):
        self.current_rc = msg

    def set_depth(self, depth_value):
        msg = Float64()
        msg.data = -depth_value
        self.depth_pub.publish(msg)
   
    def set_alpha(self, alpha_value):
        self.alpha = alpha_value
        msg = Float64()
        msg.data = alpha_value
        self.alpha_pub.publish(msg)


    def set_gain(self, gain_value):
        self.gain = gain_value
        msg = Float64()
        msg.data = gain_value
        self.gain_pub.publish(msg)


    def toggle_mode(self, manual):  
        # Publish Joy message or any other logic to switch mode
        # Here we assume button[3] = manual_mode, button[2] = auto_mode
        joy_msg = Joy()
        joy_msg.axes = []
        joy_msg.buttons = [0]*10
        if manual:
            joy_msg.buttons[3] = 1  # manual
        else:
            joy_msg.buttons[2] = 1  # automatic
        self.joy_pub.publish(joy_msg)
        self.manual_mode = manual

    def set_gripper(self, open_gripper):
        # Publish RC input to open/close gripper
        # Suppose gripper is controlled via rcinn_msg.gripper channel (1100 open, 1900 close)
        rcinn_msg = OverrideRCInput()
        # rcinn_msg.pitch = 0
        rcinn_msg.roll = 0
        rcinn_msg.throttle = 0
        rcinn_msg.yaw = 0
        rcinn_msg.forward = 0
        rcinn_msg.lateral = 0
        if open_gripper:
            # rcinn_msg.lateral = 1500 # neutral, but...
            rcinn_msg.pitch = 1100  # open
        else:
            rcinn_msg.pitch = 1900  # close
        self.rc_pub.publish(rcinn_msg)

    def move_right_left(self,right):
        # Publish RC message to move right or left
        # right channel is rcinn_msg.lateral = 1600 for right, 1400 for left, neutral = 1500
        rcinn_msg = OverrideRCInput()
        rcinn_msg.pitch = 0
        rcinn_msg.roll = 0
        rcinn_msg.throttle = 0
        rcinn_msg.yaw = 0
        rcinn_msg.forward = 0
        base_value = 1500
        range_value = int(50 * self.gain)
        if right:
            rcinn_msg.lateral = base_value + range_value
        else:
            rcinn_msg.lateral = base_value - range_value
        self.rc_pub.publish(rcinn_msg)


    def move_forward_backward(self, forward):
        # Publish RC message to move forward or backward
        # forward channel is rcinn_msg.forward = 1600 for forward, 1400 for backward, neutral = 1500
        rcinn_msg = OverrideRCInput()
        rcinn_msg.pitch = 0
        rcinn_msg.roll = 0
        rcinn_msg.throttle = 0
        rcinn_msg.yaw = 0
        rcinn_msg.lateral = 0
        base_value = 1500
        range_value = int(25 * self.gain)
        if forward:
            rcinn_msg.forward = base_value + range_value
        else:
            rcinn_msg.forward = base_value - range_value
        self.rc_pub.publish(rcinn_msg)
    def reset_rc(self):
        # Reset all RC inputs to neutral (1500)
        rcinn_msg = OverrideRCInput()
        rcinn_msg.pitch = 1500
        rcinn_msg.roll = 0
        rcinn_msg.throttle = 0
        rcinn_msg.yaw = 0
        rcinn_msg.forward = 1500
        rcinn_msg.lateral = 1500
        self.rc_pub.publish(rcinn_msg)
class GUIApp(QtWidgets.QMainWindow):
    def __init__(self, node: GUINode):
        super().__init__()
        self.node = node

        self.setWindowTitle("ROV Control GUI")
        self.showMaximized()


        # Main layout
        main_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(main_widget)

        # Camera and Depth Slider Layout
        camera_depth_layout = QtWidgets.QHBoxLayout()

        # Camera display
        self.image_label = QtWidgets.QLabel("No image")
        self.image_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        camera_depth_layout.addWidget(self.image_label)


        depth_slider_layout = QtWidgets.QVBoxLayout()

        # Depth slider label
        depth_slider_layout.addWidget(QtWidgets.QLabel("Desired Depth:"))

        # Depth slider (vertical)
        self.depth_slider = QtWidgets.QSlider(QtCore.Qt.Vertical)  # Make slider vertical
        self.depth_slider.setMinimum(0)
        self.depth_slider.setMaximum(500)  # 0 to 5 m in increments of 0.1
        self.depth_slider.setValue(1)
        self.depth_slider.setTickInterval(1)
        self.depth_slider.setInvertedAppearance(True)  # Reverse the slider orientation
        self.depth_slider.valueChanged.connect(self.depth_slider_changed)
        depth_slider_layout.addWidget(self.depth_slider)

        # Depth value label
        self.depth_value_label = QtWidgets.QLabel("0.0 m")
        depth_slider_layout.addWidget(self.depth_value_label)

        camera_depth_layout.addLayout(depth_slider_layout)

        main_layout.addLayout(camera_depth_layout)


        # Alpha control
        alpha_layout = QtWidgets.QHBoxLayout()
        
        alpha_label = QtWidgets.QLabel("Alpha:")
        alpha_layout.addWidget(alpha_label)

        self.alpha_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.alpha_slider.setMinimum(0)  # Corresponds to alpha = 0.0
        self.alpha_slider.setMaximum(20)  # Corresponds to alpha = 1.0
        self.alpha_slider.setValue(6)  # Default value (0.3)
        self.alpha_slider.setTickInterval(1)
        self.alpha_slider.valueChanged.connect(self.alpha_slider_changed)
        alpha_layout.addWidget(self.alpha_slider)

        self.alpha_value_label = QtWidgets.QLabel("0.30")  # Default display
        alpha_layout.addWidget(self.alpha_value_label)
        self.alpha_toggle_button = QtWidgets.QPushButton("Automatic Alpha: OFF")
        self.alpha_toggle_button.setCheckable(True)
        self.alpha_toggle_button.setChecked(False)
        self.alpha_toggle_button.clicked.connect(self.toggle_alpha)
        alpha_layout.addWidget(self.alpha_toggle_button)
        main_layout.addLayout(alpha_layout)


        # Gain control
        gain_layout = QtWidgets.QHBoxLayout()   

        gain_label = QtWidgets.QLabel("Gain:")
        gain_layout.addWidget(gain_label)

        self.gain_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.gain_slider.setMinimum(0)  # Corresponds to gain = 0.0
        self.gain_slider.setMaximum(20)  # Corresponds to gain = 1.0
        self.gain_slider.setValue(6)  # Default value (0.5)
        self.gain_slider.setTickInterval(1)
        self.gain_slider.valueChanged.connect(self.gain_slider_changed)
        gain_layout.addWidget(self.gain_slider)

        self.gain_value_label = QtWidgets.QLabel("0.30")  # Default display
        gain_layout.addWidget(self.gain_value_label)

        main_layout.addLayout(gain_layout)


        # # Camera display
        # self.image_label = QtWidgets.QLabel("No image")
        # self.image_label.setFixedSize(640, 480)
        # main_layout.addWidget(self.image_label)

        # Current depth display
        self.depth_label = QtWidgets.QLabel("Current Depth: 0.0 m")
        main_layout.addWidget(self.depth_label)

        # # Depth slider
        # depth_layout = QtWidgets.QHBoxLayout()  
        # depth_layout.addWidget(QtWidgets.QLabel("Desired Depth:"))
        # self.depth_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        # self.depth_slider.setMinimum(0)
        # self.depth_slider.setMaximum(500)  # 0 to 5 m in increments of 0.1
        # self.depth_slider.setValue(1)
        # self.depth_slider.setTickInterval(1)
        # self.depth_slider.valueChanged.connect(self.depth_slider_changed)
        # depth_layout.addWidget(self.depth_slider)
        # self.depth_value_label = QtWidgets.QLabel("0.0 m")
        # depth_layout.addWidget(self.depth_value_label)
        # main_layout.addLayout(depth_layout)

        # Mode toggle
        self.mode_button = QtWidgets.QPushButton("Manual Mode")
        self.mode_button.setCheckable(True)
        self.mode_button.setChecked(True)
        self.mode_button.clicked.connect(self.toggle_mode)
        main_layout.addWidget(self.mode_button)

        
        
        # Gripper control
        gripper_layout = QtWidgets.QHBoxLayout()

        # Open gripper button
        gripper_open_btn = QtWidgets.QPushButton("Open Gripper")
        gripper_open_btn.pressed.connect(lambda: self.node.set_gripper(True))  # Open gripper
        gripper_open_btn.released.connect(lambda: self.node.reset_rc())  # Reset to neutral (1500)
        gripper_layout.addWidget(gripper_open_btn)

        # Close gripper button
        gripper_close_btn = QtWidgets.QPushButton("Close Gripper")
        gripper_close_btn.pressed.connect(lambda: self.node.set_gripper(False))  # Close gripper
        gripper_close_btn.released.connect(lambda: self.node.reset_rc())  # Reset to neutral (1500)
        gripper_layout.addWidget(gripper_close_btn)

        main_layout.addLayout(gripper_layout)



        # Forward/Backward control
        fb_layout = QtWidgets.QHBoxLayout()
        # Forward button
        forward_btn = QtWidgets.QPushButton("Forward")
        forward_btn.pressed.connect(lambda: self.node.move_forward_backward(True))  # Move forward
        forward_btn.released.connect(lambda: self.node.reset_rc())  # Reset to neutral (1500)
        fb_layout.addWidget(forward_btn)

        # Backward button
        backward_btn = QtWidgets.QPushButton("Backward")
        backward_btn.pressed.connect(lambda: self.node.move_forward_backward(False))  # Move backward
        backward_btn.released.connect(lambda: self.node.reset_rc())  # Reset to neutral (1500)
        fb_layout.addWidget(backward_btn)

        main_layout.addLayout(fb_layout)

        rl_layout = QtWidgets.QHBoxLayout()

                # Left button
        left_btn = QtWidgets.QPushButton("Left")
        left_btn.pressed.connect(lambda: self.node.move_right_left(False))  # Move left
        left_btn.released.connect(lambda: self.node.reset_rc())  # Reset to neutral (1500)
        rl_layout.addWidget(left_btn)


        # Right button
        right_btn = QtWidgets.QPushButton("Right")
        right_btn.pressed.connect(lambda: self.node.move_right_left(True))  # Move right
        right_btn.released.connect(lambda: self.node.reset_rc())  # Reset to neutral (1500)
        rl_layout.addWidget(right_btn)


        main_layout.addLayout(rl_layout)


        
        # Yaw hold toggle button
        self.yaw_hold_button = QtWidgets.QPushButton("Yaw Hold: OFF")
        self.yaw_hold_button.setCheckable(True)
        self.yaw_hold_button.setChecked(False)
        self.yaw_hold_button.clicked.connect(self.toggle_yaw_hold)
        main_layout.addWidget(self.yaw_hold_button)


        # Detection Indicators
        detection_layout = QtWidgets.QHBoxLayout()
        self.redbox_indicator = QtWidgets.QLabel("Red Box: Not Detected")
        self.redbox_indicator.setStyleSheet("background-color: red; color: white; padding: 5px;")
        detection_layout.addWidget(self.redbox_indicator)

        self.handle_indicator = QtWidgets.QLabel("Handle: Not Detected")
        self.handle_indicator.setStyleSheet("background-color: red; color: white; padding: 5px;")
        detection_layout.addWidget(self.handle_indicator)
        main_layout.addLayout(detection_layout)



        # RC Input display
        self.rc_label = QtWidgets.QLabel("RC Inputs: pitch=0 roll=0 throttle=0 yaw=0 forward=0 lateral=0")
        main_layout.addWidget(self.rc_label)

        self.setCentralWidget(main_widget)

        # Timer to update GUI
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # update every 100ms
    
    def toggle_yaw_hold(self):
        """Toggle yaw hold mode and publish the state."""
        if self.yaw_hold_button.isChecked():
            self.yaw_hold_button.setText("Yaw Hold: ON")
            self.node.set_yaw_hold(True)
        else:
            self.yaw_hold_button.setText("Yaw Hold: OFF")
            self.node.set_yaw_hold(False)

    def toggle_alpha(self):
        if self.alpha_toggle_button.isChecked():
            self.alpha_toggle_button.setText("Automatic Alpha: ON")
            self.node.set_alpha_toggle(True)
        else:
            self.alpha_toggle_button.setText("Automatic Alpha: OFF")
            self.node.set_alpha_toggle(False)
    def alpha_slider_changed(self, value):
        alpha_value = value / 20.0
        self.alpha_value_label.setText(f"{alpha_value:.2f}")
        self.node.set_alpha(alpha_value)
        
    def gain_slider_changed(self, value):
        gain_value = value / 20.0  # Convert slider value to a float between 0.0 and 1.0
        self.gain_value_label.setText(f"{gain_value:.2f}")
        self.node.set_gain(gain_value)


    def depth_slider_changed(self, value):
        depth_in_m = value / 100.0  # since max=50 means 5.0m
        self.depth_value_label.setText(f"{depth_in_m:.2f} m")
        self.node.set_depth(depth_in_m)

    def toggle_mode(self):
        if self.mode_button.isChecked():
            self.mode_button.setText("Manual Mode")
            self.node.toggle_mode(True)
        else:
            self.mode_button.setText("Automatic Mode")
            self.node.toggle_mode(False)

    def update_gui(self):
        # Update image
        if self.node.current_image is not None:
            qimg = self.convert_cv_qt(self.node.current_image)
            self.image_label.setPixmap(qimg)
        
        
        # Update detection indicators
        if self.node.redbox_detected:
            self.redbox_indicator.setText("Red Box: Detected")
            self.redbox_indicator.setStyleSheet("background-color: green; color: white; padding: 5px;")
        else:
            self.redbox_indicator.setText("Red Box: Not Detected")
            self.redbox_indicator.setStyleSheet("background-color: red; color: white; padding: 5px;")

        if self.node.handle_detected:
            self.handle_indicator.setText("Handle: Detected")
            self.handle_indicator.setStyleSheet("background-color: green; color: white; padding: 5px;")
        else:
            self.handle_indicator.setText("Handle: Not Detected")
            self.handle_indicator.setStyleSheet("background-color: red; color: white; padding: 5px;")

        # Update depth
        self.depth_label.setText(f"Current Depth: {self.node.current_depth:.2f} m")

        # Update RC Input display
        rc = self.node.current_rc
        self.rc_label.setText(
            f"RC Inputs: pitch={rc.pitch} roll={rc.roll} throttle={rc.throttle} yaw={rc.yaw} forward={rc.forward} lateral={rc.lateral}"
        )

    def convert_cv_qt(self, cv_img):
        """Convert from an OpenCV image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_qt_format.scaled(self.image_label.width(), self.image_label.height(), QtCore.Qt.KeepAspectRatio)
        return QtGui.QPixmap.fromImage(p)

def main(args=None):
    rclpy.init(args=args)
    node = GUINode()

    app = QtWidgets.QApplication(sys.argv)
    gui = GUIApp(node)

    # Run rclpy in a separate thread so GUI doesn't freeze
    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_thread = Thread(target=ros_spin, daemon=True)
    spin_thread.start()
    def signal_handler(sig, frame):
        gui.close()
        node.destroy_node()
        rclpy.shutdown()
        app.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
