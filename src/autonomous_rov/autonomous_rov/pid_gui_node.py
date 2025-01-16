#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from threading import Thread
import signal

from PyQt5 import QtWidgets, QtCore
from std_msgs.msg import Float64


class PIDGuiNode(Node):
    def __init__(self):
        super().__init__("pid_gui_node")

        # Publishers for PID parameters
        self.kp_yaw_pub = self.create_publisher(Float64, "/bluerov2/kp_yaw", 10)
        self.ki_yaw_pub = self.create_publisher(Float64, "/bluerov2/ki_yaw", 10)
        self.kd_yaw_pub = self.create_publisher(Float64, "/bluerov2/kd_yaw", 10)
        self.kp_depth_pub = self.create_publisher(Float64, "/bluerov2/kp_depth", 10)
        self.ki_depth_pub = self.create_publisher(Float64, "/bluerov2/ki_depth", 10)
        self.kd_depth_pub = self.create_publisher(Float64, "/bluerov2/kd_depth", 10)


class PIDGUIApp(QtWidgets.QMainWindow):
    def __init__(self, node: PIDGuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("PID Parameter Tuner")
        self.setGeometry(100, 100, 500, 300)

        # Main layout
        main_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QGridLayout(main_widget)

        # Create 6 sliders and labels
        # We'll store them in a dict for convenience
        self.params = {
            "Kp Yaw": {
                "publisher": self.node.kp_yaw_pub,
                "slider": None,
                "label_value": None
            },
            "Ki Yaw": {
                "publisher": self.node.ki_yaw_pub,
                "slider": None,
                "label_value": None
            },
            "Kd Yaw": {
                "publisher": self.node.kd_yaw_pub,
                "slider": None,
                "label_value": None
            },
            "Kp Depth": {
                "publisher": self.node.kp_depth_pub,
                "slider": None,
                "label_value": None
            },
            "Ki Depth": {
                "publisher": self.node.ki_depth_pub,
                "slider": None,
                "label_value": None
            },
            "Kd Depth": {
                "publisher": self.node.kd_depth_pub,
                "slider": None,
                "label_value": None
            }
        }

        # We will create rows of (Label, Slider, ValueLabel)
        row = 0
        for name, d in self.params.items():
            # Parameter name label
            label = QtWidgets.QLabel(name)
            main_layout.addWidget(label, row, 0)

            # Slider (0 to 5000 -> 0.00 to 50.00)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(5000)
            slider.setValue(0)  # Default 0.00
            slider.setTickInterval(500)
            slider.setSingleStep(1)
            slider.valueChanged.connect(lambda val, pname=name: self.slider_changed(pname, val))
            main_layout.addWidget(slider, row, 1)
            self.params[name]["slider"] = slider

            # Value label
            value_label = QtWidgets.QLabel("0.00")
            main_layout.addWidget(value_label, row, 2)
            self.params[name]["label_value"] = value_label

            row += 1

        self.setCentralWidget(main_widget)

    def slider_changed(self, param_name, slider_value):
        # Convert slider value to float with 0.01 precision
        # slider_value in [0..5000], so actual = slider_value / 100.0
        actual_value = slider_value / 100.0
        # Update the label
        self.params[param_name]["label_value"].setText(f"{actual_value:.2f}")

        # Publish the updated parameter
        msg = Float64()
        msg.data = actual_value
        self.params[param_name]["publisher"].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDGuiNode()

    app = QtWidgets.QApplication(sys.argv)
    gui = PIDGUIApp(node)

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
