#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink
import pygame
from pygame.locals import *
from std_msgs.msg import UInt16, Bool, Float64
from sensor_msgs.msg import BatteryState, Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_msgs.msg import OverrideRCInput

class ManualController(Node):
    def __init__(self):
        super().__init__("manual_controller")

        # MAVLink configuration
        self.type = mavlink.MAV_TYPE_GCS
        self.autopilot = mavlink.MAV_AUTOPILOT_INVALID
        self.base_mode = mavlink.MAV_MODE_PREFLIGHT
        self.custom_mode = 0
        self.mavlink_version = 0
        self.heartbeat_period = 0.02

        # RC channel default values
        self.pitch = 1500
        self.roll = 1500
        self.throttle = 1500
        self.yaw = 1500
        self.forward = 1500
        self.lateral = 1500
        self.camera_pan = 1500
        self.camera_tilt = 1500
        self.lights = 1100
        self.gripper = 1500  # Neutral gripper position


        self.gain = 0.3
        # ROS publishers
        self.battery_pub = self.create_publisher(BatteryState, "/bluerov2/battery", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm_status", 10)
        self.publisher_rc = self.create_publisher(OverrideRCInput, '/rc_input', 10)
        self.publisher_ = self.create_publisher(Joy, 'joyy', 10)

        # Declare parameters for MAVLink connection
        self.declare_parameter("ip", "192.168.2.1")
        self.declare_parameter("port", 14552)
        self.declare_parameter("baudrate", 115200)

        self.bluerov_ip = self.get_parameter("ip").value
        self.bluerov_port = self.get_parameter("port").value
        self.bluerov_baudrate = self.get_parameter("baudrate").value

        # Connect to BlueROV2 via MAVLink
        self.get_logger().info("Connecting to BlueROV2...")
        self.connection = mavutil.mavlink_connection(
            f"udpin:{self.bluerov_ip}:{self.bluerov_port}", baudrate=self.bluerov_baudrate
        )
        self.connection.wait_heartbeat()
        self.get_logger().info("BlueROV2 connection established.")

        self.mav = self.connection.mav
        self.target = (self.connection.target_system, self.connection.target_component)

        # Request data streams
        self.get_logger().info("Requesting data stream...")
        self.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )

        # Initialize joystick
        pygame.init()
        self.joysticks = []
        for i in range(pygame.joystick.get_count()):
            self.joysticks.append(pygame.joystick.Joystick(i))
            self.joysticks[-1].init()
            self.get_logger().info(f"Joystick detected: {self.joysticks[-1].get_name()}")

        # Arm the vehicle
        self.arm()
 
        # Start manual control loop
        self.create_timer(0.04, self.update_inputs)

        # Start heartbeat loop
        self.create_timer(self.heartbeat_period, self.send_bluerov_commands)

        # Send Joy commands
        self.create_timer(self.heartbeat_period, self.joy_publish_data)

        self.set_mode = [True, False, False]


        # Create subscribers
        self.subscriber()
    
    def joy_publish_data(self):
        pygame.event.pump()
        joy_msg = Joy()

        if not self.joysticks:
            return  # No joystick connected

        # Assume first joystick
        joystick = self.joysticks[0]

        joy_msg.axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        joy_msg.buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

        # Publish the message
        self.publisher_.publish(joy_msg)
        # self.get_logger().info(f"Published Joy message: Axes: {joy_msg.axes}, Buttons: {joy_msg.buttons}")

    def arm(self):
        self.connection.arducopter_arm()
        self.get_logger().info("Arm requested, waiting...")
        self.connection.motors_armed_wait()
        self.get_logger().info("Thrusters armed!")

    def disarm(self):
        self.connection.arducopter_disarm()
        self.get_logger().info("Disarm requested, waiting...")
        self.connection.motors_disarmed_wait()
        self.get_logger().info("Thrusters disarmed!")

    def mapValueScalSat(self, value):
        # Scale joystick input (-1 to 1) to PWM range (1100-1900) with neutral at 1500
        pulse_width = value * 400 + 1500


        # Saturate values within range
        pulse_width = min(max(pulse_width, 1100), 1900)
        return int(pulse_width)
    def apply_custom_deadzone(self, value):
        # value is the final PWM after scaling and saturation (1100-1900)
        if value > 1500:
            # Start immediately at a higher value above 1500
            # For example, value=1501 would map to 1501+37=1538
            return value + 37
        elif value < 1500:
            # Start immediately at a lower value below 1500
            # For example, value=1499 would map to 1499-37=1462
            return value - 37
        else:
            # Exactly 1500 remains neutral
            return 1500
        

    def apply_custom_deadzone2(self, value):
        # value is the final PWM after scaling and saturation (1100-1900)
        if value > 1500:
            # Start immediately at a higher value above 1500
            # For example, value=1501 would map to 1501+37=1538
            return value + 19
        elif value < 1500:
            # Start immediately at a lower value below 1500
            # For example, value=1499 would map to 1499-37=1462
            return value - 17
        else:
            # Exactly 1500 remains neutral
            return 1500
        

    
    # def setOverrideRCIN(self, pitch, roll, throttle, yaw, forward, lateral):
    #     # Map thruster control values to RC channel

    #     rc_channel_values = (
    #         self.gripper, # Gripper control
    #         self.mapValueScalSat(roll),
    #         self.mapValueScalSat(throttle),
    #         self.mapValueScalSat(yaw),
    #         self.mapValueScalSat(forward),
    #         self.mapValueScalSat(lateral),
    #         self.lights,  # Camera pan (neutral)
    #         self.camera_tilt,  # Camera tilt (neutral)
    #         1500,  # Lights control
    #         self.gripper,  # Gripper control
    #         65535, 65535, 65535, 65535, 65535, 65535  # Unused channels
    #     )

    #     self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

    #     rcinn_msg = OverrideRCInput()
    #     rcinn_msg.pitch = self.mapValueScalSat(pitch)
    #     rcinn_msg.roll =    self.mapValueScalSat(roll)
    #     rcinn_msg.throttle =     self.mapValueScalSat(throttle)
    #     rcinn_msg.yaw =     self.mapValueScalSat(yaw)
    #     rcinn_msg.forward =   self.mapValueScalSat(forward)
    #     rcinn_msg.lateral =    self.mapValueScalSat(lateral)
    #     print(rcinn_msg)
    #     self.publisher_rc.publish(rcinn_msg)
    def setOverrideRCIN(self, pitch, roll, throttle, yaw, forward, lateral):
        # Map thruster control values
        pitch_pwm = self.mapValueScalSat(pitch)
        roll_pwm = self.mapValueScalSat(roll)
        throttle_pwm = self.mapValueScalSat(throttle)
        yaw_pwm = self.mapValueScalSat(yaw)
        forward_pwm = self.mapValueScalSat(forward)
        lateral_pwm = self.mapValueScalSat(lateral)

        # Apply custom deadzone logic before sending to thrusters
        pitch_pwm_mapped = self.apply_custom_deadzone(pitch_pwm)
        roll_pwm_mapped = self.apply_custom_deadzone(roll_pwm)
        throttle_pwm_mapped = self.apply_custom_deadzone(throttle_pwm)
        yaw_pwm_mapped = self.apply_custom_deadzone2(yaw_pwm)
        forward_pwm_mapped = self.apply_custom_deadzone(forward_pwm)
        lateral_pwm_mapped = self.apply_custom_deadzone(lateral_pwm)

        # Set the RC channel values with the adjusted PWM
        rc_channel_values = (
            pitch_pwm_mapped, # Gripper control
            roll_pwm_mapped,
            throttle_pwm_mapped,
            yaw_pwm_mapped,
            forward_pwm_mapped,
            lateral_pwm_mapped,
            self.lights,  # Camera pan (neutral)
            self.camera_tilt,  # Camera tilt (neutral)
            1500,  # Lights control
            self.gripper,  # Gripper control
            65535, 65535, 65535, 65535, 65535, 65535  # Unused channels
        ) 

        self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

        rcinn_msg = OverrideRCInput()
        # Note: If you do not want to publish the remapped values to /rc_input, skip updating rcinn_msg with the remapped values.
        # If you still want to show the final commanded values in logs or for debugging, you can update them:
        rcinn_msg.pitch = pitch_pwm
        rcinn_msg.roll = roll_pwm
        rcinn_msg.throttle = throttle_pwm
        rcinn_msg.yaw = yaw_pwm
        rcinn_msg.forward = forward_pwm
        rcinn_msg.lateral = lateral_pwm

        # If you do not want to publish the remapped RC values to /rc_input, comment out the next line.
        # If you only want them for debugging, leave it in.
        self.publisher_rc.publish(rcinn_msg)
        print(rcinn_msg)

    def send_bluerov_commands(self):
        # Example inputs (replace with actual joystick/sensor inputs)
        pitch = (self.pitch - 1500) / 400.0
        roll = (self.roll - 1500) / 400.0
        throttle = (self.throttle - 1500) / 400.0
        yaw = (self.yaw - 1500) / 400.0
        forward = (self.forward - 1500) / 400.0
        lateral = (self.lateral - 1500) / 400.0
 
        self.setOverrideRCIN(pitch, roll, throttle, yaw, forward, lateral)

    def update_inputs(self):
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                self.handle_joystick_motion(event)
            elif event.type == JOYBUTTONDOWN:
                self.handle_button_press(event)
            elif event.type == JOYBUTTONUP:
                self.handle_button_release(event)
            elif event.type == JOYHATMOTION:
                self.handle_hat_motion(event)

    def handle_joystick_motion(self, event):
            # value = event.value
            value = self.gain *event.value
            # print("value",value)
            # print("event.axis",event.axis)
            
            if event.axis == 0 and self.set_mode[0]:  # Right joystick horizontal (yaw - RC4)
                self.yaw = self.mapValueScalSat(value)
                print("dsdsdssdasasdasddas")
            elif event.axis == 1 and self.set_mode[0]:  # Right joystick vertical (throttle - RC3)
                self.throttle = self.mapValueScalSat(-value)
            elif event.axis == 4:  # Left joystick vertical (forward/backward - RC5)
                # Invert the value to correct the reversed forward movement
                # print("value",value)
                self.forward = self.mapValueScalSat(-value)
            elif event.axis == 3:  # Left joystick horizontal (lateral movement - RC6)
                self.lateral = self.mapValueScalSat(value)

    def handle_button_press(self, event):
        print(event.button)
        if event.button == 0:  # Button A: Close gripper
            self.gripper = 1900
        elif event.button == 1:  # Button B: Open gripper
            self.gripper = 1100
        elif event.button == 4:  # Left bumper: Decrease light intensity
            self.camera_tilt = 1300
        elif event.button == 5:  # Right bumper: Increase light intensity
            self.camera_tilt = 1700
        elif event.button == 7:  # Start button: Toggle arm/disarm
            self.disarm() if self.connection.motors_armed() else self.arm()
    
    def handle_button_release(self, event):
        if event.button == 4 or event.button == 5:  # Left or Right bumper released
            self.camera_tilt = 1500  # Return camera tilt to neutral
        elif event.button == 0 or event.button == 1:  # Gripper buttons released
            self.gripper = 1500

    def handle_hat_motion(self, event):
                # event.value contains the (x, y) direction of the D-pad
        x, y = event.value

        if x == -1:  # D-pad left: Decrease light intensity
            self.lights = max(1100, self.lights - 100)
            self.get_logger().info(f"Decreased lights intensity to {self.lights}")
        elif x == 1:  # D-pad right: Increase light intensity
            self.lights = min(1900, self.lights + 100)
            self.get_logger().info(f"Increased lights intensity to {self.lights}")

        if y == 1:  # D-pad up: Open gripper
            self.gripper = 1300
            self.get_logger().info("Gripper opened")
        elif y == -1:  # D-pad down: Close gripper
            self.gripper = 1700
            self.get_logger().info("Gripper closed")

        if x == 0 and y == 0:
            # D-pad released
            if self.gripper != 1500:
                self.gripper = 1500  # Reset gripper to neutral
                self.get_logger().info("D-pad released, gripper neutral")

    def shutdown(self):
        self.disarm()
        self.connection.close()
        self.get_logger().info("Controller shut down.")

    def RCCallback(self, data):
        # Execute only when AUTOMATIC Mode
        if self.set_mode[1] or self.set_mode[2]:
            msg = data
            self.pitch = msg.pitch if msg.pitch != 0 else self.pitch
            self.roll= msg.roll if msg.roll != 0 else self.roll
            self.throttle= msg.throttle if msg.throttle != 0 else self.throttle
            self.yaw = msg.yaw if msg.yaw != 0 else self.yaw
            self.forward =msg.forward if msg.forward != 0 else self.forward
            self.lateral =msg.lateral if msg.lateral !=0 else self.lateral


    def joyCallback(self, data):
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_depth_hold_mode = data.buttons[1]

        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.pitch = 1500
            self.roll = 1500
            self.throttle = 1500
            self.yaw = 1500
            self.forward = 1500
            self.lateral = 1500

            # self.reset_variables()
            self.get_logger().info("Mode manual")
        if btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            # self.reset_variables()
            self.get_logger().info("Mode automatic")
        # if btn_depth_hold_mode and not self.set_mode[2]:
        #     # self.init_a0 = self.init_p0 = True
        #     self.set_mode = [False, False, True]
        #     self.reset_variables()
        #     self.get_logger().info("Mode Deph Hold")
    def gainCallback(self, data):
        self.gain = data.data
        self.get_logger().info(f"Gain set to: {self.gain}")


    def subscriber(self):
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1)
        self.RCSub = self.create_subscription(OverrideRCInput, "/rc_input", self.RCCallback,qos_profile)
        self.sub_joy = self.create_subscription(Joy, '/joyy', self.joyCallback, 10)
        self.gain_sub = self.create_subscription(Float64, '/bluerov2/gain', self.gainCallback, 10)

def main(args=None):
    rclpy.init(args=args)
    node = ManualController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()