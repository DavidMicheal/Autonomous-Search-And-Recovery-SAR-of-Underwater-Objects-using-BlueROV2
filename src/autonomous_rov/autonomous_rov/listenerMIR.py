
#!/usr/bin/env python
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
import traceback
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from struct import pack, unpack
from std_msgs.msg import Int16, Float64, Empty, Float64MultiArray, String
from sensor_msgs.msg import Joy, Imu, FluidPressure, LaserScan
from mavros_msgs.srv import CommandLong, SetMode, StreamRate
from mavros_msgs.msg import OverrideRCIn, Mavlink 
from geometry_msgs.msg import Twist
from my_python_package.msg import OverrideRCInput


class AlphaBetaFilter:
    def _init_(self,initial_depth,a=0.1,b=0.005,dt=1/58):
        self.a = a
        self.b = b
        self.dt = dt
        self.vk=0
        self.xk=0
        self.xk_0=initial_depth
        self.vk_0=0

    def filter_step(self, xm):
        self.xk = self.xk_0 + self.vk_0*self.dt
        self.vk = self.vk_0

        self.rk = xm - self.xk
        self.xk +=self.a*self.rk
        self.vk += (self.b*self.rk)/self.dt

        self.xk_0 = self.xk
        self.vk_0 = self.vk 
        
        return self.vk_0

class MyPythonNode(Node):
    def __init__(self):
        super().__init__("listenerMIR2")
        self.get_logger().info("This node is named listenerMIR")

        self.ns = self.get_namespace()
        self.get_logger().info("namespace = " + self.ns)

        # Publishers
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.pub_angle_degre = self.create_publisher(Twist, 'angle_degree', 10)
        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_angular_velocity = self.create_publisher(Twist, 'angular_velocity', 10)
        self.pub_linear_velocity = self.create_publisher(Twist, 'linear_velocity', 10)
        # self.pub_depth_desired = self.create_publisher(Float64, 'depth_desired', 10)
        self.get_logger().info("Publishers created.")

        # Arm disarm, stream rate, and timer
        self.armDisarm(False)
        self.setStreamRate(25)  # 25 Hz
        self.subscriber()
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50 msec - 20 Hz

        # Variables
        self.set_mode = [True, False, False, False]
        self.init_a0 = True
        self.init_p0 = True
        self.arming = False
        self.angle_roll_a0 = self.angle_pitch_a0 = self.angle_yaw_a0 = 0.0
        self.depth_wrt_startup = self.depth_p0 = 0
        self.pinger_confidence = self.pinger_distance = 0
        self.Vmax_mot = 1900
        self.Vmin_mot = 1100
        self.Correction_yaw = self.Correction_depth = 1500

      # Depth control variables
        self.depth_hold_enabled = False
        self.prev_btn_depth_hold = 0
        self.depth_integral = 0.0
        self.depth_prev_error = 0.0
        self.depth_last_time = None
        self.depth_target = 1.0  # Desired depth in meters
        self.depth_Kp = 20.0
        self.depth_Ki = 0.4
        self.depth_Kd = 0.2
        self.max_control_signal = 100.0  # Max control signal for mapping
        self.init_p0 = True  # To initialize depth offset
        self.latest_pressure = 0

    def timer_callback(self):
        if self.set_mode[0]:
            # self.setOverrideRCIN(1500, 1800, 1800, 1800, 1800, 1500)
              # Commands sent inside joyCallback
            return
        elif self.set_mode[1]:
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
        elif self.set_mode[2]:
            self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, 1500, 1500)

    def armDisarm(self, armed):
        cli = self.create_client(CommandLong, 'cmd/command')
        result = cli.wait_for_service(timeout_sec=2.0)
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 400
        req.param1 = 1.0 if armed else 0.0
        cli.call_async(req)
        self.get_logger().info("Arming" if armed else "Disarming" + " Succeeded")

    def manageStabilize(self, stabilized):
        cli = self.create_client(SetMode, 'set_mode')
        result = cli.wait_for_service(timeout_sec=2.0)
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "0" if stabilized else "19"
        cli.call_async(req)
        self.get_logger().info("set mode to " + ("STABILIZE" if stabilized else "MANUAL") + " Succeeded")

    def setStreamRate(self, rate):
        cli = self.create_client(StreamRate, 'set_stream_rate')
        result = cli.wait_for_service(timeout_sec=2.0)
        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = rate
        req.on_off = True
        cli.call_async(req)
        self.get_logger().info("Set stream rate Succeeded")
    
    def reset_variables(self):
        self.depth_integral = 0
        self.depth_last_time = None
        self.depth_prev_error = 0

    def joyCallback(self, data):
        btn_arm = data.buttons[7]
        btn_disarm = data.buttons[6]
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_corrected_mode = data.buttons[0]
        btn_depth_hold_mode = data.buttons[1]
        
        if btn_disarm == 1 and self.arming:
            self.arming = False
            self.armDisarm(self.arming)

        if btn_arm == 1 and not self.arming:
            self.arming = True
            self.armDisarm(self.arming)

        if btn_depth_hold_mode and not self.prev_btn_depth_hold:
            self.set_mode = [False, False, False, True]
            rho = 1000.0  # Density of water in kg/m^3
            g = 9.80665   # Gravity in m/s^2
            current_pressure = self.latest_pressure  # Store the latest pressure in the callback
            current_depth = (current_pressure - 101300) / (rho * g) - self.depth_p0
            self.depth_target = current_depth
            self.depth_hold_enabled = True
            self.get_logger().info(f"Depth target set to: {self.depth_target}")
        self.prev_btn_depth_hold = btn_depth_hold_mode

        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False, False]
            self.reset_variables()
            self.get_logger().info("Mode manual")
        if btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False, False]
            self.reset_variables()
            self.get_logger().info("Mode automatic")
        if btn_corrected_mode and not self.set_mode[2]:
            self.init_a0 = self.init_p0 = True
            self.set_mode = [False, False, True, False]
            self.reset_variables()
            self.get_logger().info("Mode correction")

    def velCallback(self, cmd_vel):
        if not (self.set_mode[1] or self.set_mode[2]):
            roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
            yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
            ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
            forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
            lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
            pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)
            self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)
			

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
        channels = [1500] * 18
        channels[0] = int(channel_pitch)       # Channel 1: Pitch
        channels[1] = int(channel_roll)        # Channel 2: Roll
        channels[2] = int(channel_throttle)    # Channel 3: Throttle (Heave)
        channels[3] = int(channel_yaw)         # Channel 4: Yaw
        channels[4] = int(channel_forward)     # Channel 5: Forward (Surge)
        channels[5] = int(channel_lateral)     # Channel 6: Lateral (Sway)
        msg_override = OverrideRCIn()
        msg_override.channels = channels
        self.pub_msg_override.publish(msg_override)

    def mapValueScalSat(self, value):
        pulse_width = value * 400 + 1500
        return int(max(min(pulse_width, 1900), 1100))

    def OdoCallback(self, data):
        # print("shubh")
        # Code for processing orientation and angular velocity
        pass

    def RelAltCallback(self, data):
        pass

    def DvlCallback(self, data):
        u = data.velocity.x
        v = data.velocity.y
        w = data.velocity.z
        vel = Twist()
        vel.linear.x, vel.linear.y, vel.linear.z = u, v, w
        self.pub_linear_velocity.publish(vel)
        
    
    def mavlink_callback(self, data):
        # Code for processing mavlink data
        pass

    def pingerCallback(self, data):
        self.pinger_distance, self.pinger_confidence = data.data[0], data.data[1]

    def PressureCallback(self, data):
        pressure = data.fluid_pressure
        self.latest_pressure = pressure
        rho = 1000.0  # Density of water in kg/m^3
        g = 9.80665   # Gravity in m/s^2

        if self.init_p0:
            self.depth_p0 = (pressure - 101300) / (rho * g)
            self.init_p0 = False

        depth = (pressure - 101300) / (rho * g) - self.depth_p0
        
        # Publish depth for monitoring
        depth_msg = Float64()
        depth_msg.data = depth

        self.pub_depth.publish(depth_msg)

        if self.set_mode[2]:
        # PID control
            depth_error = self.depth_target - depth
            current_time = self.get_clock().now().nanoseconds * 1e-9  # Convert to seconds

            if self.depth_last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.depth_last_time

            self.depth_last_time = current_time

            if dt > 0.0:
                derivative = (depth_error - self.depth_prev_error) / dt
            else:
                derivative = 0.0

            self.depth_integral += depth_error * dt

            control_signal = (self.depth_Kp * depth_error +
                            self.depth_Ki * self.depth_integral +
                            self.depth_Kd * derivative)

            self.depth_prev_error = depth_error

            # Map control signal to PWM
            self.Correction_depth = self.map_control_signal_to_pwm(control_signal)
        else:
            return
    
    # def desiredDepthCallback(self, desriedDepth):
    #     self.depth_target = desriedDepth

    def map_control_signal_to_pwm(self, control_signal):
        pwm_neutral = 1500
        pwm_range = 400  # PWM varies from 1100 to 1900
        max_signal = self.max_control_signal
        pwm = pwm_neutral + (control_signal / max_signal) * pwm_range
        pwm = max(min(pwm, 1900), 1100)
        return int(pwm)

    def subscriber(self):
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.subjoy = self.create_subscription(Joy, "joy", self.joyCallback, qos_profile)
        self.subcmdvel = self.create_subscription(Twist, "cmd_vel", self.velCallback, qos_profile)
        self.subimu = self.create_subscription(Imu, "imu/data", self.OdoCallback, qos_profile)
        self.subrel_alt = self.create_subscription(Float64, "global_position/rel_alt", self.RelAltCallback, qos_profile)
        self.subwater_pressure = self.create_subscription(Mavlink, "mavlink/from", self.mavlink_callback, qos_profile)
        self.subping = self.create_subscription(Float64MultiArray, "distance_sonar", self.pingerCallback, qos_profile)
        self.subpressure = self.create_subscription(FluidPressure, "/bluerov2/scaled_pressure2", self.PressureCallback, qos_profile)
        # self.depth_desired_sub = self.create_subscription(Float64, "/bluerov2/depth_desired", self.desiredDepthCallback ,qos_profile)
        self.get_logger().info("Subscriptions done.")
		
def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# #!/usr/bin/env python
# import rclpy
# import math
# import numpy as np
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from sensor_msgs.msg import Joy, Imu, FluidPressure
# from std_msgs.msg import Float64
# from mavros_msgs.msg import OverrideRCIn
# from geometry_msgs.msg import Twist
# from mavros_msgs.srv import CommandLong, SetMode, StreamRate

# class MyPythonNode(Node):
#     def _init_(self):
#         super()._init_("listenerMIR")
#         self.get_logger().info("This node is named listenerMIR")

#         self.ns = self.get_namespace()
#         self.get_logger().info("namespace = " + self.ns)

#         # Publishers
#         self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
#         self.pub_angle_degre = self.create_publisher(Twist, 'angle_degree', 10)
#         self.pub_depth = self.create_publisher(Float64, 'depth', 10)
#         self.pub_angular_velocity = self.create_publisher(Twist, 'angular_velocity', 10)
#         self.pub_linear_velocity = self.create_publisher(Twist, 'linear_velocity', 10)
#         self.get_logger().info("Publishers created.")

#         # Arm disarm, stream rate, and timer
#         self.armDisarm(False)
#         self.setStreamRate(25)  # 25 Hz
#         self.subscriber()
#         self.timer = self.create_timer(0.05, self.timer_callback)  # 50 msec - 20 Hz

#         # Variables
#         self.set_mode = [True, False, False]
#         self.init_a0 = True
#         self.init_p0 = True
#         self.arming = False
#         self.angle_roll_a0 = self.angle_pitch_a0 = self.angle_yaw_a0 = 0.0
#         self.depth_wrt_startup = self.depth_p0 = 0.0
#         self.pinger_confidence = self.pinger_distance = 0
#         self.Vmax_mot = 1900
#         self.Vmin_mot = 1100
#         self.Correction_yaw = self.Correction_depth = 1500

#         # Depth control variables
#         self.depth_hold_enabled = False
#         self.prev_btn_depth_hold = 0
#         self.depth_integral = 0.0
#         self.depth_prev_error = 0.0
#         self.depth_last_time = None
#         self.depth_target = 0.5  # Desired depth in meters
#         self.depth_Kp = 20.0
#         self.depth_Ki = 0.4
#         self.depth_Kd = 0.2
#         self.max_control_signal = 100.0  # Max control signal for mapping
#         self.init_p0 = True  # To initialize depth offset

#     def timer_callback(self):
#         if self.set_mode[0]:  # Manual mode
#             return
#         elif self.set_mode[1]:
#             self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
#         elif self.set_mode[2]:
#             self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, 1500, 1500)
#         elif self.depth_hold_enabled:
#             self.setOverrideRCIN(1500, 1500, self.Correction_depth, 1500, 1500, 1500)
#         else:
#             self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)

#     def armDisarm(self, armed):
#         cli = self.create_client(CommandLong, 'cmd/command')
#         result = cli.wait_for_service(timeout_sec=2.0)
#         req = CommandLong.Request()
#         req.broadcast = False
#         req.command = 400
#         req.param1 = 1.0 if armed else 0.0
#         cli.call_async(req)
#         self.get_logger().info(("Arming" if armed else "Disarming") + " Succeeded")

#     def setStreamRate(self, rate):
#         cli = self.create_client(StreamRate, 'set_stream_rate')
#         result = cli.wait_for_service(timeout_sec=2.0)
#         req = StreamRate.Request()
#         req.stream_id = 0
#         req.message_rate = rate
#         req.on_off = True
#         cli.call_async(req)
#         self.get_logger().info("Set stream rate Succeeded")

#     def joyCallback(self, data):
#         btn_arm = data.buttons[7]
#         btn_disarm = data.buttons[6]
#         btn_manual_mode = data.buttons[3]
#         btn_automatic_mode = data.buttons[2]
#         btn_corrected_mode = data.buttons[0]
#         btn_depth_hold = data.buttons[4]  # Assign a button for depth hold

#         # Depth hold mode toggle
#         if btn_depth_hold == 1 and self.prev_btn_depth_hold == 0:
#             self.depth_hold_enabled = not self.depth_hold_enabled
#             self.get_logger().info("Depth hold mode " + ("activated" if self.depth_hold_enabled else "deactivated"))
#             if self.depth_hold_enabled:
#                 self.init_p0 = True  # Initialize depth offset
#                 self.depth_target = 0.5  # Set desired depth here
#         self.prev_btn_depth_hold = btn_depth_hold

#         if btn_disarm == 1 and self.arming:
#             self.arming = False
#             self.armDisarm(self.arming)

#         if btn_arm == 1 and not self.arming:
#             self.arming = True
#             self.armDisarm(self.arming)

#         if btn_manual_mode and not self.set_mode[0]:
#             self.set_mode = [True, False, False]
#             self.get_logger().info("Mode manual")
#         if btn_automatic_mode and not self.set_mode[1]:
#             self.set_mode = [False, True, False]
#             self.get_logger().info("Mode automatic")
#         if btn_corrected_mode and not self.set_mode[2]:
#             self.init_a0 = self.init_p0 = True
#             self.set_mode = [False, False, True]
#             self.get_logger().info("Mode correction")

#     def velCallback(self, cmd_vel):
#         if not (self.set_mode[1] or self.set_mode[2]):
#             roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
#             yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
#             ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
#             forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
#             lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
#             pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)
#             self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)

#     def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
#         channels = [1500] * 18
#         channels[0] = int(channel_pitch)       # Channel 1: Pitch
#         channels[1] = int(channel_roll)        # Channel 2: Roll
#         channels[2] = int(channel_throttle)    # Channel 3: Throttle (Heave)
#         channels[3] = int(channel_yaw)         # Channel 4: Yaw
#         channels[4] = int(channel_forward)     # Channel 5: Forward (Surge)
#         channels[5] = int(channel_lateral)     # Channel 6: Lateral (Sway)
#         msg_override = OverrideRCIn()
#         msg_override.channels = channels
#         self.pub_msg_override.publish(msg_override)

#     def mapValueScalSat(self, value):
#         pulse_width = value * 400 + 1500
#         return int(max(min(pulse_width, 1900), 1100))

#     def OdoCallback(self, data):
#         orientation = data.orientation
#         angular_velocity = data.angular_velocity

#         # Extraction of yaw angle
#         q = [orientation.x, orientation.y, orientation.z, orientation.w]
#         euler = self.euler_from_quaternion(q)
#         angle_roll = euler[0]
#         angle_pitch = euler[1]
#         angle_yaw = euler[2]

#         if self.init_a0:
#             # First execution, initialize
#             self.angle_roll_a0 = angle_roll
#             self.angle_pitch_a0 = angle_pitch
#             self.angle_yaw_a0 = angle_yaw
#             self.init_a0 = False

#         self.angle_wrt_startup = [
#             ((angle_roll - self.angle_roll_a0 + 3.0 * math.pi) % (2.0 * math.pi) - math.pi) * 180 / math.pi,
#             ((angle_pitch - self.angle_pitch_a0 + 3.0 * math.pi) % (2.0 * math.pi) - math.pi) * 180 / math.pi,
#             ((angle_yaw - self.angle_yaw_a0 + 3.0 * math.pi) % (2.0 * math.pi) - math.pi) * 180 / math.pi
#         ]

#         angle = Twist()
#         angle.angular.x = self.angle_wrt_startup[0]
#         angle.angular.y = self.angle_wrt_startup[1]
#         angle.angular.z = self.angle_wrt_startup[2]
#         self.pub_angle_degre.publish(angle)

#     def PressureCallback(self, data):
#         pressure = data.fluid_pressure
#         rho = 1000.0  # Density of water in kg/m^3
#         g = 9.80665   # Gravity in m/s^2

#         if self.init_p0:
#             self.depth_p0 = (pressure - 101300) / (rho * g)
#             self.init_p0 = False

#         depth = (pressure - 101300) / (rho * g) - self.depth_p0

#         # Publish depth for monitoring
#         depth_msg = Float64()
#         depth_msg.data = depth

#         print('Publishing Depth',depth)
#         self.pub_depth.publish(depth_msg)

#         if not self.depth_hold_enabled:
#             return

#         # PID control
#         depth_error = self.depth_target - depth
#         current_time = self.get_clock().now().nanoseconds * 1e-9  # Convert to seconds

#         if self.depth_last_time is None:
#             dt = 0.0
#         else:
#             dt = current_time - self.depth_last_time

#         self.depth_last_time = current_time

#         if dt > 0.0:
#             derivative = (depth_error - self.depth_prev_error) / dt
#         else:
#             derivative = 0.0

#         self.depth_integral += depth_error * dt

#         control_signal = (self.depth_Kp * depth_error +
#                           self.depth_Ki * self.depth_integral +
#                           self.depth_Kd * derivative)

#         self.depth_prev_error = depth_error

#         # Map control signal to PWM
#         self.Correction_depth = self.map_control_signal_to_pwm(control_signal)

#     def map_control_signal_to_pwm(self, control_signal):
#         pwm_neutral = 1500
#         pwm_range = 400  # PWM varies from 1100 to 1900
#         max_signal = self.max_control_signal
#         pwm = pwm_neutral + (control_signal / max_signal) * pwm_range
#         pwm = max(min(pwm, 1900), 1100)
#         return int(pwm)

#     def euler_from_quaternion(self, quat):
#         x, y, z, w = quat
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll_x = math.atan2(t0, t1)
#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch_y = math.asin(t2)
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw_z = math.atan2(t3, t4)
#         return roll_x, pitch_y, yaw_z

#     def subscriber(self):
#         qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
#                                  history=QoSHistoryPolicy.KEEP_LAST,
#                                  depth=1)
#         self.subjoy = self.create_subscription(Joy, "joy", self.joyCallback, qos_profile)
#         self.subcmdvel = self.create_subscription(Twist, "cmd_vel", self.velCallback, qos_profile)
#         self.subimu = self.create_subscription(Imu, "imu/data", self.OdoCallback, qos_profile)
#         self.subpressure = self.create_subscription(FluidPressure, "/bluerov2/imu/static_pressure",
#                                                     self.PressureCallback, qos_profile)
#         self.get_logger().info("Subscriptions done.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MyPythonNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if _name_ == "_main_":
#     main()