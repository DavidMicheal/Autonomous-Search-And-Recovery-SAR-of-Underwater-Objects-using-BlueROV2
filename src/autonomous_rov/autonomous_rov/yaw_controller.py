import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from custom_msgs.msg import OverrideRCInput

class YawController(Node):
    def __init__(self):
        super().__init__('yaw_controller')
        self.subscription = self.create_subscription(
            Imu,
            'bluerov2/imu/data',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.pub_rcinn = self.create_publisher(Float64, '/rc_input', 10)
        self.joy_sub = self.create_subscription(Joy, '/joyy', self.joyCallback, 10)
        self.yaw_sub = self.create_subscription(Float64, '/yaw_target', self.desireYawCallback, 10)

        # PID parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05 # Derivative gain
        self.prev_btn_yaw_hold = 0

        # Mode params
        self.set_mode = [True, False, False]

        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

        # Desired yaw (setpoint in radians)
        self.desired_yaw = 0.0  # Example: keep the yaw at 0 (facing forward)
        self.current_yaw = None

    def desireYawCallback(self, data):
        # Only in AUTOMATIC Mode
        if self.set_mode[1]:
            self.desired_yaw = data

    def joyCallback(self, data):
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_yaw_hold_mode = data.buttons[1]

        if btn_depth_hold_mode and not self.prev_btn_yaw_hold:
            self.depth_target = current_yaw
            # self.depth_hold_enabled = True
            self.get_logger().info(f"Yaw target set to: {self.yaw_target}")
        self.prev_btn_yaw_hold = btn_yaw_hold_mode

        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.reset_variables()
            self.get_logger().info("Mode manual")
        if btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            self.reset_variables()
            self.get_logger().info("Mode automatic")
        if btn_corrected_mode and not self.set_mode[2]:
            # self.init_a0 = self.init_p0 = True
            self.set_mode = [False, False, True]
            self.reset_variables()
            self.get_logger().info("Mode Yaw Hold")


    def imu_callback(self, msg: Imu):
        # Extract the yaw angle from the quaternion
        q = msg.orientation
        yaw = self.quaternion_to_euler_yaw(q.x, q.y, q.z, q.w)

        # Compute the control signal
        self.current_yaw = yaw
        control_signal = self.pid_control(yaw)

        # Publish the control signal to the thruster topic
        self.send_thruster_command(control_signal)

    def quaternion_to_euler_yaw(self, x, y, z, w):
        """Convert quaternion to yaw (in radians)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def pid_control(self, current_yaw):
        """Calculate the PID control signal."""
        error = self.desired_yaw - current_yaw

        # Ensure error is within [-pi, pi]
        error = (error + math.pi) % (2 * math.pi) - math.pi

        # Calculate time step
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.prev_time if self.prev_time else 0.0
        self.prev_time = current_time

        if dt > 0:
            # Proportional term
            proportional = self.kp * error

            # Integral term
            self.integral += error * dt
            integral = self.ki * self.integral

            # Derivative term
            derivative = self.kd * (error - self.prev_error) / dt
            self.prev_error = error

            # Total control signal
            control_signal = proportional + integral + derivative
            pwm_control = mapValueScalSat(control_signal)

            rcinn_msg = OverrideRCInput()
            rcinn_msg.pitch = 1500
            rcinn_msg.roll = 1500
            rcinn_msg.throttle = 1500
            rcinn_msg.yaw = control_signal
            rcinn_msg.forward = 1500
            rcinn_msg.lateral = 1500
            return self.pub_rcinn.publish(rcinn_msg)
 

    def mapValueScalSat(self, value):
        # Scale joystick input (-1 to 1) to PWM range (1100-1900) with neutral at 1500
        pulse_width = value * 400 + 1500
        # Saturate values within range
        pulse_width = min(max(pulse_width, 1100), 1900)
        return int(pulse_width)

def main(args=None):
    rclpy.init(args=args)
    node = YawController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
