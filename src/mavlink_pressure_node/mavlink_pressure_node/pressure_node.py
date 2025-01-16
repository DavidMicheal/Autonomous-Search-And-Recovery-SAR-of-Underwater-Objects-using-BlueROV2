# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# from pymavlink import mavutil

# class MavlinkPressureNode(Node):
#     def __init__(self):
#         super().__init__('mavlink_pressure_node')

#         self.declare_parameter('port', 14555)  # Default MAVLink UDP port
#         port = self.get_parameter('port').value

#         self.pressure_publisher = self.create_publisher(FluidPressure, '/bluerov2/scaled_pressure2', 10)

#         self.get_logger().info(f"Listening for MAVLink messages on UDP port {port}")
#         self.connection = mavutil.mavlink_connection(f'udp:0.0.0.0:{port}')

#         self.timer = self.create_timer(0.01, self.read_mavlink)

#     def read_mavlink(self):
#         try:
#             message = self.connection.recv_match(blocking=False)
#             if not message:
#                 return

#             if message.get_type() == 'SCALED_PRESSURE2':
#                 pressure = message.press_abs  # Pressure in hPa
#                 # self.get_logger().info(f"SCALED_PRESSURE2: {pressure:.2f} hPa")

#                 pressure_msg = Float32()
#                 pressure_msg.data = pressure
#                 self.pressure_publisher.publish(pressure_msg)

#         except Exception as e:
#             self.get_logger().error(f"Error reading MAVLink message: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MavlinkPressureNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header
import time

class MavlinkPressureNode(Node):
    def __init__(self):
        super().__init__('mavlink_pressure_node')

        self.declare_parameter('port', 14555)  # Default MAVLink UDP port
        port = self.get_parameter('port').value

        self.pressure_publisher = self.create_publisher(FluidPressure, '/bluerov2/scaled_pressure2', 10)

        self.get_logger().info(f"Listening for MAVLink messages on UDP port {port}")
        self.connection = mavutil.mavlink_connection(f'udp:0.0.0.0:{port}')

        self.timer = self.create_timer(0.001, self.read_mavlink)

    def read_mavlink(self):
        try:
            message = self.connection.recv_match(blocking=False)
            if not message:
                return

            if message.get_type() == 'SCALED_PRESSURE2':
                pressure = message.press_abs  # Pressure in hPa
                self.get_logger().info(f"SCALED_PRESSURE2: {pressure:.2f} hPa")

                # Create FluidPressure message
                pressure_msg = FluidPressure()
                pressure_msg.header = Header()
                pressure_msg.header.stamp = self.get_clock().now().to_msg()  # Use ROS time
                pressure_msg.header.frame_id = 'base_link'
                pressure_msg.fluid_pressure = pressure * 100.0  # Convert hPa to Pa if needed
                pressure_msg.variance = 0.0  # Placeholder, update if variance is available

                self.pressure_publisher.publish(pressure_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading MAVLink message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkPressureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
