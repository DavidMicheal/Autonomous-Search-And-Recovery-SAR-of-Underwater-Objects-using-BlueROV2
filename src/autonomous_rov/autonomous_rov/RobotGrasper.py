import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64,Bool
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from geometry_msgs.msg import Point


class RobotGrasper(Node):
    def __init__(self):
        super().__init__('RobotGrasper')

        # Subscriptions
        self.redbox_subscriber = self.create_subscription(
            Point, '/redbox_center', self.redbox_callback, 10
        )
        self.handle_subscriber = self.create_subscription(
            Point, '/handle_center', self.handle_callback, 10
        )

        self.alpha_subscriber = self.create_subscription(
            Float64, '/alpha', self.alpha_callback, 10
        )

        self.redbox_distance_subscriber = self.create_subscription(
            Float64, '/redbox_distance', self.redbox_distance_callback, 10
        )
        self.automatic_alpha_subscriber = self.create_subscription(
            Bool, '/automatic_alpha', self.automatic_alpha_callback, 10
        )

        self.redbox_distance = None

        # self.target_subscriber = self.create_subscription(
        #     PointStamped, '/target_coordinates', self.target_callback, 10
        # )
        
        # Initialize variables
        self.redbox_point = None
        self.handle_point = None
        self.desired_x = 600.0
        
        # Track last detection times
        self.redbox_last_detected = None
        self.handle_last_detected = None


        self.filtered_error = 0.0  # Initialize filtered error
        self.alpha = 0.2  # Low-pass filter gain (tune this value)

        self.automatic_alpha= True
        # We'll consider a detection stale if no message arrives after this many seconds
        self.detection_timeout = 0.5
        
        self.timer = self.create_timer(0.1, self.process_target)



        # Publishers
        self.depth_publisher = self.create_publisher(Float64, '/bluerov2/depth_desired', 10)
        self.yaw_publisher = self.create_publisher(Float64, '/bluerov2/yaw_target', 10)
        self.yaw_error_publisher = self.create_publisher(Float64, '/bluerov2/yaw_error', 10)
        

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Setpoint Processing
        # camera parameters
        self.u0 = 320
        self.v0 = 240
        self.lx = 455
        self.ly = 455
        # self.kud =0.00683 
        # self.kdu = -0.01424  

        self.u0 = 653.34  # Principal point x-coordinate (cx)
        self.v0 = 321.61  # Principal point y-coordinate (cy)
        self.lx = 913.63  # Focal length in pixels along the x-axis (fx)
        self.ly = 687.89  # Focal length in pixels along the y-axis (fy)
    def automatic_alpha_callback(self, msg: Bool):
        self.automatic_alpha = msg.data

    def redbox_distance_callback(self, msg: Float64):
        self.redbox_distance = msg.data

    def alpha_callback(self, msg: Float64):
        self.alpha = msg.data
        
    def redbox_callback(self, msg: Point):
        # Update stored redbox center
        self.redbox_point = msg
        self.redbox_last_detected = self.get_clock().now()

    def handle_callback(self, msg: Point):
        # Update stored handle center
        self.handle_point = msg
        self.handle_last_detected = self.get_clock().now()


    def process_target(self):
        current_time = self.get_clock().now()
        
        # If we have a redbox detection time, check if too old
        if self.redbox_last_detected is not None:
            if (current_time - self.redbox_last_detected).nanoseconds * 1e-9 > self.detection_timeout:
                self.redbox_point = None

        # If we have a handle detection time, check if too old
        if self.handle_last_detected is not None:
            if (current_time - self.handle_last_detected).nanoseconds * 1e-9 > self.detection_timeout:
                self.handle_point = None
        # Decision logic:
        # if self.handle_point is not None:
        #     target_x = self.handle_point.x
        #     target_detected = True
        #     target_name = "handle"
        if self.redbox_point is not None:
            target_x = self.redbox_point.x
            target_detected = True
            target_name = "redbox"
        else:
            target_detected = False

        if target_detected:
            if self.automatic_alpha:
                self.alpha = self.redbox_distance/2
                # set the maximum of alpha is 1 using max
                self.alpha = max(0.0, min(1.0, self.alpha))
            # log the alpha
            self.get_logger().info(f"Alpha: {self.alpha:.2f}")
            
            error = target_x - self.desired_x
            normalized_error = error / 600.0  # adjust if image width differs
            self.filtered_error = self.alpha * normalized_error + (1 - self.alpha) * self.filtered_error

            self.get_logger().info(
                f"Target = {target_name}, X = {target_x:.2f}, Error = {error:.2f}, Normalized = {normalized_error:.2f}, Filtered = {self.filtered_error:.2f}"
            )
            # Publish error
            error_msg = Float64()
            # error_msg.data = normalized_error
            error_msg.data = self.filtered_error

            self.yaw_error_publisher.publish(error_msg)

        else:
            self.filtered_error = 0.0  # Reset filtered error if no target
            self.yaw_error_publisher.publish(Float64(data=0.0))
            self.get_logger().info("No target detected")


    
    # # THIS CODE SHOULD BE IN THE OBJECT DETECTION SCRIPT where the point is getting published and same goes for camera parameters
    # def convPixel2Meters(self, target_point):
    #     target_point.point.z = 1.0
    #     convertedX = (float(target_point.point.x)-self.u0) * target_point.point.z / self.lx
    #     convertedY = (float(target_point.point.y)-self.v0) * target_point.point.z / self.ly

    #     target_point.point.x = convertedX
    #     target_point.point.y = convertedY

    # def target_callback(self, target_point: PointStamped):
    #     self.convPixel2Meters(target_point)
    #     print(F"Desired Point: {target_point.point.x, target_point.point.y, target_point.point.z}")

    #     # R = np.array([
    #     #     [0, 1, 0],  # +z_camera -> +x_robot
    #     #     [0, 0, 1],  # +x_camera -> +y_robot
    #     #     [1, 0, 0]   # +y_camera -> +z_robot
    #     # ])
    #     R = np.array([
    #         [0,0,1],
    #         [1,0,0],
    #         [0,1,0]
    #     ])
    #     point = np.array([target_point.point.x, target_point.point.y, target_point.point.z])
    #     transformed_point = R @ point
        
    #     # print(f"Transformed point= {transformed_point}")
    #     # Compute desired depth (z-axis in body frame)
    #     desired_depth = transformed_point[2]

    #     # Compute desired yaw (angle to target in x-y plane)
    #     x, y = transformed_point[0], transformed_point[1]
    #     desired_yaw = math.atan2(y, x)

    #     # Publish desired depth
    #     depth_msg = Float64()
    #     # depth_msg.data = desired_depth
    #     # self.depth_publisher.publish(depth_msg)

    #     # Publish desired yaw
    #     yaw_msg = Float64()
    #     yaw_msg.data = desired_yaw
    #     # yaw_msg.data = math.degrees(desired_yaw)  # Convert to degrees if required by PID
    #     self.yaw_publisher.publish(yaw_msg)

    #     self.get_logger().info(
    #         f"Target transformed. Depth:, Yaw: {math.degrees(desired_yaw):.2f} degrees"
    #     )

def main(args=None):
    rclpy.init(args=args)
    node = RobotGrasper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
