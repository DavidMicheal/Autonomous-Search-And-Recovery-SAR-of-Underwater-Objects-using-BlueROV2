# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import torch
# import numpy as np
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Point

# class RedBoxDetector(Node):
#     def __init__(self):
#         super().__init__('red_box_detector')
#         self.subscription = self.create_subscription(
#             Image,
#             '/bluerov2/camera/image_raw',  # Replace with your camera topic
#             self.image_callback,
#             10)
#         self.bridge = CvBridge()

        
#         # Create publishers for the detected object centers
#         self.redbox_center_pub = self.create_publisher(Point, '/redbox_center', 10)
#         self.handle_center_pub = self.create_publisher(Point, '/handle_center', 10)



#         self.fx = 913.6264196313776
#         self.fy = 687.8934159348482
#         self.cx = 653.343101640157
#         self.cy = 321.6068376513913
#         self.focal_length = (self.fx + self.fy) / 2  # Average focal length

#         # Known object dimensions (in meters)
#         self.object_dimensions = {
#             'redbox': {'height': 0.30, 'width': 0.16},
#             'Handle': {'height': 0.10, 'width': 0.03}
#         }

#         self.robot_depth = None

#         # Subscribe to the depth topic
#         self.create_subscription(
#             Float64,
#             '/depth',
#             self.depth_callback,
#             10
#         )

#         # Load the trained YOLOv5 model
#         self.model = torch.hub.load('yolov5', 'custom', path='yolov5/runs/train/exp6/weights/best.pt', source='local')
#         self.model.conf = 0.5  # Confidence threshold (adjust as needed)
    
    
#     def depth_callback(self, msg):
#         self.robot_depth = msg.data    
#         # self.robot_depth = None
    
#     def image_callback(self, msg):
#         # Convert ROS Image message to OpenCV format
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # Perform detection
#         self.detect_red_box(frame)

#     def detect_red_box(self, frame):
#         # Convert BGR to RGB (YOLOv5 expects RGB images)
#         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#         # Run inference
#         results = self.model(img_rgb)

#         # Extract detections
#         detections = results.xyxy[0]  # [x1, y1, x2, y2, confidence, class]

#         # Define colors for each class
#         colors = {
#             'redbox': (0, 255, 0),  # Green for the red box
#             'Handle': (255, 0, 0)    # Blue for the handle
#         }

#         # Swap class names mapping
#         class_mapping = {
#             'redbox': 'Handle',
#             'Handle': 'redbox'
#         }




#         object_depth = 4.5  # Adjust as needed

#         # Check if robot depth is available
#         if self.robot_depth is not None:
#             delta_Z = abs(self.robot_depth - object_depth)
#         else:
#             delta_Z = None  # Unable to compute vertical distance




#         # Draw bounding boxes and labels on the frame
#         for *box, conf, cls in detections:
#             x1, y1, x2, y2 = map(int, box)
#             original_class_name = self.model.names[int(cls)]

#             # Swap the class names
#             class_name = class_mapping.get(original_class_name, original_class_name)

#             # Choose color based on the swapped class name
#             color = colors.get(class_name, (0, 255, 255))  # Default to yellow if class not in colors

#             # Compute the width and height of the bounding box in pixels
#             box_width_pixels = x2 - x1
#             box_height_pixels = y2 - y1

#             # Compute the center of the bounding box
#             x_center = (x1 + x2) / 2
#             y_center = (y1 + y2) / 2

#             # Publish the center of the red box
#             center_point = Point()
#             center_point.x = float(x_center)
#             center_point.y = float(y_center)
#             center_point.z = 0.0
#             if class_name == 'redbox':
#                 self.redbox_center_pub.publish(center_point)
#             elif class_name == 'Handle':
#                 self.handle_center_pub.publish(center_point)

#             # Get actual object dimensions
#             dimensions = self.object_dimensions.get(class_name, None)

#             if dimensions and self.focal_length:
#                 # Compute distance using width
#                 Z_width = (dimensions['width'] * self.focal_length) / box_width_pixels
#                 Z_height = (dimensions['height'] * self.focal_length) / box_height_pixels
#                 horizontal_distance = (Z_width + Z_height) / 2

#                 if delta_Z is not None:
#                     # Compute total distance
#                     total_distance = np.sqrt(horizontal_distance**2 + delta_Z**2)
#                     distance_text = f', Dist: {total_distance:.2f}m'
#                 else:
#                     distance_text = f', Dist: {horizontal_distance:.2f}m'
#             else:
#                 distance_text = ''


#             label = f'{class_name}: {conf:.2f}{distance_text}'

#             # Draw rectangle
#             cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
#             # Put label
#             cv2.putText(frame, label, (x1, y1 - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

#         # Display the processed frame
#         cv2.imshow("Object Detection", frame)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RedBoxDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class RedBoxDetector(Node):
    def __init__(self):
        super().__init__('red_box_detector')
        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # Publishers for detected object centers
        self.redbox_center_pub = self.create_publisher(Point, '/redbox_center', 10)
        self.handle_center_pub = self.create_publisher(Point, '/handle_center', 10)
        self.distance_pub = self.create_publisher(Float64, '/redbox_distance', 10)

        # Publisher for processed (annotated) image
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1)
        self.processed_image_pub = self.create_publisher(Image, '/Bluerov2/detected_objects/image', 10)

        # Camera and object parameters
        self.fx = 913.6264196313776
        self.fy = 687.8934159348482
        self.cx = 653.343101640157
        self.cy = 321.6068376513913
        self.focal_length = (self.fx + self.fy) / 2  # Average focal length

        # Known object dimensions (in meters)
        self.object_dimensions = {
            'redbox': {'height': 0.30, 'width': 0.16},
            'Handle': {'height': 0.10, 'width': 0.03}
        }

        self.robot_depth = None

        # Subscribe to the depth topic
        self.create_subscription(
            Float64,
            '/depth',
            self.depth_callback,
            10
        )

        # Load YOLOv5 model
        self.model = torch.hub.load('yolov5', 'custom', path='yolov5/runs/train/exp6/weights/best.pt', source='local')
        self.model.conf = 0.5  # Confidence threshold

    def depth_callback(self, msg):
        self.robot_depth = msg.data

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform detection and annotate the frame
        annotated_frame = self.detect_red_box(frame)

        # Convert the annotated frame back to a ROS Image and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.processed_image_pub.publish(annotated_msg)

    def detect_red_box(self, frame):
        # Convert BGR to RGB for YOLOv5 inference
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run inference
        results = self.model(img_rgb)
        detections = results.xyxy[0]  # [x1, y1, x2, y2, confidence, class]

        # Color definitions
        colors = {
            'redbox': (0, 255, 0),  # Green for red box
            'Handle': (255, 0, 0)   # Blue for handle
        }

        # Class name swapping
        class_mapping = {
            'redbox': 'Handle',
            'Handle': 'redbox'
        }

        object_depth = 4.5  # Adjust as needed
        delta_Z = abs(self.robot_depth - object_depth) if self.robot_depth is not None else None

        # Process each detection
        for *box, conf, cls in detections:
            x1, y1, x2, y2 = map(int, box)
            original_class_name = self.model.names[int(cls)]
            class_name = class_mapping.get(original_class_name, original_class_name)
            color = colors.get(class_name, (0, 255, 255))

            box_width_pixels = x2 - x1
            box_height_pixels = y2 - y1
            x_center = (x1 + x2) / 2.0
            y_center = (y1 + y2) / 2.0

            # Publish center points
            center_point = Point(x=float(x_center), y=float(y_center), z=0.0)
            if class_name == 'redbox':
                self.redbox_center_pub.publish(center_point)
            elif class_name == 'Handle':
                self.handle_center_pub.publish(center_point)

            # Calculate distances if dimensions are known
            dimensions = self.object_dimensions.get(class_name, None)
            if dimensions and self.focal_length:
                Z_width = (dimensions['width'] * self.focal_length) / box_width_pixels
                Z_height = (dimensions['height'] * self.focal_length) / box_height_pixels
                horizontal_distance = (Z_width + Z_height) / 2.0
                

                if delta_Z is not None:
                    total_distance = np.sqrt(horizontal_distance**2 + delta_Z**2)
                    if class_name == 'redbox':
                        distance_msg = Float64()
                        distance_msg.data = total_distance
                        self.distance_pub.publish(distance_msg)
                    distance_text = f', Dist: {total_distance:.2f}m'
                else:
                    distance_text = f', Dist: {horizontal_distance:.2f}m'
            else:
                distance_text = ''

            label = f'{class_name}: {conf:.2f}{distance_text}'

            # Draw bounding box and label on frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return frame

def main(args=None):
    rclpy.init(args=args)
    node = RedBoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
