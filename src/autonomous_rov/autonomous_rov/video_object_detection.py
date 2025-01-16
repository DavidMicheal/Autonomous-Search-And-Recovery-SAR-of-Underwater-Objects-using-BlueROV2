import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import String


class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection')
        
        # Parameters
        self.declare_parameter('input_topic', '/bluerov2/camera/image_raw')
        self.declare_parameter('output_topic', 'detected_frames')
        self.declare_parameter('detection_topic', 'object_centroids')  # New topic for object centroids

        
        # Set up YOLO model
        self.model = YOLO('yolov8n.pt')  # Load the YOLOv8 model
        self.bridge = CvBridge()
        
        detected_objects = []
        # ROS 2 subscriptions and publications
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('input_topic').get_parameter_value().string_value,
            self.frame_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            self.get_parameter('output_topic').get_parameter_value().string_value,
            10
        )
        self.detection_publisher = self.create_publisher(
            String,
            self.get_parameter('detection_topic').get_parameter_value().string_value,
            10
        )
        
        self.get_logger().info("YOLO Object Detection Node initialized")

    def frame_callback(self, msg):
        # Convert ROS 2 Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


        detected_objects = []
        # Perform object detection
        results = self.model(frame)
        
                #  result contains the bounding boxes and other details for each detected object
        for result in results:
            for detection in result.boxes:
                # Extract class id, confidence score, and bounding box coordinates
                class_id = int(detection.cls)  # Object class ID
                confidence = float(detection.conf)  # Confidence score
                x1, y1, x2, y2 = detection.xyxy[0]  # Bounding box coordinates (top-left and bottom-right)

                # Calculate the center of the bounding box (x, y position of the object)
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)

                # Draw the bounding box and label on the frame
                label = f"{self.model.names[class_id]}: {confidence:.2f}"
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (x_center, y_center), 5, (0, 0, 255), -1)
                cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)   
        
                detected_objects.append({
                        'class': label,
                        'centroid': {'x': x_center, 'y': y_center}
                })
        if detected_objects:
            # Create a string message with object class and centroid info
            detection_data = ", ".join([f"{{'class': '{obj['class']}', 'centroid': {obj['centroid']}}}" for obj in detected_objects])
            detection_msg = String(data=detection_data)
            self.detection_publisher.publish(detection_msg)
# Convert back to ROS 2 Image message and publish
        output_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(output_msg)
        self.get_logger().info("Published detected frame with bounding boxes")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
