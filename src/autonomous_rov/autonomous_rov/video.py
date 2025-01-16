# # #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import cv2
# import gi
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image

# gi.require_version('Gst', '1.0')
# from gi.repository import Gst

# class Controller(Node):
#     """BlueROV2 video capture and publisher class

#     Attributes:
#         port (int): Video UDP port
#         video_source (string): Udp source ip and port
#         video_codec (string): Source h264 parser
#         video_decode (string): Transform YUV (12bits) to BGR (24bits)
#         video_sink_conf (string): Sink configuration
#         video_pipe (object): GStreamer top-level pipeline
#         video_sink (object): GStreamer sink element
#         publisher_ (Publisher): ROS 2 Image publisher
#         bridge (CvBridge): CvBridge for converting images
#     """

#     def __init__(self):
#         super().__init__("video")

#         self.declare_parameter("port", 5600)

#         self.port = self.get_parameter("port").value
#         self._frame = None
#         self.video_source = 'udpsrc port={}'.format(self.port)
#         self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
#         self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
#         self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

#         self.video_pipe = None
#         self.video_sink = None

#         # Initialize CvBridge and publisher
#         self.bridge = CvBridge()
#         self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

#         Gst.init()

#         self.run()

#         # Start update loop
#         self.create_timer(0.01, self.update)

#     def start_gst(self, config=None):
#         """Start GStreamer pipeline and sink."""
#         if not config:
#             config = [
#                 'videotestsrc ! decodebin',
#                 '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
#                 '! appsink'
#             ]

#         command = ' '.join(config)
#         self.video_pipe = Gst.parse_launch(command)
#         self.video_pipe.set_state(Gst.State.PLAYING)
#         self.video_sink = self.video_pipe.get_by_name('appsink0')

#     @staticmethod
#     def gst_to_opencv(sample):
#         """Transform GStreamer sample to OpenCV image."""
#         buf = sample.get_buffer()
#         caps = sample.get_caps()
#         array = np.ndarray(
#             (
#                 caps.get_structure(0).get_value('height'),
#                 caps.get_structure(0).get_value('width'),
#                 3
#             ),
#             buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
#         return array

#     def frame(self):
#         """Get the current frame."""
#         return self._frame

#     def frame_available(self):
#         """Check if a frame is available."""
#         return self._frame is not None

#     def run(self):
#         """Set up and start the GStreamer pipeline."""
#         self.start_gst([
#             self.video_source,
#             self.video_codec,
#             self.video_decode,
#             self.video_sink_conf
#         ])

#         self.video_sink.connect('new-sample', self.callback)

#     def callback(self, sink):
#         """Callback function for new GStreamer samples."""
#         sample = sink.emit('pull-sample')
#         new_frame = self.gst_to_opencv(sample)
#         self._frame = new_frame
#         return Gst.FlowReturn.OK

#     def update(self):
#         """Update method called periodically."""
#         if not self.frame_available():
#             return

#         frame = self.frame()
#         width = int(1920 / 1.5)
#         height = int(1080 / 1.5)
#         dim = (width, height)
#         img = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

#         # self.draw_gui(img, width, height)

#         # Convert OpenCV image to ROS 2 image message
#         image_message = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')

#         # Publish the image message
#         self.publisher_.publish(image_message)

#         # Optionally display the image using OpenCV
#         cv2.imshow('BlueROV2 Camera', img)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             self.destroy_node()

#     def draw_gui(self, img, width, height):
#         """Draw GUI elements on the image."""
#         img = cv2.rectangle(img, (0, height - 100), (520, height), (0, 0, 0), -1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = Controller()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import gi
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class Controller(Node):
    """BlueROV2 video capture and publisher class

    Attributes:
        port (int): Video UDP port
        video_source (string): Udp source ip and port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_sink_conf (string): Sink configuration
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): GStreamer sink element
        publisher_ (Publisher): ROS 2 Image publisher
        bridge (CvBridge): CvBridge for converting images
    """

    def __init__(self):
        super().__init__("video")

        self.declare_parameter("port", 5600)
        self.port = self.get_parameter("port").value
        self._frame = None

        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        # Initialize CvBridge and publisher
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

        Gst.init()

        self.run()

        # Start update loop
        self.create_timer(0.01, self.update)

    def start_gst(self, config=None):
        """Start GStreamer pipeline and sink."""
        if not config:
            config = [
                'videotestsrc ! decodebin',
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform GStreamer sample to OpenCV image."""
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8
        )
        return array

    def frame(self):
        """Get the current frame."""
        return self._frame

    def frame_available(self):
        """Check if a frame is available."""
        return self._frame is not None

    def run(self):
        """Set up and start the GStreamer pipeline."""
        self.start_gst([
            self.video_source,
            self.video_codec,
            self.video_decode,
            self.video_sink_conf
        ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        """Callback function for new GStreamer samples."""
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame
        return Gst.FlowReturn.OK

    def update(self):
        """Update method called periodically."""
        if not self.frame_available():
            return

        frame = self.frame()
        width = int(1920 / 1.5)
        height = int(1080 / 1.5)
        dim = (width, height)
        img = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

        # Convert OpenCV image to ROS 2 image message
        image_message = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')

        # Publish the image message
        self.publisher_.publish(image_message)

        # Removed cv2.imshow() and cv2.waitKey() to run headless

    def draw_gui(self, img, width, height):
        """Draw GUI elements on the image (optional if needed)."""
        img = cv2.rectangle(img, (0, height - 100), (520, height), (0, 0, 0), -1)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    