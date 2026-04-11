import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

class FusionVisualizer(Node):
    def __init__(self):
        super().__init__('fusion_visualizer')
        self.bridge = CvBridge()
        
        self.laptop_ip = "192.168.0.93"
        self.port = 8554
        
        gst_out = (
            "appsrc ! video/x-raw, format=BGR ! queue ! "
            "videoconvert ! video/x-raw, format=I420 ! "
            "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
            "rtph264pay config-interval=1 ! "
            f"udpsink host={self.laptop_ip} port={self.port} sync=false"
        )
        self.out = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, 20.0, (640, 480), True)

        # Buffers to hold data
        self.latest_scan = None

        # 1. Independent Subscribers
        self.create_subscription(Image, '/camera/yolo_stream', self.image_cb, 10)

        self.get_logger().info("Fusion Visualizer Started: Sending stream to laptop...")

    def scan_cb(self, msg):
        # Just update the scan data whenever it arrives
        self.latest_scan = msg

    def image_cb(self, image_msg):

        try:
            # 1. Convert ROS Image to OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            frame = cv2.resize(frame, (640, 480))

            # 4. Stream the "Fused" frame
            self.out.write(frame)
            
        except Exception as e:
            self.get_logger().error(f"Error in fusion: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FusionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.out.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
