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
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/yolo_stream', self.image_cb, 10)

        self.get_logger().info("Fusion Visualizer Started: Sending stream to laptop...")

    def scan_cb(self, msg):
        # Just update the scan data whenever it arrives
        self.latest_scan = msg

    def image_cb(self, image_msg):
        #if self.latest_scan is None:
        #   self.get_logger().info("Waiting for first Lidar scan...", once=True)
        #    return

        try:
            # 1. Convert ROS Image to OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            frame = cv2.resize(frame, (640, 480))

            # 2. Draw Radar HUD
            #overlay = frame.copy()
            #radar_center = (540, 380)
            #cv2.circle(overlay, radar_center, 80, (0, 0, 0), -1)
            #cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

            # 3. Project Lidar Points (using self.latest_scan)
            #for i, dist in enumerate(self.latest_scan.ranges):
            #    if self.latest_scan.range_min < dist < 5.0:
            #        angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
            #        x = int(radar_center[0] + (dist * np.cos(angle) * 15))
            #        y = int(radar_center[1] - (dist * np.sin(angle) * 15))
            #        cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)

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
