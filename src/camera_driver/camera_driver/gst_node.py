#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
import os
import cv2
import numpy as np
import torch
import traceback

# GStreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

class GstYoloNode(Node):
    def __init__(self):
        super().__init__('gst_yolo_node')
        
        # 1. Initialize ROS 2 Publisher and Bridge
        self.publisher_ = self.create_publisher(Image, '/camera/yolo_stream', 10)
        self.bridge = CvBridge()

        # 2. Enable GPS coordinates capture
        self.current_lat = 0.0
        self.current_lon = 0.0

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile_sensor_data
        )

        # 2. Declare the parameter with a default value
        self.declare_parameter('camera_type', 'rgb')

        # 3. Read the parameter
        camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        # 4. Load YOLO Weights using Torch Hub
        try:
            package_share = get_package_share_directory('camera_driver')

            if camera_type == 'thermal':
                weights_path = os.path.join(package_share, 'weights', 'thermal_best.pt')
            else:
                weights_path = os.path.join(package_share, 'weights', 'rgb_best.pt')
            
            # Load the model
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path, force_reload=False)
            self.model.names[0] = 'person'

            # GPU optimization
            if torch.cuda.is_available():
                self.model.cuda()
                self.get_logger().info("YOLOv5 running on GPU (Cuda)")
            else:
                self.get_logger().info("Cuda not available, running on CPU")

            
            self.get_logger().info(f"Loaded YOLOv5 weights from: {weights_path} successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv5: {e}")
            self.get_logger().error(traceback.format_exc())
            return

        # 5. Define the GStreamer Pipeline String
        if camera_type == 'thermal':
            # Using v4l2src for Boston Interface RHP-BOS-VPC3-IF thermal camera
            self.pipeline_str = (
                "v4l2src device=/dev/video0 ! " #extra-controls=\"s,exposure_auto=1,exposure_absolute=500,brightness=128\" ! "
                "video/x-raw, format=NV12, width=640, height=512, framerate=60/1 ! "
                "videorate ! "
                "video/x-raw, framerate=10/1 ! "
                "nvvidconv ! "
                "video/x-raw, format=BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink name=sink emit-signals=True sync=false max-buffers=1 drop=True"
            )
        else:
            # Using v4l2src for eMeet Webcam C970L 
            self.pipeline_str = (
                "v4l2src device=/dev/video0 extra-controls=\"s,exposure_auto=1,exposure_absolute=5,gain=0,brightness=128\" ! "
                
                "video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! "
                "videorate ! "
                "video/x-raw, framerate=15/1 ! "
                "nvvidconv ! "
                "video/x-raw, format=BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink name=sink emit-signals=True sync=false max-buffers=1 drop=True"
            )

        # 6. Set up GStreamer Pipeline
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.connect("new-sample", self.on_new_sample)

        # Start streaming
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info("GStreamer Pipeline Started.")

    def on_new_sample(self, sink):
        self.get_logger().debug("Received new sample from GStreamer")
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        # Get the buffer data
        buf = sample.get_buffer()
        caps = sample.get_caps()
        
        # Extract metadata (width/height) from caps
        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")

        # Convert Gst.Buffer to NumPy array (BGR format)
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        frame = np.ndarray(
            shape=(height, width, 3),
            dtype=np.uint8,
            buffer=map_info.data
        )

        # --- YOLO INFERENCE ---
        # Run YOLO on the frame
        results = self.model(frame)

        # Create a clean copy of the frame to draw on
        annotated_frame = frame.copy()

        # Iterate through every detected object in the frame
        for *box, conf, cls in results.xyxy[0]:
            # Convert bounding box tensor to integers
            x1, y1, x2, y2 = map(int, box)
            
            # Create your custom label using the ArduPilot GPS data
            class_name = self.model.names[int(cls)]
            label = f"{class_name}: {self.current_lat:.5f}, {self.current_lon:.5f}"
            
            # Draw the bounding box (Green, thickness 2)
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw a solid background for the text so it's readable
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated_frame, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)
            
            # Print the custom label on the frame
            cv2.putText(annotated_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # --- PUBLISH TO ROS ---
        try:
            # 1. Convert the BGR numpy array to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            
            # 2. ADD THE TIMESTAMP HERE
            # This gives the image the current 'robot time'
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_link"
            
            # 3. Publish the message with the correct time
            self.publisher_.publish(ros_image)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to publish image: {e}")

        # Clean up
        buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def gps_callback(self, msg):
        self.get_logger().info(f"Callback Fired! Lat: {self.current_lat}, Lon: {self.current_lon}")
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

def main(args=None):
    rclpy.init(args=args)
    node = GstYoloNode()
    
    # Use a MultiThreadedExecutor to prevent GStreamer/YOLO from starving ROS callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
