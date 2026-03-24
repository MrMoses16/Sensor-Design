#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
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

        # 2. Load YOLO Weights using Torch Hub
        try:
            package_share = get_package_share_directory('camera_driver')
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
            self.get_logger().error("Failed to load YOLOv5: {e}")
            self.get_logger().error(traceback.format_exc())
            return

        # 3. Define the GStreamer Pipeline String
        # Using v4l2src for your eMeet C970L on /dev/video0
        self.pipeline_str = (
            "v4l2src device=/dev/video0 extra-controls=\"s,exposure_auto=1,exposure_absolute=500,brightness=164\" ! "
            "video/x-raw, width=640, height=480, framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink name=sink emit-signals=True sync=false drop=True"
        )

        # # thermal camera
        # self.pipeline_str = (
        #     "v4l2src device=/dev/video0 extra-controls=\"s,exposure_auto=1,exposure_absolute=500,brightness=96\" ! "
        #     "video/x-raw, format=NV12, width=640, height=512, framerate=30/1 ! "
        #     "videoconvert ! "
        #     "video/x-raw, format=BGR ! "
        #     "appsink name=sink emit-signals=True sync=false drop=True"
        # )

        # 4. Set up GStreamer Pipeline
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
        annotated_frame = np.copy(results.render()[0]) # Draws boxes on the frame

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

def main(args=None):
    rclpy.init(args=args)
    node = GstYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
