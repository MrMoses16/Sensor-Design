#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
import os
import cv2
import numpy as np
import torch
import traceback
import json
import time

# GStreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

class UnifiedCameraNode(Node):
    def __init__(self):
        super().__init__('unified_camera_node')
        
        # 1. Initialize ROS 2 Publisher and Bridge
        self.publisher_ = self.create_publisher(Image, '/camera/yolo_stream', 10)
        self.bridge = CvBridge()

        # 2. Parameters
        self.declare_parameter('camera_type', 'rgb')      # 'rgb' or 'thermal'
        self.declare_parameter('camera_node', 'drone')    # 'surveyor' (logs data) or 'drone' (stream only)

        self.camera_type = self.get_parameter('camera_type').value
        self.camera_node_mode = self.get_parameter('camera_node').value

        # 3. GPS & JSON Setup (Only if in surveyor mode)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.detections_log = []
        self.detection_id_counter = 0
        self.last_log_time = 0.0
        self.cooldown_seconds = 2.0 

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile_sensor_data
        )

        if self.camera_node_mode == 'surveyor':
            self.get_logger().info("MODE: SURVEYOR - GPS Logging Enabled.")
        else:
            self.get_logger().info("MODE: DRONE - Streaming Only (Logging Disabled).")

        # 4. Load YOLO Model
        self.setup_yolo()

        # 5. Define and Start GStreamer Pipeline
        self.setup_gstreamer()

    def setup_yolo(self):
        try:
            package_share = get_package_share_directory('camera_driver')
            weights = 'thermal_best.pt' if self.camera_type == 'thermal' else 'rgb_best.pt'
            weights_path = os.path.join(package_share, 'weights', weights)
            
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path, force_reload=False)
            self.model.names[0] = 'person'

            if torch.cuda.is_available():
                self.model.cuda()
                self.get_logger().info("YOLOv5 running on GPU (Cuda)")
            
            self.get_logger().info(f"Weights: {weights_path} loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv5: {e}")
            self.get_logger().error(traceback.format_exc())

    def setup_gstreamer(self):
        if self.camera_type == 'thermal':
            # Pipeline for Thermal Camera
            self.pipeline_str = (
                "v4l2src device=/dev/video0 ! "
                "video/x-raw, format=NV12, width=640, height=512, framerate=60/1 ! "
                "videorate ! video/x-raw, framerate=10/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink name=sink emit-signals=True sync=false max-buffers=1 drop=True"
            )
        else:
            # Pipeline for RGB Camera
            self.pipeline_str = (
                "v4l2src device=/dev/video0 ! " #extra-controls=\"s,exposure_auto=1,exposure_absolute=5,gain=0,brightness=60\" ! "
                "video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! "
                "videorate ! video/x-raw, framerate=15/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink name=sink emit-signals=True sync=false max-buffers=1 drop=True"
            )

        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f"GStreamer Pipeline ({self.camera_type}) Started.")

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        frame = np.ndarray(shape=(height, width, 3), dtype=np.uint8, buffer=map_info.data)

        # --- YOLO INFERENCE ---
        results = self.model(frame)
        annotated_frame = frame.copy()

        people_count = 0
        highest_confidence = 0.0

        for *box, conf, cls in results.xyxy[0]:
            if int(cls) == 0:  # If class is 'person'
                people_count += 1
                conf_val = float(conf)
                if conf_val > highest_confidence:
                    highest_confidence = conf_val

                x1, y1, x2, y2 = map(int, box)
                label = f"{conf_val:.2f}: {self.current_lat:.5f}, {self.current_lon:.5f}"
                
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated_frame, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)
                cv2.putText(annotated_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # --- JSON LOGGING (Surveyor Mode Only) ---
        if self.camera_node_mode == 'surveyor' and people_count > 0: # and self.current_lat != 0.0:
            current_time = time.time()
            if (current_time - self.last_log_time) > self.cooldown_seconds:
                log_entry = {
                    "id": self.detection_id_counter,
                    "lat": self.current_lat,
                    "lon": self.current_lon,
                    "sensor_type": self.camera_type.upper(),
                    "confidence": round(highest_confidence, 2),
                    "people_count": people_count
                }
                self.detections_log.append(log_entry)
                self.detection_id_counter += 1
                self.last_log_time = current_time
                self.get_logger().info(f"Target Logged! Count: {people_count}")

        # --- PUBLISH IMAGE ---
        try:
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_link"
            self.publisher_.publish(ros_image)
        except Exception as e:
            self.get_logger().warn(f"Publish error: {e}")

        buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def save_json_log(self):
        if self.camera_node_mode == 'surveyor' and self.detections_log:
            # This ensures it always goes to your home folder, regardless of where you launch from
            home_path = os.path.expanduser('~')
            filename = os.path.join(home_path, "surveyor_detections.json")
        
            try:
                with open(filename, 'w') as f:
                    json.dump(self.detections_log, f, indent=4)
                self.get_logger().info(f"SUCCESS: Saved to {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save JSON: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedCameraNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_json_log()
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
