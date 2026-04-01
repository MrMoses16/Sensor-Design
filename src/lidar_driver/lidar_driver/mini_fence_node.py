import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math

class MiniFenceNode(Node):
    def __init__(self):
        super().__init__('mini_fence_node')

        # Subscribe to the raw RPLidar C1 scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publish the raw 8-sector distance array (Useful for debugging)
        self.fence_pub = self.create_publisher(Float32MultiArray, '/mini_fence/distances', 10)

        # Publish a downsampled 8-point LaserScan for MAVROS to send to the Cube Black
        self.scan_pub = self.create_publisher(LaserScan, '/mini_fence/scan', 10)

        self.get_logger().info("Mini Fence Node started. Dividing 360-degree scan into 8 sectors.")

    def scan_callback(self, msg):
        # Initialize 8 sectors to infinity
        # Updated to match Flight Controller Clockwise (FRD) coordinates:
        # 0: Forward (0°), 1: Forward-Right (45°), 2: Right (90°), 3: Back-Right (135°)
        # 4: Back (180°), 5: Back-Left (225°), 6: Left (270°), 7: Forward-Left (315°)
        min_distances = [float('inf')] * 8

        # Iterate through all raw LiDAR points
        for i, r in enumerate(msg.ranges):
            # Filter out invalid or out-of-range readings
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue

            # Calculate the original raw angle in degrees
            angle_rad = msg.angle_min + i * msg.angle_increment
            raw_angle_deg = math.degrees(angle_rad)

            # --- THE CLOCKWISE FIX ---
            # The LiDAR spins clockwise, and MAVLink/ArduPilot wants clockwise.
            # We ONLY add 180.0 to rotate the "rear-facing" 0° point to the front.
            corrected_angle_deg = raw_angle_deg + 180.0

            # Normalize angle to strictly fall within [0, 360)
            angle_deg = corrected_angle_deg % 360.0

            # Map the angle to one of the 8 sectors
            # Offset by +22.5° so Sector 0 is perfectly centered forward (-22.5° to 22.5°)
            sector_index = int(((angle_deg + 22.5) % 360) / 45.0)

            # Update if we found a closer point in this sector
            if r < min_distances[sector_index]:
                min_distances[sector_index] = r

        # --- Publisher 1: Float32 Array ---
        fence_msg = Float32MultiArray()
        # Replace remaining infinities with range_max to signal "clear path"
        fence_msg.data = [d if d != float('inf') else msg.range_max for d in min_distances]
        self.fence_pub.publish(fence_msg)

        # --- Publisher 2: Downsampled LaserScan ---
        fence_scan = LaserScan()
        fence_scan.header = msg.header
        fence_scan.angle_min = 0.0
        # 7 sectors * 45 degrees = 315 degrees
        fence_scan.angle_max = 2 * math.pi * (7/8) 
        fence_scan.angle_increment = math.pi / 4   # 45-degree increment
        fence_scan.time_increment = 0.0
        fence_scan.scan_time = msg.scan_time
        fence_scan.range_min = msg.range_min
        fence_scan.range_max = msg.range_max
        fence_scan.ranges = fence_msg.data
        self.scan_pub.publish(fence_scan)

def main(args=None):
    rclpy.init(args=args)
    node = MiniFenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
