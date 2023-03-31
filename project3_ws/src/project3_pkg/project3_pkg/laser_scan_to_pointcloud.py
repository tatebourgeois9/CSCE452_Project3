import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
import numpy as np

class LaserScanToPointCloud(Node):

    def __init__(self):
        super().__init__('laser_scan_to_pointcloud')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PointCloud, '/scan_pointcloud', 10)

        # Parameters for filtering and smoothing
        self.min_range = 0.2
        self.max_range = 5.0
        self.moving_average_size = 5
        self.range_diff_threshold = 0.2
        self.buffer_zone_threshold = 0.05
        self.angle_diff_threshold = 0.05
        self.angle_decimal_places = 3
        self.scan_history = []
        self.initial_ranges = None

    def scan_callback(self, scan_msg):
        # Apply moving average filter
        self.scan_history.append(scan_msg.ranges)
        if len(self.scan_history) > self.moving_average_size:
            self.scan_history.pop(0)
        filtered_ranges = np.mean(self.scan_history, axis=0)

        # Initialize initial_ranges with the first scan
        if self.initial_ranges is None:
            self.initial_ranges = np.copy(filtered_ranges)
            return

        # Replace NaN and Inf values in both arrays with a large value
        large_value = 1e6
        filtered_ranges = np.nan_to_num(filtered_ranges, nan=large_value, posinf=large_value, neginf=large_value)
        initial_ranges = np.nan_to_num(self.initial_ranges, nan=large_value, posinf=large_value, neginf=large_value)

        # Convert LaserScan to PointCloud
        header = scan_msg.header
        points = []

        range_diffs = np.abs(np.subtract(filtered_ranges, initial_ranges))
        changed_angles = np.where(range_diffs > self.range_diff_threshold)[0]

        for i in changed_angles:
            r = filtered_ranges[i]
            if self.min_range < r < self.max_range:
                # Calculate the angle of the ray
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                # Round the angle
                angle = round(angle, self.angle_decimal_places)

                # Convert polar coordinates to Cartesian coordinates
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0

                initial_r = initial_ranges[i]
                initial_x = initial_r * math.cos(angle)
                initial_y = initial_r * math.sin(angle)

                # Calculate the distance between the initial point and the current point
                distance = math.sqrt((x - initial_x) ** 2 + (y - initial_y) ** 2)

                # Calculate the angle difference
                angle_diff = abs(angle - (scan_msg.angle_min + i * scan_msg.angle_increment))

                # Check if the current point is within the buffer zone of the initial point and if the angle difference is significant
                if distance > self.buffer_zone_threshold and angle_diff > self.angle_diff_threshold:
                    point = Point32(x=x, y=y, z=z)
                    # Add the point to the list of points
                    points.append(point)

        cloud_msg = PointCloud(header=header, points=points)

        # Publish the point cloud messages
        self.pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
