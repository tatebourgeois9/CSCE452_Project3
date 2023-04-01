import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.signal import medfilt

past_scans = []
WINDOW_SIZE = 5

class FilterLaser(Node):

    def __init__(self):
        super().__init__('laser_scan_to_pointcloud')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/filtered', 10)

    def scan_callback(self, msg):
    
        past_scans.append(msg)

        # If the past_scans list is longer than the window size, remove the oldest message
        if len(past_scans) > WINDOW_SIZE:
            past_scans.pop(0)

        # If there are not enough past LaserScan messages, return without filtering
        if len(past_scans) < 2:
            self.pub.publish(msg)
            return

        # Convert the current and past LaserScan messages to numpy arrays
        current_ranges = np.array(msg.ranges)
        past_ranges = np.array(past_scans[-2].ranges)

        # Calculate the absolute difference between the current and past ranges
        diff = np.abs(current_ranges - past_ranges)

        # Create a boolean mask for the non-static ranges
        mask = diff > 5

        # Apply the mask to the ranges
        ranges_filtered = np.where(mask, current_ranges, 0)

        ranges_filtered = medfilt(ranges_filtered, 1)
        # Create a new LaserScan message with the filtered ranges
        msg_filtered = LaserScan()
        msg_filtered.header = msg.header
        msg_filtered.angle_min = msg.angle_min
        msg_filtered.angle_max = msg.angle_max
        msg_filtered.angle_increment = msg.angle_increment
        msg_filtered.time_increment = msg.time_increment
        msg_filtered.scan_time = msg.scan_time
        msg_filtered.range_min = msg.range_min
        msg_filtered.range_max = msg.range_max
        msg_filtered.ranges = ranges_filtered.tolist()

        # Publish the filtered LaserScan message
        self.pub.publish(msg_filtered)


def main(args=None):
    rclpy.init(args=args)
    node = FilterLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


