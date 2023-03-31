import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
import numpy as np
from sklearn.cluster import DBSCAN

class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_scan_to_pointcloud')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PointCloud, '/scan_pointcloud', 10)
        self.prev_points_history = []  # Store history of previous points
        self.history_size = 50  # Number of previous point clouds to track
        self.stationary_velocity_threshold = 30.0  # Threshold to consider a cluster's average velocity as stationary
        self.cluster_eps = 0.5  # Maximum distance between two points to be considered in the same cluster
        self.cluster_min_samples = 4  # Minimum number of points in a cluster
        self.vicinity_threshold = 0.3  # Maximum distance between two points to be considered in the same vicinity
        self.action_dict = {}
        self.counter = 0
        self.first = True
        self.cluster_eps = 0.5  # Maximum distance between two points to be considered in the same cluster
        self.cluster_min_samples = 3

    def scan_callback(self, scan_msg):
        header = scan_msg.header
        points = []
        range_min = 100000
        for i, r in enumerate(scan_msg.ranges):
            if r != float('inf') and r != float('nan') and (r > scan_msg.range_min and r < scan_msg.range_max):
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                angle_key = round(angle, 3)
                print(r)
                if self.first:
                    self.action_dict[angle_key] = r

                if r > range_min:
                    continue
                if not self.first:
                    if angle_key in self.action_dict.keys():
                        if r >=  self.action_dict[angle_key] - .2:
                            #print("This happened")
                            continue
                        
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0

                point = Point32(x=x, y=y, z=z)
                points.append(point)


        X = np.array([[p.x, p.y] for p in points])

        

        self.counter += 1
        if self.counter == 5:
            self.first = False
        #print(self.counter)
        self.prev_points_history.append(points)
        # moving_centroid_points = [Point32(x=x, y=y, z=0.0) for x, y in points]
        moving_centroid_cloud_msg = PointCloud(header=header, points=points)
        self.pub.publish(moving_centroid_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy()

main()
   