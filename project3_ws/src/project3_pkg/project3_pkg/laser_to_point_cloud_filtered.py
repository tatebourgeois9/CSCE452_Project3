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

    def scan_callback(self, scan_msg):
        header = scan_msg.header
        points = []
        range_min = 100000
        for i, r in enumerate(scan_msg.ranges):
            if r != float('inf') and r != float('nan') and (r > scan_msg.range_min and r < scan_msg.range_max):
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                angle_key = round(angle, 4)
                if self.first:
                    # if r < range_min:
                    #     range_min = r
                    self.action_dict[angle_key] = r
                if r > range_min:
                    continue
                if not self.first:
                    if angle_key in self.action_dict.keys():
                        if r >= .9 * self.action_dict[angle_key]:
                            #print("This happened")
                            continue
                        
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0

                point = Point32(x=x, y=y, z=z)
                points.append(point)
        self.counter += 1
        if self.counter == 5:
            self.first = False
        #print(self.counter)
        self.prev_points_history.append(points)
        # moving_centroid_points = [Point32(x=x, y=y, z=0.0) for x, y in points]
        moving_centroid_cloud_msg = PointCloud(header=header, points=points)
        self.pub.publish(moving_centroid_cloud_msg)
    #     if len(self.prev_points_history) > self.history_size:
    #         self.prev_points_history.pop(0)  # Remove the oldest point cloud

    #     # Cluster the points using DBSCAN
    #     X = np.array([[p.x, p.y] for p in points])

    #     # Remove any NaN values from X
    #     X = X[~np.isnan(X).any(axis=1)]

    #     dbscan = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples)
    #     dbscan.fit(X)

    #     labels = dbscan.labels_
    #     unique_labels = np.unique(labels)
    #     moving_centroids = []

    #     for label in unique_labels:
    #         if label != -1:
    #             cluster_points = X[labels == label]
    #             cluster_velocities = []

    #             # Calculate the velocities of each point in the cluster
    #             for i, point in enumerate(cluster_points):
    #                 if len(self.prev_points_history) >= 2:
    #                     prev_point = self.prev_points_history[-2][i]
    #                     dx = point[0] - prev_point.x
    #                     dy = point[1] - prev_point.y
    #                     dt = scan_msg.scan_time
    #                     vx = dx / dt
    #                     vy = dy / dt
    #                     cluster_velocities.append((vx, vy))

    #             # Calculate the average velocity of the cluster
    #             if cluster_velocities:
    #                 avg_velocity = np.mean(cluster_velocities, axis=0)
    #                 magnitude = np.linalg.norm(avg_velocity)
    #                 if magnitude > self.stationary_velocity_threshold:
    #                     centroid = np.mean(cluster_points, axis=0)
    #                     # Check if the centroid is close to any previously detected moving clusters
    #                     is_close_to_previous = False
    #                     for prev_centroid in moving_centroids:
    #                         if np.linalg.norm(centroid - prev_centroid) < self.vicinity_threshold:
    #                             is_close_to_previous = True
    #                             break
    #                     # Only add the centroid to the list of moving centroids if it is not close to any previous ones
    #                     if not is_close_to_previous:
    #                         moving_centroids.append(centroid)

    # # Publish the filtered point cloud containing only the centroids of the moving clusters
    #     moving_centroid_points = [Point32(x=x, y=y, z=0.0) for x, y in moving_centroids]
    #     moving_centroid_cloud_msg = PointCloud(header=header, points=moving_centroid_points)
    #     self.pub.publish(moving_centroid_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy()

main()
   