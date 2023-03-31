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

        self.action_dict = {}
        self.counter = 0
        self.first = True
        self.cluster_eps = 0.325  # Maximum distance between two points to be considered in the same cluster
        self.cluster_min_samples = 4



    def scan_callback(self, scan_msg):
        header = scan_msg.header
        points = []
        range_min = 100000
        for i, r in enumerate(scan_msg.ranges):
            if r != float('inf') and r != float('nan') and (r > scan_msg.range_min and r < scan_msg.range_max):
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                angle_key = round(angle, 3)
                if self.first:
                    self.action_dict[angle_key] = r
                if not self.first:
                    if angle_key in self.action_dict.keys():
                        if r >=  self.action_dict[angle_key] - .4:
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
            return  
        if self.first:
            return
        
        # Perform DBSCAN clustering
        X = np.array([[p.x, p.y] for p in points])
        
        dbscan = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples)
        
        print(X)

        
        if X.size == 0:
            return
        dbscan.fit(X)

        # Extract cluster labels and their centroids
        cluster_labels = set(dbscan.labels_)
        cluster_centroids = []
        for label in cluster_labels:
            if label == -1:  # Ignore noise points
                continue
            indices = np.where(dbscan.labels_ == label)[0]
            cluster_points = X[indices]
            centroid = np.mean(cluster_points, axis=0)
            cluster_centroids.append(Point32(x=centroid[0], y=centroid[1], z=0.0))

        # Create a new PointCloud message with the cluster centroids
        moving_centroid_cloud_msg = PointCloud(header=header, points=cluster_centroids)
        
        # Publish the new message
        self.pub.publish(moving_centroid_cloud_msg)


class PeopleCounter(Node):
    def __init__(self):
        super().__init__('people_counter')
        self.sub = self.create_subscription(PointCloud, '/scan_pointcloud', self.pointcloud_callback, 10)
        self.total_unique_people = 0
        self.current_people = 0
        self.last_centroids = set()

    def pointcloud_callback(self, pointcloud_msg):
        current_centroids = set()
        for point in pointcloud_msg.points:
            current_centroids.add((round(point.x, 3), round(point.y, 3), round(point.z, 3)))

        if not self.last_centroids:
            self.total_unique_people += len(current_centroids)
        else:
            new_people = current_centroids - self.last_centroids
            self.total_unique_people += len(new_people)

        self.current_people = len(current_centroids)
        self.last_centroids = current_centroids

        self.get_logger().info(f'Current people: {self.current_people}, Total unique people: {self.total_unique_people}')
        print(f'Current people: {self.current_people}, Total unique people: {self.total_unique_people}')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_to_pointcloud_node = LaserScanToPointCloud()
    people_counter_node = PeopleCounter()
    try:
        rclpy.spin(laser_scan_to_pointcloud_node)
        rclpy.spin(people_counter_node)
    except KeyboardInterrupt:
        pass
    laser_scan_to_pointcloud_node.destroy()
    people_counter_node.destroy()

main()
   