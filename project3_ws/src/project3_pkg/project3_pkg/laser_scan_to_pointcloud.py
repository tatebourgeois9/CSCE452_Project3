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

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud
        header = scan_msg.header
        points = []
        for i, r in enumerate(scan_msg.ranges):
            if r != float('inf') and not math.isnan(r):  # Updated condition
                # Calculate the angle of the ray
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                # Convert polar coordinates to Cartesian coordinates
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0

                point = Point32(x=x, y=y, z=z)
                # Add the point to the list of points
                points.append(point)
        cloud_msg = PointCloud(header=header, points=points)

        # Filter out points that are not moving
        filtered_points = []
        for p in points:
            if abs(p.x) > 0.3 or abs(p.y) > 0.3:
                filtered_points.append(p)
        filtered_cloud_msg = PointCloud(header=header, points=filtered_points)

        # Cluster the filtered points using DBSCAN
        if len(filtered_points) > 0:
            X = np.array([[p.x, p.y] for p in filtered_points])
            dbscan = DBSCAN(eps=0.5, min_samples=3)  # Customize eps and min_samples as needed
            dbscan.fit(X)
            labels = dbscan.labels_
            unique_labels = np.unique(labels)

            # Calculate the centroid for each cluster
            centroids = []
            for label in unique_labels:
                if label != -1:
                    cluster_points = X[labels == label]
                    centroid = np.mean(cluster_points, axis=0)
                    centroids.append(centroid)
            centroid_points = [Point32(x=x, y=y, z=0.0) for x, y in centroids]

            # Create PointCloud message for centroids
            centroid_cloud_msg = PointCloud(header=header, points=centroid_points)

            # Publish the centroid point cloud message
            self.pub.publish(centroid_cloud_msg)


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


