import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
import numpy as np
from sklearn.cluster import KMeans


class LaserScanToPointCloud(Node):

    def __init__(self):
        super().__init__('laser_scan_to_pointcloud')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PointCloud, '/scan_pointcloud', 10)

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud
        header = scan_msg.header
        print(header)
        points = []
        for i, r in enumerate(scan_msg.ranges):
            if r != float('inf') and r != float('nan'):
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
            if abs(p.x) > 0.01 or abs(p.y) > 0.01:
                filtered_points.append(p)
        filtered_cloud_msg = PointCloud(header=header, points=filtered_points)

        # Cluster the filtered points using K-means
        if len(filtered_points) > 0:
            X = np.array([[p.x, p.y] for p in filtered_points])
            n_clusters = 15  # Set the number of clusters
            kmeans = KMeans(n_clusters=n_clusters)  # Initialize KMeans
            kmeans.fit(X)
            labels = kmeans.labels_

            # Calculate the centroid for each cluster
            centroids = kmeans.cluster_centers_
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

