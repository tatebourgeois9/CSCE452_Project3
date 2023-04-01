import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import math
import numpy as np
from sklearn.cluster import DBSCAN

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
    people_counter_node = PeopleCounter()
    try:
        rclpy.spin(people_counter_node)
    except KeyboardInterrupt:
        pass
    people_counter_node.destroy()

main()   