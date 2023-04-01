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
        self.people_history = [0, 0, 0, 0, 0 ,0 ,0, 0]
        self.increase_count = 0

    def pointcloud_callback(self, pointcloud_msg):
        current_centroids = set()
        for point in pointcloud_msg.points:
            current_centroids.add((round(point.x, 3), round(point.y, 3), round(point.z, 3)))

        self.current_people = len(current_centroids)

        print(str(self.current_people) + " >? " + str(self.people_history[2]))
        if self.current_people > self.people_history[1]:
            self.increase_count += 1
            print("Count = ", self.increase_count)
        else:
            self.increase_count = 0

        if self.increase_count >= 5:
            self.total_unique_people = self.total_unique_people + (self.current_people - self.people_history[1])
            self.increase_count = 0

        self.people_history.pop(0)
        self.people_history.append(self.current_people)

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
