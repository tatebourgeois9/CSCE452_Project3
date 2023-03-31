import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
import math
class PeopleCounter(Node):
    def __init__(self):
        super().__init__('people_counter')
        self.sub = self.create_subscription(PointCloud, '/scan_pointcloud', self.pointcloud_callback, 10)
        self.total_unique_people = 0
        self.current_people = 0
        self.last_people = []
        self.matching_distance = 0.5

    def pointcloud_callback(self, pointcloud_msg):
        current_people = []
        for point in pointcloud_msg.points:
            current_people.append((point.x, point.y, point.z))

        # Match the current people with the last detected people
        matched_indices = set()
        for person in self.last_people:
            min_distance = float('inf')
            min_index = None
            for i, current_person in enumerate(current_people):
                if i in matched_indices:
                    continue
                distance = math.sqrt((current_person[0] - person[0])**2 + (current_person[1] - person[1])**2)
                if distance < min_distance:
                    min_distance = distance
                    min_index = i

            if min_distance < self.matching_distance:
                matched_indices.add(min_index)

        new_people_count = len(current_people) - len(matched_indices)
        self.total_unique_people += new_people_count
        self.current_people = len(current_people)
        self.last_people = current_people

        self.get_logger().info(f'Current people: {self.current_people}, Total unique people: {self.total_unique_people}')

    
def main(args=None):
    rclpy.init(args=args)
    people_counter_node = PeopleCounter()
    try:
        rclpy.spin(people_counter_node)
    except KeyboardInterrupt:
        pass
    people_counter_node.destroy()

main()   