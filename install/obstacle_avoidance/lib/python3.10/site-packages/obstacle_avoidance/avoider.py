import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 1.50
        self.turn_speed = 0.7
        self.forward_speed = 0.3

    def scan_callback(self, msg):
        front_ranges = []
        left_ranges = []
        right_ranges = []

        for i, r in enumerate(msg.ranges):
            if r == float('inf') or r <= 0.05:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # FRONT = ±180 degrees (~0.52 rad)
            if -0.1 < angle < 0.1:
                front_ranges.append(r)
            # LEFT = ±90 degrees (~1.57 rad)
            elif 0.15 < angle < 1.6:
                left_ranges.append(r)
            # RIGHT = ±90 degrees (~1.57 rad)
            elif -1.6 < angle < -0.15:
                right_ranges.append(r)

        twist = Twist()

        if len(front_ranges) == 0:
            return

        avg_dist = sum(front_ranges) / len(front_ranges)

        self.get_logger().info(f"Front dist: {avg_dist:.2f} Left: {len(left_ranges)} Right: {len(right_ranges)}")

        if avg_dist < self.safe_distance:
            twist.linear.x = 0.0
            if(len(left_ranges) > 0 and len(right_ranges) > 0):
                avg_left = sum(left_ranges) / len(left_ranges)
                avg_right = sum(right_ranges) / len(right_ranges)
                if abs(avg_left - avg_right) < 10:
                    twist.linear.x = -self.forward_speed
                    twist.angular.z = 0.0
                
                elif avg_left < avg_right:
                    twist.angular.z = self.turn_speed
                else:
                    twist.angular.z = -self.turn_speed
        else:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()