import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
import math


class OdometryReader(Node):
    """Subscribes to odometry topic to read X position"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x


class LidarReader(Node):
    """Subscribes to LiDAR topic to read front obstacle distance"""
    def __init__(self, topic='/scan'):
        super().__init__('lidar_reader')
        self.min_distance_front = float('inf')
        self.subscription = self.create_subscription(
            LaserScan,
            topic,
            self.scan_callback,
            10
        )

    def scan_callback(self, msg: LaserScan):
        # Take ~20° slice in front
        ranges = list(msg.ranges)
        n = len(ranges)
        front_indices = ranges[n // 2 - 10 : n // 2 + 10]
        # Replace inf values with a large number
        front_indices = [r if not math.isinf(r) else 10.0 for r in front_indices]
        self.min_distance_front = min(front_indices)


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x, angular_z, duration):
        """Send a velocity command for a specific duration"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)

    # ------------------- Time-based movement -------------------
    def forward(self, speed=0.2, duration=2.0):
        self.send_twist(linear_x=speed, angular_z=0.0, duration=duration)

    def backward(self, speed=0.2, duration=2.0):
        self.send_twist(linear_x=-speed, angular_z=0.0, duration=duration)

    def turn_left(self, speed=0.5, duration=2.0, angle=5.0):
        self.send_twist(linear_x=speed, angular_z=angle, duration=duration)

    def turn_right(self, speed=0.5, duration=2.0, angle=5.0):
        self.send_twist(linear_x=speed, angular_z=-angle, duration=duration)

    # ------------------- Stop -------------------
    def stop(self, duration=1.0):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)

    # ------------------- Distance-based movement -------------------
    def move_distance(self, target_distance, speed=0.2,
                      odom_topic='/odom', lidar_topic='/scan'):
        """Move forward/backward a distance in meters using odometry + LiDAR"""
        odom_sub = OdometryReader(topic=odom_topic)
        lidar_sub = LidarReader(topic=lidar_topic)

        rclpy.spin_once(odom_sub)  # get initial position
        start_x = odom_sub.x_pos

        print(f"Starting X position: {start_x:.2f} m")
        while rclpy.ok() and abs(odom_sub.x_pos - start_x) < abs(target_distance):
            # 🚧 Check for obstacles only when moving forward
            if speed > 0 and lidar_sub.min_distance_front < 0.3:
                print("⚠️ Obstacle detected! Stopping early.")
                self.stop()
                return

            self.send_twist(linear_x=speed, angular_z=0.0, duration=0.1)
            rclpy.spin_once(odom_sub)   # update odometry
            rclpy.spin_once(lidar_sub)  # update lidar

        self.stop()
        print(f"Target reached: {odom_sub.x_pos - start_x:.2f} m traveled")

    def move_forward(self, distance, speed=0.2, odom_topic='/odom'):
        """Move forward a distance in meters"""
        self.move_distance(target_distance=distance, speed=abs(speed),
                           odom_topic=odom_topic)

    def move_backward(self, distance, speed=0.2, odom_topic='/odom'):
        """Move backward a distance in meters"""
        self.move_distance(target_distance=-distance, speed=-abs(speed),
                           odom_topic=odom_topic)


def main():
    rclpy.init()
    node = CmdVelPublisher()

    # Example usage with distance-based movement + LiDAR check
    node.move_forward(distance=2.0, speed=0.3)   # Move forward 2 meters
    time.sleep(0.2)
    node.move_backward(distance=1.0, speed=0.3)  # Move backward 1 meter

    # Stop after actions
    node.stop(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
