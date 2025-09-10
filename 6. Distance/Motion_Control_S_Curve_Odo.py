import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

    # 🚗 Movement helpers
    def forward(self, speed=0.2, duration=2.0):
        self.send_twist(linear_x=speed, angular_z=0.0, duration=duration)

    def turn(self, speed=0.3, duration=2.0, angle=0.5):
        """Drive forward while turning (left if +angle, right if -angle)"""
        self.send_twist(linear_x=speed, angular_z=angle, duration=duration)

    def stop(self, duration=1.0):
        twist = Twist()
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)


def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()

    print("[INFO] Driving in X-shape for ~3 meters...")

    target_distance = 3.0   # meters in X direction
    base_speed = 0.3        # forward speed
    max_angle = math.radians(30)  # ≈ 0.52 rad

    rclpy.spin_once(odom_reader, timeout_sec=0.1)
    start_x = odom_reader.x_pos

    while rclpy.ok() and (odom_reader.x_pos - start_x) < target_distance:
        # Left diagonal stroke of "X"
        node.turn(speed=base_speed, duration=1.5, angle=max_angle)
        rclpy.spin_once(odom_reader, timeout_sec=0.1)
        print(f"[ODOM] X: {odom_reader.x_pos:.2f} m")

        # Right diagonal stroke of "X"
        node.turn(speed=base_speed, duration=1.5, angle=-max_angle)
        rclpy.spin_once(odom_reader, timeout_sec=0.1)
        print(f"[ODOM] X: {odom_reader.x_pos:.2f} m")

    node.stop(1.0)

    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
