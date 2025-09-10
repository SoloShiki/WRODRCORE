import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class OdometryReader(Node):
    """Subscribes to odometry topic to read X,Y position"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x, angular_z, duration):
        """Send a velocity command for a specific duration"""
        twist = Twist()
        twist.linear.x = linear_x      # forward speed
        twist.angular.z = angular_z    # steering angle for Ackermann

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)

    def stop(self, duration=1.0):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)


def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()

    print("[INFO] Driving in S-shape until 3 meters traveled...")

    target_distance = 3.0
    speed = 0.3
    max_angle = math.radians(30)  # ±30° steering
    segment_time = 1.5            # seconds per half-wave

    # Get starting X,Y
    rclpy.spin_once(odom_reader, timeout_sec=0.1)
    start_x, start_y = odom_reader.x_pos, odom_reader.y_pos

    direction = 1  # +1 left, -1 right

    while rclpy.ok():
        # Drive one S-segment
        node.send_twist(linear_x=-speed, angular_z=direction * max_angle, duration=segment_time)

        # Flip steering for next segment
        direction *= -1

        # Update odometry and check distance
        rclpy.spin_once(odom_reader, timeout_sec=0.1)
        dx = odom_reader.x_pos - start_x
        dy = odom_reader.y_pos - start_y
        distance = math.sqrt(dx*dx + dy*dy)
        print(f"[ODOM] Distance traveled: {distance:.2f} m")

        if distance >= target_distance:
            break

    node.stop(1.0)
    print("[INFO] Target distance reached, stopping.")

    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
