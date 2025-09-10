import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time


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

    # 🚗 Basic helpers
    def forward(self, speed=0.2, duration=2.0):
        self.send_twist(linear_x=speed, angular_z=0.0, duration=duration)

    def backward(self, speed=0.2, duration=2.0):
        self.send_twist(linear_x=-speed, angular_z=0.0, duration=duration)

    def turn_left(self, speed=0.5, duration=2.0, angle=0.5):
        self.send_twist(linear_x=speed, angular_z=-angle, duration=duration)

    def turn_right(self, speed=0.5, duration=2.0, angle=0.5):
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
    odom_reader = OdometryReader()  # ✅ Odometry kept

    print("[INFO] Driving in S-shape for 10 seconds...")

    total_time = 10.0
    segment_time = 2.5   # seconds per curve
    speed = 0.3
    angle = 0.6          # steering angle

    start_time = time.time()
    while time.time() - start_time < total_time:
        node.turn_left(speed=-speed, duration=segment_time, angle=angle)
        node.turn_right(speed=-speed, duration=segment_time, angle=angle)

        # 🔎 You can still check odometry updates here
        rclpy.spin_once(odom_reader, timeout_sec=0.1)
        print(f"[ODOM] X pos: {odom_reader.x_pos:.2f} m")

    node.stop(1.0)

    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
