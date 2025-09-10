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
        """Send velocity command for a specific duration"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        print(f"[CMD] linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")  # Debug

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)

    def stop(self, duration=1.0):
        """Stop the robot"""
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

    target_distance = 3.0   # meters
    speed = 0.3             # m/s
    max_angle = math.radians(30)  # ±30 degrees steering
    segment_time = 1.5      # seconds per segment

    # Get starting odometry
    rclpy.spin_once(odom_reader, timeout_sec=0.1)
    start_x = odom_reader.x_pos
    print(f"[INFO] Starting X = {start_x:.2f} m")

    direction = 1  # start turning right (+)

    while rclpy.ok():
        # Drive in alternating S pattern
        if direction == 1:
            node.send_twist(linear_x=-speed, angular_z=max_angle, duration=segment_time)
        else:
            node.send_twist(linear_x=-speed, angular_z=-max_angle, duration=segment_time)
            
        

        # Update odometry
        rclpy.spin_once(odom_reader, timeout_sec=0.1)
        traveled = abs(odom_reader.x_pos - start_x)
        print(f"[ODOM] X={odom_reader.x_pos:.2f}, traveled={traveled:.2f} m")

        if traveled >= target_distance:
            print(f"[INFO] Target distance {target_distance} m reached.")
            break

        # Alternate direction
        direction *= -1

    node.stop(1.0)

    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
