import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


def quaternion_to_yaw(q):
    """Convert quaternion to yaw (rotation around Z axis)."""
    import tf_transformations
    _, _, yaw = tf_transformations.euler_from_quaternion([
        q.x, q.y, q.z, q.w
    ])
    return yaw


class OdometryReader(Node):
    """Subscribes to odometry topic to read X, Y, and yaw."""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x=0.0, angular_z=0.0, duration=0.1):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)

    # ------------------- Distance-based movement -------------------
    def move_distance(self, target_distance, speed=0.2, odom_topic='/odom'):
        odom_sub = OdometryReader(topic=odom_topic)
        rclpy.spin_once(odom_sub)  
        start_x = odom_sub.x

        print(f"Starting X position: {start_x:.2f} m")
        while rclpy.ok() and abs(odom_sub.x - start_x) < target_distance:
            self.send_twist(linear_x=speed, angular_z=0.0, duration=0.1)
            rclpy.spin_once(odom_sub)

        self.stop()
        print(f"Target reached: {odom_sub.x - start_x:.2f} m traveled")

    # ------------------- Angle-based turns -------------------
    def turn_angle(self, angle_deg, angular_speed=0.5, odom_topic='/odom'):
        """Turn robot by specific angle (deg) using odometry yaw."""
        odom_sub = OdometryReader(topic=odom_topic)
        rclpy.spin_once(odom_sub)

        start_yaw = odom_sub.yaw
        target_yaw = start_yaw + math.radians(angle_deg)

        print(f"Turning {angle_deg}°, start={math.degrees(start_yaw):.1f}° target={math.degrees(target_yaw):.1f}°")

        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            error = target_yaw - odom_sub.yaw
            error = math.atan2(math.sin(error), math.cos(error))  # normalize
            if abs(error) < math.radians(2.0):  # within 2° tolerance
                break
            self.send_twist(linear_x=0.0, angular_z=math.copysign(angular_speed, error), duration=0.1)

        self.stop()
        print("Turn complete")


def main():
    rclpy.init()
    node = CmdVelPublisher()

    # Move forward 2 m
    node.move_distance(target_distance=2.0, speed=0.3)

    # Turn 90° left
    node.turn_angle(angle_deg=90, angular_speed=0.5)

    # Move backward 1 m
    node.move_distance(target_distance=1.0, speed=-0.3)

    # Turn 90° right
    node.turn_angle(angle_deg=-90, angular_speed=0.5)

    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
