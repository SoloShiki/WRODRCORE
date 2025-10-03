#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time, math


class OdometryReader(Node):
    """Subscribes to odometry topic to read X and Y position"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = None
        self.y_pos = None
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

    def stop(self, duration=1.0):
        """Stop robot safely"""
        twist = Twist()
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

    # ------------------- Distance-based movement -------------------
    def move_distance(self, target_distance, speed=0.2, odom_topic='/odom'):
        """Move a specific distance in meters using odometry feedback."""
        odom_sub = OdometryReader(topic=odom_topic)

        # Wait until odometry is available
        while odom_sub.x_pos is None or odom_sub.y_pos is None:
            rclpy.spin_once(odom_sub, timeout_sec=0.1)

        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos
        print(f"Starting position: x={start_x:.2f}, y={start_y:.2f}")

        twist = Twist()
        twist.linear.x = speed

        rate_hz = 20   # publish at 20 Hz
        dt = 1.0 / rate_hz

        while rclpy.ok():
            rclpy.spin_once(odom_sub, timeout_sec=0.01)
            if odom_sub.x_pos is None or odom_sub.y_pos is None:
                continue

            dx = odom_sub.x_pos - start_x
            dy = odom_sub.y_pos - start_y
            dist = math.sqrt(dx**2 + dy**2)

            print(f"Distance traveled: {dist:.3f} m")

            if dist >= target_distance:
                break

            # continuously publish velocity command
            self.publisher_.publish(twist)
            time.sleep(dt)

        self.stop()
        print(f"✅ Target reached: {dist:.2f} m traveled")

    # Wrappers for forward/backward
    def move_forward(self, distance, speed=0.2, odom_topic='/odom'):
        self.move_distance(target_distance=distance, speed=abs(speed), odom_topic=odom_topic)

    def move_backward(self, distance, speed=0.2, odom_topic='/odom'):
        self.move_distance(target_distance=distance, speed=-abs(speed), odom_topic=odom_topic)


def main():
    rclpy.init()
    node = CmdVelPublisher()

    # Example usage
    node.move_forward(distance=2.0, speed=0.3)   # Move forward 2 meters
    time.sleep(0.5)
    node.move_backward(distance=1.0, speed=0.3)  # Move backward 1 meter

    node.stop(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
