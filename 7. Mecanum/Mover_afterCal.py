#!/usr/bin/env python3
import rclpy
import time
from math import sqrt, pow, copysign
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class Mover(Node):
    def __init__(self, name="mover"):
        super().__init__(name)

        # publisher for velocity commands
        self.cmd_vel = self.create_publisher(Twist, "/controller/cmd_vel", 1)

        # tf listener for odometry
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # frame names (match your robot’s TF tree)
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"

    def get_position(self):
        """Get current transform odom->base_footprint"""
        try:
            return self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF Exception: could not get odometry transform")
            return None

    def move_distance(self, distance, speed=0.2, tolerance=0.02):
        """
        Move forward/backward by distance (m).
        Positive = forward, negative = backward.
        """
        # Get starting position
        start_pos = self.get_position()
        if start_pos is None:
            self.get_logger().error("No odometry available, cannot move.")
            return

        x_start = start_pos.transform.translation.x
        y_start = start_pos.transform.translation.y

        # Set up move command
        move_cmd = Twist()
        move_cmd.linear.x = copysign(speed, distance)

        self.get_logger().info(f"Moving {distance:.2f} m at {speed:.2f} m/s")

        while rclpy.ok():
            current = self.get_position()
            if current is None:
                continue

            dx = current.transform.translation.x - x_start
            dy = current.transform.translation.y - y_start
            traveled = sqrt(pow(dx, 2) + pow(dy, 2))

            if traveled + tolerance >= abs(distance):
                break

            self.cmd_vel.publish(move_cmd)
            time.sleep(0.05)

        # stop
        self.cmd_vel.publish(Twist())
        self.get_logger().info("Movement complete.")

def main():
    rclpy.init()
    node = Mover("mover")

    # Example test movements
    node.move_distance(1.0, speed=0.2)   # forward 1 m
    time.sleep(1)
    node.move_distance(-0.5, speed=0.2)  # backward 0.5 m

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
