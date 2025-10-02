#!/usr/bin/env python3
import rclpy
import time
from math import pi, atan2, asin, copysign, degrees, sqrt
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def qua2rpy(quat):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = asin(2 * (w * y - x * z))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw


def normalize_angle(angle):
    """Keep angle in range [-pi, pi]."""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


class Turner(Node):
    def __init__(self, name="turner"):
        super().__init__(name)
        self.cmd_vel = self.create_publisher(Twist, "/controller/cmd_vel", 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"

    def get_yaw(self):
        """Return yaw angle (radians) from odom->base_footprint TF."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1),
            )
            return qua2rpy(trans.transform.rotation)[2]
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF Exception: could not get orientation")
            return None

    def turn_angle(self, angle_deg, speed=0.5, tolerance_deg=2.0):
        """
        Turn the robot by a given angle (degrees).
        Positive = left (CCW), Negative = right (CW).
        """
        yaw_start = self.get_yaw()
        if yaw_start is None:
            self.get_logger().error("No odometry orientation available.")
            return

        # Target angle in radians
        target_angle = yaw_start + (angle_deg * pi / 180.0)

        move_cmd = Twist()
        direction = copysign(1.0, angle_deg)
        move_cmd.angular.z = direction * abs(speed)

        self.get_logger().info(f"Turning {angle_deg:.1f} degrees at {speed:.2f} rad/s")

        while rclpy.ok():
            yaw = self.get_yaw()
            if yaw is None:
                continue

            error = normalize_angle(target_angle - yaw)

            if abs(degrees(error)) <= tolerance_deg:
                break

            self.cmd_vel.publish(move_cmd)
            time.sleep(0.05)

        # Stop
        self.cmd_vel.publish(Twist())
        self.get_logger().info("Turn complete.")


def main():
    rclpy.init()
    node = Turner("turner")

    # Example tests
    node.turn_angle(90)   # turn left 90 degrees
    time.sleep(1)
    node.turn_angle(-90)  # turn right 90 degrees
    time.sleep(1)
    node.turn_angle(180)  # turn around

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
