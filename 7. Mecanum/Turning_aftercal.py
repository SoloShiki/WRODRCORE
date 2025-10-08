#!/usr/bin/env python3
import rclpy
from math import pi, atan2, asin, copysign, degrees
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def qua2rpy(quat):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = asin(max(-1.0, min(1.0, 2 * (w * y - x * z))))  # clamp for safety
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
        self.cmd_vel = self.create_publisher(Twist, "/controller/cmd_vel", 10)
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
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return qua2rpy(trans.transform.rotation)[2]
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def turn_angle(self, angle_deg, max_speed=0.5, tolerance_deg=3.0):
        """
        Turn the robot by a given angle (degrees).
        Positive = left (CCW), Negative = right (CW).
        """
        yaw_start = None
        # Wait for valid yaw
        while yaw_start is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            yaw_start = self.get_yaw()

        target_angle = yaw_start + (angle_deg * pi / 180.0)
        self.get_logger().info(f"Turning {angle_deg:.1f}°")

        twist = Twist()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            yaw = self.get_yaw()
            if yaw is None:
                continue

            error = normalize_angle(target_angle - yaw)

            if abs(degrees(error)) <= tolerance_deg:
                break

            # Dynamic angular speed proportional to error (min/max speed)
            speed = max(0.1, min(max_speed, abs(error)))
            twist.angular.z = copysign(speed, error)
            self.cmd_vel.publish(twist)

        # Stop the robot
        self.cmd_vel.publish(Twist())
        self.get_logger().info("✅ Turn complete")


def main():
    rclpy.init()
    node = Turner()

    # Test turns
    node.turn_angle(90)    # Turn left 90°
    rclpy.spin_once(node, timeout_sec=3.0)
    node.turn_angle(-90)   # Turn right 90°
    rclpy.spin_once(node, timeout_sec=3.0)
    node.turn_angle(180)   # Turn around

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
