#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class CompareRotation(Node):
    def __init__(self):
        super().__init__('compare_rotation_auto_calibrate')

        # ---- State ----
        self.odom = None
        self.start_yaw_odom = None
        self.start_yaw_tf = None

        # ---- Test parameters ----
        self.target_angle = math.radians(180)   # target rotation (90°)
        self.angular_speed = 0.3               # rad/s
        # la variable de abjo es la que hay que cambiar para ajuste
        self.odom_angular_scale = 1.0          # software correction multiplier
        self.moving = True

        # ---- User measurement ----
        self.physical_angle_measured = None  # degrees you observed physically (enter after test)

        # ---- ROS interfaces ----
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        # ---- Timer ----
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("🧭 Starting angular calibration test (software-corrected)...")

    def odom_cb(self, msg):
        self.odom = msg

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def lookup_yaw_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            q = trans.transform.rotation
            return self.get_yaw_from_quaternion(q)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.moving = False
        self.get_logger().info("✅ Stopped robot at target rotation")
        self.prompt_correction_suggestion()

    def prompt_correction_suggestion(self):
        # Prompt user to enter measured physical rotation
        self.get_logger().info("\n📏 Please measure the physical rotation (degrees).")
        self.get_logger().info("Then type it below when prompted.")
        try:
            physical_deg = float(input("Enter measured physical rotation in degrees: "))
        except Exception:
            self.get_logger().warn("Invalid input — skipping correction computation.")
            return

        # Calculate correction suggestion
        odom_deg = math.degrees(self.target_angle)
        correction = odom_deg / physical_deg if physical_deg != 0 else None
        if correction is not None:
            self.get_logger().info(
                f"🧮 Suggested angular correction = {correction:.3f}"
                f" (use this as odom_angular_scale_correction in YAML)"
            )
        else:
            self.get_logger().warn("Could not compute correction — check measurement.")

    def timer_cb(self):
        if self.odom is None:
            return

        tf_yaw = self.lookup_yaw_tf()
        odom_q = self.odom.pose.pose.orientation
        odom_yaw = self.get_yaw_from_quaternion(odom_q)

        # Capture starting yaw
        if self.start_yaw_odom is None and tf_yaw is not None:
            self.start_yaw_odom = odom_yaw
            self.start_yaw_tf = tf_yaw
            self.get_logger().info("Start yaw captured.")
            return

        if self.start_yaw_odom is None:
            return

        # Compute delta angles
        delta_odom = self.normalize_angle(odom_yaw - self.start_yaw_odom) * self.odom_angular_scale
        delta_tf = None
        if tf_yaw is not None:
            delta_tf = self.normalize_angle(tf_yaw - self.start_yaw_tf)

        # Log progress
        if delta_tf is not None:
            self.get_logger().info(
                f"odom Δyaw (scaled): {math.degrees(delta_odom):.1f}° | tf Δyaw: {math.degrees(delta_tf):.1f}°"
            )
        else:
            self.get_logger().info(
                f"odom Δyaw (scaled): {math.degrees(delta_odom):.1f}° | tf Δyaw: (no tf)"
            )

        # Movement control
        if self.moving:
            if abs(delta_odom) < self.target_angle:
                twist = Twist()
                twist.angular.z = self.angular_speed
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()


def main():
    rclpy.init()
    node = CompareRotation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#this a command