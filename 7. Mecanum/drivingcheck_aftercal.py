#!/usr/bin/env python3
# compare_odom_tf_and_move_calibrated.py
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class CompareOdomTF(Node):
    def __init__(self):
        super().__init__('compare_odom_tf')

        # ---- state ----
        self.odom = None
        self.start_odom = None
        self.start_tf = None
        self.target_distance = 1.0  # meters
        self.speed = 0.2            # m/s
        self.moving = True          # control flag
        #self.odom_scale = 0.64      # apply your calibration factor here
        self.odom_scale = 0.406      # apply your calibration factor here
        self.odom_scale = 1.45      # apply your calibration factor here


        # ---- subscribers/publishers ----
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        # ---- timers ----
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("Starting calibrated movement + odom vs TF comparison...")

    def odom_cb(self, msg):
        self.odom = msg

    def lookup_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.moving = False
        self.get_logger().info("✅ Stopped robot at target distance")

    def timer_cb(self):
        if self.odom is None:
            return

        tf_trans = self.lookup_tf()

        # capture starting positions
        if self.start_odom is None and tf_trans is not None:
            self.start_odom = (
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y
            )
            self.start_tf = (
                tf_trans.transform.translation.x,
                tf_trans.transform.translation.y
            )
            self.get_logger().info("Start positions captured.")
            return

        if self.start_odom is None:
            return

        # odom distance (scaled)
        ox = self.odom.pose.pose.position.x - self.start_odom[0]
        oy = self.odom.pose.pose.position.y - self.start_odom[1]
        odom_dist = math.sqrt(ox * ox + oy * oy) * self.odom_scale

        # tf distance (unscaled)
        tf_dist = None
        if tf_trans is not None and self.start_tf is not None:
            tx = tf_trans.transform.translation.x - self.start_tf[0]
            ty = tf_trans.transform.translation.y - self.start_tf[1]
            tf_dist = math.sqrt(tx * tx + ty * ty)

        # logging
        if tf_dist is None:
            self.get_logger().info(f"odom (scaled): {odom_dist:.3f} m | tf: (no tf yet)")
        else:
            self.get_logger().info(f"odom (scaled): {odom_dist:.3f} m | tf: {tf_dist:.3f} m")

        # movement logic
        if self.moving:
            if odom_dist < self.target_distance:
                twist = Twist()
                twist.linear.x = self.speed
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()


def main():
    rclpy.init()
    node = CompareOdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
