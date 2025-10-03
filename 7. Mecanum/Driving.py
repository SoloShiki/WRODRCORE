#!/usr/bin/env python3
# compare_odom_tf.py
import rclpy
import time
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CompareOdomTF(Node):
    def __init__(self):
        super().__init__('compare_odom_tf')
        self.odom = None
        self.start_odom = None
        self.start_tf = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("Waiting for odom and TF...")

    def odom_cb(self, msg):
        self.odom = msg

    def lookup_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def timer_cb(self):
        if self.odom is None:
            return

        tf_trans = self.lookup_tf()
        if self.start_odom is None and tf_trans is not None:
            # set both starts simultaneously if possible
            self.start_odom = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
            self.start_tf = (tf_trans.transform.translation.x, tf_trans.transform.translation.y)
            self.get_logger().info("Start positions captured.")
            return

        if self.start_odom is None:
            return

        # odom distance
        ox = self.odom.pose.pose.position.x - self.start_odom[0]
        oy = self.odom.pose.pose.position.y - self.start_odom[1]
        odom_dist = math.sqrt(ox*ox + oy*oy)

        # tf distance (protected)
        tf_dist = None
        if tf_trans is not None and self.start_tf is not None:
            tx = tf_trans.transform.translation.x - self.start_tf[0]
            ty = tf_trans.transform.translation.y - self.start_tf[1]
            tf_dist = math.sqrt(tx*tx + ty*ty)

        if tf_dist is None:
            self.get_logger().info(f"odom: {odom_dist:.3f} m | tf: (no tf yet)")
        else:
            self.get_logger().info(f"odom: {odom_dist:.3f} m | tf: {tf_dist:.3f} m")

def main():
    rclpy.init()
    node = CompareOdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
