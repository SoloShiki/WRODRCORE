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
        self.final_odom = None      # Stores final raw Odom position
        self.final_tf = None        # Stores final raw TF position
        self.logged_final = False   # New flag to ensure final values are logged once
        self.odom_scale = 1.00      # apply your calibration factor here
        
        # se uso 1.00 for odom_scale, 1.45 hace que el robot avance perfectamente asi
        # que se hizo permanente en el archivo YAML , el anterior era 1.39
        # asi que el nuevo es 1.39 X 1.45 =2.02       

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
        
        # LOG FINAL VALUES HERE
        if not self.logged_final:
            self._log_final_values()
        
        self.moving = False
        self.get_logger().info("✅ Stopped robot at target distance")

    def _log_final_values(self):
        """Helper to log final positions and distances. Called by stop_robot."""
        if self.final_odom is None or self.final_tf is None:
            self.get_logger().warn("Could not log final values: missing data.")
            return

        self.logged_final = True

        # --- FINAL RAW VALUES ---
        final_odom_x, final_odom_y = self.final_odom
        final_tf_x, final_tf_y = self.final_tf
        
        self.get_logger().info("--- FINAL POSITIONS (Raw) ---")
        self.get_logger().info(f"Odom Final (X, Y): ({final_odom_x:.4f}, {final_odom_y:.4f})")
        self.get_logger().info(f"TF Final (X, Y): ({final_tf_x:.4f}, {final_tf_y:.4f})")
        
        # --- FINAL CALCULATED DISTANCE TRAVELED ---
        
        # Odom Distance (Scaled by self.odom_scale)
        ox = self.final_odom[0] - self.start_odom[0]
        oy = self.final_odom[1] - self.start_odom[1]
        odom_dist = math.sqrt(ox * ox + oy * oy) * self.odom_scale

        # TF Distance (Unscaled)
        tx = self.final_tf[0] - self.start_tf[0]
        ty = self.final_tf[1] - self.start_tf[1]
        tf_dist = math.sqrt(tx * tx + ty * ty)

        self.get_logger().info("--- FINAL DISTANCE TRAVELED ---")
        self.get_logger().info(f"Odom Distance (Scaled): {odom_dist:.4f} m")
        self.get_logger().info(f"TF Distance (Unscaled): {tf_dist:.4f} m")


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
            
            # --- Initial Logging (Including odom_scale) ---
            self.get_logger().info("--- INITIAL POSITIONS (Raw) ---")
            self.get_logger().info(f"Odom Start (X, Y): ({self.start_odom[0]:.4f}, {self.start_odom[1]:.4f})")
            self.get_logger().info(f"TF Start (X, Y): ({self.start_tf[0]:.4f}, {self.start_tf[1]:.4f})")
            self.get_logger().info(f"Odom Scale (self.odom_scale) being used: {self.odom_scale:.4f}") # <--- ADDED LINE
            self.get_logger().info("Start positions captured.")
            return

        if self.start_odom is None:
            return

        # odom distance (