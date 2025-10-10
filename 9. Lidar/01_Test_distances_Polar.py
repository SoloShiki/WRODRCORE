#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import matplotlib.pyplot as plt
import time

# ---------------- CONFIG ----------------
LIDAR_TOPIC = '/scan_raw'
DIRECTION_WINDOW_DEG = 15
UPDATE_INTERVAL = 1.0      # seconds between display updates
MAX_DISTANCE_CM = 100      # max detectable distance (1 m = 100 cm)
Y_AXIS_MAX_CM = 105        # y-axis max for display (space above bars)

def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)
        self.latest_scan = None
        self.last_update = time.time()

        # --- Setup figure with polar + bar chart ---
        plt.ion()
        self.fig = plt.figure(figsize=(10, 5))
        self.ax_polar = self.fig.add_subplot(1, 2, 1, polar=True)
        self.ax_bars = self.fig.add_subplot(1, 2, 2)

        # Pre-create bar chart placeholders
        self.bar_labels = ['Left', 'Front', 'Right', 'Back']  # relabeled
        self.bar_vals = [0, 0, 0, 0]
        self.bars = self.ax_bars.bar(self.bar_labels, self.bar_vals)
        self.ax_bars.set_ylim(0, Y_AXIS_MAX_CM)
        self.ax_bars.set_ylabel("Distance (cm)")
        self.ax_bars.set_title("Obstacle Distances")

        plt.tight_layout()
        plt.show(block=False)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def compute_directional_ranges(self, scan, window_deg=DIRECTION_WINDOW_DEG):
        if scan is None:
            return {'Front': np.inf, 'Right': np.inf, 'Back': np.inf, 'Left': np.inf}

        # Keep original LIDAR directions: E=0, N=pi/2, W=pi, S=-pi/2
        directions = {
            'Front': 0.0,       # East
            'Right': -math.pi/2, # South
            'Back': math.pi,    # West
            'Left': math.pi/2   # North
        }

        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=float) * 100       # convert to cm
        ranges = np.clip(ranges, 0, MAX_DISTANCE_CM)           # cap at 100 cm

        results = {}
        half_window = math.radians(window_deg)

        for label, dir_angle in directions.items():
            angle_diff = np.array([wrap_to_pi(a - dir_angle) for a in angles])
            mask = np.abs(angle_diff) <= half_window
            valid_ranges = ranges[mask]
            valid_ranges = valid_ranges[np.isfinite(valid_ranges)]
            results[label] = float(np.min(valid_ranges)) if len(valid_ranges) > 0 else np.inf
        return results

    def update_plot(self, scan, dists):
        # ---- POLAR PLOT ----
        self.ax_polar.clear()
        if scan is not None:
            angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
            ranges = np.array(scan.ranges) * 100
            ranges = np.clip(ranges, 0, MAX_DISTANCE_CM)
            self.ax_polar.scatter(angles, ranges, s=5, c='b', alpha=0.5)
            self.ax_polar.set_title("LIDAR Scan (Polar View)")
            self.ax_polar.set_theta_zero_location('E')
            self.ax_polar.set_theta_direction(-1)
            self.ax_polar.set_rmax(MAX_DISTANCE_CM)
        else:
            self.ax_polar.text(0.5, 0.5, "Waiting for scan...", ha='center', va='center')

        # ---- BAR CHART ----
        self.ax_bars.clear()
        labels = ['Left', 'Front', 'Right', 'Back']
        vals = [dists.get(l, np.inf) for l in labels]
        display_vals = [v if np.isfinite(v) else MAX_DISTANCE_CM for v in vals]  # bars capped at 100 cm
        bars = self.ax_bars.bar(labels, display_vals, color='skyblue')
        self.ax_bars.set_ylim(0, Y_AXIS_MAX_CM)
        self.ax_bars.set_ylabel('Distance (cm)')
        self.ax_bars.set_title('Obstacle Distances')

        for rect, v in zip(bars, vals):
            h = rect.get_height()
            label = f"{v:.1f} cm" if np.isfinite(v) else "no hit"
            self.ax_bars.text(rect.get_x() + rect.get_width()/2.0, h + 1, label, ha='center', va='bottom')

        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)

    def timer_callback(self):
        now = time.time()
        if self.latest_scan is not None and (now - self.last_update) >= UPDATE_INTERVAL:
            dists = self.compute_directional_ranges(self.latest_scan)
            self.get_logger().info(f"LIDAR distances (cm): {dists}")
            self.update_plot(self.latest_scan, dists)
            self.last_update = now


def main(args=None):
    rclpy.init(args=args)
    node = LidarTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close()


if __name__ == '__main__':
    main()
