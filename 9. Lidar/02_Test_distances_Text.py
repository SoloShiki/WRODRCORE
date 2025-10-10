#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import matplotlib.pyplot as plt
import time

# Configuration
LIDAR_TOPIC = '/scan'
DIRECTION_WINDOW_DEG = 15   # angular window around each direction
UPDATE_INTERVAL = 1.0       # seconds between updates

def wrap_to_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)
        self.latest_scan = None
        self.last_update = time.time()

        # Initialize live plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(5,4))
        self.bars = self.ax.bar(['N', 'E', 'S', 'W'], [0, 0, 0, 0])
        self.ax.set_ylim(0, 5)
        self.ax.set_ylabel("Distance (m)")
        self.ax.set_title("Cardinal Obstacle Distances")
        plt.show(block=False)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def compute_directional_ranges(self, scan, window_deg=DIRECTION_WINDOW_DEG):
        if scan is None:
            return {'N': np.inf, 'S': np.inf, 'E': np.inf, 'W': np.inf}

        # Cardinal directions in robot frame (E=0, N=pi/2, W=pi, S=-pi/2)
        directions = {'E': 0.0, 'N': math.pi/2, 'W': math.pi, 'S': -math.pi/2}
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=float)

        results = {}
        half_window = math.radians(window_deg)

        for label, dir_angle in directions.items():
            angle_diff = np.array([wrap_to_pi(a - dir_angle) for a in angles])
            mask = np.abs(angle_diff) <= half_window
            valid_ranges = ranges[mask]
            valid_ranges = valid_ranges[np.isfinite(valid_ranges)]
            if len(valid_ranges) > 0:
                results[label] = float(np.min(valid_ranges))
            else:
                results[label] = np.inf
        return results

    def update_plot(self, dists):
        labels = ['N', 'E', 'S', 'W']
        vals = [dists.get(l, np.inf) for l in labels]
        display_vals = [v if np.isfinite(v) else 5.0 for v in vals]

        self.ax.clear()
        bars = self.ax.bar(labels, display_vals, color='skyblue')
        self.ax.set_ylim(0, 5)
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_title('Cardinal Obstacle Distances')

        for rect, v in zip(bars, vals):
            h = rect.get_height()
            label = f"{v:.2f} m" if np.isfinite(v) else "no hit"
            self.ax.text(rect.get_x() + rect.get_width()/2.0, h + 0.05, label, ha='center', va='bottom')

        plt.draw()
        plt.pause(0.001)

    def timer_callback(self):
        now = time.time()
        if self.latest_scan is not None and (now - self.last_update) >= UPDATE_INTERVAL:
            dists = self.compute_directional_ranges(self.latest_scan)
            self.get_logger().info(f"LIDAR distances: {dists}")
            self.update_plot(dists)
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
