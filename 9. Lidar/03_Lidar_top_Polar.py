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
UPDATE_INTERVAL = 1.0
MAX_DISTANCE_CM = 100
Y_AXIS_MAX_CM = 105
ROBOT_SIZE_CM = 10

def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)
        self.latest_scan = None
        self.last_update = time.time()

        # --- Setup figure with polar + robot-centric plot ---
        plt.ion()
        self.fig = plt.figure(figsize=(10, 5))
        self.ax_robot = self.fig.add_subplot(1, 2, 1)
        self.ax_polar = self.fig.add_subplot(1, 2, 2, polar=True)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def compute_directional_ranges(self, scan):
        if scan is None:
            return {'Front': 0, 'Right': 0, 'Back': 0, 'Left': 0}

        directions = {
            'Front': 0.0,        # East
            'Right': -np.pi/2,   # South
            'Back': np.pi,       # West
            'Left': np.pi/2      # North
        }

        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=float) * 100
        ranges = np.clip(ranges, 0, MAX_DISTANCE_CM)

        results = {}
        window_rad = np.deg2rad(15)
        for label, dir_angle in directions.items():
            mask = np.abs(np.array([wrap_to_pi(a - dir_angle) for a in angles])) <= window_rad
            valid_ranges = ranges[mask]
            valid_ranges = valid_ranges[np.isfinite(valid_ranges)]
            results[label] = float(np.min(valid_ranges)) if len(valid_ranges) > 0 else 0
        return results

    def update_robot_plot(self, dists):
        self.ax_robot.clear()
        self.ax_robot.set_xlim(-Y_AXIS_MAX_CM, Y_AXIS_MAX_CM)
        self.ax_robot.set_ylim(-Y_AXIS_MAX_CM, Y_AXIS_MAX_CM)
        self.ax_robot.set_aspect('equal')
        self.ax_robot.set_title("Robot Top-Down View")

        # Draw robot square
        half = ROBOT_SIZE_CM / 2
        self.ax_robot.add_patch(plt.Rectangle((-half, -half), ROBOT_SIZE_CM, ROBOT_SIZE_CM, facecolor='gray'))

        # Draw lines only if distance > 0
        if np.isfinite(dists['Front']) and dists['Front'] > 0:
            self.ax_robot.plot([0, dists['Front']], [0, 0], color='red', lw=4)
            self.ax_robot.text(dists['Front']+2, 0, f"Front\n{dists['Front']:.1f} cm", color='red')
        if np.isfinite(dists['Right']) and dists['Right'] > 0:
            self.ax_robot.plot([0, 0], [0, -dists['Right']], color='blue', lw=4)
            self.ax_robot.text(0, -dists['Right']-5, f"Right\n{dists['Right']:.1f} cm", color='blue', ha='center')
        if np.isfinite(dists['Back']) and dists['Back'] > 0:
            self.ax_robot.plot([0, -dists['Back']], [0, 0], color='green', lw=4)
            self.ax_robot.text(-dists['Back']-10, 0, f"Back\n{dists['Back']:.1f} cm", color='green', ha='right')
        if np.isfinite(dists['Left']) and dists['Left'] > 0:
            self.ax_robot.plot([0, 0], [0, dists['Left']], color='orange', lw=4)
            self.ax_robot.text(0, dists['Left']+2, f"Left\n{dists['Left']:.1f} cm", color='orange', ha='center')

    def update_polar_plot(self, scan):
        self.ax_polar.clear()
        if scan is not None:
            angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
            ranges = np.array(scan.ranges) * 100
            ranges = np.clip(ranges, 0, MAX_DISTANCE_CM)
            self.ax_polar.scatter(angles, ranges, s=5, c='b', alpha=0.5)
            self.ax_polar.set_theta_zero_location('E')
            self.ax_polar.set_theta_direction(-1)
            self.ax_polar.set_rmax(MAX_DISTANCE_CM)
            self.ax_polar.set_title("LIDAR Scan (Polar View)")
        else:
            self.ax_polar.text(0.5, 0.5, "Waiting for scan...", ha='center', va='center')

    def timer_callback(self):
        now = time.time()
        if self.latest_scan is not None and (now - self.last_update) >= UPDATE_INTERVAL:
            dists = self.compute_directional_ranges(self.latest_scan)
            self.update_robot_plot(dists)
            self.update_polar_plot(self.latest_scan)
            plt.draw()
            plt.pause(0.001)
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
