#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time

# ---------------- CONFIG ----------------
LIDAR_TOPIC = '/scan_raw'
UPDATE_INTERVAL = 1.0
MAX_DISTANCE_CM = 100  # max detectable distance
Y_AXIS_MAX_CM = 105    # for plotting space above bars
ROBOT_SIZE_CM = 10     # robot square size for plot

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)
        self.latest_scan = None
        self.last_update = time.time()

        # --- Setup figure for robot-centric plot ---
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.ax.set_xlim(-Y_AXIS_MAX_CM, Y_AXIS_MAX_CM)
        self.ax.set_ylim(-Y_AXIS_MAX_CM, Y_AXIS_MAX_CM)
        self.ax.set_aspect('equal')
        self.ax.set_title("Robot LIDAR Top-Down View")
        self.robot_patch = None

        self.timer = self.create_timer(0.05, self.timer_callback)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def compute_directional_ranges(self, scan):
        if scan is None:
            return {'Front': 0, 'Right': 0, 'Back': 0, 'Left': 0}

        directions = {
            'Front': 0.0,       # East
            'Right': -np.pi/2,  # South
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
            results[label] = float(np.min(valid_ranges)) if len(valid_ranges) > 0 else 0
        return results

    def update_plot(self, dists):
        self.ax.clear()
        # Draw robot as a square
        robot_half = ROBOT_SIZE_CM / 2
        self.ax.add_patch(plt.Rectangle((-robot_half, -robot_half), ROBOT_SIZE_CM, ROBOT_SIZE_CM,
                                        facecolor='gray'))

        # Draw lines from robot in 4 directions
        # Front (East)
        self.ax.plot([0, dists['Front']], [0, 0], color='red', lw=4)
        # Right (South)
        self.ax.plot([0, 0], [0, -dists['Right']], color='blue', lw=4)
        # Back (West)
        self.ax.plot([0, -dists['Back']], [0, 0], color='green', lw=4)
        # Left (North)
        self.ax.plot([0, 0], [0, dists['Left']], color='orange', lw=4)

        # Add text labels
        self.ax.text(dists['Front'] + 2, 0, f"Front\n{dists['Front']:.1f} cm", color='red')
        self.ax.text(0, -dists['Right'] - 5, f"Right\n{dists['Right']:.1f} cm", color='blue', ha='center')
        self.ax.text(-dists['Back'] - 10, 0, f"Back\n{dists['Back']:.1f} cm", color='green', ha='right')
        self.ax.text(0, dists['Left'] + 2, f"Left\n{dists['Left']:.1f} cm", color='orange', ha='center')

        self.ax.set_xlim(-Y_AXIS_MAX_CM, Y_AXIS_MAX_CM)
        self.ax.set_ylim(-Y_AXIS_MAX_CM, Y_AXIS_MAX_CM)
        self.ax.set_aspect('equal')
        self.ax.set_title("Robot LIDAR Top-Down View")
        plt.draw()
        plt.pause(0.001)

    def timer_callback(self):
        now = time.time()
        if self.latest_scan is not None and (now - self.last_update) >= UPDATE_INTERVAL:
            dists = self.compute_directional_ranges(self.latest_scan)
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
