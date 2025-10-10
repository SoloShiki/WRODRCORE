#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

# ---------------- CONFIG ----------------
LIDAR_TOPIC = '/scan_raw'
UPDATE_INTERVAL = 0.2       # 5 Hz update
MAX_BAR_LENGTH_CM = 100     # max bar length
AXIS_LIMIT_CM = 110         # x and y axis limits
ROBOT_WIDTH_CM = 12
ROBOT_LENGTH_CM = 18
BAR_WIDTH = 8               # thicker bars
CAMERA_SIZE_CM = 3          # front sensor square
ALARM_THRESHOLD_CM = 10     # alarm threshold, can be changed later

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)
        self.latest_scan = None
        self.last_update = time.time()

        # --- Setup figure with robot-centric + polar plot ---
        plt.ion()
        self.fig = plt.figure(figsize=(10,5))
        self.ax_robot = self.fig.add_subplot(1,2,1)
        self.ax_polar = self.fig.add_subplot(1,2,2, polar=True)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def compute_directional_ranges(self, scan):
        if scan is None:
            return {'Front': 0, 'Right': 0, 'Back': 0, 'Left': 0}

        directions = {
            'Front': 0.0,       # East
            'Right': -np.pi/2,  # South
            'Back': np.pi,      # West
            'Left': np.pi/2     # North
        }

        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=float) * 100
        ranges = np.clip(ranges, 0, MAX_BAR_LENGTH_CM)

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
        self.ax_robot.set_aspect('equal')
        self.ax_robot.set_title("Robot Top-Down View")

        # Draw robot as rounded rectangle
        robot_patch = patches.FancyBboxPatch(
            (-ROBOT_LENGTH_CM/2, -ROBOT_WIDTH_CM/2),
            ROBOT_LENGTH_CM, ROBOT_WIDTH_CM,
            boxstyle="Round,pad=1,rounding_size=4",
            facecolor='gray'
        )
        self.ax_robot.add_patch(robot_patch)

        # Draw front camera/sensor as small square
        cam_patch = patches.Rectangle(
            (ROBOT_LENGTH_CM/2, -CAMERA_SIZE_CM/2),
            CAMERA_SIZE_CM, CAMERA_SIZE_CM,
            facecolor='black'
        )
        self.ax_robot.add_patch(cam_patch)

        # Draw bars (lines) if finite and >0, capped at MAX_BAR_LENGTH_CM
        for direction, color, dx, dy in [
            ('Front','red',1,0),
            ('Right','blue',0,-1),
            ('Back','green',-1,0),
            ('Left','orange',0,1)
        ]:
            length = dists[direction]
            if np.isfinite(length) and length > 0:
                length = min(length, MAX_BAR_LENGTH_CM)

                # --- Alarm check ---
                if length < ALARM_THRESHOLD_CM:
                    # Mark alarm on plot (circle at end of bar)
                    alarm_x = dx * length
                    alarm_y = dy * length
                    self.ax_robot.plot(alarm_x, alarm_y, 'ro', markersize=15, markeredgecolor='black')
                    # Placeholder for custom action
                    # <<< PLACEHOLDER: do something when obstacle too close >>>
                    print(f"ALARM: {direction} distance {length:.1f} cm!")

                # Draw bars and text
                if dx != 0:
                    self.ax_robot.plot([0, dx*length],[0,0], color=color, lw=BAR_WIDTH)
                    self.ax_robot.text(dx*length + (2 if dx>0 else -2),0,
                                       f"{direction}\n{length:.1f} cm",
                                       color=color,
                                       ha='left' if dx>0 else 'right', va='center')
                if dy != 0:
                    self.ax_robot.plot([0,0],[0, dy*length], color=color, lw=BAR_WIDTH)
                    self.ax_robot.text(0, dy*length + (2 if dy>0 else -2),
                                       f"{direction}\n{length:.1f} cm",
                                       color=color, ha='center',
                                       va='bottom' if dy>0 else 'top')

        # Force axes limits **after plotting**
        self.ax_robot.set_xlim(-AXIS_LIMIT_CM, AXIS_LIMIT_CM)
        self.ax_robot.set_ylim(-AXIS_LIMIT_CM, AXIS_LIMIT_CM)

    def update_polar_plot(self, scan):
        self.ax_polar.clear()
        if scan is not None:
            angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
            ranges = np.array(scan.ranges) * 100
            ranges = np.clip(ranges, 0, MAX_BAR_LENGTH_CM)

            # Normal points
            self.ax_polar.scatter(angles, ranges, s=5, c='b', alpha=0.5)

            # Alarm points (distance < threshold)
            alarm_mask = ranges < ALARM_THRESHOLD_CM
            if np.any(alarm_mask):
                self.ax_polar.scatter(angles[alarm_mask], ranges[alarm_mask],
                                      s=50, c='r', marker='o', edgecolors='black', label='ALARM')
                # Placeholder for custom action
                # <<< PLACEHOLDER: do something when obstacle too close >>>
                for i, angle in enumerate(angles[alarm_mask]):
                    print(f"ALARM POLAR: Angle {np.degrees(angle):.1f}°, Distance {ranges[alarm_mask][i]:.1f} cm")

            self.ax_polar.set_theta_zero_location('E')
            self.ax_polar.set_theta_direction(-1)
            self.ax_polar.set_rmax(MAX_BAR_LENGTH_CM)
            self.ax_polar.set_title("LIDAR Scan (Polar View)")
            if np.any(alarm_mask):
                self.ax_polar.legend()
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
