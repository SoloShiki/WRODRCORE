#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan # New
import time
import numpy as np
import random
from collections import deque
import matplotlib.pyplot as plt
import math
import heapq # For A*

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # meters per grid cell
LIDAR_TOPIC = '/scan_raw'
OBSTACLE_THRESHOLD_CM = 20.0 # Lidar range under this is considered an obstacle
LIDAR_WINDOW_RAD = math.pi/8 # +/- 22.5 degrees

# ---------------- Utility Functions ----------------
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    """Reads robot position (x,y) and yaw from /odom"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.subscription = self.create_subscription(Odometry, topic, self.odom_callback, 10)

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

# ---------------- Lidar Reader and Map Updater ----------------
class LidarMapUpdater(Node):
    """Reads Lidar and provides methods to update the map"""
    def __init__(self, maze, grid_size):
        super().__init__('lidar_map_updater')
        self.maze = maze # Reference to the main maze map
        self.grid_size = grid_size
        self.latest_scan = None
        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def _world_to_grid(self, x, y):
        # Assumes robot starts at (0,0) which is grid (0,0)
        # Grid indexing: (row, col) = (y, x)
        row = int(round(y / self.grid_size))
        col = int(round(x / self.grid_size))
        return np.clip(row, 0, self.maze.shape[0]-1), np.clip(col, 0, self.maze.shape[1]-1)

    def update_map(self, odom_sub):
        if self.latest_scan is None:
            return False # No update

        # Get robot position and orientation
        rx, ry, ryaw = odom_sub.x_pos, odom_sub.y_pos, odom_sub.yaw
        map_changed = False

        angles = self.latest_scan.angle_min + np.arange(len(self.latest_scan.ranges)) * self.latest_scan.angle_increment

        # Consider each scan ray
        for angle_idx, scan_range_m in enumerate(self.latest_scan.ranges):
            if not np.isfinite(scan_range_m) or scan_range_m == 0.0:
                continue

            scan_range_cm = scan_range_m * 100.0

            # Only check for new obstacles nearby
            if scan_range_cm < OBSTACLE_THRESHOLD_CM:
                # Global angle of the obstacle relative to the map frame
                global_angle = ryaw + angles[angle_idx]

                # Calculate obstacle world coordinates (relative to robot start)
                obs_x = rx + scan_range_m * math.cos(global_angle)
                obs_y = ry + scan_range_m * math.sin(global_angle)

                # Convert to grid coordinates
                obs_row, obs_col = self._world_to_grid(obs_x, obs_y)

                # Update map if a non-obstacle cell is now an obstacle (1)
                if self.maze[obs_row, obs_col] == 0:
                    self.maze[obs_row, obs_col] = 1
                    map_changed = True

        return map_changed

# ---------------- Command Velocity Publisher ----------------
class CmdVelPublisher(Node):
    """Publishes Twist commands to control the Mecanum chassis"""
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=0.1):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

    def stop(self, duration=0.1):
        self.send_twist(0.0, 0.0, 0.0, duration)

    # ---------------- Smooth Move in grid direction (patched) ----------------
    def move_direction(self, dx, dy, odom_sub, distance=GRID_SIZE, speed=0.25, min_speed=0.05):
        """
        Move the robot in the grid direction (dx, dy) with a cosine accel/decel profile.
        Ensures a small minimum speed to overcome deadband/static friction.
        dx = row change (-1 up, +1 down)
        dy = col change (-1 left, +1 right)

        NOTE: preserves the linear_x/linear_y mapping used in your dynamic version:
              linear_x corresponds to 'dy' and linear_y corresponds to 'dx'.
        """
        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos
        moved = 0.0

        accel_distance = max(1e-3, min(0.2 * distance, 0.05))
        decel_distance = accel_distance

        rate_sleep = 0.05  # control loop period (s)

        while rclpy.ok() and moved < distance:
            # update odom
            rclpy.spin_once(odom_sub)

            dx_pos = odom_sub.x_pos - start_x
            dy_pos = odom_sub.y_pos - start_y
            moved = math.hypot(dx_pos, dy_pos)

            # cosine-based smooth speed factor
            if moved < accel_distance:
                phase = moved / accel_distance if accel_distance > 0 else 1.0
                factor = 0.5 * (1 - math.cos(math.pi * phase))
            elif moved > distance - decel_distance:
                phase = (distance - moved) / decel_distance if decel_distance > 0 else 0.0
                phase = max(min(phase, 1.0), 0.0)
                factor = 0.5 * (1 - math.cos(math.pi * phase))
            else:
                factor = 1.0

            # enforce a small non-zero factor so we actually start moving
            if speed > 0:
                min_factor = min(1.0, max(0.0, min_speed / speed))
            else:
                min_factor = 0.0
            factor = max(factor, min_factor)

            current_speed = speed * factor

            # build twist and publish directly (non-blocking)
            twist = Twist()
            # Keep the mapping your dynamic code used:
            twist.linear.x = current_speed * dy
            twist.linear.y = current_speed * dx
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

            time.sleep(rate_sleep)

        # final stop
        self.stop()

    # ---------------- Optional rotation to yaw ----------------
    def rotate_to_yaw(self, target_yaw, odom_sub, yaw_tol=0.05, max_speed=0.3):
        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            err = math.atan2(math.sin(target_yaw - odom_sub.yaw), math.cos(target_yaw - odom_sub.yaw))
            if abs(err) <= yaw_tol:
                break
            twist = Twist()
            twist.angular.z = max(-max_speed, min(max_speed, err))
            self.publisher_.publish(twist)
            time.sleep(0.05)
        self.stop()


# ---------------- Maze Generation ----------------
def generate_maze(x_size=None, y_size=None, num_walls=None):
    if x_size is None:
        x_size = random.randint(6,12)
    if y_size is None:
        y_size = random.randint(6,12)
    maze = np.zeros((x_size, y_size), dtype=int)
    if num_walls is None:
        num_walls = int(x_size * 1.5)
    wall_indices = random.sample(range(x_size*y_size), num_walls)
    for idx in wall_indices:
        row = idx // y_size
        col = idx % y_size
        maze[row, col] = 1
    start = (0,0)
    goal  = (x_size-1, y_size-1)
    maze[start] = 0
    maze[goal] = 0
    return maze, start, goal

# ---------------- A* Path Planning (Replacement for BFS) ----------------
def a_star_path(maze, start, goal):
    """A* search for the shortest path"""
    rows, cols = maze.shape
    # f = g + h, where g is cost from start, h is heuristic (Manhattan distance)
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    # Priority queue: (f_score, node)
    frontier = [(f_score[start], start)]
    parent = {}

    directions =
