#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
import numpy as np
import random
from collections import deque
import matplotlib.pyplot as plt
import math
import heapq
from threading import Thread

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # meters per grid cell
LIDAR_TOPIC = '/scan_raw'
OBSTACLE_THRESHOLD_CM = 20.0  # Lidar range under this is considered an obstacle
LIDAR_WINDOW_RAD = math.pi / 8  # +/- 22.5 degrees


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
        self.subscription = self.create_subscription(
            Odometry, topic, self.odom_callback, 10
        )

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)


# ---------------- Lidar Reader and Map Updater ----------------
class LidarMapUpdater(Node):
    """Reads Lidar and updates the map"""

    def __init__(self, maze, grid_size):
        super().__init__('lidar_map_updater')
        self.maze = maze
        self.grid_size = grid_size
        self.latest_scan = None
        self.subscription = self.create_subscription(
            LaserScan, LIDAR_TOPIC, self.scan_callback, 10
        )

    def scan_callback(self, msg):
        self.latest_scan = msg

    def _world_to_grid(self, x, y):
        row = int(round(y / self.grid_size))
        col = int(round(x / self.grid_size))
        return np.clip(row, 0, self.maze.shape[0] - 1), np.clip(col, 0, self.maze.shape[1] - 1)

    def update_map(self, odom_sub):
        if self.latest_scan is None:
            return False
        rx, ry, ryaw = odom_sub.x_pos, odom_sub.y_pos, odom_sub.yaw
        map_changed = False

        angles = self.latest_scan.angle_min + np.arange(len(self.latest_scan.ranges)) * self.latest_scan.angle_increment
        for angle_idx, scan_range_m in enumerate(self.latest_scan.ranges):
            if not np.isfinite(scan_range_m) or scan_range_m == 0.0:
                continue
            scan_range_cm = scan_range_m * 100.0
            if scan_range_cm < OBSTACLE_THRESHOLD_CM:
                global_angle = ryaw + angles[angle_idx]
                obs_x = rx + scan_range_m * math.cos(global_angle)
                obs_y = ry + scan_range_m * math.sin(global_angle)
                obs_row, obs_col = self._world_to_grid(obs_x, obs_y)
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
        while time.time() < end_time and rclpy.ok():
            self.publisher_.publish(twist)
            time.sleep(0.05)

    def stop(self, duration=0.1):
        self.send_twist(0.0, 0.0, 0.0, duration)

    def move_direction(self, dx, dy, odom_sub, distance=GRID_SIZE, speed=0.2):
        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos
        while (
            rclpy.ok()
            and math.hypot(odom_sub.x_pos - start_x, odom_sub.y_pos - start_y) < distance
        ):
            self.send_twist(
                linear_x=speed * dy, linear_y=speed * dx, angular_z=0.0, duration=0.05
            )
            rclpy.spin_once(odom_sub, timeout_sec=0.05)
        self.stop()

    def rotate_to_yaw(self, target_yaw, odom_sub, yaw_tol=0.05, max_speed=0.3):
        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            err = math.atan2(
                math.sin(target_yaw - odom_sub.yaw),
                math.cos(target_yaw - odom_sub.yaw),
            )
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
        x_size = random.randint(6, 12)
    if y_size is None:
        y_size = random.randint(6, 12)
    maze = np.zeros((x_size, y_size), dtype=int)
    if num_walls is None:
        num_walls = int(x_size * 1.5)
    wall_indices = random.sample(range(x_size * y_size), num_walls)
    for idx in wall_indices:
        row = idx // y_size
        col = idx % y_size
        maze[row, col] = 1
    start = (0, 0)
    goal = (x_size - 1, y_size - 1)
    maze[start] = 0
    maze[goal] = 0
    return maze, start, goal


# ---------------- A* Path Planning ----------------
def a_star_path(maze, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    frontier = [(f_score[start], start)]
    parent = {}
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if (
                0 <= neighbor[0] < maze.shape[0]
                and 0 <= neighbor[1] < maze.shape[1]
                and maze[neighbor] == 0
            ):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float("inf")):
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(frontier, (f_score[neighbor], neighbor))

    path = []
    node = goal
    while node in parent:
        path.append(node)
        node = parent[node]
    path.append(start)
    path.reverse()
    return path if path and path[0] == start and path[-1] == goal else []


# ---------------- Live Maze Plot ----------------
def plot_maze(maze, start, goal, path=None, robot_pos=None):
    plt.clf()
    plt.imshow(maze, cmap="gray_r")
    if path:
        px, py = zip(*path)
        plt.plot(py, px, "b.-", label="Path")
    if robot_pos:
        x, y = robot_pos
        gx = int(round(x / GRID_SIZE))
        gy = int(round(y / GRID_SIZE))
        gx = np.clip(gx, 0, maze.shape[0] - 1)
        gy = np.clip(gy, 0, maze.shape[1] - 1)
        plt.plot(gy, gx, "ro", label="Robot")
    plt.plot(start[1], start[0], "go", markersize=10, label="Start")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal")
    plt.legend()
    plt.draw()
    plt.pause(0.001)


# ---------------- Dynamic Path Following ----------------
def follow_path_dynamic(node, path, odom_sub, maze_updater, maze, start, goal):
    plt.ion()
    current_path = path
    rclpy.spin_once(odom_sub)
    plot_maze(maze, start, goal, current_path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))

    while rclpy.ok() and current_path:
        rclpy.spin_once(odom_sub)
        rx, ry = odom_sub.x_pos, odom_sub.y_pos
        current_grid = maze_updater._world_to_grid(rx, ry)

        if not current_path or current_grid != current_path[0]:
            print("Replanning (off-path)...")
            current_path = a_star_path(maze, current_grid, goal)
            if not current_path:
                print("Goal unreachable. Stopping.")
                break

        if maze_updater.update_map(odom_sub):
            print("Map changed, replanning...")
            current_path = a_star_path(maze, current_grid, goal)
            if not current_path:
                print("Goal now unreachable. Stopping.")
                break

        if len(current_path) > 1:
            cur, nxt = current_path[0], current_path[1]
            dx, dy = nxt[0] - cur[0], nxt[1] - cur[1]
            node.move_direction(dx, dy, odom_sub, distance=GRID_SIZE)
            rclpy.spin_once(odom_sub)
            plot_maze(maze, start, goal, current_path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))
        elif len(current_path) == 1 and current_path[0] == goal:
            print("Goal reached!")
            break
        else:
            print("Path invalid, stopping.")
            break


# ---------------- Main ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()

    # Setup map
    maze = np.array([
        [0,1,0,0,0,0],
        [0,1,0,1,1,0],
        [0,0,0,1,0,0],
        [0,1,1,1,0,1],
        [0,0,1,0,0,0],
        [0,1,1,1,1,0]
    ])
    start = (0, 0)
    goal = (1, 5)

    path = a_star_path(maze, start, goal)
    if not path:
        print("Initial path failed! Goal unreachable.")
        return

    lidar_map_updater = LidarMapUpdater(maze, GRID_SIZE)

    # Run executor in background
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(odom_reader)
    executor.add_node(lidar_map_updater)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        print("Starting navigation...")
        follow_path_dynamic(node, path, odom_reader, lidar_map_updater, maze, start, goal)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Shutting down...")
        node.stop()
        executor.shutdown()
        node.destroy_node()
        odom_reader.destroy_node()
        lidar_map_updater.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
