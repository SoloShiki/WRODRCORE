#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import math

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # meters per grid cell
MOVE_SPEED = 0.2
YAW_CORRECTION_GAIN = 0.5

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

    def stop(self):
        self.send_twist(0.0, 0.0, 0.0, 0.1)

    # ---------------- Move continuously in direction ----------------
    def move_direction(self, dx, dy, odom_sub, distance=GRID_SIZE):
        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos
        target_yaw = odom_sub.yaw  # hold heading
        while rclpy.ok() and math.hypot(odom_sub.x_pos - start_x, odom_sub.y_pos - start_y) < distance:
            rclpy.spin_once(odom_sub)
            # yaw correction
            err = math.atan2(math.sin(target_yaw - odom_sub.yaw), math.cos(target_yaw - odom_sub.yaw))
            self.send_twist(
                linear_x=MOVE_SPEED * dx,
                linear_y=MOVE_SPEED * dy,
                angular_z=YAW_CORRECTION_GAIN * err,
                duration=0.05
            )
        self.stop()

    # ---------------- Rotate to desired yaw ----------------
    def rotate_to_yaw(self, target_yaw, odom_sub, yaw_tol=0.05, max_speed=0.3):
        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            yaw_err = math.atan2(math.sin(target_yaw - odom_sub.yaw), math.cos(target_yaw - odom_sub.yaw))
            if abs(yaw_err) <= yaw_tol:
                break
            twist = Twist()
            twist.angular.z = max(-max_speed, min(max_speed, yaw_err))
            self.publisher_.publish(twist)
            time.sleep(0.05)
        self.stop()

# ---------------- Map Parsing ----------------
def parse_map(layout):
    """
    Convert a map defined with:
      '0' = free
      '1' = wall
      'R' = start
      'G' = goal
    """
    maze = np.zeros((len(layout), len(layout[0])), dtype=int)
    start = goal = None
    for r, row in enumerate(layout):
        for c, val in enumerate(row):
            if val == "1":
                maze[r, c] = 1
            elif val == "R":
                start = (r, c)
                maze[r, c] = 0
            elif val == "G":
                goal = (r, c)
                maze[r, c] = 0
            else:
                maze[r, c] = 0
    if start is None or goal is None:
        raise ValueError("Map must contain both 'R' (start) and 'G' (goal)!")
    return maze, start, goal

# ---------------- Convert Orientation Choice to Map Yaw ----------------
def orientation_to_map_yaw(orientation_str):
    mapping = {
        "north": math.pi/2,
        "south": -math.pi/2,
        "east": 0.0,
        "west": math.pi
    }
    return mapping.get(orientation_str.lower(), 0.0)

# ---------------- BFS Path Planning ----------------
def bfs_path(maze, start, goal):
    visited = np.zeros_like(maze)
    parent = {}
    frontier = deque([start])
    visited[start] = 1
    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    while frontier:
        current = frontier.popleft()
        if current == goal:
            break
        for d in directions:
            neighbor = (current[0]+d[0], current[1]+d[1])
            if (0 <= neighbor[0] < maze.shape[0] and 0 <= neighbor[1] < maze.shape[1] 
                and maze[neighbor] == 0 and visited[neighbor] == 0):
                frontier.append(neighbor)
                visited[neighbor] = 1
                parent[neighbor] = current
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = parent.get(node, start)
    path.append(start)
    path.reverse()
    return path

# ---------------- Plotting ----------------
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
        gx = np.clip(gx, 0, maze.shape[0]-1)
        gy = np.clip(gy, 0, maze.shape[1]-1)
        plt.plot(gy, gx, "ro", label="Robot")
    plt.plot(start[1], start[0], "go", markersize=10, label="Start (R)")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal (G)")
    plt.legend()
    plt.draw()
    plt.pause(0.001)

# ---------------- Follow Path ----------------
def follow_path(node, path, odom_sub, maze, start, goal):
    plt.ion()
    prev_dx = prev_dy = None
    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]
        # if direction same as previous, move without stopping
        node.move_direction(dx, dy, odom_sub, distance=GRID_SIZE)
        prev_dx, prev_dy = dx, dy
        rclpy.spin_once(odom_sub)
        plot_maze(maze, start, goal, path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))

# ---------------- Main ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()
    rclpy.spin_once(odom_reader)

    # Fixed map layout
    maze_layout = [
        ["R", "1", "0", "0", "0", "0"],
        ["0", "1", "0", "1", "1", "0"],
        ["0", "0", "0", "1", "0", "0"],
        ["0", "1", "1", "1", "0", "1"],
        ["0", "0", "1", "0", "0", "0"],
        ["G", "1", "1", "1", "1", "0"]
    ]
    maze, start, goal = parse_map(maze_layout)

    # --- Choose initial orientation relative to map ---
    #print("Choose robot initial orientation (north, south, east, west):")
    choice = "south"
    desired_yaw = orientation_to_map_yaw(choice)
    print(f"Adjusted initial yaw to match map: {desired_yaw:.2f} rad")
    node.rotate_to_yaw(desired_yaw, odom_reader)

    path = bfs_path(maze, start, goal)

    plt.figure()
    plot_maze(maze, start, goal, path, robot_pos=(odom_reader.x_pos, odom_reader.y_pos))

    follow_path(node, path, odom_sub=odom_reader, maze=maze, start=start, goal=goal)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
