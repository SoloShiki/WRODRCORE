#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import random
from collections import deque
import matplotlib.pyplot as plt
import math

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # meters per grid cell

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

    def stop(self, duration=0.1):
        self.send_twist(0.0, 0.0, 0.0, duration)

    # ---------------- Smooth Move in grid direction (patched) ----------------
    def move_direction(self, dx, dy, odom_sub, distance=GRID_SIZE, speed=0.25, min_speed=0.05):
        """
        Move the robot in the grid direction (dx, dy) with a cosine accel/decel profile.
        Ensures a small minimum speed to overcome deadband/static friction.
        dx = row change (-1 up, +1 down)
        dy = col change (-1 left, +1 right)
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
            twist.linear.x = current_speed * dx
            twist.linear.y = current_speed * dy
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

            time.sleep(rate_sleep)

        # final stop
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
            if (0<=neighbor[0]<maze.shape[0] and 0<=neighbor[1]<maze.shape[1] 
                and maze[neighbor]==0 and visited[neighbor]==0):
                frontier.append(neighbor)
                visited[neighbor]=1
                parent[neighbor]=current
    path=[]
    node=goal
    while node!=start:
        path.append(node)
        node=parent.get(node,start)
    path.append(start)
    path.reverse()
    return path

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
        gx = np.clip(gx, 0, maze.shape[0]-1)
        gy = np.clip(gy, 0, maze.shape[1]-1)
        plt.plot(gy, gx, "ro", label="Robot")
    plt.plot(start[1], start[0], "go", markersize=10, label="Start")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal")
    plt.legend()
    plt.draw()
    plt.pause(0.001)

# ---------------- Follow Path with Continuous Straight Segments ----------------
def follow_path(node, path, odom_sub, maze, start, goal):
    plt.ion()
    i = 1
    while i < len(path):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0]-cur[0]
        dy = nxt[1]-cur[1]

        # Extend straight line distance
        segment_length = GRID_SIZE
        while i+1 < len(path) and (path[i+1][0]-nxt[0], path[i+1][1]-nxt[1]) == (dx, dy):
            segment_length += GRID_SIZE
            i += 1
            nxt = path[i]

        node.move_direction(dx, dy, odom_sub, distance=segment_length)
        rclpy.spin_once(odom_sub)
        plot_maze(maze, start, goal, path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))
        i += 1

# ---------------- Main ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()
    rclpy.spin_once(odom_reader)

    USE_FIXED_MAP = True
    if USE_FIXED_MAP:
        maze = np.array([
            [0,1,0,0,0,0],
            [0,1,0,1,1,0],
            [0,0,0,1,0,0],
            [0,1,1,1,0,1],
            [0,0,1,0,0,0],
            [0,1,1,1,1,0]
        ])
        start = (0,0)
        goal  = (1,5)
    else:
        maze, start, goal = generate_maze()

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
