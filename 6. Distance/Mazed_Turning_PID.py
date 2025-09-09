#Tuning tips
#If turns are too wide/tight, tweak in _rotate_to_angle(...):
#fwd_speed (linear.x), start at 0.10–0.20.
#steer_gain (scales angular.z), start at 1.0–1.5.
#yaw_tol (stop tolerance), ~0.05 rad (~3°) is a good start.

#🔧 Notes on the PID controller
#Gains are set inside _rotate_to_angle:
#Kp=1.2, Ki=0.0, Kd=0.3
#Start with just P (Kp) to get basic turning.
#Add D (Kd) to reduce overshoot.
#If you see steady-state error, add a small I.
#Adjust fwd_speed if turns are too wide/narrow (slower = tighter).
#Adjust yaw_tol (rad) if it keeps oscillating near the target (increase tolerance to e.g. 0.1 rad ≈ 6°).

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import random
from collections import deque
import matplotlib.pyplot as plt
import math  # --- NEW: for yaw math

# ---------------- Try importing tf_transformations ----------------
try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    print("[WARN] tf_transformations not found. Install with:")
    print("       pip install tf-transformations OR sudo apt install ros-${ROS_DISTRO}-tf-transformations")

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # <<< Define a unique variable for grid cell size (was hard-coded before)

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    """Subscribes to odometry topic to read robot X,Y position"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0  # --- NEW: track heading (rad)
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y

        # --- NEW: quaternion -> yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

# ---------------- Command Velocity Publisher ----------------
class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x, angular_z, duration):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            plt.pause(0.01)  # ← allow GUI update

    def move_distance(self, distance, odom_sub, speed=0.2):
        """Move forward/backward a specific distance using odometry (X axis only)"""
        print(f"[INFO] Moving {distance} meters at speed {speed}")
        rclpy.spin_once(odom_sub)
        start_x = odom_sub.x_pos
        while rclpy.ok() and abs(odom_sub.x_pos - start_x) < abs(distance):
            direction = 1 if distance > 0 else -1
            self.send_twist(linear_x=speed*direction, angular_z=0.0, duration=0.1)
            rclpy.spin_once(odom_sub)
            plt.pause(0.01)  # ← allow GUI update
        self.stop(0.1)

    # --- NEW (PID): helper for angle wrapping
    @staticmethod
    def _angle_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    # --- NEW (PID): odometry-based rotation with PID control
    def _rotate_to_angle(self, target_delta_rad, odom_sub,
                         fwd_speed=0.15, yaw_tol=0.05,
                         Kp=1.2, Ki=0.0, Kd=0.3):
        """
        Rotate by target_delta_rad using odometry yaw with PID steering.
        Ackermann: we cannot spin in place, so we move forward with steering.
        - fwd_speed: constant forward speed (m/s)
        - yaw_tol: stop when within this yaw error (rad)
        - Kp, Ki, Kd: PID gains for angular.z
        """
        rclpy.spin_once(odom_sub)
        start_yaw = odom_sub.yaw
        target_yaw = start_yaw + target_delta_rad

        print(f"[INFO] Ackermann PID rotate: from {math.degrees(start_yaw):.1f}° "
              f"to {math.degrees(target_yaw):.1f}° (Δ={math.degrees(target_delta_rad):.1f}°)")

        # PID state
        prev_err = 0.0
        integral = 0.0
        dt = 0.05  # control period

        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            err = self._angle_diff(target_yaw, odom_sub.yaw)

            if abs(err) <= yaw_tol:
                break

            # PID compute
            integral += err * dt
            derivative = (err - prev_err) / dt
            control = Kp * err + Ki * integral + Kd * derivative
            prev_err = err

            # Clamp steering
            control = max(min(control, 1.5), -1.5)

            twist = Twist()
            twist.linear.x = fwd_speed
            twist.angular.z = control
            self.publisher_.publish(twist)

            plt.pause(dt)  # ← allow GUI update

        self.stop(0.15)

    def turn_left(self, speed=0.5, duration=1.0, odom_sub=None):
        if odom_sub is not None:
            self._rotate_to_angle(target_delta_rad=math.pi/2, odom_sub=odom_sub,
                                  fwd_speed=0.15, yaw_tol=0.05)
        else:
            print(f"[INFO] Turning left for {duration}s at speed {speed} (legacy)")
            self.send_twist(linear_x=0.15, angular_z=+1.0, duration=duration)

    def turn_right(self, speed=0.5, duration=1.0, odom_sub=None):
        if odom_sub is not None:
            self._rotate_to_angle(target_delta_rad=-math.pi/2, odom_sub=odom_sub,
                                  fwd_speed=0.15, yaw_tol=0.05)
        else:
            print(f"[INFO] Turning right for {duration}s at speed {speed} (legacy)")
            self.send_twist(linear_x=0.15, angular_z=-1.0, duration=duration)

    def stop(self, duration=1.0):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            plt.pause(0.01)  # ← allow GUI update

# ---------------- Maze Generation ----------------
def generate_maze(x_size=None, y_size=None, num_walls=None):
    if x_size is None:
        x_size = random.randint(6, 30)
    if y_size is None:
        y_size = random.randint(6, 30)
    maze = np.zeros((x_size, y_size), dtype=int)
    if num_walls is None:
        num_walls = int(x_size * 1.8)
    wall_indices = random.sample(range(x_size * y_size), num_walls)
    for idx in wall_indices:
        row = idx // y_size
        col = idx % y_size
        maze[row, col] = 1
    start = (random.randint(0, x_size-1), random.randint(0, y_size-1))
    goal  = (random.randint(0, x_size-1), random.randint(0, y_size-1))
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
            if (0 <= neighbor[0] < maze.shape[0] and
                0 <= neighbor[1] < maze.shape[1] and
                maze[neighbor] == 0 and
                visited[neighbor] == 0):
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

# ---------------- Live Plot ----------------
def plot_maze(maze, start, goal, path=None, current=None):
    plt.clf()
    plt.imshow(maze, cmap="gray_r")
    if path:
        px, py = zip(*path)
        plt.plot(py, px, "b.-", label="Path")
    if current:
        plt.plot(current[1], current[0], "ro", label="Robot")
    plt.plot(start[1], start[0], "go", markersize=10, label="Start")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal")
    plt.legend()
    plt.pause(0.01)  # ← allow GUI update

# ---------------- Map Grid Navigation ----------------
def follow_path(node, path, odom_sub, maze, start, goal):
    plt.ion()
    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]

        if dx == 1:     # down
            node.turn_right(duration=0.5, odom_sub=odom_sub)
        elif dx == -1:  # up
            node.turn_left(duration=0.5, odom_sub=odom_sub)
        elif dy == 1:   # right
            node.turn_right(duration=0.5, odom_sub=odom_sub)
        elif dy == -1:  # left
            node.turn_left(duration=0.5, odom_sub=odom_sub)

        node.move_distance(GRID_SIZE, odom_sub=odom_sub)
        plot_maze(maze, start, goal, path, current=nxt)

# ---------------- Main Program ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()

    USE_FIXED_MAP = True

    if USE_FIXED_MAP:
        maze = np.array([
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 1, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 1, 1, 1, 0, 1],
            [0, 0, 1, 0, 0, 0],
            [0, 1, 1, 1, 1, 0]
        ])
        start = (0,0)
        goal  = (1,5)
    else:
        maze, start, goal = generate_maze()

    path = bfs_path(maze, start, goal)

    plt.ion()
    plt.figure()
    plot_maze(maze, start, goal, path, current=start)

    follow_path(node, path, odom_sub=odom_reader, maze=maze, start=start, goal=goal)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
