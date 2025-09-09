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
        self.yaw = 0.0  # track heading (rad)
        self.start_x = None
        self.start_y = None
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        if self.start_x is None:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y

        self.x_pos = msg.pose.pose.position.x - self.start_x
        self.y_pos = msg.pose.pose.position.y - self.start_y

        # quaternion -> yaw
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
            time.sleep(0.05)

    def move_distance(self, distance, odom_sub, fwd_speed=0.15):
        """Move forward a specific distance using odometry"""
        rclpy.spin_once(odom_sub)
        start_x = odom_sub.x_pos
        start_y = odom_sub.y_pos
        traveled = 0.0

        while rclpy.ok() and traveled < abs(distance):
            dx = odom_sub.x_pos - start_x
            dy = odom_sub.y_pos - start_y
            traveled = math.hypot(dx, dy)
            direction = 1 if distance > 0 else -1
            self.send_twist(linear_x=fwd_speed*direction, angular_z=0.0, duration=0.05)
            rclpy.spin_once(odom_sub)

        self.stop(0.05)

    # --- NEW (PID helper)
    @staticmethod
    def _angle_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    # --- PID rotation for Ackermann
    def _rotate_to_angle(self, target_delta_rad, odom_sub,
                         fwd_speed=0.15, yaw_tol=0.05,
                         Kp=1.2, Ki=0.0, Kd=0.3):
        rclpy.spin_once(odom_sub)
        start_yaw = odom_sub.yaw
        target_yaw = start_yaw + target_delta_rad

        prev_err = 0.0
        integral = 0.0
        dt = 0.05

        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            err = self._angle_diff(target_yaw, odom_sub.yaw)
            if abs(err) <= yaw_tol:
                break

            integral += err * dt
            derivative = (err - prev_err) / dt
            prev_err = err
            control = Kp*err + Ki*integral + Kd*derivative
            control = max(min(control, 1.5), -1.5)

            twist = Twist()
            twist.linear.x = fwd_speed
            twist.angular.z = control
            self.publisher_.publish(twist)
            time.sleep(dt)

        self.stop(0.05)

    def turn_left(self, odom_sub=None):
        if odom_sub is not None:
            self._rotate_to_angle(math.pi/2, odom_sub)
        else:
            self.send_twist(0.15, +1.0, 1.0)

    def turn_right(self, odom_sub=None):
        if odom_sub is not None:
            self._rotate_to_angle(-math.pi/2, odom_sub)
        else:
            self.send_twist(0.15, -1.0, 1.0)

    def stop(self, duration=0.2):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

# ---------------- Maze Generation ----------------
def generate_maze(x_size=None, y_size=None, num_walls=None):
    if x_size is None:
        x_size = random.randint(6,30)
    if y_size is None:
        y_size = random.randint(6,30)
    maze = np.zeros((x_size,y_size), dtype=int)
    if num_walls is None:
        num_walls = int(x_size*1.8)
    wall_indices = random.sample(range(x_size*y_size), num_walls)
    for idx in wall_indices:
        row = idx // y_size
        col = idx % y_size
        maze[row, col] = 1
    start = (random.randint(0,x_size-1), random.randint(0,y_size-1))
    goal = (random.randint(0,x_size-1), random.randint(0,y_size-1))
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
def plot_maze(maze, start, goal, robot_pos, path=None):
    plt.clf()
    plt.imshow(maze, cmap="gray_r")
    if path:
        px, py = zip(*path)
        plt.plot(py, px, "b.-", label="Path")
    if robot_pos:
        plt.plot(robot_pos[1], robot_pos[0], "ro", label="Robot")
    plt.plot(start[1], start[0], "go", markersize=10, label="Start")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal")
    plt.legend()
    plt.pause(0.001)

# ---------------- Map Grid Navigation ----------------
def follow_path(node, path, odom_sub, maze, start, goal):
    plt.ion()
    for i in range(1,len(path)):
        cur = path[i-1]
        nxt = path[i]

        # calculate target in meters
        target_x = nxt[0]*GRID_SIZE
        target_y = nxt[1]*GRID_SIZE

        # rotate toward target
        dx = target_x - odom_sub.x_pos
        dy = target_y - odom_sub.y_pos
        target_angle = math.atan2(dy, dx)
        angle_diff = node._angle_diff(target_angle, odom_sub.yaw)
        node._rotate_to_angle(angle_diff, odom_sub)

        # move toward target
        distance = math.hypot(dx, dy)
        node.move_distance(distance, odom_sub)

        # update plot
        robot_grid = (odom_sub.x_pos/GRID_SIZE, odom_sub.y_pos/GRID_SIZE)
        plot_maze(maze, start, goal, robot_pos=robot_grid, path=path)

# ---------------- Main Program ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()

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
        goal = (1,5)
    else:
        maze, start, goal = generate_maze()

    path = bfs_path(maze, start, goal)

    plt.figure()
    plot_maze(maze, start, goal, robot_pos=(0,0), path=path)

    follow_path(node, path, odom_reader, maze, start, goal)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()

if __name__=="__main__":
    main()
