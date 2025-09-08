import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import random
from collections import deque
import math

# ---------------- Quaternion to Euler Conversion ----------------
def euler_from_quaternion(quat):
    """
    Convert quaternion (x, y, z, w) to roll, pitch, yaw
    """
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    """Subscribes to odometry topic to read robot pose"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

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
            time.sleep(0.1)

    def move_distance(self, distance, speed=0.2):
        """Move forward/backward a specific distance using odometry"""
        odom_sub = OdometryReader()
        rclpy.spin_once(odom_sub)
        start_x = odom_sub.x_pos
        while rclpy.ok() and abs(odom_sub.x_pos - start_x) < abs(distance):
            direction = 1 if distance > 0 else -1
            self.send_twist(linear_x=speed*direction, angular_z=0.0, duration=0.1)
            rclpy.spin_once(odom_sub)
        self.stop()

    def rotate_to_angle(self, target_angle, speed=0.3):
        """Rotate robot to a target yaw (radians)"""
        odom_sub = OdometryReader()
        rclpy.spin_once(odom_sub)
        while rclpy.ok():
            current_yaw = odom_sub.yaw
            error = target_angle - current_yaw
            # Normalize angle error to [-pi, pi]
            error = math.atan2(math.sin(error), math.cos(error))
            if abs(error) < 0.05:  # tolerance in radians
                break
            angular_speed = speed if error > 0 else -speed
            self.send_twist(linear_x=0.0, angular_z=angular_speed, duration=0.1)
            rclpy.spin_once(odom_sub)
        self.stop()

    def stop(self, duration=0.5):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)

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

# ---------------- Display Maze ----------------
def print_maze(maze, start, goal, path=None):
    for i in range(maze.shape[0]):
        line = ""
        for j in range(maze.shape[1]):
            if (i,j) == start:
                line += " R "
            elif (i,j) == goal:
                line += " G "
            elif path and (i,j) in path:
                line += " x "
            elif maze[i,j] == 1:
                line += " 0 "
            else:
                line += " . "
        print(line)
    print("\n")

# ---------------- Map Grid Navigation ----------------
def follow_path(node, path, cell_size=0.2):
    orientation = -math.pi/2  # Facing downwards (south) by default
    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]

        if dx == 1:      # move down
            target_angle = -math.pi/2
        elif dx == -1:   # move up
            target_angle = math.pi/2
        elif dy == 1:    # move right
            target_angle = 0.0
        elif dy == -1:   # move left
            target_angle = math.pi

        node.rotate_to_angle(target_angle)
        node.move_distance(cell_size)

# ---------------- Main Program ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()

    USE_FIXED_MAP = True  # switch between random and fixed map

    if USE_FIXED_MAP:
        # Fixed 6x6 maze example
        maze = np.array([
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 1, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 1, 1, 1, 0, 1],
            [0, 0, 1, 0, 0, 0],
            [0, 1, 1, 1, 1, 0]
        ])
        start = (0,0)
        goal  = (5,5)
    else:
        maze, start, goal = generate_maze()

    print("Original Maze:")
    print_maze(maze, start, goal)

    path = bfs_path(maze, start, goal)
    print("Solution Path:")
    print_maze(maze, start, goal, path)

    follow_path(node, path, cell_size=0.2)

    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
