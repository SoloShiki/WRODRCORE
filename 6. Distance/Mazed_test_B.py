import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import numpy as np
import random
from collections import deque
import threading

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    """Subscribes to odometry topic to read robot position and yaw"""
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
        # Convert quaternion to yaw
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

    def stop(self, duration=0.5):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

    # Move forward/backward using odometry
    def move_distance(self, distance, odom, speed=0.15):
        start_x, start_y = odom.x_pos, odom.y_pos
        traveled = 0.0
        while rclpy.ok() and traveled < abs(distance):
            direction = 1 if distance > 0 else -1
            self.send_twist(speed * direction, 0.0, 0.1)
            dx = odom.x_pos - start_x
            dy = odom.y_pos - start_y
            traveled = math.sqrt(dx*dx + dy*dy)
        self.stop()

    # Rotate robot on its axis
    def rotate_to_angle(self, target_angle, odom, speed=0.3):
        def normalize_angle(a):
            return math.atan2(math.sin(a), math.cos(a))

        target_angle = normalize_angle(target_angle)
        while rclpy.ok():
            current = normalize_angle(odom.yaw)
            error = normalize_angle(target_angle - current)
            if abs(error) < 0.05:  # ~3 degrees tolerance
                break
            angular_z = speed if error > 0 else -speed
            self.send_twist(0.0, angular_z, 0.05)
        self.stop()


# ---------------- Maze Generation ----------------
def generate_maze(x_size=None, y_size=None, num_walls=None):
    if x_size is None:
        x_size = random.randint(6, 12)
    if y_size is None:
        y_size = random.randint(6, 12)
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
    directions = [(-1,0),(1,0),(0,-1),(0,1)]  # up, down, left, right
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
                line += " # "
            else:
                line += " . "
        print(line)
    print("\n")


# ---------------- Map Grid Navigation ----------------
def follow_path(cmd_node, odom, path, cell_size=0.2):
    """
    Move the robot along the BFS path.
    Robot starts facing south (negative Y).
    """
    orientation = "S"  # N,E,S,W
    orientation_map = {
        "N":  math.pi/2,
        "E":  0.0,
        "S": -math.pi/2,
        "W":  math.pi
    }

    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]

        if dx == 1:      # down
            orientation = "S"
        elif dx == -1:   # up
            orientation = "N"
        elif dy == 1:    # right
            orientation = "E"
        elif dy == -1:   # left
            orientation = "W"

        target_yaw = orientation_map[orientation]
        cmd_node.rotate_to_angle(target_yaw, odom)

        # --- Print movement log ---
        print(f"[STEP {i}] Moving {orientation} | "
              f"Pos=({odom.x_pos:.2f}, {odom.y_pos:.2f}) | "
              f"Yaw={math.degrees(odom.yaw):.1f}°")

        cmd_node.move_distance(cell_size, odom)


# ---------------- Main Program ----------------
def main():
    rclpy.init()

    # Nodes
    odom_reader = OdometryReader("/odom")
    cmd_node = CmdVelPublisher()

    # Executor in background thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_reader)
    executor.add_node(cmd_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Choose map type
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
        goal  = (5,5)
    else:
        maze, start, goal = generate_maze()

    print("Original Maze:")
    print_maze(maze, start, goal)

    path = bfs_path(maze, start, goal)
    print("Solution Path:")
    print_maze(maze, start, goal, path)

    follow_path(cmd_node, odom_reader, path, cell_size=0.25)

    cmd_node.stop()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
