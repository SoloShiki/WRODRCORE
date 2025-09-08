import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import random
from collections import deque
import math
from tf_transformations import euler_from_quaternion

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    """Subscribes to odometry topic to read robot position and orientation (yaw)"""
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
        quat = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = euler_from_quaternion(quat)

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

    # Movement functions
    def move_distance(self, distance, speed=0.2):
        """Move forward/backward a specific distance using odometry (X axis only for simplicity)"""
        odom_sub = OdometryReader()
        rclpy.spin_once(odom_sub)
        start_x = odom_sub.x_pos
        while rclpy.ok() and abs(odom_sub.x_pos - start_x) < abs(distance):
            direction = 1 if distance > 0 else -1
            self.send_twist(linear_x=speed*direction, angular_z=0.0, duration=0.1)
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

    # ---- Precise turning using odometry yaw ----
    def turn_in_place(self, angle_deg, speed=0.3):
        """Rotate robot in place using odometry yaw feedback"""
        odom_sub = OdometryReader()
        rclpy.spin_once(odom_sub)

        start_yaw = odom_sub.yaw
        target_yaw = start_yaw + math.radians(angle_deg)

        def normalize_angle(a):
            return math.atan2(math.sin(a), math.cos(a))

        target_yaw = normalize_angle(target_yaw)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = speed if angle_deg > 0 else -speed

        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            cur_yaw = odom_sub.yaw
            err = normalize_angle(target_yaw - cur_yaw)

            if abs(err) < math.radians(2):  # tolerance 2 degrees
                break

            self.publisher_.publish(twist)
            time.sleep(0.05)

        self.stop()
        print(f"✅ Turned {angle_deg}° (final yaw: {odom_sub.yaw:.2f} rad)")

    def turn_left(self):
        self.turn_in_place(90)

    def turn_right(self):
        self.turn_in_place(-90)

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
    # Robot starts facing SOUTH (downwards)
    orientation = "S"

    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]

        if dx == 1:      # move down (south)
            desired = "S"
        elif dx == -1:   # move up (north)
            desired = "N"
        elif dy == 1:    # move right (east)
            desired = "E"
        elif dy == -1:   # move left (west)
            desired = "W"
        else:
            continue

        # Adjust orientation
        if orientation != desired:
            orientation = rotate_to_orientation(node, orientation, desired)

        node.move_distance(cell_size)

def rotate_to_orientation(node, current, desired):
    """Rotate robot from current orientation to desired orientation"""
    order = ["N", "E", "S", "W"]  # clockwise order
    idx_cur = order.index(current)
    idx_des = order.index(desired)
    diff = (idx_des - idx_cur) % 4

    if diff == 1:   # 90° right
        node.turn_right()
    elif diff == 2: # 180°
        node.turn_right()
        node.turn_right()
    elif diff == 3: # 90° left
        node.turn_left()

    return desired

# ---------------- Main Program ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()

    USE_FIXED_MAP = True  # Set to False for random map

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
        start = (0,0)  # top-left
        goal  = (5,5)  # bottom-right
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
