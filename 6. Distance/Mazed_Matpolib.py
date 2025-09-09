import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import random
from collections import deque
import matplotlib.pyplot as plt

# ---------------- Try importing tf_transformations ----------------
try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    print("[WARN] tf_transformations not found. Install with:")
    print("       pip install tf-transformations OR sudo apt install ros-${ROS_DISTRO}-tf-transformations")

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    """Subscribes to odometry topic to read robot X,Y position"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y

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
    def move_distance(self, distance, odom_sub, speed=0.2):
        print(f"[INFO] Moving {distance} meters at speed {speed}")
        rclpy.spin_once(odom_sub)
        start_x = odom_sub.x_pos
        while rclpy.ok() and abs(odom_sub.x_pos - start_x) < abs(distance):
            direction = 1 if distance > 0 else -1
            self.send_twist(linear_x=speed*direction, angular_z=0.0, duration=0.1)
            rclpy.spin_once(odom_sub)
        self.stop(0.1) # small stop

    def turn_left(self, speed=0.5, duration=1.0):
        print(f"[INFO] Turning left for {duration}s at speed {speed}")
        self.send_twist(0.0, speed, duration)

    def turn_right(self, speed=0.5, duration=1.0):
        print(f"[INFO] Turning right for {duration}s at speed {speed}")
        self.send_twist(0.0, -speed, duration)

    def stop(self, duration=1.0):
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

# ---------------- Map Grid Navigation + Live Visualization ----------------
def follow_path(node, path, odom_sub, cell_size=0.2, maze=None, start=None, goal=None, ax=None):
    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]

        # Send robot movement
        if dx == 1:     # down
            node.turn_right(duration=0.5)
        elif dx == -1:  # up
            node.turn_left(duration=0.5)
        elif dy == 1:   # right
            node.turn_right(duration=0.5)
        elif dy == -1:  # left
            node.turn_left(duration=0.5)

        node.move_distance(cell_size, odom_sub=odom_sub)

        # Update visualization (robot moves step by step)
        if maze is not None and ax is not None:
            img = np.copy(maze)
            img[start] = 2
            img[goal] = 3
            for visited in path[:i+1]:  # leave trail
                img[visited] = 5
            img[nxt] = 4  # current robot position

            ax.clear()
            ax.imshow(img, cmap="tab20", origin="upper")
            ax.set_title("Maze Navigation (Live)")
            plt.pause(0.4)

# ---------------- Main Program ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()

    USE_FIXED_MAP = True  # Set to False to use random map

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

    # Prepare matplotlib figure
    fig, ax = plt.subplots()

    # Move robot while updating visualization live
    follow_path(node, path, odom_sub=odom_reader, cell_size=0.2, maze=maze, start=start, goal=goal, ax=ax)

    plt.show()  # keep window open after movement finishes

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
