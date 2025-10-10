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

# ---------------- Utility Functions ----------------
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    # ... (Keep the class as in Mecanum_Maze_A.py/B.py) ...
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
    # ... (Keep the class as in Program 1, no changes needed for B) ...
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
        row = int(round(y / self.grid_size)) # NOTE: Grid is typically (row, col) = (y, x) in array
        col = int(round(x / self.grid_size))
        return np.clip(row, 0, self.maze.shape[0]-1), np.clip(col, 0, self.maze.shape[1]-1)

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
    # ... (Keep send_twist, stop as in Mecanum_Maze_B.py) ...
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

    # ---------------- Smooth Move in grid direction (from Mecanum_Maze_B.py) ----------------
    def move_direction(self, dx, dy, odom_sub, distance=GRID_SIZE, speed=0.25):
        """
        Move the robot in the grid direction (dx, dy)
        dx = row change, dy = col change (swapped in the original code, but maintained here)
        Smooth cosine acceleration/deceleration profile.
        """
        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos
        moved = 0.0
        accel_distance = min(0.2 * distance, 0.05)
        decel_distance = accel_distance

        while rclpy.ok() and moved < distance:
            rclpy.spin_once(odom_sub)
            dx_pos = odom_sub.x_pos - start_x
            dy_pos = odom_sub.y_pos - start_y
            moved = math.hypot(dx_pos, dy_pos)

            # --- Cosine-based smooth speed factor ---
            if moved < accel_distance:
                phase = moved / accel_distance
                factor = 0.5 * (1 - math.cos(math.pi * phase))
            elif moved > distance - decel_distance:
                phase = (distance - moved) / decel_distance
                phase = max(min(phase, 1.0), 0.0)
                factor = 0.5 * (1 - math.cos(math.pi * phase))
            else:
                factor = 1.0

            current_speed = speed * factor

            self.send_twist(
                linear_x=current_speed * dy, # Swapped for consistency with A.py's implementation
                linear_y=current_speed * dx, # Swapped for consistency with A.py's implementation
                angular_z=0.0,
                duration=0.05
            )

        self.stop()

# ---------------- Maze Generation ----------------
# ... (Keep generate_maze from Mecanum_Maze_A.py) ...
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

# ---------------- A* Path Planning (from Program 1) ----------------
def a_star_path(maze, start, goal):
    """A* search for the shortest path"""
    rows, cols = maze.shape
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    frontier = [(f_score[start], start)]
    parent = {}
    directions = [(-1,0),(1,0),(0,-1),(0,1)]

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        for d in directions:
            neighbor = (current[0]+d[0], current[1]+d[1])
            if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols
                and maze[neighbor] == 0):
                
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(frontier, (f_score[neighbor], neighbor))
    
    path=[]
    node=goal
    while node in parent:
        path.append(node)
        node=parent[node]
    path.append(start)
    path.reverse()
    
    if path and path[0] == start and path[-1] == goal:
        return path
    else:
        return []

# ---------------- Live Maze Plot ----------------
# ... (Keep plot_maze from Mecanum_Maze_A.py) ...
def plot_maze(maze, start, goal, path=None, robot_pos=None):
    plt.clf()
    plt.imshow(maze, cmap="gray_r")
    if path:
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

# ---------------- Follow Path with Continuous Straight Segments (Modified for D* simulation) ----------------
def follow_path_dynamic(node, path, odom_sub, maze_updater, maze, start, goal):
    plt.ion()
    current_path = path

    # --- Initial position for the plot ---
    rclpy.spin_once(odom_sub)
    plot_maze(maze, start, goal, current_path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))

    i = 0
    while rclpy.ok() and current_path:
        
        # 1. Check current position against planned path / Check for end of path segment
        rclpy.spin_once(odom_sub)
        rx, ry = odom_sub.x_pos, odom_sub.y_pos
        current_grid_pos = maze_updater._world_to_grid(rx, ry)
        
        # Find where we are on the planned path (i.e., which path segment to start from)
        try:
            current_path_index = current_path.index(current_grid_pos)
        except ValueError:
            # We are not on the planned path. Re-plan from current position.
            current_path_index = 0 
            print("Re-planning: Off-path.")
            current_path = a_star_path(maze, current_grid_pos, goal)
            if not current_path:
                print("Goal unreachable from current position. Stopping.")
                break
        
        if current_grid_pos == goal:
            print("Goal reached!")
            break
        
        i = current_path_index + 1
        if i >= len(current_path):
            print("Path segment completed. Goal reached or near. Stopping.")
            break
            

        # 2. Map Update / Obstacle Detection
        map_changed = maze_updater.update_map(odom_sub)
        
        if map_changed:
            print("Map changed due to Lidar detection. Re-planning...")
            # Re-plan from current grid position
            current_path = a_star_path(maze, current_grid_pos, goal)
            i = 1 # Start from the first step of the new path
            if not current_path:
                print("Goal now unreachable. Stopping.")
                break

        # 3. Movement with continuous segments
        if len(current_path) > 1:
            cur = current_path[i-1]
            nxt = current_path[i]
            dx = nxt[0]-cur[0]
            dy = nxt[1]-cur[1]

            # Extend straight line distance (Mecanum_Maze_B.py logic)
            segment_length = GRID_SIZE
            temp_i = i
            while temp_i + 1 < len(current_path) and (current_path[temp_i+1][0]-nxt[0], current_path[temp_i+1][1]-nxt[1]) == (dx, dy):
                segment_length += GRID_SIZE
                temp_i += 1
                nxt = current_path[temp_i]

            # Perform the extended move
            node.move_direction(dx, dy, odom_sub, distance=segment_length)
            
            # Update the main loop index to reflect the segment completion
            i = temp_i + 1
            
            # 4. Update plot after movement
            rclpy.spin_once(odom_sub)
            plot_maze(maze, start, goal, current_path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))
        else:
            break

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

    initial_path = a_star_path(maze, start, goal)
    if not initial_path:
        print("Initial path failed! Goal unreachable in initial map.")
        return

    lidar_map_updater = LidarMapUpdater(maze, GRID_SIZE)

    # Use a multithreaded executor to spin multiple nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(odom_reader)
    executor.add_node(lidar_map_updater)
    
    # Run the main loop
    executor.create_task(follow_path_dynamic, node, initial_path, odom_reader, lidar_map_updater, maze, start, goal)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        node.stop()
        node.destroy_node()
        odom_reader.destroy_node()
        lidar_map_updater.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()