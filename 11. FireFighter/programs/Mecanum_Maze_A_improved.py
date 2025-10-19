#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import math

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # meters per grid cell

# ---------------- IMU Reader ----------------
class IMUReader(Node):
    """Reads IMU yaw from /imu"""
    def __init__(self, topic='/imu'):
        super().__init__('imu_reader')
        self.yaw = 0.0
        self.subscription = self.create_subscription(Imu, topic, self.imu_callback, 10)

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    """Reads robot position (x,y) and yaw from /odom, fused with IMU yaw"""
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.odom_yaw = 0.0
        self.imu_yaw = 0.0
        # CRITICAL CHANGE: The offset to align the physical robot's
        # current orientation to a map-defined yaw (e.g., 'south' = -pi/2)
        self.map_yaw_offset = 0.0 
        self.fused_yaw = 0.0 # This now represents the map-frame yaw
        self.subscription = self.create_subscription(Odometry, topic, self.odom_callback, 10)

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        alpha = 0.98
        
        # Apply the fusion using raw IMU/Odometry data
        geographic_yaw = alpha * self.odom_yaw + (1 - alpha) * self.imu_yaw
        
        # Apply the map offset to get the yaw in the map frame
        self.fused_yaw = geographic_yaw + self.map_yaw_offset
        # Normalize the yaw to [-pi, pi]
        self.fused_yaw = math.atan2(math.sin(self.fused_yaw), math.cos(self.fused_yaw))


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

    # --- MODIFIED: Use holonomic motion relative to the map frame ---
    def move_direction(self, dx, dy, odom_sub, distance=GRID_SIZE, speed=0.2):
        """
        Move the robot smoothly in a direction (dx, dy) defined by the map axes.
        The robot strafes/moves in the required direction while maintaining its current map-frame yaw.
        """
        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos
        current_yaw = odom_sub.fused_yaw # This is now the yaw in the map frame
        
        # Target map-frame linear velocity components:
        # Assuming map: Rows (dx) are -Y, Columns (dy) are +X
        map_vel_x = speed * dy    
        map_vel_y = speed * (-dx) 

        while rclpy.ok() and math.hypot(odom_sub.x_pos - start_x, odom_sub.y_pos - start_y) < distance:
            rclpy.spin_once(odom_sub)
            
            # Use the FUSED_YAW (which includes the offset) for the transformation
            fused_yaw = odom_sub.fused_yaw 

            # 1. Yaw Correction (to maintain the current map-frame heading)
            err = math.atan2(math.sin(current_yaw - fused_yaw),
                             math.cos(current_yaw - fused_yaw))
            angular_z = 0.5 * err
            
            # 2. Body-Frame Velocity Calculation (Inverse Rotation)
            # Transform map-frame velocity (map_vel_x, map_vel_y) into the body frame
            
            cos_yaw = math.cos(fused_yaw)
            sin_yaw = math.sin(fused_yaw)
            
            # x_body = x_map*cos(yaw) + y_map*sin(yaw)
            linear_x_body = map_vel_x * cos_yaw + map_vel_y * sin_yaw
            
            # y_body = -x_map*sin(yaw) + y_map*cos(yaw)
            linear_y_body = -map_vel_x * sin_yaw + map_vel_y * cos_yaw
            
            self.send_twist(
                linear_x=linear_x_body,
                linear_y=linear_y_body,
                angular_z=angular_z,
                duration=0.05
            )
        self.stop()
    # --- END MODIFIED move_direction ---

    def rotate_to_yaw(self, target_yaw, odom_sub, yaw_tol=0.05, max_speed=0.3):
        # This function is now **unused** but kept for code completeness.
        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            yaw_err = math.atan2(math.sin(target_yaw - odom_sub.fused_yaw),
                                 math.cos(target_yaw - odom_sub.fused_yaw))
            if abs(yaw_err) <= yaw_tol:
                break
            twist = Twist()
            twist.angular.z = max(-max_speed, min(max_speed, yaw_err))
            self.publisher_.publish(twist)
            time.sleep(0.05)
        self.stop()

# ---------------- Parse Map with 'R' and 'G' ----------------
def parse_map(layout):
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

# ---------------- Live Maze Plot ----------------
def plot_maze(maze, start, goal, path=None, robot_pos=None):
    plt.clf()
    plt.imshow(maze, cmap="gray_r")
    if path:
        px, py = zip(*path)
        plt.plot(py, px, "b.-", label="Path")
    if robot_pos:
        x, y = robot_pos
        # Note: Coordinate transformation for plot assumes map Y is -Row, map X is +Col
        gx = int(round(-y / GRID_SIZE))  
        gy = int(round(x / GRID_SIZE))
        gx = np.clip(gx, 0, maze.shape[0]-1)
        gy = np.clip(gy, 0, maze.shape[1]-1)
        plt.plot(gy, gx, "ro", label="Robot")
    plt.plot(start[1], start[0], "go", markersize=10, label="Start (R)")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal (G)")
    plt.legend()
    plt.draw()
    plt.pause(0.001)

# ---------------- Follow Path Smoothly ----------------
def follow_path(node, path, odom_sub, imu_sub, maze, start, goal):
    plt.ion()
    i = 1
    while i < len(path):
        cur = path[i-1]
        nxt = path[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]

        # Combine consecutive moves in same direction
        run_len = 1
        while (i + run_len < len(path) and
               path[i + run_len][0] - path[i + run_len - 1][0] == dx and
               path[i + run_len][1] - path[i + run_len - 1][1] == dy):
            run_len += 1

        total_distance = GRID_SIZE * run_len

        # Update IMU yaw for smoother heading correction
        rclpy.spin_once(imu_sub)
        odom_sub.imu_yaw = imu_sub.yaw

        node.move_direction(dx, dy, odom_sub, distance=total_distance)
        i += run_len

        # Update visualization
        rclpy.spin_once(odom_sub)
        plot_maze(maze, start, goal, path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))

# ---------------- Main ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()
    imu_reader = IMUReader()

    # flush initial readings
    rclpy.spin_once(imu_reader)
    odom_reader.imu_yaw = imu_reader.yaw # Set IMU yaw before first odom update
    rclpy.spin_once(odom_reader)


    # Map layout with R=start, G=goal, 1=wall, 0=free
    maze_layout = [
        ["R", "1", "0", "0", "0", "0"],
        ["0", "1", "0", "1", "1", "0"],
        ["0", "0", "0", "1", "0", "0"],
        ["0", "1", "1", "1", "0", "1"],
        ["0", "0", "1", "0", "0", "0"],
        ["G", "1", "1", "1", "1", "0"]
    ]

    maze, start, goal = parse_map(maze_layout)
    path = bfs_path(maze, start, goal)

    plt.figure()
    plot_maze(maze, start, goal, path, robot_pos=(odom_reader.x_pos, odom_reader.y_pos))

    # --- Initial Orientation Setup (Align current physical heading to map frame) ---
    
    ORIENTATION_TO_YAW = {
        "north": math.pi / 2,
        "south": -math.pi / 2,
        "east": 0.0,
        "west": math.pi
    }

    # Set which map direction the robot is currently facing (e.g., 'south' on the map)
    map_direction_to_align = "south"
    target_map_yaw = ORIENTATION_TO_YAW[map_direction_to_align.lower()]

    # To get the robot's true geographic/IMU yaw for the offset calculation:
    # We must temporarily use the raw, non-offset-adjusted yaw calculation.
    # Note: This is an approximation as it bypasses the alpha blending in odom_callback.
    geographic_yaw = odom_reader.odom_yaw * 0.98 + odom_reader.imu_yaw * 0.02
    
    # Calculate the offset needed to make the current geographic yaw equal the target map yaw.
    # New Fused Yaw = Geographic Yaw + Offset
    # Target Map Yaw = Geographic Yaw + Offset
    # Offset = Target Map Yaw - Geographic Yaw
    odom_reader.map_yaw_offset = target_map_yaw - geographic_yaw
    
    # The 'fused_yaw' in odom_reader will be recalculated on the next update and will 
    # now be correctly registered in the map frame.
    
    print(f"Robot's current physical heading ({geographic_yaw:.2f} rad) is now aliased to map's {map_direction_to_align.upper()} ({target_map_yaw:.2f} rad).")
    print(f"Using map yaw offset: {odom_reader.map_yaw_offset:.2f} rad.")
    
    # No rotation call is needed, as the movement controller uses the newly-offset yaw.

    follow_path(node, path, odom_sub=odom_reader, imu_sub=imu_reader,
                maze=maze, start=start, goal=goal)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    imu_reader.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()