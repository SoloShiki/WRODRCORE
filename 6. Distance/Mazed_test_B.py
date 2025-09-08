import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from collections import deque

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.subscription = self.create_subscription(
            Odometry, topic, self.odom_callback, 10
        )

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

# ---------------- CmdVel Publisher ----------------
class CmdVelPublisher(Node):
    def __init__(self, wheelbase=0.16, max_steering_angle=0.4):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.wheelbase = wheelbase
        self.max_steering = max_steering_angle

    def send_twist(self, linear_x, angular_z, duration):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

    def stop(self, duration=0.5):
        self.send_twist(0.0, 0.0, duration)

    # ---------------- Ackermann Arc Turn ----------------
    def ackermann_turn_to_yaw(self, target_yaw, odom, speed=0.1):
        """Turn along an arc until target yaw is reached (Ackermann)"""
        def normalize(a):
            return math.atan2(math.sin(a), math.cos(a))
        
        target_yaw = normalize(target_yaw)
        while rclpy.ok():
            current_yaw = normalize(odom.yaw)
            yaw_error = normalize(target_yaw - current_yaw)
            if abs(yaw_error) < 0.02:  # ~1 degree tolerance
                break
            # Compute steering angle using Ackermann geometry
            steering_angle = max(-self.max_steering,
                                 min(self.max_steering, yaw_error))
            angular_vel = speed * math.tan(steering_angle) / self.wheelbase
            print(f"Turning: target {math.degrees(target_yaw):.1f}°, current {math.degrees(current_yaw):.1f}°, steering {math.degrees(steering_angle):.1f}°")
            self.send_twist(speed, angular_vel, 0.05)
        self.stop()

    # ---------------- Move Straight ----------------
    def move_distance(self, distance, odom, speed=0.2):
        start_x, start_y = odom.x_pos, odom.y_pos
        while rclpy.ok() and math.hypot(odom.x_pos - start_x, odom.y_pos - start_y) < abs(distance):
            direction = 1 if distance > 0 else -1
            self.send_twist(speed*direction, 0.0, 0.05)
            rclpy.spin_once(odom)
        self.stop()

# ---------------- Maze / Grid ----------------
def generate_maze(x_size=6, y_size=6):
    maze = np.zeros((x_size, y_size), dtype=int)
    # Place walls manually or randomly
    maze[0,1] = 1
    maze[1,3] = 1
    maze[1,4] = 1
    maze[3,1] = 1
    maze[3,2] = 1
    maze[5,1] = 1
    maze[5,2] = 1
    start = (0,0)
    goal  = (1,5)
    return maze, start, goal

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

def print_maze(maze, start, goal, path=None):
    for i in range(maze.shape[0]):
        line=""
        for j in range(maze.shape[1]):
            if (i,j)==start:
                line+=" R "
            elif (i,j)==goal:
                line+=" G "
            elif path and (i,j) in path:
                line+=" x "
            elif maze[i,j]==1:
                line+=" 0 "
            else:
                line+=" . "
        print(line)
    print("\n")

# ---------------- Follow Path Ackermann ----------------
def follow_path_ackermann(node, odom, path, cell_size=0.3):
    dir_angles = {(-1,0): math.pi, (1,0):0, (0,-1):-math.pi/2, (0,1):math.pi/2}
    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dx, dy = nxt[0]-cur[0], nxt[1]-cur[1]
        move_dir = (dx, dy)
        target_yaw = dir_angles[move_dir]
        print(f"Moving {cur} -> {nxt}, target yaw: {math.degrees(target_yaw):.1f}°")
        node.ackermann_turn_to_yaw(target_yaw, odom)
        node.move_distance(cell_size, odom)

# ---------------- Main ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom = OdometryReader()
    rclpy.spin_once(odom)

    maze, start, goal = generate_maze()
    print("Original Maze:")
    print_maze(maze,start,goal)

    path = bfs_path(maze,start,goal)
    print("Solution Path:")
    print_maze(maze,start,goal,path)

    follow_path_ackermann(node, odom, path, cell_size=0.3)

    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
