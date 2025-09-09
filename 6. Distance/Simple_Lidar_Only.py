#Here’s a simple LiDAR test script that:
#Subscribes to /scan (the standard LiDAR topic).
#Reads sensor_msgs/LaserScan messages.
#Prints the minimum distance in front of the robot.
#Optionally plots the scan live with matplotlib.

#🔧 How to run
#Make sure your LiDAR driver is running (you should see /scan in ros2 topic list).
#Example:
#ros2 topic list | grep scan

#If you see /scan, you’re good.
#Save the script as lidar_test.py in your ROS2 workspace (src/your_package/scripts/).
#Run it:
#ros2 run your_package lidar_test.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',   # check your topic name with: ros2 topic list
            self.scan_callback,
            10
        )
        self.ranges = []

    def scan_callback(self, msg: LaserScan):
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)

        # Filter invalid values (0 or inf)
        ranges = np.where(np.isinf(ranges), np.nan, ranges)
        ranges = np.where(ranges == 0.0, np.nan, ranges)

        # Get minimum distance in front (angle = 0)
        mid_index = len(msg.ranges) // 2
        front_dist = msg.ranges[mid_index]

        self.get_logger().info(f"Front distance: {front_dist:.2f} m")

        # Save ranges for plotting
        self.ranges = msg.ranges

def main(args=None):
    rclpy.init(args=args)
    node = LidarTester()

    plt.ion()
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    line, = ax.plot([], [], '.')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if len(node.ranges) > 0:
                angles = np.linspace(0, 2*np.pi, len(node.ranges))
                line.set_xdata(angles)
                line.set_ydata(node.ranges)
                ax.set_ylim(0, 3.0)  # adjust max distance
                plt.draw()
                plt.pause(0.01)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
