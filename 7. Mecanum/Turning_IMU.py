#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import time
from transforms3d.euler import quat2euler  # pip3 install transforms3d

def quaternion_to_yaw(q):
    """Convert quaternion to yaw (rotation around Z axis)."""
    # transforms3d expects quaternion in (w, x, y, z) order
    yaw, _, _ = quat2euler([q.w, q.x, q.y, q.z])
    return yaw

class IMUReader(Node):
    """Subscribes to IMU to get yaw orientation"""
    def __init__(self, topic='/imu'):
        super().__init__('imu_reader')
        self.yaw = None
        self.subscription = self.create_subscription(
            Imu,
            topic,
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        self.yaw = quaternion_to_yaw(msg.orientation)

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x=0.0, angular_z=0.0, duration=0.1):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)

    def turn_angle_imu(self, angle_deg, angular_speed=0.5, imu_topic='/imu'):
        """Turn robot by specific angle using IMU yaw."""
        imu_sub = IMUReader(topic=imu_topic)
        rclpy.spin_once(imu_sub)

        while imu_sub.yaw is None:
            self.get_logger().info("Waiting for IMU data...")
            rclpy.spin_once(imu_sub, timeout_sec=0.1)

        start_yaw = imu_sub.yaw
        target_yaw = start_yaw + math.radians(angle_deg)

        print(f"Turning {angle_deg}°, start={math.degrees(start_yaw):.1f}° target={math.degrees(target_yaw):.1f}°")

        while rclpy.ok():
            rclpy.spin_once(imu_sub)
            error = target_yaw - imu_sub.yaw
            # normalize error to [-pi, pi]
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < math.radians(2.0):  # within 2° tolerance
                break

            self.send_twist(angular_z=math.copysign(angular_speed, error))

        self.stop()
        print("Turn complete")

def main():
    rclpy.init()
    node = CmdVelPublisher()

    # Example: Turn 90° left
    node.turn_angle_imu(angle_deg=90, angular_speed=0.5)

    # Example: Turn 90° right
    node.turn_angle_imu(angle_deg=-90, angular_speed=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
