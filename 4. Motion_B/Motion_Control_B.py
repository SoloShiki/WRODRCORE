import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from ros_robot_controller.srv import SetMotor
import time

class RepeatingMotionController(Node):
    def __init__(self):
        super().__init__('repeating_motion_controller')

        # Publishers
        self.servo_pub = self.create_publisher(Int32MultiArray, '/ros_robot_controller/pwm_servo/set_state', 10)

        # Motor service client
        self.motor_client = self.create_client(SetMotor, '/ros_robot_controller/set_motor')
        while not self.motor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /set_motor service...')

        # Robot parameters
        self.wheel_diameter = 0.067  # meters
        self.track_width = 0.133     # meters
        self.linear_speed = 0.2      # m/s
        self.rps = self.linear_speed / (math.pi * self.wheel_diameter)

        # Start motion loop
        self.motion_loop()

    def set_servo_angle(self, angle_deg):
        """Convert angle to PWM and publish to servo."""
        pwm = int(1500 + 2000 * (-angle_deg) / 180)
        msg = Int32MultiArray()
        msg.data = [1, pwm]  # Assuming servo ID 1
        self.servo_pub.publish(msg)

    def set_motor_speeds(self, speeds):
        """Send motor speeds via service."""
        for i, rps in enumerate(speeds):
            request = SetMotor.Request()
            request.id = i + 1
            request.rps = float(rps)
            self.motor_client.call_async(request)

    def stop_motors(self):
        self.set_motor_speeds([0.0, 0.0, 0.0, 0.0])

    def motion_loop(self):
        while rclpy.ok():
            # Phase 1: Straight 2 meters
            self.set_servo_angle(0)  # Straight
            self.set_motor_speeds([0.0, self.rps, 0.0, -self.rps])
            time.sleep(10)

            self.stop_motors()
            time.sleep(3)

            # Phase 2: 2 meters with 10° steering
            self.set_servo_angle(10)
            vr = self.linear_speed + (self.linear_speed * math.tan(math.radians(10)) * self.track_width / 2)
            vl = self.linear_speed - (self.linear_speed * math.tan(math.radians(10)) * self.track_width / 2)
            rps_vr = vr / (math.pi * self.wheel_diameter)
            rps_vl = vl / (math.pi * self.wheel_diameter)
            self.set_motor_speeds([0.0, rps_vl, 0.0, -rps_vr])
            time.sleep(10)

            self.stop_motors()
            time.sleep(3)

def main(args=None):
    rclpy.init(args=args)
    node = RepeatingMotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
