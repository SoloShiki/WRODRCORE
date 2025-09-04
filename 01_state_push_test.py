import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from your_msgs.msg import MotorState, MotorsState  # Replace with actual message imports

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        self.publisher_ = self.create_publisher(MotorsState, '/motor_states', 10)
        self.servo_pub = self.create_publisher(Int32, '/servo_angle', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Chassis parameters
        self.wheelbase = 0.145
        self.track_width = 0.133
        self.wheel_diameter = 0.067

        # Desired motion
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.1  # rad/s

    def speed_convert(self, speed):
        return speed / (math.pi * self.wheel_diameter)

    def control_loop(self):
        servo_angle = 1500
        data = []

        if abs(self.linear_speed) >= 1e-8:
            if abs(self.angular_speed) >= 1e-8:
                theta = math.atan(self.wheelbase * self.angular_speed / self.linear_speed)
                steering_angle = max(min(theta, math.radians(29)), -math.radians(29))
                servo_angle = int(1500 + 2000 * math.degrees(-steering_angle) / 180)

                vr = self.linear_speed + self.angular_speed * self.track_width / 2
                vl = self.linear_speed - self.angular_speed * self.track_width / 2
                v_s = [self.speed_convert(v) for v in [0, vl, 0, -vr]]
            else:
                v_s = [0.0, self.speed_convert(self.linear_speed), 0.0, -self.speed_convert(self.linear_speed)]
        else:
            v_s = [0.0] * 4

        for i, v in enumerate(v_s):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v)
            data.append(msg)

        motor_msg = MotorsState()
        motor_msg.data = data
        self.publisher_.publish(motor_msg)

        servo_msg = Int32()
        servo_msg.data = servo_angle
        self.servo_pub.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = AckermannController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
