#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState

class PWMServoController(Node):
    def __init__(self):
        super().__init__('pwm_servo_controller')

        # Servo IDs to control
        self.servo_ids = [1, 2, 3]

        # Declare parameters
        self.declare_parameter('initial_position', 1500)
        self.declare_parameter('offset', 0)
        self.declare_parameter('step_size', 200)
        self.declare_parameter('timer_interval', 0.5)

        # Get parameters
        self.initial_position = self.get_parameter('initial_position').get_parameter_value().integer_value
        self.offset = self.get_parameter('offset').get_parameter_value().integer_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().integer_value
        self.timer_interval = self.get_parameter('timer_interval').get_parameter_value().double_value

        # Publisher
        self.publisher = self.create_publisher(SetPWMServoState,
                                               '/ros_robot_controller/pwm_servo/set_state',
                                               10)

        # Initialize servo states
        self.positions = {sid: self.initial_position for sid in self.servo_ids}
        self.directions = {sid: 1 for sid in self.servo_ids}

        # Timer
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

        # Send initial positions
        self.send_servo_positions()

    def send_servo_positions(self):
        # Send one message per servo
        for sid in self.servo_ids:
            msg = SetPWMServoState()
            servo_state = PWMServoState()
            servo_state.id = [sid]
            servo_state.position = [self.positions[sid]]
            servo_state.offset = [self.offset]
            msg.state.append(servo_state)
            self.publisher.publish(msg)
        
        pos_text = ', '.join(f'ID {sid}: {pos}' for sid, pos in self.positions.items())
        self.get_logger().info(f'Sent positions -> {pos_text}')

    def timer_callback(self):
        # Update each servo’s position and direction
        for sid in self.servo_ids:
            self.positions[sid] += self.directions[sid] * self.step_size
            if self.positions[sid] >= 2500 or self.positions[sid] <= 500:
                self.directions[sid] *= -1
                self.positions[sid] = max(500, min(self.positions[sid], 2500))
        self.send_servo_positions()

def main(args=None):
    rclpy.init(args=args)
    node = PWMServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
